#include "TXModuleEndpoint.h"

#include "Arduino.h"
#include "CRSFRouter.h"
#include "common.h"
#include "elrs_config.h"
#include "handset.h"
#include "logging.h"
#include "OTA.h"

namespace {
uint32_t g_lastBenchRcLogMs = 0;
uint16_t g_lastBenchRcUs[5] = {};
bool g_lastBenchRcValid = false;
bool g_lastBenchArmState = false;

static bool benchChannelMoved(uint16_t current, uint16_t previous) {
  const uint16_t delta = current > previous ? (current - previous)
                                            : (previous - current);
  return delta >= 8U;
}
} // namespace

extern "C" void elrs_tx_schedule_model_id_sync(uint8_t modelId);
extern "C" int elrs_tx_save_config_with_rf_rearm(void);
#if defined(TARGET_TX) && defined(SIW917_ELRS_TX_PC_BENCH)
extern "C" void siw917_pcbench_capture_crsf_channels(const uint32_t *channels,
                                                      uint8_t channelCount,
                                                      uint8_t armed);
#endif

void TXModuleEndpoint::begin() {
  if (!parametersRegistered) {
    registerParameters();
    parametersRegistered = true;
  }
  updateParameters();
}

bool TXModuleEndpoint::handleRaw(const crsf_header_t *message) {
  if (message->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
    RcPacketToChannelsData(message);
    return true;
  }
  return false;
}

void TXModuleEndpoint::handleMessage(const crsf_header_t *message) {
  const auto *extMessage = reinterpret_cast<const crsf_ext_header_t *>(message);
  const uint8_t *payload =
      reinterpret_cast<const uint8_t *>(message) + sizeof(crsf_ext_header_t);
  const uint8_t payloadLen =
      message->frame_size >= CRSF_FRAME_LENGTH_EXT_TYPE_CRC
          ? (uint8_t)(message->frame_size - CRSF_FRAME_LENGTH_EXT_TYPE_CRC)
          : 0;

  if (message->type == CRSF_FRAMETYPE_COMMAND && payloadLen >= 2 &&
      payload[0] == CRSF_COMMAND_SUBCMD_RX &&
      payload[1] == CRSF_COMMAND_SUBCMD_RX_BIND) {
    DBGLN("CRSF bench CMD: bind requested from handset");
    EnterBindingModeSafely();
    return;
  }

  if (message->type == CRSF_FRAMETYPE_COMMAND && payloadLen >= 3 &&
      payload[0] == CRSF_COMMAND_SUBCMD_RX &&
      payload[1] == CRSF_COMMAND_MODEL_SELECT_ID) {
    DBGLN("CRSF bench CMD: model select id=%u", payload[2]);
    modelId = payload[2] <= MODELMATCH_MASK ? payload[2] : 0xFF;
    if (elrs_config_t *cfg = elrs_config_get()) {
      cfg->model_id =
          modelMatchEnabled && modelId <= MODELMATCH_MASK ? modelId : 0xFF;
      (void)elrs_tx_save_config_with_rf_rearm();
    }
    updateParameters();
    scheduleModelIdSync();
    return;
  }

  if (message->type == CRSF_FRAMETYPE_DEVICE_PING ||
      message->type == CRSF_FRAMETYPE_PARAMETER_READ ||
      message->type == CRSF_FRAMETYPE_PARAMETER_WRITE) {
    const bool isElrsCalling = extMessage->orig_addr == CRSF_ADDRESS_ELRS_LUA;
    const crsf_addr_e requestOrigin =
        isElrsCalling ? CRSF_ADDRESS_RADIO_TRANSMITTER : extMessage->orig_addr;

    if (message->type == CRSF_FRAMETYPE_DEVICE_PING) {
      DBGLN("CRSF bench PARAM: device ping origin=0x%02X routed=0x%02X",
            extMessage->orig_addr, requestOrigin);
    } else if (payloadLen >= 1) {
      DBGLN("CRSF bench PARAM: %s id=0x%02X origin=0x%02X routed=0x%02X",
            message->type == CRSF_FRAMETYPE_PARAMETER_WRITE ? "write" : "read",
            payload[0], extMessage->orig_addr, requestOrigin);
    }

    updateParameters();

    if (message->type == CRSF_FRAMETYPE_PARAMETER_WRITE && payloadLen >= 1) {
      if (payload[0] == 0) {
        sendELRSstatus(requestOrigin);
      } else if (payload[0] == 0x2E) {
        supressCriticalErrors();
      }
    }

    if (message->type == CRSF_FRAMETYPE_DEVICE_PING) {
      uint8_t emptyPayload = 0;
      parameterUpdateReq(requestOrigin, isElrsCalling, message->type, 0,
                         &emptyPayload);
    } else if (payloadLen >= 1) {
      parameterUpdateReq(requestOrigin, isElrsCalling, message->type,
                         payload[0], (void *)(payload + 1));
    }
  }
}

void TXModuleEndpoint::RcPacketToChannelsData(
    const crsf_header_t *message) {
  const auto *payload =
      reinterpret_cast<const uint8_t *>(message) + sizeof(crsf_header_t);
  constexpr unsigned srcBits = 11;
  constexpr unsigned dstBits = 11;
  constexpr unsigned inputChannelMask = (1U << srcBits) - 1U;
  constexpr unsigned precisionShift = dstBits - srcBits;

  uint32_t localChannelData[CRSF_NUM_CHANNELS] = {};

  uint8_t bitsMerged = 0;
  uint32_t readValue = 0;
  unsigned readByteIndex = 0;
  for (uint32_t &channel : localChannelData) {
    while (bitsMerged < srcBits) {
      const uint8_t readByte = payload[readByteIndex++];
      readValue |= ((uint32_t)readByte) << bitsMerged;
      bitsMerged += 8;
    }
    channel = (readValue & inputChannelMask) << precisionShift;
    readValue >>= srcBits;
    bitsMerged -= srcBits;
  }

  handset->PerformChannelOverrides(localChannelData, CRSF_NUM_CHANNELS);

  bool armCmd = false;
  constexpr uint8_t armingModeCh5Mask = 0x02U;
  constexpr uint8_t armedMask = 0x01U;
  if (message->frame_size == CRSF_FRAME_SIZE(sizeof(crsf_channels_t))) {
    armCmd = CRSF_to_BIT(localChannelData[AUX1]);
  } else {
    const uint8_t status = payload[readByteIndex];
    if ((status & armingModeCh5Mask) != 0U) {
      armCmd = CRSF_to_BIT(localChannelData[AUX1]);
    } else {
      armCmd = (status & armedMask) != 0U;
    }
  }

  isArmed = armCmd;
#if defined(TARGET_TX) && defined(SIW917_ELRS_TX_PC_BENCH)
  siw917_pcbench_capture_crsf_channels(localChannelData, CRSF_NUM_CHANNELS,
                                       armCmd ? 1U : 0U);
#endif
  handset->RCDataReceived(localChannelData, CRSF_NUM_CHANNELS);

  const uint16_t chUs[] = {
      CRSF_to_US((uint16_t)localChannelData[0]),
      CRSF_to_US((uint16_t)localChannelData[1]),
      CRSF_to_US((uint16_t)localChannelData[2]),
      CRSF_to_US((uint16_t)localChannelData[3]),
      CRSF_to_US((uint16_t)localChannelData[AUX1]),
  };
  bool shouldLog = !g_lastBenchRcValid || (g_lastBenchArmState != isArmed);
  for (unsigned i = 0; i < 5U && !shouldLog; ++i) {
    shouldLog = benchChannelMoved(chUs[i], g_lastBenchRcUs[i]);
  }

  const uint32_t now = millis();
  if (shouldLog || (uint32_t)(now - g_lastBenchRcLogMs) >= 2000U) {
    DBGLN(
        "CRSF bench RC: ch1=%u ch2=%u ch3=%u ch4=%u aux1=%u armed=%u",
        (unsigned)chUs[0], (unsigned)chUs[1], (unsigned)chUs[2],
        (unsigned)chUs[3], (unsigned)chUs[4], isArmed ? 1U : 0U);
    for (unsigned i = 0; i < 5U; ++i) {
      g_lastBenchRcUs[i] = chUs[i];
    }
    g_lastBenchRcValid = true;
    g_lastBenchArmState = isArmed;
    g_lastBenchRcLogMs = now;
  }
}

uint8_t TXModuleEndpoint::getSyncUID5() const {
  if (!modelMatchEnabled || modelId > MODELMATCH_MASK) {
    return UID[5];
  }

  const uint8_t modelXor = (uint8_t)((~modelId) & MODELMATCH_MASK);
  return (uint8_t)(UID[5] ^ modelXor);
}

uint8_t TXModuleEndpoint::getModelIdForSync() const {
  if (!modelMatchEnabled || modelId > MODELMATCH_MASK) {
    return 0xFF;
  }
  return modelId;
}

void TXModuleEndpoint::scheduleModelIdSync() {
  elrs_tx_schedule_model_id_sync(getModelIdForSync());
}

void TXModuleEndpoint::flagConnectedWarning() {
  setWarningFlag(LUA_FLAG_ERROR_CONNECTED, true);
}
