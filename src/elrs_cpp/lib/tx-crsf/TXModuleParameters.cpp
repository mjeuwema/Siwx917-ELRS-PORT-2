#include "TXModuleEndpoint.h"

#include "Arduino.h"
#include "CRSFHandset.h"
#include "CRSFRouter.h"
#include "common.h"
#include "elrs_config.h"
#include "logging.h"
#include "OTA.h"
#include "options.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

extern "C" const char *elrs_get_rate_name(void);
extern "C" void elrs_cpp_request_wifi_mode(void);
extern "C" bool elrs_tx_request_rx_wifi_mode(void);
extern "C" const char *elrs_tx_rate_name_for_index(uint8_t index);
extern "C" uint8_t elrs_tx_get_rate_index(void);
extern "C" bool elrs_tx_is_rate_change_pending(void);
extern "C" bool elrs_tx_set_rate_index(uint8_t index);
extern "C" uint8_t elrs_tx_get_tlm_ratio_setting(void);
extern "C" bool elrs_tx_set_tlm_ratio_setting(uint8_t tlmRatio);
extern "C" int8_t elrs_tx_get_max_power_dbm(void);
extern "C" bool elrs_tx_set_max_power_dbm(int8_t dbm);
extern "C" int elrs_tx_save_config_with_rf_rearm(void);

static constexpr char luastrOffOn[] = "Off;On";
static constexpr char luaTlmRatioOptions[] =
    "Std;Off;1:128;1:64;1:32;1:16;1:8;1:4;1:2;Race";
static constexpr int8_t TX_POWER_DBM_BY_SELECTION[] = {
    10, 14, 17, 20, 24, 27, 30, 33,
};

static char luaBadGoodString[16] = {};
static char modelMatchUnit[16] = " (ID: Off)";
static char txRateString[24] = "TX Bootstrap";
static char luaPacketRateOptions[192] = {};
static uint8_t luaPacketRateSelectionToIndex[RATE_MAX] = {};
static uint8_t luaPacketRateSelectionCount = 0;

static commandParameter luaBind = {
    {"Bind", CRSF_COMMAND, 0, 0},
    lcsIdle,
    STR_EMPTYSPACE,
};

static selectionParameter luaPacketRate = {
    {"Packet Rate", CRSF_TEXT_SELECTION, 0, 0},
    0,
    luaPacketRateOptions,
    STR_EMPTYSPACE,
};

static selectionParameter luaTlmRatio = {
    {"Telem Ratio", CRSF_TEXT_SELECTION, 0, 0},
    0,
    luaTlmRatioOptions,
    STR_EMPTYSPACE,
};

static selectionParameter luaPower = {
    {"Max Power", CRSF_TEXT_SELECTION, 0, 0},
    3,
    "10;25;50;100;250;500;1000;2000",
    "mW",
};

static folderParameter luaWiFiFolder = {
    {"WiFi Connectivity", CRSF_FOLDER, 0, 0},
    nullptr,
};

static commandParameter luaWebUpdate = {
    {"Enable WiFi", CRSF_COMMAND, 0, 0},
    lcsIdle,
    STR_EMPTYSPACE,
};

static commandParameter luaRxWebUpdate = {
    {"Enable Rx WiFi", CRSF_COMMAND, 0, 0},
    lcsIdle,
    STR_EMPTYSPACE,
};

static selectionParameter luaModelMatch = {
    {"Model Match", CRSF_TEXT_SELECTION, 0, 0},
    0,
    luastrOffOn,
    modelMatchUnit,
};

static stringParameter luaInfo = {
    {"Bad/Good", (crsf_value_type_e)(CRSF_INFO | CRSF_FIELD_ELRS_HIDDEN), 0, 0},
    luaBadGoodString,
};

static stringParameter luaRate = {
    {"Active Rate", CRSF_INFO, 0, 0},
    txRateString,
};

static stringParameter luaELRSversion = {
    {version, CRSF_INFO, 0, 0},
    commit,
};

TXModuleEndpoint crsfTransmitter;

static uint8_t powerDbmToSelection(int8_t dbm) {
  for (uint8_t i = 0;
       i < sizeof(TX_POWER_DBM_BY_SELECTION) / sizeof(TX_POWER_DBM_BY_SELECTION[0]);
       ++i) {
    if (TX_POWER_DBM_BY_SELECTION[i] == dbm) {
      return i;
    }
  }
  return 3;
}

static int8_t powerSelectionToDbm(uint8_t selection) {
  if (selection >=
      sizeof(TX_POWER_DBM_BY_SELECTION) / sizeof(TX_POWER_DBM_BY_SELECTION[0])) {
    return TX_POWER_DBM_BY_SELECTION[3];
  }
  return TX_POWER_DBM_BY_SELECTION[selection];
}

static void rebuildPacketRateOptions() {
  luaPacketRateOptions[0] = '\0';
  luaPacketRateSelectionCount = 0;

  const int minInterval = handset->getMinPacketInterval();
  size_t used = 0;
  for (uint8_t index = 0; index < RATE_MAX; ++index) {
    if (!isSupportedRFRate(index)) {
      continue;
    }

    const expresslrs_mod_settings_s *modParams = get_elrs_airRateConfig(index);
    if (modParams == nullptr) {
      continue;
    }

    const uint32_t handsetInterval =
        (uint32_t)modParams->interval * (uint32_t)modParams->numOfSends;
    if (handsetInterval < (uint32_t)minInterval) {
      continue;
    }

    const char *label = elrs_tx_rate_name_for_index(index);
    const int written = snprintf(luaPacketRateOptions + used,
                                 sizeof(luaPacketRateOptions) - used, "%s%s",
                                 used == 0 ? "" : ";", label);
    if (written <= 0 ||
        (size_t)written >= (sizeof(luaPacketRateOptions) - used)) {
      break;
    }

    luaPacketRateSelectionToIndex[luaPacketRateSelectionCount++] = index;
    used += (size_t)written;
  }

  if (luaPacketRateSelectionCount == 0) {
    snprintf(luaPacketRateOptions, sizeof(luaPacketRateOptions), "ELRS");
    luaPacketRateSelectionToIndex[0] = elrs_tx_get_rate_index();
    luaPacketRateSelectionCount = 1;
  }
}

static uint8_t packetRateSelectionForIndex(uint8_t rateIndex) {
  for (uint8_t i = 0; i < luaPacketRateSelectionCount; ++i) {
    if (luaPacketRateSelectionToIndex[i] == rateIndex) {
      return i;
    }
  }
  return 0;
}

void TXModuleEndpoint::supressCriticalErrors() { luaWarningFlags &= 0x1F; }

void TXModuleEndpoint::devicePingCalled() {
  supressCriticalErrors();
  snprintf(luaBadGoodString, sizeof(luaBadGoodString), "%lu/%lu",
           (unsigned long)CRSFHandset::BadPktsCountResult,
           (unsigned long)CRSFHandset::GoodPktsCountResult);
  luaBadGoodString[sizeof(luaBadGoodString) - 1] = '\0';
}

void TXModuleEndpoint::setWarningFlag(const warningFlags flag,
                                      const bool value) {
  if (value) {
    luaWarningFlags |= (uint8_t)(1U << (uint8_t)flag);
  } else {
    luaWarningFlags &= (uint8_t)~(1U << (uint8_t)flag);
  }
}

void TXModuleEndpoint::sendELRSstatus(const crsf_addr_e origin) {
  static constexpr const char *messages[] = {
      "",
      "Switching rate",
      "Model Mismatch",
      "[ ! Armed ! ]",
      "",
      "Not while connected",
      "Baud rate too low",
      "",
  };

  setWarningFlag(LUA_FLAG_CONNECTED, connectionState == connected);
  setWarningFlag(LUA_FLAG_STATUS1, elrs_tx_is_rate_change_pending());
  setWarningFlag(LUA_FLAG_MODEL_MATCH,
                 connectionState == connected && !connectionHasModelMatch);
  setWarningFlag(LUA_FLAG_ISARMED, isArmed);

  const char *warningInfo = "";
  for (int i = 7; i >= 0; --i) {
    if ((luaWarningFlags & (uint8_t)(1U << i)) != 0U) {
      warningInfo = messages[i];
      break;
    }
  }

  const uint8_t payloadSize =
      (uint8_t)(sizeof(elrsStatusParameter) + strlen(warningInfo));
  uint8_t buffer[sizeof(crsf_ext_header_t) + sizeof(elrsStatusParameter) + 32] =
      {};
  auto *params =
      reinterpret_cast<elrsStatusParameter *>(&buffer[sizeof(crsf_ext_header_t)]);

  params->pktsBad =
      (uint8_t)std::min<uint32_t>(CRSFHandset::BadPktsCountResult, 0xFFU);
  params->pktsGood = htobe16((uint16_t)std::min<uint32_t>(
      CRSFHandset::GoodPktsCountResult, 0xFFFFU));
  params->flags = luaWarningFlags;
  strcpy(params->msg, warningInfo);

  crsfRouter.SetExtendedHeaderAndCrc(
      reinterpret_cast<crsf_ext_header_t *>(buffer), CRSF_FRAMETYPE_ELRS_STATUS,
      CRSF_EXT_FRAME_SIZE(payloadSize), origin, CRSF_ADDRESS_CRSF_TRANSMITTER);
  crsfRouter.deliverMessageTo(origin, reinterpret_cast<crsf_header_t *>(buffer));
}

void TXModuleEndpoint::updateModelID() {
  if (modelId > MODELMATCH_MASK) {
    snprintf(modelMatchUnit, sizeof(modelMatchUnit), " (ID: Off)");
  } else {
    snprintf(modelMatchUnit, sizeof(modelMatchUnit), " (ID: %u)", modelId);
  }
  modelMatchUnit[sizeof(modelMatchUnit) - 1] = '\0';
}

void TXModuleEndpoint::handleSimpleSendCmd(propertiesCommon *item, int32_t arg) {
  auto *cmd = reinterpret_cast<commandParameter *>(item);
  if (cmd == &luaRxWebUpdate) {
    if (arg < lcsCancel) {
      bindCommandStartedAtMs = millis();
      if (connectionState != connected || !elrs_tx_request_rx_wifi_mode()) {
        DBGLN("CRSF bench LUA: RX WiFi request rejected (connected=%u)",
              connectionState == connected ? 1U : 0U);
        sendCommandResponse(cmd, lcsIdle, "Not connected");
        return;
      }
      DBGLN("CRSF bench LUA: RX WiFi request queued");
      sendCommandResponse(cmd, lcsExecuting, "Sending...");
      return;
    }

    if (arg == lcsQuery &&
        (uint32_t)(millis() - bindCommandStartedAtMs) <= 2000U) {
      sendCommandResponse(cmd, lcsExecuting, "Sending...");
      return;
    }

    sendCommandResponse(cmd, lcsIdle, STR_EMPTYSPACE);
    return;
  }

  if (cmd != &luaBind) {
    sendCommandResponse(cmd, lcsIdle, STR_EMPTYSPACE);
    return;
  }

  if (arg < lcsCancel) {
    bindCommandStartedAtMs = millis();
    DBGLN("CRSF bench LUA: bind command requested");
    EnterBindingModeSafely();
    sendCommandResponse(cmd, lcsExecuting, "Binding...");
    return;
  }

  if (arg == lcsQuery &&
      (uint32_t)(millis() - bindCommandStartedAtMs) <= 2000U) {
    sendCommandResponse(cmd, lcsExecuting, "Binding...");
    return;
  }

  sendCommandResponse(cmd, lcsIdle, STR_EMPTYSPACE);
}

void TXModuleEndpoint::updateFolderNamesAndVisibility() { updateModelID(); }

void TXModuleEndpoint::registerParameters() {
  setStringValue(&luaInfo, luaBadGoodString);
  setStringValue(&luaRate, txRateString);
  rebuildPacketRateOptions();

  registerParameter(&luaBind, [this](propertiesCommon *item, int32_t arg) {
    handleSimpleSendCmd(item, arg);
  });
  registerParameter(&luaPacketRate, [this](propertiesCommon *, int32_t arg) {
    const uint8_t selection = (uint8_t)arg;
    if (selection >= luaPacketRateSelectionCount) {
      updateParameters();
      return;
    }

    const uint8_t rateIndex = luaPacketRateSelectionToIndex[selection];
    if (!elrs_tx_set_rate_index(rateIndex)) {
      DBGLN("CRSF bench LUA: packet rate request rejected -> %s",
            elrs_tx_rate_name_for_index(rateIndex));
      flagConnectedWarning();
      updateParameters();
      return;
    }

    DBGLN("CRSF bench LUA: packet rate request accepted -> %s",
          elrs_tx_rate_name_for_index(rateIndex));

    if (elrs_config_t *cfg = elrs_config_get()) {
      cfg->rate_index = rateIndex;
      (void)elrs_tx_save_config_with_rf_rearm();
    }
    updateParameters();
  });
  registerParameter(&luaTlmRatio, [this](propertiesCommon *, int32_t arg) {
    const uint8_t tlmRatio = (uint8_t)arg;
    if (!elrs_tx_set_tlm_ratio_setting(tlmRatio)) {
      DBGLN("CRSF bench LUA: telemetry ratio request rejected -> %u",
            (unsigned)tlmRatio);
      updateParameters();
      return;
    }

    DBGLN("CRSF bench LUA: telemetry ratio request accepted -> %u",
          (unsigned)tlmRatio);

    if (elrs_config_t *cfg = elrs_config_get()) {
      cfg->tlm_interval = tlmRatio;
      (void)elrs_tx_save_config_with_rf_rearm();
    }
    updateParameters();
  });
  registerParameter(&luaPower, [this](propertiesCommon *, int32_t arg) {
    const int8_t dbm = powerSelectionToDbm((uint8_t)arg);
    if (!elrs_tx_set_max_power_dbm(dbm)) {
      DBGLN("CRSF bench LUA: max power request rejected -> %d dBm", dbm);
      updateParameters();
      return;
    }

    DBGLN("CRSF bench LUA: max power request accepted -> %d dBm", dbm);

    if (elrs_config_t *cfg = elrs_config_get()) {
      cfg->tx_power = dbm;
      (void)elrs_tx_save_config_with_rf_rearm();
    }
    updateParameters();
  });
  registerParameter(&luaWiFiFolder);
  registerParameter(&luaWebUpdate, [this](propertiesCommon *item, int32_t arg) {
    auto *cmd = reinterpret_cast<commandParameter *>(item);
    if (arg == lcsClick) {
      sendCommandResponse(cmd, lcsAskConfirm, "Confirm");
      return;
    }
    if (arg == lcsConfirmed) {
      DBGLN("CRSF bench LUA: local WiFi request accepted");
      elrs_cpp_request_wifi_mode();
      sendCommandResponse(cmd, lcsExecuting, "Entering...");
      return;
    }
    sendCommandResponse(cmd, lcsIdle, STR_EMPTYSPACE);
  }, luaWiFiFolder.common.id);
  registerParameter(&luaRxWebUpdate,
                    [this](propertiesCommon *item, int32_t arg) {
                      handleSimpleSendCmd(item, arg);
                    },
                    luaWiFiFolder.common.id);
  registerParameter(&luaModelMatch,
                    [this](propertiesCommon *, int32_t arg) {
                      modelMatchEnabled = arg != 0;
                      DBGLN("CRSF bench LUA: model match %s",
                            modelMatchEnabled ? "enabled" : "disabled");
                      if (elrs_config_t *cfg = elrs_config_get()) {
                        cfg->model_id =
                            modelMatchEnabled && modelId <= MODELMATCH_MASK
                                ? modelId
                                : 0xFF;
                        (void)elrs_tx_save_config_with_rf_rearm();
                      }
                      updateParameters();
                      scheduleModelIdSync();
                    });
  registerParameter(&luaInfo);
  registerParameter(&luaRate);
  registerParameter(&luaELRSversion);
}

void TXModuleEndpoint::updateParameters() {
  devicePingCalled();
  updateFolderNamesAndVisibility();
  rebuildPacketRateOptions();

  setTextSelectionValue(&luaPacketRate,
                        packetRateSelectionForIndex(elrs_tx_get_rate_index()));
  setTextSelectionValue(&luaTlmRatio, elrs_tx_get_tlm_ratio_setting());
  setTextSelectionValue(&luaPower,
                        powerDbmToSelection(elrs_tx_get_max_power_dbm()));
  setTextSelectionValue(&luaModelMatch, modelMatchEnabled ? 1U : 0U);
  const char *activeRateName = elrs_get_rate_name();
  const char *requestedRateName =
      elrs_tx_rate_name_for_index(elrs_tx_get_rate_index());
  if (elrs_tx_is_rate_change_pending()) {
    if (strcmp(activeRateName, requestedRateName) != 0) {
      snprintf(txRateString, sizeof(txRateString), "%s>%s", activeRateName,
               requestedRateName);
    } else {
      snprintf(txRateString, sizeof(txRateString), "%s (sync)", activeRateName);
    }
  } else {
    snprintf(txRateString, sizeof(txRateString), "%s", activeRateName);
  }
  txRateString[sizeof(txRateString) - 1] = '\0';
  setStringValue(&luaInfo, luaBadGoodString);
  setStringValue(&luaRate, txRateString);
}
