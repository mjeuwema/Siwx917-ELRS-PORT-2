#include "SiW917RXEndpoint.h"

#include "Arduino.h"
#include "elrs_config.h"
#include "logging.h"
#include "options.h"

#include <cstdio>
#include <cstring>

#define RX_EP_DIAG 0
#define RX_EP_EVENT_LOG 1

extern "C" void elrs_cpp_request_wifi_mode(void);
extern "C" void elrs_enter_binding_mode(void);
extern "C" void siw917_rx_set_model_match_id(uint8_t modelId);

extern uint8_t ExpressLRS_currTlmDenom;

static char modelString[8] = "Off";
static char tlmRatioString[8] = "1:1";
static char modelIdOptions[200] = {};
static bool modelIdOptionsInitialized = false;

static selectionParameter luaSerialProtocol = {
    {"Protocol", CRSF_TEXT_SELECTION, 0, 0},
    0,
    "CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro;HoTT Telemetry;MAVLink;DisplayPort;GPS",
    STR_EMPTYSPACE,
};

static selectionParameter luaSBUSFailsafeMode = {
    {"SBUS failsafe", CRSF_TEXT_SELECTION, 0, 0},
    0,
    "No Pulses;Last Pos",
    STR_EMPTYSPACE,
};

static int8Parameter luaTargetSysId = {
    {"Target SysID", CRSF_UINT8, 0, 0},
    {{1, 1, 255}},
    STR_EMPTYSPACE,
};

static int8Parameter luaSourceSysId = {
    {"Source SysID", CRSF_UINT8, 0, 0},
    {{255, 1, 255}},
    STR_EMPTYSPACE,
};

static selectionParameter luaForceTlm = {
    {"Force Tlm", CRSF_TEXT_SELECTION, 0, 0},
    0,
    "Off;On",
    STR_EMPTYSPACE,
};

static commandParameter luaWifiMode = {
    {"WiFi Mode", CRSF_COMMAND, 0, 0},
    lcsIdle,
    STR_EMPTYSPACE,
};

static folderParameter luaTeamraceFolder = {
    {"Team Race", CRSF_FOLDER, 0, 0},
    nullptr,
};

static selectionParameter luaTeamraceChannel = {
    {"Channel", CRSF_TEXT_SELECTION, 0, 0},
    0,
    "AUX2;AUX3;AUX4;AUX5;AUX6;AUX7;AUX8;AUX9;AUX10;AUX11;AUX12",
    STR_EMPTYSPACE,
};

static selectionParameter luaTeamracePosition = {
    {"Position", CRSF_TEXT_SELECTION, 0, 0},
    0,
    "Disabled;1/Low;2;3;Mid;4;5;6/High",
    STR_EMPTYSPACE,
};

static selectionParameter luaBindStorage = {
    {"Bind Storage", CRSF_TEXT_SELECTION, 0, 0},
    0,
    "Persistent;Volatile;Returnable;Administered",
    STR_EMPTYSPACE,
};

static commandParameter luaBindMode = {
    {STR_EMPTYSPACE, CRSF_COMMAND, 0, 0},
    lcsIdle,
    STR_EMPTYSPACE,
};

static selectionParameter luaModelId = {
    {"Model Id", CRSF_TEXT_SELECTION, 0, 0},
    0,
    modelIdOptions,
    STR_EMPTYSPACE,
};

static stringParameter luaLiveTlm = {
    {"Live Tlm", CRSF_INFO, 0, 0},
    tlmRatioString,
};

static stringParameter luaELRSversion = {
    {version, CRSF_INFO, 0, 0},
    commit,
};

static uint8_t clampU8(uint8_t value, uint8_t minValue, uint8_t maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

static void initModelOptions() {
  if (modelIdOptionsInitialized) {
    return;
  }

  size_t used = (size_t)snprintf(modelIdOptions, sizeof(modelIdOptions), "Off");
  for (uint8_t i = 0; i <= 63 && used < sizeof(modelIdOptions); ++i) {
    const int written =
        snprintf(modelIdOptions + used, sizeof(modelIdOptions) - used, ";%u", i);
    if (written <= 0) {
      break;
    }
    used += (size_t)written;
  }
  modelIdOptions[sizeof(modelIdOptions) - 1] = '\0';
  modelIdOptionsInitialized = true;
}

static uint8_t modelIdToSelection(uint8_t modelId) {
  return modelId == 0xFF ? 0 : (uint8_t)(clampU8(modelId, 0, 63) + 1);
}

static uint8_t selectionToModelId(uint8_t selection) {
  return selection == 0 ? 0xFF : (uint8_t)(clampU8(selection, 1, 64) - 1);
}

#if RX_EP_EVENT_LOG
static void logParameterWrite(const char *name, int32_t value) {
  DBGLN("[RX_LUA] PARAM_WRITE %s=%ld", name, (long)value);
}

static void logModelIdWrite(uint8_t modelId) {
  if (modelId == 0xFF) {
    DBGLN("[RX_LUA] PARAM_WRITE Model Id=Off");
  } else {
    DBGLN("[RX_LUA] PARAM_WRITE Model Id=%u", modelId);
  }
}
#endif

SiW917RXEndpoint::SiW917RXEndpoint()
    : CRSFEndpoint(CRSF_ADDRESS_CRSF_RECEIVER) {
  initModelOptions();
  registerParameters();
  updateParameters();
}

bool SiW917RXEndpoint::handleRaw(const crsf_header_t *message) {
  if (message->sync_byte == CRSF_ADDRESS_CRSF_RECEIVER &&
      message->frame_size >= 4 && message->type == CRSF_FRAMETYPE_COMMAND) {
    const uint8_t *payload = ((const uint8_t *)message) + sizeof(crsf_header_t);
#if RX_EP_DIAG
    DBGLN("[RX_EP] raw command p=%02X %02X %02X", payload[0], payload[1],
          payload[2]);
#endif
    if (payload[0] == 'b' && payload[1] == 'd') {
#if RX_EP_EVENT_LOG
      DBGLN("[RX_LUA] RAW_BIND_COMMAND");
#endif
      elrs_enter_binding_mode();
      return true;
    }
  }
  return false;
}

void SiW917RXEndpoint::handleMessage(const crsf_header_t *message) {
  const auto *extMessage = (const crsf_ext_header_t *)message;
  const uint8_t *payload =
      reinterpret_cast<const uint8_t *>(message) + sizeof(crsf_ext_header_t);
  const uint8_t payloadLen =
      message->frame_size >= CRSF_FRAME_LENGTH_EXT_TYPE_CRC
          ? (uint8_t)(message->frame_size - CRSF_FRAME_LENGTH_EXT_TYPE_CRC)
          : 0;
#if RX_EP_DIAG
  DBGLN("[RX_EP] msg type=0x%02X len=%u dest=0x%02X orig=0x%02X p0=%u p1=%u",
        message->type, message->frame_size, extMessage->dest_addr,
        extMessage->orig_addr, payloadLen > 0 ? payload[0] : 0,
        payloadLen > 1 ? payload[1] : 0);
#endif

  if (message->type == CRSF_FRAMETYPE_COMMAND && payloadLen >= 2 &&
      payload[0] == CRSF_COMMAND_SUBCMD_RX &&
      payload[1] == CRSF_COMMAND_SUBCMD_RX_BIND) {
#if RX_EP_DIAG
    DBGLN("[RX_EP] command bind");
#endif
#if RX_EP_EVENT_LOG
    DBGLN("[RX_LUA] DIRECT_BIND_COMMAND");
#endif
    elrs_enter_binding_mode();
    return;
  }

  if (message->type == CRSF_FRAMETYPE_DEVICE_PING) {
#if RX_EP_DIAG
    DBGLN("[RX_EP] device ping");
#endif
    updateParameters();
    uint8_t emptyPayload = 0;
    parameterUpdateReq(extMessage->orig_addr, false, extMessage->type, 0,
                       &emptyPayload);
    return;
  }

  if (message->type == CRSF_FRAMETYPE_PARAMETER_READ ||
      message->type == CRSF_FRAMETYPE_PARAMETER_WRITE) {
    if (payloadLen < 1) {
#if RX_EP_DIAG
      DBGLN("[RX_EP] short parameter frame type=0x%02X payloadLen=%u",
            message->type, payloadLen);
#endif
      return;
    }
    updateParameters();
    parameterUpdateReq(extMessage->orig_addr, false, extMessage->type,
                       payload[0], (void *)(payload + 1));
  }
}

void SiW917RXEndpoint::registerParameters() {
  registerParameter(&luaSerialProtocol, [this](propertiesCommon *, int32_t arg) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr) {
      cfg->serial_protocol =
          (uint8_t)clampU8((uint8_t)arg, ELRS_SERIAL_CRSF, ELRS_SERIAL_GPS);
#if RX_EP_EVENT_LOG
      logParameterWrite("Protocol", cfg->serial_protocol);
#endif
      requestConfigSave(true);
    }
  });

  registerParameter(&luaSBUSFailsafeMode, [this](propertiesCommon *, int32_t arg) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr) {
      cfg->failsafe_mode =
          (uint8_t)clampU8((uint8_t)arg, ELRS_FAILSAFE_NO_PULSES,
                           ELRS_FAILSAFE_LAST);
#if RX_EP_EVENT_LOG
      logParameterWrite("SBUS failsafe", cfg->failsafe_mode);
#endif
      requestConfigSave();
    }
  });

  registerParameter(&luaTargetSysId, [this](propertiesCommon *, int32_t arg) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr) {
      cfg->mavlink_target_sys_id = clampU8((uint8_t)arg, 1, 255);
#if RX_EP_EVENT_LOG
      logParameterWrite("Target SysID", cfg->mavlink_target_sys_id);
#endif
      requestConfigSave();
    }
  });

  registerParameter(&luaSourceSysId, [this](propertiesCommon *, int32_t arg) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr) {
      cfg->mavlink_source_sys_id = clampU8((uint8_t)arg, 1, 255);
#if RX_EP_EVENT_LOG
      logParameterWrite("Source SysID", cfg->mavlink_source_sys_id);
#endif
      requestConfigSave();
    }
  });

  registerParameter(&luaForceTlm, [this](propertiesCommon *, int32_t arg) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr) {
      cfg->force_tlm = arg != 0 ? 1 : 0;
#if RX_EP_EVENT_LOG
      logParameterWrite("Force Tlm", cfg->force_tlm);
#endif
      requestConfigSave();
    }
  });

  registerParameter(&luaWifiMode, [this](propertiesCommon *item, int32_t arg) {
    handleWiFiCommand(item, arg);
  });

  registerParameter(&luaTeamraceFolder);
  registerParameter(&luaTeamraceChannel, [this](propertiesCommon *, int32_t arg) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr) {
      cfg->teamrace_channel = clampU8((uint8_t)arg, 0, 10);
#if RX_EP_EVENT_LOG
      logParameterWrite("Team Race Channel", cfg->teamrace_channel);
#endif
      requestConfigSave();
    }
  }, luaTeamraceFolder.common.id);
  registerParameter(&luaTeamracePosition,
                    [this](propertiesCommon *, int32_t arg) {
                      elrs_config_t *cfg = elrs_config_get();
                      if (cfg != nullptr) {
                        cfg->teamrace_position = clampU8((uint8_t)arg, 0, 7);
#if RX_EP_EVENT_LOG
                        logParameterWrite("Team Race Position",
                                          cfg->teamrace_position);
#endif
                        requestConfigSave();
                      }
                    },
                    luaTeamraceFolder.common.id);

  registerParameter(&luaBindStorage, [this](propertiesCommon *, int32_t arg) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr) {
      cfg->bind_storage = clampU8((uint8_t)arg, 0, 3);
#if RX_EP_EVENT_LOG
      logParameterWrite("Bind Storage", cfg->bind_storage);
#endif
      requestConfigSave();
    }
  });

  registerParameter(&luaBindMode, [this](propertiesCommon *item, int32_t arg) {
    handleBindCommand(item, arg);
  });

  registerParameter(&luaModelId, [this](propertiesCommon *, int32_t arg) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr) {
      cfg->model_id = selectionToModelId((uint8_t)arg);
      siw917_rx_set_model_match_id(cfg->model_id);
#if RX_EP_EVENT_LOG
      logModelIdWrite(cfg->model_id);
#endif
      requestConfigSave();
    }
  });

  registerParameter(&luaLiveTlm);
  registerParameter(&luaELRSversion);
}

void SiW917RXEndpoint::updateParameters() {
  elrs_config_t *cfg = elrs_config_get();
  const uint8_t protocol =
      cfg != nullptr && cfg->serial_protocol <= ELRS_SERIAL_GPS
          ? cfg->serial_protocol
          : (uint8_t)ELRS_SERIAL_CRSF;

  setTextSelectionValue(&luaSerialProtocol, protocol);
  setTextSelectionValue(&luaSBUSFailsafeMode,
                        cfg != nullptr ? clampU8(cfg->failsafe_mode, 0, 1) : 0);
  setUint8Value(&luaTargetSysId,
                cfg != nullptr ? clampU8(cfg->mavlink_target_sys_id, 1, 255)
                               : 1);
  setUint8Value(&luaSourceSysId,
                cfg != nullptr ? clampU8(cfg->mavlink_source_sys_id, 1, 255)
                               : 255);
  setTextSelectionValue(&luaForceTlm,
                        cfg != nullptr && cfg->force_tlm != 0 ? 1 : 0);
  setTextSelectionValue(&luaTeamraceChannel,
                        cfg != nullptr ? clampU8(cfg->teamrace_channel, 0, 10)
                                       : 0);
  setTextSelectionValue(&luaTeamracePosition,
                        cfg != nullptr ? clampU8(cfg->teamrace_position, 0, 7)
                                       : 0);
  setTextSelectionValue(&luaBindStorage,
                        cfg != nullptr ? clampU8(cfg->bind_storage, 0, 3) : 0);
  setTextSelectionValue(&luaModelId,
                        cfg != nullptr ? modelIdToSelection(cfg->model_id) : 0);

  if (cfg == nullptr || cfg->model_id == 0xFF) {
    strcpy(modelString, "Off");
  } else {
    snprintf(modelString, sizeof(modelString), "%u", cfg->model_id);
  }

  snprintf(tlmRatioString, sizeof(tlmRatioString), "1:%u", ExpressLRS_currTlmDenom);
  LUA_FIELD_VISIBLE(luaSourceSysId, protocol == ELRS_SERIAL_MAVLINK);
  LUA_FIELD_VISIBLE(luaTargetSysId, protocol == ELRS_SERIAL_MAVLINK);
  luaBindMode.common.name = "Enter Bind Mode";
}

void SiW917RXEndpoint::requestConfigSave(bool applySerialAfterSave) {
  configSavePending = true;
  serialApplyPending = serialApplyPending || applySerialAfterSave;
  configSaveAtMs = millis() + 500U;
#if RX_EP_EVENT_LOG
  DBGLN("[RX_LUA] CONFIG_SAVE_QUEUED serialApply=%u",
        serialApplyPending ? 1 : 0);
#endif
#if RX_EP_DIAG
  DBGLN("[RX_EP] defer config save serial=%u", serialApplyPending ? 1 : 0);
#endif
}

void SiW917RXEndpoint::handleWiFiCommand(propertiesCommon *item, int32_t arg) {
  commandStep_e step = lcsIdle;
  const char *message = STR_EMPTYSPACE;

  if (arg == lcsClick) {
    step = lcsAskConfirm;
    message = "Confirm";
  } else if (arg == lcsConfirmed) {
    step = lcsExecuting;
    message = "Entering...";
    wifiPending = true;
    pendingActionAtMs = millis();
  }

#if RX_EP_EVENT_LOG
  DBGLN("[RX_LUA] WIFI_COMMAND arg=%ld step=%u pending=%u", (long)arg, step,
        wifiPending ? 1 : 0);
#endif
#if RX_EP_DIAG
  DBGLN("[RX_EP] wifi cmd arg=%ld step=%u pending=%u", (long)arg, step,
        wifiPending ? 1 : 0);
#endif
  sendCommandResponse((commandParameter *)item, step, message);
}

void SiW917RXEndpoint::handleBindCommand(propertiesCommon *item, int32_t arg) {
  if (arg == lcsQuery) {
    bindPending = true;
    pendingActionAtMs = millis();
  }

#if RX_EP_EVENT_LOG
  DBGLN("[RX_LUA] BIND_COMMAND arg=%ld pending=%u", (long)arg,
        bindPending ? 1 : 0);
#endif
#if RX_EP_DIAG
  DBGLN("[RX_EP] bind cmd arg=%ld pending=%u", (long)arg,
        bindPending ? 1 : 0);
#endif
  sendCommandResponse((commandParameter *)item,
                      arg < lcsCancel ? lcsExecuting : lcsIdle,
                      arg < lcsCancel ? "Entering..." : STR_EMPTYSPACE);
}

void SiW917RXEndpoint::processPending(bool telemetryBusy) {
  const uint32_t now = millis();

  if (configSavePending &&
      (uint32_t)(now - configSaveAtMs) < 0x80000000UL && !telemetryBusy) {
    configSavePending = false;
    const bool applySerial = serialApplyPending;
    serialApplyPending = false;
    const int saveResult = elrs_config_save();
#if RX_EP_EVENT_LOG
    DBGLN("[RX_LUA] CONFIG_SAVE_DONE result=%d serialApply=%u", saveResult,
          applySerial ? 1 : 0);
#endif
    if (saveResult == 0 && applySerial) {
      serialApplyRequested = true;
    }
  }

  if ((wifiPending || bindPending) &&
      (uint32_t)(now - pendingActionAtMs) >= 200U && !telemetryBusy) {
    if (wifiPending) {
      wifiPending = false;
#if RX_EP_EVENT_LOG
      DBGLN("[RX_LUA] WIFI_COMMAND_EXECUTE");
#endif
#if RX_EP_DIAG
      DBGLN("[RX_EP] enter wifi mode");
#endif
      elrs_cpp_request_wifi_mode();
    }

    if (bindPending) {
      bindPending = false;
#if RX_EP_EVENT_LOG
      DBGLN("[RX_LUA] BIND_COMMAND_EXECUTE requires_tx_bind=1");
#endif
#if RX_EP_DIAG
      DBGLN("[RX_EP] enter bind mode");
#endif
      elrs_enter_binding_mode();
    }
  }
}

bool SiW917RXEndpoint::consumeSerialApplyRequest() {
  const bool requested = serialApplyRequested;
  serialApplyRequested = false;
  return requested;
}
