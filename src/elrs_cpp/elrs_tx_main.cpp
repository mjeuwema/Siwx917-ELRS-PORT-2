/**
 * @file elrs_tx_main.cpp
 * @brief ExpressLRS TX entrypoint port for SiW917
 *
 * This file keeps the upstream TX state machine shape while adapting the
 * device, storage, WiFi, button, LED, and LR1121 HAL hooks to the SiW917 SDK.
 */

#include "elrs_tx_main.h"

#include "common.h"
#include "elrs_main.h"
#include "handset.h"
#include "hwTimer.h"
#include "logging.h"
#include "OTA.h"
#include "FHSS.h"
#include "msptypes.h"
#include "options.h"
#include "crsf_protocol.h"
#include "CRSFHandset.h"
#include "CRSFRouter.h"
#include "stubborn_receiver.h"
#include "stubborn_sender.h"
#include "telemetry_protocol.h"
#include "TXOTAConnector.h"
#include "TXModuleEndpoint.h"

#include <new>
#include <string.h>

extern "C" {
#include "bind_button.h"
#include "elrs_config.h"
#include "status_led.h"
void elrs_cpp_request_wifi_mode(void);
void elrs_enter_binding_mode(void);
void elrs_exit_binding_mode(void);
}

extern CRSFRouter crsfRouter;
extern Handset *handset;
extern StubbornSender DataUlSender;
extern StubbornReceiver DataDlReceiver;
extern connectionState_e connectionState;

namespace {
CRSFHandset g_crsfHandset;
TXOTAConnector g_txOtaConnector;
bool g_txInitialized = false;
bool g_txRadioReady = false;
bool g_txRunning = false;
bool g_txWifiMode = false;
bool g_txBindingMode = false;
bool g_txBusyTransmitting = false;
elrs_link_stats_t g_txLinkStats = {};
const char *g_txRateName = "TX Bootstrap";
uint32_t g_lastTlmPacketRecvMs = 0;
uint32_t g_syncPacketLastSentMs = 0;
uint32_t g_lastHandsetLinkStatsMs = 0;
TxTlmRcvPhase_e g_telemetryRcvPhase = ttrpTransmitting;
bool g_nextPacketIsDataUl = false;
uint8_t g_syncSlot = 0;
uint8_t g_bindingPayload[5] = {};
uint8_t g_bindingSendCount = 0;
uint8_t g_dataDlBuffer[CRSF_MAX_PACKET_LEN + 1] = {};
bool g_modelIdSyncPending = false;
uint8_t g_pendingModelId = 0xFF;
uint8_t g_tlmRatioSetting = (uint8_t)TLM_RATIO_STD;
int8_t g_powerDbm = ELRS_TX_POWER_DEFAULT_DBM;
uint8_t g_requestedRateIndex = 0;
bool g_rateChangePending = false;
uint8_t g_syncSpamCounter = 0;
uint8_t g_syncSpamCounterAfterRateChange = 0;
uint32_t g_rateChangeStartedMs = 0;
uint32_t g_lastBenchStatusLogMs = 0;
bool g_lastBenchStatusValid = false;
elrs_connection_state_t g_lastBenchStatusState = ELRS_DISCONNECTED;
uint8_t g_lastBenchStatusLq = 0;
uint8_t g_lastBenchStatusRateIndex = RATE_MAX;
bool g_lastBenchStatusBinding = false;
bool g_lastBenchStatusWifi = false;

#if defined(SIW917_ELRS_TX_PC_BENCH)
constexpr bool TX_PC_BENCH_MODE = true;
bool g_benchFixedRateLogged = false;
#else
constexpr bool TX_PC_BENCH_MODE = false;
#endif
constexpr uint8_t BINDING_SPAM_AMOUNT = 25;
constexpr uint8_t SYNC_SPAM_AMOUNT = 3;
constexpr uint8_t SYNC_SPAM_AMOUNT_AFTER_RATE_CHANGE = 10;
constexpr uint32_t TX_BENCH_STATUS_INTERVAL_MS = 1000U;

void queueBindPayload() {
  g_bindingPayload[0] = MSP_ELRS_BIND;
  memcpy(&g_bindingPayload[1], &UID[2], 4);
  g_bindingSendCount = 0;
  DataUlSender.ResetState();
  DataUlSender.SetDataToTransmit(g_bindingPayload, sizeof(g_bindingPayload));
  g_nextPacketIsDataUl = true;
}

bool queueControlPayload(const uint8_t *payload, uint8_t length) {
  if (!g_txInitialized || !g_txRadioReady || !g_txRunning ||
      connectionState != connected || g_txBindingMode || InBindingMode) {
    return false;
  }

  if (!g_txOtaConnector.queuePayload(payload, length)) {
    return false;
  }

  g_nextPacketIsDataUl = true;
  return true;
}

void tryQueuePendingModelIdSync() {
  if (!g_modelIdSyncPending) {
    return;
  }

  const uint8_t payload[] = {
      MSP_ELRS_RXTX_CONFIG,
      (uint8_t)MSP_ELRS_RXTX_CONFIG_SUBCMD::MODEL_ID,
      g_pendingModelId,
  };
  if (queueControlPayload(payload, sizeof(payload))) {
    g_modelIdSyncPending = false;
  }
}

uint32_t uidMacSeedGetTx() {
  return ((uint32_t)UID[2] << 24) | ((uint32_t)UID[3] << 16) |
         ((uint32_t)UID[4] << 8) | (UID[5] ^ OTA_VERSION_ID);
}

bool use2G4DomainTx() { return firmwareOptions.domain >= 8; }

const char *rateNameForEnum(expresslrs_RFrates_e rate) {
  switch (rate) {
  case RATE_FSK_900_1000HZ_8CH:
    return "900 FSK 1000Hz";
  case RATE_LORA_900_25HZ:
    return "900 25Hz";
  case RATE_LORA_900_50HZ:
    return "900 50Hz";
  case RATE_LORA_900_100HZ:
    return "900 100Hz";
  case RATE_LORA_900_100HZ_8CH:
    return "900 100Hz 8CH";
  case RATE_LORA_900_150HZ:
    return "900 150Hz";
  case RATE_LORA_900_200HZ:
    return "900 200Hz";
  case RATE_LORA_900_200HZ_8CH:
    return "900 200Hz 8CH";
  case RATE_LORA_900_250HZ:
    return "900 250Hz";
  case RATE_LORA_900_333HZ_8CH:
    return "900 333Hz 8CH";
  case RATE_LORA_900_500HZ:
    return "900 500Hz";
  case RATE_LORA_900_50HZ_DVDA:
    return "900 50Hz DVDA";
  case RATE_FSK_2G4_1000HZ:
    return "2G4 FSK 1000Hz";
  case RATE_FSK_2G4_500HZ_DVDA:
    return "2G4 FSK 500Hz DVDA";
  case RATE_FSK_2G4_250HZ_DVDA:
    return "2G4 FSK 250Hz DVDA";
  case RATE_LORA_2G4_25HZ:
    return "2G4 25Hz";
  case RATE_LORA_2G4_50HZ:
    return "2G4 50Hz";
  case RATE_LORA_2G4_100HZ:
    return "2G4 100Hz";
  case RATE_LORA_2G4_100HZ_8CH:
    return "2G4 100Hz 8CH";
  case RATE_LORA_2G4_150HZ:
    return "2G4 150Hz";
  case RATE_LORA_2G4_200HZ:
    return "2G4 200Hz";
  case RATE_LORA_2G4_200HZ_8CH:
    return "2G4 200Hz 8CH";
  case RATE_LORA_2G4_250HZ:
    return "2G4 250Hz";
  case RATE_LORA_2G4_333HZ_8CH:
    return "2G4 333Hz 8CH";
  case RATE_LORA_2G4_500HZ:
    return "2G4 500Hz";
  case RATE_LORA_DUAL_150HZ:
    return "Dual 150Hz";
  case RATE_LORA_DUAL_100HZ_8CH:
    return "Dual 100Hz 8CH";
  default:
    return "ELRS";
  }
}

uint8_t powerDbmToCrsfLevel(int8_t dbm) {
  switch (dbm) {
  case 10:
    return 1;
  case 14:
    return 2;
  case 17:
    return 3;
  case 20:
    return 4;
  case 24:
    return 5;
  case 27:
    return 6;
  case 30:
    return 7;
  case 33:
    return 8;
  default:
    return 4;
  }
}

int8_t normalizePowerDbm(int8_t dbm) {
  switch (dbm) {
  case 10:
  case 14:
  case 17:
  case 20:
  case 24:
  case 27:
  case 30:
  case 33:
    return dbm;
  default:
    return ELRS_TX_POWER_DEFAULT_DBM;
  }
}

expresslrs_tlm_ratio_e effectiveTlmRatioSetting() {
  if (ExpressLRS_currAirRate_Modparams == nullptr) {
    return TLM_RATIO_NO_TLM;
  }

  const auto configured =
      (expresslrs_tlm_ratio_e)g_tlmRatioSetting;
  if (configured == TLM_RATIO_STD) {
    return ExpressLRS_currAirRate_Modparams->TLMinterval;
  }
  if (configured == TLM_RATIO_DISARMED) {
    return isArmed ? TLM_RATIO_NO_TLM
                   : ExpressLRS_currAirRate_Modparams->TLMinterval;
  }
  return configured;
}

void refreshTlmSettings() {
  if (ExpressLRS_currAirRate_Modparams == nullptr) {
    ExpressLRS_currTlmDenom = 1;
    return;
  }

  const expresslrs_tlm_ratio_e effective = effectiveTlmRatioSetting();
  ExpressLRS_currTlmDenom = TLMratioEnumToValue(effective);
  uint16_t hz = 1000000U / ExpressLRS_currAirRate_Modparams->interval;
  if (hz == 0U) {
    hz = 1U;
  }
  const uint8_t burst =
      TLMBurstMaxForRateRatio(hz, ExpressLRS_currTlmDenom);
  DataUlSender.UpdateTelemetryRate(hz, ExpressLRS_currTlmDenom, burst);
}

void schedulePowerUpdate(int8_t dbm) {
  g_powerDbm = normalizePowerDbm(dbm);
  linkStats.uplink_TX_Power = powerDbmToCrsfLevel(g_powerDbm);
  Radio.SetOutputPower(g_powerDbm, true);
  Radio.SetOutputPower(g_powerDbm, false);
}

void updateTxStatusLedMode() {
  status_led_mode_t desired = LED_MODE_DISCONNECTED;

  if (g_txWifiMode) {
    desired = LED_MODE_WIFI;
  } else if (g_txBindingMode || InBindingMode) {
    desired = LED_MODE_BINDING;
  } else {
    switch (connectionState) {
    case connected:
      desired = LED_MODE_CONNECTED;
      break;
    case awaitingModelId:
      desired = LED_MODE_TENTATIVE;
      break;
    case radioFailed:
    case hardwareUndefined:
      desired = LED_MODE_ERROR;
      break;
    case disconnected:
    default:
      desired = LED_MODE_DISCONNECTED;
      break;
    }
  }

  if (status_led_get_mode() != desired) {
    status_led_set_mode(desired);
  }
}

void onTxBindButtonEvent(bool longPress) {
  if (longPress) {
    DBGLN("TX bind button long press - toggling binding mode");
    if (!g_txBindingMode && !InBindingMode) {
      elrs_enter_binding_mode();
    } else {
      elrs_exit_binding_mode();
    }
    return;
  }

  DBGLN("TX bind button short press - requesting WiFi mode");
  if (!g_txWifiMode) {
    elrs_cpp_request_wifi_mode();
  }
}

uint8_t currentRateIndex() {
  return ExpressLRS_currAirRate_Modparams != nullptr
             ? ExpressLRS_currAirRate_Modparams->index
             : RATE_MAX;
}

elrs_connection_state_t txBenchConnectionState() {
  if (g_txBindingMode || InBindingMode) {
    return ELRS_BINDING;
  }

  switch (connectionState) {
  case connected:
    return ELRS_CONNECTED;
  case awaitingModelId:
    return ELRS_TENTATIVE;
  case radioFailed:
  case hardwareUndefined:
    return ELRS_RADIO_FAILED;
  case disconnected:
  default:
    return ELRS_DISCONNECTED;
  }
}

const char *txBenchConnectionStateName(elrs_connection_state_t state) {
  switch (state) {
  case ELRS_CONNECTED:
    return "connected";
  case ELRS_TENTATIVE:
    return "tentative";
  case ELRS_BINDING:
    return "binding";
  case ELRS_RADIO_FAILED:
    return "radio_failed";
  case ELRS_DISCONNECTED:
  default:
    return "disconnected";
  }
}

void logTxBenchStatus(bool force = false) {
  const uint32_t now = millis();
  const elrs_connection_state_t state = txBenchConnectionState();
  const uint8_t rateIndex = currentRateIndex();
  const bool binding = g_txBindingMode || InBindingMode;
  const uint32_t telemetryAgeMs =
      g_lastTlmPacketRecvMs == 0U ? 0xFFFFFFFFUL
                                  : (uint32_t)(now - g_lastTlmPacketRecvMs);

  const bool changed =
      !g_lastBenchStatusValid || state != g_lastBenchStatusState ||
      g_txLinkStats.lq != g_lastBenchStatusLq ||
      rateIndex != g_lastBenchStatusRateIndex ||
      binding != g_lastBenchStatusBinding ||
      g_txWifiMode != g_lastBenchStatusWifi;

  if (!force && !changed &&
      (uint32_t)(now - g_lastBenchStatusLogMs) < TX_BENCH_STATUS_INTERVAL_MS) {
    return;
  }

  DBGLN("[TXBENCH] state=%s rate=\"%s\" rate_index=%u lq=%u rssi1=%d "
        "rssi2=%d snr=%d rf_mode=%u power_dbm=%d tlm=%u bind=%u wifi=%u "
        "running=%u radio=%u tx_busy=%u tlm_phase=%u pc_bench=%u "
        "internal_rc=%u tlm_age_ms=%lu",
        txBenchConnectionStateName(state), g_txRateName, (unsigned)rateIndex,
        (unsigned)g_txLinkStats.lq, (int)g_txLinkStats.rssi_1,
        (int)g_txLinkStats.rssi_2, (int)g_txLinkStats.snr,
        (unsigned)g_txLinkStats.rf_mode, (int)g_powerDbm,
        (unsigned)g_tlmRatioSetting, binding ? 1U : 0U,
        g_txWifiMode ? 1U : 0U, g_txRunning ? 1U : 0U,
        g_txRadioReady ? 1U : 0U, g_txBusyTransmitting ? 1U : 0U,
        (unsigned)g_telemetryRcvPhase, TX_PC_BENCH_MODE ? 1U : 0U,
        TX_PC_BENCH_MODE ? 1U : 0U, (unsigned long)telemetryAgeMs);

  g_lastBenchStatusLogMs = now;
  g_lastBenchStatusValid = true;
  g_lastBenchStatusState = state;
  g_lastBenchStatusLq = g_txLinkStats.lq;
  g_lastBenchStatusRateIndex = rateIndex;
  g_lastBenchStatusBinding = binding;
  g_lastBenchStatusWifi = g_txWifiMode;
}

bool hasSupportedRateIndex(uint8_t index) {
  return index < RATE_MAX && isSupportedRFRate(index) &&
         get_elrs_airRateConfig(index) != nullptr;
}

void armSyncSpam(uint8_t preRateChange, uint8_t postRateChange) {
  g_syncSpamCounter = preRateChange;
  g_syncSpamCounterAfterRateChange = postRateChange;
  g_syncPacketLastSentMs = 0;
}

bool syncSpamActive() {
  return g_syncSpamCounter != 0U || g_syncSpamCounterAfterRateChange != 0U;
}

bool rateChangeTransitionActive() {
  return g_rateChangePending || syncSpamActive() ||
         connectionState == awaitingModelId;
}

uint8_t advertisedSyncRateIndex() {
  if (syncSpamActive() && hasSupportedRateIndex(g_requestedRateIndex)) {
    return g_requestedRateIndex;
  }
  return currentRateIndex();
}

void consumeSyncSpamCounter() {
  if (g_syncSpamCounter != 0U) {
    --g_syncSpamCounter;
  } else if (g_syncSpamCounterAfterRateChange != 0U) {
    --g_syncSpamCounterAfterRateChange;
  }
}

void initDefaultChannels() {
  for (unsigned i = 0; i < CRSF_NUM_CHANNELS; ++i) {
    ChannelData[i] = CRSF_CHANNEL_VALUE_MID;
  }
  ChannelData[2] = CRSF_CHANNEL_VALUE_MIN;
  ChannelData[AUX1] = CRSF_CHANNEL_VALUE_MIN;
}

uint8_t chooseBindingRateIndex() {
  const expresslrs_RFrates_e bindingRate =
      use2G4DomainTx() ? RATE_DUALBAND_BINDING : RATE_BINDING;
  const uint8_t index = enumRatetoIndex(bindingRate);
  return (index < RATE_MAX) ? index : 0;
}

uint8_t chooseStartupRateIndex() {
  const elrs_config_t *cfg = elrs_config_get();
  if (cfg != nullptr && cfg->rate_index < RATE_MAX &&
      isSupportedRFRate(cfg->rate_index)) {
    return cfg->rate_index;
  }

  const expresslrs_RFrates_e preferredRate =
      use2G4DomainTx() ? RATE_LORA_2G4_250HZ : RATE_LORA_900_250HZ;
  uint8_t index = enumRatetoIndex(preferredRate);
  if (index < RATE_MAX && isSupportedRFRate(index)) {
    return index;
  }

  const expresslrs_RFrates_e fallbackRate =
      use2G4DomainTx() ? RATE_DUALBAND_BINDING : RATE_BINDING;
  index = enumRatetoIndex(fallbackRate);
  return (index < RATE_MAX) ? index : 0;
}

bool configureStartupRate(uint8_t index, bool bindMode) {
  expresslrs_mod_settings_s *const modParams = get_elrs_airRateConfig(index);
  expresslrs_rf_pref_params_s *const rfPerf = get_elrs_RFperfParams(index);
  if (modParams == nullptr || rfPerf == nullptr) {
    DBGLN("ELRS TX invalid rate index %u", index);
    return false;
  }

  const bool invertIq = bindMode || (UID[5] & 0x01);
  hwTimer::updateInterval(modParams->interval);

  FHSSusePrimaryFreqBand =
      !(modParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) &&
      !(modParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
  FHSSuseDualBand = false;

  Radio.Config(modParams->bw, modParams->sf, modParams->cr, FHSSgetInitialFreq(),
               modParams->PreambleLen, invertIq, modParams->PayloadLength,
               modParams->radio_type == RADIO_TYPE_LR1121_GFSK_900 ||
                   modParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4,
               (uint8_t)UID[5], (uint8_t)UID[4]);
  Radio.FuzzySNRThreshold =
      (rfPerf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE)
          ? 0
          : (rfPerf->DynpowerSnrThreshUp - rfPerf->DynpowerSnrThreshDn);

  OtaUpdateSerializers(smWideOr8ch, modParams->PayloadLength);
  DataUlSender.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
  DataDlReceiver.setMaxPackageIndex(OtaIsFullRes ? ELRS8_DATA_DL_MAX_PACKAGES
                                                 : ELRS4_DATA_DL_MAX_PACKAGES);
  ExpressLRS_currAirRate_Modparams = modParams;
  ExpressLRS_currAirRate_RFperfParams = rfPerf;
  refreshTlmSettings();
  g_txLinkStats.rf_mode = modParams->enum_rate;
  g_txRateName = rateNameForEnum(modParams->enum_rate);
  handset->setPacketInterval(modParams->interval * modParams->numOfSends);
  return true;
}

void resetTxRuntimeState(bool clearQueuedData = true) {
  OtaNonce = 0;
  FHSSsetCurrIndex(0);
  g_lastTlmPacketRecvMs = 0;
  g_syncPacketLastSentMs = 0;
  g_lastHandsetLinkStatsMs = 0;
  g_telemetryRcvPhase = ttrpTransmitting;
  g_txBusyTransmitting = false;
  g_nextPacketIsDataUl = false;
  g_syncSlot = 0;
  g_bindingSendCount = 0;
  connectionHasModelMatch = false;
  DataUlSender.ResetState();
  DataDlReceiver.ResetState();
  g_syncSpamCounter = 0;
  g_syncSpamCounterAfterRateChange = 0;
  g_rateChangeStartedMs = 0;
  g_lastBenchStatusLogMs = 0;
  g_lastBenchStatusValid = false;
  if (clearQueuedData) {
    g_txOtaConnector.resetOutputQueue();
  }
}

bool applyRuntimeRateIndex(uint8_t index, bool clearQueuedData) {
  const bool wasRunning = g_txRunning;
  if (wasRunning) {
    hwTimer::stop();
  }
  if (g_txRadioReady) {
    Radio.SetTxIdleMode();
  }

  resetTxRuntimeState(clearQueuedData);
  const bool ok = configureStartupRate(index, false);
  if (ok && g_txRadioReady) {
    Radio.SetFrequencyReg(FHSSgetInitialFreq(), SX12XX_Radio_All, false);
    connectionState = awaitingModelId;
    g_rateChangeStartedMs = millis();
    armSyncSpam(0, SYNC_SPAM_AMOUNT_AFTER_RATE_CHANGE);
  }

  if (wasRunning) {
    hwTimer::resume();
  }
  return ok;
}

void maybeApplyPendingRateChange() {
  if (!g_rateChangePending || g_txBindingMode || InBindingMode ||
      !hasSupportedRateIndex(g_requestedRateIndex) ||
      ExpressLRS_currAirRate_Modparams == nullptr) {
    return;
  }

  if (g_syncSpamCounter != 0U || g_txBusyTransmitting ||
      g_telemetryRcvPhase != ttrpTransmitting || DataUlSender.IsActive()) {
    return;
  }

  if (currentRateIndex() == g_requestedRateIndex) {
    g_rateChangePending = false;
    g_rateChangeStartedMs = 0;
    return;
  }

  if (!applyRuntimeRateIndex(g_requestedRateIndex, false)) {
    DBGLN("ELRS TX failed pending rate change %u", g_requestedRateIndex);
    return;
  }

  g_rateChangePending = false;
  DBGLN("ELRS TX applied requested rate %u", g_requestedRateIndex);
}

void maybeReportBenchFixedRate() {
#if defined(SIW917_ELRS_TX_PC_BENCH)
  if (!g_benchFixedRateLogged && g_txRunning && g_txRadioReady &&
      ExpressLRS_currAirRate_Modparams != nullptr) {
    DBGLN("[TXBENCH] fixed RF rate bench mode - no automatic packet-rate "
          "cycling");
    g_benchFixedRateLogged = true;
  }
#endif
}

void sendLinkStatsToHandset() {
  if (firmwareOptions.tlm_report_interval == 0U) {
    return;
  }

  const uint32_t now = millis();
  if ((uint32_t)(now - g_lastHandsetLinkStatsMs) <
      firmwareOptions.tlm_report_interval) {
    return;
  }

  CRSF_MK_FRAME_T(crsfLinkStatistics_t) linkStatisticsFrame = {};
  linkStatisticsFrame.h.sync_byte = CRSF_SYNC_BYTE;
  linkStatisticsFrame.h.frame_size =
      CRSF_FRAME_SIZE(sizeof(crsfLinkStatistics_t));
  linkStatisticsFrame.h.type = CRSF_FRAMETYPE_LINK_STATISTICS;
  linkStatisticsFrame.p.uplink_RSSI_1 =
      (uint8_t)(g_txLinkStats.rssi_1 < 0 ? -g_txLinkStats.rssi_1 : 0);
  linkStatisticsFrame.p.uplink_RSSI_2 =
      (uint8_t)(g_txLinkStats.rssi_2 < 0 ? -g_txLinkStats.rssi_2 : 0);
  linkStatisticsFrame.p.uplink_Link_quality = g_txLinkStats.lq;
  linkStatisticsFrame.p.uplink_SNR = g_txLinkStats.snr;
  linkStatisticsFrame.p.active_antenna = g_txLinkStats.active_ant;
  linkStatisticsFrame.p.rf_Mode = g_txLinkStats.rf_mode;
  linkStatisticsFrame.p.uplink_TX_Power = linkStats.uplink_TX_Power;
  linkStatisticsFrame.p.downlink_RSSI_1 = 0;
  linkStatisticsFrame.p.downlink_Link_quality = 0;
  linkStatisticsFrame.p.downlink_SNR = 0;
  linkStatisticsFrame.crc = crsfRouter.crsf_crc.calc(
      (uint8_t *)&linkStatisticsFrame.h.type,
      linkStatisticsFrame.h.frame_size - 1U);
  crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER,
                              &linkStatisticsFrame.h);
  g_lastHandsetLinkStatsMs = now;
}

uint32_t telemetryLossTimeoutMs() {
  constexpr uint32_t RX_LOSS_CNT = 5U;
  constexpr uint32_t MIN_TIMEOUT_MS = 512U;
  if (ExpressLRS_currAirRate_Modparams == nullptr) {
    return MIN_TIMEOUT_MS + 2U;
  }

  const uint32_t scaledTimeout =
      (uint32_t)ExpressLRS_currTlmDenom *
      ExpressLRS_currAirRate_Modparams->interval / (1000U / RX_LOSS_CNT);
  return (scaledTimeout > MIN_TIMEOUT_MS ? scaledTimeout : MIN_TIMEOUT_MS) +
         2U;
}

void updateConnectionStateFromTelemetry() {
  if (!g_txRunning || ExpressLRS_currAirRate_RFperfParams == nullptr) {
    return;
  }

  if (g_lastTlmPacketRecvMs == 0) {
    if (!g_txBindingMode && connectionState == awaitingModelId &&
        g_rateChangeStartedMs != 0U) {
      const uint32_t now = millis();
      const uint32_t timeoutMs =
          ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs;
      if ((uint32_t)(now - g_rateChangeStartedMs) > timeoutMs) {
        connectionState = disconnected;
        g_rateChangeStartedMs = 0;
      }
      return;
    }

    if (!g_txBindingMode && connectionState == connected) {
      connectionState = disconnected;
    }
    return;
  }

  const uint32_t now = millis();
  const uint32_t timeoutMs = telemetryLossTimeoutMs();
  if ((uint32_t)(now - g_lastTlmPacketRecvMs) <= timeoutMs) {
    if (!g_txBindingMode) {
      connectionState = connected;
      g_rateChangeStartedMs = 0;
    }
  } else if (!g_txBindingMode) {
    connectionState = disconnected;
    g_rateChangeStartedMs = 0;
  }
}

void updateTelemetryLinkStats(const OTA_LinkStats_s *stats) {
  g_txLinkStats.rssi_1 = -(int8_t)stats->uplink_RSSI_1;
  g_txLinkStats.rssi_2 = -(int8_t)stats->uplink_RSSI_2;
  g_txLinkStats.lq = stats->lq;
  g_txLinkStats.snr = SNR_DESCALE(stats->SNR);
  g_txLinkStats.active_ant = stats->antenna;
  connectionHasModelMatch = stats->modelMatch;
}

bool processDownlinkPacket(SX12xxDriverCommon::rx_status status) {
  if (status != SX12xxDriverCommon::SX12XX_RX_OK) {
    return false;
  }

  OTA_Packet_s *const otaPktPtr = (OTA_Packet_s *const)Radio.RXdataBuffer;
  if (!OtaValidatePacketCrc(otaPktPtr)) {
    return false;
  }

  Radio.GetLastPacketStats();
  g_lastTlmPacketRecvMs = millis();
  if (ExpressLRS_currAirRate_Modparams != nullptr) {
    g_txLinkStats.rf_mode =
        (uint8_t)ExpressLRS_currAirRate_Modparams->enum_rate;
  }

  if (OtaIsFullRes) {
    if (otaPktPtr->full.data_dl.packetType == PACKET_TYPE_LINKSTATS) {
      updateTelemetryLinkStats(&otaPktPtr->full.data_dl.ul_link_stats.stats);
      DataUlSender.ConfirmCurrentPayload(otaPktPtr->full.data_dl.stubbornAck);
      DataDlReceiver.ReceiveData(
          otaPktPtr->full.data_dl.packageIndex,
          otaPktPtr->full.data_dl.ul_link_stats.payload,
          sizeof(otaPktPtr->full.data_dl.ul_link_stats.payload));
    } else if (otaPktPtr->full.data_dl.packetType == PACKET_TYPE_DATA) {
      DataUlSender.ConfirmCurrentPayload(otaPktPtr->full.data_dl.stubbornAck);
      DataDlReceiver.ReceiveData(otaPktPtr->full.data_dl.packageIndex,
                                 otaPktPtr->full.data_dl.payload,
                                 sizeof(otaPktPtr->full.data_dl.payload));
    }
  } else if (otaPktPtr->std.type == PACKET_TYPE_LINKSTATS) {
    updateTelemetryLinkStats(&otaPktPtr->std.data_dl.ul_link_stats.stats);
    DataUlSender.ConfirmCurrentPayload(otaPktPtr->std.data_dl.stubbornAck);
    DataDlReceiver.ReceiveData(otaPktPtr->std.data_dl.packageIndex,
                               otaPktPtr->std.data_dl.ul_link_stats.payload,
                               sizeof(otaPktPtr->std.data_dl.ul_link_stats.payload));
  } else if (otaPktPtr->std.type == PACKET_TYPE_DATA) {
    DataUlSender.ConfirmCurrentPayload(otaPktPtr->std.data_dl.stubbornAck);
    DataDlReceiver.ReceiveData(otaPktPtr->std.data_dl.packageIndex,
                               otaPktPtr->std.data_dl.payload,
                               sizeof(otaPktPtr->std.data_dl.payload));
  }

  updateConnectionStateFromTelemetry();
  return true;
}

void processCompletedDataDlFrame() {
  if (!DataDlReceiver.HasFinishedData()) {
    return;
  }

  const uint32_t frameLen =
      g_dataDlBuffer[CRSF_TELEMETRY_LENGTH_INDEX] + CRSF_FRAME_NOT_COUNTED_BYTES;
  if (frameLen >= CRSF_MIN_PACKET_LEN && frameLen <= CRSF_MAX_PACKET_LEN &&
      (g_dataDlBuffer[0] == CRSF_SYNC_BYTE ||
       g_dataDlBuffer[0] == CRSF_ADDRESS_CRSF_RECEIVER ||
       g_dataDlBuffer[0] == CRSF_ADDRESS_RADIO_TRANSMITTER)) {
    crsfRouter.processMessage(
        &g_txOtaConnector,
        reinterpret_cast<const crsf_header_t *>(g_dataDlBuffer));
  }

  DataDlReceiver.Unlock();
}

void generateSyncPacketData(OTA_Sync_s *const syncPtr) {
  g_syncPacketLastSentMs = millis();
  syncPtr->fhssIndex = FHSSgetCurrIndex();
  syncPtr->nonce = OtaNonce;
  const expresslrs_mod_settings_s *const syncRate =
      get_elrs_airRateConfig(advertisedSyncRateIndex());
  syncPtr->rfRateEnum = syncRate != nullptr ? syncRate->enum_rate
                                            : ExpressLRS_currAirRate_Modparams->enum_rate;
  syncPtr->switchEncMode = smWideOr8ch;
  syncPtr->newTlmRatio = (uint8_t)(effectiveTlmRatioSetting() - TLM_RATIO_NO_TLM);
  syncPtr->geminiMode = false;
  syncPtr->otaProtocol = TX_NORMAL_MODE;
  syncPtr->UID4 = UID[4];
  syncPtr->UID5 = crsfTransmitter.getSyncUID5();
  consumeSyncSpamCounter();
}

void sendRcDataToRf() {
  if (!g_txRunning || ExpressLRS_currAirRate_Modparams == nullptr ||
      ExpressLRS_currAirRate_RFperfParams == nullptr) {
    return;
  }

  const uint32_t lastRcData = handset->GetRCdataLastRecv();
  if (lastRcData != 0U && (uint32_t)(micros() - lastRcData) > 1000000U) {
    return;
  }

  WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {};
  const uint32_t now = millis();
  const bool isTlmDisarmed =
      ((expresslrs_tlm_ratio_e)g_tlmRatioSetting) == TLM_RATIO_DISARMED;
  const uint32_t syncIntervalMs =
      (connectionState == connected && !isTlmDisarmed)
          ? ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalConnected
          : ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalDisconnected;
  const uint8_t nonceFhssResult =
      OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
  const bool skipSync =
      g_txBindingMode || InBindingMode ||
      (isTlmDisarmed && isArmed && ExpressLRS_currTlmDenom == 1U);
  const bool sendSyncSpam =
      (g_syncSpamCounter != 0U ||
       (g_syncSpamCounterAfterRateChange != 0U && FHSSonSyncChannel())) &&
      (nonceFhssResult == 1U || nonceFhssResult == 2U);
  const bool sendRegularSync =
      !skipSync && (g_syncSlot / 2U) <= nonceFhssResult &&
      ((uint32_t)(now - g_syncPacketLastSentMs) >= syncIntervalMs) &&
      FHSSonSyncChannel();

  if (sendSyncSpam || sendRegularSync) {
    otaPkt.std.type = PACKET_TYPE_SYNC;
    if (OtaIsFullRes) {
      otaPkt.full.sync.packetType = PACKET_TYPE_SYNC;
      generateSyncPacketData(&otaPkt.full.sync.sync);
    } else {
      generateSyncPacketData(&otaPkt.std.sync);
    }
    if (sendSyncSpam) {
      g_syncSlot = 0;
    } else {
      g_syncSlot = (uint8_t)((g_syncSlot + 1U) %
                             (ExpressLRS_currAirRate_Modparams->FHSShopInterval *
                              2U));
    }
  } else if (g_nextPacketIsDataUl && DataUlSender.IsActive()) {
    otaPkt.std.type = PACKET_TYPE_DATA;
    if (OtaIsFullRes) {
      otaPkt.full.data_ul.packageIndex = DataUlSender.GetCurrentPayload(
          otaPkt.full.data_ul.payload, sizeof(otaPkt.full.data_ul.payload));
      otaPkt.full.data_ul.stubbornAck = DataDlReceiver.GetCurrentConfirm();
    } else {
      otaPkt.std.data_ul.packageIndex = DataUlSender.GetCurrentPayload(
          otaPkt.std.data_ul.payload, sizeof(otaPkt.std.data_ul.payload));
      otaPkt.std.data_ul.stubbornAck = DataDlReceiver.GetCurrentConfirm();
    }
    g_nextPacketIsDataUl = false;
    if (g_txBindingMode || InBindingMode) {
      ++g_bindingSendCount;
    }
  } else {
    g_nextPacketIsDataUl = true;
    OtaPackChannelData(&otaPkt, ChannelData,
                       DataDlReceiver.GetCurrentConfirm());
  }

  OtaGeneratePacketCrc(&otaPkt);
  g_txBusyTransmitting = true;
  if (Radio.HasPendingOutputPower()) {
    Radio.SetTxIdleMode();
    Radio.CommitOutputPowerForNextTx();
  }
  Radio.TXnb((uint8_t *)&otaPkt, false, (uint8_t *)&otaPkt, SX12XX_Radio_All);
}

void nonceAdvance() {
  OtaNonce++;
  if (((OtaNonce + 1U) % ExpressLRS_currAirRate_Modparams->FHSShopInterval) ==
      0U) {
    ++FHSSptr;
  }
}

bool ICACHE_RAM_ATTR txRxDoneISR(SX12xxDriverCommon::rx_status status) {
  if (g_txBusyTransmitting || g_telemetryRcvPhase != ttrpExpectingTelem) {
    return false;
  }
  return processDownlinkPacket(status);
}

void ICACHE_RAM_ATTR txTxDoneISR() {
  if (!g_txBusyTransmitting || ExpressLRS_currAirRate_Modparams == nullptr) {
    return;
  }

  const uint8_t modResult =
      (OtaNonce + 1U) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
  const bool nextIsTelemetrySlot =
      (ExpressLRS_currTlmDenom != 1U) &&
      (((OtaNonce + 1U) % ExpressLRS_currTlmDenom) == 0U);
  const bool doRx = nextIsTelemetrySlot;

  if (!InBindingMode && modResult == 0U) {
    Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_All, doRx);
  } else if (doRx) {
    Radio.RXnb();
  }

  g_telemetryRcvPhase =
      nextIsTelemetrySlot ? ttrpPreReceiveGap : ttrpTransmitting;
  g_txBusyTransmitting = false;
}

void ICACHE_RAM_ATTR txTimerCallback() {
  if (!g_txRunning || !g_txRadioReady || g_txBusyTransmitting ||
      ExpressLRS_currAirRate_Modparams == nullptr) {
    return;
  }

  if ((OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends) == 0U) {
    handset->JustSentRFpacket();
  }

  if (!InBindingMode && !g_txBindingMode) {
    nonceAdvance();
  }

  if (g_telemetryRcvPhase == ttrpPreReceiveGap) {
    g_telemetryRcvPhase = ttrpExpectingTelem;
    return;
  }

  g_telemetryRcvPhase = ttrpTransmitting;
  sendRcDataToRf();
}
} // namespace

CRSFRouter crsfRouter;
Handset *handset = &g_crsfHandset;
StubbornSender DataUlSender;
StubbornReceiver DataDlReceiver;
bool isArmed = false;
uint8_t UID[UID_LEN] = {0};
bool connectionHasModelMatch = false;
bool teamraceHasModelMatch = true;
bool InBindingMode = false;
uint8_t ExpressLRS_currTlmDenom = 1;
expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams = nullptr;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams = nullptr;
uint32_t ChannelData[CRSF_NUM_CHANNELS] = {};
connectionState_e connectionState = disconnected;
RXtimerState_e RXtimerState = tim_disconnected;

extern "C" void elrs_tx_abort_data_uplink(void) {
  DataUlSender.ResetState();
  g_txOtaConnector.resetOutputQueue();
  g_nextPacketIsDataUl = false;
}

extern "C" bool elrs_tx_init(void) {
  g_txInitialized = false;
  g_txRadioReady = false;
  g_txRunning = false;
  g_txWifiMode = false;
  g_txBindingMode = false;
  InBindingMode = false;
  connectionState = disconnected;
  RXtimerState = tim_disconnected;
  g_requestedRateIndex = 0;
  g_rateChangePending = false;
  g_syncSpamCounter = 0;
  g_syncSpamCounterAfterRateChange = 0;
  g_rateChangeStartedMs = 0;
#if defined(SIW917_ELRS_TX_PC_BENCH)
  g_benchFixedRateLogged = false;
#endif
  initDefaultChannels();
  if (TX_PC_BENCH_MODE) {
    DBGLN("[TXBENCH] PC RF bench mode active - no radio handset required");
    DBGLN("[TXBENCH] Internal RC channels: roll/pitch/yaw=mid throttle=low "
          "AUX1=disarmed");
  }
  memset(&g_txLinkStats, 0, sizeof(g_txLinkStats));
  ExpressLRS_currAirRate_Modparams = nullptr;
  ExpressLRS_currAirRate_RFperfParams = nullptr;
  g_txRateName = "TX Bootstrap";

  status_led_init();
  status_led_set_mode(LED_MODE_DISCONNECTED);

  if (elrs_config_init() != 0) {
    connectionState = hardwareUndefined;
    DBGLN("ELRS TX config init failed");
    return false;
  }

  if (!options_init()) {
    connectionState = hardwareUndefined;
    DBGLN("ELRS TX options init failed");
    return false;
  }

  if (firmwareOptions.hasUID) {
    memcpy(UID, firmwareOptions.uid, UID_LEN);
  } else {
    memset(UID, 0, sizeof(UID));
  }

  OtaUpdateCrcInitFromUid();
  FHSSrandomiseFHSSsequence(uidMacSeedGetTx());
  ::new (static_cast<void *>(&crsfRouter)) CRSFRouter();
  ::new (static_cast<void *>(&crsfTransmitter)) TXModuleEndpoint();
  ::new (static_cast<void *>(&g_txOtaConnector)) TXOTAConnector();
  ::new (static_cast<void *>(&DataUlSender)) StubbornSender();
  ::new (static_cast<void *>(&DataDlReceiver)) StubbornReceiver();
  DataDlReceiver.SetDataToReceive(g_dataDlBuffer, sizeof(g_dataDlBuffer));
  resetTxRuntimeState();

  crsfTransmitter.begin();
  crsfRouter.addEndpoint(&crsfTransmitter);
  crsfRouter.addConnector(&g_txOtaConnector);

  handset->Begin();

  Radio.RXdoneCallback = SX12xxDriverCommon::nullCallbackRx;
  Radio.TXdoneCallback = SX12xxDriverCommon::nullCallbackTx;
  Radio.currFreq = FHSSgetInitialFreq();

  g_txRadioReady = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
  if (!g_txRadioReady) {
    connectionState = radioFailed;
    DBGLN("ELRS TX radio init failed");
    return false;
  }

  const elrs_config_t *cfg = elrs_config_get();
  g_tlmRatioSetting =
      (cfg != nullptr && cfg->tlm_interval <= (uint8_t)TLM_RATIO_DISARMED)
          ? cfg->tlm_interval
          : (uint8_t)TLM_RATIO_STD;
  schedulePowerUpdate(cfg != nullptr ? cfg->tx_power : ELRS_TX_POWER_DEFAULT_DBM);
  crsfTransmitter.modelId = cfg != nullptr ? cfg->model_id : 0xFF;
  crsfTransmitter.modelMatchEnabled =
      cfg != nullptr && cfg->model_id <= MODELMATCH_MASK;

  const uint8_t startupRateIndex = chooseStartupRateIndex();
  g_requestedRateIndex = startupRateIndex;
  if (!configureStartupRate(startupRateIndex, false)) {
    connectionState = radioFailed;
    DBGLN("ELRS TX startup rate config failed");
    return false;
  }

  crsfTransmitter.updateParameters();
  bind_button_init();
  bind_button_set_callback(onTxBindButtonEvent);

  g_txInitialized = true;
  DBGLN("ELRS TX bootstrap init complete");
  return true;
}

extern "C" void elrs_tx_start(void) {
  if (!g_txInitialized || !g_txRadioReady) {
    return;
  }
  g_txWifiMode = false;
  resetTxRuntimeState();
  connectionState = disconnected;
  Radio.RXdoneCallback = txRxDoneISR;
  Radio.TXdoneCallback = txTxDoneISR;
  Radio.SetFrequencyReg(FHSSgetInitialFreq(), SX12XX_Radio_All, false);
  hwTimer::init(nullptr, txTimerCallback);
  hwTimer::resume();
  g_txRunning = true;
  updateTxStatusLedMode();
  DBGLN("ELRS TX scheduler started");
  logTxBenchStatus(true);
}

extern "C" void elrs_tx_loop(void) {
  bind_button_poll();
  updateTxStatusLedMode();
  status_led_update();
  handset->handleInput();
  hwTimer::service();
  refreshTlmSettings();
  updateConnectionStateFromTelemetry();
  processCompletedDataDlFrame();
  if (g_txBindingMode || InBindingMode) {
    if (g_bindingSendCount > BINDING_SPAM_AMOUNT) {
      elrs_exit_binding_mode();
    }
  } else {
    tryQueuePendingModelIdSync();
    if (!DataUlSender.IsActive()) {
      g_txOtaConnector.pumpSender();
    }
  }
  maybeApplyPendingRateChange();
  maybeReportBenchFixedRate();
  sendLinkStatsToHandset();
  logTxBenchStatus();
}

extern "C" void elrs_tx_stop(void) {
  if (g_txRunning) {
    hwTimer::stop();
  }
  if (g_txRadioReady) {
    Radio.SetTxIdleMode();
  }
  Radio.RXdoneCallback = SX12xxDriverCommon::nullCallbackRx;
  Radio.TXdoneCallback = SX12xxDriverCommon::nullCallbackTx;
  g_txRunning = false;
  g_txBusyTransmitting = false;
  g_telemetryRcvPhase = ttrpTransmitting;
  DBGLN("ELRS TX scheduler stopped");
}

extern "C" elrs_connection_state_t elrs_get_connection_state(void) {
  if (g_txBindingMode || InBindingMode) {
    return ELRS_BINDING;
  }
  switch (connectionState) {
  case connected:
    return ELRS_CONNECTED;
  case radioFailed:
  case hardwareUndefined:
    return ELRS_RADIO_FAILED;
  default:
    return ELRS_DISCONNECTED;
  }
}

extern "C" bool elrs_is_connected(void) { return connectionState == connected; }

extern "C" uint8_t elrs_get_channels(uint32_t *channels) {
  if (channels != nullptr) {
    memcpy(channels, ChannelData, sizeof(ChannelData));
  }
  return CRSF_NUM_CHANNELS;
}

extern "C" void elrs_get_link_stats(elrs_link_stats_t *stats) {
  if (stats != nullptr) {
    *stats = g_txLinkStats;
  }
}

extern "C" void elrs_set_channel_callback(elrs_channel_callback_t callback) {
  (void)callback;
}

extern "C" void elrs_enter_binding_mode(void) {
  if (g_txBindingMode) {
    return;
  }
  if (g_txRunning) {
    hwTimer::stop();
  }
  g_txBindingMode = true;
  InBindingMode = true;
  g_rateChangePending = false;
  connectionState = disconnected;
  resetTxRuntimeState();
  (void)configureStartupRate(chooseBindingRateIndex(), true);
  queueBindPayload();
  if (g_txRunning) {
    Radio.SetFrequencyReg(FHSSgetInitialFreq(), SX12XX_Radio_All, false);
    hwTimer::resume();
  }
  status_led_set_mode(LED_MODE_BINDING);
  crsfTransmitter.updateParameters();
  DBGLN("ELRS TX bind mode active");
}

extern "C" void elrs_exit_binding_mode(void) {
  if (!g_txBindingMode) {
    return;
  }
  if (g_txRunning) {
    hwTimer::stop();
  }
  g_txBindingMode = false;
  InBindingMode = false;
  g_rateChangePending = false;
  resetTxRuntimeState();
  const uint8_t restoreRate =
      hasSupportedRateIndex(g_requestedRateIndex) ? g_requestedRateIndex
                                                  : chooseStartupRateIndex();
  (void)configureStartupRate(restoreRate, false);
  if (g_txRunning) {
    Radio.SetFrequencyReg(FHSSgetInitialFreq(), SX12XX_Radio_All, false);
    hwTimer::resume();
  }
  status_led_set_mode(LED_MODE_DISCONNECTED);
  crsfTransmitter.updateParameters();
  DBGLN("ELRS TX bind mode exited");
}

extern "C" void elrs_enter_wifi_mode(void) {
  g_txWifiMode = true;
  status_led_set_mode(LED_MODE_WIFI);
  DBGLN("ELRS TX WiFi bootstrap placeholder");
}

extern "C" bool elrs_is_wifi_mode(void) { return g_txWifiMode; }

extern "C" bool elrs_tx_request_rx_wifi_mode(void) {
  static const uint8_t payload[] = {MSP_ELRS_SET_RX_WIFI_MODE};
  return queueControlPayload(payload, sizeof(payload));
}

extern "C" void elrs_tx_schedule_model_id_sync(uint8_t modelId) {
  g_pendingModelId = modelId <= MODELMATCH_MASK ? modelId : 0xFF;
  g_modelIdSyncPending = true;
  tryQueuePendingModelIdSync();
}

extern "C" const char *elrs_tx_rate_name_for_index(uint8_t index) {
  const expresslrs_mod_settings_s *modParams = get_elrs_airRateConfig(index);
  return modParams != nullptr ? rateNameForEnum(modParams->enum_rate) : "ELRS";
}

extern "C" bool elrs_tx_is_rate_change_pending(void) {
  return rateChangeTransitionActive();
}

extern "C" uint8_t elrs_tx_get_rate_index(void) {
  if (hasSupportedRateIndex(g_requestedRateIndex)) {
    return g_requestedRateIndex;
  }

  const uint8_t activeRateIndex = currentRateIndex();
  return activeRateIndex < RATE_MAX ? activeRateIndex : chooseStartupRateIndex();
}

extern "C" bool elrs_tx_set_rate_index(uint8_t index) {
  if (!hasSupportedRateIndex(index) || g_txBindingMode || InBindingMode) {
    return false;
  }

  g_requestedRateIndex = index;
  if (currentRateIndex() == index) {
    g_rateChangePending = false;
    armSyncSpam(0, 0);
    return true;
  }

  if (connectionState == connected) {
    g_rateChangePending = true;
    armSyncSpam(SYNC_SPAM_AMOUNT, 0);
    return true;
  }

  g_rateChangePending = false;
  return applyRuntimeRateIndex(index, false);
}

extern "C" uint8_t elrs_tx_get_tlm_ratio_setting(void) {
  return g_tlmRatioSetting;
}

extern "C" bool elrs_tx_set_tlm_ratio_setting(uint8_t tlmRatio) {
  if (tlmRatio > (uint8_t)TLM_RATIO_DISARMED) {
    return false;
  }

  g_tlmRatioSetting = tlmRatio;
  refreshTlmSettings();
  g_syncPacketLastSentMs = 0;
  return true;
}

extern "C" int8_t elrs_tx_get_max_power_dbm(void) { return g_powerDbm; }

extern "C" bool elrs_tx_set_max_power_dbm(int8_t dbm) {
  schedulePowerUpdate(dbm);
  return true;
}

extern "C" int elrs_tx_save_config_with_rf_rearm(void) {
  if (InBindingMode || connectionState >= NO_CONFIG_SAVE_STATES) {
    return 1;
  }

  const bool wasRunning = g_txRunning;
  if (wasRunning) {
    hwTimer::stop();
  }
  if (g_txRadioReady) {
    Radio.SetTxIdleMode();
  }
  const int saveResult = elrs_config_save();
  if (wasRunning) {
    Radio.SetFrequencyReg(FHSSgetInitialFreq(), SX12XX_Radio_All, false);
    hwTimer::resume();
  }
  return saveResult;
}

extern "C" bool elrs_set_uid(const uint8_t new_uid[6]) {
  if (new_uid == nullptr) {
    return false;
  }

  memcpy(UID, new_uid, UID_LEN);
  OtaUpdateCrcInitFromUid();
  FHSSrandomiseFHSSsequence(uidMacSeedGetTx());
  resetTxRuntimeState();
  if (g_txRunning) {
    Radio.SetFrequencyReg(FHSSgetInitialFreq(), SX12XX_Radio_All, false);
  }
  return true;
}

extern "C" uint8_t elrs_get_lq(void) { return g_txLinkStats.lq; }

extern "C" const char *elrs_get_rate_name(void) { return g_txRateName; }
