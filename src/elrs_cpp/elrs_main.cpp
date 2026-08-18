/**
 * @file elrs_main.cpp
 * @brief Main ELRS RX application for SiW917
 *
 * This is a working RX implementation adapted from upstream ELRS 4.0
 * for the SiW917 platform with LR1121 radio.
 *
 * Key differences from upstream:
 * - CRSF serial output uses the SiW917 platform UART driver
 * - Board services (LEDs, button, WiFi/NVM) are provided by SiW917 modules
 * - Dual LR1121 operation supports FCC915 normal, same-band Gemini, and
 *   LR1121 crossband with SiW917-specific HAL timing adaptations
 */

#include "elrs_main.h"

// ELRS includes
#include "FHSS.h"
#include "LQCALC.h"
#include "LR1121.h"
#include "LR1121Driver.h"
#include "LR1121_hal.h"
#include "LowPassFilter.h"
#include "MeanAccumulator.h"
#include "OTA.h"
#include "PFD.h"
#include "CRSFRouter.h"
#include "RXOTAConnector.h"
#include "SiW917RXEndpoint.h"
#include "common.h"
#include "crc.h"
#include "crsf_protocol.h"
#include "elrs_task_wakeup.h"
#include "hwTimer.h"
#include "logging.h"
#include "options.h"
#include "stubborn_receiver.h"
#include "stubborn_sender.h"
#include "telemetry_protocol.h"
#include "msptypes.h"
#include "siw917_elrs_timing.h"
#include "mlrs_ota.h"

// Standard includes
#include <new>
#include <stdio.h>
#include <string.h>

// Status LED, bind button, persistent config, WiFi integration, and CRSF output
extern "C" {
#include "bind_button.h"
#include "ble_remote_id.h"
#include "crsf_serial.h"
#include "elrs_config.h"
#include "status_led.h"
#include "wifi_http_test.h"
int ble_remote_id_service_start(bool enabled);
void elrs_cpp_request_wifi_mode(void);
int lr1121_dio1_read(void);
int lr1121_dio2_read(void);
uint32_t lr1121_dio1_irq_enabled(void);
uint32_t lr1121_dio1_irq_pending(void);
uint32_t lr1121_dio1_gpio_intr_status(void);
uint32_t lr1121_dio1_get_isr_count(void);
uint32_t lr1121_dio2_get_isr_count(void);
uint32_t lr1121_hal_get_last_dio1_edge_us(void);
uint32_t lr1121_hal_get_last_deferred_us(void);
uint32_t lr1121_hal_get_direct_dio_count(void);
uint32_t lr1121_hal_get_direct_reentrant_count(void);
uint32_t lr1121_hal_get_level_requeue_count(void);
uint32_t lr1121_hal_get_stage_max_us(void);
uint32_t lr1121_hal_get_deferred_max_us(void);
uint32_t lr1121_get_last_rxnbisr_entry_us(void);
uint32_t lr1121_get_last_packet_ready_us(void);
uint32_t lr1121_get_busy_fast_max_iterations(void);
uint32_t lr1121_get_busy_fast_fail_count(void);
uint32_t lr1121_get_raw_gspi_max_us(void);
uint32_t lr1121_get_raw_gspi_count(void);
uint32_t lr1121_get_raw_gspi_fail_count(void);
bool lr1121_hal_prepare_radio2_image_calibration(bool highBand);
void lr1121_get_isr_stats(uint32_t *isr_count, uint32_t *rx_count,
                          uint32_t *tx_count, uint32_t *other_count,
                          uint32_t *last_irq);
void lr1121_get_radio_isr_stats(uint32_t *isr_1, uint32_t *isr_2,
                                uint32_t *rx_1, uint32_t *rx_2);
bool lr1121_get_status(uint8_t *stat1, uint8_t *stat2, uint8_t *irq_status);
void elrs_enter_binding_mode(void);
}

//// CONSTANTS ////
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
// Desired buffer time between Packet ISR and Tock ISR. Keep this aligned with
// upstream ELRS; platform latency belongs in the HAL/timer adapter, not here.
#define PACKET_TO_TOCK_SLACK 200
#define CRSF_RC_OUTPUT_INTERVAL 4 // Output RC channels every 4ms (~250Hz)
#define SBUS_RC_OUTPUT_INTERVAL 9 // Upstream SBUS period is 9ms
#define SUMD_RC_OUTPUT_INTERVAL 10 // Upstream SUMD period is 10ms
#define RFmodeCycleMultiplierSlow 10
#define BindingRateChangeCyclePeriodMs 125U
#define ELRS_DIAG_DISABLE_DOWNLINK_TLM SIW917_ELRS_DISABLE_DOWNLINK_TLM
#define ELRS_DIAG_DISABLE_CRSF_SERIAL SIW917_ELRS_DISABLE_CRSF_SERIAL
#define ELRS_DIAG_USE_DIO_PFD_TIMESTAMP SIW917_ELRS_DIO_PFD_TIMESTAMP
#define ELRS_DIAG_CRC_NONCE_WINDOW (!SIW917_ELRS_TIMING_TEST_BUILD ? 64 : 0)
#define ELRS_DIAG_PACKET_CAPTURE (!SIW917_ELRS_TIMING_TEST_BUILD)
#define ELRS_DIAG_TX_TURNAROUND (!SIW917_ELRS_TIMING_TEST_BUILD)
#define ELRS_DIAG_TLM_RATE_LOG SIW917_ELRS_TLM_RATE_DIAG
#define ELRS_DIAG_TLM_STALE_RX 0
#define ELRS_DIAG_TLM150_SNAPSHOT (!SIW917_ELRS_TIMING_TEST_BUILD)
#define ELRS_DIAG_RATE_CHANGE_LOG (!SIW917_ELRS_TIMING_TEST_BUILD)
#define ELRS_DIAG_PERIODIC_STATS SIW917_ELRS_DISCONNECTED_SCAN_DIAG
#define ELRS_DIAG_RF_RATE_LOG SIW917_ELRS_RF_RATE_DIAG
#define ELRS_DIAG_PERIODIC_STATS_WHEN_CONNECTED 0
#define ELRS_DIAG_PRINT_AFTER_LOSS 0
#define ELRS_DIAG_LOSS_PACKET_STATS 0
#define ELRS_DIAG_RX_LUA_UL 0
#define ELRS_DIAG_RX_LUA_UL_VERBOSE 0
#define ELRS_DIAG_RX_LUA_UL_PRINT_LIMIT 24
#define ELRS_DIAG_RX_LUA_UL_PRINT_EVERY 128
#define ELRS_DIAG_RX_LUA_LOAD_SNAPSHOT SIW917_ELRS_LUA_PROGRESS_DIAG
#define ELRS_DIAG_LUA_PROGRESS SIW917_ELRS_LUA_PROGRESS_DIAG
#define ELRS_DIAG_LINK_PROGRESS SIW917_ELRS_LINK_PROGRESS_DIAG
#define ELRS_DIAG_RX_LUA_LOAD_DELAY_MS 1500U
#define ELRS_DIAG_RX_LUA_LOAD_COOLDOWN_MS 5000U
#define ELRS_DIAG_RX_LUA_BIND_POST_DELAY_MS 250U
#define ELRS_DIAG_CRSF_OTA_VERBOSE 0
#define ELRS_DIAG_LUA_DISCOVERY 0
#define ELRS_DIAG_TX_POWER 0
#define ELRS_DIAG_DYNPOWER_STATS SIW917_ELRS_DYNPOWER_STATS_DIAG
#define SIW917_ELRS_PERSIST_STARTUP_RATE 0
#define STARTUP_RATE_SAVE_DELAY_MS 5000U
///////////////////

// Model match ID (0xFF = disabled, 0-63 = specific model)
static uint8_t modelMatchId = 0xFF;
static volatile bool forceTelemetryOff = false;

extern "C" void siw917_rx_set_model_match_id(uint8_t modelId) {
  modelMatchId = modelId;
}

extern "C" void siw917_rx_set_force_telemetry_off(uint8_t forceOff) {
  forceTelemetryOff = forceOff != 0;
}

// Link statistics (for external API)
static elrs_link_stats_t currentLinkStats = {};

// Channel callback
static elrs_channel_callback_t channelCallback = nullptr;

// Antenna tracking
uint8_t antenna = 0;
uint8_t geminiMode = 0;

// Connection timing
static const uint32_t ConsiderConnGoodMillis = 1000;

//=============================================================================
// Global state variables (required by ELRS protocol)
//=============================================================================
connectionState_e connectionState = disconnected;
uint8_t UID[UID_LEN] = {0};
bool connectionHasModelMatch = false;
bool teamraceHasModelMatch = true;
static bool lastConnectionHadModelMatch = false;
bool InBindingMode = false;
static bool bindingModeRequest = false;
bool InWiFiMode = false; // WiFi configuration mode
uint8_t ExpressLRS_currTlmDenom = 1;

enum teamraceOutputInhibitState_e : uint8_t {
  troiPass = 0,
  troiDisableAwaitConfirm,
  troiInhibit,
  troiEnableAwaitConfirm,
};

static teamraceOutputInhibitState_e teamraceOutputInhibitState = troiPass;
static uint8_t lastTeamracePosition = 0;

expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams = nullptr;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams = nullptr;

uint32_t ChannelData[CRSF_NUM_CHANNELS];

// Radio driver instance (defined in common.cpp)
extern LR1121Driver Radio;

// Hardware Abstraction Layer instance
static LR1121Hal radioHal;

// Link stats (defined in common.cpp)
extern elrsLinkStatistics_t linkStats;

// CRC calculator
Crc2Byte ota_crc;

// PFD (Phase Frequency Detector) for timing synchronization
PFD PFDloop;

// LQ calculation
LQCALC<100> LQCalc;
uint8_t uplinkLQ = 0;

// Low pass filters
LPF LPF_Offset(2);
LPF LPF_OffsetDx(4);
LPF LPF_UplinkRSSI0(5);
LPF LPF_UplinkRSSI1(5);
MeanAccumulator<int32_t, int8_t, -16> SnrMean;

// Timing variables
volatile uint32_t LastValidPacket = 0;
volatile uint32_t LastSyncPacket = 0;
static uint32_t RFmodeLastCycled = 0;
static uint8_t RFmodeCycleMultiplier = 1;
static bool LockRFmode = false;
static int8_t SwitchModePending = 0;
static tx_transmission_mode_e TxOtaProtocol = TX_NORMAL_MODE;
static bool warnedUnsupportedTxProtocol = false;
static uint32_t lastObservedUplinkTxPowerDecodeCount = 0;
static uint8_t lastDecodedUplinkTxPower = 0;
static uint8_t lastDecodedUplinkTxPowerSource = 0;
static uint8_t lastDecodedUplinkTxPowerSwitchMode = 0xFF;
static uint8_t lastDecodedUplinkTxPowerRate = 0xFF;
static volatile bool uplinkTxPowerChangePending = false;
static volatile uint8_t pendingUplinkTxPower = 0;
static volatile uint8_t pendingUplinkTxPowerSource = 0;
static volatile uint8_t pendingUplinkTxPowerFullRes = 0;
static volatile uint8_t pendingUplinkTxPowerSwitchMode = 0;
static volatile uint8_t pendingUplinkTxPowerRate = 0;
static volatile uint8_t pendingUplinkTxPowerNonce = 0;

// Rate/mode scanning
static uint8_t scanIndex = 0;
uint8_t ExpressLRS_nextAirRateIndex = 0;
static bool startupRateSavePending = false;
static uint8_t startupRateSaveIndex = 0;
static uint32_t startupRateSaveAtMs = 0;

// Timer state
RXtimerState_e RXtimerState = tim_disconnected;
static bool doStartTimer = false;

// Connection tracking
uint32_t GotConnectionMillis = 0;

// Phase lock
int32_t PfdPrevRawOffset = 0;
static volatile int32_t pfdLastRawOffset = 0;
static volatile int32_t pfdLastNormalizedOffset = 0;
static volatile int32_t pfdLastOffset = 0;
static volatile int32_t pfdLastOffsetDx = 0;
static volatile int32_t pfdLastPhaseShift = 0;
static volatile uint32_t pfdResultCount = 0;

enum DisconnectReason : uint8_t {
  DISC_NONE = 0,
  DISC_RX_LOCK_TIMEOUT,
  DISC_PACKET_TIMEOUT,
  DISC_RATE_CHANGE,
  DISC_EXTERNAL
};

static volatile DisconnectReason lastDisconnectReason = DISC_NONE;

static const char *disconnectReasonName(DisconnectReason reason) {
  switch (reason) {
  case DISC_NONE:
    return "none";
  case DISC_RX_LOCK_TIMEOUT:
    return "rx-lock-timeout";
  case DISC_PACKET_TIMEOUT:
    return "packet-timeout";
  case DISC_RATE_CHANGE:
    return "rate-change";
  case DISC_EXTERNAL:
    return "external";
  default:
    return "unknown";
  }
}
#if ELRS_DIAG_PRINT_AFTER_LOSS
static volatile bool diagPrintAfterLossPending = false;
#endif

// Cycle interval for rate scanning
uint32_t cycleInterval = 0;

// TLM state

// Telemetry TX (downlink to TX module)
StubbornSender TelemetrySender;
StubbornReceiver DataUlReceiver;
static uint8_t TelemetryBuffer[ELRS_DATA_UL_BUFFER];
static uint8_t DataUlBuffer[ELRS_DATA_UL_BUFFER];
CRSFRouter crsfRouter;
static RXOTAConnector otaConnector;
static SiW917RXEndpoint crsfReceiver;
static uint8_t NextTelemetryType = PACKET_TYPE_LINKSTATS;
static uint8_t telemetryBurstCount = 0;
static uint8_t telemetryBurstMax = 1;
static bool telemBurstValid = false;
static bool alreadyTLMresp = false;
static volatile uint32_t telemetryTxCount = 0;
static volatile uint32_t telemetrySuppressedCount = 0;
static volatile uint32_t telemetryRcConfirmCount = 0;
static volatile bool telemetryLastRcConfirm = false;
static volatile uint32_t telemetryDataUlAckCount = 0;
static volatile bool telemetryLastDataUlAck = false;
static volatile uint32_t dataUlChunkCount = 0;
static volatile uint32_t dataUlCompleteCount = 0;
static volatile uint32_t dataUlHandledCount = 0;
static volatile uint32_t dataUlMalformedCount = 0;
static volatile uint32_t dataUlLastChunkMs = 0;
static volatile uint32_t dataUlLastCompleteMs = 0;
static volatile uint8_t dataUlLastPackageIndex = 0;
static volatile uint8_t dataUlLastPayload0 = 0;
static volatile uint8_t dataUlLastPayload1 = 0;
static volatile uint8_t dataUlLastFrameType = 0;
static volatile uint8_t dataUlLastFrameLen = 0;
static volatile uint32_t crcFailTypeCount[4] = {};
static volatile uint32_t crcPassTypeCount[4] = {};
static volatile uint32_t rxLqCurrentSetSkipCount = 0;
#if SIW917_ELRS_PACKET_STATS_DIAG
#define ELRS_PACKET_STAT_INC(var_) ((var_)++)
#define ELRS_PACKET_STAT_SET(var_, value_) ((var_) = (value_))
#else
#define ELRS_PACKET_STAT_INC(var_) do { } while (0)
#define ELRS_PACKET_STAT_SET(var_, value_) do { } while (0)
#endif
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
static volatile uint32_t telemetryBuildMaxUs = 0;
static volatile uint32_t telemetrySendMaxUs = 0;
static volatile uint32_t telemetryHandleMaxUs = 0;

static inline void updateHotpathMax(volatile uint32_t &maxValue,
                                    uint32_t durationUs) {
  if (durationUs > maxValue) {
    maxValue = durationUs;
  }
}
#endif
#if SIW917_ELRS_PREBUILD_TLM_PACKET
static WORD_ALIGNED_ATTR OTA_Packet_s prebuiltTelemetryPacket = {};
static WORD_ALIGNED_ATTR OTA_Packet_s prebuiltTelemetryGeminiPacket = {};
static StubbornSenderPreparedPayload prebuiltTelemetryPayload = {};
static bool prebuiltTelemetryValid = false;
static bool prebuiltTelemetrySendGemini = false;
static uint8_t prebuiltTelemetryNonce = 0;
static uint8_t prebuiltNextTelemetryType = PACKET_TYPE_LINKSTATS;
static uint8_t prebuiltTelemetryBurstCount = 0;
static volatile uint32_t telemetryPrebuildCount = 0;
static volatile uint32_t telemetryPrebuildHitCount = 0;
static volatile uint32_t telemetryPrebuildMissCount = 0;
#endif
#if SIW917_ELRS_DEFER_TLM_TX_FROM_TIMER
static WORD_ALIGNED_ATTR OTA_Packet_s deferredTelemetryPacket = {};
static WORD_ALIGNED_ATTR OTA_Packet_s deferredTelemetryGeminiPacket = {};
static volatile bool deferredTelemetryPending = false;
static volatile bool deferredTelemetrySendGemini = false;
static volatile uint8_t deferredTelemetryNonce = 0;
static volatile uint8_t deferredTelemetryFhss = 0;
static volatile uint32_t telemetryDeferredQueueCount = 0;
static volatile uint32_t telemetryDeferredSendCount = 0;
static volatile uint32_t telemetryDeferredDropCount = 0;

static inline uint32_t telemetryEnterCritical() {
  uint32_t primask;
  __asm volatile("mrs %0, primask" : "=r"(primask));
  __asm volatile("cpsid i" ::: "memory");
  return primask;
}

static inline void telemetryExitCritical(uint32_t primask) {
  __asm volatile("msr primask, %0" ::"r"(primask) : "memory");
}
#endif
#if ELRS_DIAG_TX_TURNAROUND
static volatile uint32_t telemetryTxStartUs = 0;
static volatile uint32_t telemetryTxDoneUs = 0;
static volatile uint32_t telemetryRxnbDoneUs = 0;
static volatile uint32_t telemetryTxToDoneUs = 0;
static volatile uint32_t telemetryDoneToRxnbDoneUs = 0;
static volatile uint32_t telemetryTxToRxnbDoneUs = 0;
static volatile uint32_t telemetryRxnbToRxOkUs = 0;
static volatile uint32_t telemetryMissedAfterTx = 0;
static volatile uint32_t telemetryTxBeforeRx = 0;
static volatile uint32_t telemetryRxAfterTx = 0;
static volatile uint8_t telemetryAwaitingRx = 0;
static volatile uint8_t telemetryStaleRxReported = 0;
static volatile uint32_t telemetryStaleRxReportCount = 0;
static volatile uint32_t telemetryStaleRxLastAgeUs = 0;
static volatile uint32_t telemetryLqSetWhileAwaitingCount = 0;
static volatile uint8_t telemetryTxNonce = 0;
static volatile uint8_t telemetryTxFhss = 0;
static volatile uint8_t telemetryRxNonce = 0;
static volatile uint8_t telemetryRxFhss = 0;
#if ELRS_DIAG_TLM150_SNAPSHOT
static bool telemetry150DiagNeedArm = false;
static bool telemetry150DiagArmed = false;
static bool telemetry150DiagPrinted = false;
static uint32_t telemetry150DiagArmMs = 0;
static uint32_t telemetry150DiagArmTxCount = 0;
static uint32_t telemetry150DiagArmRxAfterTx = 0;
static uint32_t telemetry150DiagArmTxBeforeRx = 0;
static uint32_t telemetry150DiagArmRcConfirm = 0;
static uint32_t telemetry150DiagArmDataUlAck = 0;
#endif
#endif
static volatile bool dataUlReady = false;
static volatile bool uidSavePending = false;
static volatile bool uidRefreshPending = false;
static volatile bool bindCompletePending = false;
static uint8_t pendingUid[UID_LEN] = {0};

static void ICACHE_RAM_ATTR InvalidatePrebuiltTelemetry();

#if ELRS_DIAG_LUA_DISCOVERY
static bool luaDiscoveryActive = false;
static uint8_t luaDiscoveryType = 0;
static uint8_t luaDiscoveryLen = 0;
static uint8_t luaDiscoveryPrintCount = 0;
static uint32_t luaDiscoveryStartMs = 0;
static uint32_t luaDiscoveryLastPrintMs = 0;
static uint32_t luaDiscoveryStartTxCount = 0;
static volatile uint32_t luaDiscoveryPingCount = 0;
#endif

// Raw SNR is kept for diagnostics; OTA link stats use SnrMean like upstream.
static int8_t lastSnrRaw = 0;

static int8_t crsfPowerToDbm(uint8_t crsfPower) {
  switch (crsfPower) {
  case 1:
    return 10; // 10 mW
  case 2:
    return 14; // 25 mW
  case 3:
    return 20; // 100 mW
  case 4:
    return 27; // 500 mW
  case 5:
    return 30; // 1000 mW
  case 6:
    return 33; // 2000 mW
  case 7:
    return 24; // 250 mW
  case 8:
    return 17; // 50 mW
  default:
    return 10;
  }
}

static constexpr int8_t RX_TLM_POWER_DBM_BY_SELECTION[] = {
    10, 14, 17, 20,
};
static constexpr uint8_t RX_TLM_POWER_MATCH_TX_SELECTION =
    (uint8_t)(sizeof(RX_TLM_POWER_DBM_BY_SELECTION) /
              sizeof(RX_TLM_POWER_DBM_BY_SELECTION[0]));
static constexpr char RX_TLM_POWER_OPTIONS[] =
    "10;25;50;100;MatchTX";

static int8_t clampRxTelemetryPowerDbm(int8_t dbm, bool isSubGHz) {
  const int8_t maxDbm =
      isSubGHz ? ELRS_TX_POWER_SUBGHZ_MAX_DBM : ELRS_TX_POWER_2G4_MAX_DBM;

  if (dbm < ELRS_TX_POWER_MIN_DBM) {
    return ELRS_TX_POWER_MIN_DBM;
  }
  if (dbm > maxDbm) {
    return maxDbm;
  }
  return dbm;
}

static int8_t rxTlmPowerSelectionToDbm(uint8_t selection) {
  if (selection >= RX_TLM_POWER_MATCH_TX_SELECTION) {
    return ELRS_TX_POWER_MATCH_TX_DBM;
  }

  return RX_TLM_POWER_DBM_BY_SELECTION[selection];
}

static uint8_t rxTlmPowerDbmToSelection(int8_t dbm) {
  if (dbm == ELRS_TX_POWER_MATCH_TX_DBM) {
    return RX_TLM_POWER_MATCH_TX_SELECTION;
  }

  for (uint8_t i = 0; i < RX_TLM_POWER_MATCH_TX_SELECTION; ++i) {
    if (RX_TLM_POWER_DBM_BY_SELECTION[i] == dbm) {
      return i;
    }
  }

  return 3; // 100 mW / 20 dBm default.
}

static void updateRxDownlinkPower(bool initialize) {
  static int8_t requestedDbm = 127;

  const elrs_config_t *cfg = elrs_config_get();
  int8_t desiredDbm = cfg ? cfg->tx_power : ELRS_TX_POWER_DEFAULT_DBM;
  const bool matchTxPower = (desiredDbm == ELRS_TX_POWER_MATCH_TX_DBM);
  uint8_t matchedCrsfPower = 0;

  if (matchTxPower) {
    matchedCrsfPower = linkStats.uplink_TX_Power;
    if (matchedCrsfPower == 0) {
      if (!initialize) {
        return;
      }
      desiredDbm = ELRS_TX_POWER_DEFAULT_DBM;
    } else {
      desiredDbm = crsfPowerToDbm(matchedCrsfPower);
    }
  }

  const int8_t subGhzDbm = clampRxTelemetryPowerDbm(desiredDbm, true);
  const int8_t highBandDbm = clampRxTelemetryPowerDbm(desiredDbm, false);
  const bool capped = (subGhzDbm != desiredDbm);

  if (!initialize && subGhzDbm == requestedDbm) {
    return;
  }

  requestedDbm = subGhzDbm;

  // Program both LR1121 PA tables. On SiW917 the pending value is committed
  // just before the next telemetry TX instead of from TX_DONE.
  Radio.SetOutputPower(subGhzDbm, true);
  Radio.SetOutputPower(highBandDbm, false);
  if (initialize
#if SIW917_ELRS_LOG_CONNECTED_PWR_UPDATES
      || true
#else
      || !hwTimer::isRunning()
#endif
  ) {
    DBGLN("RX downlink power scheduled: %d dBm%s", subGhzDbm,
          capped ? (matchTxPower ? " (matching TX, capped)" : " (capped)")
                 : (matchTxPower ? " (matching TX)" : ""));
  }
}

static void ICACHE_RAM_ATTR noteDecodedUplinkTxPower() {
  const uint32_t decodeCount = OtaUplinkPowerDecodeCount;
  if (decodeCount == lastObservedUplinkTxPowerDecodeCount) {
    return;
  }
  lastObservedUplinkTxPowerDecodeCount = decodeCount;

  const uint8_t decodedPower = linkStats.uplink_TX_Power;
  const uint8_t decodeSource = OtaLastUplinkPowerDecodeSource;
  const uint8_t fullRes = OtaIsFullRes ? 1U : 0U;
  const uint8_t switchMode = (uint8_t)OtaSwitchModeCurrent;
  const uint8_t rate = ExpressLRS_currAirRate_Modparams
                           ? ExpressLRS_currAirRate_Modparams->index
                           : 0xFFU;

  if (decodedPower == 0) {
    return;
  }

  const bool newContext =
      decodedPower != lastDecodedUplinkTxPower ||
      decodeSource != lastDecodedUplinkTxPowerSource ||
      switchMode != lastDecodedUplinkTxPowerSwitchMode ||
      rate != lastDecodedUplinkTxPowerRate;
  if (!newContext) {
    return;
  }

  lastDecodedUplinkTxPower = decodedPower;
  lastDecodedUplinkTxPowerSource = decodeSource;
  lastDecodedUplinkTxPowerSwitchMode = switchMode;
  lastDecodedUplinkTxPowerRate = rate;
  pendingUplinkTxPower = decodedPower;
  pendingUplinkTxPowerSource = decodeSource;
  pendingUplinkTxPowerFullRes = fullRes;
  pendingUplinkTxPowerSwitchMode = switchMode;
  pendingUplinkTxPowerRate = rate;
  pendingUplinkTxPowerNonce = OtaNonce;
  uplinkTxPowerChangePending = true;
}

static void processDecodedUplinkTxPower() {
  if (!uplinkTxPowerChangePending) {
    return;
  }

  const uint8_t power = pendingUplinkTxPower;
  const uint8_t source = pendingUplinkTxPowerSource;
  const uint8_t fullRes = pendingUplinkTxPowerFullRes;
  const uint8_t switchMode = pendingUplinkTxPowerSwitchMode;
  const uint8_t rate = pendingUplinkTxPowerRate;
  const uint8_t nonce = pendingUplinkTxPowerNonce;
  uplinkTxPowerChangePending = false;

#if ELRS_DIAG_TX_POWER
  DBGLN("TXPWR decoded enum=%u dbm=%d src=%u full=%u switch=%u rate=%u nonce=%u",
        power, crsfPowerToDbm(power), source, fullRes, switchMode, rate, nonce);
#endif

  updateRxDownlinkPower(false);
}

//=============================================================================
// RX Lua / CRSF-over-OTA bridge
//=============================================================================
//
// Downstream ESP32 routes uplink DATA frames through CRSFRouter/RXEndpoint and
// feeds RXOTAConnector into StubbornSender. This is the same protocol shape, but
// kept deliberately small for the SiW917 bring-up so the TX Lua can discover the
// receiver and read/write a safe starter set of parameters.

enum RxLuaCommandStep : uint8_t {
  RX_LUA_CMD_IDLE = 0,
  RX_LUA_CMD_CLICK = 1,
  RX_LUA_CMD_EXECUTING = 2,
  RX_LUA_CMD_ASK_CONFIRM = 3,
  RX_LUA_CMD_CONFIRMED = 4,
  RX_LUA_CMD_CANCEL = 5,
  RX_LUA_CMD_QUERY = 6,
};

enum RxLuaParamId : uint8_t {
  RX_LUA_PARAM_ROOT = 0,
  RX_LUA_PARAM_PROTOCOL = 1,
  RX_LUA_PARAM_FAILSAFE = 2,
  RX_LUA_PARAM_TARGET_SYS_ID = 3,
  RX_LUA_PARAM_SOURCE_SYS_ID = 4,
  RX_LUA_PARAM_FORCE_TLM = 5,
  RX_LUA_PARAM_WIFI = 6,
  RX_LUA_PARAM_TEAM_RACE = 7,
  RX_LUA_PARAM_TEAM_RACE_CHANNEL = 8,
  RX_LUA_PARAM_TEAM_RACE_POSITION = 9,
  RX_LUA_PARAM_BIND_STORAGE = 10,
  RX_LUA_PARAM_BIND = 11,
  RX_LUA_PARAM_MODEL_ID = 12,
  RX_LUA_PARAM_TLM_RATIO = 13,
  RX_LUA_PARAM_VERSION = 14,
  RX_LUA_PARAM_TLM_POWER = 15,
  RX_LUA_PARAM_ACTIVE_MODE = 16,
  RX_LUA_PARAM_BLE_REMOTE_ID = 17,
  RX_LUA_PARAM_COUNT = RX_LUA_PARAM_BLE_REMOTE_ID,
};

static uint8_t getConfiguredBindStorage();
extern "C" bool elrs_is_on_loan(void);
extern "C" void elrs_apply_bind_storage_change(uint8_t bindStorage);

static const char *rxLuaBindCommandName() {
  if (getConfiguredBindStorage() == ELRS_BIND_STORAGE_ADMINISTERED) {
    return "Bind Admin Only";
  }
  return elrs_is_on_loan() ? "Return Model" : "Enter Bind Mode";
}

static char rxLuaModelIdValue[8] = "Off";
static char rxLuaTlmRatioValue[8] = "1:1";
static char rxLuaActiveModeValue[24] = "CRSF";
static uint8_t rxLuaWifiCommandStep = RX_LUA_CMD_IDLE;
static uint8_t rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
static const char *rxLuaWifiCommandInfo = "";
static const char *rxLuaBindCommandInfo = "";
static bool rxLuaWifiPending = false;
static bool rxLuaBindPending = false;
static uint32_t rxLuaPendingActionAtMs = 0;
static volatile uint32_t rxLuaUlChunkCount = 0;
static volatile uint32_t rxLuaUlCompleteCount = 0;
static volatile uint32_t rxLuaCrsfHandledCount = 0;
static volatile uint32_t rxLuaCrsfRejectCount = 0;
static volatile uint32_t rxLuaCrsfIgnoredCount = 0;
static volatile uint8_t rxLuaLastUlPackageIndex = 0;
static volatile uint32_t rxLuaQueueDropCount = 0;

#define RX_LUA_QUEUE_DEPTH 4
#define RX_LUA_PENDING_ACTION_DELAY_MS 200U
#define RX_LUA_CONFIG_SAVE_DELAY_MS 500U
static uint8_t rxLuaQueue[RX_LUA_QUEUE_DEPTH][CRSF_FRAME_SIZE_MAX];
static uint8_t rxLuaQueueLen[RX_LUA_QUEUE_DEPTH] = {};
static uint8_t rxLuaQueueHead = 0;
static uint8_t rxLuaQueueTail = 0;
static uint8_t rxLuaQueueCount = 0;
static bool rxLuaConfigSavePending = false;
static bool rxLuaSerialApplyPending = false;
static uint32_t rxLuaConfigSaveAtMs = 0;

#if ELRS_DIAG_RX_LUA_LOAD_SNAPSHOT
static bool rxLuaDiagSnapshotPending = false;
static uint8_t rxLuaDiagSnapshotTrigger = 0;
static uint8_t rxLuaDiagSnapshotParam = 0xFF;
static uint8_t rxLuaDiagSnapshotAux = 0;
static uint32_t rxLuaDiagSnapshotStartMs = 0;
static uint32_t rxLuaDiagSnapshotAtMs = 0;
static uint32_t rxLuaDiagLastSnapshotMs = 0;
static uint32_t rxLuaDiagStartUlChunkCount = 0;
static uint32_t rxLuaDiagStartUlCompleteCount = 0;
static uint32_t rxLuaDiagStartCrsfHandledCount = 0;
static uint32_t rxLuaDiagStartCrsfRejectCount = 0;
static uint32_t rxLuaDiagStartCrsfIgnoredCount = 0;
static uint32_t rxLuaDiagStartQueueDropCount = 0;
static uint32_t rxLuaDiagStartTelemetryTxCount = 0;
static uint32_t rxLuaDiagStartTelemetrySuppressedCount = 0;
static uint32_t rxLuaDiagStartTelemetryDataUlAckCount = 0;
#if ELRS_DIAG_TX_TURNAROUND
static uint32_t rxLuaDiagStartTelemetryRxAfterTx = 0;
static uint32_t rxLuaDiagStartTelemetryMissedAfterTx = 0;
static uint32_t rxLuaDiagStartTelemetryTxBeforeRx = 0;
#endif

static bool rxLuaBindPostDiagPending = false;
static uint8_t rxLuaBindPostDiagChunk = 0;
static uint32_t rxLuaBindPostDiagStartMs = 0;
static uint32_t rxLuaBindPostDiagAtMs = 0;
static uint32_t rxLuaBindPostDiagStartUlChunkCount = 0;
static uint32_t rxLuaBindPostDiagStartUlCompleteCount = 0;
static uint32_t rxLuaBindPostDiagStartCrsfHandledCount = 0;
static uint32_t rxLuaBindPostDiagStartQueueDropCount = 0;
static uint32_t rxLuaBindPostDiagStartTelemetryTxCount = 0;
static uint32_t rxLuaBindPostDiagStartTelemetrySuppressedCount = 0;
static uint32_t rxLuaBindPostDiagStartTelemetryDataUlAckCount = 0;
#if ELRS_DIAG_TX_TURNAROUND
static uint32_t rxLuaBindPostDiagStartTelemetryRxAfterTx = 0;
static uint32_t rxLuaBindPostDiagStartTelemetryMissedAfterTx = 0;
static uint32_t rxLuaBindPostDiagStartTelemetryTxBeforeRx = 0;
#endif

static void rxLuaArmLoadSnapshot(uint8_t frameType, uint8_t parameterIndex,
                                 uint8_t aux, bool force) {
  const uint32_t nowMs = millis();

  if (!force &&
      (rxLuaDiagSnapshotPending ||
      (rxLuaDiagLastSnapshotMs != 0 &&
       (uint32_t)(nowMs - rxLuaDiagLastSnapshotMs) <
           ELRS_DIAG_RX_LUA_LOAD_COOLDOWN_MS))) {
    return;
  }

  rxLuaDiagSnapshotPending = true;
  rxLuaDiagSnapshotTrigger = frameType;
  rxLuaDiagSnapshotParam = parameterIndex;
  rxLuaDiagSnapshotAux = aux;
  rxLuaDiagSnapshotStartMs = nowMs;
  rxLuaDiagSnapshotAtMs =
      nowMs + (force ? 0U : ELRS_DIAG_RX_LUA_LOAD_DELAY_MS);
  rxLuaDiagStartUlChunkCount = rxLuaUlChunkCount;
  rxLuaDiagStartUlCompleteCount = rxLuaUlCompleteCount;
  rxLuaDiagStartCrsfHandledCount = rxLuaCrsfHandledCount;
  rxLuaDiagStartCrsfRejectCount = rxLuaCrsfRejectCount;
  rxLuaDiagStartCrsfIgnoredCount = rxLuaCrsfIgnoredCount;
  rxLuaDiagStartQueueDropCount = rxLuaQueueDropCount;
  rxLuaDiagStartTelemetryTxCount = telemetryTxCount;
  rxLuaDiagStartTelemetrySuppressedCount = telemetrySuppressedCount;
  rxLuaDiagStartTelemetryDataUlAckCount = telemetryDataUlAckCount;
#if ELRS_DIAG_TX_TURNAROUND
  rxLuaDiagStartTelemetryRxAfterTx = telemetryRxAfterTx;
  rxLuaDiagStartTelemetryMissedAfterTx = telemetryMissedAfterTx;
  rxLuaDiagStartTelemetryTxBeforeRx = telemetryTxBeforeRx;
#endif
}

static void rxLuaArmBindPostSnapshot(uint8_t fieldChunk) {
  const uint32_t nowMs = millis();

  rxLuaBindPostDiagPending = true;
  rxLuaBindPostDiagChunk = fieldChunk;
  rxLuaBindPostDiagStartMs = nowMs;
  rxLuaBindPostDiagAtMs = nowMs + ELRS_DIAG_RX_LUA_BIND_POST_DELAY_MS;
  rxLuaBindPostDiagStartUlChunkCount = rxLuaUlChunkCount;
  rxLuaBindPostDiagStartUlCompleteCount = rxLuaUlCompleteCount;
  rxLuaBindPostDiagStartCrsfHandledCount = rxLuaCrsfHandledCount;
  rxLuaBindPostDiagStartQueueDropCount = rxLuaQueueDropCount;
  rxLuaBindPostDiagStartTelemetryTxCount = telemetryTxCount;
  rxLuaBindPostDiagStartTelemetrySuppressedCount = telemetrySuppressedCount;
  rxLuaBindPostDiagStartTelemetryDataUlAckCount = telemetryDataUlAckCount;
#if ELRS_DIAG_TX_TURNAROUND
  rxLuaBindPostDiagStartTelemetryRxAfterTx = telemetryRxAfterTx;
  rxLuaBindPostDiagStartTelemetryMissedAfterTx = telemetryMissedAfterTx;
  rxLuaBindPostDiagStartTelemetryTxBeforeRx = telemetryTxBeforeRx;
#endif
}
#endif

#if ELRS_DIAG_RX_LUA_UL
static bool rxLuaShouldPrintDiag(uint32_t count) {
  return count <= ELRS_DIAG_RX_LUA_UL_PRINT_LIMIT ||
         (ELRS_DIAG_RX_LUA_UL_PRINT_EVERY != 0 &&
          (count % ELRS_DIAG_RX_LUA_UL_PRINT_EVERY) == 0);
}

static void rxLuaDiagReject(const char *reason, const uint8_t *frame) {
  const uint32_t count = ++rxLuaCrsfRejectCount;
  if (!rxLuaShouldPrintDiag(count)) {
    return;
  }

  if (frame == nullptr) {
    DBGLN("[RX_LUA] CRSF reject #%lu %s null",
          (unsigned long)count, reason);
    return;
  }

  DBGLN("[RX_LUA] CRSF reject #%lu %s bytes:%02X %02X %02X %02X %02X %02X %02X %02X",
        (unsigned long)count, reason, frame[0], frame[1], frame[2], frame[3],
        frame[4], frame[5], frame[6], frame[7]);
}

static void rxLuaDiagIgnore(const char *reason, const uint8_t *frame) {
  const uint32_t count = ++rxLuaCrsfIgnoredCount;
  if (!rxLuaShouldPrintDiag(count)) {
    return;
  }

  DBGLN("[RX_LUA] CRSF ignore #%lu %s type=0x%02X dest=0x%02X orig=0x%02X",
        (unsigned long)count, reason, frame[CRSF_TELEMETRY_TYPE_INDEX],
        frame[3], frame[4]);
}
#endif

static uint8_t *appendCString(uint8_t *dst, const char *src) {
  if (src == nullptr) {
    src = "";
  }

  do {
    *dst++ = (uint8_t)*src;
  } while (*src++ != '\0');

  return dst;
}

static uint8_t selectionOptionMax(const char *options) {
  uint8_t max = 0;
  if (options == nullptr) {
    return 0;
  }

  while (*options != '\0') {
    if (*options++ == ';') {
      max++;
    }
  }
  return max;
}

static uint8_t clampU8(uint8_t value, uint8_t minValue, uint8_t maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

static const char *rxLuaModelIdOptions() {
  static char options[200] = {};
  static bool initialized = false;

  if (!initialized) {
    size_t used = (size_t)snprintf(options, sizeof(options), "Off");
    for (uint8_t i = 0; i <= 63 && used < sizeof(options); i++) {
      int written = snprintf(options + used, sizeof(options) - used, ";%u", i);
      if (written <= 0) {
        break;
      }
      used += (size_t)written;
    }
    options[sizeof(options) - 1] = '\0';
    initialized = true;
  }

  return options;
}

static uint8_t rxLuaModelIdToSelection(uint8_t modelId) {
  return modelId == 0xFF ? 0 : (uint8_t)(clampU8(modelId, 0, 63) + 1);
}

static uint8_t rxLuaSelectionToModelId(uint8_t selection) {
  return selection == 0 ? 0xFF : (uint8_t)(clampU8(selection, 1, 64) - 1);
}

static void rxLuaHandleCommandWrite(uint8_t arg, uint8_t *step,
                                    const char **info, bool *pending) {
  if (step == nullptr || info == nullptr || pending == nullptr) {
    return;
  }

  switch (arg) {
  case RX_LUA_CMD_CLICK:
    *step = RX_LUA_CMD_ASK_CONFIRM;
    *info = "Confirm";
    *pending = false;
    break;
  case RX_LUA_CMD_CONFIRMED:
    *step = RX_LUA_CMD_EXECUTING;
    *info = "Entering...";
    *pending = true;
    rxLuaPendingActionAtMs = millis();
    break;
  case RX_LUA_CMD_CANCEL:
    *step = RX_LUA_CMD_IDLE;
    *info = "";
    *pending = false;
    break;
  case RX_LUA_CMD_QUERY:
  default:
    break;
  }
}

static void rxLuaHandleBindCommandWrite(uint8_t arg) {
  if (getConfiguredBindStorage() == ELRS_BIND_STORAGE_ADMINISTERED) {
    rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
    rxLuaBindCommandInfo = "Admin only";
    rxLuaBindPending = false;
    return;
  }

  // Mirrors upstream RXParameters: bind enters only after the TX polls QUERY.
  if (arg == RX_LUA_CMD_QUERY) {
    rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
    rxLuaBindCommandInfo = "";
    rxLuaBindPending = true;
    rxLuaPendingActionAtMs = millis();
  } else if (arg < RX_LUA_CMD_CANCEL) {
    rxLuaBindCommandStep = RX_LUA_CMD_EXECUTING;
    rxLuaBindCommandInfo = "Entering...";
    rxLuaBindPending = false;
  } else {
    rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
    rxLuaBindCommandInfo = "";
    rxLuaBindPending = false;
  }
}

static uint32_t versionStringToU32(const char *verStr) {
  uint32_t retVal = 0;
  uint8_t accumulator = 0;
  bool trailingData = false;

  while (verStr != nullptr && *verStr != '\0') {
    const char c = *verStr++;
    if (c == '.') {
      retVal = (retVal << 8) | accumulator;
      accumulator = 0;
      trailingData = false;
    } else if (c >= '0' && c <= '9') {
      accumulator = (uint8_t)((accumulator * 10) + (c - '0'));
      trailingData = true;
    } else {
      break;
    }
  }

  if (trailingData) {
    retVal = (retVal << 8) | accumulator;
  }
  return retVal < 0x010000 ? ((uint32_t)OTA_VERSION_ID << 16) : retVal;
}

static uint8_t crsfCalcFrameCrc(const uint8_t *data, uint8_t len);

static void setCrsfHeaderAndCrc(uint8_t *frame, crsf_frame_type_e frameType,
                                uint8_t frameSize) {
  frame[0] = CRSF_SYNC_BYTE;
  frame[1] = frameSize;
  frame[2] = frameType;
  frame[frameSize + CRSF_FRAME_NOT_COUNTED_BYTES - 1] =
      crsfCalcFrameCrc(frame + CRSF_FRAME_NOT_COUNTED_BYTES, frameSize - 1);
}

static void setCrsfExtendedHeaderAndCrc(uint8_t *frame,
                                        crsf_frame_type_e frameType,
                                        uint8_t frameSize,
                                        crsf_addr_e destAddr,
                                        crsf_addr_e origAddr) {
  frame[3] = destAddr;
  frame[4] = origAddr;
  setCrsfHeaderAndCrc(frame, frameType, frameSize);
}

static uint8_t crsfCalcFrameCrc(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (uint8_t)((crc & 0x80) ? ((crc << 1) ^ CRSF_CRC_POLY)
                                  : (crc << 1));
    }
  }
  return crc;
}

static bool rxLuaIsKnownCrsfFrameAddress(uint8_t address) {
  return address == CRSF_SYNC_BYTE ||
         address == CRSF_ADDRESS_RADIO_TRANSMITTER ||
         address == CRSF_ADDRESS_CRSF_RECEIVER ||
         address == CRSF_ADDRESS_CRSF_TRANSMITTER ||
         address == CRSF_ADDRESS_ELRS_LUA;
}

static bool validateCrsfFrame(const uint8_t *frame, uint8_t *frameLen) {
  if (frame == nullptr) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagReject("null", frame);
#endif
    return false;
  }

  // CRSF byte 0 is the bus/device address, not always CRSF_SYNC_BYTE. The TX
  // Lua "Other Devices" ping arrives via OTA as 0xEE, matching upstream routing.
  if (!rxLuaIsKnownCrsfFrameAddress(frame[0])) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagReject("addr", frame);
#endif
    return false;
  }

  const uint8_t frameSize = frame[CRSF_TELEMETRY_LENGTH_INDEX];
  if (frameSize < 2 ||
      frameSize > (CRSF_FRAME_SIZE_MAX - CRSF_FRAME_NOT_COUNTED_BYTES)) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagReject("len", frame);
#endif
    return false;
  }

  const uint8_t totalLen = frameSize + CRSF_FRAME_NOT_COUNTED_BYTES;
  if (totalLen > ELRS_DATA_UL_BUFFER) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagReject("buffer", frame);
#endif
    return false;
  }

  const uint8_t crc =
      crsfCalcFrameCrc(frame + CRSF_FRAME_NOT_COUNTED_BYTES, frameSize - 1);
  if (crc != frame[totalLen - 1]) {
#if ELRS_DIAG_RX_LUA_UL
    DBGLN("[RX_LUA] Dropping CRSF frame with bad CRC type=0x%02X calc=0x%02X got=0x%02X",
          frame[CRSF_TELEMETRY_TYPE_INDEX], crc, frame[totalLen - 1]);
    rxLuaDiagReject("crc", frame);
#endif
    return false;
  }

  if (frameLen != nullptr) {
    *frameLen = totalLen;
  }
  return true;
}

static void rxLuaQueueFrame(const uint8_t *frame, uint8_t len) {
  if (frame == nullptr || len == 0 || len > CRSF_FRAME_SIZE_MAX) {
    return;
  }

  if (rxLuaQueueCount >= RX_LUA_QUEUE_DEPTH) {
    // Keep latest UI responses moving; stale parameter entries are worse than
    // dropping the oldest one during a burst of Lua polling.
    rxLuaQueueHead = (uint8_t)((rxLuaQueueHead + 1) % RX_LUA_QUEUE_DEPTH);
    rxLuaQueueCount--;
    rxLuaQueueDropCount++;
  }

  memcpy(rxLuaQueue[rxLuaQueueTail], frame, len);
  rxLuaQueueLen[rxLuaQueueTail] = len;
  rxLuaQueueTail = (uint8_t)((rxLuaQueueTail + 1) % RX_LUA_QUEUE_DEPTH);
  rxLuaQueueCount++;
#if ELRS_DIAG_RX_LUA_UL && ELRS_DIAG_RX_LUA_UL_VERBOSE
  DBGLN("[RX_LUA] queued response type=0x%02X len=%u q=%u",
        frame[CRSF_TELEMETRY_TYPE_INDEX], len, rxLuaQueueCount);
#endif
}

[[maybe_unused]] static void rxLuaPumpTelemetry() {
  if (TelemetrySender.IsActive() || rxLuaQueueCount == 0) {
    return;
  }

  const uint8_t len = rxLuaQueueLen[rxLuaQueueHead];
  memcpy(TelemetryBuffer, rxLuaQueue[rxLuaQueueHead], len);
  rxLuaQueueHead = (uint8_t)((rxLuaQueueHead + 1) % RX_LUA_QUEUE_DEPTH);
  rxLuaQueueCount--;
  TelemetrySender.SetDataToTransmit(TelemetryBuffer, len);
  InvalidatePrebuiltTelemetry();
}

static uint8_t getProtocolSelectionFromConfig(const elrs_config_t *cfg) {
  if (cfg == nullptr) {
    return ELRS_SERIAL_PROTOCOL_LUA_SELECTION_CRSF;
  }

  return elrs_serial_protocol_to_lua_selection(cfg->serial_protocol);
}

static uint8_t activeSerialProtocol = ELRS_SERIAL_CRSF;

extern "C" uint8_t siw917_rx_get_active_serial_protocol(void) {
  return activeSerialProtocol;
}

static const char *serialProtocolName(uint8_t protocol) {
  switch (protocol) {
  case ELRS_SERIAL_MAVLINK:
    return "MAVLink";
  case ELRS_SERIAL_SBUS:
    return "SBUS";
  case ELRS_SERIAL_SUMD:
    return "SUMD";
  case ELRS_SERIAL_CRSF:
  default:
    return "CRSF";
  }
}

static const char *activeSerialProtocolName() {
  return serialProtocolName(activeSerialProtocol);
}

static void formatActiveModeString(char *buffer, size_t bufferLen) {
  if (buffer == nullptr || bufferLen == 0) {
    return;
  }

  const char *rfMode = FHSSuseDualBand ? " Crossband"
                       : geminiMode   ? " Gemini"
                                      : "";
  snprintf(buffer, bufferLen, "%s%s", activeSerialProtocolName(), rfMode);
  buffer[bufferLen - 1] = '\0';
}

extern "C" const char *siw917_rx_get_active_mode_string(void) {
  static char activeModeString[24] = "CRSF";
  formatActiveModeString(activeModeString, sizeof(activeModeString));
  return activeModeString;
}

static uint8_t getStoredSerialProtocol() {
  const elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr ||
      !elrs_serial_protocol_is_supported(cfg->serial_protocol)) {
    return ELRS_SERIAL_CRSF;
  }

  return cfg->serial_protocol;
}

static uint8_t getConfiguredSerialProtocol() {
  return activeSerialProtocol;
}

static uint8_t getNormalOtaSerialProtocol(uint8_t storedProtocol) {
  if (storedProtocol == ELRS_SERIAL_MAVLINK) {
    return ELRS_SERIAL_CRSF;
  }

  return elrs_serial_protocol_is_supported(storedProtocol)
             ? storedProtocol
             : (uint8_t)ELRS_SERIAL_CRSF;
}

static uint8_t getDesiredActiveSerialProtocol() {
  if (mlrs_ota_is_active()) {
    return ELRS_SERIAL_MAVLINK;
  }
  if (TxOtaProtocol == TX_MAVLINK_MODE) {
    return ELRS_SERIAL_MAVLINK;
  }

  return getNormalOtaSerialProtocol(getStoredSerialProtocol());
}

static bool updateActiveSerialProtocol() {
  const uint8_t desiredProtocol = getDesiredActiveSerialProtocol();
  if (activeSerialProtocol == desiredProtocol) {
    return false;
  }

  activeSerialProtocol = desiredProtocol;
  return true;
}

static bool serialProtocolUsesCrsf(uint8_t protocol) {
  return protocol == ELRS_SERIAL_CRSF ||
         protocol == ELRS_SERIAL_INVERTED_CRSF;
}

static bool serialProtocolUsesSbus(uint8_t protocol) {
  return protocol == ELRS_SERIAL_SBUS;
}

static bool serialProtocolUsesSumd(uint8_t protocol) {
  return protocol == ELRS_SERIAL_SUMD;
}

static bool serialProtocolSendsRc(uint8_t protocol) {
  return serialProtocolUsesCrsf(protocol) || serialProtocolUsesSbus(protocol) ||
         serialProtocolUsesSumd(protocol);
}

static bool configuredSerialProtocolUsesCrsf() {
  return serialProtocolUsesCrsf(getConfiguredSerialProtocol());
}

static bool configuredSerialProtocolSendsRc() {
  return serialProtocolSendsRc(getConfiguredSerialProtocol());
}

static uint32_t serialRcOutputIntervalMs(uint8_t protocol) {
  if (serialProtocolUsesSbus(protocol)) {
    return SBUS_RC_OUTPUT_INTERVAL;
  }
  if (serialProtocolUsesSumd(protocol)) {
    return SUMD_RC_OUTPUT_INTERVAL;
  }
  return CRSF_RC_OUTPUT_INTERVAL;
}

static int32_t mapInt32(int32_t x, int32_t inMin, int32_t inMax,
                        int32_t outMin, int32_t outMax) {
  if (inMax == inMin) {
    return outMin;
  }
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

static void prepareCrsfSerialChannels(uint32_t *outChannels) {
  if (outChannels == nullptr) {
    return;
  }

  memcpy(outChannels, ChannelData, CRSF_NUM_CHANNELS * sizeof(outChannels[0]));

  // Upstream CRSF serial uses channels 15/16 for LQ/RSSI unless the OTA mode is
  // carrying a real 16-channel payload.
  if (OtaIsFullRes && OtaSwitchModeCurrent == smHybridOr16ch) {
    return;
  }

  outChannels[14] = UINT10_to_CRSF(
      fmap((uint16_t)constrain(linkStats.uplink_Link_quality, 0, 100), 0, 100,
           0, 1023));

  const int32_t sensitivity =
      ExpressLRS_currAirRate_RFperfParams != nullptr
          ? ExpressLRS_currAirRate_RFperfParams->RXsensitivity
          : -130;
  const int32_t rssiDbm = linkStats.active_antenna == 0
                              ? -(int32_t)linkStats.uplink_RSSI_1
                              : -(int32_t)linkStats.uplink_RSSI_2;
  const int32_t clampedRssi = constrain(rssiDbm, sensitivity, -50);
  outChannels[15] =
      UINT10_to_CRSF((uint16_t)mapInt32(clampedRssi, sensitivity, -50, 0, 1023));
}

static volatile bool serialProtocolApplyRequested = false;
static volatile bool serialProtocolApplyRequiresConnected = false;
static uint32_t serialProtocolApplyDueMs = 0;
static uint8_t appliedSerialProtocol = 0xFF;
static bool serialRxEnabled = false;
static uint32_t mavlinkSerialPendingSinceMs = 0;

static constexpr uint32_t SERIAL_PROTOCOL_SYNC_APPLY_DELAY_MS = 100;
static constexpr uint8_t MAVLINK_SERIAL_PAYLOAD_MAX =
    ELRS_DATA_UL_BUFFER - CRSF_FRAME_NOT_COUNTED_BYTES;
static constexpr uint8_t MAVLINK_SERIAL_MIN_CHUNK = 24;
static constexpr uint32_t MAVLINK_SERIAL_MAX_WAIT_MS = 4;
#if SIW917_ELRS_ENABLE_CRSF_FC_TELEMETRY
static constexpr uint32_t CRSF_SERIAL_RX_BYTES_PER_LOOP = 128;
#endif

static void ICACHE_RAM_ATTR requestSerialProtocolApply() {
  serialProtocolApplyRequested = true;
  serialProtocolApplyRequiresConnected = false;
  serialProtocolApplyDueMs = 0;
}

static void ICACHE_RAM_ATTR
requestSerialProtocolApplyAfter(uint32_t nowMs, uint32_t delayMs) {
  serialProtocolApplyRequested = true;
  serialProtocolApplyRequiresConnected = true;
  serialProtocolApplyDueMs = nowMs + delayMs;
}

static void applyConfiguredSerialProtocol() {
  const uint8_t protocol = getConfiguredSerialProtocol();
  const bool wantsCrsf = serialProtocolUsesCrsf(protocol);
  const bool wantsSbus = serialProtocolUsesSbus(protocol);
  const bool wantsSumd = serialProtocolUsesSumd(protocol);
  const bool wantsRcSerial = wantsCrsf || wantsSbus || wantsSumd;
  const bool wantsMavlink = protocol == ELRS_SERIAL_MAVLINK;

#if ELRS_DIAG_DISABLE_CRSF_SERIAL
  (void)wantsCrsf;
  (void)wantsSbus;
  (void)wantsSumd;
  (void)wantsRcSerial;
  (void)wantsMavlink;
  if (crsf_serial_is_ready()) {
    crsf_serial_deinit();
  }
  const bool changed = appliedSerialProtocol != protocol;
  appliedSerialProtocol = protocol;
  if (changed) {
    DBGLN("FC serial UART disabled for RF-only build");
  }
  return;
#endif

  if (!wantsRcSerial && !wantsMavlink) {
    if (crsf_serial_is_ready()) {
      crsf_serial_deinit();
    }
    serialRxEnabled = false;
    appliedSerialProtocol = protocol;
    DBGLN("Serial protocol %u selected; UART output inactive", protocol);
    return;
  }

  if (wantsMavlink && TxOtaProtocol != TX_MAVLINK_MODE &&
      !mlrs_ota_is_active()) {
    if (crsf_serial_is_ready()) {
      crsf_serial_deinit();
    }
    serialRxEnabled = false;
    appliedSerialProtocol = 0xFF;
    DBGLN("MAVLink serial output deferred until TX MAVLink OTA mode");
    return;
  }

  const bool wasReady = crsf_serial_is_ready();
  const uint32_t baud = wantsMavlink   ? 460800U
                        : wantsSbus    ? SBUS_SERIAL_BAUDRATE
                        : wantsSumd    ? SUMD_SERIAL_BAUDRATE
                                       : firmwareOptions.uart_baud;
  const crsf_serial_format_t format =
      wantsSbus ? CRSF_SERIAL_FORMAT_8E2 : CRSF_SERIAL_FORMAT_8N1;
  const bool serialReady = (crsf_serial_init_ex(baud, format) == 0);
  if (!serialReady) {
    DBGLN("WARNING: serial init failed for protocol %u", protocol);
  } else if (!wasReady) {
    DBGLN("%s serial output initialized at %lu baud",
          serialProtocolName(protocol), (unsigned long)baud);
  } else if (appliedSerialProtocol != protocol) {
    DBGLN("%s serial output reconfigured at %lu baud",
          serialProtocolName(protocol), (unsigned long)baud);
  }

  if (serialReady && crsf_serial_is_ready()) {
    // MAVLink needs UART RX as soon as the serial mode is active. CRSF FC
    // telemetry RX is armed after RF lock so a floating/no-FC RX pin cannot
    // flood DMA/parser work while the receiver is scanning.
    const bool enableSerialRx = wantsMavlink;
    if (crsf_serial_set_rx_enabled(enableSerialRx) != 0) {
      DBGLN("WARNING: serial RX enable failed for protocol %u", protocol);
    } else {
      serialRxEnabled = enableSerialRx;
    }
  }

  appliedSerialProtocol = protocol;
}

static void processSerialProtocolApply() {
  if (!serialProtocolApplyRequested) {
    return;
  }

  if (serialProtocolApplyDueMs != 0 &&
      (int32_t)(millis() - serialProtocolApplyDueMs) < 0) {
    return;
  }

  // Upstream defers reconfigureSerial() after a SYNC-driven OTA protocol
  // change. On SiW917, keep that heavy UART reconfigure out of the RF lock
  // window entirely; the MAVLink UART is not needed until the link is valid.
  if (serialProtocolApplyRequiresConnected && connectionState != connected) {
    return;
  }

  if (!serialProtocolApplyRequiresConnected && connectionState == tentative) {
    return;
  }

  if (TelemetrySender.IsActive() || !otaConnector.IsEmpty()) {
    return;
  }

  serialProtocolApplyRequested = false;
  serialProtocolApplyRequiresConnected = false;
  serialProtocolApplyDueMs = 0;
  (void)updateActiveSerialProtocol();
  applyConfiguredSerialProtocol();
  crsfReceiver.requestActiveModeRefresh();
}

static bool queueMavlinkSerialPayload(uint32_t nowMs) {
  if (TxOtaProtocol != TX_MAVLINK_MODE ||
      getConfiguredSerialProtocol() != ELRS_SERIAL_MAVLINK ||
      !crsf_serial_is_ready()) {
    mavlinkSerialPendingSinceMs = 0;
    return false;
  }

  uint32_t available = crsf_serial_rx_available();
  if (available == 0) {
    mavlinkSerialPendingSinceMs = 0;
    return false;
  }

  if (mavlinkSerialPendingSinceMs == 0) {
    mavlinkSerialPendingSinceMs = nowMs;
  }

  if (available < MAVLINK_SERIAL_PAYLOAD_MAX &&
      available < MAVLINK_SERIAL_MIN_CHUNK &&
      (uint32_t)(nowMs - mavlinkSerialPendingSinceMs) <
          MAVLINK_SERIAL_MAX_WAIT_MS) {
    return false;
  }

  if (available > MAVLINK_SERIAL_PAYLOAD_MAX) {
    available = MAVLINK_SERIAL_PAYLOAD_MAX;
  }

  TelemetryBuffer[0] = MSP_ELRS_MAVLINK_TLM;
  const uint32_t copied =
      crsf_serial_read(&TelemetryBuffer[CRSF_FRAME_NOT_COUNTED_BYTES],
                       available);
  if (copied == 0) {
    return false;
  }

  TelemetryBuffer[1] = (uint8_t)copied;
  TelemetrySender.SetDataToTransmit(
      TelemetryBuffer, (uint8_t)(copied + CRSF_FRAME_NOT_COUNTED_BYTES));
  InvalidatePrebuiltTelemetry();

  if (crsf_serial_rx_available() == 0) {
    mavlinkSerialPendingSinceMs = 0;
  } else {
    mavlinkSerialPendingSinceMs = nowMs;
  }

  return true;
}

static void updateSerialRxState() {
#if ELRS_DIAG_DISABLE_CRSF_SERIAL
  serialRxEnabled = false;
  return;
#else
  bool shouldEnable = false;
  const uint8_t protocol = getConfiguredSerialProtocol();

  if (protocol == ELRS_SERIAL_MAVLINK) {
    shouldEnable = (TxOtaProtocol == TX_MAVLINK_MODE || mlrs_ota_is_active()) &&
                   crsf_serial_is_ready();
  }
#if SIW917_ELRS_ENABLE_CRSF_FC_TELEMETRY
  else if (serialProtocolUsesCrsf(protocol)) {
    shouldEnable = TxOtaProtocol == TX_NORMAL_MODE &&
                   connectionState == connected && crsf_serial_is_ready();
  }
#endif

  if (shouldEnable == serialRxEnabled) {
    return;
  }

  if (crsf_serial_set_rx_enabled(shouldEnable) == 0) {
    serialRxEnabled = shouldEnable;
  } else {
    DBGLN("WARNING: serial RX %s failed for protocol %u",
          shouldEnable ? "enable" : "disable", protocol);
  }
#endif
}

#if SIW917_ELRS_ENABLE_CRSF_FC_TELEMETRY
#ifndef SIW917_ELRS_CRSF_SERIAL_DIAG_LOGS
#define SIW917_ELRS_CRSF_SERIAL_DIAG_LOGS 0
#endif

static bool crsfSerialShouldForwardFrame(const uint8_t *frame, uint8_t frameLen) {
  (void)frameLen;
  const uint8_t frameType = frame[CRSF_TELEMETRY_TYPE_INDEX];

  // These are generated by this RX toward the FC. If the UART wiring echoes
  // them, do not send our own control/link frames back to the handset.
  return frameType != CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
         frameType != CRSF_FRAMETYPE_LINK_STATISTICS;
}

static uint16_t readBe16(const uint8_t *data) {
  return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static int32_t readBe32Signed(const uint8_t *data) {
  const uint32_t raw = ((uint32_t)data[0] << 24) |
                       ((uint32_t)data[1] << 16) |
                       ((uint32_t)data[2] << 8) |
                       (uint32_t)data[3];
  return (int32_t)raw;
}

static uint16_t crsfGpsSpeedToCms(uint16_t speedDmh) {
  const uint32_t cms = ((uint32_t)speedDmh * 25U + 4U) / 9U;
  return cms > UINT16_MAX ? UINT16_MAX : (uint16_t)cms;
}

static void updateRemoteIdFromCrsfGpsFrame(const uint8_t *frame,
                                           uint8_t frameLen) {
  if (frame == nullptr || frameLen < (3U + sizeof(crsf_sensor_gps_t) + 1U) ||
      frame[CRSF_TELEMETRY_TYPE_INDEX] != CRSF_FRAMETYPE_GPS) {
    return;
  }

  const uint8_t payloadLen =
      (uint8_t)(frame[CRSF_TELEMETRY_LENGTH_INDEX] - 2U);
  if (payloadLen < sizeof(crsf_sensor_gps_t)) {
    return;
  }

  const uint8_t *gps = &frame[CRSF_TELEMETRY_TYPE_INDEX + 1U];
  const uint8_t satellites = gps[14];
  if (satellites < 3U) {
    return;
  }

  const int32_t latitudeE7 = readBe32Signed(&gps[0]);
  const int32_t longitudeE7 = readBe32Signed(&gps[4]);
  const uint16_t speedDmh = readBe16(&gps[8]);
  uint16_t headingCdeg = readBe16(&gps[10]);
  const uint16_t altitudeRaw = readBe16(&gps[12]);
  const int32_t altitudeM = (int32_t)altitudeRaw - 1000;

  if (headingCdeg > 36000U) {
    headingCdeg = 36100U;
  }

  (void)ble_remote_id_update_from_crsf_gps(
      latitudeE7,
      longitudeE7,
      (int16_t)constrain(altitudeM, -1000, 31767),
      crsfGpsSpeedToCms(speedDmh),
      headingCdeg,
      millis());
}

static void serviceCrsfSerialTelemetry() {
  if (TxOtaProtocol != TX_NORMAL_MODE ||
      connectionState != connected ||
      !configuredSerialProtocolUsesCrsf() ||
      !serialRxEnabled ||
      !crsf_serial_is_ready()) {
    return;
  }

  uint8_t bytes[CRSF_SERIAL_RX_BYTES_PER_LOOP];
  const uint32_t available = crsf_serial_rx_available();
  if (available == 0) {
    return;
  }

  const uint32_t toRead = available > sizeof(bytes) ? sizeof(bytes) : available;
  const uint32_t read = crsf_serial_read(bytes, toRead);
#if SIW917_ELRS_CRSF_SERIAL_DIAG_LOGS
  static uint8_t serialRxByteDiagCount = 0;
  if (serialRxByteDiagCount < 12U) {
    DBGLN("CRSF_RX_BYTES avail=%lu read=%lu first=0x%02X overrun=%lu",
          (unsigned long)available,
          (unsigned long)read,
          read > 0 ? bytes[0] : 0,
          (unsigned long)crsf_serial_get_rx_overrun_count());
    serialRxByteDiagCount++;
  }
#endif

  static uint8_t frame[CRSF_FRAME_SIZE_MAX] = {};
  static uint8_t pos = 0;
  static uint8_t expectedLen = 0;
#if SIW917_ELRS_CRSF_SERIAL_DIAG_LOGS
  static uint8_t serialRxFrameDiagCount = 0;
#endif

  for (uint32_t i = 0; i < read; ++i) {
    const uint8_t byte = bytes[i];

    if (pos == 0) {
      if (byte != CRSF_SYNC_BYTE) {
        continue;
      }
      frame[pos++] = byte;
      continue;
    }

    frame[pos++] = byte;

    if (pos == CRSF_TELEMETRY_TYPE_INDEX) {
      const uint8_t frameSize = frame[CRSF_TELEMETRY_LENGTH_INDEX];
      if (frameSize < 2 ||
          frameSize > (CRSF_FRAME_SIZE_MAX - CRSF_FRAME_NOT_COUNTED_BYTES)) {
        pos = 0;
        expectedLen = 0;
        continue;
      }
      expectedLen = frameSize + CRSF_FRAME_NOT_COUNTED_BYTES;
    }

    if (expectedLen != 0 && pos >= expectedLen) {
      uint8_t frameLen = 0;
      if (validateCrsfFrame(frame, &frameLen)) {
#if SIW917_ELRS_CRSF_SERIAL_DIAG_LOGS
        if (serialRxFrameDiagCount < 12U) {
          DBGLN("CRSF_RX_FRAME type=0x%02X len=%u", frame[CRSF_TELEMETRY_TYPE_INDEX],
                frameLen);
          serialRxFrameDiagCount++;
        }
#endif
        updateRemoteIdFromCrsfGpsFrame(frame, frameLen);
        if (crsfSerialShouldForwardFrame(frame, frameLen)) {
          crsfRouter.processMessage(
              nullptr,
              reinterpret_cast<const crsf_header_t *>(frame));
        }
      }
      pos = 0;
      expectedLen = 0;
    } else if (pos >= sizeof(frame)) {
      pos = 0;
      expectedLen = 0;
    }
  }
}
#else
static void serviceCrsfSerialTelemetry() {}
#endif

static void rxLuaRefreshDynamicValues() {
  elrs_config_t *cfg = elrs_config_get();
  const uint8_t modelId = cfg != nullptr ? cfg->model_id : modelMatchId;

  if (modelId == 0xFF) {
    strncpy(rxLuaModelIdValue, "Off", sizeof(rxLuaModelIdValue));
  } else {
    snprintf(rxLuaModelIdValue, sizeof(rxLuaModelIdValue), "%u", modelId);
  }
  rxLuaModelIdValue[sizeof(rxLuaModelIdValue) - 1] = '\0';

  snprintf(rxLuaTlmRatioValue, sizeof(rxLuaTlmRatioValue), "1:%u",
           ExpressLRS_currTlmDenom);
  rxLuaTlmRatioValue[sizeof(rxLuaTlmRatioValue) - 1] = '\0';

  formatActiveModeString(rxLuaActiveModeValue, sizeof(rxLuaActiveModeValue));
}

static bool rxLuaBuildDeviceInfo(crsf_addr_e destAddr, uint8_t *frame,
                                 uint8_t *frameLen) {
  const uint8_t nameLen = (uint8_t)(strlen(device_name) + 1);
  const uint8_t payloadLen = (uint8_t)(nameLen + sizeof(deviceInformationPacket_t));
  const uint8_t frameSize = CRSF_EXT_FRAME_SIZE(payloadLen);

  if ((frameSize + CRSF_FRAME_NOT_COUNTED_BYTES) > CRSF_FRAME_SIZE_MAX) {
    return false;
  }

  memcpy(frame + sizeof(crsf_ext_header_t), device_name, nameLen);
  auto *device = reinterpret_cast<deviceInformationPacket_t *>(
      frame + sizeof(crsf_ext_header_t) + nameLen);
  device->serialNo = htobe32(0x454C5253); // "ELRS"
  device->hardwareVer = 0;
  device->softwareVer = htobe32(versionStringToU32(version));
  device->fieldCnt = RX_LUA_PARAM_COUNT;
  device->parameterVersion = 3;

  setCrsfExtendedHeaderAndCrc(frame, CRSF_FRAMETYPE_DEVICE_INFO, frameSize,
                              destAddr, CRSF_ADDRESS_CRSF_RECEIVER);
  *frameLen = frameSize + CRSF_FRAME_NOT_COUNTED_BYTES;
  return true;
}

static bool rxLuaBuildParameter(crsf_addr_e destAddr, uint8_t parameterIndex,
                                uint8_t fieldChunk, uint8_t *frame,
                                uint8_t *frameLen) {
  rxLuaRefreshDynamicValues();

  uint8_t chunkBuffer[128] = {};
  uint8_t parent = RX_LUA_PARAM_ROOT;
  uint8_t paramType = 0;
  const char *name = "";
  const char *value = "";
  const char *options = "";
  const char *units = "";
  uint8_t selectionValue = 0;
  uint8_t selectionMin = 0;
  uint8_t selectionMax = 0;
  uint8_t intValue = 0;
  uint8_t intMin = 0;
  uint8_t intMax = 0;
  uint8_t intDefault = 0;
  uint8_t commandStep = RX_LUA_CMD_IDLE;
  const char *commandInfo = "";
  const uint8_t *folderChildren = nullptr;
  bool hidden = false;

  elrs_config_t *cfg = elrs_config_get();

  switch (parameterIndex) {
  case RX_LUA_PARAM_ROOT:
    paramType = CRSF_FOLDER;
    name = device_name;
    {
      static const uint8_t rootChildren[] = {
          RX_LUA_PARAM_PROTOCOL,
          RX_LUA_PARAM_ACTIVE_MODE,
          RX_LUA_PARAM_FAILSAFE,
          RX_LUA_PARAM_TARGET_SYS_ID,
          RX_LUA_PARAM_SOURCE_SYS_ID,
          RX_LUA_PARAM_FORCE_TLM,
          RX_LUA_PARAM_WIFI,
          RX_LUA_PARAM_TEAM_RACE,
          RX_LUA_PARAM_BIND_STORAGE,
          RX_LUA_PARAM_BIND,
          RX_LUA_PARAM_MODEL_ID,
          RX_LUA_PARAM_TLM_POWER,
          RX_LUA_PARAM_BLE_REMOTE_ID,
          RX_LUA_PARAM_TLM_RATIO,
          RX_LUA_PARAM_VERSION,
          0xFF,
      };
      folderChildren = rootChildren;
    }
    break;
  case RX_LUA_PARAM_PROTOCOL:
    paramType = CRSF_TEXT_SELECTION;
    name = "Protocol";
    options = ELRS_SERIAL_PROTOCOL_LUA_OPTIONS;
    selectionValue = getProtocolSelectionFromConfig(cfg);
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_ACTIVE_MODE:
    paramType = CRSF_INFO;
    name = "Active Mode";
    value = rxLuaActiveModeValue;
    break;
  case RX_LUA_PARAM_FAILSAFE:
    paramType = CRSF_TEXT_SELECTION;
    name = "SBUS failsafe";
    options = "No Pulses;Last Pos";
    selectionValue =
        cfg != nullptr && cfg->failsafe_mode <= ELRS_FAILSAFE_LAST
            ? cfg->failsafe_mode
            : (uint8_t)ELRS_FAILSAFE_NO_PULSES;
    selectionMax = selectionOptionMax(options);
    hidden = (cfg == nullptr || cfg->serial_protocol != ELRS_SERIAL_SBUS) &&
             getConfiguredSerialProtocol() != ELRS_SERIAL_SBUS;
    break;
  case RX_LUA_PARAM_TARGET_SYS_ID:
    paramType = CRSF_UINT8;
    name = "Target SysID";
    intValue = clampU8(cfg != nullptr ? cfg->mavlink_target_sys_id : 1, 1, 255);
    intMin = 1;
    intMax = 255;
    intDefault = 1;
    units = "";
    hidden = (cfg == nullptr || cfg->serial_protocol != ELRS_SERIAL_MAVLINK) &&
             getConfiguredSerialProtocol() != ELRS_SERIAL_MAVLINK;
    break;
  case RX_LUA_PARAM_SOURCE_SYS_ID:
    paramType = CRSF_UINT8;
    name = "Source SysID";
    intValue = clampU8(cfg != nullptr ? cfg->mavlink_source_sys_id : 255, 1, 255);
    intMin = 1;
    intMax = 255;
    intDefault = 255;
    units = "";
    hidden = (cfg == nullptr || cfg->serial_protocol != ELRS_SERIAL_MAVLINK) &&
             getConfiguredSerialProtocol() != ELRS_SERIAL_MAVLINK;
    break;
  case RX_LUA_PARAM_FORCE_TLM:
    paramType = CRSF_TEXT_SELECTION;
    name = "Tlm Off";
    options = "Off;On";
    selectionValue = (cfg != nullptr && cfg->force_tlm != 0) ? 1 : 0;
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_WIFI:
    paramType = CRSF_COMMAND;
    name = "WiFi Mode";
    commandStep = rxLuaWifiCommandStep;
    commandInfo = rxLuaWifiCommandInfo;
    break;
  case RX_LUA_PARAM_TEAM_RACE:
    paramType = CRSF_FOLDER;
    name = "Team Race";
    {
      static const uint8_t teamRaceChildren[] = {
          RX_LUA_PARAM_TEAM_RACE_CHANNEL,
          RX_LUA_PARAM_TEAM_RACE_POSITION,
          0xFF,
      };
      folderChildren = teamRaceChildren;
    }
    break;
  case RX_LUA_PARAM_TEAM_RACE_CHANNEL:
    parent = RX_LUA_PARAM_TEAM_RACE;
    paramType = CRSF_TEXT_SELECTION;
    name = "Channel";
    options = "AUX2;AUX3;AUX4;AUX5;AUX6;AUX7;AUX8;AUX9;AUX10;AUX11;AUX12";
    selectionValue = clampU8(cfg != nullptr ? cfg->teamrace_channel : 0, 0, 10);
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_TEAM_RACE_POSITION:
    parent = RX_LUA_PARAM_TEAM_RACE;
    paramType = CRSF_TEXT_SELECTION;
    name = "Position";
    options = "Disabled;1/Low;2;3;Mid;4;5;6/High";
    selectionValue = clampU8(cfg != nullptr ? cfg->teamrace_position : 0, 0, 7);
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_BIND_STORAGE:
    paramType = CRSF_TEXT_SELECTION;
    name = "Bind Storage";
    options = "Persistent;Volatile;Returnable;Administered";
    selectionValue = clampU8(cfg != nullptr ? cfg->bind_storage : 0, 0, 3);
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_BIND:
    paramType = CRSF_COMMAND;
    name = rxLuaBindCommandName();
    commandStep = rxLuaBindCommandStep;
    commandInfo =
        getConfiguredBindStorage() == ELRS_BIND_STORAGE_ADMINISTERED
            ? "Admin only"
            : rxLuaBindCommandInfo;
    break;
  case RX_LUA_PARAM_MODEL_ID:
    paramType = CRSF_TEXT_SELECTION;
    name = "Model Id";
    options = rxLuaModelIdOptions();
    selectionValue =
        rxLuaModelIdToSelection(cfg != nullptr ? cfg->model_id : modelMatchId);
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_TLM_POWER:
    paramType = CRSF_TEXT_SELECTION;
    name = "Tlm Power";
    options = RX_TLM_POWER_OPTIONS;
    units = "mW";
    selectionValue =
        rxTlmPowerDbmToSelection(cfg != nullptr ? cfg->tx_power
                                                : ELRS_TX_POWER_DEFAULT_DBM);
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_BLE_REMOTE_ID:
    paramType = CRSF_TEXT_SELECTION;
    name = "BLE RemoteID";
    options = "Off;On";
    selectionValue = elrs_config_get_ble_remote_id() ? 1 : 0;
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_TLM_RATIO:
    paramType = CRSF_INFO;
    name = "Live Tlm";
    value = rxLuaTlmRatioValue;
    break;
  case RX_LUA_PARAM_VERSION:
    paramType = CRSF_INFO;
    name = version;
    value = commit;
    break;
  default:
    return false;
  }

  chunkBuffer[2] = parent;
  chunkBuffer[3] = hidden ? (paramType | CRSF_FIELD_HIDDEN) : paramType;

  uint8_t *next = appendCString(&chunkBuffer[4], name);
  switch (paramType & CRSF_FIELD_TYPE_MASK) {
  case CRSF_FOLDER:
    if (folderChildren != nullptr) {
      next = appendCString(&chunkBuffer[4], name);
      for (const uint8_t *child = folderChildren; *child != 0xFF; child++) {
        *next++ = *child;
      }
      *next++ = 0xFF;
    }
    break;
  case CRSF_TEXT_SELECTION:
    next = appendCString(next, options);
    *next++ = selectionValue;
    *next++ = selectionMin;
    *next++ = selectionMax;
    *next++ = 0; // default
    next = appendCString(next, units);
    break;
  case CRSF_UINT8:
    *next++ = intValue;
    *next++ = intMin;
    *next++ = intMax;
    *next++ = intDefault;
    next = appendCString(next, units);
    break;
  case CRSF_COMMAND:
    *next++ = commandStep;
    *next++ = 200; // timeout in 10 ms units, matching upstream
    next = appendCString(next, commandInfo);
    break;
  case CRSF_INFO:
  case CRSF_STRING:
    next = appendCString(next, value);
    break;
  default:
    return false;
  }

  const uint8_t dataSize = (uint8_t)(next - &chunkBuffer[2]);
  const uint8_t chunkMax = CRSF_MAX_PACKET_LEN - 6 - 2;
  const uint8_t chunkCnt = (uint8_t)((dataSize + chunkMax - 1) / chunkMax);
  if (fieldChunk >= chunkCnt) {
    return false;
  }

  const uint8_t chunkOffset = (uint8_t)(fieldChunk * chunkMax);
  const uint8_t chunkSize =
      (uint8_t)((dataSize - chunkOffset) > chunkMax ? chunkMax
                                                    : (dataSize - chunkOffset));
  const uint8_t payloadLen = (uint8_t)(chunkSize + 2);
  const uint8_t frameSize = CRSF_EXT_FRAME_SIZE(payloadLen);

  uint8_t *payload = frame + sizeof(crsf_ext_header_t);
  payload[0] = parameterIndex;
  payload[1] = (uint8_t)(chunkCnt - (fieldChunk + 1));
  memcpy(payload + 2, &chunkBuffer[2 + chunkOffset], chunkSize);

  setCrsfExtendedHeaderAndCrc(frame, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY,
                              frameSize, destAddr,
                              CRSF_ADDRESS_CRSF_RECEIVER);
  *frameLen = frameSize + CRSF_FRAME_NOT_COUNTED_BYTES;
  return true;
}

static void rxLuaQueueDeviceInfo(crsf_addr_e destAddr) {
  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {};
  uint8_t frameLen = 0;
  if (rxLuaBuildDeviceInfo(destAddr, frame, &frameLen)) {
    rxLuaQueueFrame(frame, frameLen);
  }
}

static bool rxLuaQueueParameter(crsf_addr_e destAddr, uint8_t parameterIndex,
                                uint8_t fieldChunk) {
  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {};
  uint8_t frameLen = 0;
  if (rxLuaBuildParameter(destAddr, parameterIndex, fieldChunk, frame,
                          &frameLen)) {
    rxLuaQueueFrame(frame, frameLen);
    return true;
  }

  return false;
}

static void rxLuaSaveConfig(bool applySerialAfterSave = false) {
  rxLuaConfigSavePending = true;
  rxLuaSerialApplyPending = rxLuaSerialApplyPending || applySerialAfterSave;
  rxLuaConfigSaveAtMs = millis() + RX_LUA_CONFIG_SAVE_DELAY_MS;
}

[[maybe_unused]] static void rxLuaProcessConfigSave() {
  if (!rxLuaConfigSavePending ||
      (uint32_t)(millis() - rxLuaConfigSaveAtMs) >= 0x80000000UL) {
    return;
  }

  if (rxLuaQueueCount != 0 || TelemetrySender.IsActive()) {
    return;
  }

  rxLuaConfigSavePending = false;
  const bool applySerial = rxLuaSerialApplyPending;
  rxLuaSerialApplyPending = false;

  if (elrs_config_save() != 0) {
    DBGLN("[RX_LUA] Config save failed");
    return;
  }

  if (applySerial) {
    requestSerialProtocolApply();
  }
}

static void rxLuaHandleParameterWrite(crsf_addr_e origin, uint8_t parameterIndex,
                                      uint8_t arg) {
  elrs_config_t *cfg = elrs_config_get();

  switch (parameterIndex) {
  case RX_LUA_PARAM_PROTOCOL:
    if (cfg != nullptr) {
      cfg->serial_protocol = elrs_serial_protocol_from_lua_selection(
          clampU8(arg, 0, ELRS_SERIAL_PROTOCOL_LUA_SELECTION_MAX));
      rxLuaSaveConfig(true);
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_FAILSAFE:
    if (cfg != nullptr) {
      cfg->failsafe_mode =
          arg <= ELRS_FAILSAFE_LAST ? arg : (uint8_t)ELRS_FAILSAFE_NO_PULSES;
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_TARGET_SYS_ID:
    if (cfg != nullptr) {
      cfg->mavlink_target_sys_id = clampU8(arg, 1, 255);
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_SOURCE_SYS_ID:
    if (cfg != nullptr) {
      cfg->mavlink_source_sys_id = clampU8(arg, 1, 255);
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_FORCE_TLM:
    if (cfg != nullptr) {
      cfg->force_tlm = arg ? 1 : 0;
      siw917_rx_set_force_telemetry_off(cfg->force_tlm);
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_WIFI:
    rxLuaHandleCommandWrite(arg, &rxLuaWifiCommandStep, &rxLuaWifiCommandInfo,
                            &rxLuaWifiPending);
    rxLuaQueueParameter(origin, parameterIndex, 0);
    break;
  case RX_LUA_PARAM_TEAM_RACE_CHANNEL:
    if (cfg != nullptr) {
      cfg->teamrace_channel = clampU8(arg, 0, 10);
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_TEAM_RACE_POSITION:
    if (cfg != nullptr) {
      cfg->teamrace_position = clampU8(arg, 0, 7);
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_BIND_STORAGE:
    if (cfg != nullptr) {
      elrs_apply_bind_storage_change(arg);
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
      rxLuaQueueParameter(origin, RX_LUA_PARAM_BIND, 0);
    }
    break;
  case RX_LUA_PARAM_BIND:
    rxLuaHandleBindCommandWrite(arg);
    rxLuaQueueParameter(origin, parameterIndex, 0);
    break;
  case RX_LUA_PARAM_MODEL_ID:
    if (cfg != nullptr) {
      cfg->model_id = rxLuaSelectionToModelId(arg);
      modelMatchId = cfg->model_id;
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_TLM_POWER:
    if (cfg != nullptr) {
      cfg->tx_power = rxTlmPowerSelectionToDbm(arg);
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_BLE_REMOTE_ID:
    if (cfg != nullptr) {
      const bool enabled = arg != 0;
      elrs_config_set_ble_remote_id(enabled);
      (void)ble_remote_id_service_start(enabled);
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  default:
    break;
  }
}

[[maybe_unused]] static bool rxLuaHandleCrsfFrame(const uint8_t *frame) {
  uint8_t frameLen = 0;
  if (!validateCrsfFrame(frame, &frameLen)) {
    return false;
  }

  const crsf_frame_type_e frameType =
      (crsf_frame_type_e)frame[CRSF_TELEMETRY_TYPE_INDEX];
  if (frameType < CRSF_FRAMETYPE_DEVICE_PING) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagIgnore("legacy", frame);
#endif
    return false;
  }

  const auto *ext = reinterpret_cast<const crsf_ext_header_t *>(frame);
  if (ext->dest_addr != CRSF_ADDRESS_CRSF_RECEIVER &&
      ext->dest_addr != CRSF_ADDRESS_BROADCAST) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagIgnore("dest", frame);
#endif
    return false;
  }

  const crsf_addr_e origin = ext->orig_addr;
  const uint8_t *payload = frame + sizeof(crsf_ext_header_t);

  switch (frameType) {
  case CRSF_FRAMETYPE_DEVICE_PING:
    rxLuaCrsfHandledCount++;
#if ELRS_DIAG_RX_LUA_LOAD_SNAPSHOT
    rxLuaArmLoadSnapshot(frameType, 0xFF, 0, false);
#endif
#if ELRS_DIAG_RX_LUA_UL
    DBGLN("[RX_LUA] DEVICE_PING from 0x%02X", origin);
#endif
    rxLuaQueueDeviceInfo(origin);
    return true;
  case CRSF_FRAMETYPE_PARAMETER_READ:
    rxLuaCrsfHandledCount++;
    if (frameLen >= sizeof(crsf_ext_header_t) + 2 + CRSF_FRAME_CRC_SIZE) {
      const uint8_t parameterIndex = payload[0];
      const uint8_t fieldChunk = payload[1];
#if ELRS_DIAG_RX_LUA_LOAD_SNAPSHOT
      rxLuaArmLoadSnapshot(frameType, parameterIndex, fieldChunk,
                           parameterIndex == RX_LUA_PARAM_BIND);
#endif
#if ELRS_DIAG_RX_LUA_UL && ELRS_DIAG_RX_LUA_UL_VERBOSE
      DBGLN("[RX_LUA] PARAM_READ id=%u chunk=%u from=0x%02X",
            parameterIndex, fieldChunk, origin);
#endif
      if ((parameterIndex == RX_LUA_PARAM_WIFI) && fieldChunk == 0) {
        rxLuaWifiCommandStep = RX_LUA_CMD_IDLE;
        rxLuaWifiCommandInfo = "";
      } else if ((parameterIndex == RX_LUA_PARAM_BIND) && fieldChunk == 0) {
        rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
        rxLuaBindCommandInfo = "";
      }
      const bool queued = rxLuaQueueParameter(origin, parameterIndex, fieldChunk);
      if (parameterIndex == RX_LUA_PARAM_BIND) {
#if ELRS_DIAG_RX_LUA_LOAD_SNAPSHOT
        rxLuaArmBindPostSnapshot(fieldChunk);
#endif
        DBGLN("[RX_LUA] PARAM_READ bind chunk=%u step=%u queued=%u q=%u active=%u",
              fieldChunk, rxLuaBindCommandStep, queued ? 1 : 0,
              rxLuaQueueCount, TelemetrySender.IsActive() ? 1 : 0);
      }
    }
    return true;
  case CRSF_FRAMETYPE_PARAMETER_WRITE:
    rxLuaCrsfHandledCount++;
    if (frameLen >= sizeof(crsf_ext_header_t) + 2 + CRSF_FRAME_CRC_SIZE) {
#if ELRS_DIAG_RX_LUA_LOAD_SNAPSHOT
      rxLuaArmLoadSnapshot(frameType, payload[0], payload[1],
                           payload[0] == RX_LUA_PARAM_BIND);
#endif
#if ELRS_DIAG_RX_LUA_UL
      DBGLN("[RX_LUA] PARAM_WRITE id=%u arg=%u", payload[0], payload[1]);
#endif
      rxLuaHandleParameterWrite(origin, payload[0], payload[1]);
    }
    return true;
  default:
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagIgnore("type", frame);
#endif
    return false;
  }
}

[[maybe_unused]] static void rxLuaProcessPendingActions() {
  if (!rxLuaWifiPending && !rxLuaBindPending) {
    return;
  }

  const uint32_t elapsedMs = millis() - rxLuaPendingActionAtMs;
  if (elapsedMs < RX_LUA_PENDING_ACTION_DELAY_MS) {
    return;
  }
  if ((rxLuaQueueCount != 0 || TelemetrySender.IsActive()) && elapsedMs < 5000) {
    return;
  }

  if (rxLuaWifiPending) {
    rxLuaWifiPending = false;
    rxLuaWifiCommandStep = RX_LUA_CMD_IDLE;
    rxLuaWifiCommandInfo = "";
    DBGLN("[RX_LUA] Requesting WiFi mode");
    elrs_cpp_request_wifi_mode();
  }

  if (rxLuaBindPending) {
    rxLuaBindPending = false;
    rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
    rxLuaBindCommandInfo = "";
    DBGLN("[RX_LUA] Entering bind mode");
    elrs_enter_binding_mode();
  }
}

// --- PACKET CAPTURE BUFFER ---
#define PKT_CAPTURE_SIZE 16
volatile uint8_t pkt_capture_buf[PKT_CAPTURE_SIZE][8];
volatile uint8_t pkt_capture_len[PKT_CAPTURE_SIZE];
volatile uint32_t pkt_capture_idx = 0;
volatile uint32_t pkt_capture_count = 0;
volatile bool do_packet_dump = false;

extern "C" void trigger_packet_dump() { do_packet_dump = true; }

extern "C" void dump_capture_buffer() {
  DBGLN("=========================================");
  DBGLN("  PACKET CAPTURE BUFFER DUMP");
  DBGLN("=========================================");
  uint32_t total = pkt_capture_count;
  uint32_t max_print = total > PKT_CAPTURE_SIZE ? PKT_CAPTURE_SIZE : total;
  uint32_t start_idx =
      total > PKT_CAPTURE_SIZE ? (pkt_capture_idx % PKT_CAPTURE_SIZE) : 0;

  DBGLN("Total packets captured: %lu", total);

  for (uint32_t i = 0; i < max_print; i++) {
    uint32_t idx = (start_idx + i) % PKT_CAPTURE_SIZE;
    DBGLN("  [%02lu] RAW: %02X %02X %02X %02X %02X %02X %02X %02X",
          (unsigned long)i, pkt_capture_buf[idx][0], pkt_capture_buf[idx][1],
          pkt_capture_buf[idx][2], pkt_capture_buf[idx][3],
          pkt_capture_buf[idx][4], pkt_capture_buf[idx][5],
          pkt_capture_buf[idx][6], pkt_capture_buf[idx][7]);
  }
  DBGLN("=========================================");
}
// -----------------------------

// Forward declarations
void SetMode(connectionState_e NewMode);
static void SetRFLinkRate(uint8_t index, bool bindMode);

// DMA debug test removed
extern bool SerialrxUpdatePacketComplete; // This seems to be a new declaration

// Hardware interrupt flags from LR1121_hal.cpp
extern volatile uint32_t isr_1_total_count;

static void ICACHE_RAM_ATTR getRFlinkInfo();
static void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR HWtimerCallbackTick();
static void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR HWtimerCallbackTock();
static void ICACHE_RAM_ATTR HandleFHSS();
static void ICACHE_RAM_ATTR updatePhaseLock();
static void waitForRecentTockBeforeTimerStop();
static void armTelemetry150Snapshot(const expresslrs_mod_settings_s *params,
                                    bool bindMode);
static void maybePrintTelemetry150Snapshot(unsigned long now);
static void ICACHE_RAM_ATTR TentativeConnection(unsigned long now);
static void GotConnection(unsigned long now);
static void LostConnection(bool resumeRx);
#if ELRS_DIAG_LOSS_PACKET_STATS
static void printLossPacketStats();
#endif
static uint8_t minLqForChaos();
static void enterBindingModeNow();
static void updateBindingMode(unsigned long now);
static void LinkStatsToOta(OTA_LinkStats_s *ls);
static bool SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR HandleSendDataDl();
static void ICACHE_RAM_ATTR InvalidatePrebuiltTelemetry();
static void ICACHE_RAM_ATTR PrepareTelemetryForNextTock();
static void updateTelemetryBurst();
static void ICACHE_RAM_ATTR updateSwitchModePendingFromOta(uint8_t newSwitchMode);
static void updateSwitchMode();
static void ICACHE_RAM_ATTR OnELRSBindMSP(uint8_t *newUid4);
static void ICACHE_RAM_ATTR ProcessRfPacket_DataUl(
    OTA_Packet_s const *const otaPktPtr);
static void DataUlReceiveComplete();
static void maybeReportStaleTelemetryRx();
static void maybePrintLuaProgress(unsigned long now);
static void maybePrintLinkProgress(unsigned long now);

//=============================================================================
// Channel data initialization
//=============================================================================
void ChannelDataReset() {
  for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
    ChannelData[i] = CRSF_CHANNEL_VALUE_MID;
  }
}

static uint8_t getConfiguredFailsafeMode() {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr || cfg->failsafe_mode > ELRS_FAILSAFE_SET) {
    return ELRS_FAILSAFE_NO_PULSES;
  }
  return cfg->failsafe_mode;
}

static uint8_t getConfiguredBindStorage() {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr || cfg->bind_storage > ELRS_BIND_STORAGE_ADMINISTERED) {
    return ELRS_BIND_STORAGE_PERSISTENT;
  }
  return cfg->bind_storage;
}

extern "C" bool elrs_is_on_loan(void) {
  elrs_config_t *cfg = elrs_config_get();
  return cfg != nullptr && cfg->bind_storage == ELRS_BIND_STORAGE_RETURNABLE &&
         firmwareOptions.hasUID && elrs_config_is_bound() &&
         memcmp(cfg->uid, firmwareOptions.uid, UID_LEN) != 0;
}

static bool returnLoanIfNeeded(bool refreshRuntime = true) {
  if (!elrs_is_on_loan()) {
    return false;
  }

  uint8_t returnUid[UID_LEN] = {};
  if (firmwareOptions.hasUID) {
    memcpy(returnUid, firmwareOptions.uid, UID_LEN);
  }

  (void)elrs_config_set_uid(returnUid);
  memcpy(UID, returnUid, UID_LEN);
  if (refreshRuntime) {
    uidRefreshPending = true;
  }

  if (firmwareOptions.hasUID) {
    DBGLN("Returnable bind returned to flashed UID");
  } else {
    DBGLN("Returnable bind returned to unbound state");
  }
  return true;
}

extern "C" void elrs_apply_bind_storage_change(uint8_t bindStorage) {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    return;
  }

  const uint8_t newBindStorage =
      clampU8(bindStorage, ELRS_BIND_STORAGE_PERSISTENT,
              ELRS_BIND_STORAGE_ADMINISTERED);
  if (cfg->bind_storage == newBindStorage) {
    return;
  }

  // Upstream SetBindStorage() calls ReturnLoan() before changing the mode.
  (void)returnLoanIfNeeded(true);
  cfg->bind_storage = newBindStorage;
}

static uint8_t getConfiguredTeamracePosition() {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr || cfg->teamrace_position > 7) {
    return 0;
  }
  return cfg->teamrace_position;
}

static uint8_t getConfiguredTeamraceChannel() {
  elrs_config_t *cfg = elrs_config_get();
  const uint8_t selection =
      (cfg != nullptr && cfg->teamrace_channel <= 10) ? cfg->teamrace_channel
                                                      : 0;
  return (uint8_t)(AUX2 + selection);
}

static bool teamraceIsEnabled() { return getConfiguredTeamracePosition() != 0; }

static uint8_t teamraceChannelToConfigValue() {
  const uint8_t channel = getConfiguredTeamraceChannel();
  if (channel >= CRSF_NUM_CHANNELS) {
    return 0;
  }

  const uint8_t switchPosition =
      CRSF_to_SWITCH3b((uint16_t)ChannelData[channel]);
  switch (switchPosition) {
  case 0:
  case 1:
  case 2:
    return (uint8_t)(switchPosition + 1);
  case 3:
  case 4:
  case 5:
    return (uint8_t)(switchPosition + 2);
  case 7:
    return 4;
  default:
    return 0;
  }
}

static void resetTeamraceModelMatch() {
  teamraceOutputInhibitState = troiPass;
  lastTeamracePosition = 0;
  teamraceHasModelMatch = true;
}

static bool updateTeamraceModelMatch() {
  const uint8_t configuredPosition = getConfiguredTeamracePosition();
  if (configuredPosition == 0) {
    resetTeamraceModelMatch();
    return connectionHasModelMatch;
  }

  if (!connectionHasModelMatch) {
    teamraceHasModelMatch = false;
    return false;
  }

  const bool shouldForwardChannels = teamraceOutputInhibitState < troiInhibit;
  const uint8_t newTeamracePosition = teamraceChannelToConfigValue();

  switch (teamraceOutputInhibitState) {
  case troiPass:
    if (newTeamracePosition != configuredPosition) {
      teamraceOutputInhibitState = troiDisableAwaitConfirm;
    }
    break;

  case troiDisableAwaitConfirm:
    if (newTeamracePosition == lastTeamracePosition) {
      teamraceOutputInhibitState =
          (newTeamracePosition != configuredPosition) ? troiInhibit : troiPass;
    }
    break;

  case troiInhibit:
    if (newTeamracePosition == configuredPosition) {
      teamraceOutputInhibitState = troiEnableAwaitConfirm;
    }
    break;

  case troiEnableAwaitConfirm:
    if (newTeamracePosition == lastTeamracePosition) {
      teamraceOutputInhibitState =
          (newTeamracePosition == configuredPosition) ? troiPass : troiInhibit;
    }
    break;
  }

  lastTeamracePosition = newTeamracePosition;
  teamraceHasModelMatch = teamraceOutputInhibitState == troiPass;
  return shouldForwardChannels;
}

static bool shouldOutputSerialRcFrames() {
  if (mlrs_ota_is_active()) {
    return false;
  }
  if (InBindingMode || InWiFiMode || !crsf_serial_is_ready() ||
      !configuredSerialProtocolSendsRc() ||
      (TxOtaProtocol != TX_NORMAL_MODE)) {
    return false;
  }

  if (connectionState == connected) {
    return connectionHasModelMatch &&
           (!teamraceIsEnabled() || teamraceHasModelMatch);
  }

  const uint8_t failsafeMode = getConfiguredFailsafeMode();
  if (failsafeMode == ELRS_FAILSAFE_LAST) {
    return lastConnectionHadModelMatch;
  }

  if (failsafeMode == ELRS_FAILSAFE_SET) {
    ChannelDataReset();
    return lastConnectionHadModelMatch;
  }

  return false;
}

//=============================================================================
// UID/MAC seed helpers
//=============================================================================
uint32_t uidMacSeedGet() {
  return ((uint32_t)UID[2] << 24) | ((uint32_t)UID[3] << 16) |
         ((uint32_t)UID[4] << 8) | (UID[5] ^ OTA_VERSION_ID);
}

static bool uidIsBound(const uint8_t uid[UID_LEN]) {
  for (unsigned i = 0; i < UID_LEN; ++i) {
    if (uid[i] != 0) {
      return true;
    }
  }
  return false;
}

static bool use2G4Domain(void) { return firmwareOptions.domain >= 8; }

static uint8_t getStartupAcquisitionRateIndex(uint8_t rateIndex) {
  if (rateIndex >= RATE_MAX || !isSupportedRFRate(rateIndex)) {
    return rateIndex;
  }

  const expresslrs_mod_settings_s *const params =
      get_elrs_airRateConfig(rateIndex);
  if (params != nullptr && params->enum_rate == RATE_LORA_900_200HZ) {
    const uint8_t scanIndex = enumRatetoIndex(RATE_LORA_900_50HZ_DVDA);
    if (scanIndex < RATE_MAX && isSupportedRFRate(scanIndex)) {
      return scanIndex;
    }
  }

  return rateIndex;
}

static uint8_t getStartupOrBindingRateIndex(void) {
  if (!InBindingMode) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr && cfg->rate_index < RATE_MAX &&
        isSupportedRFRate(cfg->rate_index)) {
      return getStartupAcquisitionRateIndex(cfg->rate_index);
    }
  }

  return enumRatetoIndex(use2G4Domain() ? RATE_LORA_2G4_50HZ : RATE_BINDING);
}

static void scheduleStartupRateSave(uint32_t now) {
#if !SIW917_ELRS_PERSIST_STARTUP_RATE
  (void)now;
  return;
#else
  if (InBindingMode || ExpressLRS_currAirRate_Modparams == nullptr) {
    return;
  }

  const uint8_t rateIndex = ExpressLRS_currAirRate_Modparams->index;
  if (rateIndex >= RATE_MAX || !isSupportedRFRate(rateIndex)) {
    return;
  }

  const uint8_t startupIndex = getStartupAcquisitionRateIndex(rateIndex);
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr || cfg->rate_index == startupIndex) {
    return;
  }

  cfg->rate_index = startupIndex;
  startupRateSaveIndex = startupIndex;
  startupRateSaveAtMs = now + STARTUP_RATE_SAVE_DELAY_MS;
  startupRateSavePending = true;
#endif
}

static void processStartupRateSave(uint32_t now) {
  if (!startupRateSavePending ||
      (uint32_t)(now - startupRateSaveAtMs) >= 0x80000000UL) {
    return;
  }

  if (connectionState != connected || RXtimerState != tim_locked ||
      TelemetrySender.IsActive() || !otaConnector.IsEmpty()) {
    return;
  }

  startupRateSavePending = false;
  const int saveResult = elrs_config_save();

  if (saveResult == 0) {
    DBGLN("Startup RF rate saved: index=%u", startupRateSaveIndex);
  } else {
    DBGLN("WARNING: startup RF rate save failed: %d", saveResult);
  }
}

static uint8_t getFirstSupportedRFRateIndex(void) {
  for (uint8_t i = 0; i < RATE_MAX; i++) {
    if (isSupportedRFRate(i)) {
      return i;
    }
  }

  return enumRatetoIndex(use2G4Domain() ? RATE_LORA_2G4_50HZ : RATE_BINDING);
}

static uint8_t getNextSupportedRFRateIndex(uint8_t index) {
  for (uint8_t i = 1; i <= RATE_MAX; i++) {
    uint8_t candidate = (uint8_t)((index + i) % RATE_MAX);
    if (isSupportedRFRate(candidate)) {
      return candidate;
    }
  }

  return getFirstSupportedRFRateIndex();
}

//=============================================================================
// Chaos detection (noise floor)
//=============================================================================

//=============================================================================
// Link statistics collection
//=============================================================================
static void ICACHE_RAM_ATTR getRFlinkInfo() {
  int32_t rssiDBM = Radio.LastPacketRSSI;

  // Single radio - no dual antenna handling
  rssiDBM = LPF_UplinkRSSI0.update(rssiDBM);
  if (rssiDBM > 0)
    rssiDBM = 0;

  // BetaFlight/iNav expect positive values for -dBm
  linkStats.uplink_RSSI_1 = -rssiDBM;
  linkStats.uplink_RSSI_2 = -rssiDBM; // Same for single radio

  // Track raw SNR for telemetry
  lastSnrRaw = Radio.LastPacketSNRRaw;
  SnrMean.add(Radio.LastPacketSNRRaw);

  linkStats.active_antenna = antenna;
  linkStats.uplink_SNR = Radio.LastPacketSNRRaw / RADIO_SNR_SCALE;
  linkStats.rf_Mode = ExpressLRS_currAirRate_Modparams->enum_rate;
}

static uint8_t minLqForChaos() {
  const uint32_t numfhss = FHSSgetChannelCount();
  const uint8_t interval = ExpressLRS_currAirRate_Modparams->FHSShopInterval;
  return interval * ((interval * numfhss + 99) / (interval * numfhss));
}

#if ELRS_DIAG_DYNPOWER_STATS
static void printDynpowerStatsDiag(int8_t snrMean) {
  static uint32_t lastPrintMs = 0;
  if (connectionState != connected || RXtimerState != tim_locked) {
    return;
  }

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastPrintMs) < 2000U) {
    return;
  }
  lastPrintMs = nowMs;

  const expresslrs_mod_settings_s *mod = ExpressLRS_currAirRate_Modparams;
  const expresslrs_rf_pref_params_s *perf = ExpressLRS_currAirRate_RFperfParams;
  const int8_t upThresh =
      perf ? perf->DynpowerSnrThreshUp : DYNPOWER_SNR_THRESH_NONE;
  const int8_t downThresh =
      perf ? perf->DynpowerSnrThreshDn : DYNPOWER_SNR_THRESH_NONE;
  const bool usesSnr = (upThresh != DYNPOWER_SNR_THRESH_NONE) &&
                       (downThresh != DYNPOWER_SNR_THRESH_NONE);

  DBGLN("DYNPOWER metric=%s rate=%u enum=%u radio=%u lq=%u rssi=-%u "
        "snrRaw=%d snrDb=%d lastRaw=%d up=%d dn=%d den=%u",
        usesSnr ? "SNR" : "RSSI", mod ? mod->index : 0xFFU,
        mod ? (uint8_t)mod->enum_rate : 0xFFU, mod ? mod->radio_type : 0xFFU,
        uplinkLQ, linkStats.uplink_RSSI_1, snrMean, SNR_DESCALE(snrMean),
        lastSnrRaw, upThresh, downThresh, ExpressLRS_currTlmDenom);
}
#endif

static void LinkStatsToOta(OTA_LinkStats_s *ls) {
  if (ls == nullptr) {
    return;
  }

  ls->uplink_RSSI_1 = linkStats.uplink_RSSI_1;
  ls->uplink_RSSI_2 = linkStats.uplink_RSSI_2;
  ls->antenna = antenna;
  ls->modelMatch = connectionHasModelMatch;
  ls->lq = linkStats.uplink_Link_quality;
  ls->trueDiversityAvailable = isDualRadio() ? 1 : 0;
  if (SnrMean.getCount()) {
    ls->SNR = SnrMean.mean();
  } else {
    ls->SNR = SnrMean.previousMean();
  }

#if ELRS_DIAG_DYNPOWER_STATS
  printDynpowerStatsDiag(ls->SNR);
#endif
}

static void ICACHE_RAM_ATTR OnELRSBindMSP(uint8_t *newUid4) {
  const uint8_t bindStorage = getConfiguredBindStorage();
  if (bindStorage == ELRS_BIND_STORAGE_ADMINISTERED) {
    DBGLN("MSP bind ignored - Bind Storage is Administered");
    return;
  }

  UID[0] = 0;
  UID[1] = 0;
  for (unsigned i = 0; i < 4; i++) {
    UID[i + 2] = newUid4[i];
  }

  if (bindStorage == ELRS_BIND_STORAGE_VOLATILE) {
    uidSavePending = false;
    uidRefreshPending = true;
    DBGLN("Volatile bind UID applied for this boot");
  } else {
    memcpy(pendingUid, UID, UID_LEN);
    uidSavePending = true;
    if (bindStorage == ELRS_BIND_STORAGE_RETURNABLE &&
        firmwareOptions.hasUID &&
        memcmp(UID, firmwareOptions.uid, UID_LEN) != 0) {
      DBGLN("Returnable bind loan UID queued");
    }
  }
  bindCompletePending = true;

  DBGLN("New UID from MSP bind = %u,%u,%u,%u,%u,%u", UID[0], UID[1], UID[2],
        UID[3], UID[4], UID[5]);
}

static void ICACHE_RAM_ATTR updatePhaseLock() {
  if (connectionState != disconnected && PFDloop.hasResult()) {
    int32_t rawOffset = PFDloop.calcResult();
    int32_t offset = LPF_Offset.update(rawOffset);
    int32_t offsetDx =
        LPF_OffsetDx.update(rawOffset - PfdPrevRawOffset);
    PfdPrevRawOffset = rawOffset;
    pfdLastRawOffset = rawOffset;
    pfdLastNormalizedOffset = rawOffset;
    pfdLastOffset = offset;
    pfdLastOffsetDx = offsetDx;
    pfdResultCount++;

    if (RXtimerState == tim_locked) {
      if ((OtaNonce % 8) == 0) {
        if (offset > 0) {
          hwTimer::incFreqOffset();
        } else if (offset < 0) {
          hwTimer::decFreqOffset();
        }
      }
    }

    int32_t phaseShift =
        connectionState != connected ? (rawOffset >> 1) : (offset >> 2);
    pfdLastPhaseShift = phaseShift;
    hwTimer::phaseShift(phaseShift);

    (void)offsetDx;
  }

  PFDloop.reset();
}

static void ICACHE_RAM_ATTR HandleFHSS() {
  if (ExpressLRS_currAirRate_Modparams == nullptr) {
    return;
  }

  uint8_t modresultFHSS =
      OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

  if ((ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) ||
      InBindingMode || (modresultFHSS != 0) ||
      (connectionState == disconnected)) {
    return;
  }

  if (isDualRadio() && geminiMode) {
    if (FHSSuseDualBand) {
      Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_1,
                            SIW917_ELRS_FHSS_SET_FREQ_RX);
      Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_2,
                            SIW917_ELRS_FHSS_SET_FREQ_RX);
    } else if (((OtaNonce /
                 ExpressLRS_currAirRate_Modparams->FHSShopInterval) %
                2U) == 0U) {
      Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_1,
                            SIW917_ELRS_FHSS_SET_FREQ_RX);
      Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_2,
                            SIW917_ELRS_FHSS_SET_FREQ_RX);
    } else {
      // Match upstream Gemini: alternate which physical radio tracks the
      // offset frequency so both antennas see both hop positions over time.
      const uint32_t freqRadio2 = FHSSgetNextFreq();
      Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_1,
                            SIW917_ELRS_FHSS_SET_FREQ_RX);
      Radio.SetFrequencyReg(freqRadio2, SX12XX_Radio_2,
                            SIW917_ELRS_FHSS_SET_FREQ_RX);
    }
  } else {
    Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_All,
                          SIW917_ELRS_FHSS_SET_FREQ_RX);
  }
}

static void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR HWtimerCallbackTick() {
#if ELRS_DIAG_TX_TURNAROUND
  if (telemetryAwaitingRx && !LQCalc.currentIsSet()) {
    telemetryMissedAfterTx++;
  }
#endif

  uplinkLQ = LQCalc.getLQ();
  linkStats.uplink_Link_quality = uplinkLQ;
  if (!alreadyTLMresp) {
    LQCalc.inc();
  }
  alreadyTLMresp = false;
}

static void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR HWtimerCallbackTock() {
  PFDloop.intEvent(hwTimer::eventMicros());
  OtaNonce++;
  HandleFHSS();
  if (HandleSendDataDl()) {
#if !SIW917_ELRS_DEFER_TLM_TX_FROM_TIMER
    telemetryTxCount++;
#endif
  }
  updatePhaseLock();
}

static void waitForRecentTockBeforeTimerStop() {
  const uint32_t interval =
      ExpressLRS_currAirRate_Modparams ? ExpressLRS_currAirRate_Modparams->interval
                                       : 20000U;
  const uint32_t start = micros();
  const uint32_t deadline = start + interval + 1000U;

  while ((uint32_t)(micros() - PFDloop.getIntEventTime()) > 250U) {
    hwTimer::service();
    if ((int32_t)(deadline - micros()) <= 0) {
      break;
    }
  }
}

static void ICACHE_RAM_ATTR TentativeConnection(unsigned long now) {
  PFDloop.reset();
  setConnectionState(tentative);
  connectionHasModelMatch = false;
  resetTeamraceModelMatch();
  lastDisconnectReason = DISC_NONE;
  RXtimerState = tim_disconnected;
  PfdPrevRawOffset = 0;
  GotConnectionMillis = 0;
  LPF_Offset.init(0);
  LPF_OffsetDx.init(0);
  SnrMean.reset();
  alreadyTLMresp = false;
  lastDecodedUplinkTxPower = 0;
  lastDecodedUplinkTxPowerSource = 0;
  lastDecodedUplinkTxPowerSwitchMode = 0xFF;
  lastDecodedUplinkTxPowerRate = 0xFF;
  uplinkTxPowerChangePending = false;
  RFmodeLastCycled = now;
}

static void GotConnection(unsigned long now) {
  if (connectionState == connected) {
    return;
  }

  LockRFmode = firmwareOptions.lock_on_first_connection;
  setConnectionState(connected);
  RXtimerState = tim_tentative;
  GotConnectionMillis = now;
  scheduleStartupRateSave(now);
}

static void LostConnection(bool resumeRx) {
  if (lastDisconnectReason == DISC_NONE) {
    lastDisconnectReason = DISC_EXTERNAL;
  }

  DBGLN("LostConnection reason=%u/%s age=%lu pfd raw=%ld norm=%ld off=%ld dx=%ld phase=%ld "
        "fo=%ld nonce=%u fhss=%u dio:%lu/%lu/%lu",
        (unsigned)lastDisconnectReason,
        disconnectReasonName(lastDisconnectReason),
        (unsigned long)(millis() - LastValidPacket),
        (long)pfdLastRawOffset, (long)pfdLastNormalizedOffset,
        (long)pfdLastOffset, (long)pfdLastOffsetDx, (long)pfdLastPhaseShift,
        (long)hwTimer::FreqOffset, OtaNonce, FHSSgetCurrIndex(),
        (unsigned long)lr1121_hal_get_direct_dio_count(),
        (unsigned long)lr1121_hal_get_direct_reentrant_count(),
        (unsigned long)lr1121_hal_get_level_requeue_count());

#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  if (lastDisconnectReason != DISC_RATE_CHANGE || ELRS_DIAG_RATE_CHANGE_LOG) {
    DBGLN("HOTPATH tick:%lu tock:%lu dio:%lu/%lu gspi:%lu/%lu/%lu "
          "busy:%lu/%lu tlm:%lu/%lu/%lu",
          (unsigned long)hwTimer::getMaxTickDurationUs(),
          (unsigned long)hwTimer::getMaxTockDurationUs(),
          (unsigned long)lr1121_hal_get_stage_max_us(),
          (unsigned long)lr1121_hal_get_deferred_max_us(),
          (unsigned long)lr1121_get_raw_gspi_max_us(),
          (unsigned long)lr1121_get_raw_gspi_count(),
          (unsigned long)lr1121_get_raw_gspi_fail_count(),
          (unsigned long)lr1121_get_busy_fast_max_iterations(),
          (unsigned long)lr1121_get_busy_fast_fail_count(),
          (unsigned long)telemetryBuildMaxUs,
          (unsigned long)telemetrySendMaxUs,
          (unsigned long)telemetryHandleMaxUs);
  }
#endif

#if ELRS_DIAG_LOSS_PACKET_STATS
  if (lastDisconnectReason != DISC_RATE_CHANGE) {
    printLossPacketStats();
  }
#endif

#if ELRS_DIAG_PRINT_AFTER_LOSS
  if (!InBindingMode && !InWiFiMode && lastDisconnectReason != DISC_RATE_CHANGE) {
    diagPrintAfterLossPending = true;
  }
#endif

  setConnectionState(disconnected);
  RXtimerState = tim_disconnected;
  PfdPrevRawOffset = 0;
  GotConnectionMillis = 0;
  uplinkLQ = 0;
  lastConnectionHadModelMatch = connectionHasModelMatch && teamraceHasModelMatch;
  if (getConfiguredFailsafeMode() == ELRS_FAILSAFE_SET) {
    ChannelDataReset();
  }
  connectionHasModelMatch = false;
  resetTeamraceModelMatch();
  LQCalc.reset();
  LPF_Offset.init(0);
  LPF_OffsetDx.init(0);
  alreadyTLMresp = false;
  SwitchModePending = 0;
  dataUlReady = false;
  DataUlReceiver.ResetState();
  InvalidatePrebuiltTelemetry();

  hwTimer::resetFreqOffset();

  if (!InBindingMode) {
    if (hwTimer::isRunning()) {
      if (lastDisconnectReason == DISC_RATE_CHANGE) {
        waitForRecentTockBeforeTimerStop();
      }
      hwTimer::stop();
    }
    SetRFLinkRate(ExpressLRS_nextAirRateIndex, false);
    if (resumeRx) {
      Radio.RXnb();
    }
  } else if (resumeRx) {
    SetRFLinkRate(getStartupOrBindingRateIndex(), true);
    Radio.RXnb();
  }
}

extern "C" int elrs_config_save_with_rf_rearm(void) {
  if (InBindingMode || connectionState >= NO_CONFIG_SAVE_STATES) {
    return 1;
  }

  lastDisconnectReason = DISC_EXTERNAL;
  LostConnection(false);
  const int saveResult = elrs_config_save();
  Radio.RXnb();
  return saveResult;
}

//=============================================================================
// Connection state management
//=============================================================================

//=============================================================================
// Process SYNC packet
//=============================================================================
static void ICACHE_RAM_ATTR updateSwitchModePendingFromOta(uint8_t newSwitchMode) {
  if (OtaSwitchModeCurrent == newSwitchMode) {
    SwitchModePending = 0;
    return;
  }

  int8_t newSwitchModePending = -(int8_t)newSwitchMode - 1;
  if ((connectionState == disconnected) ||
      (SwitchModePending == newSwitchModePending)) {
    SwitchModePending = (int8_t)newSwitchMode + 1;
  } else {
    SwitchModePending = newSwitchModePending;
  }
}

static uint8_t ICACHE_RAM_ATTR
ClampTelemetryDenomForSiw917(uint8_t denom, uint8_t rateIndex) {
  if (!isDualRadio() || !geminiMode ||
      (denom >= SIW917_ELRS_MIN_TLM_DENOM_200HZ_FULL_GEMINI)) {
    return denom;
  }

  const expresslrs_mod_settings_s *const params =
      get_elrs_airRateConfig(rateIndex);
  if (params->enum_rate != RATE_LORA_900_200HZ_8CH) {
    return denom;
  }

  return SIW917_ELRS_MIN_TLM_DENOM_200HZ_FULL_GEMINI;
}

static bool ICACHE_RAM_ATTR
ProcessRfPacket_SYNC(uint32_t const now, OTA_Sync_s const *const otaSync) {
  // Verify the binding ID
  if (otaSync->UID4 != UID[4])
    return false;

  if ((otaSync->UID5 & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK))
    return false;

  LastSyncPacket = now;

  TxOtaProtocol = (otaSync->otaProtocol == TX_MAVLINK_MODE)
                      ? TX_MAVLINK_MODE
                      : TX_NORMAL_MODE;
  bool otaProtocolSelectionChanged = false;
  if (updateActiveSerialProtocol()) {
    requestSerialProtocolApplyAfter(now, SERIAL_PROTOCOL_SYNC_APPLY_DELAY_MS);
    otaProtocolSelectionChanged = true;
    DBGLN("TX OTA protocol selected active serial protocol %u",
          activeSerialProtocol);
  } else if ((TxOtaProtocol == TX_MAVLINK_MODE) &&
             (activeSerialProtocol == ELRS_SERIAL_MAVLINK) &&
             (appliedSerialProtocol != ELRS_SERIAL_MAVLINK)) {
    requestSerialProtocolApplyAfter(now, SERIAL_PROTOCOL_SYNC_APPLY_DELAY_MS);
  }

  if ((TxOtaProtocol == TX_MAVLINK_MODE) && !warnedUnsupportedTxProtocol) {
    warnedUnsupportedTxProtocol = true;
    DBGLN("TX requested MAVLink OTA mode");
  } else if ((TxOtaProtocol == TX_NORMAL_MODE) && warnedUnsupportedTxProtocol) {
    warnedUnsupportedTxProtocol = false;
    DBGLN("TX returned to normal OTA mode");
  }

  // A single-LR1121 RX must not mirror the TX's Gemini request locally. The
  // link-stat trueDiversityAvailable bit tells a Gemini TX to step down.
  const uint8_t nextGeminiMode = isDualRadio() ? otaSync->geminiMode : 0;
  if (nextGeminiMode != geminiMode) {
    DBGLN("Gemini mode changed: %u -> %u", (unsigned)geminiMode,
          (unsigned)nextGeminiMode);
  }
  geminiMode = nextGeminiMode;

  // Will change the packet air rate in loop() if this changes
  ExpressLRS_nextAirRateIndex =
      enumRatetoIndex((expresslrs_RFrates_e)otaSync->rfRateEnum);
  updateSwitchModePendingFromOta(otaSync->switchEncMode);

  // Update TLM ratio
  expresslrs_tlm_ratio_e TLMrateIn =
      (expresslrs_tlm_ratio_e)(otaSync->newTlmRatio +
                               (uint8_t)TLM_RATIO_NO_TLM);
  const uint8_t requestedTlmDenom = TLMratioEnumToValue(TLMrateIn);
  uint8_t TlmDenom =
      ClampTelemetryDenomForSiw917(requestedTlmDenom,
                                   ExpressLRS_nextAirRateIndex);
  if (ExpressLRS_currTlmDenom != TlmDenom) {
#if ELRS_DIAG_TLM_RATE_LOG
    const uint8_t previousTlmDenom = ExpressLRS_currTlmDenom;
    const uint8_t syncFlags = ((const uint8_t *)otaSync)[3];
    DBGLN("New TLMrate 1:%u (sync=%u rate=%u)", TlmDenom,
          otaSync->newTlmRatio, ExpressLRS_nextAirRateIndex);
    if (TlmDenom == 2 || previousTlmDenom == 2) {
      static uint32_t prevBoostTraceDataPass = 0;
      static uint32_t prevBoostTraceDataFail = 0;
      static uint32_t prevBoostTraceRcPass = 0;
      static uint32_t prevBoostTraceSyncPass = 0;
      static uint32_t prevBoostTraceLqSkip = 0;
      static uint32_t prevBoostTraceUlChunk = 0;
      static uint32_t prevBoostTraceUlComplete = 0;
      static uint32_t prevBoostTraceUlHandled = 0;
      static uint32_t prevBoostTraceUlMalformed = 0;
      const uint32_t nowMs = millis();
      const uint32_t chunkAge =
          dataUlLastChunkMs != 0 ? nowMs - dataUlLastChunkMs : 0xFFFFFFFFU;
      const uint32_t completeAge =
          dataUlLastCompleteMs != 0 ? nowMs - dataUlLastCompleteMs
                                    : 0xFFFFFFFFU;
      const uint32_t dataPass = crcPassTypeCount[PACKET_TYPE_DATA];
      const uint32_t dataFail = crcFailTypeCount[PACKET_TYPE_DATA];
      const uint32_t rcPass = crcPassTypeCount[PACKET_TYPE_RCDATA];
      const uint32_t syncPass = crcPassTypeCount[PACKET_TYPE_SYNC];
      const uint32_t lqSkip = rxLqCurrentSetSkipCount;
      const uint32_t ulChunk = dataUlChunkCount;
      const uint32_t ulComplete = dataUlCompleteCount;
      const uint32_t ulHandled = dataUlHandledCount;
      const uint32_t ulMalformed = dataUlMalformedCount;
      DBGLN("TLM_BOOST_TRACE prev:%u next:%u raw:0x%02X ul:%lu/%lu/%lu/%lu "
            "last:%u/%02X/%02X frame:%02X/%u age:%lu/%lu q:%u tx:%lu active:%u st:%u wait:%u/%u",
            previousTlmDenom, TlmDenom, syncFlags,
            (unsigned long)dataUlChunkCount,
            (unsigned long)dataUlCompleteCount,
            (unsigned long)dataUlHandledCount,
            (unsigned long)dataUlMalformedCount, dataUlLastPackageIndex,
            dataUlLastPayload0, dataUlLastPayload1, dataUlLastFrameType,
            dataUlLastFrameLen, (unsigned long)chunkAge,
            (unsigned long)completeAge, rxLuaQueueCount,
            (unsigned long)telemetryTxCount,
            TelemetrySender.IsActive() ? 1 : 0,
            (unsigned)TelemetrySender.GetState(),
            TelemetrySender.GetWaitCount(),
            TelemetrySender.GetMaxPacketsBeforeResync());
      DBGLN("TLM_BOOST_DELTA data:%lu/%lu rc:%lu sync:%lu lqskip:%lu "
            "ul:%lu/%lu/%lu/%lu",
            (unsigned long)(dataPass - prevBoostTraceDataPass),
            (unsigned long)(dataFail - prevBoostTraceDataFail),
            (unsigned long)(rcPass - prevBoostTraceRcPass),
            (unsigned long)(syncPass - prevBoostTraceSyncPass),
            (unsigned long)(lqSkip - prevBoostTraceLqSkip),
            (unsigned long)(ulChunk - prevBoostTraceUlChunk),
            (unsigned long)(ulComplete - prevBoostTraceUlComplete),
            (unsigned long)(ulHandled - prevBoostTraceUlHandled),
            (unsigned long)(ulMalformed - prevBoostTraceUlMalformed));
      prevBoostTraceDataPass = dataPass;
      prevBoostTraceDataFail = dataFail;
      prevBoostTraceRcPass = rcPass;
      prevBoostTraceSyncPass = syncPass;
      prevBoostTraceLqSkip = lqSkip;
      prevBoostTraceUlChunk = ulChunk;
      prevBoostTraceUlComplete = ulComplete;
      prevBoostTraceUlHandled = ulHandled;
      prevBoostTraceUlMalformed = ulMalformed;
    }
#endif
    if (requestedTlmDenom != TlmDenom) {
      DBGLN("TLM denom clamped 1:%u -> 1:%u for 200Hz Full Gemini",
            requestedTlmDenom, TlmDenom);
    }
    ExpressLRS_currTlmDenom = TlmDenom;
    telemBurstValid = false;
    InvalidatePrebuiltTelemetry();
  }

  // Model match check
  // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in
  // that case Must use bitwise NOT (~) to match upstream ELRS behavior
  uint8_t modelXor = (~modelMatchId) & MODELMATCH_MASK;
  bool modelMatched = otaSync->UID5 == (UID[5] ^ modelXor);

  if (otaProtocolSelectionChanged && connectionState == disconnected &&
      !mlrs_ota_take_elrs_first_sync()) {
    RFmodeLastCycled = now;
    DBGLN("TX OTA protocol changed; waiting for next SYNC before lock");
    return false;
  }

  if (connectionState == disconnected || OtaNonce != otaSync->nonce ||
      FHSSgetCurrIndex() != otaSync->fhssIndex ||
      connectionHasModelMatch != modelMatched) {
    FHSSsetCurrIndex(otaSync->fhssIndex);
    OtaNonce = otaSync->nonce;
    TentativeConnection(now);
    connectionHasModelMatch = modelMatched;
    return true;
  }

  return false;
}

//=============================================================================
// Process RC data packet
//=============================================================================
static void ICACHE_RAM_ATTR
ProcessRfPacket_RC(OTA_Packet_s const *const otaPktPtr) {
  if ((connectionState != connected) || SwitchModePending) {
    return;
  }

  bool telemetryConfirmValue = OtaUnpackChannelData(otaPktPtr, ChannelData);
  noteDecodedUplinkTxPower();
  TelemetrySender.ConfirmCurrentPayload(telemetryConfirmValue);
  telemetryLastRcConfirm = telemetryConfirmValue;
  if (telemetryConfirmValue) {
    telemetryRcConfirmCount++;
  }
  InvalidatePrebuiltTelemetry();

  const bool shouldForwardChannels = updateTeamraceModelMatch();
  if (connectionHasModelMatch && shouldForwardChannels && channelCallback) {
    channelCallback(ChannelData, CRSF_NUM_CHANNELS);
  }
}

static void ICACHE_RAM_ATTR
ProcessRfPacket_DataUl(OTA_Packet_s const *const otaPktPtr) {
  uint8_t packageIndex;
  uint8_t const *payload;
  uint8_t dataLen;
  bool stubbornAck;

  if (OtaIsFullRes) {
    packageIndex = otaPktPtr->full.data_ul.packageIndex;
    stubbornAck = otaPktPtr->full.data_ul.stubbornAck;
    payload = otaPktPtr->full.data_ul.payload;
    dataLen = sizeof(otaPktPtr->full.data_ul.payload);
  } else {
    packageIndex = otaPktPtr->std.data_ul.packageIndex;
    stubbornAck = otaPktPtr->std.data_ul.stubbornAck;
    payload = otaPktPtr->std.data_ul.payload;
    dataLen = sizeof(otaPktPtr->std.data_ul.payload);
  }

  if (InBindingMode && packageIndex == 1 && payload[0] == MSP_ELRS_BIND) {
    OnELRSBindMSP((uint8_t *)&payload[1]);
    return;
  }

  if (connectionState != connected) {
    return;
  }

  dataUlChunkCount++;
  dataUlLastChunkMs = millis();
  dataUlLastPackageIndex = packageIndex;
  dataUlLastPayload0 = dataLen > 0 ? payload[0] : 0;
  dataUlLastPayload1 = dataLen > 1 ? payload[1] : 0;

  // Match upstream RX behavior: DATA_UL carries a valid downlink stubborn ACK
  // only for MAVLink data-over-OTA. CRSF Lua/device-management replies are
  // acknowledged by the normal RC telemetry-confirm bit instead.
  if (TelemetrySender.IsActive() &&
      getConfiguredSerialProtocol() == ELRS_SERIAL_MAVLINK) {
    TelemetrySender.ConfirmCurrentPayload(stubbornAck);
    InvalidatePrebuiltTelemetry();
    telemetryDataUlAckCount++;
    telemetryLastDataUlAck = stubbornAck;
  }

#if ELRS_DIAG_RX_LUA_UL
  const uint32_t ulChunkCount = ++rxLuaUlChunkCount;
  rxLuaLastUlPackageIndex = packageIndex;
#if ELRS_DIAG_RX_LUA_UL_VERBOSE
  if (rxLuaShouldPrintDiag(ulChunkCount)) {
    DBGLN("[RX_LUA] UL chunk #%lu pkg=%u len=%u bytes:%02X %02X %02X %02X %02X %02X",
          (unsigned long)ulChunkCount, packageIndex, dataLen, payload[0],
          dataLen > 1 ? payload[1] : 0, dataLen > 2 ? payload[2] : 0,
          dataLen > 3 ? payload[3] : 0, dataLen > 4 ? payload[4] : 0,
          dataLen > 5 ? payload[5] : 0);
  }
#endif
#endif

  DataUlReceiver.ReceiveData(packageIndex, payload, dataLen);
  if (DataUlReceiver.HasFinishedData()) {
    dataUlReady = true;
    dataUlCompleteCount++;
    dataUlLastCompleteMs = millis();
#if ELRS_DIAG_RX_LUA_UL && ELRS_DIAG_RX_LUA_UL_VERBOSE
    DBGLN("[RX_LUA] UL complete pending after pkg=%u first:%02X %02X %02X %02X %02X %02X %02X %02X",
          packageIndex, DataUlBuffer[0], DataUlBuffer[1], DataUlBuffer[2],
          DataUlBuffer[3], DataUlBuffer[4], DataUlBuffer[5], DataUlBuffer[6],
          DataUlBuffer[7]);
#endif
  }
}

//=============================================================================
// Main packet processing ISR
//=============================================================================

// Debug counter for CRC failures
static uint32_t crcFailCount = 0;
static uint32_t crcPassCount = 0;
static uint32_t lastCrcDebugTime = 0;
static volatile uint8_t lastValidPacketType = 0xFF;
static volatile uint8_t lastValidExpectedNonce = 0;
static volatile uint8_t lastValidExpectedFhss = 0;
static volatile uint8_t lastValidSyncNonce = 0xFF;
static volatile uint8_t lastValidSyncFhss = 0xFF;
static volatile uint32_t lastValidFreq = 0;
static volatile uint32_t lastRxDoneLatencyUs = 0;
static volatile uint32_t lastDioToDeferredUs = 0;
static volatile uint32_t lastDeferredToRxIsrUs = 0;
static volatile uint32_t lastRxIsrToPacketUs = 0;
static volatile uint32_t lastPacketToCallbackUs = 0;
static volatile int32_t lastPfdSlackUs = PACKET_TO_TOCK_SLACK;
static volatile uint8_t lastPfdUsedEdgeTimestamp = 0;
static volatile uint8_t lastRadioStat1 = 0;
static volatile uint8_t lastRadioStat2 = 0;
static volatile uint8_t lastRadioIrqByte = 0;
static volatile uint8_t lastRadioStatusOk = 0;
static volatile uint32_t pfdSkippedNoTimestampCount = 0;
static volatile int8_t lastCrcNonceDelta = 127;
static volatile uint8_t lastCrcNonceType = 0xFF;
static volatile uint8_t lastCrcNonceExpected = 0;
static volatile uint8_t lastCrcNonceMatched = 0xFF;
static volatile uint32_t crcNonceDiagHitCount = 0;
static volatile uint32_t crcNonceDiagMissCount = 0;

#if ELRS_DIAG_LUA_PROGRESS
static void maybePrintLuaProgress(unsigned long now) {
  static uint32_t lastPrintMs = 0;
  static uint32_t lastActivityMs = 0;
  static uint32_t lastHandledCount = 0;
  static uint32_t lastUlCompleteCount = 0;
  static uint32_t lastQueueDropCount = 0;

  const uint32_t handledCount = rxLuaCrsfHandledCount;
  const uint32_t ulCompleteCount = rxLuaUlCompleteCount;
  const uint32_t queueDropCount = rxLuaQueueDropCount;
  const bool senderActive = TelemetrySender.IsActive();
  const bool fifoPending = !otaConnector.IsEmpty();
  const bool queuePending = rxLuaQueueCount != 0;
  const bool activityChanged =
      (handledCount != lastHandledCount) ||
      (ulCompleteCount != lastUlCompleteCount) ||
      (queueDropCount != lastQueueDropCount);

  if (senderActive || fifoPending || queuePending || activityChanged) {
    lastActivityMs = now;
  } else if (lastActivityMs == 0 ||
             (uint32_t)(now - lastActivityMs) > 3000U) {
    return;
  }

  lastHandledCount = handledCount;
  lastUlCompleteCount = ulCompleteCount;
  lastQueueDropCount = queueDropCount;

  if (lastPrintMs != 0 &&
      (uint32_t)(now - lastPrintMs) < SIW917_ELRS_LUA_PROGRESS_INTERVAL_MS) {
    return;
  }
  lastPrintMs = now;

  const uint8_t rateIndex = ExpressLRS_currAirRate_Modparams
                                ? ExpressLRS_currAirRate_Modparams->index
                                : 0;
  DBGLN("LUA_PROGRESS conn:%d rxst:%d rate:%u den:%u burst:%u "
        "active:%u st:%u pkg:%u off:%u bytes:%u wait:%u/%u expack:%u "
        "q:%u fifo:%u drop:%lu ul:%lu/%lu crsf:%lu/%lu/%lu "
        "tlm:%lu/%lu ulack:%lu/%u last:%u exp:%u/%u age:%lu lq:%u/%u "
        "rssi:%d snr:%d",
        connectionState, RXtimerState, rateIndex, ExpressLRS_currTlmDenom,
        telemetryBurstMax, senderActive ? 1 : 0, TelemetrySender.GetState(),
        TelemetrySender.GetCurrentPackage(), TelemetrySender.GetCurrentOffset(),
        TelemetrySender.GetBytesLastPayload(), TelemetrySender.GetWaitCount(),
        TelemetrySender.GetMaxPacketsBeforeResync(),
        TelemetrySender.GetExpectedAck() ? 1 : 0, rxLuaQueueCount,
        fifoPending ? 1 : 0, (unsigned long)queueDropCount,
        (unsigned long)rxLuaUlChunkCount, (unsigned long)ulCompleteCount,
        (unsigned long)handledCount, (unsigned long)rxLuaCrsfRejectCount,
        (unsigned long)rxLuaCrsfIgnoredCount, (unsigned long)telemetryTxCount,
        (unsigned long)telemetrySuppressedCount,
        (unsigned long)telemetryDataUlAckCount,
        telemetryLastDataUlAck ? 1 : 0, lastValidPacketType,
        lastValidExpectedNonce, lastValidExpectedFhss,
        (unsigned long)(now - LastValidPacket), LQCalc.getLQRaw(),
        LQCalc.getCount(), Radio.LastPacketRSSI, Radio.LastPacketSNRRaw);
}
#else
static inline void maybePrintLuaProgress(unsigned long) {}
#endif

#if ELRS_DIAG_LINK_PROGRESS
static void maybePrintLinkProgress(unsigned long now) {
  static uint32_t lastPrintMs = 0;
  static uint32_t lastTelemetryTx = 0;
  static uint32_t lastTelemetrySuppressed = 0;
  static uint32_t lastPacketCapture = 0;
  static uint32_t lastCrcFail = 0;
  static uint32_t lastRcPass = 0;
  static uint32_t lastDataPass = 0;
  static uint32_t lastSyncPass = 0;
  static uint32_t lastRcConfirm = 0;
  static uint32_t lastDataUlAck = 0;
  static uint32_t lastPfdResult = 0;
  static uint32_t lastTimerTick = 0;
  static uint32_t lastTimerTock = 0;
  static uint32_t lastTimerOverflow = 0;
  static uint32_t lastIsrCount = 0;
  static uint32_t lastRxIrqCount = 0;
  static uint32_t lastTxIrqCount = 0;
  static uint32_t lastOtherIrqCount = 0;
  static uint32_t lastBusyFailCount = 0;
  static uint32_t lastRawGspiFailCount = 0;

  if (connectionState != connected && connectionState != tentative) {
    lastPrintMs = 0;
    return;
  }

  if (lastPrintMs != 0 &&
      (uint32_t)(now - lastPrintMs) < SIW917_ELRS_LINK_PROGRESS_INTERVAL_MS) {
    return;
  }

  uint32_t isrCount = 0;
  uint32_t rxIrqCount = 0;
  uint32_t txIrqCount = 0;
  uint32_t otherIrqCount = 0;
  uint32_t lastIrq = 0;
  lr1121_get_isr_stats(&isrCount, &rxIrqCount, &txIrqCount, &otherIrqCount,
                       &lastIrq);

  const uint32_t timerTick = hwTimer::getProcessedTickCount();
  const uint32_t timerTock = hwTimer::getProcessedTockCount();
  const uint32_t timerOverflow = hwTimer::getQueueOverflowCount();
  const uint32_t busyMaxIterations = lr1121_get_busy_fast_max_iterations();
  const uint32_t busyFailCount = lr1121_get_busy_fast_fail_count();
  const uint32_t rawGspiCount = lr1121_get_raw_gspi_count();
  const uint32_t rawGspiFailCount = lr1121_get_raw_gspi_fail_count();
  const uint32_t deltaTimerTock = timerTock - lastTimerTock;
  const uint32_t deltaRxIrq = rxIrqCount - lastRxIrqCount;
  const uint32_t deltaTxIrq = txIrqCount - lastTxIrqCount;
  const uint32_t deltaOtherIrq = otherIrqCount - lastOtherIrqCount;
  const uint32_t deltaIrqSlots = deltaRxIrq + deltaTxIrq + deltaOtherIrq;
  const uint32_t deltaNoIrqSlots =
      deltaTimerTock > deltaIrqSlots ? deltaTimerTock - deltaIrqSlots : 0;
  uint8_t radioStat1 = 0;
  uint8_t radioStat2 = 0;
  uint8_t radioIrqByte = 0;
  const uint8_t radioStatusOk =
      lr1121_get_status(&radioStat1, &radioStat2, &radioIrqByte) ? 1 : 0;
  const uint8_t radioCmdStatus =
      radioStatusOk ? ((radioStat1 >> 1) & 0x07) : 0xFF;
  const uint8_t radioMode = radioStatusOk ? ((radioStat2 >> 1) & 0x07) : 0xFF;
  const int dioLevel = lr1121_dio1_read();
  const uint32_t dioIrqEnabled = lr1121_dio1_irq_enabled();
  const uint32_t dioIrqPending = lr1121_dio1_irq_pending();
  const uint32_t dioGpioStatus = lr1121_dio1_gpio_intr_status();
  const uint8_t rateIndex = ExpressLRS_currAirRate_Modparams
                                ? ExpressLRS_currAirRate_Modparams->index
                                : 0;

  DBGLN("LINK_RF conn:%d rxst:%d rate:%u den:%u age:%lu lq:%u/%u "
        "rssi:%d snr:%d pkt:%lu/+%lu crc:%lu/+%lu "
        "pass:%lu/+%lu/%lu/+%lu/%lu/+%lu irq:%lu/+%lu rx:%lu/+%lu "
        "tx:%lu/+%lu oth:%lu/+%lu noirq:+%lu last:0x%08lX",
        connectionState, RXtimerState, rateIndex, ExpressLRS_currTlmDenom,
        (unsigned long)(now - LastValidPacket), LQCalc.getLQRaw(),
        LQCalc.getCount(), Radio.LastPacketRSSI, Radio.LastPacketSNRRaw,
        (unsigned long)pkt_capture_count,
        (unsigned long)(pkt_capture_count - lastPacketCapture),
        (unsigned long)crcFailCount,
        (unsigned long)(crcFailCount - lastCrcFail),
        (unsigned long)crcPassTypeCount[PACKET_TYPE_RCDATA],
        (unsigned long)(crcPassTypeCount[PACKET_TYPE_RCDATA] - lastRcPass),
        (unsigned long)crcPassTypeCount[PACKET_TYPE_DATA],
        (unsigned long)(crcPassTypeCount[PACKET_TYPE_DATA] - lastDataPass),
        (unsigned long)crcPassTypeCount[PACKET_TYPE_SYNC],
        (unsigned long)(crcPassTypeCount[PACKET_TYPE_SYNC] - lastSyncPass),
        (unsigned long)isrCount, (unsigned long)(isrCount - lastIsrCount),
        (unsigned long)rxIrqCount, (unsigned long)deltaRxIrq,
        (unsigned long)txIrqCount, (unsigned long)deltaTxIrq,
        (unsigned long)otherIrqCount, (unsigned long)deltaOtherIrq,
        (unsigned long)deltaNoIrqSlots,
        (unsigned long)lastIrq);
  DBGLN("LINK_TLM tlm:%lu/+%lu sup:%lu/+%lu rcack:%lu/+%lu/%u "
        "ulack:%lu/+%lu/%u next:%u bcnt:%u active:%u st:%u wait:%u/%u "
        "expack:%u q:%u fifo:%u",
        (unsigned long)telemetryTxCount,
        (unsigned long)(telemetryTxCount - lastTelemetryTx),
        (unsigned long)telemetrySuppressedCount,
        (unsigned long)(telemetrySuppressedCount - lastTelemetrySuppressed),
        (unsigned long)telemetryRcConfirmCount,
        (unsigned long)(telemetryRcConfirmCount - lastRcConfirm),
        telemetryLastRcConfirm ? 1 : 0,
        (unsigned long)telemetryDataUlAckCount,
        (unsigned long)(telemetryDataUlAckCount - lastDataUlAck),
        telemetryLastDataUlAck ? 1 : 0, NextTelemetryType, telemetryBurstCount,
        TelemetrySender.IsActive() ? 1 : 0, (unsigned)TelemetrySender.GetState(),
        TelemetrySender.GetWaitCount(),
        TelemetrySender.GetMaxPacketsBeforeResync(),
        TelemetrySender.GetExpectedAck() ? 1 : 0, rxLuaQueueCount,
        otaConnector.IsEmpty() ? 0 : 1);
  DBGLN("LINK_RAD stat:%u/%02X/%02X/%02X cmd:%u mode:%u dio:%d "
        "nvic:%lu/%lu gpio:0x%08lX",
        radioStatusOk, radioStat1, radioStat2, radioIrqByte, radioCmdStatus,
        radioMode, dioLevel, (unsigned long)dioIrqEnabled,
        (unsigned long)dioIrqPending, (unsigned long)dioGpioStatus);
  DBGLN("LINK_TIM tick:%lu/+%lu tock:%lu/+%lu of:%lu/+%lu hw:%lu/%lu/%lu "
        "pfd:%ld/%ld/%ld ph:%ld fo:%ld res:%lu/+%lu src:%u "
        "busy:%lu/%lu/+%lu gspi:%lu/%lu/+%lu "
        "crcN:%d/%u/%u/%lu/%lu nf:%u/%u",
        (unsigned long)timerTick,
        (unsigned long)(timerTick - lastTimerTick), (unsigned long)timerTock,
        (unsigned long)(timerTock - lastTimerTock),
        (unsigned long)timerOverflow,
        (unsigned long)(timerOverflow - lastTimerOverflow),
        (unsigned long)hwTimer::getHardwareCount(),
        (unsigned long)hwTimer::getHardwareMatch(),
        (unsigned long)hwTimer::getHardwareFreqHz(), (long)pfdLastRawOffset,
        (long)pfdLastOffset, (long)pfdLastOffsetDx, (long)pfdLastPhaseShift,
        (long)hwTimer::FreqOffset, (unsigned long)pfdResultCount,
        (unsigned long)(pfdResultCount - lastPfdResult),
        lastPfdUsedEdgeTimestamp, (unsigned long)busyMaxIterations,
        (unsigned long)busyFailCount,
        (unsigned long)(busyFailCount - lastBusyFailCount),
        (unsigned long)rawGspiCount, (unsigned long)rawGspiFailCount,
        (unsigned long)(rawGspiFailCount - lastRawGspiFailCount),
        (int)lastCrcNonceDelta, lastCrcNonceExpected, lastCrcNonceMatched,
        (unsigned long)crcNonceDiagHitCount,
        (unsigned long)crcNonceDiagMissCount, OtaNonce, FHSSgetCurrIndex());

  lastPrintMs = now;
  lastTelemetryTx = telemetryTxCount;
  lastTelemetrySuppressed = telemetrySuppressedCount;
  lastPacketCapture = pkt_capture_count;
  lastCrcFail = crcFailCount;
  lastRcPass = crcPassTypeCount[PACKET_TYPE_RCDATA];
  lastDataPass = crcPassTypeCount[PACKET_TYPE_DATA];
  lastSyncPass = crcPassTypeCount[PACKET_TYPE_SYNC];
  lastRcConfirm = telemetryRcConfirmCount;
  lastDataUlAck = telemetryDataUlAckCount;
  lastPfdResult = pfdResultCount;
  lastTimerTick = timerTick;
  lastTimerTock = timerTock;
  lastTimerOverflow = timerOverflow;
  lastIsrCount = isrCount;
  lastRxIrqCount = rxIrqCount;
  lastTxIrqCount = txIrqCount;
  lastOtherIrqCount = otherIrqCount;
  lastBusyFailCount = busyFailCount;
  lastRawGspiFailCount = rawGspiFailCount;
}
#else
static inline void maybePrintLinkProgress(unsigned long) {}
#endif

#if ELRS_DIAG_LOSS_PACKET_STATS
static void printLossPacketStats() {
  uint32_t isrCount = 0;
  uint32_t rxIrqCount = 0;
  uint32_t txIrqCount = 0;
  uint32_t otherIrqCount = 0;
  uint32_t lastIrq = 0;
  lr1121_get_isr_stats(&isrCount, &rxIrqCount, &txIrqCount, &otherIrqCount,
                       &lastIrq);

  const uint32_t deferredQueueCount =
#if SIW917_ELRS_DEFER_TLM_TX_FROM_TIMER
      telemetryDeferredQueueCount;
#else
      0;
#endif
  const uint32_t deferredSendCount =
#if SIW917_ELRS_DEFER_TLM_TX_FROM_TIMER
      telemetryDeferredSendCount;
#else
      0;
#endif
  const uint32_t deferredDropCount =
#if SIW917_ELRS_DEFER_TLM_TX_FROM_TIMER
      telemetryDeferredDropCount;
#else
      0;
#endif

  DBGLN("LOSSSTAT irq:%lu/%lu/%lu/%lu/0x%08lX dio:%d "
        "tmr:%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu "
        "rxok:%lu crc:%lu skip:%lu "
        "ok:%lu/%lu/%lu fail:%lu/%lu/%lu last:%u exp:%u/%u sync:%u/%u "
        "crcN:%d/%u/%u/%u/%lu/%lu "
        "freq:%lu/%lu lq:%u/%u tlm:%lu/%u/%u def:%lu/%lu/%lu rssi:%d snr:%d "
        "lat:%lu/%lu/%lu/%lu/%lu",
        (unsigned long)isrCount, (unsigned long)rxIrqCount,
        (unsigned long)txIrqCount, (unsigned long)otherIrqCount,
        (unsigned long)lastIrq, lr1121_dio1_read(),
        (unsigned long)hwTimer::getHardwareHalfTicks(),
        (unsigned long)hwTimer::getProcessedTickCount(),
        (unsigned long)hwTimer::getProcessedTockCount(),
        (unsigned long)hwTimer::getImmediateTockDeliveredCount(),
        (unsigned long)hwTimer::getQueuedTickCount(),
        (unsigned long)hwTimer::getQueueOverflowCount(),
        (unsigned long)hwTimer::getHardwareCount(),
        (unsigned long)hwTimer::getHardwareMatch(),
        (unsigned long)hwTimer::getHardwareFreqHz(),
        (unsigned long)pkt_capture_count, (unsigned long)crcFailCount,
        (unsigned long)rxLqCurrentSetSkipCount,
        (unsigned long)crcPassTypeCount[PACKET_TYPE_RCDATA],
        (unsigned long)crcPassTypeCount[PACKET_TYPE_DATA],
        (unsigned long)crcPassTypeCount[PACKET_TYPE_SYNC],
        (unsigned long)crcFailTypeCount[PACKET_TYPE_RCDATA],
        (unsigned long)crcFailTypeCount[PACKET_TYPE_DATA],
        (unsigned long)crcFailTypeCount[PACKET_TYPE_SYNC],
        lastValidPacketType, lastValidExpectedNonce, lastValidExpectedFhss,
        lastValidSyncNonce, lastValidSyncFhss,
        (int)lastCrcNonceDelta, lastCrcNonceExpected, lastCrcNonceMatched,
        lastCrcNonceType, (unsigned long)crcNonceDiagHitCount,
        (unsigned long)crcNonceDiagMissCount, (unsigned long)Radio.currFreq,
        (unsigned long)lastValidFreq, LQCalc.getLQRaw(), LQCalc.getCount(),
        (unsigned long)telemetryTxCount, ExpressLRS_currTlmDenom,
        telemetryBurstMax, (unsigned long)deferredQueueCount,
        (unsigned long)deferredSendCount, (unsigned long)deferredDropCount,
        Radio.LastPacketRSSI, Radio.LastPacketSNRRaw,
        (unsigned long)lastRxDoneLatencyUs,
        (unsigned long)lastDioToDeferredUs,
        (unsigned long)lastDeferredToRxIsrUs,
        (unsigned long)lastRxIsrToPacketUs,
        (unsigned long)lastPacketToCallbackUs);
}
#endif

static inline int32_t ICACHE_RAM_ATTR absI32(int32_t value) {
  return value < 0 ? -value : value;
}

static inline int32_t ICACHE_RAM_ATTR maxI32(int32_t lhs, int32_t rhs) {
  return lhs > rhs ? lhs : rhs;
}

static void ICACHE_RAM_ATTR
diagnoseCrcFailureNonce(OTA_Packet_s const *const otaPktPtr,
                        uint8_t const type) {
#if ELRS_DIAG_CRC_NONCE_WINDOW > 0
  if (type == PACKET_TYPE_SYNC) {
    return;
  }

  const uint8_t expectedNonce = OtaNonce;
  lastCrcNonceType = type;
  lastCrcNonceExpected = expectedNonce;

  for (int8_t delta = -ELRS_DIAG_CRC_NONCE_WINDOW;
       delta <= ELRS_DIAG_CRC_NONCE_WINDOW; delta++) {
    OTA_Packet_s packetCopy;
    memcpy(&packetCopy, otaPktPtr, sizeof(packetCopy));
    const uint8_t candidateNonce = (uint8_t)(expectedNonce + delta);
    if (OtaValidatePacketCrcForNonce(&packetCopy, candidateNonce)) {
      lastCrcNonceDelta = delta;
      lastCrcNonceMatched = candidateNonce;
      crcNonceDiagHitCount++;
      return;
    }
  }

  lastCrcNonceDelta = 127;
  lastCrcNonceMatched = 0xFF;
  crcNonceDiagMissCount++;
#else
  (void)otaPktPtr;
  (void)type;
#endif
}

static bool ICACHE_RAM_ATTR
ProcessRFPacket(SX12xxDriverCommon::rx_status const status) {
  if (status != SX12xxDriverCommon::SX12XX_RX_OK) {
    return false;
  }

  uint32_t const beginProcessing = micros();
  uint32_t packetTimeUs = beginProcessing;
  uint8_t pfdSourceForPacket = 0;
#if SIW917_ELRS_HOTPATH_TIMING_DIAG || ELRS_DIAG_USE_DIO_PFD_TIMESTAMP
  uint32_t const dio1EdgeUs = lr1121_hal_get_last_dio1_edge_us();
  uint32_t const deferredUs = lr1121_hal_get_last_deferred_us();
  uint32_t const rxIsrEntryUs = lr1121_get_last_rxnbisr_entry_us();
  uint32_t const packetReadyUs = lr1121_get_last_packet_ready_us();
  const bool timerRunningForPfd = hwTimer::isRunning();
  if (dio1EdgeUs != 0U) {
    const uint32_t irqLatencyUs = beginProcessing - dio1EdgeUs;
    if (irqLatencyUs < 10000U) {
      lastRxDoneLatencyUs = irqLatencyUs;
#if ELRS_DIAG_USE_DIO_PFD_TIMESTAMP
      packetTimeUs = dio1EdgeUs;
      if (timerRunningForPfd) {
        pfdSourceForPacket = 1;
      }
#endif
    }
    const uint32_t dioToDeferred = deferredUs - dio1EdgeUs;
    if (deferredUs != 0U && dioToDeferred < 10000U) {
      lastDioToDeferredUs = dioToDeferred;
    }
  }
  if (deferredUs != 0U && rxIsrEntryUs != 0U) {
    const uint32_t deferredToRxIsr = rxIsrEntryUs - deferredUs;
    if (deferredToRxIsr < 10000U) {
      lastDeferredToRxIsrUs = deferredToRxIsr;
    }
  }
  if (rxIsrEntryUs != 0U && packetReadyUs != 0U) {
    const uint32_t rxIsrToPacket = packetReadyUs - rxIsrEntryUs;
    if (rxIsrToPacket < 10000U) {
      lastRxIsrToPacketUs = rxIsrToPacket;
    }
  }
  if (packetReadyUs != 0U) {
    const uint32_t packetToCallback = beginProcessing - packetReadyUs;
    if (packetToCallback < 10000U) {
      lastPacketToCallbackUs = packetToCallback;
    }
  }
#endif
  OTA_Packet_s *const otaPktPtr = (OTA_Packet_s *const)Radio.RXdataBuffer;
  uint8_t const type = Radio.RXdataBuffer[0] & 0x03;

  // Validate CRC
  if (!OtaValidatePacketCrc(otaPktPtr)) {
    ELRS_PACKET_STAT_INC(crcFailCount);
    ELRS_PACKET_STAT_INC(crcFailTypeCount[type]);
    diagnoseCrcFailureNonce(otaPktPtr, type);
    return false;
  }

  ELRS_PACKET_STAT_INC(crcPassCount);
  ELRS_PACKET_STAT_INC(crcPassTypeCount[type]);
  ELRS_PACKET_STAT_SET(lastValidPacketType, type);
  ELRS_PACKET_STAT_SET(lastValidExpectedNonce, OtaNonce);
  ELRS_PACKET_STAT_SET(lastValidExpectedFhss, FHSSgetCurrIndex());
  ELRS_PACKET_STAT_SET(lastValidFreq, Radio.currFreq);
  ELRS_PACKET_STAT_SET(lastPfdUsedEdgeTimestamp, pfdSourceForPacket);
  uint32_t const now = millis();

  if (ExpressLRS_currAirRate_Modparams != nullptr &&
      ExpressLRS_currAirRate_RFperfParams != nullptr) {
    const int32_t slack = maxI32(
        (int32_t)ExpressLRS_currAirRate_Modparams->interval -
            2 * (int32_t)ExpressLRS_currAirRate_RFperfParams->TOA,
        (int32_t)PACKET_TO_TOCK_SLACK);
    lastPfdSlackUs = slack;
    PFDloop.extEvent(packetTimeUs + slack);
  }

#if ELRS_DIAG_PACKET_CAPTURE
  uint32_t cidx = pkt_capture_idx % PKT_CAPTURE_SIZE;
  memcpy((void *)pkt_capture_buf[cidx], (void *)Radio.RXdataBuffer, 8);
  pkt_capture_len[cidx] = 8;
  pkt_capture_idx++;
  pkt_capture_count++;
#endif

  // Record valid packet time
  LastValidPacket = now;

  doStartTimer = false;

  // Handle packet based on type
  switch (type) {
  case PACKET_TYPE_RCDATA:
    ProcessRfPacket_RC(otaPktPtr);
    break;

  case PACKET_TYPE_SYNC: {
    OTA_Sync_s const *const sync =
        OtaIsFullRes ? &otaPktPtr->full.sync.sync : &otaPktPtr->std.sync;
    ELRS_PACKET_STAT_SET(lastValidSyncNonce, sync->nonce);
    ELRS_PACKET_STAT_SET(lastValidSyncFhss, sync->fhssIndex);
    doStartTimer = ProcessRfPacket_SYNC(now, sync) && !InBindingMode;
    break;
  }

  case PACKET_TYPE_DATA:
    ProcessRfPacket_DataUl(otaPktPtr);
    break;
  }

  // Match downstream RX core ordering: packet contents are handled before the
  // current LQ period is marked as consumed.
  Radio.GetLastPacketStats();
  getRFlinkInfo();
  LQCalc.add();
  RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

  return true;
}

//=============================================================================
// Radio ISR callbacks
//=============================================================================
static bool SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
RXdoneISR(SX12xxDriverCommon::rx_status rxStatus) {
  if (LQCalc.currentIsSet() && connectionState == connected) {
    ELRS_PACKET_STAT_INC(rxLqCurrentSetSkipCount);
#if ELRS_DIAG_TX_TURNAROUND
    if (telemetryAwaitingRx) {
      telemetryLqSetWhileAwaitingCount++;
    }
#endif
    return false;
  }

  if (ProcessRFPacket(rxStatus)) {
#if ELRS_DIAG_TX_TURNAROUND
    if (telemetryAwaitingRx) {
      const uint32_t now = micros();
      telemetryRxnbToRxOkUs = now - telemetryRxnbDoneUs;
      telemetryRxNonce = OtaNonce;
      telemetryRxFhss = FHSSgetCurrIndex();
      telemetryRxAfterTx++;
      telemetryAwaitingRx = 0;
      telemetryStaleRxReported = 0;
    }
#endif
    if (doStartTimer) {
      doStartTimer = false;
      hwTimer::resume();
    }
    return true;
  }

  return false;
}

static void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR TXdoneISR() {
  // After sending telemetry, switch back to RX
#if ELRS_DIAG_TX_TURNAROUND
  const uint32_t txDoneUs = micros();
  telemetryTxDoneUs = txDoneUs;
  telemetryTxToDoneUs = txDoneUs - telemetryTxStartUs;
#endif
  Radio.RXnb();
#if ELRS_DIAG_TX_TURNAROUND
  const uint32_t rxnbDoneUs = micros();
  telemetryRxnbDoneUs = rxnbDoneUs;
  telemetryDoneToRxnbDoneUs = rxnbDoneUs - txDoneUs;
  telemetryTxToRxnbDoneUs = rxnbDoneUs - telemetryTxStartUs;
  telemetryAwaitingRx = 1;
  telemetryStaleRxReported = 0;
#endif
}

static void maybeReportStaleTelemetryRx() {
#if ELRS_DIAG_TX_TURNAROUND && ELRS_DIAG_TLM_STALE_RX
  if (!telemetryAwaitingRx || telemetryStaleRxReported ||
      connectionState != connected ||
      ExpressLRS_currAirRate_Modparams == nullptr) {
    return;
  }

  const uint32_t ageUs = micros() - telemetryRxnbDoneUs;
  const uint32_t intervalUs = ExpressLRS_currAirRate_Modparams->interval;
  const uint32_t thresholdUs = intervalUs + 1000U;
  if (ageUs < thresholdUs || ageUs > 1000000U) {
    return;
  }

  telemetryStaleRxReported = 1;
  telemetryStaleRxReportCount++;
  telemetryStaleRxLastAgeUs = ageUs;

  uint32_t isrCount = 0;
  uint32_t rxIrqCount = 0;
  uint32_t txIrqCount = 0;
  uint32_t otherIrqCount = 0;
  uint32_t lastIrq = 0;
  lr1121_get_isr_stats(&isrCount, &rxIrqCount, &txIrqCount, &otherIrqCount,
                       &lastIrq);

  DBGLN("TLM_STALE_RX age:%lu int:%lu den:%u rate:%u tx:%u/%u rx:%u/%u "
        "exp:%u/%u sync:%u/%u vf:%lu lq:%u/%u dio:%lu/%lu/%lu irq:%lu/%lu/%lu/%lu/0x%08lX "
        "lqskip:%lu stale:%lu",
        (unsigned long)ageUs, (unsigned long)intervalUs,
        ExpressLRS_currTlmDenom,
        ExpressLRS_currAirRate_Modparams
            ? ExpressLRS_currAirRate_Modparams->index
            : 0,
        telemetryTxNonce, telemetryTxFhss, telemetryRxNonce, telemetryRxFhss,
        lastValidExpectedNonce, lastValidExpectedFhss, lastValidSyncNonce,
        lastValidSyncFhss, (unsigned long)lastValidFreq, LQCalc.getLQRaw(),
        LQCalc.getCount(), (unsigned long)lr1121_hal_get_direct_dio_count(),
        (unsigned long)lr1121_hal_get_direct_reentrant_count(),
        (unsigned long)lr1121_hal_get_level_requeue_count(),
        (unsigned long)isrCount, (unsigned long)rxIrqCount,
        (unsigned long)txIrqCount, (unsigned long)otherIrqCount,
        (unsigned long)lastIrq,
        (unsigned long)telemetryLqSetWhileAwaitingCount,
        (unsigned long)telemetryStaleRxReportCount);
#endif
}

#if ELRS_DIAG_TX_TURNAROUND && ELRS_DIAG_TLM150_SNAPSHOT
static bool isTelemetry150Rate(const expresslrs_mod_settings_s *params) {
  return params &&
         ((params->enum_rate == RATE_LORA_900_100HZ) ||
          (params->enum_rate == RATE_LORA_900_100HZ_8CH) ||
          (params->enum_rate == RATE_LORA_2G4_100HZ) ||
          (params->enum_rate == RATE_LORA_2G4_100HZ_8CH) ||
          (params->enum_rate == RATE_LORA_DUAL_100HZ_8CH) ||
          (params->enum_rate == RATE_LORA_2G4_150HZ) ||
          (params->enum_rate == RATE_LORA_DUAL_150HZ));
}

static void armTelemetry150Snapshot(const expresslrs_mod_settings_s *params,
                                    bool bindMode) {
  const bool shouldArm = !bindMode && isTelemetry150Rate(params);
  telemetry150DiagNeedArm = shouldArm;
  telemetry150DiagArmed = false;
  telemetry150DiagPrinted = false;
}

static void maybePrintTelemetry150Snapshot(unsigned long now) {
  if (telemetry150DiagNeedArm && connectionState == connected &&
      isTelemetry150Rate(ExpressLRS_currAirRate_Modparams)) {
    telemetry150DiagNeedArm = false;
    telemetry150DiagArmed = true;
    telemetry150DiagPrinted = false;
    telemetry150DiagArmMs = now;
    telemetry150DiagArmTxCount = telemetryTxCount;
    telemetry150DiagArmRxAfterTx = telemetryRxAfterTx;
    telemetry150DiagArmTxBeforeRx = telemetryTxBeforeRx;
    telemetry150DiagArmRcConfirm = telemetryRcConfirmCount;
    telemetry150DiagArmDataUlAck = telemetryDataUlAckCount;
  }

  if (!telemetry150DiagArmed || telemetry150DiagPrinted ||
      !isTelemetry150Rate(ExpressLRS_currAirRate_Modparams)) {
    return;
  }

  const uint32_t elapsedMs = now - telemetry150DiagArmMs;
  const uint32_t txDelta = telemetryTxCount - telemetry150DiagArmTxCount;
  if (txDelta == 0) {
    if (elapsedMs < 1000U) {
      return;
    }

    DBGLN("TLMFAST_NO_TX age:%lu den:%u burst:%u active:%u st:%u wait:%u/%u "
          "lq:%u/%u dio:%lu/%lu/%lu",
          (unsigned long)elapsedMs, ExpressLRS_currTlmDenom,
          telemetryBurstMax, TelemetrySender.IsActive() ? 1 : 0,
          (unsigned)TelemetrySender.GetState(), TelemetrySender.GetWaitCount(),
          TelemetrySender.GetMaxPacketsBeforeResync(), LQCalc.getLQRaw(),
          LQCalc.getCount(),
          (unsigned long)lr1121_hal_get_direct_dio_count(),
          (unsigned long)lr1121_hal_get_direct_reentrant_count(),
          (unsigned long)lr1121_hal_get_level_requeue_count());
    telemetry150DiagPrinted = true;
    return;
  }

  const bool txPathDone = (telemetryTxDoneUs != 0) && (telemetryRxnbDoneUs != 0);
  if (!txPathDone && elapsedMs < 250U) {
    return;
  }

  const uint32_t interval = ExpressLRS_currAirRate_Modparams->interval;
  const uint32_t nowUs = micros();
  const bool rxSettled =
      !telemetryAwaitingRx || (telemetryMissedAfterTx != 0) ||
      ((uint32_t)(nowUs - telemetryRxnbDoneUs) > (interval + 500U));
  if (txPathDone && !rxSettled && elapsedMs < 750U) {
    return;
  }

  const int32_t fitUs =
      (int32_t)interval - (int32_t)telemetryTxToRxnbDoneUs;
  DBGLN("TLMFAST_TX age:%lu tx:%lu interval:%lu fit:%ld den:%u burst:%u "
        "active:%u st:%u wait:%u/%u t2d:%lu d2rx:%lu tx2rx:%lu rxok:%lu "
        "miss:%lu pend:%u overlap:%lu ok:%lu rcack:%lu/%u ulack:%lu/%u tx:%u/%u rx:%u/%u "
        "lq:%u/%u dio:%lu/%lu/%lu",
        (unsigned long)elapsedMs, (unsigned long)txDelta,
        (unsigned long)interval, (long)fitUs, ExpressLRS_currTlmDenom,
        telemetryBurstMax, TelemetrySender.IsActive() ? 1 : 0,
        (unsigned)TelemetrySender.GetState(), TelemetrySender.GetWaitCount(),
        TelemetrySender.GetMaxPacketsBeforeResync(),
        (unsigned long)telemetryTxToDoneUs,
        (unsigned long)telemetryDoneToRxnbDoneUs,
        (unsigned long)telemetryTxToRxnbDoneUs,
        (unsigned long)telemetryRxnbToRxOkUs,
        (unsigned long)telemetryMissedAfterTx, telemetryAwaitingRx,
        (unsigned long)(telemetryTxBeforeRx - telemetry150DiagArmTxBeforeRx),
        (unsigned long)(telemetryRxAfterTx - telemetry150DiagArmRxAfterTx),
        (unsigned long)(telemetryRcConfirmCount - telemetry150DiagArmRcConfirm),
        telemetryLastRcConfirm ? 1 : 0,
        (unsigned long)(telemetryDataUlAckCount - telemetry150DiagArmDataUlAck),
        telemetryLastDataUlAck ? 1 : 0, telemetryTxNonce, telemetryTxFhss,
        telemetryRxNonce, telemetryRxFhss, LQCalc.getLQRaw(), LQCalc.getCount(),
        (unsigned long)lr1121_hal_get_direct_dio_count(),
        (unsigned long)lr1121_hal_get_direct_reentrant_count(),
        (unsigned long)lr1121_hal_get_level_requeue_count());
  telemetry150DiagPrinted = true;
}
#else
static void armTelemetry150Snapshot(const expresslrs_mod_settings_s *params,
                                    bool bindMode) {
  (void)params;
  (void)bindMode;
}

static void maybePrintTelemetry150Snapshot(unsigned long now) { (void)now; }
#endif

//=============================================================================
// RF Link rate setting
//=============================================================================
static inline bool isCrossbandRate(const expresslrs_mod_settings_s *params) {
  return params != nullptr &&
         params->radio_type == RADIO_TYPE_LR1121_LORA_DUAL;
}

static void SetRFLinkRate(uint8_t index, bool bindMode) {
#if ELRS_DIAG_RF_RATE_LOG
  DBGLN("SetRFLinkRate begin: index=%d bind=%d", index, bindMode ? 1 : 0);
#endif

  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

  if (!ModParams || !RFperf) {
    DBGLN("Invalid rate index: %d", index);
    return;
  }

  // Binding always uses invertIQ
  bool invertIQ = bindMode || (UID[5] & 0x01);

  uint32_t interval = ModParams->interval;
  hwTimer::updateInterval(interval);
#if ELRS_DIAG_RF_RATE_LOG
  DBGLN("SetRFLinkRate timer updated: interval=%lu",
        (unsigned long)interval);
#endif

  // Configure FHSS band selection
  const bool crossbandRate = isDualRadio() && isCrossbandRate(ModParams);
  FHSSusePrimaryFreqBand =
      crossbandRate ||
      ((ModParams->radio_type != RADIO_TYPE_LR1121_LORA_2G4) &&
       (ModParams->radio_type != RADIO_TYPE_LR1121_GFSK_2G4));
  FHSSuseDualBand = crossbandRate;

  uint32_t initFreq = FHSSgetInitialFreq();
#if ELRS_DIAG_RF_RATE_LOG
  DBGLN("SetRFLinkRate: index=%d, radio_type=%d, primaryBand=%d, initFreq=%u",
        index, ModParams->radio_type, FHSSusePrimaryFreqBand,
        (unsigned int)initFreq);
#endif

  updateRxDownlinkPower(false);

  // Configure radio
  if (crossbandRate) {
    const uint32_t radio2InitFreq = FHSSgetInitialGeminiFreq();
    if (!lr1121_hal_prepare_radio2_image_calibration(true)) {
      DBGLN("Crossband warning: radio2 2.4GHz image calibration failed");
    }
    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, initFreq,
                 ModParams->PreambleLen, invertIQ, ModParams->PayloadLength,
                 false, (uint8_t)UID[5], (uint8_t)UID[4], SX12XX_Radio_1);
    Radio.Config(ModParams->bw2, ModParams->sf2, ModParams->cr2,
                 radio2InitFreq, ModParams->PreambleLen2, invertIQ,
                 ModParams->PayloadLength, false, (uint8_t)UID[5],
                 (uint8_t)UID[4], SX12XX_Radio_2);
#if ELRS_DIAG_RF_RATE_LOG
    DBGLN("SetRFLinkRate crossband radio1=%u radio2=%u",
          (unsigned int)initFreq, (unsigned int)radio2InitFreq);
#endif
#if SIW917_ELRS_CROSSBAND_TEST_LOG
    DBGLN("Crossband active: idx=%u radio1=%u radio2=%u", index,
          (unsigned int)initFreq, (unsigned int)radio2InitFreq);
#endif
  } else {
    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, initFreq,
                 ModParams->PreambleLen, invertIQ, ModParams->PayloadLength,
                 ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_900 ||
                     ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4,
                 (uint8_t)UID[5], (uint8_t)UID[4]);
  }
  if (!crossbandRate && isDualRadio() && geminiMode) {
    const uint32_t geminiInitFreq = FHSSgetInitialGeminiFreq();
    if (geminiInitFreq < 1000000000UL &&
        !lr1121_hal_prepare_radio2_image_calibration(false)) {
      DBGLN("Gemini warning: radio2 sub-GHz image calibration failed");
    }
    Radio.SetFrequencyReg(geminiInitFreq, SX12XX_Radio_2, false);
#if ELRS_DIAG_RF_RATE_LOG
    DBGLN("SetRFLinkRate Gemini radio2 initFreq=%u mode=%u",
          (unsigned int)geminiInitFreq, (unsigned)geminiMode);
#endif
  }
  Radio.FuzzySNRThreshold =
      (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE)
          ? 0
          : (RFperf->DynpowerSnrThreshDn - RFperf->DynpowerSnrThreshUp);
#if ELRS_DIAG_RF_RATE_LOG
  DBGLN("SetRFLinkRate radio configured");
#endif

  // Update OTA serializers
  OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);
  DataUlReceiver.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
  TelemetrySender.setMaxPackageIndex(OtaIsFullRes ? ELRS8_DATA_DL_MAX_PACKAGES
                                                  : ELRS4_DATA_DL_MAX_PACKAGES);

  // Calculate cycle interval for rate scanning
  cycleInterval = ((uint32_t)11U * FHSSgetChannelCount() *
                   ModParams->FHSShopInterval * interval) /
                  (10U * 1000U);

  ExpressLRS_currAirRate_Modparams = ModParams;
  ExpressLRS_currAirRate_RFperfParams = RFperf;
  ExpressLRS_nextAirRateIndex = index;
  telemBurstValid = false;
  InvalidatePrebuiltTelemetry();
  armTelemetry150Snapshot(ModParams, bindMode);

#if ELRS_DIAG_RF_RATE_LOG
  const uint8_t defaultTlmDenom = TLMratioEnumToValue(ModParams->TLMinterval);
  DBGLN("Set RF rate index %d, interval %lu us, default TLM 1:%u", index,
        interval, defaultTlmDenom);
#endif
}

static bool ICACHE_RAM_ATTR isTelemetrySlotForNonce(uint8_t nonce) {
  // The SiW917/LR1121 path is very sensitive during K1000 tentative lock,
  // especially when MAVLink asks for 1:2 telemetry. Keep RF receive-only until
  // the local timer is locked, then allow the normal downlink schedule.
  return (connectionState == connected) && (RXtimerState == tim_locked) &&
         (ExpressLRS_currTlmDenom != 1) &&
         !alreadyTLMresp && teamraceHasModelMatch &&
         ((nonce % ExpressLRS_currTlmDenom) == 0);
}

static void ICACHE_RAM_ATTR GenerateTelemetryPacketCrcForNonce(OTA_Packet_s *pkt,
                                                               uint8_t nonce) {
  const uint8_t savedNonce = OtaNonce;
  OtaNonce = nonce;
  OtaGeneratePacketCrc(pkt);
  OtaNonce = savedNonce;
}

static bool ICACHE_RAM_ATTR shouldSendGeminiTelemetry() {
  return isDualRadio() && geminiMode;
}

static bool ICACHE_RAM_ATTR isRxTelemetryForcedOff() {
  return forceTelemetryOff;
}

static bool ICACHE_RAM_ATTR shouldDeferPowerCommitForTelemetrySlot() {
#if SIW917_ELRS_DEFER_PWR_COMMIT_200HZ_FULL_GEMINI
  return isDualRadio() && geminiMode && OtaIsFullRes &&
         ExpressLRS_currAirRate_Modparams &&
         ExpressLRS_currAirRate_Modparams->enum_rate ==
             RATE_LORA_900_200HZ_8CH &&
         ExpressLRS_currTlmDenom <= 2U;
#else
  return false;
#endif
}

static uint8_t ICACHE_RAM_ATTR PrepareTelemetryPayloadSpan(
    uint8_t *primaryPayload, uint8_t *geminiPayload, uint8_t payloadLen,
    bool useGemini, StubbornSenderPreparedPayload *preparedPayload) {
  if (useGemini) {
    WORD_ALIGNED_ATTR uint8_t geminiSpanBuffer[2 * ELRS8_DATA_DL_BYTES_PER_CALL] =
        {};
    const uint8_t spanLen = (uint8_t)(payloadLen * 2U);
    const uint8_t packageIndex = TelemetrySender.PrepareCurrentPayload(
        geminiSpanBuffer, spanLen, preparedPayload);
    memcpy(primaryPayload, geminiSpanBuffer, payloadLen);
    if (geminiPayload) {
      memcpy(geminiPayload, &geminiSpanBuffer[payloadLen], payloadLen);
    }
    return packageIndex;
  }

  return TelemetrySender.PrepareCurrentPayload(primaryPayload, payloadLen,
                                               preparedPayload);
}

static bool ICACHE_RAM_ATTR
BuildTelemetryPacket(OTA_Packet_s *otaPkt, OTA_Packet_s *otaPktGemini,
                     bool *sendGeminiBuffer,
                     StubbornSenderPreparedPayload *preparedPayload,
                     uint8_t nonce, uint8_t *nextTelemetryType,
                     uint8_t *nextTelemetryBurstCount) {
  memset(otaPkt, 0, sizeof(*otaPkt));
  if (otaPktGemini) {
    memset(otaPktGemini, 0, sizeof(*otaPktGemini));
  }
  if (sendGeminiBuffer) {
    *sendGeminiBuffer = false;
  }

  uint8_t localNextTelemetryType = NextTelemetryType;
  uint8_t localTelemetryBurstCount = telemetryBurstCount;
  bool tlmQueued = TelemetrySender.IsActive();
  const bool useGemini =
      shouldSendGeminiTelemetry() && otaPktGemini && sendGeminiBuffer;

  if ((localNextTelemetryType == PACKET_TYPE_LINKSTATS) || !tlmQueued) {
    otaPkt->std.type = PACKET_TYPE_LINKSTATS;

    if (OtaIsFullRes) {
      otaPkt->full.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm();
      const uint8_t payloadLen =
          sizeof(otaPkt->full.data_dl.ul_link_stats.payload);
      if (useGemini) {
        *otaPktGemini = *otaPkt;
      }
      otaPkt->full.data_dl.packageIndex = PrepareTelemetryPayloadSpan(
          otaPkt->full.data_dl.ul_link_stats.payload,
          useGemini ? otaPktGemini->full.data_dl.ul_link_stats.payload
                    : nullptr,
          payloadLen, useGemini, preparedPayload);
      LinkStatsToOta(&otaPkt->full.data_dl.ul_link_stats.stats);
      if (useGemini) {
        otaPktGemini->full.data_dl.packageIndex =
            otaPkt->full.data_dl.packageIndex;
        otaPktGemini->full.data_dl.ul_link_stats.stats =
            otaPkt->full.data_dl.ul_link_stats.stats;
        *sendGeminiBuffer = true;
      }
    } else {
      otaPkt->std.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm();
      const uint8_t payloadLen =
          sizeof(otaPkt->std.data_dl.ul_link_stats.payload);
      if (useGemini) {
        *otaPktGemini = *otaPkt;
      }
      otaPkt->std.data_dl.packageIndex = PrepareTelemetryPayloadSpan(
          otaPkt->std.data_dl.ul_link_stats.payload,
          useGemini ? otaPktGemini->std.data_dl.ul_link_stats.payload
                    : nullptr,
          payloadLen, useGemini, preparedPayload);
      LinkStatsToOta(&otaPkt->std.data_dl.ul_link_stats.stats);
      if (useGemini) {
        otaPktGemini->std.data_dl.packageIndex =
            otaPkt->std.data_dl.packageIndex;
        otaPktGemini->std.data_dl.ul_link_stats.stats =
            otaPkt->std.data_dl.ul_link_stats.stats;
        *sendGeminiBuffer = true;
      }
    }

    localNextTelemetryType = PACKET_TYPE_DATA;
    localTelemetryBurstCount = 1;
  } else {
    if (localTelemetryBurstCount < telemetryBurstMax) {
      localTelemetryBurstCount++;
    } else {
      localNextTelemetryType = PACKET_TYPE_LINKSTATS;
    }

    otaPkt->std.type = PACKET_TYPE_DATA;
    if (OtaIsFullRes) {
      otaPkt->full.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm();
      const uint8_t payloadLen = sizeof(otaPkt->full.data_dl.payload);
      if (useGemini) {
        *otaPktGemini = *otaPkt;
      }
      otaPkt->full.data_dl.packageIndex = PrepareTelemetryPayloadSpan(
          otaPkt->full.data_dl.payload,
          useGemini ? otaPktGemini->full.data_dl.payload : nullptr, payloadLen,
          useGemini, preparedPayload);
      if (useGemini) {
        otaPktGemini->full.data_dl.packageIndex =
            otaPkt->full.data_dl.packageIndex;
        *sendGeminiBuffer = true;
      }
    } else {
      otaPkt->std.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm();
      const uint8_t payloadLen = sizeof(otaPkt->std.data_dl.payload);
      if (useGemini) {
        *otaPktGemini = *otaPkt;
      }
      otaPkt->std.data_dl.packageIndex = PrepareTelemetryPayloadSpan(
          otaPkt->std.data_dl.payload,
          useGemini ? otaPktGemini->std.data_dl.payload : nullptr, payloadLen,
          useGemini, preparedPayload);
      if (useGemini) {
        otaPktGemini->std.data_dl.packageIndex =
            otaPkt->std.data_dl.packageIndex;
        *sendGeminiBuffer = true;
      }
    }
  }

  GenerateTelemetryPacketCrcForNonce(otaPkt, nonce);
  if (useGemini) {
    GenerateTelemetryPacketCrcForNonce(otaPktGemini, nonce);
  }
  *nextTelemetryType = localNextTelemetryType;
  *nextTelemetryBurstCount = localTelemetryBurstCount;
  return true;
}

static void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
SendTelemetryPacket(OTA_Packet_s *otaPkt, OTA_Packet_s *otaPktGemini,
                    bool sendGeminiBuffer, uint8_t diagNonce = OtaNonce,
                    uint8_t diagFhss = FHSSgetCurrIndex()) {
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  const uint32_t sendStartUs = micros();
#endif
#if ELRS_DIAG_TX_TURNAROUND
  if (telemetryAwaitingRx) {
    telemetryTxBeforeRx++;
  }
  telemetryTxStartUs = micros();
  telemetryTxDoneUs = 0;
  telemetryRxnbDoneUs = 0;
  telemetryTxToDoneUs = 0;
  telemetryDoneToRxnbDoneUs = 0;
  telemetryTxToRxnbDoneUs = 0;
  telemetryRxnbToRxOkUs = 0;
  telemetryMissedAfterTx = 0;
  telemetryAwaitingRx = 0;
  telemetryStaleRxReported = 0;
  telemetryStaleRxLastAgeUs = 0;
  telemetryTxNonce = diagNonce;
  telemetryTxFhss = diagFhss;
#endif
  if (Radio.HasPendingOutputPower() &&
      !shouldDeferPowerCommitForTelemetrySlot()) {
    Radio.SetTxIdleMode();
    Radio.CommitOutputPowerForNextTx();
  }

  const SX12XX_Radio_Number_t transmittingRadio =
      isRxTelemetryForcedOff() ? SX12XX_Radio_NONE : SX12XX_Radio_All;

  if (sendGeminiBuffer && otaPktGemini) {
    const bool swapGemini =
        ExpressLRS_currAirRate_Modparams &&
        (ExpressLRS_currAirRate_Modparams->FHSShopInterval != 0) &&
        (((diagNonce / ExpressLRS_currAirRate_Modparams->FHSShopInterval) %
          2U) != 0U) &&
        !FHSSuseDualBand;
    if (swapGemini) {
      Radio.TXnb((uint8_t *)otaPktGemini, true, (uint8_t *)otaPkt,
                 transmittingRadio);
    } else {
      Radio.TXnb((uint8_t *)otaPkt, true, (uint8_t *)otaPktGemini,
                 transmittingRadio);
    }
  } else {
    Radio.TXnb((uint8_t *)otaPkt, false, nullptr, transmittingRadio);
  }

  if (transmittingRadio == SX12XX_Radio_NONE) {
    TXdoneISR();
  }
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  updateHotpathMax(telemetrySendMaxUs, micros() - sendStartUs);
#endif
}

#if SIW917_ELRS_DEFER_TLM_TX_FROM_TIMER
static bool ICACHE_RAM_ATTR TelemetryTxQueueHasRoom() {
  const uint32_t primask = telemetryEnterCritical();
  const bool hasRoom = !deferredTelemetryPending;
  telemetryExitCritical(primask);
  return hasRoom;
}

static bool ICACHE_RAM_ATTR QueueTelemetryPacketForTask(
    OTA_Packet_s *otaPkt, OTA_Packet_s *otaPktGemini,
    bool sendGeminiBuffer) {
  bool queued = false;
  const uint32_t primask = telemetryEnterCritical();
  if (!deferredTelemetryPending) {
    memcpy(&deferredTelemetryPacket, otaPkt, sizeof(deferredTelemetryPacket));
    deferredTelemetrySendGemini = sendGeminiBuffer;
    if (sendGeminiBuffer && otaPktGemini) {
      memcpy(&deferredTelemetryGeminiPacket, otaPktGemini,
             sizeof(deferredTelemetryGeminiPacket));
    }
    deferredTelemetryNonce = OtaNonce;
    deferredTelemetryFhss = FHSSgetCurrIndex();
    deferredTelemetryPending = true;
    telemetryDeferredQueueCount++;
    queued = true;
  }
  telemetryExitCritical(primask);

  if (!queued) {
    telemetryDeferredDropCount++;
    return false;
  }

  elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_TIMER);
  return true;
}

static void ServiceDeferredTelemetryTx() {
  WORD_ALIGNED_ATTR OTA_Packet_s packet = {};
  WORD_ALIGNED_ATTR OTA_Packet_s geminiPacket = {};
  bool sendGeminiBuffer = false;
  uint8_t diagNonce = 0;
  uint8_t diagFhss = 0;

  const uint32_t primask = telemetryEnterCritical();
  if (!deferredTelemetryPending) {
    telemetryExitCritical(primask);
    return;
  }

  memcpy(&packet, &deferredTelemetryPacket, sizeof(packet));
  sendGeminiBuffer = deferredTelemetrySendGemini;
  if (sendGeminiBuffer) {
    memcpy(&geminiPacket, &deferredTelemetryGeminiPacket, sizeof(geminiPacket));
  }
  diagNonce = deferredTelemetryNonce;
  diagFhss = deferredTelemetryFhss;
  deferredTelemetryPending = false;
  telemetryExitCritical(primask);

  SendTelemetryPacket(&packet, sendGeminiBuffer ? &geminiPacket : nullptr,
                      sendGeminiBuffer, diagNonce, diagFhss);
  telemetryDeferredSendCount++;
  telemetryTxCount++;
}

static bool ICACHE_RAM_ATTR DispatchTelemetryPacket(
    OTA_Packet_s *otaPkt, OTA_Packet_s *otaPktGemini,
    bool sendGeminiBuffer) {
  return QueueTelemetryPacketForTask(otaPkt, otaPktGemini, sendGeminiBuffer);
}
#else
static bool ICACHE_RAM_ATTR TelemetryTxQueueHasRoom() { return true; }
static void ServiceDeferredTelemetryTx() {}
static bool ICACHE_RAM_ATTR DispatchTelemetryPacket(
    OTA_Packet_s *otaPkt, OTA_Packet_s *otaPktGemini,
    bool sendGeminiBuffer) {
  SendTelemetryPacket(otaPkt, otaPktGemini, sendGeminiBuffer);
  return true;
}
#endif

#if SIW917_ELRS_PREBUILD_TLM_PACKET
static void ICACHE_RAM_ATTR InvalidatePrebuiltTelemetry() {
  prebuiltTelemetryValid = false;
}

static void ICACHE_RAM_ATTR PrepareTelemetryForNextTock() {
  const uint8_t targetNonce = (uint8_t)(OtaNonce + 1U);

  if (!isTelemetrySlotForNonce(targetNonce)) {
    prebuiltTelemetryValid = false;
    return;
  }

  if (prebuiltTelemetryValid && prebuiltTelemetryNonce == targetNonce) {
    return;
  }

  if (BuildTelemetryPacket(&prebuiltTelemetryPacket,
                           &prebuiltTelemetryGeminiPacket,
                           &prebuiltTelemetrySendGemini,
                           &prebuiltTelemetryPayload, targetNonce,
                           &prebuiltNextTelemetryType,
                           &prebuiltTelemetryBurstCount)) {
    prebuiltTelemetryNonce = targetNonce;
    prebuiltTelemetryValid = true;
    telemetryPrebuildCount++;
  }
}
#else
static void ICACHE_RAM_ATTR InvalidatePrebuiltTelemetry() {}
static void ICACHE_RAM_ATTR PrepareTelemetryForNextTock() {}
#endif

static bool SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR HandleSendDataDl() {
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  const uint32_t handleStartUs = micros();
#endif
  if (!isTelemetrySlotForNonce(OtaNonce)) {
    return false;
  }

#if ELRS_DIAG_DISABLE_DOWNLINK_TLM
  // Isolation test: reserve the telemetry slot without keying the LR1121 TX.
  // This preserves LQ timing while proving whether TX/TX_DONE/RX return causes
  // the post-connect packet loss.
  alreadyTLMresp = true;
  telemetrySuppressedCount++;
  return false;
#endif

  if (!TelemetryTxQueueHasRoom()) {
    telemetrySuppressedCount++;
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
    updateHotpathMax(telemetryHandleMaxUs, micros() - handleStartUs);
#endif
    return false;
  }

#if SIW917_ELRS_PREBUILD_TLM_PACKET
  if (prebuiltTelemetryValid && prebuiltTelemetryNonce == OtaNonce) {
    prebuiltTelemetryValid = false;
    if (TelemetrySender.CommitPreparedPayload(prebuiltTelemetryPayload)) {
      alreadyTLMresp = true;
      NextTelemetryType = prebuiltNextTelemetryType;
      telemetryBurstCount = prebuiltTelemetryBurstCount;
      telemetryPrebuildHitCount++;
      if (DispatchTelemetryPacket(
              &prebuiltTelemetryPacket,
              prebuiltTelemetrySendGemini ? &prebuiltTelemetryGeminiPacket
                                          : nullptr,
              prebuiltTelemetrySendGemini)) {
        return true;
      }
      telemetrySuppressedCount++;
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
      updateHotpathMax(telemetryHandleMaxUs, micros() - handleStartUs);
#endif
      return false;
    }

    telemetryPrebuildMissCount++;
  }
#endif

  WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {};
  WORD_ALIGNED_ATTR OTA_Packet_s otaPktGemini = {};
  bool sendGeminiBuffer = false;
  StubbornSenderPreparedPayload preparedPayload = {};
  uint8_t nextTelemetryType = NextTelemetryType;
  uint8_t nextTelemetryBurstCount = telemetryBurstCount;

#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  const uint32_t buildStartUs = micros();
#endif
  if (!BuildTelemetryPacket(&otaPkt, &otaPktGemini, &sendGeminiBuffer,
                            &preparedPayload, OtaNonce, &nextTelemetryType,
                            &nextTelemetryBurstCount) ||
      !TelemetrySender.CommitPreparedPayload(preparedPayload)) {
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
    updateHotpathMax(telemetryBuildMaxUs, micros() - buildStartUs);
    updateHotpathMax(telemetryHandleMaxUs, micros() - handleStartUs);
#endif
    return false;
  }
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  updateHotpathMax(telemetryBuildMaxUs, micros() - buildStartUs);
#endif

  alreadyTLMresp = true;
  NextTelemetryType = nextTelemetryType;
  telemetryBurstCount = nextTelemetryBurstCount;
  if (!DispatchTelemetryPacket(&otaPkt, sendGeminiBuffer ? &otaPktGemini : nullptr,
                               sendGeminiBuffer)) {
    telemetrySuppressedCount++;
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
    updateHotpathMax(telemetryHandleMaxUs, micros() - handleStartUs);
#endif
    return false;
  }
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  updateHotpathMax(telemetryHandleMaxUs, micros() - handleStartUs);
#endif
  return true;
}

static void updateTelemetryBurst() {
  if (telemBurstValid || (ExpressLRS_currAirRate_Modparams == nullptr) ||
      (ExpressLRS_currTlmDenom == 0)) {
    return;
  }

  telemBurstValid = true;
  uint16_t hz = 1000000 / ExpressLRS_currAirRate_Modparams->interval;
  telemetryBurstMax = TLMBurstMaxForRateRatio(hz, ExpressLRS_currTlmDenom);
  TelemetrySender.UpdateTelemetryRate(hz, ExpressLRS_currTlmDenom,
                                      telemetryBurstMax);
}

static void updateSwitchMode() {
  if ((SwitchModePending <= 0) || (ExpressLRS_currAirRate_Modparams == nullptr)) {
    return;
  }

  OtaUpdateSerializers((OtaSwitchMode_e)(SwitchModePending - 1),
                       ExpressLRS_currAirRate_Modparams->PayloadLength);
  SwitchModePending = 0;
}

static void DataUlReceiveComplete() {
  dataUlHandledCount++;
  dataUlLastFrameType = 0;
  dataUlLastFrameLen = 0;
#if ELRS_DIAG_RX_LUA_UL
  const uint32_t completeCount = ++rxLuaUlCompleteCount;
#if ELRS_DIAG_RX_LUA_UL_VERBOSE
  if (rxLuaShouldPrintDiag(completeCount)) {
    DBGLN("[RX_LUA] UL complete #%lu first:%02X %02X %02X %02X %02X %02X %02X %02X",
          (unsigned long)completeCount, DataUlBuffer[0], DataUlBuffer[1],
          DataUlBuffer[2], DataUlBuffer[3], DataUlBuffer[4], DataUlBuffer[5],
          DataUlBuffer[6], DataUlBuffer[7]);
  }
#endif
#endif

  switch (DataUlBuffer[0]) {
  case MSP_ELRS_SET_AIR_PROTOCOL:
  case MSP_ELRS_RXTX_CONFIG:
    {
      uint8_t n = DataUlReceiver.GetReceivedLength();
      if (n < 4) {
        n = 4;
      }
      mlrs_ota_handle_msp(DataUlBuffer, n);
    }
    break;
  case MSP_ELRS_SET_RX_WIFI_MODE:
    elrs_cpp_request_wifi_mode();
    break;
  case MSP_ELRS_BIND:
    if (InBindingMode) {
      OnELRSBindMSP(&DataUlBuffer[1]);
    }
    break;
  case MSP_ELRS_MAVLINK_TLM:
    if (TxOtaProtocol == TX_MAVLINK_MODE) {
      uint32_t mavlinkLen = DataUlBuffer[1];
      if ((mavlinkLen > 0) &&
          ((mavlinkLen + CRSF_FRAME_NOT_COUNTED_BYTES) <= sizeof(DataUlBuffer)) &&
          crsf_serial_is_ready()) {
        (void)crsf_serial_send_frame(&DataUlBuffer[CRSF_FRAME_NOT_COUNTED_BYTES],
                                     mavlinkLen);
      }
    }
    break;
  default:
    {
      const crsf_header_t *receivedHeader =
          reinterpret_cast<const crsf_header_t *>(DataUlBuffer);
      const uint32_t frameLen =
          receivedHeader->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES;
      dataUlLastFrameType = receivedHeader->type;
      dataUlLastFrameLen =
          frameLen > UINT8_MAX ? UINT8_MAX : (uint8_t)frameLen;
      if (frameLen >= CRSF_MIN_PACKET_LEN && frameLen <= CRSF_MAX_PACKET_LEN) {
        switch (receivedHeader->type) {
        case CRSF_FRAMETYPE_DEVICE_PING:
        case CRSF_FRAMETYPE_PARAMETER_READ:
        case CRSF_FRAMETYPE_PARAMETER_WRITE:
        case CRSF_FRAMETYPE_COMMAND:
#if ELRS_DIAG_CRSF_OTA_VERBOSE
        {
          const auto *extHeader =
              reinterpret_cast<const crsf_ext_header_t *>(receivedHeader);
          const uint8_t *payload =
              reinterpret_cast<const uint8_t *>(receivedHeader) +
              sizeof(crsf_ext_header_t);
          const uint8_t payloadLen =
              receivedHeader->frame_size >= CRSF_FRAME_LENGTH_EXT_TYPE_CRC
                  ? (uint8_t)(receivedHeader->frame_size -
                              CRSF_FRAME_LENGTH_EXT_TYPE_CRC)
                  : 0;
          DBGLN("[DATA_UL] CRSF type=0x%02X len=%lu dest=0x%02X orig=0x%02X p0=%u p1=%u",
                receivedHeader->type, (unsigned long)frameLen,
                extHeader->dest_addr, extHeader->orig_addr,
                payloadLen > 0 ? payload[0] : 0,
                payloadLen > 1 ? payload[1] : 0);
        }
#endif
#if ELRS_DIAG_LUA_DISCOVERY
        if (receivedHeader->type == CRSF_FRAMETYPE_DEVICE_PING) {
          const auto *extHeader =
              reinterpret_cast<const crsf_ext_header_t *>(receivedHeader);
          const uint32_t pingCount = ++luaDiscoveryPingCount;
          DBGLN("LUA_DISC ping #%lu len=%lu dest=0x%02X orig=0x%02X den=%u active=%u",
                (unsigned long)pingCount, (unsigned long)frameLen,
                extHeader->dest_addr, extHeader->orig_addr,
                ExpressLRS_currTlmDenom, TelemetrySender.IsActive() ? 1 : 0);
        }
#endif
          // Lua/device-management frames are still CRSF in MAVLink OTA mode.
          crsfRouter.processMessage(&otaConnector, receivedHeader);
          break;
        default:
          if (TxOtaProtocol == TX_NORMAL_MODE && crsf_serial_is_ready() &&
              configuredSerialProtocolUsesCrsf()) {
            (void)crsf_serial_send_frame(DataUlBuffer, frameLen);
          }
          break;
        }
      } else {
        dataUlMalformedCount++;
        DBGLN("[DATA_UL] malformed CRSF len=%lu first=%02X %02X %02X %02X",
              (unsigned long)frameLen, DataUlBuffer[0], DataUlBuffer[1],
              DataUlBuffer[2], DataUlBuffer[3]);
      }
    }
    break;
  }

  DataUlReceiver.Unlock();
  dataUlReady = false;
}

void mlrs_elrs_rx_accept_uplink(const uint8_t *payload, uint8_t len) {
  if (payload == nullptr || len == 0) {
    return;
  }
  if ((payload[0] == 0xFD || payload[0] == 0xFE) && crsf_serial_is_ready()) {
    (void)crsf_serial_send_frame(payload, len);
    return;
  }
  if (len > ELRS_DATA_UL_BUFFER) {
    len = ELRS_DATA_UL_BUFFER;
  }
  memcpy(DataUlBuffer, payload, len);
  dataUlReady = true;
}

void mlrs_elrs_rx_write_serial(const uint8_t *payload, uint8_t len) {
  if (payload == nullptr || len == 0 || !crsf_serial_is_ready()) {
    return;
  }
  (void)crsf_serial_send_frame(payload, len);
}

uint8_t mlrs_elrs_rx_take_downlink(uint8_t *payload, uint8_t maxLen) {
  if (payload == nullptr || maxLen == 0) {
    return 0;
  }
  uint8_t nextPayloadSize = 0;
  if (otaConnector.GetNextPayload(&nextPayloadSize, payload) &&
      nextPayloadSize != 0) {
    if (nextPayloadSize > maxLen) {
      nextPayloadSize = maxLen;
    }
    return nextPayloadSize;
  }
  return 0;
}

uint8_t mlrs_elrs_rx_take_serial(uint8_t *payload, uint8_t maxLen) {
  if (payload == nullptr || maxLen == 0 || !crsf_serial_is_ready()) {
    return 0;
  }
  const uint32_t copied = crsf_serial_read(payload, maxLen);
  return copied > UINT8_MAX ? UINT8_MAX : (uint8_t)copied;
}

static uint16_t crsf_chan_to_us(uint32_t crsf) {
  if (crsf < 172U) {
    crsf = 172U;
  }
  if (crsf > 1811U) {
    crsf = 1811U;
  }
  return (uint16_t)(988U + ((crsf - 172U) * 1024U) / 1639U);
}

static uint16_t mavlink1_crc(const uint8_t *buf, uint8_t len, uint8_t extra) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; ++i) {
    uint8_t tmp = buf[i] ^ (uint8_t)(crc & 0xFF);
    tmp ^= (uint8_t)(tmp << 4);
    crc = (uint16_t)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
  }
  uint8_t tmp = extra ^ (uint8_t)(crc & 0xFF);
  tmp ^= (uint8_t)(tmp << 4);
  crc = (uint16_t)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
  return crc;
}

static void sendMavlinkRcOverride() {
  if (!crsf_serial_is_ready()) {
    return;
  }
  uint8_t buf[26];
  static uint8_t seq;
  buf[0] = 0xFE;
  buf[1] = 18;
  buf[2] = seq++;
  buf[3] = 255;
  buf[4] = 190;
  buf[5] = 70;
  for (uint8_t i = 0; i < 8; ++i) {
    const uint16_t us = crsf_chan_to_us(ChannelData[i]);
    buf[6U + (i * 2U)] = (uint8_t)(us & 0xFF);
    buf[7U + (i * 2U)] = (uint8_t)(us >> 8);
  }
  const uint16_t crc = mavlink1_crc(&buf[1], 23, 124);
  buf[24] = (uint8_t)(crc & 0xFF);
  buf[25] = (uint8_t)(crc >> 8);
  (void)crsf_serial_send_frame(buf, sizeof(buf));
}

static void serviceMlrsHostBridge(unsigned long now) {
  if (mlrs_ota_tlm_busy()) {
    return;
  }
  if (updateActiveSerialProtocol() ||
      appliedSerialProtocol != ELRS_SERIAL_MAVLINK) {
    applyConfiguredSerialProtocol();
  }
  if (dataUlReady) {
    DataUlReceiveComplete();
  }
  updateSerialRxState();
  crsfReceiver.processPending(!otaConnector.IsEmpty());

  static uint32_t lastRcOverrideMs = 0;
  if ((now - lastRcOverrideMs) >= 20U) {
    lastRcOverrideMs = now;
    sendMavlinkRcOverride();
  }
}

//=============================================================================
// Rate cycling for connection scanning
//=============================================================================
static bool rateCyclingStarted = false;

static void cycleRfMode() {
  if (LockRFmode || InBindingMode)
    return;

  uint32_t now = millis();

  const uint32_t requestedDwellMs = cycleInterval * RFmodeCycleMultiplier;
  const uint32_t dwellMs =
      requestedDwellMs < SIW917_ELRS_SCAN_MIN_DWELL_MS
          ? SIW917_ELRS_SCAN_MIN_DWELL_MS
          : requestedDwellMs;

  if ((now - RFmodeLastCycled) > dwellMs) {
    RFmodeLastCycled = now;
    LastSyncPacket = now;

    const uint8_t currentScanIndex = scanIndex % RATE_MAX;
#if ELRS_DIAG_RF_RATE_LOG
    DBGLN("cycleRfMode begin: scan=%u interval=%lu dwell=%lu multiplier=%u",
          currentScanIndex, (unsigned long)cycleInterval,
          (unsigned long)dwellMs, (unsigned)RFmodeCycleMultiplier);
#endif
    SetRFLinkRate(currentScanIndex, false);
    LQCalc.reset100();

    do {
      scanIndex = (scanIndex + 1) % RATE_MAX;
    } while (!isSupportedRFRate(scanIndex));

    Radio.RXnb();
#if ELRS_DIAG_RF_RATE_LOG
    DBGLN("cycleRfMode RX re-armed");
#endif
    RFmodeCycleMultiplier = 1;

#if ELRS_DIAG_RF_RATE_LOG
    DBGLN("Cycling to rate index %d", currentScanIndex);
#endif

    // Enable per-iteration diagnostics in elrs_loop to catch hang
    rateCyclingStarted = true;
  }
}

static void updateBindingMode(unsigned long now) {
  static uint32_t bindingRateChangeMs = 0;

  if (bindingModeRequest) {
    bindingModeRequest = false;
    DBGLN("Binding mode request pending - entering safely");
    enterBindingModeNow();
    return;
  }

  if (!InBindingMode || !ExpressLRS_currAirRate_Modparams) {
    return;
  }

  if (use2G4Domain()) {
    return;
  }

  if ((now - bindingRateChangeMs) > BindingRateChangeCyclePeriodMs) {
    bindingRateChangeMs = now;

    uint8_t bindingIndex = enumRatetoIndex(RATE_BINDING);
    uint8_t dualBandBindingIndex = enumRatetoIndex(RATE_DUALBAND_BINDING);

    if (ExpressLRS_currAirRate_Modparams->enum_rate == RATE_DUALBAND_BINDING) {
      SetRFLinkRate(bindingIndex, true);
    } else {
      SetRFLinkRate(dualBandBindingIndex, true);
    }

    Radio.RXnb();
  }
}

//=============================================================================
// Bind Button Callback
//=============================================================================

// External function to request WiFi mode (defined in gspi_example.c)
// This sets a flag that's checked in the main task loop
extern "C" void elrs_cpp_request_wifi_mode(void);

/**
 * @brief Callback when bind button is pressed/released
 * @param long_press true if button was held >= 3 seconds
 *
 * Button behavior:
 *   Short press: Request WiFi configuration mode (handled in main task)
 *   Long press (3+ seconds): Toggle binding mode
 *
 * Note: WiFi mode must be started from main task context (not callback)
 * to ensure osDelay() works properly. This matches the working C
 * implementation.
 */
static void onBindButtonEvent(bool long_press) {
  if (long_press) {
    // Long press = toggle binding mode
    DBGLN("Bind button long press - toggling binding mode");
    if (!InBindingMode) {
      elrs_enter_binding_mode();
      status_led_set_mode(LED_MODE_BINDING);
    } else {
      elrs_exit_binding_mode();
      status_led_set_mode(LED_MODE_DISCONNECTED);
    }
  } else {
    // Short press = request WiFi mode (handled in main task loop)
    DBGLN("Bind button short press - requesting WiFi mode");
    if (!InWiFiMode) {
      // Set flag - main task will handle starting WiFi
      elrs_cpp_request_wifi_mode();
    }
  }
}

static void enterBindingModeNow() {
  if (InBindingMode) {
    return;
  }

  bindingModeRequest = false;

  if (getConfiguredBindStorage() == ELRS_BIND_STORAGE_ADMINISTERED) {
    DBGLN("Binding mode blocked - Bind Storage is Administered");
    return;
  }

  if ((connectionState != disconnected) || hwTimer::isRunning()) {
    lastDisconnectReason = DISC_EXTERNAL;
    LostConnection(false);
  }

  if (returnLoanIfNeeded(false)) {
    (void)elrs_config_save();
  }

  TelemetrySender.ResetState();
  InvalidatePrebuiltTelemetry();
  DataUlReceiver.ResetState();
  dataUlReady = false;
  alreadyTLMresp = false;
  connectionHasModelMatch = false;
  resetTeamraceModelMatch();
  lastConnectionHadModelMatch = false;

  OtaCrcInitializer = OTA_VERSION_ID;
  OtaNonce = 0;
  InBindingMode = true;
  setConnectionState(disconnected);
  RXtimerState = tim_disconnected;
  scanIndex = getStartupOrBindingRateIndex();
  ExpressLRS_nextAirRateIndex = scanIndex;
  SetRFLinkRate(scanIndex, true);
  Radio.RXnb();
  LastValidPacket = millis();
  LastSyncPacket = LastValidPacket;
  RFmodeLastCycled = LastValidPacket;
  status_led_set_mode(LED_MODE_BINDING);
  DBGLN("Entering binding mode");
}

//=============================================================================
// Public API implementation
//=============================================================================

extern "C" {

bool elrs_init(void) {
  DBGLN("ELRS RX init starting...");

  // Initialize status LEDs first for visual feedback
  status_led_init();
  status_led_set_mode(LED_MODE_DISCONNECTED);

  // Initialize bind button
  bind_button_init();
  bind_button_set_callback(onBindButtonEvent);

  // Initialize persistent config from NVM3 (must be before options_init)
  // Note: sl_net_init() must be called before this for NVM3 access
  if (elrs_config_init() != 0) {
    DBGLN("WARNING: Config init failed, using defaults");
  }

  // Initialize options/configuration (loads UID from elrs_config)
  if (!options_init()) {
    DBGLN("Options init failed!");
    return false;
  }

  // Runtime UID comes from config; firmwareOptions.uid remains the flashed UID
  // so Returnable Bind Storage can detect and return loaned receivers.
  memset(UID, 0, UID_LEN);
  if (elrs_config_is_bound()) {
    (void)elrs_config_get_uid(UID);
  }
  DBGLN("UID: %02X:%02X:%02X:%02X:%02X:%02X", UID[0], UID[1], UID[2], UID[3],
        UID[4], UID[5]);
  const bool startInVolatileBindMode =
      getConfiguredBindStorage() == ELRS_BIND_STORAGE_VOLATILE &&
      !uidIsBound(UID);
  if (startInVolatileBindMode) {
    DBGLN("Bind Storage Volatile - starting in bind mode");
  }

  // Initialize channel data
  ChannelDataReset();
  memset(DataUlBuffer, 0, sizeof(DataUlBuffer));
  memset(TelemetryBuffer, 0, sizeof(TelemetryBuffer));
  rxLuaQueueHead = 0;
  rxLuaQueueTail = 0;
  rxLuaQueueCount = 0;
  rxLuaWifiPending = false;
  rxLuaBindPending = false;
  rxLuaWifiCommandStep = RX_LUA_CMD_IDLE;
  rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
  rxLuaWifiCommandInfo = "";
  rxLuaBindCommandInfo = "";
  rxLuaUlChunkCount = 0;
  rxLuaUlCompleteCount = 0;
  rxLuaCrsfHandledCount = 0;
  rxLuaCrsfRejectCount = 0;
  rxLuaCrsfIgnoredCount = 0;
  rxLuaLastUlPackageIndex = 0;
  rxLuaQueueDropCount = 0;
  rxLuaConfigSavePending = false;
  rxLuaSerialApplyPending = false;
  rxLuaConfigSaveAtMs = 0;
#if ELRS_DIAG_RX_LUA_LOAD_SNAPSHOT
  rxLuaDiagSnapshotPending = false;
  rxLuaBindPostDiagPending = false;
  rxLuaDiagLastSnapshotMs = 0;
#endif
  DataUlReceiver.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
  DataUlReceiver.SetDataToReceive(DataUlBuffer, sizeof(DataUlBuffer));

  // The SiWx917 startup path does not reliably run C++ global constructors for
  // these CRSF objects. Construct them explicitly before routing Lua traffic so
  // the router CRC table, endpoint base device id, and virtual tables are valid.
  new (&crsfRouter) CRSFRouter();
  new (&otaConnector) RXOTAConnector();
  new (&crsfReceiver) SiW917RXEndpoint();
  DBGLN("[CRSF_INIT] rx endpoint dev=0x%02X", crsfReceiver.getDeviceId());
  crsfRouter.addEndpoint(&crsfReceiver);
  crsfRouter.addConnector(&otaConnector);
  crsfReceiver.updateParameters();

  // NOTE: Do NOT call radioHal.init() here!
  // Radio.Begin() below calls hal.init() which does the full init sequence.
  // Calling init() twice causes sl_si91x_gspi_init() to reinitialize the
  // GSPI, corrupting DMA channel state and causing ARM_DRIVER_ERROR on all
  // subsequent SPI transfers.

  // Initialize OTA CRC
  OtaUpdateCrcInitFromUid();

  // Initialize FHSS
  FHSSrandomiseFHSSsequence(uidMacSeedGet());
  DBGLN("FHSS initialized with %d channels", FHSSgetChannelCount());
#if defined(RADIO_LR1121) && SIW917_ELRS_SCAN_ALL_LR1121_RATES
#if SIW917_ELRS_ENABLE_CROSSBAND_RATES
  DBGLN("LR1121 all-band scan enabled (sub-GHz + 2.4GHz + crossband)");
#else
  DBGLN("LR1121 single-radio scan enabled (sub-GHz + 2.4GHz)");
#endif
#endif

  // Initialize radio with frequency range from FHSS config
  bool radioOk = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
  if (!radioOk) {
    DBGLN("Radio init failed!");
    connectionState = radioFailed;
    return false;
  }

  // Set up radio callbacks
  Radio.RXdoneCallback = RXdoneISR;
  Radio.TXdoneCallback = TXdoneISR;
  updateRxDownlinkPower(true);

  // CRITICAL: Bind the hardware interrupt callbacks to the HAL
  // Without this, the C++ HAL explicitly ignores the hardware DIO interrupts!
  if (LR1121Hal::instance) {
    LR1121Hal::instance->IsrCallback_1 = LR1121Driver::IsrCallback_1;
    LR1121Hal::instance->IsrCallback_2 = LR1121Driver::IsrCallback_2;
  }

  hwTimer::init(HWtimerCallbackTick, HWtimerCallbackTock);

  RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow / 2;

  if (startInVolatileBindMode) {
    enterBindingModeNow();
  } else {
    // Start on the configured rate, matching upstream's scan-start behavior.
    scanIndex = getStartupOrBindingRateIndex();
    SetRFLinkRate(scanIndex, false);

    // Start receiving
    Radio.RXnb();
    DBGLN("Radio.RXnb() called - LR1121 should be in continuous RX mode");

    // Record start time for rate cycling
    RFmodeLastCycled = millis();
  }

  activeSerialProtocol = getDesiredActiveSerialProtocol();
  applyConfiguredSerialProtocol();

  // Load model match ID from config (0xFF = disabled)
  elrs_config_t *cfg = elrs_config_get();
  if (cfg) {
    modelMatchId = cfg->model_id;
    siw917_rx_set_force_telemetry_off(cfg->force_tlm);
    DBGLN("Model match ID: %d (%s)", modelMatchId,
          modelMatchId == 0xFF ? "disabled" : "enabled");
  }

  DBGLN("ELRS RX init complete");
  mlrs_ota_on_elrs_ready();
  return true;
}

void elrs_loop(void) {
  // CRITICAL: Process deferred DIO1 interrupts in main-loop context.
  // The ISR only sets a flag (no SPI). We process it here where SPI is safe.
  LR1121Hal::handleDeferredISR();
  if (mlrs_ota_is_active()) {
    mlrs_ota_rx_send_slot();
    hwTimer::service();
  }
  mlrs_ota_loop();
  if (mlrs_ota_is_active()) {
    hwTimer::service();
    status_led_update();
    bind_button_poll();
    /* Finish downlink TX (and hop back to RX) before Lua/CRSF/UART. Those
     * handlers were blocking TXdone past the watchdog and dropping Lua
     * replies, which made the handset retry the whole dump. */
    if (mlrs_ota_tlm_busy()) {
      const uint32_t t0 = millis();
      const int32_t wait_ms =
          (int32_t)mlrs_ota_tlm_busy_timeout_ms();
      while (mlrs_ota_tlm_busy() &&
             (int32_t)(millis() - t0) < wait_ms) {
        LR1121Hal::handleDeferredISR();
        mlrs_ota_rx_send_slot();
      }
    }
    LR1121Hal::handleDeferredISR();
    mlrs_ota_rx_send_slot();
    mlrs_ota_loop();
    serviceMlrsHostBridge(millis());
    return;
  }
  ServiceDeferredTelemetryTx();
  maybeReportStaleTelemetryRx();
  PrepareTelemetryForNextTock();
  hwTimer::service();
  ServiceDeferredTelemetryTx();

  unsigned long now = millis();

  if (do_packet_dump) {
    do_packet_dump = false;
    dump_capture_buffer();
  }

#if ELRS_DIAG_PERIODIC_STATS || ELRS_DIAG_PRINT_AFTER_LOSS
#if ELRS_DIAG_PERIODIC_STATS
  static uint32_t lastDiag = 0;
#endif
  bool shouldPrintDiag = false;
#if ELRS_DIAG_PRINT_AFTER_LOSS
  if (diagPrintAfterLossPending && connectionState == disconnected) {
    diagPrintAfterLossPending = false;
    shouldPrintDiag = true;
  }
#endif
#if ELRS_DIAG_PERIODIC_STATS
  if (!shouldPrintDiag) {
    const bool shouldPrintPeriodicDiag =
#if ELRS_DIAG_PERIODIC_STATS_WHEN_CONNECTED
        true;
#else
        connectionState != connected;
#endif
    shouldPrintDiag = shouldPrintPeriodicDiag && millis() - lastDiag > 2000;
  }
#endif
  if (shouldPrintDiag) {
#if ELRS_DIAG_PERIODIC_STATS
    lastDiag = millis();
#endif

    uint32_t isrCount = 0;
    uint32_t rxIrqCount = 0;
    uint32_t txIrqCount = 0;
    uint32_t otherIrqCount = 0;
    uint32_t lastIrq = 0;
    uint32_t radio1IsrCount = 0;
    uint32_t radio2IsrCount = 0;
    uint32_t radio1RxIrqCount = 0;
    uint32_t radio2RxIrqCount = 0;
    lr1121_get_isr_stats(&isrCount, &rxIrqCount, &txIrqCount, &otherIrqCount,
                         &lastIrq);
    lr1121_get_radio_isr_stats(&radio1IsrCount, &radio2IsrCount,
                               &radio1RxIrqCount, &radio2RxIrqCount);
    int8_t instantRssi = 0;
    if (isrCount == 0 && connectionState == disconnected) {
      uint8_t stat1 = 0;
      uint8_t stat2 = 0;
      uint8_t irq = 0;
      lastRadioStatusOk = lr1121_get_status(&stat1, &stat2, &irq) ? 1 : 0;
      lastRadioStat1 = stat1;
      lastRadioStat2 = stat2;
      lastRadioIrqByte = irq;
    }
    if (connectionState == disconnected) {
      Radio.StartRssiInst(SX12XX_Radio_1);
      instantRssi = Radio.GetRssiInst(SX12XX_Radio_1);
    }
    const int dio1Level = lr1121_dio1_read();
    const uint32_t dio1Edges = lr1121_dio1_get_isr_count();
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
    const int dio2Level = lr1121_dio2_read();
    const uint32_t dio2Edges = lr1121_dio2_get_isr_count();
#else
    const int dio2Level = 0;
    const uint32_t dio2Edges = 0;
#endif
    DBGLN("IRQ isr:%lu rx:%lu tx:%lu other:%lu cb:%lu/%lu rxr:%lu/%lu "
          "dio:%d/%d edge:%lu/%lu irq:0x%08lX "
          "rxok:%lu crcfail:%lu okT:%lu/%lu/%lu failT:%lu/%lu/%lu "
          "stat:%u/%02X/%02X/%02X irssi:%d",
          isrCount, rxIrqCount, txIrqCount, otherIrqCount,
          radio1IsrCount, radio2IsrCount, radio1RxIrqCount, radio2RxIrqCount,
          dio1Level, dio2Level, (unsigned long)dio1Edges,
          (unsigned long)dio2Edges, lastIrq, pkt_capture_count, crcFailCount,
          (unsigned long)crcPassTypeCount[PACKET_TYPE_RCDATA],
          (unsigned long)crcPassTypeCount[PACKET_TYPE_DATA],
          (unsigned long)crcPassTypeCount[PACKET_TYPE_SYNC],
          (unsigned long)crcFailTypeCount[PACKET_TYPE_RCDATA],
          (unsigned long)crcFailTypeCount[PACKET_TYPE_DATA],
          (unsigned long)crcFailTypeCount[PACKET_TYPE_SYNC],
          lastRadioStatusOk, lastRadioStat1, lastRadioStat2, lastRadioIrqByte,
          instantRssi);
    hwTimer::service();
    DBGLN("LINK conn:%d rxst:%d rate:%d freq:%lu age:%lu nonce:%u fhss:%u "
          "pfd:%ld/%ld/%ld/%ld ph:%ld fo:%ld lq:%u/%u rssi:%d snr:%d disc:%u",
          connectionState, RXtimerState,
          ExpressLRS_currAirRate_Modparams
              ? ExpressLRS_currAirRate_Modparams->index
              : 0,
          Radio.currFreq, (unsigned long)(now - LastValidPacket), OtaNonce,
          FHSSgetCurrIndex(),
          (long)pfdLastRawOffset, (long)pfdLastNormalizedOffset,
          (long)pfdLastOffset,
          (long)pfdLastOffsetDx, (long)pfdLastPhaseShift,
          (long)hwTimer::FreqOffset, LQCalc.getLQRaw(), LQCalc.getCount(),
          Radio.LastPacketRSSI, Radio.LastPacketSNRRaw,
          (unsigned)lastDisconnectReason);
    hwTimer::service();
#if ELRS_DIAG_CRC_NONCE_WINDOW > 0
    DBGLN("TIM lat:%lu e2loop:%lu loop2rx:%lu rx2pkt:%lu pkt2cb:%lu "
          "slack:%ld pfdsrc:%u pfdskip:%lu last:%u exp:%u/%u sync:%u/%u "
          "vf:%lu crcN:%d/%u/%u/%lu/%lu tlm:%lu/%lu den:%u "
          "lua:%lu/%lu/%lu/%lu/%lu/%u/%u "
          "tmr:%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu",
          (unsigned long)lastRxDoneLatencyUs,
          (unsigned long)lastDioToDeferredUs,
          (unsigned long)lastDeferredToRxIsrUs,
          (unsigned long)lastRxIsrToPacketUs,
          (unsigned long)lastPacketToCallbackUs, (long)lastPfdSlackUs,
          lastPfdUsedEdgeTimestamp,
          (unsigned long)pfdSkippedNoTimestampCount, lastValidPacketType,
          lastValidExpectedNonce, lastValidExpectedFhss, lastValidSyncNonce,
          lastValidSyncFhss, (unsigned long)lastValidFreq,
          (int)lastCrcNonceDelta, lastCrcNonceExpected,
          lastCrcNonceMatched, (unsigned long)crcNonceDiagHitCount,
          (unsigned long)crcNonceDiagMissCount,
          (unsigned long)telemetryTxCount,
          (unsigned long)telemetrySuppressedCount, ExpressLRS_currTlmDenom,
          (unsigned long)rxLuaUlChunkCount,
          (unsigned long)rxLuaUlCompleteCount,
          (unsigned long)rxLuaCrsfHandledCount,
          (unsigned long)rxLuaCrsfRejectCount,
          (unsigned long)rxLuaCrsfIgnoredCount, rxLuaQueueCount,
          rxLuaLastUlPackageIndex,
          (unsigned long)hwTimer::getHardwareHalfTicks(),
          (unsigned long)hwTimer::getHardwareCount(),
          (unsigned long)hwTimer::getHardwareMatch(),
          (unsigned long)hwTimer::getHardwareFreqHz(),
          (unsigned long)hwTimer::getQueuedTickCount(),
          (unsigned long)hwTimer::getQueuedTockCount(),
          (unsigned long)hwTimer::getProcessedTickCount(),
          (unsigned long)hwTimer::getProcessedTockCount(),
          (unsigned long)hwTimer::getQueueOverflowCount(),
          (unsigned long)hwTimer::getImmediateTockDeliveredCount());
#else
    DBGLN("TIM lat:%lu e2loop:%lu loop2rx:%lu rx2pkt:%lu pkt2cb:%lu "
          "slack:%ld pfdsrc:%u pfdskip:%lu last:%u exp:%u/%u sync:%u/%u "
          "vf:%lu tlm:%lu/%lu den:%u "
          "lua:%lu/%lu/%lu/%lu/%lu/%u/%u "
          "tmr:%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu",
          (unsigned long)lastRxDoneLatencyUs,
          (unsigned long)lastDioToDeferredUs,
          (unsigned long)lastDeferredToRxIsrUs,
          (unsigned long)lastRxIsrToPacketUs,
          (unsigned long)lastPacketToCallbackUs, (long)lastPfdSlackUs,
          lastPfdUsedEdgeTimestamp,
          (unsigned long)pfdSkippedNoTimestampCount, lastValidPacketType,
          lastValidExpectedNonce, lastValidExpectedFhss, lastValidSyncNonce,
          lastValidSyncFhss, (unsigned long)lastValidFreq,
          (unsigned long)telemetryTxCount,
          (unsigned long)telemetrySuppressedCount, ExpressLRS_currTlmDenom,
          (unsigned long)rxLuaUlChunkCount,
          (unsigned long)rxLuaUlCompleteCount,
          (unsigned long)rxLuaCrsfHandledCount,
          (unsigned long)rxLuaCrsfRejectCount,
          (unsigned long)rxLuaCrsfIgnoredCount, rxLuaQueueCount,
          rxLuaLastUlPackageIndex,
          (unsigned long)hwTimer::getHardwareHalfTicks(),
          (unsigned long)hwTimer::getHardwareCount(),
          (unsigned long)hwTimer::getHardwareMatch(),
          (unsigned long)hwTimer::getHardwareFreqHz(),
          (unsigned long)hwTimer::getQueuedTickCount(),
          (unsigned long)hwTimer::getQueuedTockCount(),
          (unsigned long)hwTimer::getProcessedTickCount(),
          (unsigned long)hwTimer::getProcessedTockCount(),
          (unsigned long)hwTimer::getQueueOverflowCount(),
          (unsigned long)hwTimer::getImmediateTockDeliveredCount());
#endif
#if ELRS_DIAG_TX_TURNAROUND
    DBGLN("TXTRN t2d:%lu d2rx:%lu tx2rx:%lu rxok:%lu miss:%lu "
          "pend:%u pre:%lu ok:%lu tx:%u/%u rx:%u/%u",
          (unsigned long)telemetryTxToDoneUs,
          (unsigned long)telemetryDoneToRxnbDoneUs,
          (unsigned long)telemetryTxToRxnbDoneUs,
          (unsigned long)telemetryRxnbToRxOkUs,
          (unsigned long)telemetryMissedAfterTx, telemetryAwaitingRx,
          (unsigned long)telemetryTxBeforeRx, (unsigned long)telemetryRxAfterTx,
          telemetryTxNonce, telemetryTxFhss, telemetryRxNonce,
          telemetryRxFhss);
#endif
  }
#endif

#if ELRS_DIAG_RX_LUA_LOAD_SNAPSHOT
  if (rxLuaDiagSnapshotPending &&
      (uint32_t)(now - rxLuaDiagSnapshotAtMs) < 0x80000000UL) {
    rxLuaDiagSnapshotPending = false;
    rxLuaDiagLastSnapshotMs = now;

    const uint8_t rateIndex = ExpressLRS_currAirRate_Modparams
                                  ? ExpressLRS_currAirRate_Modparams->index
                                  : 0;
#if ELRS_DIAG_TX_TURNAROUND
    DBGLN("LUA_DIAG age:%lu trig:0x%02X param:%u aux:%u conn:%d rxst:%d "
          "rate:%u den:%u burst:%u q:%u active:%u st:%u wait:%u/%u "
          "ulack:%lu/%u drop:%lu "
          "ul:%lu/%lu crsf:%lu/%lu/%lu "
          "tlm:%lu/%lu txrx:%lu/%lu/%lu pend:%u last:%u exp:%u/%u lq:%u/%u "
          "rssi:%d snr:%d",
          (unsigned long)(now - rxLuaDiagSnapshotStartMs),
          rxLuaDiagSnapshotTrigger, rxLuaDiagSnapshotParam,
          rxLuaDiagSnapshotAux, connectionState, RXtimerState, rateIndex,
          ExpressLRS_currTlmDenom, telemetryBurstMax, rxLuaQueueCount,
          TelemetrySender.IsActive() ? 1 : 0, TelemetrySender.GetState(),
          TelemetrySender.GetWaitCount(), TelemetrySender.GetMaxPacketsBeforeResync(),
          (unsigned long)(telemetryDataUlAckCount -
                          rxLuaDiagStartTelemetryDataUlAckCount),
          telemetryLastDataUlAck ? 1 : 0,
          (unsigned long)(rxLuaQueueDropCount - rxLuaDiagStartQueueDropCount),
          (unsigned long)(rxLuaUlChunkCount - rxLuaDiagStartUlChunkCount),
          (unsigned long)(rxLuaUlCompleteCount - rxLuaDiagStartUlCompleteCount),
          (unsigned long)(rxLuaCrsfHandledCount -
                          rxLuaDiagStartCrsfHandledCount),
          (unsigned long)(rxLuaCrsfRejectCount -
                          rxLuaDiagStartCrsfRejectCount),
          (unsigned long)(rxLuaCrsfIgnoredCount -
                          rxLuaDiagStartCrsfIgnoredCount),
          (unsigned long)(telemetryTxCount - rxLuaDiagStartTelemetryTxCount),
          (unsigned long)(telemetrySuppressedCount -
                          rxLuaDiagStartTelemetrySuppressedCount),
          (unsigned long)(telemetryRxAfterTx -
                          rxLuaDiagStartTelemetryRxAfterTx),
          (unsigned long)(telemetryMissedAfterTx -
                          rxLuaDiagStartTelemetryMissedAfterTx),
          (unsigned long)(telemetryTxBeforeRx -
                          rxLuaDiagStartTelemetryTxBeforeRx),
          telemetryAwaitingRx, lastValidPacketType, lastValidExpectedNonce,
          lastValidExpectedFhss, LQCalc.getLQRaw(), LQCalc.getCount(),
          Radio.LastPacketRSSI, Radio.LastPacketSNRRaw);
#else
    DBGLN("LUA_DIAG age:%lu trig:0x%02X param:%u aux:%u conn:%d rxst:%d "
          "rate:%u den:%u burst:%u q:%u active:%u st:%u wait:%u/%u "
          "ulack:%lu/%u drop:%lu "
          "ul:%lu/%lu crsf:%lu/%lu/%lu "
          "tlm:%lu/%lu last:%u exp:%u/%u lq:%u/%u rssi:%d snr:%d",
          (unsigned long)(now - rxLuaDiagSnapshotStartMs),
          rxLuaDiagSnapshotTrigger, rxLuaDiagSnapshotParam,
          rxLuaDiagSnapshotAux, connectionState, RXtimerState, rateIndex,
          ExpressLRS_currTlmDenom, telemetryBurstMax, rxLuaQueueCount,
          TelemetrySender.IsActive() ? 1 : 0, TelemetrySender.GetState(),
          TelemetrySender.GetWaitCount(), TelemetrySender.GetMaxPacketsBeforeResync(),
          (unsigned long)(telemetryDataUlAckCount -
                          rxLuaDiagStartTelemetryDataUlAckCount),
          telemetryLastDataUlAck ? 1 : 0,
          (unsigned long)(rxLuaQueueDropCount - rxLuaDiagStartQueueDropCount),
          (unsigned long)(rxLuaUlChunkCount - rxLuaDiagStartUlChunkCount),
          (unsigned long)(rxLuaUlCompleteCount - rxLuaDiagStartUlCompleteCount),
          (unsigned long)(rxLuaCrsfHandledCount -
                          rxLuaDiagStartCrsfHandledCount),
          (unsigned long)(rxLuaCrsfRejectCount -
                          rxLuaDiagStartCrsfRejectCount),
          (unsigned long)(rxLuaCrsfIgnoredCount -
                          rxLuaDiagStartCrsfIgnoredCount),
          (unsigned long)(telemetryTxCount - rxLuaDiagStartTelemetryTxCount),
          (unsigned long)(telemetrySuppressedCount -
                          rxLuaDiagStartTelemetrySuppressedCount),
          lastValidPacketType, lastValidExpectedNonce, lastValidExpectedFhss,
          LQCalc.getLQRaw(), LQCalc.getCount(), Radio.LastPacketRSSI,
          Radio.LastPacketSNRRaw);
#endif
  }

  if (rxLuaBindPostDiagPending &&
      (uint32_t)(now - rxLuaBindPostDiagAtMs) < 0x80000000UL) {
    rxLuaBindPostDiagPending = false;

    const uint8_t rateIndex = ExpressLRS_currAirRate_Modparams
                                  ? ExpressLRS_currAirRate_Modparams->index
                                  : 0;
#if ELRS_DIAG_TX_TURNAROUND
    DBGLN("LUA_BIND_POST age:%lu chunk:%u conn:%d rxst:%d rate:%u den:%u "
          "burst:%u q:%u active:%u st:%u pkg:%u off:%u bytes:%u wait:%u/%u "
          "expack:%u ulack:%lu/%u drop:%lu ul:%lu/%lu crsf:%lu "
          "tlm:%lu/%lu txrx:%lu/%lu/%lu pend:%u last:%u exp:%u/%u "
          "lq:%u/%u rssi:%d snr:%d",
          (unsigned long)(now - rxLuaBindPostDiagStartMs),
          rxLuaBindPostDiagChunk, connectionState, RXtimerState, rateIndex,
          ExpressLRS_currTlmDenom, telemetryBurstMax, rxLuaQueueCount,
          TelemetrySender.IsActive() ? 1 : 0, TelemetrySender.GetState(),
          TelemetrySender.GetCurrentPackage(), TelemetrySender.GetCurrentOffset(),
          TelemetrySender.GetBytesLastPayload(), TelemetrySender.GetWaitCount(),
          TelemetrySender.GetMaxPacketsBeforeResync(),
          TelemetrySender.GetExpectedAck() ? 1 : 0,
          (unsigned long)(telemetryDataUlAckCount -
                          rxLuaBindPostDiagStartTelemetryDataUlAckCount),
          telemetryLastDataUlAck ? 1 : 0,
          (unsigned long)(rxLuaQueueDropCount -
                          rxLuaBindPostDiagStartQueueDropCount),
          (unsigned long)(rxLuaUlChunkCount -
                          rxLuaBindPostDiagStartUlChunkCount),
          (unsigned long)(rxLuaUlCompleteCount -
                          rxLuaBindPostDiagStartUlCompleteCount),
          (unsigned long)(rxLuaCrsfHandledCount -
                          rxLuaBindPostDiagStartCrsfHandledCount),
          (unsigned long)(telemetryTxCount -
                          rxLuaBindPostDiagStartTelemetryTxCount),
          (unsigned long)(telemetrySuppressedCount -
                          rxLuaBindPostDiagStartTelemetrySuppressedCount),
          (unsigned long)(telemetryRxAfterTx -
                          rxLuaBindPostDiagStartTelemetryRxAfterTx),
          (unsigned long)(telemetryMissedAfterTx -
                          rxLuaBindPostDiagStartTelemetryMissedAfterTx),
          (unsigned long)(telemetryTxBeforeRx -
                          rxLuaBindPostDiagStartTelemetryTxBeforeRx),
          telemetryAwaitingRx, lastValidPacketType, lastValidExpectedNonce,
          lastValidExpectedFhss, LQCalc.getLQRaw(), LQCalc.getCount(),
          Radio.LastPacketRSSI, Radio.LastPacketSNRRaw);
#else
    DBGLN("LUA_BIND_POST age:%lu chunk:%u conn:%d rxst:%d rate:%u den:%u "
          "burst:%u q:%u active:%u st:%u pkg:%u off:%u bytes:%u wait:%u/%u "
          "expack:%u ulack:%lu/%u drop:%lu ul:%lu/%lu crsf:%lu "
          "tlm:%lu/%lu last:%u exp:%u/%u lq:%u/%u rssi:%d snr:%d",
          (unsigned long)(now - rxLuaBindPostDiagStartMs),
          rxLuaBindPostDiagChunk, connectionState, RXtimerState, rateIndex,
          ExpressLRS_currTlmDenom, telemetryBurstMax, rxLuaQueueCount,
          TelemetrySender.IsActive() ? 1 : 0, TelemetrySender.GetState(),
          TelemetrySender.GetCurrentPackage(), TelemetrySender.GetCurrentOffset(),
          TelemetrySender.GetBytesLastPayload(), TelemetrySender.GetWaitCount(),
          TelemetrySender.GetMaxPacketsBeforeResync(),
          TelemetrySender.GetExpectedAck() ? 1 : 0,
          (unsigned long)(telemetryDataUlAckCount -
                          rxLuaBindPostDiagStartTelemetryDataUlAckCount),
          telemetryLastDataUlAck ? 1 : 0,
          (unsigned long)(rxLuaQueueDropCount -
                          rxLuaBindPostDiagStartQueueDropCount),
          (unsigned long)(rxLuaUlChunkCount -
                          rxLuaBindPostDiagStartUlChunkCount),
          (unsigned long)(rxLuaUlCompleteCount -
                          rxLuaBindPostDiagStartUlCompleteCount),
          (unsigned long)(rxLuaCrsfHandledCount -
                          rxLuaBindPostDiagStartCrsfHandledCount),
          (unsigned long)(telemetryTxCount -
                          rxLuaBindPostDiagStartTelemetryTxCount),
          (unsigned long)(telemetrySuppressedCount -
                          rxLuaBindPostDiagStartTelemetrySuppressedCount),
          lastValidPacketType, lastValidExpectedNonce, lastValidExpectedFhss,
          LQCalc.getLQRaw(), LQCalc.getCount(), Radio.LastPacketRSSI,
          Radio.LastPacketSNRRaw);
#endif
  }
#endif

  updateTelemetryBurst();
  maybePrintTelemetry150Snapshot(now);

  if (uidSavePending) {
    uidSavePending = false;
    (void)elrs_config_set_uid(pendingUid);
    (void)elrs_config_save();
    if (!InBindingMode) {
      OtaUpdateCrcInitFromUid();
      FHSSrandomiseFHSSsequence(uidMacSeedGet());
    }
    DBGLN("Persisted UID from MSP bind");
  }

  if (uidRefreshPending) {
    uidRefreshPending = false;
    if (!InBindingMode) {
      OtaUpdateCrcInitFromUid();
      FHSSrandomiseFHSSsequence(uidMacSeedGet());
    }
  }

  if (bindCompletePending && !uidSavePending && !uidRefreshPending) {
    bindCompletePending = false;
    if (InBindingMode) {
      DBGLN("Bind UID applied - leaving binding mode");
      elrs_exit_binding_mode();
    }
  }

  if (dataUlReady) {
    DataUlReceiveComplete();
  }

  // Keep the ELRS link-state maintenance ahead of telemetry-downlink queuing,
  // matching the ESP32 RX loop ordering.
  if ((connectionState != disconnected) &&
      (ExpressLRS_nextAirRateIndex != ExpressLRS_currAirRate_Modparams->index)) {
    uint8_t requestedRateIndex = ExpressLRS_nextAirRateIndex;
#if ELRS_DIAG_RATE_CHANGE_LOG
    DBGLN("Req air rate change %u->%u",
          (unsigned)ExpressLRS_currAirRate_Modparams->index,
          (unsigned)requestedRateIndex);
    DBGLN("RATECHG_TLM tx:%lu rcack:%lu/%u ulack:%lu/%u den:%u burst:%u state:%u wait:%u/%u "
          "last:%u exp:%u/%u sync:%u/%u lq:%u/%u",
          (unsigned long)telemetryTxCount,
          (unsigned long)telemetryRcConfirmCount,
          telemetryLastRcConfirm ? 1 : 0,
          (unsigned long)telemetryDataUlAckCount,
          telemetryLastDataUlAck ? 1 : 0, ExpressLRS_currTlmDenom,
          telemetryBurstMax, (unsigned)TelemetrySender.GetState(),
          TelemetrySender.GetWaitCount(),
          TelemetrySender.GetMaxPacketsBeforeResync(), lastValidPacketType,
          lastValidExpectedNonce, lastValidExpectedFhss, lastValidSyncNonce,
          lastValidSyncFhss, LQCalc.getLQRaw(), LQCalc.getCount());
#if ELRS_DIAG_TX_TURNAROUND
    DBGLN("RATECHG_TXTRN t2d:%lu d2rx:%lu tx2rx:%lu rxok:%lu miss:%lu "
          "pend:%u pre:%lu ok:%lu tx:%u/%u rx:%u/%u",
          (unsigned long)telemetryTxToDoneUs,
          (unsigned long)telemetryDoneToRxnbDoneUs,
          (unsigned long)telemetryTxToRxnbDoneUs,
          (unsigned long)telemetryRxnbToRxOkUs,
          (unsigned long)telemetryMissedAfterTx, telemetryAwaitingRx,
          (unsigned long)telemetryTxBeforeRx, (unsigned long)telemetryRxAfterTx,
          telemetryTxNonce, telemetryTxFhss, telemetryRxNonce,
          telemetryRxFhss);
#endif
#endif

    if (!isSupportedRFRate(requestedRateIndex)) {
      DBGLN("Mode %u not supported, ignoring", (unsigned)requestedRateIndex);
      requestedRateIndex = ExpressLRS_currAirRate_Modparams->index;
      ExpressLRS_nextAirRateIndex = requestedRateIndex;
    }

    // If the immediate landing on the requested rate fails, do not let the
    // first-connection rate lock strand the RX on a rate it cannot decode.
    LockRFmode = false;
    RFmodeCycleMultiplier = 1;
    scanIndex = getNextSupportedRFRateIndex(requestedRateIndex);

    lastDisconnectReason = DISC_RATE_CHANGE;
    LostConnection(true);
    LastSyncPacket = now;
    RFmodeLastCycled = now;
  }

  if ((connectionState == tentative) &&
      ((now - LastSyncPacket) >
       ExpressLRS_currAirRate_RFperfParams->RxLockTimeoutMs)) {
    lastDisconnectReason = DISC_RX_LOCK_TIMEOUT;
    LostConnection(true);
    RFmodeLastCycled = now;
    LastSyncPacket = now;
  }

  if (connectionState == disconnected) {
    cycleRfMode();
  }

  const uint32_t localLastValidPacket = LastValidPacket;
  if ((connectionState == connected) &&
      ((int32_t)ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs <
       (int32_t)(now - localLastValidPacket))) {
    lastDisconnectReason = DISC_PACKET_TIMEOUT;
    LostConnection(true);
  }

  if (connectionState == tentative) {
    if ((absI32(LPF_OffsetDx.value()) <= 10) && (LPF_Offset.value() < 100) &&
        (LQCalc.getLQRaw() > minLqForChaos())) {
      GotConnection(now);
    }
  }

  if ((RXtimerState == tim_tentative) &&
      ((now - GotConnectionMillis) > ConsiderConnGoodMillis) &&
      (absI32(LPF_OffsetDx.value()) <= 5)) {
    RXtimerState = tim_locked;
  }

  updateSerialRxState();
  serviceCrsfSerialTelemetry();

  if (!TelemetrySender.IsActive()) {
    uint8_t nextPayloadSize = 0;
    if (otaConnector.GetNextPayload(&nextPayloadSize, TelemetryBuffer) &&
        nextPayloadSize > 0) {
#if ELRS_DIAG_CRSF_OTA_VERBOSE
      DBGLN("[DATA_DL] TelemetrySender set type=0x%02X len=%u den=%u active=%u",
            TelemetryBuffer[CRSF_TELEMETRY_TYPE_INDEX], nextPayloadSize,
            ExpressLRS_currTlmDenom, TelemetrySender.IsActive() ? 1 : 0);
#endif
#if ELRS_DIAG_LUA_DISCOVERY
      const uint8_t payloadType = TelemetryBuffer[CRSF_TELEMETRY_TYPE_INDEX];
      if (payloadType == CRSF_FRAMETYPE_DEVICE_INFO ||
          payloadType == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY) {
        luaDiscoveryActive = true;
        luaDiscoveryType = payloadType;
        luaDiscoveryLen = nextPayloadSize;
        luaDiscoveryPrintCount = 0;
        luaDiscoveryStartMs = now;
        luaDiscoveryLastPrintMs = 0;
        luaDiscoveryStartTxCount = telemetryTxCount;
        DBGLN("LUA_DISC dl-start type=0x%02X len=%u den=%u burst=%u fifoEmpty=%u",
              payloadType, nextPayloadSize, ExpressLRS_currTlmDenom,
              telemetryBurstMax, otaConnector.IsEmpty() ? 1 : 0);
      }
#endif
      TelemetrySender.SetDataToTransmit(TelemetryBuffer, nextPayloadSize);
      InvalidatePrebuiltTelemetry();
    } else {
      (void)queueMavlinkSerialPayload(now);
    }
  }

  const bool telemetryBusy = TelemetrySender.IsActive() ||
                             !otaConnector.IsEmpty();
  crsfReceiver.processPending(telemetryBusy);
  if (crsfReceiver.consumeSerialApplyRequest()) {
    requestSerialProtocolApply();
  }
  processSerialProtocolApply();
  processStartupRateSave(now);
  maybePrintLuaProgress(now);
  maybePrintLinkProgress(now);

#if ELRS_DIAG_LUA_DISCOVERY
  if (luaDiscoveryActive) {
    if (!TelemetrySender.IsActive() && otaConnector.IsEmpty()) {
      DBGLN("LUA_DISC dl-done type=0x%02X len=%u age=%lu tx=%lu",
            luaDiscoveryType, luaDiscoveryLen,
            (unsigned long)(now - luaDiscoveryStartMs),
            (unsigned long)(telemetryTxCount - luaDiscoveryStartTxCount));
      luaDiscoveryActive = false;
    } else if (luaDiscoveryPrintCount < 5 &&
               (luaDiscoveryLastPrintMs == 0 ||
                (uint32_t)(now - luaDiscoveryLastPrintMs) >= 250U)) {
      luaDiscoveryLastPrintMs = now;
      luaDiscoveryPrintCount++;
      DBGLN("LUA_DISC dl-state type=0x%02X age=%lu active=%u st=%u pkg=%u off=%u bytes=%u wait=%u/%u expAck=%u tx=%lu fifoEmpty=%u",
            luaDiscoveryType, (unsigned long)(now - luaDiscoveryStartMs),
            TelemetrySender.IsActive() ? 1 : 0, TelemetrySender.GetState(),
            TelemetrySender.GetCurrentPackage(), TelemetrySender.GetCurrentOffset(),
            TelemetrySender.GetBytesLastPayload(), TelemetrySender.GetWaitCount(),
            TelemetrySender.GetMaxPacketsBeforeResync(),
            TelemetrySender.GetExpectedAck() ? 1 : 0,
            (unsigned long)(telemetryTxCount - luaDiscoveryStartTxCount),
            otaConnector.IsEmpty() ? 1 : 0);
    }
  }
#endif

  updateSwitchMode();
  processDecodedUplinkTxPower();
  updateRxDownlinkPower(false);

  // Send serial RC channels, honoring the configured failsafe mode after RF loss.
  static uint32_t lastRcOutput = 0;
  const uint8_t serialProtocol = getConfiguredSerialProtocol();
  const bool shouldSendSerialRc = shouldOutputSerialRcFrames();
  const uint32_t rcOutputAge = now - lastRcOutput;

#if SIW917_ELRS_CRSF_SERIAL_DIAG_LOGS
  static uint8_t serialRcGateDiagCount = 0;
  static uint32_t serialRcGateDiagLastMs = 0;
  if (connectionState == connected && serialRcGateDiagCount < 8U &&
      (uint32_t)(now - serialRcGateDiagLastMs) >= 500U) {
    serialRcGateDiagLastMs = now;
    serialRcGateDiagCount++;
    DBGLN("CRSF_RC_GATE send=%u bind=%u wifi=%u ready=%u proto=%u sendsRc=%u ota=%u model=%u teamEn=%u team=%u age=%lu tx=%lu",
          shouldSendSerialRc ? 1 : 0, InBindingMode ? 1 : 0,
          InWiFiMode ? 1 : 0, crsf_serial_is_ready() ? 1 : 0,
          serialProtocol, configuredSerialProtocolSendsRc() ? 1 : 0,
          TxOtaProtocol, connectionHasModelMatch ? 1 : 0,
          teamraceIsEnabled() ? 1 : 0, teamraceHasModelMatch ? 1 : 0,
          (unsigned long)rcOutputAge,
          (unsigned long)crsf_serial_get_tx_count());
  }
#endif

  if (shouldSendSerialRc &&
      rcOutputAge >= serialRcOutputIntervalMs(serialProtocol)) {
    lastRcOutput = now;
    if (serialProtocolUsesSbus(serialProtocol)) {
      const bool failsafeActive = connectionState != connected;
      (void)crsf_serial_send_sbus_channels(ChannelData, failsafeActive,
                                           failsafeActive);
    } else if (serialProtocolUsesSumd(serialProtocol)) {
      (void)crsf_serial_send_sumd_channels(ChannelData);
    } else {
      uint32_t crsfChannels[CRSF_NUM_CHANNELS] = {};
      prepareCrsfSerialChannels(crsfChannels);
      (void)crsf_serial_send_channels(crsfChannels);
    }
  }

  // Update external link stats and send to FC periodically
  static uint32_t lastLinkStatsUpdate = 0;
  if ((now - lastLinkStatsUpdate) > SEND_LINK_STATS_TO_FC_INTERVAL) {
    lastLinkStatsUpdate = now;

    currentLinkStats.rssi_1 = linkStats.uplink_RSSI_1;
    currentLinkStats.rssi_2 = linkStats.uplink_RSSI_2;
    currentLinkStats.snr = linkStats.uplink_SNR;
    currentLinkStats.lq = uplinkLQ;
    currentLinkStats.active_ant = antenna;
    if (ExpressLRS_currAirRate_Modparams) {
      currentLinkStats.rf_mode = ExpressLRS_currAirRate_Modparams->index;
    }

    // Send link stats to FC via CRSF
    if (crsf_serial_is_ready() && configuredSerialProtocolUsesCrsf() &&
        (connectionState == connected) && (TxOtaProtocol == TX_NORMAL_MODE)) {
      crsf_link_stats_t crsfStats;
      crsfStats.uplink_rssi_1 = linkStats.uplink_RSSI_1;
      crsfStats.uplink_rssi_2 = linkStats.uplink_RSSI_2;
      crsfStats.uplink_lq = uplinkLQ;
      crsfStats.uplink_snr = linkStats.uplink_SNR;
      crsfStats.active_antenna = antenna;
      crsfStats.rf_mode = ExpressLRS_currAirRate_Modparams
                              ? ExpressLRS_currAirRate_Modparams->index
                              : 0;
      crsfStats.uplink_tx_power = linkStats.uplink_TX_Power;
      crsfStats.downlink_rssi = 0; // RX doesn't have downlink stats
      crsfStats.downlink_lq = 0;
      crsfStats.downlink_snr = 0;
      crsf_serial_send_link_stats(&crsfStats);
    }
  }

  // Update status LED based on connection state
  static connectionState_e lastLedState = disconnected;
  if (connectionState != lastLedState) {
    lastLedState = connectionState;
    switch (connectionState) {
    case connected:
      status_led_set_mode(LED_MODE_CONNECTED);
      break;
    case tentative:
      status_led_set_mode(LED_MODE_TENTATIVE);
      break;
    case disconnected:
      status_led_set_mode(LED_MODE_DISCONNECTED);
      break;
    case radioFailed:
      status_led_set_mode(LED_MODE_ERROR);
      break;
    default:
      break;
    }
  }

  // Update LED blinking patterns
  status_led_update();

  // Poll bind button for long-press detection
  bind_button_poll();

  updateBindingMode(now);

  //=========================================================================
  // HP GPIO DIO1 Interrupt Mode
  //
  // DIO1 is connected to GPIO_46 (HP domain) on the SiW917. The interrupt
  // is configured in LR1121_hal.cpp init() via lr1121_dio1_init().
  //
  // The interrupt callback chain is:
  //   DIO1 rising edge -> NVIC IRQ -> dio1_gpio_interrupt_callback()
  //   -> dioISR_1() -> IsrCallback_1() -> Radio.IsrCallback() -> packet
  //   processing
  //
  // No polling needed - hardware interrupts handle packet reception.
  //=========================================================================

  // NOTE: Periodic DBGLN printing of ISR stats removed.
  // DBGLN calls printf which uses UART interrupts/DMA. That was taking too
  // long and starving the GSPI DMA completion callback when hwTimer frequency
  // hopped!
  hwTimer::service();
}

elrs_connection_state_t elrs_get_connection_state(void) {
  switch (connectionState) {
  case connected:
    return ELRS_CONNECTED;
  case tentative:
    return ELRS_TENTATIVE;
  case disconnected:
    return ELRS_DISCONNECTED;
  case radioFailed:
    return ELRS_RADIO_FAILED;
  default:
    return ELRS_DISCONNECTED;
  }
}

bool elrs_is_connected(void) {
  return (connectionState == connected || connectionState == tentative);
}

uint8_t elrs_get_channels(uint32_t *channels) {
  if (channels) {
    memcpy(channels, ChannelData, sizeof(ChannelData));
  }
  return CRSF_NUM_CHANNELS;
}

void elrs_get_link_stats(elrs_link_stats_t *stats) {
  if (stats) {
    *stats = currentLinkStats;
  }
}

void elrs_set_channel_callback(elrs_channel_callback_t callback) {
  channelCallback = callback;
}

void elrs_enter_binding_mode(void) {
  if (InBindingMode) {
    return;
  }

  if (getConfiguredBindStorage() == ELRS_BIND_STORAGE_ADMINISTERED) {
    DBGLN("Binding mode request ignored - Bind Storage is Administered");
    return;
  }

  if (connectionState == connected || connectionState == tentative ||
      hwTimer::isRunning()) {
    bindingModeRequest = true;
    DBGLN("Binding mode requested");
    return;
  }

  enterBindingModeNow();
}

void elrs_exit_binding_mode(void) {
  if (!InBindingMode) {
    return;
  }

  bindingModeRequest = false;
  TelemetrySender.ResetState();
  InvalidatePrebuiltTelemetry();
  DataUlReceiver.ResetState();
  dataUlReady = false;
  Radio.SetTxIdleMode();

  bindCompletePending = false;
  OtaUpdateCrcInitFromUid();
  FHSSrandomiseFHSSsequence(uidMacSeedGet());
  LockRFmode = false;
  RFmodeCycleMultiplier = 1;
  scanIndex = getFirstSupportedRFRateIndex();
  ExpressLRS_nextAirRateIndex = scanIndex;
  InBindingMode = false;
  lastDisconnectReason = DISC_EXTERNAL;
  LostConnection(false);
  Radio.RXnb();
  LastValidPacket = millis();
  LastSyncPacket = LastValidPacket;
  RFmodeLastCycled = LastValidPacket;
  status_led_set_mode(LED_MODE_DISCONNECTED);
  DBGLN("Exiting binding mode");
}

void elrs_enter_wifi_mode(void) {
  if (InWiFiMode) {
    DBGLN("Already in WiFi mode");
    return;
  }

  DBGLN("Entering WiFi configuration mode...");
  InWiFiMode = true;

  // Stop the radio to free up resources
  hwTimer::stop();

  // Set LED to WiFi mode (fast blink LED1)
  status_led_set_mode(LED_MODE_WIFI);

  // Start WiFi AP and HTTP server
  // Note: This function blocks and handles HTTP requests in a loop
  // Device will reset when user triggers reboot from web UI
  wifi_http_test_run();

  // If we ever return from WiFi mode (shouldn't normally happen)
  // Note: Normally device reboots from WiFi web UI, so this code path
  // is only reached if wifi_http_test_run() returns unexpectedly
  InWiFiMode = false;
  status_led_set_mode(LED_MODE_DISCONNECTED);
  DBGLN("Exited WiFi mode - recommend rebooting to restore full functionality");
}

bool elrs_is_wifi_mode(void) { return InWiFiMode; }

bool elrs_set_uid(const uint8_t new_uid[6]) {
  if (new_uid == nullptr) {
    return false;
  }

  // Update runtime UID
  memcpy(UID, new_uid, UID_LEN);

  // Save to persistent storage
  elrs_config_set_uid(new_uid);
  int result = elrs_config_save();

  if (result == 0) {
    DBGLN("New UID saved: %02X:%02X:%02X:%02X:%02X:%02X", UID[0], UID[1],
          UID[2], UID[3], UID[4], UID[5]);

    // Re-initialize FHSS with new UID
    OtaUpdateCrcInitFromUid();
    FHSSrandomiseFHSSsequence(uidMacSeedGet());

    return true;
  } else {
    DBGLN("ERROR: Failed to save UID to NVM3");
    return false;
  }
}

uint8_t elrs_get_lq(void) { return uplinkLQ; }

const char *elrs_get_rate_name(void) {
  if (!ExpressLRS_currAirRate_Modparams) {
    return "Unknown";
  }

  switch (ExpressLRS_currAirRate_Modparams->enum_rate) {
  case RATE_LORA_900_50HZ:
    return "900M 50Hz";
  case RATE_LORA_900_100HZ:
    return "900M 100Hz";
  case RATE_LORA_900_200HZ:
    return "900M 200Hz";
  case RATE_LORA_900_250HZ:
    return "900M 250Hz";
  case RATE_LORA_2G4_50HZ:
    return "2.4G 50Hz";
  case RATE_LORA_2G4_150HZ:
    return "2.4G 150Hz";
  case RATE_LORA_2G4_250HZ:
    return "2.4G 250Hz";
  case RATE_LORA_2G4_500HZ:
    return "2.4G 500Hz";
  default:
    return "Custom";
  }
}

//=============================================================================
// Legacy C API wrappers for gspi_example.c compatibility
//=============================================================================

void elrs_rx_test(void) {
  DBGLN("=== ELRS C++ Integration Test ===");
  DBGLN("Platform: SiW917");
  DBGLN("Radio: LR1121 (Waveshare Core1121-HF)");
  DBGLN("Mode: 900MHz Sub-GHz");
  DBGLN("UID: 0x%02X%02X%02X%02X%02X%02X", UID[0], UID[1], UID[2], UID[3],
        UID[4], UID[5]);
  DBGLN("=== End Integration Test ===");
}

void elrs_rx_init(void) { elrs_init(); }

void elrs_rx_start(void) { DBGLN("ELRS RX Started"); }

void elrs_rx_stop(void) {
  DBGLN("ELRS RX Stopped");
  hwTimer::stop();
}

void elrs_rx_loop(void) { elrs_loop(); }

} // extern "C"
