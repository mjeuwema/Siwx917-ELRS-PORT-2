#include "Arduino.h"
#include "LR1121.h"
#include "LR1121_hal.h"
#include "../../include/common.h"
#include "OTA.h"
#include "logging.h"
#include "siw917_elrs_timing.h"

// C functions from lr1121_driver.c
extern "C" {
#include "lr1121_driver.h"
}

// LittleFS is ESP32-specific, not needed for SiW917
#if defined(PLATFORM_ESP32)
#include <LittleFS.h>
#endif
#include <SPIEx.h>

LR1121Hal hal;
LR1121Driver *LR1121Driver::instance = NULL;
extern RXtimerState_e RXtimerState;

extern "C" {
volatile uint8_t lr1121_tlm_miss_irq_trace_active = 0;
}

#if SIW917_ELRS_TLM_MISS_IRQ_TRACE
// This trace observes only IRQs already delivered by the normal DIO mask. It
// never reads the radio or changes the DIO configuration on the RF hot path.
static volatile uint32_t tlmMissIrqTraceEventCount = 0;
static volatile uint32_t tlmMissIrqTraceRxDoneCount = 0;
static volatile uint32_t tlmMissIrqTracePacketRejectCount = 0;
static volatile uint32_t tlmMissIrqTraceCrcCount = 0;
static volatile uint32_t tlmMissIrqTraceSyncCount = 0;
static volatile uint32_t tlmMissIrqTraceTimeoutCount = 0;
static volatile uint32_t tlmMissIrqTraceErrorCount = 0;
static volatile uint32_t tlmMissIrqTraceLastStatus = 0;
static volatile uint32_t tlmMissIrqTraceFifoReadCount = 0;
static volatile uint32_t tlmMissIrqTraceFifoBusyEntryCount = 0;
static volatile uint32_t tlmMissIrqTraceFifoBusyTimeoutCount = 0;

static void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
lr1121TlmMissIrqTraceRecord(uint32_t irqStatus) {
  if (lr1121_tlm_miss_irq_trace_active == 0U) {
    return;
  }

  tlmMissIrqTraceEventCount++;
  tlmMissIrqTraceLastStatus = irqStatus;
  if ((irqStatus & (LR1121_IRQ_RX_DONE | LR20XX_IRQ_RX_FIFO)) != 0U) {
    tlmMissIrqTraceRxDoneCount++;
  }
  // These flags are recorded only if co-latched with an IRQ already routed to
  // DIO; enabling their DIO sources would change the timing being measured.
  if ((irqStatus &
       (LR20XX_IRQ_CRC_ERROR | LR20XX_IRQ_LORA_HEADER_CRC_ERROR)) != 0U) {
    tlmMissIrqTraceCrcCount++;
  }
  if ((irqStatus & LR20XX_IRQ_SYNC_FAIL) != 0U) {
    tlmMissIrqTraceSyncCount++;
  }
  if ((irqStatus & LR1121_IRQ_TIMEOUT) != 0U) {
    tlmMissIrqTraceTimeoutCount++;
  }
  if ((irqStatus &
       (LR20XX_IRQ_ERROR | LR20XX_IRQ_LEN_ERROR | LR20XX_IRQ_ADDR_ERROR)) !=
      0U) {
    tlmMissIrqTraceErrorCount++;
  }
}

extern "C" void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
lr1121_tlm_miss_irq_trace_arm(void) {
  tlmMissIrqTraceEventCount = 0;
  tlmMissIrqTraceRxDoneCount = 0;
  tlmMissIrqTracePacketRejectCount = 0;
  tlmMissIrqTraceCrcCount = 0;
  tlmMissIrqTraceSyncCount = 0;
  tlmMissIrqTraceTimeoutCount = 0;
  tlmMissIrqTraceErrorCount = 0;
  tlmMissIrqTraceLastStatus = 0;
  tlmMissIrqTraceFifoReadCount = 0;
  tlmMissIrqTraceFifoBusyEntryCount = 0;
  tlmMissIrqTraceFifoBusyTimeoutCount = 0;
  lr1121_tlm_miss_irq_trace_active = 1;
}

extern "C" void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
lr1121_tlm_miss_irq_trace_complete(void) {
  lr1121_tlm_miss_irq_trace_active = 0;
}

extern "C" void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
lr1121_tlm_miss_irq_trace_record_fifo_busy_wait(bool busyAtEntry,
                                                bool ready) {
  if (lr1121_tlm_miss_irq_trace_active == 0U) {
    return;
  }
  tlmMissIrqTraceFifoReadCount++;
  if (busyAtEntry) {
    tlmMissIrqTraceFifoBusyEntryCount++;
  }
  if (!ready) {
    tlmMissIrqTraceFifoBusyTimeoutCount++;
  }
}

extern "C" void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
lr1121_tlm_miss_irq_trace_latch(
    uint32_t *eventCount, uint32_t *rxDoneCount, uint32_t *packetRejectCount,
    uint32_t *crcCount, uint32_t *syncCount, uint32_t *timeoutCount,
    uint32_t *errorCount, uint32_t *lastIrq, uint32_t *fifoReadCount,
    uint32_t *fifoBusyEntryCount, uint32_t *fifoBusyTimeoutCount) {
  if (eventCount) *eventCount = tlmMissIrqTraceEventCount;
  if (rxDoneCount) *rxDoneCount = tlmMissIrqTraceRxDoneCount;
  if (packetRejectCount) *packetRejectCount = tlmMissIrqTracePacketRejectCount;
  if (crcCount) *crcCount = tlmMissIrqTraceCrcCount;
  if (syncCount) *syncCount = tlmMissIrqTraceSyncCount;
  if (timeoutCount) *timeoutCount = tlmMissIrqTraceTimeoutCount;
  if (errorCount) *errorCount = tlmMissIrqTraceErrorCount;
  if (lastIrq) *lastIrq = tlmMissIrqTraceLastStatus;
  if (fifoReadCount) *fifoReadCount = tlmMissIrqTraceFifoReadCount;
  if (fifoBusyEntryCount) {
    *fifoBusyEntryCount = tlmMissIrqTraceFifoBusyEntryCount;
  }
  if (fifoBusyTimeoutCount) {
    *fifoBusyTimeoutCount = tlmMissIrqTraceFifoBusyTimeoutCount;
  }
  lr1121_tlm_miss_irq_trace_active = 0;
}
#else
static void lr1121TlmMissIrqTraceRecord(uint32_t) {}
extern "C" void lr1121_tlm_miss_irq_trace_arm(void) {}
extern "C" void lr1121_tlm_miss_irq_trace_complete(void) {}
extern "C" void lr1121_tlm_miss_irq_trace_record_fifo_busy_wait(bool, bool) {}
extern "C" void lr1121_tlm_miss_irq_trace_latch(
    uint32_t *eventCount, uint32_t *rxDoneCount, uint32_t *packetRejectCount,
    uint32_t *crcCount, uint32_t *syncCount, uint32_t *timeoutCount,
    uint32_t *errorCount, uint32_t *lastIrq, uint32_t *fifoReadCount,
    uint32_t *fifoBusyEntryCount, uint32_t *fifoBusyTimeoutCount) {
  if (eventCount) *eventCount = 0;
  if (rxDoneCount) *rxDoneCount = 0;
  if (packetRejectCount) *packetRejectCount = 0;
  if (crcCount) *crcCount = 0;
  if (syncCount) *syncCount = 0;
  if (timeoutCount) *timeoutCount = 0;
  if (errorCount) *errorCount = 0;
  if (lastIrq) *lastIrq = 0;
  if (fifoReadCount) *fifoReadCount = 0;
  if (fifoBusyEntryCount) *fifoBusyEntryCount = 0;
  if (fifoBusyTimeoutCount) *fifoBusyTimeoutCount = 0;
}
#endif

static volatile uint8_t siw917_last_payload_length = 8;
static volatile uint32_t siw917_rxnbisr_entry_us = 0;
static volatile uint32_t siw917_packet_ready_us = 0;
static uint8_t siw917_gfsk_fail_dump_count = 0;
static uint8_t siw917_lora_fail_dump_count = 0;
static volatile uint8_t siw917_debug_lora_sf = 0;
static volatile uint8_t siw917_debug_lora_cr = 0;
static volatile uint8_t siw917_debug_lora_pre = 0;
static volatile uint8_t siw917_debug_lora_payload = 0;
static uint32_t siw917_fe_cal_missing_log_count = 0;
static int8_t siw917_lr2021_freq_offset_ppm = SIW917_ELRS_LR2021_FREQ_OFFSET_FIXED_PPM;
static uint8_t siw917_lr2021_freq_offset_slot = 0xFF;
static bool siw917_lr2021_freq_offset_locked = false;

// The LR20xx FIFO path follows Semtech's direct FIFO read API: the two command
// bytes are sent by the HAL, but only payload bytes are returned here.
static constexpr uint8_t LR2021_RX_FIFO_COMMAND_PHASE_BYTES = 0;
static constexpr uint8_t LR2021_RX_FIFO_DISCONNECTED_GUARD_BYTES = 6;
static constexpr uint8_t LR20XX_RESPONSE_STATUS_LEN = 2;
static volatile uint8_t
    siw917_rx_fifo_payload_offset = LR2021_RX_FIFO_COMMAND_PHASE_BYTES;
static constexpr uint8_t LR2021_VERSION_HW = 0x20;
static constexpr uint8_t LR2021_VERSION_TYPE = 0x21;
static constexpr uint32_t LR20XX_LORA_SX1276_COMPAT_ADDR = 0x00F30A14UL;
static constexpr uint32_t LR20XX_LORA_SX1276_COMPAT_MASK = (3UL << 18);
static constexpr uint32_t LR20XX_LORA_SX1276_COMPAT_ENABLE = (1UL << 19);
static constexpr uint32_t LR20XX_LORA_SX1276_COMPAT_DISABLE = 0;
static constexpr uint32_t LR20XX_LORA_RX_CFG_ADDR = 0x00F30A2CUL;
static constexpr uint32_t LR20XX_LORA_FREQ_RANGE_MASK = (3UL << 16);
static constexpr uint32_t LR20XX_DCDC_ADC_CTRL_ADDR = 0x00F40200UL;
static constexpr uint32_t LR20XX_DCDC_RX_PATH_ADDR = 0x00F40430UL;
static constexpr uint32_t LR20XX_DCDC_SWITCHER_ADDR = 0x00F20024UL;
static constexpr uint32_t LR20XX_DCDC_SWITCHER_RISE_MASK = (0xFUL << 20);
static constexpr uint32_t LR20XX_DCDC_SWITCHER_FALL_MASK = (0xFUL << 16);
static constexpr uint32_t LR20XX_DCDC_FREQ_LF_ADDR = 0x0080004CUL;
static constexpr uint32_t LR20XX_DCDC_RF_FREQ_ADDR = 0x00F40144UL;
static constexpr uint32_t LR20XX_FE_CAL_FCC915_LOW_HZ = 903500000UL;
static constexpr uint32_t LR20XX_FE_CAL_FCC915_MID_HZ = 915500000UL;
static constexpr uint32_t LR20XX_FE_CAL_FCC915_HIGH_HZ = 926900000UL;
static constexpr uint32_t LR20XX_FE_CAL_ISM2G4_LOW_HZ = 2400400000UL;
static constexpr uint32_t LR20XX_FE_CAL_ISM2G4_MID_HZ = 2440000000UL;
static constexpr uint32_t LR20XX_FE_CAL_ISM2G4_HIGH_HZ = 2479400000UL;
static constexpr uint16_t LR20XX_FE_CAL_BUCKET_MASK = 0x7FFFU;
static constexpr uint16_t LR20XX_FE_CAL_COVERAGE_BUCKETS = 12U; // 48 MHz

static int32_t SIW917_ELRS_RAMFUNC_ATTR
siw917Lr2021FreqOffsetHz(uint32_t freq) {
#if SIW917_ELRS_LR2021_FREQ_OFFSET_PPM_SWEEP
  static constexpr int8_t offsetPpmSweep[] = {-40, -25, -12, 0, 12, 25, 40};
  static constexpr uint8_t offsetPpmSweepCount =
      sizeof(offsetPpmSweep) / sizeof(offsetPpmSweep[0]);
  const uint32_t now = millis();

  if (connectionState == disconnected) {
    siw917_lr2021_freq_offset_locked = false;
    const uint32_t dwellMs =
        SIW917_ELRS_LR2021_FREQ_OFFSET_SWEEP_DWELL_MS == 0
            ? 2500UL
            : (uint32_t)SIW917_ELRS_LR2021_FREQ_OFFSET_SWEEP_DWELL_MS;
    const uint8_t slot = (now / dwellMs) % offsetPpmSweepCount;
    if (slot != siw917_lr2021_freq_offset_slot) {
      siw917_lr2021_freq_offset_slot = slot;
      siw917_lr2021_freq_offset_ppm = offsetPpmSweep[slot];
      const int32_t offsetHz =
          (int32_t)(((int64_t)freq * siw917_lr2021_freq_offset_ppm) /
                    1000000LL);
      DBGLN("LR2021 freq-offset diag: slot=%u/%u ppm=%d offset=%ldHz",
            (unsigned)slot + 1U, (unsigned)offsetPpmSweepCount,
            (int)siw917_lr2021_freq_offset_ppm, (long)offsetHz);
    }
  } else if (!siw917_lr2021_freq_offset_locked) {
    siw917_lr2021_freq_offset_locked = true;
    const int32_t offsetHz =
        (int32_t)(((int64_t)freq * siw917_lr2021_freq_offset_ppm) /
                  1000000LL);
    DBGLN("LR2021 freq-offset diag: locked ppm=%d offset=%ldHz state=%u",
          (int)siw917_lr2021_freq_offset_ppm, (long)offsetHz,
          (unsigned)connectionState);
  }
#endif
  return (int32_t)(((int64_t)freq * siw917_lr2021_freq_offset_ppm) /
                   1000000LL);
}

static uint32_t SIW917_ELRS_RAMFUNC_ATTR
siw917Lr2021ApplyFreqOffset(uint32_t freq) {
  const int32_t offsetHz = siw917Lr2021FreqOffsetHz(freq);
  const int64_t adjusted = (int64_t)freq + (int64_t)offsetHz;
  if (adjusted < 100000000LL) {
    return freq;
  }
  if (adjusted > 3000000000LL) {
    return freq;
  }
  return (uint32_t)adjusted;
}

#if defined(SIW917_ELRS_DISCONNECTED_SCAN_DIAG) &&                            \
    SIW917_ELRS_DISCONNECTED_SCAN_DIAG
static bool SIW917_ELRS_RAMFUNC_ATTR siw917FindOtaCrcMatch(
    const uint8_t *buffer, uint8_t availableLength, uint8_t payloadLength,
    bool syncOnly, uint8_t *matchOffset, uint8_t *matchNonce,
    uint8_t *matchType) {
  if (buffer == nullptr || payloadLength == 0 ||
      availableLength < payloadLength) {
    return false;
  }

  const uint8_t maxOffset = availableLength - payloadLength;
  for (uint8_t offset = 0; offset <= maxOffset; offset++) {
    OTA_Packet_s packet = {};
    memcpy(&packet, buffer + offset, payloadLength);
    const uint8_t type = ((const uint8_t *)&packet)[0] & 0x03U;

    if (type == PACKET_TYPE_SYNC) {
      OTA_Packet_s packetCopy = packet;
      if (OtaValidatePacketCrcForNonce(&packetCopy, 0)) {
        *matchOffset = offset;
        *matchNonce = 0;
        *matchType = type;
        return true;
      }
      continue;
    }

    if (syncOnly) {
      continue;
    }

    for (uint16_t nonce = 0; nonce <= 0xFFU; nonce++) {
      OTA_Packet_s packetCopy = packet;
      if (OtaValidatePacketCrcForNonce(&packetCopy, (uint8_t)nonce)) {
        *matchOffset = offset;
        *matchNonce = (uint8_t)nonce;
        *matchType = type;
        return true;
      }
    }
  }

  return false;
}

static uint8_t SIW917_ELRS_RAMFUNC_ATTR siw917ReverseBits8(uint8_t value) {
  value = (uint8_t)(((value & 0xF0U) >> 4) | ((value & 0x0FU) << 4));
  value = (uint8_t)(((value & 0xCCU) >> 2) | ((value & 0x33U) << 2));
  value = (uint8_t)(((value & 0xAAU) >> 1) | ((value & 0x55U) << 1));
  return value;
}

static bool SIW917_ELRS_RAMFUNC_ATTR siw917FindGfskFecCrcVariant(
    const uint8_t *fecBuffer, uint8_t fecLength, uint8_t variant,
    uint8_t *decoded, uint8_t *matchNonce, uint8_t *matchType) {
  if (fecBuffer == nullptr || fecLength < 14 || decoded == nullptr ||
      matchNonce == nullptr || matchType == nullptr) {
    return false;
  }

  uint8_t transformed[14] = {};
  for (uint8_t i = 0; i < sizeof(transformed); i++) {
    uint8_t value = fecBuffer[i];
    if ((variant & 0x01U) != 0) {
      value = siw917ReverseBits8(value);
    }
    if ((variant & 0x02U) != 0) {
      value = (uint8_t)~value;
    }
    transformed[i] = value;
  }

  FECDecode(transformed, decoded);
  OTA_Packet_s packet = {};
  memcpy(&packet, decoded, OTA4_PACKET_SIZE);
  const uint8_t type = decoded[0] & 0x03U;

  if (type == PACKET_TYPE_SYNC) {
    OTA_Packet_s packetCopy = packet;
    if (OtaValidatePacketCrcForNonce(&packetCopy, 0)) {
      *matchNonce = 0;
      *matchType = type;
      return true;
    }
    return false;
  }

  for (uint16_t nonce = 0; nonce <= 0xFFU; nonce++) {
    OTA_Packet_s packetCopy = packet;
    if (OtaValidatePacketCrcForNonce(&packetCopy, (uint8_t)nonce)) {
      *matchNonce = (uint8_t)nonce;
      *matchType = type;
      return true;
    }
  }

  *matchNonce = 0xFF;
  *matchType = type;
  return false;
}
#endif // SIW917_ELRS_DISCONNECTED_SCAN_DIAG

static uint8_t LR2021LoRaPreambleForSf(uint8_t preambleLength, uint8_t sf) {
#if SIW917_ELRS_LR2021_LORA_SF5_MIN_PREAMBLE > 0
  if ((sf == LR11XX_RADIO_LORA_SF5 || sf == LR11XX_RADIO_LORA_SF6) &&
      (preambleLength < SIW917_ELRS_LR2021_LORA_SF5_MIN_PREAMBLE)) {
    return SIW917_ELRS_LR2021_LORA_SF5_MIN_PREAMBLE;
  }
#endif
  return preambleLength;
}

static bool LR2021LoRaSx1276CompatForSf(uint8_t sf) {
#if SIW917_ELRS_LR2021_LORA_SF5_SX1276_COMPAT
  return sf <= LR11XX_RADIO_LORA_SF6;
#else
  return sf == LR11XX_RADIO_LORA_SF6;
#endif
}

static uint8_t LR2021SpiRadioForMask(SX12XX_Radio_Number_t radioNumber) {
  return (radioNumber == SX12XX_Radio_2) ? LR1121_RADIO_2 : LR1121_RADIO_1;
}

static uint32_t LR20xxPllStepToHz(uint32_t pllSteps) {
  const uint64_t numerator =
      (uint64_t)pllSteps * (uint64_t)15625ULL;
  const uint64_t denominator = (uint64_t)(1UL << 14);
  return (uint32_t)((numerator + denominator - 1ULL) / denominator);
}

extern "C" uint32_t lr1121_get_last_rxnbisr_entry_us(void) {
  return siw917_rxnbisr_entry_us;
}

extern "C" uint32_t lr1121_get_last_packet_ready_us(void) {
  return siw917_packet_ready_us;
}

// DEBUG_LR1121_OTA_TIMING

#if defined(DEBUG_LR1121_OTA_TIMING)
static uint32_t beginTX;
static uint32_t endTX;
#endif

class FECCodec final : public BufferCodec {
public:
  void encode(uint8_t *out, uint8_t *in, uint32_t len) override;
  void decode(uint8_t *out, uint8_t *in, uint32_t len) override;
} fecCodec;

class CopyCodec final : public BufferCodec {
public:
  void encode(uint8_t *out, uint8_t *in, uint32_t len) override;
  void decode(uint8_t *out, uint8_t *in, uint32_t len) override;
} copyCodec;

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
FECCodec::encode(uint8_t *out, uint8_t *in, uint32_t len) {
  memset(out, 0, len); // ensure that the buffer is zeroed to start
  FECEncode(in, out);
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
FECCodec::decode(uint8_t *out, uint8_t *in, uint32_t len) {
  FECDecode(in, out);
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
CopyCodec::encode(uint8_t *out, uint8_t *in, const uint32_t len) {
  memcpy(out, in, len);
}
void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
CopyCodec::decode(uint8_t *out, uint8_t *in, const uint32_t len) {
  memcpy(out, in, len);
}

LR1121Driver::LR1121Driver() : SX12xxDriverCommon() {
  useFSK = false;
  rxContinuousActive = false;
  txInProgress = false;
  lastTxStartSuccessful = false;
  autoRxAfterTxArmed = false;
  pwrCurrentLF = 0;
  pwrPendingLF = PWRPENDING_NONE;
  pwrCurrentHF = 0;
  pwrPendingHF = PWRPENDING_NONE;
  pwrForceUpdate = false;
  radio1isSubGHz = true;
  radio2isSubGHz = true;
  feCalFreqRadio1 = 0;
  feCalFreqRadio2 = 0;
  ResetFrontEndCalCache();
  instance = this;
  strongestReceivingRadio = SX12XX_Radio_1;
  fallBackMode = LR1121_MODE_FS;
  codec = &copyCodec;
}

void LR1121Driver::End() {
  SetMode(LR1121_MODE_SLEEP, SX12XX_Radio_All);
  hal.end();
  RemoveCallbacks();
}

bool LR1121Driver::CheckVersion(const SX12XX_Radio_Number_t radioNumber) {
  firmware_version_t version = GetFirmwareVersion(radioNumber);
  if (version.version == 0) {
    DBGLN("LR2021 #%d version probe returned all zeros", radioNumber);
    return false;
  }
  DBGLN("LR2021 #%d Ready: device=0x%02X FW=0x%04X", radioNumber,
        version.type, version.version);
  return true;
}

bool LR1121Driver::Begin(uint32_t minimumFrequency, uint32_t maximumFrequency) {
  hal.init();
  // NOTE: Do NOT call hal.reset() here!
  // hal.init() already does GPIO/SPI/reset/DIO setup. Keep the post-init
  // sequence aligned with the Waveshare XTAL=true examples.

  // Validate that the LR2021(s) are working.
  if (!CheckVersion(SX12XX_Radio_1))
    return false;
  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    if (!CheckVersion(SX12XX_Radio_2))
      return false;
  }

  printf("LR2021 port build marker: rx-stage-v186-fe-cal-floor\n");

  hal.IsrCallback_1 = &LR1121Driver::IsrCallback_1;
  hal.IsrCallback_2 = &LR1121Driver::IsrCallback_2;

  // Clear Errors
  hal.WriteCommand(LR20XX_SYSTEM_CLEAR_ERRORS,
                   SX12XX_Radio_All); // Remove later?  Might not be required???
  ResetFrontEndCalCache();

  // ExpressLRS transitions from telemetry TX straight into the next uplink
  // window. Match upstream's FS fallback so SetRx can reuse the running
  // frequency synthesizer instead of restarting from RC standby.
#if SIW917_ELRS_LR2021_RX_TX_FALLBACK_FS
  uint8_t FBbuf[1] = {LR20XX_FALLBACK_FS};
  fallBackMode = LR1121_MODE_FS;
#else
  uint8_t FBbuf[1] = {LR20XX_FALLBACK_STDBY_RC};
  fallBackMode = LR1121_MODE_STDBY_RC;
#endif
  hal.WriteCommand(LR20XX_RADIO_SET_RX_TX_FALLBACK_MODE, FBbuf,
                   sizeof(FBbuf), SX12XX_Radio_All);
#if SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("SetRxTxFallbackMode: %s",
        SIW917_ELRS_LR2021_RX_TX_FALLBACK_FS ? "FS" : "STDBY_RC");
#endif

  SetRxPath(true, SX12XX_Radio_All);
  ConfigureRegulatorMode(SX12XX_Radio_All);
  SetDioFunctionIrq(SX12XX_Radio_All);
  SetDioIrqParams();
  CalibrateAll(SX12XX_Radio_All);
  SeedFrontEndCalibrationRange(minimumFrequency, maximumFrequency,
                               SX12XX_Radio_All);

#if SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 CalibFE FHSS range seeded");
#endif

  return true;
}

// 12.2.1 SetTxCw
void LR1121Driver::startCWTest(uint32_t freq,
                               SX12XX_Radio_Number_t radioNumber) {
  // Set a basic Config that can be used for both 2.4G and SubGHz bands.
  Config(LR11XX_RADIO_LORA_BW_62, LR11XX_RADIO_LORA_SF6,
         LR11XX_RADIO_LORA_CR_4_8, freq, 12, false, 8, false, 0, 0,
         radioNumber);
  uint8_t mode[1] = {0x00};
  hal.WriteCommand(LR20XX_RADIO_SET_TX_TEST_MODE, mode, sizeof(mode),
                   radioNumber);
}

void LR1121Driver::Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t regfreq,
                          uint8_t PreambleLength, bool InvertIQ,
                          uint8_t _PayloadLength, bool setFSKModulation,
                          uint8_t fskSyncWord1, uint8_t fskSyncWord2,
                          SX12XX_Radio_Number_t radioNumber) {
#if SIW917_ELRS_RF_RATE_DIAG
  DBGLN("Config: freq=%u, bw=%d, sf=%d, cr=%d, FSK=%d, Pre=%d, InvIQ=%d, "
        "Payload=%d",
        (unsigned int)regfreq, (int)bw, (int)sf, (int)cr, (int)setFSKModulation,
        (int)PreambleLength, (int)InvertIQ, (int)_PayloadLength);
#endif
  PayloadLength = _PayloadLength;

  bool isSubGHz = regfreq < 1000000000;

  if (radioNumber & SX12XX_Radio_1)
    radio1isSubGHz = isSubGHz;

  if (radioNumber & SX12XX_Radio_2)
    radio2isSubGHz = isSubGHz;

  IQinverted = InvertIQ;
  lr11xx_radio_lora_iq_t inverted =
      InvertIQ ? LR11XX_RADIO_LORA_IQ_INVERTED : LR11XX_RADIO_LORA_IQ_STANDARD;
  // IQinverted is always STANDARD for 900
  if (isSubGHz) {
    inverted = LR11XX_RADIO_LORA_IQ_STANDARD;
  }

#if defined(DEBUG_FREQ_CORRECTION) // TODO Check if this available with the
                                   // LR1121?
  const lr11xx_RadioLoRaPacketLengthsModes_t loraPacketLengthType =
      LR1121_LORA_PACKET_VARIABLE_LENGTH;
#else
  const lr11xx_RadioLoRaPacketLengthsModes_t loraPacketLengthType =
      LR1121_LORA_PACKET_FIXED_LENGTH;
#endif

  // Use STDBY_XOSC for Core2021-XF/Waveshare's external 32 MHz crystal.
  // STDBY_RC is not accurate enough for RF configuration.
  SetMode(LR1121_MODE_STDBY_XOSC, radioNumber);

  // CRITICAL: When reconfiguring the radio, the XOSC needs time to stabilize.
  // The SiW917 SPI is so fast that if we immediately send `SetPacketType`
  // (0x020E), the radio asserts BUSY and times out. Add a 5ms delay to prevent
  // `WriteCommand BUSY timeout`.
  delay(5);

  useFSK = setFSKModulation;
  siw917_gfsk_fail_dump_count = 0;
  siw917_lora_fail_dump_count = 0;
  siw917_rx_fifo_payload_offset = LR2021_RX_FIFO_COMMAND_PHASE_BYTES;

  // 8.1.1 SetPacketType
  uint8_t buf[1] = {useFSK ? LR20XX_PACKET_TYPE_GFSK
                           : LR20XX_PACKET_TYPE_LORA};
  hal.WriteCommand(LR20XX_RADIO_SET_PACKET_TYPE, buf, sizeof(buf), radioNumber);
  SetRxTimeoutStopOnPreamble(false, radioNumber);
  if (isSubGHz) {
    ApplyDcdcReset(radioNumber);
  }

  codec = &copyCodec;
  if (useFSK) {
#if SIW917_ELRS_RF_RATE_DIAG
    DBGLN("Config FSK");
#endif
    uint32_t bitrate = (uint32_t)bw * 10000;
    uint8_t bwf = sf;
    uint32_t fdev = (uint32_t)cr * 1000;
    ConfigModParamsFSK(bitrate, bwf, fdev, radioNumber);

    // Increase packet length for FEC used only on 1000Hz 2.5GHz.
    if (!isSubGHz) {
      codec = &fecCodec;
      PayloadLength = 14;
    }

#if SIW917_ELRS_LR2021_GFSK_SET_WHITENING_PARAMS
    SetFSKWhiteningParams(radioNumber);
#endif
    SetPacketParamsFSK(PreambleLength, PayloadLength, radioNumber);
    SetFSKSyncWord(fskSyncWord1, fskSyncWord2, radioNumber);
  } else {
#if SIW917_ELRS_RF_RATE_DIAG
    DBGLN("Config LoRa");
#endif
    const uint8_t loraPreambleLength =
        LR2021LoRaPreambleForSf(PreambleLength, sf);
    siw917_debug_lora_sf = sf;
    siw917_debug_lora_cr = cr;
    siw917_debug_lora_pre = loraPreambleLength;
    siw917_debug_lora_payload = PayloadLength;
    ConfigModParamsLoRa(bw, sf, cr, radioNumber);

    SetPacketParamsLoRa(loraPreambleLength, loraPacketLengthType,
                        PayloadLength, inverted, radioNumber);
    SetLoRaSyncWord(LR20XX_LORA_SYNC_WORD_PRIVATE, sf, radioNumber);
    ConfigureLoRaRxDetector(sf, inverted, radioNumber);
  }

  SetRxPath(isSubGHz, radioNumber);
  // CalibFE is rejected in RX/TX. Enter FS before updating the three on-chip
  // calibration slots; the command returns to this mode when complete.
  SetMode(LR1121_MODE_FS, radioNumber);
  if (isSubGHz) {
    CalibrateFrontEndDefaultSet(radioNumber);
  } else {
    CalibrateFrontEnd24GSet(radioNumber);
  }
  SetFrequencyReg(regfreq, radioNumber, false);

  ClearIrqStatus(radioNumber);

  SetPaConfig(isSubGHz,
              radioNumber); // Must be called after changing rf modes between
                            // subG and 2.4G.  This sets the correct rf amps,
                            // and txen pins to be used.
  if (PayloadLength > 0) {
    siw917_last_payload_length = PayloadLength;
  }
  pwrForceUpdate =
      true; // force an update of the output power because the band may have
            // changed, and we need to configure the power for the band.
  CommitOutputPower();

  // Keep the LR2021 RX arm sequence close to Waveshare RadioLib: leave RX path,
  // DIO routing, and IRQ state as the last touched radio config before SetRx.
  SetRxPath(isSubGHz, radioNumber);
  if (isSubGHz) {
    ApplyDcdcConfigure(radioNumber);
  }
  SetDioIrqParams();
  if (!useFSK) {
    const uint8_t loraPreambleLength =
        LR2021LoRaPreambleForSf(PreambleLength, sf);
    SetPacketParamsLoRa(loraPreambleLength, loraPacketLengthType,
                        PayloadLength, inverted, radioNumber);
    ConfigureLoRaRxDetector(sf, inverted, radioNumber);
  }
  ClearRxFifo(radioNumber);
#if (defined(SIW917_ELRS_DISCONNECTED_SCAN_DIAG) &&                            \
     SIW917_ELRS_DISCONNECTED_SCAN_DIAG) ||                                    \
    (defined(SIW917_ELRS_LR2021_SCAN_TRACE) &&                                 \
     SIW917_ELRS_LR2021_SCAN_TRACE)
  hal.WriteCommand(LR20XX_RADIO_RESET_RX_STATS, radioNumber);
#endif
  ClearIrqStatus(radioNumber);
#if SIW917_ELRS_RF_RATE_DIAG
  DBGLN("Config complete");
#endif
}

void LR1121Driver::ConfigModParamsFSK(uint32_t Bitrate, uint8_t BWF,
                                      uint32_t Fdev,
                                      SX12XX_Radio_Number_t radioNumber) {
  const uint8_t rxBandwidth =
      (BWF == LR11XX_RADIO_GFSK_BW_467000) ? LR20XX_GFSK_RX_BW_476_2 : BWF;
  uint8_t buf[9];
  buf[0] = Bitrate >> 24;
  buf[1] = Bitrate >> 16;
  buf[2] = Bitrate >> 8;
  buf[3] = Bitrate >> 0;
  buf[4] = LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF; // Pulse Shape - 0x00: No filter
                                              // applied
  buf[5] = rxBandwidth;
  buf[6] = Fdev >> 16;
  buf[7] = Fdev >> 8;
  buf[8] = Fdev >> 0;
  hal.WriteCommand(LR20XX_FSK_SET_MODULATION_PARAMS, buf, sizeof(buf),
                   radioNumber);

#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 GFSK mod params: br=%lu bw=0x%02X->0x%02X fdev=%lu",
        (unsigned long)Bitrate, (unsigned)BWF, (unsigned)rxBandwidth,
        (unsigned long)Fdev);
#endif
}

void LR1121Driver::SetPacketParamsFSK(uint8_t PreambleLength,
                                      uint8_t PayloadLength,
                                      SX12XX_Radio_Number_t radioNumber) {
  const uint8_t packetConfig =
      (uint8_t)((LR20XX_GFSK_ADDR_FILT_DISABLED << 2) |
                LR20XX_GFSK_PACKET_FORMAT_FIXED);
  const uint8_t crcWhitening =
      (uint8_t)((LR20XX_GFSK_CRC_OFF << 4) |
                LR20XX_GFSK_WHITENING_ENABLED);
  uint8_t buf[7];
  buf[0] = 0;
  buf[1] = PreambleLength;
#if SIW917_ELRS_LR2021_GFSK_DETECT_ON_SYNCWORD
  buf[2] = LR20XX_GFSK_PREAMBLE_DETECTOR_OFF;
#else
  buf[2] = LR20XX_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
#endif
  buf[3] = packetConfig;
  buf[4] = 0;
  buf[5] = PayloadLength;
  buf[6] = crcWhitening;
  hal.WriteCommand(LR20XX_FSK_SET_PACKET_PARAMS, buf, sizeof(buf),
                   radioNumber);

#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 GFSK packet params: pre=%u payload=%u det=%s packed=%02X %02X "
        "%02X %02X %02X %02X %02X",
        (unsigned)PreambleLength, (unsigned)PayloadLength,
        (buf[2] == LR20XX_GFSK_PREAMBLE_DETECTOR_OFF) ? "syncword" : "8bit",
        (unsigned)buf[0], (unsigned)buf[1], (unsigned)buf[2],
        (unsigned)buf[3], (unsigned)buf[4], (unsigned)buf[5],
        (unsigned)buf[6]);
#endif
}

void LR1121Driver::SetFSKWhiteningParams(
    SX12XX_Radio_Number_t radioNumber) {
  uint8_t buf[2] = {
      (uint8_t)((LR20XX_GFSK_WHITENING_TYPE_SX126X_LR11XX << 4) |
                ((LR20XX_GFSK_WHITENING_INIT >> 8) & 0x0F)),
      (uint8_t)LR20XX_GFSK_WHITENING_INIT,
  };
  hal.WriteCommand(LR20XX_FSK_SET_WHITENING_PARAMS, buf, sizeof(buf),
                   radioNumber);

#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 GFSK whitening: type=SX126X/LR11XX seed=0x%04X order=before-packet",
        (unsigned)LR20XX_GFSK_WHITENING_INIT);
#endif
}

void LR1121Driver::SetFSKSyncWord(uint8_t fskSyncWord1, uint8_t fskSyncWord2,
                                  SX12XX_Radio_Number_t radioNumber) {
  // v121 logs showed tentative/connection attempts with LR20xx's documented
  // "least significant bits" placement. Keep that known-better arrangement.
  uint8_t synbuf[9] = {0x00, 0x00, 0x00,         0x00,         0x00,
                       0x00, fskSyncWord1, fskSyncWord2, 0x00};
  synbuf[8] = LR20XX_GFSK_SYNC_MSB_FIRST | 16;
  hal.WriteCommand(LR20XX_FSK_SET_SYNCWORD, synbuf, sizeof(synbuf),
                   radioNumber);

#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 GFSK syncword: %02X %02X len=16 msb=1 align=lsb",
        (unsigned)fskSyncWord1, (unsigned)fskSyncWord2);
#endif
}

void LR1121Driver::SetRxPath(bool isSubGHz,
                             SX12XX_Radio_Number_t radioNumber) {
  uint8_t rxPath[2] = {
      (uint8_t)(isSubGHz ? LR20XX_RX_PATH_LF : LR20XX_RX_PATH_HF),
      (uint8_t)(isSubGHz ? 0 : 4),
  };
  hal.WriteCommand(LR20XX_RADIO_SET_RX_PATH, rxPath, sizeof(rxPath),
                   radioNumber);
}

void LR1121Driver::WriteRegMem32(uint32_t addr, uint32_t data,
                                 SX12XX_Radio_Number_t radioNumber) {
  uint8_t buf[7] = {
      (uint8_t)((addr >> 16) & 0xFF),
      (uint8_t)((addr >> 8) & 0xFF),
      (uint8_t)(addr & 0xFF),
      (uint8_t)((data >> 24) & 0xFF),
      (uint8_t)((data >> 16) & 0xFF),
      (uint8_t)((data >> 8) & 0xFF),
      (uint8_t)(data & 0xFF),
  };
  hal.WriteCommand(LR20XX_REGMEM_WRITE_REGMEM32, buf, sizeof(buf),
                   radioNumber);
}

uint32_t LR1121Driver::ReadRegMem32(uint32_t addr,
                                    SX12XX_Radio_Number_t radioNumber) {
  uint8_t req[4] = {
      (uint8_t)((addr >> 16) & 0xFF),
      (uint8_t)((addr >> 8) & 0xFF),
      (uint8_t)(addr & 0xFF),
      1,
  };
  uint8_t rsp[LR20XX_RESPONSE_STATUS_LEN + 4] = {};
  hal.WriteCommand(LR20XX_REGMEM_READ_REGMEM32, req, sizeof(req),
                   radioNumber);
  hal.ReadCommand(rsp, sizeof(rsp), radioNumber);
  return ((uint32_t)rsp[2] << 24) | ((uint32_t)rsp[3] << 16) |
         ((uint32_t)rsp[4] << 8) | (uint32_t)rsp[5];
}

void LR1121Driver::WriteRegMemMask32(uint32_t addr, uint32_t mask,
                                     uint32_t data,
                                     SX12XX_Radio_Number_t radioNumber) {
  uint8_t buf[11] = {
      (uint8_t)((addr >> 16) & 0xFF),
      (uint8_t)((addr >> 8) & 0xFF),
      (uint8_t)(addr & 0xFF),
      (uint8_t)((mask >> 24) & 0xFF),
      (uint8_t)((mask >> 16) & 0xFF),
      (uint8_t)((mask >> 8) & 0xFF),
      (uint8_t)(mask & 0xFF),
      (uint8_t)((data >> 24) & 0xFF),
      (uint8_t)((data >> 16) & 0xFF),
      (uint8_t)((data >> 8) & 0xFF),
      (uint8_t)(data & 0xFF),
  };
  hal.WriteCommand(LR20XX_REGMEM_WRITE_REGMEM_MASK32, buf, sizeof(buf),
                   radioNumber);
}

void LR1121Driver::ConfigureLoraSx1276Compatibility(
    uint8_t sf, SX12XX_Radio_Number_t radioNumber) {
  // Match Semtech's LR20xx workaround behavior. v91 tried native SF5, but that
  // made SF5 decode worse; keep SF5/SF6 on the documented workaround path.
  const bool enable = LR2021LoRaSx1276CompatForSf(sf);
  WriteRegMemMask32(LR20XX_LORA_SX1276_COMPAT_ADDR,
                    LR20XX_LORA_SX1276_COMPAT_MASK,
                    enable ? LR20XX_LORA_SX1276_COMPAT_ENABLE
                           : LR20XX_LORA_SX1276_COMPAT_DISABLE,
                    radioNumber);
#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 SX1276 LoRa compatibility %s for sf=%u",
        enable ? "enabled" : "disabled", (unsigned)sf);
#endif
}

void LR1121Driver::ConfigureLoraFrequencyRange(
    uint8_t sf, SX12XX_Radio_Number_t radioNumber) {
#if SIW917_ELRS_LR2021_LORA_SF5_FREQ_RANGE > 0
  if (sf == LR11XX_RADIO_LORA_SF5) {
    const uint32_t range =
        ((uint32_t)SIW917_ELRS_LR2021_LORA_SF5_FREQ_RANGE & 0x3UL) << 16;
    WriteRegMemMask32(LR20XX_LORA_RX_CFG_ADDR, LR20XX_LORA_FREQ_RANGE_MASK,
                      range, radioNumber);
#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
    DBGLN("LR2021 LoRa SF5 freq range set=%lu",
          (unsigned long)SIW917_ELRS_LR2021_LORA_SF5_FREQ_RANGE);
#endif
  }
#else
  (void)sf;
  (void)radioNumber;
#endif
}

void LR1121Driver::ApplyDcdcReset(SX12XX_Radio_Number_t radioNumber) {
  WriteRegMemMask32(LR20XX_DCDC_SWITCHER_ADDR, LR20XX_DCDC_SWITCHER_RISE_MASK,
                    15UL << 20, radioNumber);
  WriteRegMemMask32(LR20XX_DCDC_SWITCHER_ADDR, LR20XX_DCDC_SWITCHER_FALL_MASK,
                    15UL << 16, radioNumber);
  SetDcdcFrequency(2800000UL, radioNumber);
#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 DCDC reset workaround applied");
#endif
}

void LR1121Driver::ApplyDcdcConfigure(SX12XX_Radio_Number_t radioNumber) {
  const uint32_t adcCtrl = ReadRegMem32(LR20XX_DCDC_ADC_CTRL_ADDR, radioNumber);
  const uint32_t anaDec = (adcCtrl >> 8) & 0x7UL;
  const uint32_t rxPath = ReadRegMem32(LR20XX_DCDC_RX_PATH_ADDR, radioNumber);
  const bool isRxHf = ((rxPath & 0x3UL) == 1UL);

  if (!isRxHf && (anaDec == 1UL || anaDec == 2UL)) {
    WriteRegMemMask32(LR20XX_DCDC_SWITCHER_ADDR,
                      LR20XX_DCDC_SWITCHER_RISE_MASK, 11UL << 20,
                      radioNumber);
    WriteRegMemMask32(LR20XX_DCDC_SWITCHER_ADDR,
                      LR20XX_DCDC_SWITCHER_FALL_MASK, 13UL << 16,
                      radioNumber);
  } else {
    WriteRegMemMask32(LR20XX_DCDC_SWITCHER_ADDR,
                      LR20XX_DCDC_SWITCHER_RISE_MASK, 15UL << 20,
                      radioNumber);
    WriteRegMemMask32(LR20XX_DCDC_SWITCHER_ADDR,
                      LR20XX_DCDC_SWITCHER_FALL_MASK, 15UL << 16,
                      radioNumber);
  }

  SetDcdcFrequency((anaDec == 1UL) ? 4300000UL : 2800000UL, radioNumber);
#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 DCDC configure workaround applied adc=0x%08lX rx=0x%08lX",
        (unsigned long)adcCtrl, (unsigned long)rxPath);
#endif
}

void LR1121Driver::SetDcdcFrequency(uint32_t frequencyHz,
                                    SX12XX_Radio_Number_t radioNumber) {
  const uint32_t freqLf = (uint32_t)((uint64_t)frequencyHz * 1048576ULL /
                                    1000000ULL);
  WriteRegMem32(LR20XX_DCDC_FREQ_LF_ADDR, freqLf, radioNumber);

  const uint32_t rfPllSteps = ReadRegMem32(LR20XX_DCDC_RF_FREQ_ADDR,
                                           radioNumber);
  const uint32_t rfHz = LR20xxPllStepToHz(rfPllSteps);
  if (rfHz != 0) {
    SetFrequencyReg(rfHz, radioNumber, false);
  }
}

void LR1121Driver::ConfigureRegulatorMode(SX12XX_Radio_Number_t radioNumber) {
  uint8_t regMode[5] = {
      (uint8_t)(LR20XX_REG_MODE_SIMO_NORMAL | LR20XX_REG_MODE_RAMP_RES_4_US),
      0x00,
      0x00,
      0x00,
      0x00,
  };
  hal.WriteCommand(LR20XX_SYSTEM_SET_REG_MODE, regMode, sizeof(regMode),
                   radioNumber);
}

void LR1121Driver::SetDioFunctionIrq(SX12XX_Radio_Number_t radioNumber) {
  if (LR2021_IRQ_DIO_NUM < 5 || LR2021_IRQ_DIO_NUM > 11) {
    DBGLN("Invalid LR2021 IRQ DIO number: %u",
          (unsigned)LR2021_IRQ_DIO_NUM);
    return;
  }

  const uint8_t pull = (LR2021_IRQ_DIO_NUM == 5)
                           ? LR20XX_DIO_SLEEP_PULL_UP
                           : LR20XX_DIO_SLEEP_PULL_NONE;
  uint8_t dioFunction[2] = {
      (uint8_t)LR2021_IRQ_DIO_NUM,
      (uint8_t)(LR20XX_DIO_FUNCTION_IRQ | pull),
  };
  hal.WriteCommand(LR20XX_SYSTEM_SET_DIO_FUNCTION, dioFunction,
                   sizeof(dioFunction), radioNumber);
}

uint16_t LR1121Driver::GetErrors(SX12XX_Radio_Number_t radioNumber) {
  uint8_t buffer[LR20XX_RESPONSE_STATUS_LEN + 2] = {};
  hal.WriteCommand(LR20XX_SYSTEM_GET_ERRORS, radioNumber);
  hal.ReadCommand(buffer, sizeof(buffer), radioNumber);
  return (uint16_t)((buffer[2] << 8) | buffer[3]);
}

uint8_t LR1121Driver::GetPacketType(SX12XX_Radio_Number_t radioNumber) {
  uint8_t buffer[LR20XX_RESPONSE_STATUS_LEN + 1] = {};
  hal.WriteCommand(LR20XX_RADIO_GET_PACKET_TYPE, radioNumber);
  hal.ReadCommand(buffer, sizeof(buffer), radioNumber);
  return buffer[2];
}

void LR1121Driver::GetLoRaRxStats(SX12XX_Radio_Number_t radioNumber,
                                  uint16_t *pktRxTotal,
                                  uint16_t *pktCrcError,
                                  uint16_t *headerCrcError,
                                  uint16_t *falseSync) {
  uint8_t buffer[LR20XX_RESPONSE_STATUS_LEN + 8] = {};
  hal.WriteCommand(LR20XX_LORA_GET_RX_STATS, radioNumber);
  hal.ReadCommand(buffer, sizeof(buffer), radioNumber);
  const uint8_t *stats = buffer + LR20XX_RESPONSE_STATUS_LEN;

  if (pktRxTotal != nullptr) {
    *pktRxTotal = ((uint16_t)stats[0] << 8) | (uint16_t)stats[1];
  }
  if (pktCrcError != nullptr) {
    *pktCrcError = ((uint16_t)stats[2] << 8) | (uint16_t)stats[3];
  }
  if (headerCrcError != nullptr) {
    *headerCrcError = ((uint16_t)stats[4] << 8) | (uint16_t)stats[5];
  }
  if (falseSync != nullptr) {
    *falseSync = ((uint16_t)stats[6] << 8) | (uint16_t)stats[7];
  }
}

void LR1121Driver::GetGfskRxStats(SX12XX_Radio_Number_t radioNumber,
                                  uint16_t *pktRxTotal,
                                  uint16_t *pktCrcError,
                                  uint16_t *pktLenError,
                                  uint16_t *preambleDetected,
                                  uint16_t *syncOk, uint16_t *syncFail,
                                  uint16_t *timeout) {
  uint8_t buffer[LR20XX_RESPONSE_STATUS_LEN + 14] = {};
  hal.WriteCommand(LR20XX_FSK_GET_RX_STATS, radioNumber);
  hal.ReadCommand(buffer, sizeof(buffer), radioNumber);
  const uint8_t *stats = buffer + LR20XX_RESPONSE_STATUS_LEN;

  if (pktRxTotal != nullptr) {
    *pktRxTotal = ((uint16_t)stats[0] << 8) | (uint16_t)stats[1];
  }
  if (pktCrcError != nullptr) {
    *pktCrcError = ((uint16_t)stats[2] << 8) | (uint16_t)stats[3];
  }
  if (pktLenError != nullptr) {
    *pktLenError = ((uint16_t)stats[4] << 8) | (uint16_t)stats[5];
  }
  if (preambleDetected != nullptr) {
    *preambleDetected = ((uint16_t)stats[6] << 8) | (uint16_t)stats[7];
  }
  if (syncOk != nullptr) {
    *syncOk = ((uint16_t)stats[8] << 8) | (uint16_t)stats[9];
  }
  if (syncFail != nullptr) {
    *syncFail = ((uint16_t)stats[10] << 8) | (uint16_t)stats[11];
  }
  if (timeout != nullptr) {
    *timeout = ((uint16_t)stats[12] << 8) | (uint16_t)stats[13];
  }
}

void LR1121Driver::CalibrateAll(SX12XX_Radio_Number_t radioNumber) {
  uint8_t blocks = LR20XX_CALIBRATE_ALL;
  hal.WriteCommand(LR20XX_SYSTEM_CALIBRATE, &blocks, sizeof(blocks),
                   radioNumber);
  delay(5);

  const uint16_t errors = GetErrors(radioNumber);
  if (errors != 0) {
    DBGLN("LR2021 CalibrateAll device errors: 0x%04X", errors);
  }
}

uint16_t LR1121Driver::FrontEndCalWordForFrequency(uint32_t freqHz) {
  // LR20xx section 6.4.2 specifies 4 MHz units truncated to the lower
  // multiple. Rounding upward can leave the lowest FHSS channel below every
  // stored calibration and trigger RXFREQ_NO_FE_CAL_ERR.
  uint16_t calFreq = (uint16_t)(freqHz / 4000000UL);
  if (freqHz > LR20XX_LF_HF_CUTOFF_HZ) {
    calFreq |= LR20XX_CALIB_FE_HF_PATH;
  } else {
    calFreq |= LR20XX_CALIB_FE_LF_PATH;
  }
  return calFreq;
}

void LR1121Driver::ResetFrontEndCalCache() {
  feCalFreqRadio1 = 0;
  feCalFreqRadio2 = 0;
  feCalWordCountRadio1 = 0;
  feCalWordCountRadio2 = 0;
  memset(feCalWordsRadio1, 0, sizeof(feCalWordsRadio1));
  memset(feCalWordsRadio2, 0, sizeof(feCalWordsRadio2));
}

bool LR1121Driver::HasFrontEndCalWord(
    uint16_t calWord, SX12XX_Radio_Number_t radioNumber) const {
  bool covered = true;

  if (radioNumber & SX12XX_Radio_1) {
    bool radioCovered = false;
    for (uint8_t i = 0; i < feCalWordCountRadio1; ++i) {
      if (feCalWordsRadio1[i] == calWord) {
        radioCovered = true;
        break;
      }
    }
    covered &= radioCovered;
  }

  if (radioNumber & SX12XX_Radio_2) {
    bool radioCovered = false;
    for (uint8_t i = 0; i < feCalWordCountRadio2; ++i) {
      if (feCalWordsRadio2[i] == calWord) {
        radioCovered = true;
        break;
      }
    }
    covered &= radioCovered;
  }

  return covered;
}

bool LR1121Driver::HasFrontEndCalCoverage(
    uint16_t calWord, SX12XX_Radio_Number_t radioNumber) const {
  const uint16_t path = calWord & LR20XX_CALIB_FE_HF_PATH;
  const uint16_t bucket = calWord & LR20XX_FE_CAL_BUCKET_MASK;
  bool covered = true;

  auto wordCovers = [&](uint16_t cachedWord) -> bool {
    if ((cachedWord & LR20XX_CALIB_FE_HF_PATH) != path) {
      return false;
    }
    const uint16_t cachedBucket = cachedWord & LR20XX_FE_CAL_BUCKET_MASK;
    const uint16_t diff =
        cachedBucket > bucket ? (uint16_t)(cachedBucket - bucket)
                              : (uint16_t)(bucket - cachedBucket);
    return diff <= LR20XX_FE_CAL_COVERAGE_BUCKETS;
  };

  if (radioNumber & SX12XX_Radio_1) {
    bool radioCovered = false;
    for (uint8_t i = 0; i < feCalWordCountRadio1; ++i) {
      if (wordCovers(feCalWordsRadio1[i])) {
        radioCovered = true;
        break;
      }
    }
    covered &= radioCovered;
  }

  if (radioNumber & SX12XX_Radio_2) {
    bool radioCovered = false;
    for (uint8_t i = 0; i < feCalWordCountRadio2; ++i) {
      if (wordCovers(feCalWordsRadio2[i])) {
        radioCovered = true;
        break;
      }
    }
    covered &= radioCovered;
  }

  return covered;
}

void LR1121Driver::MarkFrontEndCalWord(
    uint16_t calWord, SX12XX_Radio_Number_t radioNumber) {
  if ((radioNumber & SX12XX_Radio_1) &&
      !HasFrontEndCalWord(calWord, SX12XX_Radio_1)) {
    if (feCalWordCountRadio1 < FE_CAL_CACHE_SIZE) {
      feCalWordsRadio1[feCalWordCountRadio1++] = calWord;
    } else {
      feCalWordsRadio1[FE_CAL_CACHE_SIZE - 1] = calWord;
    }
  }

  if ((radioNumber & SX12XX_Radio_2) &&
      !HasFrontEndCalWord(calWord, SX12XX_Radio_2)) {
    if (feCalWordCountRadio2 < FE_CAL_CACHE_SIZE) {
      feCalWordsRadio2[feCalWordCountRadio2++] = calWord;
    } else {
      feCalWordsRadio2[FE_CAL_CACHE_SIZE - 1] = calWord;
    }
  }
}

void LR1121Driver::SeedFrontEndCalibrationRange(
    uint32_t minimumFrequency, uint32_t maximumFrequency,
    SX12XX_Radio_Number_t radioNumber) {
  if (minimumFrequency == 0 || maximumFrequency == 0 ||
      maximumFrequency < minimumFrequency) {
    CalibrateFrontEndDefaultSet(radioNumber, true);
    return;
  }

  const uint32_t midFrequency =
      minimumFrequency + ((maximumFrequency - minimumFrequency) / 2U);
  const uint16_t calWords[3] = {
      FrontEndCalWordForFrequency(minimumFrequency),
      FrontEndCalWordForFrequency(midFrequency),
      FrontEndCalWordForFrequency(maximumFrequency),
  };
  CalibrateFrontEndWords(calWords, 3, radioNumber, true);
}

bool LR1121Driver::CalibrateFrontEndDefaultSet(
    SX12XX_Radio_Number_t radioNumber, bool force) {
  const uint16_t calWords[3] = {
      FrontEndCalWordForFrequency(LR20XX_FE_CAL_FCC915_LOW_HZ),
      FrontEndCalWordForFrequency(LR20XX_FE_CAL_FCC915_MID_HZ),
      FrontEndCalWordForFrequency(LR20XX_FE_CAL_FCC915_HIGH_HZ),
  };

  return CalibrateFrontEndWords(calWords, 3, radioNumber, force);
}

bool LR1121Driver::CalibrateFrontEnd24GSet(SX12XX_Radio_Number_t radioNumber,
                                           bool force) {
  const uint16_t calWords[3] = {
      FrontEndCalWordForFrequency(LR20XX_FE_CAL_ISM2G4_LOW_HZ),
      FrontEndCalWordForFrequency(LR20XX_FE_CAL_ISM2G4_MID_HZ),
      FrontEndCalWordForFrequency(LR20XX_FE_CAL_ISM2G4_HIGH_HZ),
  };

  return CalibrateFrontEndWords(calWords, 3, radioNumber, force);
}

bool LR1121Driver::CalibrateFrontEndWords(
    const uint16_t *calWords, uint8_t count, SX12XX_Radio_Number_t radioNumber,
    bool force) {
  if (calWords == nullptr || count == 0) {
    return false;
  }

  if (count > 3) {
    count = 3;
  }

  bool covered = true;
  for (uint8_t i = 0; i < count; ++i) {
    if (!HasFrontEndCalWord(calWords[i], radioNumber)) {
      covered = false;
      break;
    }
  }
  if (!force && covered) {
    return true;
  }

  uint8_t buf[6] = {};
  for (uint8_t i = 0; i < count; ++i) {
    buf[i * 2] = (uint8_t)(calWords[i] >> 8);
    buf[(i * 2) + 1] = (uint8_t)calWords[i];
  }

  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    hal.WriteCommand(LR20XX_SYSTEM_CLEAR_ERRORS, radioNumber);
    hal.WriteCommand(LR20XX_SYSTEM_CALIB_FE, buf, count * 2, radioNumber);
    if (!hal.WaitOnBusy(radioNumber)) {
      delayMicroseconds(SIW917_ELRS_LR2021_CALIB_FE_POST_DELAY_US);
    }

    const uint16_t errors = GetErrors(radioNumber);
    if (errors == 0) {
      if (radioNumber & SX12XX_Radio_1) {
        feCalWordCountRadio1 = 0;
      }
      if (radioNumber & SX12XX_Radio_2) {
        feCalWordCountRadio2 = 0;
      }
      for (uint8_t i = 0; i < count; ++i) {
        MarkFrontEndCalWord(calWords[i], radioNumber);
      }
      return true;
    }

    if ((errors & LR20XX_ERROR_SRC_SATURATION_CALIB) == 0) {
      DBGLN("LR2021 CalibFE set failed: words=0x%04X/0x%04X/0x%04X "
            "count=%u err=0x%04X",
            (unsigned)calWords[0], count > 1 ? (unsigned)calWords[1] : 0U,
            count > 2 ? (unsigned)calWords[2] : 0U, (unsigned)count,
            (unsigned)errors);
      return false;
    }

    delayMicroseconds(SIW917_ELRS_LR2021_CALIB_FE_POST_DELAY_US);
  }

  DBGLN("LR2021 CalibFE set failed after RSSI saturation retries");
  return false;
}

bool LR1121Driver::CalibrateFrontEndForFrequency(
    uint32_t freqHz, SX12XX_Radio_Number_t radioNumber, bool force) {
  if (freqHz == 0) {
    return false;
  }

  const uint16_t calFreq = FrontEndCalWordForFrequency(freqHz);
  if (!force && HasFrontEndCalCoverage(calFreq, radioNumber)) {
    return true;
  }

  const uint16_t path = calFreq & LR20XX_CALIB_FE_HF_PATH;
  const uint16_t bucket = calFreq & LR20XX_FE_CAL_BUCKET_MASK;
  const uint16_t startBucket = bucket > 0 ? (uint16_t)(bucket - 1U) : bucket;
  const uint16_t calWords[3] = {
      (uint16_t)(path | startBucket),
      (uint16_t)(path | (uint16_t)(startBucket + 1U)),
      (uint16_t)(path | (uint16_t)(startBucket + 2U)),
  };
  const bool ok = CalibrateFrontEndWords(calWords, 3, radioNumber, force);
  if (ok) {
    if (radioNumber & SX12XX_Radio_1) {
      feCalFreqRadio1 = freqHz;
    }
    if (radioNumber & SX12XX_Radio_2) {
      feCalFreqRadio2 = freqHz;
    }
  }
  return ok;
}

/***
 * @brief: Schedule an output power change after the next transmit
 ***/
void LR1121Driver::SetOutputPower(int8_t power, bool isSubGHz) {
  uint8_t pwrNew;

  if (isSubGHz) {
    if (OPT_USE_SX1276_RFO_HF) {
      pwrNew = constrain(power, LR1121_POWER_MIN_LP_PA, LR1121_POWER_MAX_LP_PA);
    } else {
      pwrNew = constrain(power, LR1121_POWER_MIN_HP_PA, LR1121_POWER_MAX_HP_PA);
    }

    if ((pwrPendingLF == PWRPENDING_NONE && pwrCurrentLF != pwrNew) ||
        pwrPendingLF != pwrNew) {
      pwrPendingLF = pwrNew;
    }
  } else {
    pwrNew = constrain(power, LR1121_POWER_MIN_HF_PA, LR1121_POWER_MAX_HF_PA);

    if ((pwrPendingHF == PWRPENDING_NONE && pwrCurrentHF != pwrNew) ||
        pwrPendingHF != pwrNew) {
      pwrPendingHF = pwrNew;
    }
  }
}

bool LR1121Driver::HasPendingOutputPower() const {
  return pwrPendingLF != PWRPENDING_NONE ||
         pwrPendingHF != PWRPENDING_NONE || pwrForceUpdate;
}

void ICACHE_RAM_ATTR LR1121Driver::CommitOutputPowerForNextTx() {
  CommitOutputPower();
}

void ICACHE_RAM_ATTR LR1121Driver::CommitOutputPower() {
  if (pwrPendingLF != PWRPENDING_NONE) {
    pwrCurrentLF = pwrPendingLF;
    pwrPendingLF = PWRPENDING_NONE;
    pwrForceUpdate = true;
  }

  if (pwrPendingHF != PWRPENDING_NONE) {
    pwrCurrentHF = pwrPendingHF;
    pwrPendingHF = PWRPENDING_NONE;
    pwrForceUpdate = true;
  }

  if (pwrForceUpdate) {
    WriteOutputPower(radio1isSubGHz ? pwrCurrentLF : pwrCurrentHF,
                     radio1isSubGHz, SX12XX_Radio_1);
    if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
      WriteOutputPower(radio2isSubGHz ? pwrCurrentLF : pwrCurrentHF,
                       radio2isSubGHz, SX12XX_Radio_2);
    }
    pwrForceUpdate = false;
  }
}

void ICACHE_RAM_ATTR LR1121Driver::WriteOutputPower(
    uint8_t power, bool isSubGHz, SX12XX_Radio_Number_t radioNumber) {
  const int8_t halfDbPower = (int8_t)((int8_t)power * 2);
  uint8_t Txbuf[2] = {(uint8_t)halfDbPower, LR20XX_RAMP_48_US};

  hal.WriteCommand(LR20XX_RADIO_SET_TX_PARAMS, Txbuf, sizeof(Txbuf),
                   radioNumber);
}

void ICACHE_RAM_ATTR
LR1121Driver::SetPaConfig(bool isSubGHz, SX12XX_Radio_Number_t radioNumber) {
  uint8_t Pabuf[3] = {0};

  if (isSubGHz) {
    Pabuf[0] = 0x00; // LF PA, full single-ended mode.
    Pabuf[1] = 0x66; // 915 MHz reference-design +20 dBm duty/slices.
    Pabuf[2] = 0x10; // HF PA unused default.
  } else {
    Pabuf[0] = 0x80; // HF PA.
    Pabuf[1] = 0x76; // LF PA unused default.
    Pabuf[2] = 0x10; // 2445 MHz reference-design +12 dBm setting.
  }

  hal.WriteCommand(LR20XX_RADIO_SET_PA_CONFIG, Pabuf, sizeof(Pabuf),
                   radioNumber);
  uint8_t selPa[1] = {
      (uint8_t)(isSubGHz ? LR20XX_PA_SEL_LF : LR20XX_PA_SEL_HF)};
  hal.WriteCommand(LR20XX_RADIO_SEL_PA, selPa, sizeof(selPa), radioNumber);
}

void SIW917_ELRS_RAMFUNC_ATTR LR1121Driver::SetMode(
    lr11xx_RadioOperatingModes_t OPmode, SX12XX_Radio_Number_t radioNumber) {
  WORD_ALIGNED_ATTR uint8_t buf[5] = {0};

  switch (OPmode) {
  case LR1121_MODE_SLEEP:
    // 2.1.5.1 SetSleep
    rxContinuousActive = false;
    txInProgress = false;
    hal.WriteCommand(LR20XX_SYSTEM_SET_SLEEP, buf, 5, radioNumber);
    break;

  case LR1121_MODE_STDBY_RC:
    // 2.1.2.1 SetStandby
    rxContinuousActive = false;
    txInProgress = false;
    buf[0] = 0x00;
    hal.WriteCommand(LR20XX_SYSTEM_SET_STANDBY, buf, 1, radioNumber);
    break;

  case LR1121_MODE_STDBY_XOSC:
    // 2.1.2.1 SetStandby
    rxContinuousActive = false;
    txInProgress = false;
    buf[0] = 0x01;
    hal.WriteCommand(LR20XX_SYSTEM_SET_STANDBY, buf, 1, radioNumber);
    break;

  case LR1121_MODE_FS:
    // 2.1.9.1 SetFs
    rxContinuousActive = false;
    txInProgress = false;
    hal.WriteCommand(LR20XX_SYSTEM_SET_FS, radioNumber);
    break;

  case LR1121_MODE_RX_CONT: {
    // 7.2.2 SetRx - Continuous RX mode (0xFFFFFF = no timeout)
    static bool firstRx = true;
    if (firstRx) {
      DBGLN("SetRx(0xFFFFFF) - Entering continuous RX mode");
      firstRx = false;
    }
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0xFF; // Continuous RX
    hal.WriteCommand(LR20XX_RADIO_SET_RX, buf, 3, radioNumber);
    rxContinuousActive = true;
    txInProgress = false;
    break;
  }

  case LR1121_MODE_TX:
    // Table 7-3: SetTx Command
    rxContinuousActive = false;
    txInProgress = true;
    hal.WriteCommand(LR20XX_RADIO_SET_TX, buf, 3, radioNumber);
    break;

  case LR1121_MODE_CAD:
    break;

  default:
    break;
  }
}

void LR1121Driver::ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr,
                                       SX12XX_Radio_Number_t radioNumber) {
  uint8_t buf[2] = {
      LR20XX_LORA_MOD_PARAMS(sf, bw),
      LR20XX_LORA_MOD_CR_LDRO(cr, LR20XX_LORA_LDRO_OFF),
  };
  hal.WriteCommand(LR20XX_LORA_SET_MODULATION_PARAMS, buf, sizeof(buf),
                   radioNumber);

#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 LoRa mod params: sf=%u bw=%u cr=%u packed=%02X %02X",
        (unsigned)sf, (unsigned)bw, (unsigned)cr, (unsigned)buf[0],
        (unsigned)buf[1]);
#endif

  if (radioNumber & SX12XX_Radio_1 && radio1isSubGHz)
    ConfigureLoraSx1276Compatibility(sf, SX12XX_Radio_1);

  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    if (radioNumber & SX12XX_Radio_2 && radio2isSubGHz)
      ConfigureLoraSx1276Compatibility(sf, SX12XX_Radio_2);
  }

  ConfigureLoraFrequencyRange(sf, radioNumber);
}

void LR1121Driver::SetPacketParamsLoRa(
    uint8_t PreambleLength, lr11xx_RadioLoRaPacketLengthsModes_t HeaderType,
    uint8_t PayloadLength, uint8_t InvertIQ,
    SX12XX_Radio_Number_t radioNumber) {
  const uint8_t packetConfig =
      (uint8_t)((HeaderType == LR1121_LORA_PACKET_IMPLICIT
                     ? LR20XX_LORA_HEADER_IMPLICIT
                     : LR20XX_LORA_HEADER_EXPLICIT) |
                LR20XX_LORA_CRC_OFF |
                (InvertIQ ? LR20XX_LORA_IQ_INVERTED
                          : LR20XX_LORA_IQ_STANDARD));
  uint8_t buf[4] = {
      0x00,
      PreambleLength,
      PayloadLength,
      packetConfig,
  };
  hal.WriteCommand(LR20XX_LORA_SET_PACKET_PARAMS, buf, sizeof(buf),
                   radioNumber);

#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 LoRa packet params: pre=%u header=%u payload=%u iq=%u "
        "packed=%02X %02X %02X %02X",
        (unsigned)PreambleLength, (unsigned)HeaderType,
        (unsigned)PayloadLength, (unsigned)InvertIQ, (unsigned)buf[0],
        (unsigned)buf[1], (unsigned)buf[2], (unsigned)buf[3]);
#endif
}

void LR1121Driver::SetLoRaSyncWord(uint8_t syncWord, uint8_t sf,
                                   SX12XX_Radio_Number_t radioNumber) {
  (void)sf;
#if SIW917_ELRS_LR2021_LORA_COMPAT_EXT_SYNCWORD
  if (syncWord == LR20XX_LORA_SYNC_WORD_PRIVATE) {
    uint8_t buf[2] = {LR20XX_LORA_SYNC_WORD_PRIVATE_EXT_1,
                      LR20XX_LORA_SYNC_WORD_PRIVATE_EXT_2};
    hal.WriteCommand(LR20XX_LORA_SET_SYNCWORD_EXT, buf, sizeof(buf),
                     radioNumber);

#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
    DBGLN("LR2021 LoRa syncword-ext: private (%u,%u) sf=%u",
          (unsigned)buf[0], (unsigned)buf[1], (unsigned)sf);
#endif
    return;
  }
#endif

  uint8_t buf[1] = {syncWord};
  hal.WriteCommand(LR20XX_LORA_SET_SYNCWORD, buf, sizeof(buf), radioNumber);

#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 LoRa syncword: 0x%02X sf=%u", (unsigned)syncWord,
        (unsigned)sf);
#endif
}

void LR1121Driver::SetRxTimeoutStopOnPreamble(
    bool stopOnPreamble, SX12XX_Radio_Number_t radioNumber) {
  uint8_t buf[1] = {static_cast<uint8_t>(stopOnPreamble ? 0x01U : 0x00U)};
  hal.WriteCommand(LR20XX_RADIO_STOP_TIMEOUT_ON_PREAMBLE, buf, sizeof(buf),
                   radioNumber);
}

void LR1121Driver::ConfigureLoRaRxDetector(
    uint8_t sf, uint8_t InvertIQ, SX12XX_Radio_Number_t radioNumber) {
#if defined(SIW917_ELRS_LR2021_FORCE_LORA_DETECTOR_DISABLE) &&                \
    SIW917_ELRS_LR2021_FORCE_LORA_DETECTOR_DISABLE
  uint8_t searchSymbols[2] = {0, LR20XX_LORA_SEARCH_SYMBOL_FORMAT_NUMBER};
  hal.WriteCommand(LR20XX_LORA_SET_SYNC_TIMEOUT, searchSymbols,
                   sizeof(searchSymbols), radioNumber);

  // SetModulationParams disables LR20xx side detectors, but make that state
  // explicit during rate changes so the main detector owns the selected SF.
  hal.WriteCommand(LR20XX_LORA_SET_SIDE_DET_CONFIG, radioNumber);

#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 LoRa detector: sf=%u iq=%u search=disabled side=disabled",
        (unsigned)sf, (unsigned)InvertIQ);
#else
  (void)sf;
  (void)InvertIQ;
#endif
#else
#if defined(SIW917_ELRS_RADIO_INIT_VERBOSE) && SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR2021 LoRa detector: sf=%u iq=%u main-default",
        (unsigned)sf, (unsigned)InvertIQ);
#else
  (void)sf;
  (void)InvertIQ;
#endif
  (void)radioNumber;
#endif
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
LR1121Driver::SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber,
                              bool doRx, uint32_t rxTime) {
  const uint32_t timeout = rxTime != 0 ? rxTime : 0xFFFFFFU;
  const uint32_t radioFreq = siw917Lr2021ApplyFreqOffset(freq);
  uint8_t buf[7] = {
      (uint8_t)(radioFreq >> 24),
      (uint8_t)(radioFreq >> 16),
      (uint8_t)(radioFreq >> 8),
      (uint8_t)(radioFreq),
      (uint8_t)(timeout >> 16),
      (uint8_t)(timeout >> 8),
      (uint8_t)(timeout),
  };
#if defined(SIW917_ELRS_LR2021_RUNTIME_CALIB_FE) &&                            \
    SIW917_ELRS_LR2021_RUNTIME_CALIB_FE
  CalibrateFrontEndForFrequency(radioFreq, radioNumber);
#endif
  if (doRx) {
    const bool frequencySet = hal.WriteCommandFastRetry(
        LR20XX_RADIO_SET_RF_FREQUENCY, buf, 4, radioNumber);
    const bool rxSet = frequencySet && hal.WriteCommandFastRetry(
                                           LR20XX_RADIO_SET_RX, buf + 4, 3,
                                           radioNumber);
#if defined(SIW917_ELRS_LR2021_RX_FE_CAL_RETRY) &&                             \
    SIW917_ELRS_LR2021_RX_FE_CAL_RETRY
    const uint16_t rxErrors = GetErrors(radioNumber);
    if ((rxErrors & LR20XX_ERROR_RXFREQ_NO_FE_CAL) != 0) {
      const uint16_t calWord = FrontEndCalWordForFrequency(radioFreq);
      siw917_fe_cal_missing_log_count++;
      if (siw917_fe_cal_missing_log_count <= 8 ||
          (siw917_fe_cal_missing_log_count & 0x1FU) == 0) {
        DBGLN("LR2021 SetRx missing FE cal: freq=%luHz word=0x%04X err=0x%04X "
              "radio=%u count=%lu",
              (unsigned long)radioFreq, (unsigned)calWord, (unsigned)rxErrors,
              (unsigned)radioNumber,
              (unsigned long)siw917_fe_cal_missing_log_count);
      }
      if (CalibrateFrontEndForFrequency(radioFreq, radioNumber, true)) {
        hal.WriteCommand(LR20XX_SYSTEM_CLEAR_ERRORS, radioNumber);
        hal.WriteCommand(LR20XX_RADIO_SET_RF_FREQUENCY, buf, 4,
                         radioNumber);
        hal.WriteCommand(LR20XX_RADIO_SET_RX, buf + 4, 3,
                         radioNumber);
        const uint16_t retryErrors = GetErrors(radioNumber);
        if ((retryErrors & LR20XX_ERROR_RXFREQ_NO_FE_CAL) != 0) {
          DBGLN("LR2021 SetRx FE cal retry still failed: freq=%luHz err=0x%04X",
                (unsigned long)radioFreq, (unsigned)retryErrors);
        }
      }
    }
#endif
    rxContinuousActive = rxSet;
  } else {
    hal.WriteCommandFastRetry(LR20XX_RADIO_SET_RF_FREQUENCY, buf, 4,
                              radioNumber);
  }

  currFreq = radioFreq;
}

// LR20xx SetDioIrqConfig routes selected IRQs to one LR20xx DIO output.
// Physical wiring (Waveshare Core2021-XF):
//   LR2021 DIO11 -> SiW917 GPIO_46 (HP domain, rising-edge interrupt)
void LR1121Driver::SetDioIrqParams() {
  uint8_t buf[5] = {0};

  uint32_t irqMask = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE;
#if defined(SIW917_ELRS_LR2021_RX_FIFO_AS_RX_DONE) &&                         \
    SIW917_ELRS_LR2021_RX_FIFO_AS_RX_DONE
  irqMask |= LR20XX_IRQ_RX_FIFO;
#endif
#if defined(SIW917_ELRS_DISCONNECTED_SCAN_DIAG) &&                            \
        SIW917_ELRS_DISCONNECTED_SCAN_DIAG &&                                 \
    defined(SIW917_ELRS_LR2021_ROUTE_SCAN_DIAG_IRQS) &&                       \
        SIW917_ELRS_LR2021_ROUTE_SCAN_DIAG_IRQS
  irqMask |= LR20XX_IRQ_PREAMBLE_DETECTED | LR20XX_IRQ_LORA_HEADER_VALID |
             LR20XX_IRQ_LORA_HEADER_CRC_ERROR | LR20XX_IRQ_SYNC_FAIL |
             LR1121_IRQ_TIMEOUT | LR20XX_IRQ_CRC_ERROR | LR20XX_IRQ_LEN_ERROR |
             LR20XX_IRQ_ADDR_ERROR;
#endif

  buf[0] = LR2021_IRQ_DIO_NUM;
  buf[1] = (irqMask >> 24) & 0xFF;
  buf[2] = (irqMask >> 16) & 0xFF;
  buf[3] = (irqMask >> 8) & 0xFF;
  buf[4] = irqMask & 0xFF;

  hal.WriteCommand(LR20XX_SYSTEM_SET_DIO_IRQ_CONFIG, buf, sizeof(buf),
                   SX12XX_Radio_All);

#if SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("SetDioIrqConfig: mask=0x%08lX routed to DIO%u",
        (unsigned long)irqMask, (unsigned)LR2021_IRQ_DIO_NUM);
#endif
}

uint32_t SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
LR1121Driver::GetIrqStatus(SX12XX_Radio_Number_t radioNumber) {
#if SIW917_ELRS_FAST_CLEAR_IRQ
  uint32_t irqStatus = 0;
  if (lr1121_clear_irq_status_fast(0xFFFFFFFFU, &irqStatus)) {
    return irqStatus;
  }
#if SIW917_ELRS_STRICT_BARE_METAL_HOTPATH
  if (connectionState != disconnected) {
    return 0;
  }
#endif
#endif

  uint8_t status[LR20XX_RESPONSE_STATUS_LEN + 4] = {0};

  hal.WriteCommand(LR20XX_SYSTEM_GET_AND_CLEAR_IRQ_STATUS, radioNumber);
  hal.ReadCommand(status, sizeof(status), radioNumber);

  // LR20xx normal command responses include two status bytes followed by data.
  return (uint32_t)status[2] << 24 | (uint32_t)status[3] << 16 |
         (uint32_t)status[4] << 8 | (uint32_t)status[5];
}

uint32_t SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
LR1121Driver::PeekIrqStatus(SX12XX_Radio_Number_t radioNumber) {
  uint8_t status[LR20XX_RESPONSE_STATUS_LEN + 4] = {0};

  // LR20xx GetStatus is a direct read, not a command/read pair. It exposes the
  // current IRQ latch without clearing RX_FIFO/RX_TIMESTAMP/CMD progress bits.
  hal.WaitOnBusy(radioNumber);
  lr1121_select_radio(LR2021SpiRadioForMask(radioNumber));
  lr1121_cs_assert();
  const bool ok = lr1121_spi_transfer_raw(nullptr, status, sizeof(status));
  lr1121_cs_deassert();
  lr1121_select_radio(LR1121_RADIO_1);
  if (!ok) {
    return 0;
  }

  return (uint32_t)status[2] << 24 | (uint32_t)status[3] << 16 |
         (uint32_t)status[4] << 8 | (uint32_t)status[5];
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
LR1121Driver::ClearIrqStatus(SX12XX_Radio_Number_t radioNumber) {
  ClearIrqStatusMask(0xFFFFFFFFU, radioNumber);
}

void LR1121Driver::ClearRxFifo(SX12XX_Radio_Number_t radioNumber) {
  hal.WriteCommand(LR20XX_SYSTEM_CLEAR_RX_FIFO, radioNumber);
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR LR1121Driver::ClearIrqStatusMask(
    uint32_t irqMask, SX12XX_Radio_Number_t radioNumber) {
#if SIW917_ELRS_FAST_CLEAR_IRQ
  if (lr1121_clear_irq_status_fast(irqMask, nullptr)) {
    return;
  }
#if SIW917_ELRS_STRICT_BARE_METAL_HOTPATH
  if (connectionState != disconnected) {
    return;
  }
#endif
#endif

  // Clear IRQ status command takes 4 bytes of masks.
  uint8_t buf[4] = {
      (uint8_t)(irqMask >> 24),
      (uint8_t)(irqMask >> 16),
      (uint8_t)(irqMask >> 8),
      (uint8_t)irqMask,
  };
  hal.WriteCommand(LR20XX_SYSTEM_CLEAR_IRQ, buf, sizeof(buf), radioNumber);
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR LR1121Driver::TXnbISR() {
#ifdef DEBUG_LR1121_OTA_TIMING
  endTX = micros();
  DBGLN("TOA: %d", endTX - beginTX);
#endif
  txInProgress = false;
#if defined(SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX) &&                            \
    SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX
  if (autoRxAfterTxArmed) {
    rxContinuousActive = true;
  }
#endif
#if !defined(PLATFORM_SIW917)
  CommitOutputPower();
#else
  // SiW917 commits pending SetTxParams before TXnb; doing it after TX_DONE can
  // collide with the tight RX turn-around and BUSY-timeout the SPI command.
#endif
  TXdoneCallback();
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR LR1121Driver::TXnb(
    uint8_t *data, const bool sendGeminiBuffer, uint8_t *dataGemini,
    const SX12XX_Radio_Number_t radioNumber) {
  transmittingRadio = radioNumber;
  lastTxStartSuccessful = false;
#if defined(SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX) &&                            \
    SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX
  autoRxAfterTxArmed = false;
#endif

  // //catch TX timeout
  // if (currOpmode == SX1280_MODE_TX)
  // {
  //     DBGLN("Timeout!");
  //     SetMode(SX1280_MODE_FS, SX12XX_Radio_All);
  //     ClearIrqStatus(SX1280_IRQ_RADIO_ALL, SX12XX_Radio_All);
  //     TXnbISR();
  //     return;
  // }

  if (radioNumber == SX12XX_Radio_NONE) {
    SetMode(fallBackMode, SX12XX_Radio_All);
    return;
  }

#if defined(DEBUG_RCVR_SIGNAL_STATS)
  if (radioNumber == SX12XX_Radio_All || radioNumber == SX12XX_Radio_1) {
    rxSignalStats[0].telem_count++;
  }
  if (radioNumber == SX12XX_Radio_All || radioNumber == SX12XX_Radio_2) {
    rxSignalStats[1].telem_count++;
  }
#endif

  // Normal diversity mode
  if (GPIO_PIN_NSS_2 != UNDEF_PIN && radioNumber != SX12XX_Radio_All) {
    // Make sure the unused radio is in FS mode and will not receive the tx
    // packet.
    if (radioNumber == SX12XX_Radio_1) {
      SetMode(fallBackMode, SX12XX_Radio_2);
    } else {
      SetMode(fallBackMode, SX12XX_Radio_1);
    }
  }

#if defined(SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX) &&                            \
    SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX
  const bool enableAutoRxAfterTx =
#if SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX_LOCKED_ONLY
      connectionState == connected && RXtimerState == tim_locked;
#else
      true;
#endif
  if (enableAutoRxAfterTx) {
    // SetAutoRxTx runs the automatic operation from the configured fallback
    // mode. Leave continuous RX before arming it so the following manual TX
    // starts the documented FS -> TX -> FS -> auto-RX sequence.
    if (rxContinuousActive) {
      SetMode(fallBackMode, radioNumber);
    }

    // Let LR2021 return to continuous RX at the exact TX_DONE boundary. The
    // callback still runs to clear ELRS state, but RX is no longer gated by
    // SiW917 task/IRQ latency once the receiver has acquired timing lock.
    const uint8_t autoRxAfterTx[8] = {
        0x01, // condition: always run the automatic operation after manual TX
        0xFF, 0xFF, 0xFF, // automatic RX timeout: continuous
        0x00, 0x00, 0x00, 0x00, // delay_in_tick: immediate
    };
    hal.WriteCommand(LR20XX_RADIO_SET_AUTO_RX_TX,
                     const_cast<uint8_t *>(autoRxAfterTx),
                     sizeof(autoRxAfterTx), radioNumber);
    autoRxAfterTxArmed = true;
  }
#endif

  WORD_ALIGNED_ATTR uint8_t outBuffer[32];
  codec->encode(outBuffer, data, PayloadLength);
  outBuffer[PayloadLength] = 0;
  outBuffer[PayloadLength + 1] = 0;
  outBuffer[PayloadLength + 2] = 0;

  // Do not let an RX_DONE from the preceding continuous-RX state be treated
  // as TX_DONE while the auto-RX sequence is being prepared.
  txInProgress = true;
  rxContinuousActive = false;

  bool txCommandsOk = false;

  if (sendGeminiBuffer) {
    WORD_ALIGNED_ATTR uint8_t outBufferGemini[32];
    codec->encode(outBufferGemini, dataGemini, PayloadLength);
    outBufferGemini[PayloadLength] = 0;
    outBufferGemini[PayloadLength + 1] = 0;
    outBufferGemini[PayloadLength + 2] = 0;

    // Keep the two TX start command groups adjacent; encoding between them
    // widens Gemini skew inside the already-tight telemetry slot.
    const bool radio1FifoOk = hal.WriteCommandFastRetry(
        LR20XX_CMD_WRITE_RADIO_TX_FIFO, outBuffer, PayloadLength,
        SX12XX_Radio_1);
    const bool radio1TxOk =
        radio1FifoOk && hal.WriteCommandFastRetry(
                            LR20XX_RADIO_SET_TX, outBuffer + PayloadLength, 3,
                            SX12XX_Radio_1);
    const bool radio2FifoOk = hal.WriteCommandFastRetry(
        LR20XX_CMD_WRITE_RADIO_TX_FIFO, outBufferGemini, PayloadLength,
        SX12XX_Radio_2);
    const bool radio2TxOk =
        radio2FifoOk && hal.WriteCommandFastRetry(
                            LR20XX_RADIO_SET_TX,
                            outBufferGemini + PayloadLength, 3,
                            SX12XX_Radio_2);
    txCommandsOk = radio1TxOk && radio2TxOk;
  } else {
    const bool fifoOk = hal.WriteCommandFastRetry(
        LR20XX_CMD_WRITE_RADIO_TX_FIFO, outBuffer, PayloadLength, radioNumber);
    txCommandsOk =
        fifoOk && hal.WriteCommandFastRetry(
                      LR20XX_RADIO_SET_TX, outBuffer + PayloadLength, 3,
                      radioNumber);
  }

  lastTxStartSuccessful = txCommandsOk;
  if (!txCommandsOk) {
    txInProgress = false;
  }
#ifdef DEBUG_LLCC68_OTA_TIMING
  beginTX = micros();
#endif
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR LR1121Driver::DecodeRssiSnr(
    SX12XX_Radio_Number_t radioNumber, const uint8_t *buf) {
  uint16_t rawRssi = 0;
  int8_t snrRaw = 0;

  if (useFSK) {
    // LR20xx GFSK: status[2], len[2], rssi_avg_msb, rssi_sync_msb,
    // flags/lsbs, lqi.
    rawRssi =
        ((uint16_t)buf[4] << 1) | (uint16_t)((buf[6] & 0x04) >> 2);
  } else {
    // LR20xx LoRa: status[2], crc/cr, len, snr*4, rssi_pkt_msb,
    // rssi_signal_msb, lsbs.
    snrRaw = (int8_t)buf[4];
    rawRssi =
        ((uint16_t)buf[5] << 1) | (uint16_t)((buf[7] & 0x02) >> 1);
  }

  int16_t rssi16 = -(int16_t)(rawRssi / 2);
  if (rssi16 < -128) {
    rssi16 = -128;
  }
  const int8_t rssi = (int8_t)rssi16;

  // RssiPkt defines the average RSSI over the last packet received. RSSI value
  // in dBm is –RssiPkt/2.

  // SignalRssiPkt is an estimation of RSSI of the LoRa signal (after
  // despreading) on last packet received, in two’s complement format [negated,
  // dBm, fixdt(0,8,1)]. Actual RSSI in dB is -SignalRssiPkt/2. rssi[i =
  // -(int8_t)(status[3] / 2); // SignalRssiPkt

  // If radio # is 0, update LastPacketRSSI, otherwise LastPacketRSSI2
  radioNumber == SX12XX_Radio_1 ? LastPacketRSSI = rssi
                                : LastPacketRSSI2 = rssi;

  // Update whatever SNRs we have
  LastPacketSNRRaw = snrRaw;

#if defined(DEBUG_RCVR_SIGNAL_STATS)
  // stat updates
  int i = radioNumber == SX12XX_Radio_1 ? 0 : 1;
  rxSignalStats[i].irq_count++;
  rxSignalStats[i].rssi_sum += rssi;
  rxSignalStats[i].snr_sum += LastPacketSNRRaw;
  if (LastPacketSNRRaw > rxSignalStats[i].snr_max) {
    rxSignalStats[i].snr_max = LastPacketSNRRaw;
  }
#endif
}

bool SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
LR1121Driver::RXnbISR(SX12XX_Radio_Number_t radioNumber) {
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  siw917_rxnbisr_entry_us = micros();
#endif
  const uint8_t effectivePayloadLength =
      PayloadLength != 0 ? PayloadLength : siw917_last_payload_length;
  uint8_t fifoPayloadOffset = siw917_rx_fifo_payload_offset;
  if (fifoPayloadOffset < LR2021_RX_FIFO_COMMAND_PHASE_BYTES ||
      fifoPayloadOffset >
          (LR2021_RX_FIFO_COMMAND_PHASE_BYTES +
           LR2021_RX_FIFO_DISCONNECTED_GUARD_BYTES)) {
    fifoPayloadOffset = LR2021_RX_FIFO_COMMAND_PHASE_BYTES;
  }
  uint8_t rxReadLength = (uint8_t)(effectivePayloadLength + fifoPayloadOffset);
#if defined(SIW917_ELRS_DISCONNECTED_SCAN_DIAG) &&                              \
    SIW917_ELRS_DISCONNECTED_SCAN_DIAG
  if (!useFSK && connectionState == disconnected) {
    const uint8_t guardedReadLength =
        (uint8_t)(effectivePayloadLength + LR2021_RX_FIFO_COMMAND_PHASE_BYTES +
                  LR2021_RX_FIFO_DISCONNECTED_GUARD_BYTES);
    if (guardedReadLength > rxReadLength) {
      rxReadLength = guardedReadLength;
    }
  }
#endif
  if (rxReadLength > sizeof(rx_buf)) {
    rxReadLength = sizeof(rx_buf);
  }
  bool packetAccepted = false;

  rx_buf[0] = (uint8_t)(LR20XX_CMD_READ_RADIO_RX_FIFO >> 8);
  rx_buf[1] = (uint8_t)(LR20XX_CMD_READ_RADIO_RX_FIFO & 0xFF);
  hal.ReadCommand(rx_buf, rxReadLength, radioNumber);
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  siw917_packet_ready_us = micros();
#endif

#if defined(SIW917_ELRS_DISCONNECTED_SCAN_DIAG) &&                              \
    SIW917_ELRS_DISCONNECTED_SCAN_DIAG
  uint8_t crcOffset = 0xFF;
  uint8_t crcNonce = 0xFF;
  uint8_t crcType = 0xFF;
  uint8_t syncOffset = 0xFF;
  uint8_t syncNonce = 0xFF;
  uint8_t syncType = 0xFF;
  bool crcAny = false;
  bool crcSync = false;
  if (!useFSK && connectionState == disconnected) {
    crcSync =
        siw917FindOtaCrcMatch(rx_buf, rxReadLength, effectivePayloadLength,
                              true, &syncOffset, &syncNonce, &syncType);
    crcAny = siw917FindOtaCrcMatch(rx_buf, rxReadLength, effectivePayloadLength,
                                   false, &crcOffset, &crcNonce, &crcType);
    if (crcSync && syncOffset >= LR2021_RX_FIFO_COMMAND_PHASE_BYTES &&
        ((uint16_t)syncOffset + effectivePayloadLength) <= rxReadLength) {
      fifoPayloadOffset = syncOffset;
      siw917_rx_fifo_payload_offset = syncOffset;
    }
  }
#endif
  if (((uint16_t)fifoPayloadOffset + effectivePayloadLength) > rxReadLength) {
    fifoPayloadOffset = LR2021_RX_FIFO_COMMAND_PHASE_BYTES;
  }
  codec->decode(RXdataBuffer, rx_buf + fifoPayloadOffset,
                effectivePayloadLength);
  packetAccepted = RXdoneCallback(SX12XX_RX_OK);
#if SIW917_ELRS_TLM_MISS_IRQ_TRACE
  if (!packetAccepted && lr1121_tlm_miss_irq_trace_active != 0U) {
    tlmMissIrqTracePacketRejectCount++;
  }
#endif
  if (!packetAccepted) {
#if defined(DEBUG_RCVR_SIGNAL_STATS)
    rxSignalStats[radioNumber == SX12XX_Radio_1 ? 0 : 1].fail_count++;
#endif
#if defined(SIW917_ELRS_DISCONNECTED_SCAN_DIAG) &&                              \
    SIW917_ELRS_DISCONNECTED_SCAN_DIAG
    if (SIW917_ELRS_LR2021_LORA_FAIL_DUMP && !useFSK &&
        connectionState == disconnected &&
        siw917_lora_fail_dump_count < 4) {
      WORD_ALIGNED_ATTR uint8_t pktStatus[LR20XX_RESPONSE_STATUS_LEN + 6] = {};
      WORD_ALIGNED_ATTR uint8_t pktLength[LR20XX_RESPONSE_STATUS_LEN + 2] = {};
      uint8_t fifoDump[21] = {};
      const uint8_t fifoDumpLen =
          rxReadLength < sizeof(fifoDump)
              ? rxReadLength
              : (uint8_t)sizeof(fifoDump);
      for (uint8_t i = 0; i < fifoDumpLen; i++) {
        fifoDump[i] = rx_buf[i];
      }
      hal.WriteCommand(LR20XX_LORA_GET_PACKET_STATUS, radioNumber);
      hal.ReadCommand(pktStatus, sizeof(pktStatus), radioNumber);
      hal.WriteCommand(LR20XX_RADIO_GET_RX_PKT_LENGTH, radioNumber);
      hal.ReadCommand(pktLength, sizeof(pktLength), radioNumber);
      const uint16_t rxPktLength =
          ((uint16_t)pktLength[2] << 8) | (uint16_t)pktLength[3];
      DBGLN("LORA_FAIL raw len=%u fifo:%02X %02X %02X %02X %02X %02X %02X "
            "%02X %02X %02X %02X %02X %02X %02X %02X tail:%02X %02X %02X "
            "%02X %02X %02X",
            (unsigned)effectivePayloadLength, (unsigned)fifoDump[0],
            (unsigned)fifoDump[1], (unsigned)fifoDump[2],
            (unsigned)fifoDump[3], (unsigned)fifoDump[4],
            (unsigned)fifoDump[5], (unsigned)fifoDump[6],
            (unsigned)fifoDump[7], (unsigned)fifoDump[8],
            (unsigned)fifoDump[9], (unsigned)fifoDump[10],
            (unsigned)fifoDump[11], (unsigned)fifoDump[12],
            (unsigned)fifoDump[13], (unsigned)fifoDump[14],
            (unsigned)fifoDump[15], (unsigned)fifoDump[16],
            (unsigned)fifoDump[17], (unsigned)fifoDump[18],
            (unsigned)fifoDump[19], (unsigned)fifoDump[20]);
      DBGLN("LORA_FAIL cfg:sf%u cr%u pre%u pay%u stat:%02X %02X crcCr:%02X "
            "len:%u snr:%d rssi:%u sig:%u det:%u rxLen:%u read:%u off:%u "
            "sync:%u/%u crcAny:%u/%u/%u/%u "
            "dec:%02X %02X %02X %02X",
            (unsigned)siw917_debug_lora_sf,
            (unsigned)siw917_debug_lora_cr,
            (unsigned)siw917_debug_lora_pre,
            (unsigned)siw917_debug_lora_payload,
            (unsigned)pktStatus[0], (unsigned)pktStatus[1],
            (unsigned)pktStatus[2], (unsigned)pktStatus[3],
            (int8_t)pktStatus[4], (unsigned)pktStatus[5],
            (unsigned)pktStatus[6], (unsigned)((pktStatus[7] >> 2) & 0x0F),
            (unsigned)rxPktLength, (unsigned)rxReadLength,
            (unsigned)fifoPayloadOffset,
            (unsigned)(crcSync ? 1U : 0U), (unsigned)syncOffset,
            (unsigned)(crcAny ? 1U : 0U), (unsigned)crcOffset,
            (unsigned)crcNonce, (unsigned)crcType,
            (unsigned)RXdataBuffer[0], (unsigned)RXdataBuffer[1],
            (unsigned)RXdataBuffer[2], (unsigned)RXdataBuffer[3]);
      siw917_lora_fail_dump_count++;
    }
    if (SIW917_ELRS_LR2021_GFSK_FAIL_DUMP && useFSK &&
        effectivePayloadLength >= 13 &&
        siw917_gfsk_fail_dump_count < 4) {
      DBGLN("GFSK_FAIL raw len=%u head:%02X %02X r:%02X %02X %02X %02X %02X "
            "%02X %02X %02X %02X %02X %02X %02X %02X",
            (unsigned)effectivePayloadLength, (unsigned)rx_buf[0],
            (unsigned)rx_buf[1], (unsigned)rx_buf[2], (unsigned)rx_buf[3],
            (unsigned)rx_buf[4], (unsigned)rx_buf[5], (unsigned)rx_buf[6],
            (unsigned)rx_buf[7], (unsigned)rx_buf[8], (unsigned)rx_buf[9],
            (unsigned)rx_buf[10], (unsigned)rx_buf[11], (unsigned)rx_buf[12],
            (unsigned)rx_buf[13], (unsigned)rx_buf[14]);
      DBGLN("GFSK_FAIL dec type=%u d:%02X %02X %02X %02X %02X %02X %02X "
            "%02X %02X %02X %02X %02X %02X",
            (unsigned)(RXdataBuffer[0] & 0x03), (unsigned)RXdataBuffer[0],
            (unsigned)RXdataBuffer[1], (unsigned)RXdataBuffer[2],
            (unsigned)RXdataBuffer[3], (unsigned)RXdataBuffer[4],
            (unsigned)RXdataBuffer[5], (unsigned)RXdataBuffer[6],
            (unsigned)RXdataBuffer[7], (unsigned)RXdataBuffer[8],
            (unsigned)RXdataBuffer[9], (unsigned)RXdataBuffer[10],
            (unsigned)RXdataBuffer[11], (unsigned)RXdataBuffer[12]);
      if (effectivePayloadLength >= 14 && siw917_gfsk_fail_dump_count < 4) {
        uint8_t variantDecoded[4][OTA4_PACKET_SIZE] = {};
        uint8_t variantNonce[4] = {};
        uint8_t variantType[4] = {};
        uint8_t variantHit[4] = {};
        for (uint8_t variant = 0; variant < 4; variant++) {
          variantHit[variant] =
              siw917FindGfskFecCrcVariant(rx_buf + fifoPayloadOffset,
                                           effectivePayloadLength, variant,
                                           variantDecoded[variant],
                                           &variantNonce[variant],
                                           &variantType[variant])
                  ? 1U
                  : 0U;
        }
        const uint8_t expectedNonce = OtaNonce;
        DBGLN("GFSK_FECVAR exp:%u hit:%u/%u/%u/%u type:%u/%u/%u/%u "
              "nonce:%u/%u/%u/%u delta:%d/%d/%d/%d "
              "d0:%02X %02X %02X %02X d1:%02X %02X "
              "%02X %02X d2:%02X %02X %02X %02X d3:%02X %02X %02X %02X",
              (unsigned)expectedNonce,
              (unsigned)variantHit[0], (unsigned)variantHit[1],
              (unsigned)variantHit[2], (unsigned)variantHit[3],
              (unsigned)variantType[0], (unsigned)variantType[1],
              (unsigned)variantType[2], (unsigned)variantType[3],
              (unsigned)variantNonce[0], (unsigned)variantNonce[1],
              (unsigned)variantNonce[2], (unsigned)variantNonce[3],
              (int)(int8_t)(variantNonce[0] - expectedNonce),
              (int)(int8_t)(variantNonce[1] - expectedNonce),
              (int)(int8_t)(variantNonce[2] - expectedNonce),
              (int)(int8_t)(variantNonce[3] - expectedNonce),
              (unsigned)variantDecoded[0][0], (unsigned)variantDecoded[0][1],
              (unsigned)variantDecoded[0][2], (unsigned)variantDecoded[0][3],
              (unsigned)variantDecoded[1][0], (unsigned)variantDecoded[1][1],
              (unsigned)variantDecoded[1][2], (unsigned)variantDecoded[1][3],
              (unsigned)variantDecoded[2][0], (unsigned)variantDecoded[2][1],
              (unsigned)variantDecoded[2][2], (unsigned)variantDecoded[2][3],
              (unsigned)variantDecoded[3][0], (unsigned)variantDecoded[3][1],
              (unsigned)variantDecoded[3][2], (unsigned)variantDecoded[3][3]);
      }
      siw917_gfsk_fail_dump_count++;
    }
#endif
  }

  return packetAccepted;
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR LR1121Driver::RXnb() {
  // Match upstream ELRS: TX_DONE returns via fallback mode, then SetRx only.
  SetMode(LR1121_MODE_RX_CONT, SX12XX_Radio_All);
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
LR1121Driver::RXnbFromTxDone() {
  // This is exactly the RX_CONT branch of SetMode(), kept separate so the
  // telemetry ISR does not pay for the generic mode dispatcher.
  const uint8_t timeout[3] = {0xFF, 0xFF, 0xFF};
  rxContinuousActive = hal.WriteCommandFastRetry(
      LR20XX_RADIO_SET_RX, timeout, sizeof(timeout), SX12XX_Radio_All);
  txInProgress = false;
}

bool SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
LR1121Driver::TakeAutoRxAfterTxArmed() {
  const bool armed = autoRxAfterTxArmed;
  autoRxAfterTxArmed = false;
  return armed;
}

bool ICACHE_RAM_ATTR
LR1121Driver::GetFrequencyErrorbool(SX12XX_Radio_Number_t radioNumber) {
  return false;
}

// 7.2.8 GetRssiInst
void ICACHE_RAM_ATTR
LR1121Driver::StartRssiInst(SX12XX_Radio_Number_t radioNumber) {
  hal.WriteCommand(LR20XX_RADIO_GET_RSSI_INST, radioNumber);
}

int8_t ICACHE_RAM_ATTR
LR1121Driver::GetRssiInst(SX12XX_Radio_Number_t radioNumber) {
  uint8_t status[LR20XX_RESPONSE_STATUS_LEN + 2] = {0};
  hal.ReadCommand(status, sizeof(status), radioNumber);
  const uint16_t raw =
      ((uint16_t)status[2] << 1) | (uint16_t)((status[3] >> 7) & 0x01);
  int16_t rssi = -(int16_t)(raw / 2);
  if (rssi < -128) {
    rssi = -128;
  }
  return (int8_t)rssi;
}

void ICACHE_RAM_ATTR LR1121Driver::CheckForSecondPacket() {
  hasSecondRadioGotData = false;
  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    constexpr SX12XX_Radio_Number_t radio[2] = {SX12XX_Radio_1, SX12XX_Radio_2};
    const uint8_t processingRadioIdx =
        (instance->processingPacketRadio == SX12XX_Radio_1) ? 0 : 1;
    const uint8_t secondRadioIdx = !processingRadioIdx;
    const uint32_t secondIrqStatus =
        instance->GetIrqStatus(radio[secondRadioIdx]);
    if (secondIrqStatus & LR1121_IRQ_RX_DONE) {
      uint8_t fifoPayloadOffset = siw917_rx_fifo_payload_offset;
      if (fifoPayloadOffset < LR2021_RX_FIFO_COMMAND_PHASE_BYTES ||
          fifoPayloadOffset >
              (LR2021_RX_FIFO_COMMAND_PHASE_BYTES +
               LR2021_RX_FIFO_DISCONNECTED_GUARD_BYTES)) {
        fifoPayloadOffset = LR2021_RX_FIFO_COMMAND_PHASE_BYTES;
      }
      uint8_t rx2ReadLength = (uint8_t)(PayloadLength + fifoPayloadOffset);
      if (rx2ReadLength > sizeof(rx2_buf)) {
        rx2ReadLength = sizeof(rx2_buf);
      }
      memset(rx2_buf, 0, rx2ReadLength);
      rx2_buf[0] = (uint8_t)(LR20XX_CMD_READ_RADIO_RX_FIFO >> 8);
      rx2_buf[1] = (uint8_t)(LR20XX_CMD_READ_RADIO_RX_FIFO & 0xFF);
      hal.ReadCommand(rx2_buf, rx2ReadLength, radio[secondRadioIdx]);
      if (((uint16_t)fifoPayloadOffset + PayloadLength) > rx2ReadLength) {
        fifoPayloadOffset = LR2021_RX_FIFO_COMMAND_PHASE_BYTES;
      }
      codec->decode(RXdataBufferSecond, rx2_buf + fifoPayloadOffset,
                    PayloadLength);
      hasSecondRadioGotData = true;
    }
  }
}

void SIW917_ELRS_RAMFUNC_ATTR ICACHE_RAM_ATTR
LR1121Driver::GetLastPacketStats() {
  const SX12XX_Radio_Number_t radioNumber =
      processingPacketRadio == SX12XX_Radio_1 ? SX12XX_Radio_2 : SX12XX_Radio_1;
  WORD_ALIGNED_ATTR uint8_t pktStatus[LR20XX_RESPONSE_STATUS_LEN + 6] = {};
  WORD_ALIGNED_ATTR uint8_t pktStatus2[LR20XX_RESPONSE_STATUS_LEN + 6] = {};

  // by default, set the strongest receiving radio to be the current processing
  // radio (which got a successful packet)
  strongestReceivingRadio = processingPacketRadio;
#if defined(SIW917_ELRS_LR2021_SKIP_PACKET_STATUS) &&                          \
    SIW917_ELRS_LR2021_SKIP_PACKET_STATUS
  return;
#endif
  const uint16_t packetStatusCommand =
      useFSK ? static_cast<uint16_t>(LR20XX_FSK_GET_PACKET_STATUS)
             : static_cast<uint16_t>(LR20XX_LORA_GET_PACKET_STATUS);
  hal.WriteCommand(packetStatusCommand, processingPacketRadio);
  hal.ReadCommand(pktStatus, sizeof(pktStatus), processingPacketRadio);
  DecodeRssiSnr(processingPacketRadio, pktStatus);
#if defined(DEBUG_RCVR_SIGNAL_STATS)
  irq_count_or++;
#endif

  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    // when both radio got the packet, use the better RSSI one
    if (hasSecondRadioGotData) {
      const int8_t firstSNR = LastPacketSNRRaw;
      hal.WriteCommand(packetStatusCommand, radioNumber);
      hal.ReadCommand(pktStatus2, sizeof(pktStatus2), radioNumber);
      DecodeRssiSnr(radioNumber, pktStatus2);
      LastPacketSNRRaw =
          fuzzy_snr(LastPacketSNRRaw, firstSNR, FuzzySNRThreshold);
      // Update the strongest receiving radio to be the one with better signal
      // strength
      strongestReceivingRadio =
          LastPacketRSSI > LastPacketRSSI2 ? SX12XX_Radio_1 : SX12XX_Radio_2;
#if defined(DEBUG_RCVR_SIGNAL_STATS)
      irq_count_both++;
    } else {
      rxSignalStats[radioNumber == SX12XX_Radio_1 ? 0 : 1].fail_count++;
#endif
    }
  }
}

void SIW917_ELRS_RAMFUNC_ATTR LR1121Driver::IsrCallback_1() {
  IsrCallback(SX12XX_Radio_1);
}

void SIW917_ELRS_RAMFUNC_ATTR LR1121Driver::IsrCallback_2() {
  IsrCallback(SX12XX_Radio_2);
}

// Debug counters for ISR tracking
static volatile uint32_t isrCallCount = 0;
static volatile uint32_t isrCallCountRadio1 = 0;
static volatile uint32_t isrCallCountRadio2 = 0;
volatile uint32_t rxDoneCount = 0;
static volatile uint32_t rxDoneCountRadio1 = 0;
static volatile uint32_t rxDoneCountRadio2 = 0;
static volatile uint32_t txDoneCount = 0;
static volatile uint32_t otherIrqCount = 0;
volatile uint32_t lastIrqStatus = 0;

#if SIW917_ELRS_ISR_STATS_DIAG
#define LR1121_ISR_STAT_INC(var_) ((var_)++)
#define LR1121_ISR_STAT_SET(var_, value_) ((var_) = (value_))
#else
#define LR1121_ISR_STAT_INC(var_) do { } while (0)
#define LR1121_ISR_STAT_SET(var_, value_) do { } while (0)
#endif

extern "C" {
void lr1121_cs_assert(void);
void lr1121_cs_deassert(void);
bool lr1121_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data,
                         uint16_t length);
bool lr1121_spi_transfer_polled(const uint8_t *tx_data, uint8_t *rx_data,
                                uint16_t length);
}

void SIW917_ELRS_RAMFUNC_ATTR
LR1121Driver::IsrCallback(SX12XX_Radio_Number_t radioNumber) {
#if SIW917_ELRS_RX_FIRST_TXDONE
  if (instance->txInProgress) {
    LR1121_ISR_STAT_INC(isrCallCount);
    LR1121_ISR_STAT_INC(txDoneCount);
    LR1121_ISR_STAT_SET(lastIrqStatus, LR1121_IRQ_TX_DONE);
    instance->processingPacketRadio = radioNumber;
    instance->TXnbISR();
    instance->ClearIrqStatusMask(LR1121_IRQ_TX_DONE, radioNumber);
    if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
      const SX12XX_Radio_Number_t otherRadioNumber =
          radioNumber == SX12XX_Radio_1 ? SX12XX_Radio_2 : SX12XX_Radio_1;
      instance->ClearIrqStatusMask(LR1121_IRQ_TX_DONE, otherRadioNumber);
    }
    return;
  }
#endif

  uint32_t irqStatus = instance->GetIrqStatus(radioNumber);

  // Delegate to the version that takes pre-read status
  IsrCallbackWithStatus(radioNumber, irqStatus);
}

void SIW917_ELRS_RAMFUNC_ATTR LR1121Driver::IsrCallbackWithStatus(
    SX12XX_Radio_Number_t radioNumber, uint32_t irqStatus) {
  LR1121_ISR_STAT_INC(isrCallCount);
  if (radioNumber == SX12XX_Radio_1) {
    LR1121_ISR_STAT_INC(isrCallCountRadio1);
  } else {
    LR1121_ISR_STAT_INC(isrCallCountRadio2);
  }
  instance->processingPacketRadio = radioNumber;
  const SX12XX_Radio_Number_t otherRadioNumber =
      radioNumber == SX12XX_Radio_1 ? SX12XX_Radio_2 : SX12XX_Radio_1;

  LR1121_ISR_STAT_SET(lastIrqStatus, irqStatus);
  lr1121TlmMissIrqTraceRecord(irqStatus);

  // HOT PATH - No debug output here! Printf kills timing.

  if (irqStatus & LR1121_IRQ_TX_DONE) {
    LR1121_ISR_STAT_INC(txDoneCount);
    // Clear the paired radio before RXnb() re-arms both radios. Otherwise the
    // level-held TX_DONE line on radio 2 can leave a stale DIO stage pass and
    // steal time from the next receive window.
    if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
      instance->ClearIrqStatus(otherRadioNumber);
    }
    instance->TXnbISR();
    // Note: GetIrqStatus already cleared this radio's IRQ atomically.
  } else if (irqStatus & (LR1121_IRQ_RX_DONE
#if defined(SIW917_ELRS_LR2021_RX_FIFO_AS_RX_DONE) &&                         \
                         SIW917_ELRS_LR2021_RX_FIFO_AS_RX_DONE
                         | LR20XX_IRQ_RX_FIFO
#endif
                         )) {
    LR1121_ISR_STAT_INC(rxDoneCount);
    if (radioNumber == SX12XX_Radio_1) {
      LR1121_ISR_STAT_INC(rxDoneCountRadio1);
    } else {
      LR1121_ISR_STAT_INC(rxDoneCountRadio2);
    }
    instance->RXnbISR(radioNumber);
    // Note: GetIrqStatus already cleared the IRQ atomically
  } else if (irqStatus & LR1121_IRQ_TIMEOUT) {
    // RX timeout - re-arm receiver
    LR1121_ISR_STAT_INC(otherIrqCount);  // Count as "other" for stats
    instance->RXnb(); // Re-enter RX mode
  } else if (irqStatus != 0) {
    LR1121_ISR_STAT_INC(otherIrqCount);
  }
}

// Function to get ISR debug stats
extern "C" void lr1121_get_isr_stats(uint32_t *isr_count, uint32_t *rx_count,
                                     uint32_t *tx_count, uint32_t *other_count,
                                     uint32_t *last_irq) {
  if (isr_count)
    *isr_count = isrCallCount;
  if (rx_count)
    *rx_count = rxDoneCount;
  if (tx_count)
    *tx_count = txDoneCount;
  if (other_count)
    *other_count = otherIrqCount;
  if (last_irq)
    *last_irq = lastIrqStatus;
}

extern "C" void lr1121_get_radio_isr_stats(uint32_t *isr_1,
                                           uint32_t *isr_2,
                                           uint32_t *rx_1,
                                           uint32_t *rx_2) {
  if (isr_1)
    *isr_1 = isrCallCountRadio1;
  if (isr_2)
    *isr_2 = isrCallCountRadio2;
  if (rx_1)
    *rx_1 = rxDoneCountRadio1;
  if (rx_2)
    *rx_2 = rxDoneCountRadio2;
}

struct lr1121UpdateState_s {
  size_t expectedFilesize;
  size_t totalSize;
  SX12XX_Radio_Number_t updatingRadio;
  size_t left_over;
  struct {
    uint8_t header[6];
    uint8_t buffer[256];
  } __attribute__((packed)) packet;
};

static lr1121UpdateState_s *lr1121UpdateState;
static constexpr uint32_t LR1121BootloaderWriteHeaderSize = 6;

// The Semtech Bootloader strictly requires exactly 256-byte payload chunks for all
// transfers except the last one.
// We previously limited this to 240/250 due to a 256-byte GSPI DMA limit on SiW917.
// Now that the SiW917 SPI driver DMA buffer has been increased to 512 bytes,
// we can safely use the official 256-byte payload sizes.
static constexpr uint32_t LR1121BootloaderMaxWritePayload = 256;

firmware_version_t
LR1121Driver::GetFirmwareVersion(const SX12XX_Radio_Number_t radioNumber,
                                 const uint16_t command) {
  uint8_t buffer[LR20XX_RESPONSE_STATUS_LEN + 2] = {};
  hal.WriteCommand(command, radioNumber);
  hal.ReadCommand(buffer, sizeof(buffer), radioNumber);
  hal.WaitOnBusy(radioNumber);

#if SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("GetFirmwareVersion raw: [%02X %02X %02X %02X] -> LR2021 FW=0x%04X",
        buffer[0], buffer[1], buffer[2], buffer[3],
        (uint16_t)(buffer[2] << 8 | buffer[3]));
#endif

  return {.hardware = LR2021_VERSION_HW,
          .type = LR2021_VERSION_TYPE,
          .version = (uint16_t)(buffer[2] << 8 | buffer[3])};
}

int LR1121Driver::BeginUpdate(const SX12XX_Radio_Number_t radioNumber,
                              const uint32_t expectedSize) {
  (void)radioNumber;
  (void)expectedSize;
  DBGLN("LR2021 firmware update is not implemented in this port");
  return -1;
#if 0
  lr1121UpdateState = new lr1121UpdateState_s;
  lr1121UpdateState->expectedFilesize = expectedSize;
  lr1121UpdateState->updatingRadio = radioNumber;
  lr1121UpdateState->totalSize = 0;
  lr1121UpdateState->left_over = 0;

  // Reboot to BL mode
  DBGLN("Reboot 1121 to bootloader mode");
  uint8_t mode = 3;
  hal.WriteCommand(LR11XX_SYSTEM_REBOOT_OC, &mode, 1, radioNumber);
  while (!hal.WaitOnBusy(radioNumber)) {
    DBGLN("Waiting...");
    delay(10);
  }

  // Ensure we're in BL mode
  DBGLN("Ensure BL mode");
  const firmware_version_t version =
      GetFirmwareVersion(radioNumber, LR11XX_BL_GET_VERSION_OC);
  if (version.type != 0xDF) {
    DBGLN("%x", version);
    return -1; // Not in bootloader mode
  }

  // Erase flash
  DBGLN("Erasing");
  hal.WriteCommand(LR11XX_BL_ERASE_FLASH_OC, radioNumber);
  while (!hal.WaitOnBusy(radioNumber)) {
    DBGLN("Waiting...");
    delay(100);
  }
  DBGLN("Erased");

  lr1121UpdateState->left_over = 0;
  SPIEx.setHwCs(false);

  pinMode(radioNumber == SX12XX_Radio_1 ? GPIO_PIN_NSS : GPIO_PIN_NSS_2,
          OUTPUT);
  digitalWrite(radioNumber == SX12XX_Radio_1 ? GPIO_PIN_NSS : GPIO_PIN_NSS_2,
               HIGH);
  return 0;
#endif
}

[[maybe_unused]] static void writeBytes(const uint8_t *data,
                                        const uint32_t data_size) {
  lr1121UpdateState->packet.header[0] =
      (uint8_t)(LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC >> 8);
  lr1121UpdateState->packet.header[1] =
      (uint8_t)(LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC);
  lr1121UpdateState->packet.header[2] =
      (uint8_t)(lr1121UpdateState->totalSize >> 24);
  lr1121UpdateState->packet.header[3] =
      (uint8_t)(lr1121UpdateState->totalSize >> 16);
  lr1121UpdateState->packet.header[4] =
      (uint8_t)(lr1121UpdateState->totalSize >> 8);
  lr1121UpdateState->packet.header[5] = (uint8_t)(lr1121UpdateState->totalSize);

  uint32_t write_size = lr1121UpdateState->left_over;
  if (data != nullptr) {
    DBGLN("Left %d, new %d", lr1121UpdateState->left_over, data_size);
    memcpy(lr1121UpdateState->packet.header + 6 + lr1121UpdateState->left_over,
           data, data_size);
    write_size += data_size;
  }
  
  if (write_size == 0) {
    return; // Don't send empty firmware update packets
  }

  if (write_size > LR1121BootloaderMaxWritePayload) {
    DBGLN("LR1121 update chunk too large: %u > %u", (unsigned)write_size,
          (unsigned)LR1121BootloaderMaxWritePayload);
    return;
  }
  DBGLN("Flashing %d at %x", write_size, lr1121UpdateState->totalSize);

  // The SiW917 GSPI helper currently supports up to 256 bytes per transfer,
  // including the 6-byte bootloader header. Keep the payload capped so the
  // bootloader write stays within that proven transport limit.
  digitalWrite(lr1121UpdateState->updatingRadio == SX12XX_Radio_1
                   ? GPIO_PIN_NSS
                   : GPIO_PIN_NSS_2,
                LOW);
  // Use transferBytes with nullptr rx to do a write-only transfer.
  // This is critical: SPIEx.transfer() overwrites the source buffer in-place,
  // which corrupts the firmware payload. transferBytes(tx, nullptr, n) uses
  // a separate dummy rx buffer to preserve the firmware data.
  SPIEx.transferBytes(lr1121UpdateState->packet.header, nullptr,
                      6 + write_size);
  digitalWrite(lr1121UpdateState->updatingRadio == SX12XX_Radio_1
                   ? GPIO_PIN_NSS
                   : GPIO_PIN_NSS_2,
               HIGH);

  while (!hal.WaitOnBusy(lr1121UpdateState->updatingRadio)) {
    delay(1);
  }
  lr1121UpdateState->totalSize += write_size;
  lr1121UpdateState->left_over = 0;
  DBGLN("Flashed");
}

int LR1121Driver::WriteUpdateBytes(const uint8_t *bytes, uint32_t size) {
  (void)bytes;
  (void)size;
  return -1;
#if 0
  while (size >= LR1121BootloaderMaxWritePayload - lr1121UpdateState->left_over) {
    const uint32_t chunk_size =
        size > LR1121BootloaderMaxWritePayload - lr1121UpdateState->left_over
            ? LR1121BootloaderMaxWritePayload -
                  lr1121UpdateState->left_over
                                    : size;
    writeBytes(bytes, chunk_size);
    size -= chunk_size;
    bytes += chunk_size;
  }
  memcpy(lr1121UpdateState->packet.header + 6 + lr1121UpdateState->left_over, bytes,
         size);
  lr1121UpdateState->left_over += size;
  DBGLN("Left-over %d", lr1121UpdateState->left_over);
  return 0;
#endif
}

int LR1121Driver::EndUpdate() {
  return -1;
#if 0
  int retCode = 0;
  writeBytes(nullptr, 0);

  SPIEx.setHwCs(true);
#if defined(PLATFORM_ESP32)
  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    spiAttachSS(SPIEx.bus(), 1, GPIO_PIN_NSS_2);
  }
#endif

  if (lr1121UpdateState->totalSize == lr1121UpdateState->expectedFilesize) {
    DBGLN("Reboot LR1121");
    uint8_t buf = 0;
    hal.WriteCommand(LR11XX_BL_REBOOT_OC, &buf, 1,
                     lr1121UpdateState->updatingRadio);
    while (!hal.WaitOnBusy(lr1121UpdateState->updatingRadio)) {
      delay(1);
    }
    
    // The LR1121 requires hundreds of milliseconds to fully reboot from Bootloader to Application mode.
    // Waiting 300ms is necessary to prevent querying the bootloader instead of the application.
    delay(300);

    DBGLN("Check not in BL mode");
    const firmware_version_t version = GetFirmwareVersion(
        lr1121UpdateState->updatingRadio, LR11XX_SYSTEM_GET_VERSION_OC);
    DBGLN("Hardware %x", version.hardware >> 24);
    DBGLN("Type %x", version.type);
    DBGLN("Firmware %x", version.version & 0xFFFF);
    retCode = version.type == 0xDF ? -2 : 0; // still in bootloader mode?
  } else {
    DBGLN("Finished expected %d, total %d", lr1121UpdateState->expectedFilesize,
          lr1121UpdateState->totalSize);
    retCode = -1; // Not enough bytes uploaded
  }
  delete lr1121UpdateState;
  lr1121UpdateState = nullptr;
  return retCode;
#endif
}
