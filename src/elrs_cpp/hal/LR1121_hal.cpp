/**
 * @file LR1121_hal.cpp
 * @brief SiW917-specific LR1121 HAL implementation
 *
 * This replaces the upstream LR1121_hal.cpp with SiW917-specific setup code
 * and the proven two-phase GSPI command/response path from lr1121_driver.c.
 *
 * IMPORTANT: Core1121/Waveshare module requires TCXO initialization that
 * standard ESP32 ELRS targets don't need. This is handled in
 * lr1121_waveshare_init().
 *
 * Key functions from C driver:
 * - lr1121_send_command() - handles command writes
 * - lr1121_read_response() - handles command responses
 * - lr1121_wait_busy_timeout() - handles BUSY pin polling
 * - lr1121_dio1_*() - handles DIO1 interrupt
 */

#include "../lib/LR1121Driver/LR1121_hal.h"
#include "../lib/LR1121Driver/LR1121_Regs.h"
#include "../include/common.h"
#include "Arduino.h"
#include "elrs_task_wakeup.h"
#include "em_device.h"
#include "logging.h"
#include "siw917_elrs_timing.h"
#include "targets.h"

#include <string.h>

// Include our proven C driver implementation
extern "C" {
#include "hw_timer.h"
#include "lr1121_driver.h"
#include "lr1121_elrs_init.h"
}

#define ELRS_DIAG_GET_PACKET_SOFT_SPI 0

volatile uint32_t isr_1_pending_count = 0;
volatile uint32_t isr_1_total_count = 0;
volatile uint32_t isr_2_pending_count = 0;
volatile bool isr_1_pending = false;
volatile bool isr_2_pending = false;
volatile uint32_t busy_timeout_count = 0;
static volatile uint16_t last_command_opcode = 0;
#if SIW917_ELRS_FUSED_RX_RETUNE
static volatile bool rx_continuous_active = false;
static volatile bool pending_rx_retune = false;
#endif
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
static volatile uint32_t dio1_stage_max_us = 0;
static volatile uint32_t dio1_deferred_max_us = 0;

static inline void dio1UpdateMax(volatile uint32_t &maxValue,
                                 uint32_t durationUs) {
  if (durationUs > maxValue) {
    maxValue = durationUs;
  }
}
#endif

#if SIW917_ELRS_DIO_EDGE_TIMESTAMPS
#define SIW917_DIO_TIMESTAMP(var_) ((var_) = micros())
#else
#define SIW917_DIO_TIMESTAMP(var_) do { } while (0)
#endif

#if SIW917_ELRS_DIO_STATS_DIAG
#define SIW917_DIO_STAT_INC(var_) ((var_)++)
#else
#define SIW917_DIO_STAT_INC(var_) do { } while (0)
#endif

#if SIW917_ELRS_DIRECT_DIO_HAL_IO
#define SIW917_HAL_EGPIO_BASE 0x46130000UL
#define SIW917_HAL_EGPIO_BIT_LOAD_REG(pin_)                                  \
  (*(volatile uint32_t *)(SIW917_HAL_EGPIO_BASE + 0x004UL +                  \
                          (0x10UL * (uint32_t)(pin_))))
#define SIW917_HAL_DIO1_GPIO 46U
#define SIW917_HAL_DIO1_INT_CHANNEL 2U
#define SIW917_HAL_DIO2_INT_CHANNEL 3U
#define SIW917_HAL_DIO1_IRQN                                                  \
  ((IRQn_Type)(EGPIO_PIN_0_IRQn + SIW917_HAL_DIO1_INT_CHANNEL))
#define SIW917_HAL_DIO2_IRQN                                                  \
  ((IRQn_Type)(EGPIO_PIN_0_IRQn + SIW917_HAL_DIO2_INT_CHANNEL))

static inline int siw917HalDio1Read() {
  return (int)(SIW917_HAL_EGPIO_BIT_LOAD_REG(SIW917_HAL_DIO1_GPIO) & 1U);
}

static inline void siw917HalDio1Pause() {
  NVIC_DisableIRQ(SIW917_HAL_DIO1_IRQN);
}

static inline void siw917HalDio1Resume() {
  NVIC_EnableIRQ(SIW917_HAL_DIO1_IRQN);
}

#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
static inline int siw917HalDio2Read() {
  return (int)(SIW917_HAL_EGPIO_BIT_LOAD_REG(LR1121_PIN_DIO_2) & 1U);
}

static inline void siw917HalDio2Pause() {
  NVIC_DisableIRQ(SIW917_HAL_DIO2_IRQN);
}

static inline void siw917HalDio2Resume() {
  NVIC_EnableIRQ(SIW917_HAL_DIO2_IRQN);
}
#endif

#define lr1121_dio1_read siw917HalDio1Read
#define lr1121_dio1_pause_isr siw917HalDio1Pause
#define lr1121_dio1_resume_isr siw917HalDio1Resume
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
#define lr1121_dio2_read siw917HalDio2Read
#define lr1121_dio2_pause_isr siw917HalDio2Pause
#define lr1121_dio2_resume_isr siw917HalDio2Resume
#endif
#endif

extern LR1121Driver Radio;
extern RXtimerState_e RXtimerState;

static bool dio1StageInit();

static inline bool handleBusyTimeout(const char *context, uint16_t opcode,
                                     uint8_t size) {
  busy_timeout_count++;
#if SIW917_ELRS_CONTINUE_AFTER_BUSY_TIMEOUT
  (void)context;
  (void)opcode;
  (void)size;
  return true;
#else
  DBGLN("%s BUSY timeout (opcode=0x%04X size=%u)", context, opcode, size);
  return false;
#endif
}

static inline bool siw917DualRuntimeEnabled() {
  return SIW917_ELRS_UPSTREAM_DUAL_RADIO != 0;
}

static inline bool siw917FanoutAllRadios(SX12XX_Radio_Number_t radioNumber) {
  return radioNumber == SX12XX_Radio_All && siw917DualRuntimeEnabled();
}

static inline uint8_t siw917LrRadioFor(SX12XX_Radio_Number_t radioNumber) {
  if (radioNumber == SX12XX_Radio_2) {
    return LR1121_RADIO_2;
  }
  return LR1121_RADIO_1;
}

static inline void siw917SelectRadio(SX12XX_Radio_Number_t radioNumber) {
  lr1121_select_radio(siw917LrRadioFor(radioNumber));
}

static inline bool siw917IsValidLr1121Version(
    const lr1121_firmware_version_t &version) {
  return version.hardware == 0x22 && version.type != 0x00;
}

#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
static bool siw917SetSelectedRadioStandbyXosc() {
  const uint8_t standbyXosc = ELRS_STANDBY_XOSC;
  if (!lr1121_wait_busy_timeout(100)) {
    return false;
  }
  if (!lr1121_send_command(ELRS_CMD_SET_STANDBY, &standbyXosc, 1)) {
    return false;
  }
  return lr1121_wait_busy_timeout(200);
}
#endif

extern "C" bool lr1121_hal_prepare_radio2_image_calibration(bool highBand) {
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  static int8_t radio2ImageBand =
#if LR1121_BAND_24GHZ
      1;
#else
      0;
#endif
  const int8_t requestedBand = highBand ? 1 : 0;
  if (radio2ImageBand == requestedBand) {
    return true;
  }

  lr1121_select_radio(LR1121_RADIO_2);
  const uint32_t freqMin = highBand ? 2400400000UL : 902000000UL;
  const uint32_t freqMax = highBand ? 2479400000UL : 928000000UL;
  const bool ok = lr1121_elrs_calib_image(freqMin, freqMax) &&
                  siw917SetSelectedRadioStandbyXosc();
  lr1121_select_radio(LR1121_RADIO_1);
  if (ok) {
    radio2ImageBand = requestedBand;
  }
  return ok;
#else
  (void)highBand;
  return false;
#endif
}

// Static instance pointer
LR1121Hal *LR1121Hal::instance = nullptr;

//-----------------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------------

LR1121Hal::LR1121Hal() {
  instance = this;
  IsrCallback_1 = nullptr;
  IsrCallback_2 = nullptr;
}

//-----------------------------------------------------------------------------
// Initialization
// Note: Core1121/Waveshare requires TCXO setup not needed on standard ESP32
// targets
//-----------------------------------------------------------------------------

void LR1121Hal::init() {
  instance = this;
  DBGLN("Hal Init");

  // Initialize the underlying C driver (GPIO, SPI, etc.)
  lr1121_status_t status = lr1121_init();
  if (status != LR1121_OK) {
    DBGLN("LR1121 C driver init failed: %d", (int)status);
    return;
  }

  // Core1121/Waveshare TCXO initialization sequence:
  //   1. Hardware reset
  //   2. Wakeup (CS toggle)
  //   3. Disable SPI CRC
  //   4. SetStandby(XOSC) - External TCXO is already running
  //   5. CalibrateImage (915MHz band)
  //   6. SetRegMode(DCDC)
  //   7. SetDioAsRfSwitch
  //   8. SetTcxoMode(3.0V, 300 ticks)
  //   9. CfgLfClk
  //  10. Calibrate(0x3F)
  //  11. ClearErrors
  //  12. ClearIrq
  lr1121_select_radio(LR1121_RADIO_1);
  status = lr1121_waveshare_init();
  if (status != LR1121_OK) {
    DBGLN("LR1121 TCXO init failed: %d", (int)status);
    return;
  }

#if SIW917_ELRS_DUAL_RADIO_PROBE || SIW917_ELRS_UPSTREAM_DUAL_RADIO
#if SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR1121Hal Radio2 probe enabled: NSS=GPIO_%u BUSY=GPIO_%u "
        "DIO9=GPIO_%u RST=GPIO_%u",
        (unsigned)LR1121_PIN_NSS_2, (unsigned)LR1121_PIN_BUSY_2,
        (unsigned)LR1121_PIN_DIO_2, (unsigned)LR1121_PIN_RST_2);
#endif
  lr1121_select_radio(LR1121_RADIO_2);
  const bool radio2PinsOk = lr1121_debug_exercise_radio2_pins();
  if (!radio2PinsOk) {
    DBGLN("LR1121 #2 probe skipped: NSS2/RST2 did not idle high");
    lr1121_select_radio(LR1121_RADIO_1);
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
    return;
#endif
  } else {
    status = lr1121_waveshare_init();
    if (status != LR1121_OK) {
      DBGLN("LR1121 Radio2 TCXO/probe init failed: %d", (int)status);
      lr1121_select_radio(LR1121_RADIO_1);
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
      return;
#endif
    }

    lr1121_firmware_version_t radio2Version;
    if (status == LR1121_OK &&
        lr1121_get_firmware_version(&radio2Version, LR1121_OPCODE_GET_VERSION)) {
      if (siw917IsValidLr1121Version(radio2Version)) {
#if SIW917_ELRS_RADIO_INIT_VERBOSE
        DBGLN("LR1121 #2 Probe Ready: HW=0x%02X Type=0x%02X FW=0x%04X",
              radio2Version.hardware, radio2Version.type, radio2Version.version);
#endif
      } else {
        DBGLN("LR1121 #2 probe invalid version: HW=0x%02X Type=0x%02X "
              "FW=0x%04X (check NSS2/BUSY2/RST2/MISO wiring)",
              radio2Version.hardware, radio2Version.type, radio2Version.version);
        lr1121_debug_dump_radio_pins("r2-invalid-version");
        lr1121_select_radio(LR1121_RADIO_1);
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
        return;
#endif
      }
    } else if (status == LR1121_OK) {
      DBGLN("LR1121 #2 probe version read failed");
      lr1121_debug_dump_radio_pins("r2-version-read-failed");
      lr1121_select_radio(LR1121_RADIO_1);
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
      return;
#endif
    }
  }
  lr1121_select_radio(LR1121_RADIO_1);
#endif

  // Configure LR1121 DIO9 interrupt (SiW917 GPIO_46)
  // NOTE: Function named "dio1" for ELRS legacy compatibility, but this is
  // LR1121 DIO9! LR1121 DIO1 is NSS (chip select), DIO9 is the IRQ line.
  // Matches ESP32: attachInterrupt(digitalPinToInterrupt(GPIO_PIN_DIO1),
  // dioISR_1, RISING)
  lr1121_dio1_init();
  lr1121_dio1_set_callback(dioISR_1);
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  lr1121_dio2_init();
  lr1121_dio2_set_callback(dioISR_2);
#endif
#if SIW917_ELRS_TWO_STAGE_DIO_ISR
  const bool dioStageReady = dio1StageInit();
#else
  const bool dioStageReady = false;
#endif
  lr1121_dio1_enable();
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  lr1121_dio2_enable();
#endif

#if SIW917_ELRS_RADIO_INIT_VERBOSE
  DBGLN("LR1121Hal DIO hot path: %s",
        SIW917_ELRS_TWO_STAGE_DIO_ISR
            ? (dioStageReady ? (SIW917_ELRS_DIRECT_GPIO_DIO_WHEN_LINKED
                                    ? "hybrid-direct-locked"
                                    : "two-stage-link-active")
                              : "two-stage-fallback-task")
            : (SIW917_ELRS_DIRECT_DIO_ISR ? "direct-when-link-active"
                                            : "deferred-task"));
  DBGLN("LR1121Hal initialized");
#endif
}

void LR1121Hal::end() {
  DBGLN("LR1121Hal::end()");

  // Disable interrupts
  lr1121_dio1_disable();
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  lr1121_dio2_disable();
#endif
  IsrCallback_1 = nullptr;
  IsrCallback_2 = nullptr;

  // Deinitialize the C driver
  lr1121_deinit();
}

//-----------------------------------------------------------------------------
// Reset
//-----------------------------------------------------------------------------

void LR1121Hal::reset(bool bootloader) {
  DBGLN("LR1121Hal::reset(bootloader=%d)", bootloader);
  (void)bootloader; // Not used - no bootloader mode support

  // Perform hardware reset AND full TCXO init via C driver
  // CRITICAL: A raw lr1121_reset() kills the TCXO. We MUST re-run the full
  // waveshare TCXO init sequence every time the chip is hardware reset!
  lr1121_select_radio(LR1121_RADIO_1);
  lr1121_status_t status = lr1121_waveshare_init();
  if (status != LR1121_OK) {
    DBGLN("LR1121 reset/init failed: %d", (int)status);
  }

#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  lr1121_select_radio(LR1121_RADIO_2);
  status = lr1121_waveshare_init();
  if (status != LR1121_OK) {
    DBGLN("LR1121 Radio2 reset/init failed: %d", (int)status);
  }
  lr1121_select_radio(LR1121_RADIO_1);
#endif
}

//-----------------------------------------------------------------------------
// SPI Commands - WriteCommand (opcode only)
// Use the same two-phase C-driver command path as the standalone RX test.
//-----------------------------------------------------------------------------

void LR1121Hal::WriteCommand(uint16_t opcode,
                             SX12XX_Radio_Number_t radioNumber) {
  if (siw917FanoutAllRadios(radioNumber)) {
    WriteCommand(opcode, SX12XX_Radio_1);
    WriteCommand(opcode, SX12XX_Radio_2);
    lr1121_select_radio(LR1121_RADIO_1);
    return;
  }

  siw917SelectRadio(radioNumber);
  last_command_opcode = opcode;

  if (opcode == LR11XX_RADIO_GET_PACKET) {
    // SiW917 drains this ELRS firmware helper from ReadCommand() using the
    // port-layer soft response path. Do not send it twice.
    return;
  }

#if SIW917_ELRS_FUSED_RX_RETUNE
  if (pending_rx_retune && opcode != LR11XX_RADIO_SET_RF_FREQUENCY_OC &&
      opcode != LR11XX_SYSTEM_SET_STANDBY_OC) {
    pending_rx_retune = false;
  }
#endif

  if (!WaitOnBusy(radioNumber) &&
      !handleBusyTimeout("WriteCommand", opcode, 0)) {
    return;
  }

  if (!lr1121_send_command(opcode, nullptr, 0)) {
    DBGLN("WriteCommand failed (opcode=0x%04X)", opcode);
  }
}

//-----------------------------------------------------------------------------
// SPI Commands - WriteCommand (opcode + data)
// Use the same two-phase C-driver command path as the standalone RX test.
//-----------------------------------------------------------------------------

void LR1121Hal::WriteCommand(uint16_t opcode, uint8_t *buffer, uint8_t size,
                             SX12XX_Radio_Number_t radioNumber) {
  if (siw917FanoutAllRadios(radioNumber)) {
    WriteCommand(opcode, buffer, size, SX12XX_Radio_1);
    WriteCommand(opcode, buffer, size, SX12XX_Radio_2);
    lr1121_select_radio(LR1121_RADIO_1);
    return;
  }

  siw917SelectRadio(radioNumber);
  last_command_opcode = opcode;

  if (opcode == LR11XX_RADIO_GET_PACKET) {
    // See opcode-only overload: ReadCommand() owns this SiW917 hot-path command.
    return;
  }

#if SIW917_ELRS_FUSED_RX_RETUNE
  if (opcode == LR11XX_RADIO_SET_RF_FREQUENCY_OC && pending_rx_retune &&
      buffer != nullptr && size >= 4) {
    const uint32_t freq_hz = ((uint32_t)buffer[0] << 24) |
                             ((uint32_t)buffer[1] << 16) |
                             ((uint32_t)buffer[2] << 8) |
                             (uint32_t)buffer[3];
    pending_rx_retune = false;
    if (lr1121_elrs_set_freq_set_rx(freq_hz, true)) {
      rx_continuous_active = true;
      return;
    }
    DBGLN("SetFreq_SetRx failed, falling back to SetRfFrequency");
  }
#endif

  uint8_t patched_buffer[16];
  uint8_t *tx_buffer = buffer;
  if (opcode == LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC && buffer != nullptr &&
      size == 8 && size <= sizeof(patched_buffer)) {
    memcpy(patched_buffer, buffer, size);
    const bool dio1_mask_empty = patched_buffer[4] == 0 &&
                                 patched_buffer[5] == 0 &&
                                 patched_buffer[6] == 0 &&
                                 patched_buffer[7] == 0;
    if (dio1_mask_empty) {
      // Upstream-style code builds the enable mask first. On this board the IRQ
      // line is LR1121 DIO1, so mirror that enable mask into Dio1Mask here.
      patched_buffer[4] = patched_buffer[0];
      patched_buffer[5] = patched_buffer[1];
      patched_buffer[6] = patched_buffer[2];
      patched_buffer[7] = patched_buffer[3];
      tx_buffer = patched_buffer;
    }
  }

#if SIW917_ELRS_FUSED_RX_RETUNE
  if (pending_rx_retune && opcode != LR11XX_RADIO_SET_RF_FREQUENCY_OC &&
      opcode != LR11XX_SYSTEM_SET_STANDBY_OC) {
    pending_rx_retune = false;
  }
#endif

  bool command_ok = false;
  bool handled_hot_command = false;
  bool attempted_fast_hot_command = false;

#if SIW917_ELRS_FAST_HOT_COMMANDS
#if SIW917_ELRS_RAW_GSPI_TX
  if (opcode == LR11XX_RADIO_WRITE_BUFFER8_SET_TX) {
    attempted_fast_hot_command = true;
    command_ok = lr1121_send_command_fast(opcode, tx_buffer, size);
    handled_hot_command = command_ok;
  }
#endif
#if SIW917_ELRS_RAW_GSPI_SET_RX
  if (!handled_hot_command && opcode == LR11XX_RADIO_SET_RX_OC) {
    attempted_fast_hot_command = true;
    command_ok = lr1121_send_command_fast(opcode, tx_buffer, size);
    handled_hot_command = command_ok;
  }
#endif
#if SIW917_ELRS_RAW_GSPI_SET_FREQ
  if (!handled_hot_command && opcode == LR11XX_RADIO_SET_RF_FREQUENCY_OC) {
    attempted_fast_hot_command = true;
    command_ok = lr1121_send_command_fast(opcode, tx_buffer, size);
    handled_hot_command = command_ok;
  }
#endif
#if SIW917_ELRS_RAW_GSPI_SET_FREQ_RX
  if (!handled_hot_command && opcode == LR11XX_RADIO_SET_FREQ_SET_RX) {
    attempted_fast_hot_command = true;
    command_ok = lr1121_send_command_fast(opcode, tx_buffer, size);
    handled_hot_command = command_ok;
  }
#endif
#if SIW917_ELRS_STRICT_BARE_METAL_HOTPATH
  if (attempted_fast_hot_command && connectionState != disconnected) {
    return;
  }
#endif
#endif

  if (!handled_hot_command && !WaitOnBusy(radioNumber) &&
      !handleBusyTimeout("WriteCommand", opcode, size)) {
    return;
  }

#if SIW917_ELRS_RAW_GSPI_TX
  if (!handled_hot_command && opcode == LR11XX_RADIO_WRITE_BUFFER8_SET_TX) {
    command_ok = lr1121_send_command_raw_pub(opcode, tx_buffer, size);
    handled_hot_command = true;
  }
#endif
#if SIW917_ELRS_RAW_GSPI_SET_RX
  if (!handled_hot_command && opcode == LR11XX_RADIO_SET_RX_OC) {
    command_ok = lr1121_send_command_raw_pub(opcode, tx_buffer, size);
    handled_hot_command = true;
  }
#endif
#if SIW917_ELRS_RAW_GSPI_SET_FREQ
  if (!handled_hot_command && opcode == LR11XX_RADIO_SET_RF_FREQUENCY_OC) {
    command_ok = lr1121_send_command_raw_pub(opcode, tx_buffer, size);
    handled_hot_command = true;
  }
#endif
#if SIW917_ELRS_RAW_GSPI_SET_FREQ_RX
  if (!handled_hot_command && opcode == LR11XX_RADIO_SET_FREQ_SET_RX) {
    command_ok = lr1121_send_command_raw_pub(opcode, tx_buffer, size);
    handled_hot_command = true;
  }
#endif
#if SIW917_ELRS_POLLED_HOT_SPI
  if (!handled_hot_command &&
      (opcode == LR11XX_RADIO_WRITE_BUFFER8_SET_TX ||
       opcode == LR11XX_RADIO_SET_RX_OC ||
       opcode == LR11XX_RADIO_SET_RF_FREQUENCY_OC ||
       opcode == LR11XX_RADIO_SET_FREQ_SET_RX)) {
    command_ok = lr1121_send_command_polled_pub(opcode, tx_buffer, size);
    handled_hot_command = true;
  }
#endif
  if (!handled_hot_command) {
    command_ok = lr1121_send_command(opcode, tx_buffer, size);
  }
  if (!command_ok) {
    DBGLN("WriteCommand failed (opcode=0x%04X size=%u)", opcode, size);
  }

#if SIW917_ELRS_FUSED_RX_RETUNE
  if (opcode == LR11XX_SYSTEM_SET_STANDBY_OC && buffer != nullptr &&
      size >= 1) {
    if (rx_continuous_active) {
      pending_rx_retune = true;
    }
    rx_continuous_active = false;
  } else if (opcode == LR11XX_RADIO_SET_RX_OC) {
    rx_continuous_active = true;
    pending_rx_retune = false;
  } else if (opcode == LR11XX_RADIO_SET_TX_OC ||
             opcode == LR11XX_SYSTEM_SET_SLEEP_OC ||
             opcode == LR11XX_SYSTEM_SET_FS_OC) {
    rx_continuous_active = false;
    pending_rx_retune = false;
  }
#endif
}

//-----------------------------------------------------------------------------
// SPI Commands - ReadCommand
// ELRS SINGLE-PHASE FULL-DUPLEX (matching upstream ESP32 behavior)
//
// Upstream ESP32 behavior (SPIEx.read):
//   1. Full-duplex transfer: sends buffer contents while receiving
//   2. Response overwrites the same buffer
//-----------------------------------------------------------------------------

void LR1121Hal::ReadCommand(uint8_t *buffer, uint8_t size,
                            SX12XX_Radio_Number_t radioNumber) {
  siw917SelectRadio(radioNumber);

  if (buffer != nullptr && size >= 2) {
    const uint16_t inline_opcode =
        ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    if (inline_opcode == LR11XX_SYSTEM_CLEAR_IRQ_OC) {
      uint8_t tx_buffer[32];
      if (size > sizeof(tx_buffer)) {
        DBGLN("ReadCommand inline opcode too large (opcode=0x%04X size=%u)",
              inline_opcode, size);
        return;
      }
      memcpy(tx_buffer, buffer, size);
      if (!WaitOnBusy(radioNumber) &&
          !handleBusyTimeout("ReadCommand inline", inline_opcode, size)) {
        memset(buffer, 0, size);
        last_command_opcode = inline_opcode;
        return;
      }
      lr1121_cs_assert();
      const bool ok =
#if SIW917_ELRS_RAW_GSPI_CLEAR_IRQ
          lr1121_spi_transfer_raw(tx_buffer, buffer, size);
#elif SIW917_ELRS_POLLED_HOT_SPI
          lr1121_spi_transfer_polled(tx_buffer, buffer, size);
#else
          lr1121_spi_transfer(tx_buffer, buffer, size);
#endif
      lr1121_cs_deassert();
      if (!ok) {
        DBGLN("ReadCommand inline transfer failed opcode=0x%04X size=%u",
              inline_opcode, size);
        memset(buffer, 0, size);
      }
      last_command_opcode = inline_opcode;
      return;
    }
  }

  if (last_command_opcode == LR11XX_RADIO_GET_PACKET) {
    const bool ok = (buffer != nullptr && size > 0) &&
                    lr1121_elrs_get_packet(
                        buffer, size, ELRS_DIAG_GET_PACKET_SOFT_SPI != 0);
    last_command_opcode = 0;
    if (!ok) {
      DBGLN("ReadCommand GET_PACKET failed size=%u", size);
    }
    return;
  }

  if (!WaitOnBusy(radioNumber) &&
      !handleBusyTimeout("ReadCommand", last_command_opcode, size)) {
    if (buffer != nullptr && size > 0) {
      memset(buffer, 0, size);
    }
    return;
  }

  if (size > 0 && buffer != nullptr &&
      !lr1121_read_response(buffer, size)) {
    DBGLN("ReadCommand failed after opcode=0x%04X size=%u",
          last_command_opcode, size);
    memset(buffer, 0, size);
  }
}

//-----------------------------------------------------------------------------
// BUSY Pin - uses proven lr1121_wait_busy_timeout() from C driver
//-----------------------------------------------------------------------------

bool LR1121Hal::WaitOnBusy(SX12XX_Radio_Number_t radioNumber) {
  if (siw917FanoutAllRadios(radioNumber)) {
    const bool radio1Ready = WaitOnBusy(SX12XX_Radio_1);
    const bool radio2Ready = WaitOnBusy(SX12XX_Radio_2);
    lr1121_select_radio(LR1121_RADIO_1);
    return radio1Ready && radio2Ready;
  }

  siw917SelectRadio(radioNumber);

  if (lr1121_wait_busy_fast_us(SIW917_ELRS_BUSY_FAST_US)) {
    return true;
  }

#if SIW917_ELRS_BUSY_FAST_ONLY_WHEN_CONNECTED
  if (connectionState == tentative || connectionState == connected ||
      RXtimerState == tim_locked) {
    return false;
  }
#endif

  // Keep the long fallback for TCXO/XOSC transitions during init and rate
  // changes before the timing island is active.
  return lr1121_wait_busy_timeout(100);
}

//-----------------------------------------------------------------------------
// Hardware Interrupt Handlers (ISRs)
// These bridge to the C++ callbacks from the C driver's ISR
//-----------------------------------------------------------------------------

// DEFERRED ISR: SiW917 GSPI transactions are not re-entrant. Keep the GPIO
// callback small, mask DIO1 while the IRQ line is asserted, and do all SPI work
// from elrs_loop().
static volatile bool dio1_isr_pending = false;
static volatile bool dio1_isr_processing = false;
static volatile bool dio2_isr_pending = false;
static volatile bool dio2_isr_processing = false;
static volatile uint32_t dio1_direct_count = 0;
static volatile uint32_t dio2_direct_count = 0;
static volatile uint32_t dio1_direct_reentrant_count = 0;
static volatile uint32_t dio2_direct_reentrant_count = 0;
static volatile uint32_t dio1_level_requeue_count = 0;
static volatile uint32_t dio2_level_requeue_count = 0;
static volatile uint32_t dio1_last_edge_us = 0;
static volatile uint32_t dio1_last_deferred_us = 0;
static volatile uint32_t dio2_last_edge_us = 0;
static volatile uint32_t dio2_last_deferred_us = 0;
#if SIW917_ELRS_TWO_STAGE_DIO_ISR
static volatile bool dio1_stage_irq_installed = false;
static volatile uint32_t dio1_stage_irq_count = 0;
static volatile uint32_t dio1_stage_pend_count = 0;

#define DIO1_STAGE_IRQ EGPIO_PIN_7_IRQn
#define DIO1_STAGE_VECTOR_RESERVED_ENTRIES 16U
#define DIO1_STAGE_VECTOR_INDEX                                               \
  (DIO1_STAGE_VECTOR_RESERVED_ENTRIES + (uint32_t)DIO1_STAGE_IRQ)

static uint32_t dio1_stage_ram_vector_table[SI91X_VECTOR_TABLE_ENTRIES]
    __attribute__((aligned(512)));

static inline uint32_t dio1StageEnterCritical() {
  uint32_t primask;
  __asm volatile("mrs %0, primask" : "=r"(primask));
  __asm volatile("cpsid i" ::: "memory");
  return primask;
}

static inline void dio1StageExitCritical(uint32_t primask) {
  __asm volatile("msr primask, %0" ::"r"(primask) : "memory");
}
#endif

static inline bool dio1StagePathAllowed() {
#if SIW917_ELRS_DIRECT_DIO_ISR
  return connectionState == tentative || connectionState == connected;
#else
  return false;
#endif
}

static inline bool dio1GpioDirectPathAllowed() {
#if SIW917_ELRS_DIRECT_DIO_ISR
  return connectionState == connected && RXtimerState == tim_locked;
#else
  return false;
#endif
}

static void SIW917_ELRS_RAMFUNC_ATTR processDio1IrqNow() {
  // Keep SiW917 on the LR1121 driver's normal ISR path. That path mirrors
  // upstream LR1121 handling: atomically clear/read IRQ status before TX_DONE
  // calls back into RX re-arm, so the level-held DIO line is deasserted first.
  if (LR1121Hal::instance && LR1121Hal::instance->IsrCallback_1) {
    LR1121Driver::instance = &Radio;
    lr1121_select_radio(LR1121_RADIO_1);
    LR1121Hal::instance->IsrCallback_1();
  }
}

#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
static void SIW917_ELRS_RAMFUNC_ATTR processDio2IrqNow() {
  if (LR1121Hal::instance && LR1121Hal::instance->IsrCallback_2) {
    LR1121Driver::instance = &Radio;
    lr1121_select_radio(LR1121_RADIO_2);
    LR1121Hal::instance->IsrCallback_2();
    lr1121_select_radio(LR1121_RADIO_1);
  }
}
#endif

#if SIW917_ELRS_TWO_STAGE_DIO_ISR
static void SIW917_ELRS_RAMFUNC_ATTR dio1PendStageFromIsr() {
  SIW917_DIO_STAT_INC(dio1_stage_pend_count);
  NVIC_SetPendingIRQ((IRQn_Type)DIO1_STAGE_IRQ);
}

static void SIW917_ELRS_RAMFUNC_ATTR dio1StageIrqHandler() {
  NVIC_ClearPendingIRQ((IRQn_Type)DIO1_STAGE_IRQ);
  SIW917_DIO_STAT_INC(dio1_stage_irq_count);

  bool dio1Seen = dio1_isr_pending || (lr1121_dio1_read() != 0);
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  bool dio2Seen = dio2_isr_pending || (lr1121_dio2_read() != 0);
#else
  constexpr bool dio2Seen = false;
#endif

  if (!dio1StagePathAllowed()) {
    if (dio1Seen) {
      dio1_isr_pending = true;
      isr_1_pending = true;
    }
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
    if (dio2Seen) {
      dio2_isr_pending = true;
      isr_2_pending = true;
    }
#endif
    if (dio1Seen || dio2Seen) {
      elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
    }
    return;
  }

  if (dio1_isr_processing || dio2_isr_processing) {
    if (dio1Seen) {
      SIW917_DIO_STAT_INC(dio1_direct_reentrant_count);
      dio1_isr_pending = true;
      isr_1_pending = true;
    }
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
    if (dio2Seen) {
      SIW917_DIO_STAT_INC(dio2_direct_reentrant_count);
      dio2_isr_pending = true;
      isr_2_pending = true;
    }
#endif
    elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
    return;
  }

  if (!dio1Seen && !dio2Seen) {
    return;
  }

  if (dio1Seen) {
    dio1_isr_pending = false;
    isr_1_pending = false;
    SIW917_DIO_STAT_INC(dio1_direct_count);
    SIW917_DIO_TIMESTAMP(dio1_last_deferred_us);
    dio1_isr_processing = true;
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
    const uint32_t processStartUs = micros();
#endif
    processDio1IrqNow();
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
    dio1UpdateMax(dio1_stage_max_us, micros() - processStartUs);
#endif
    dio1_isr_processing = false;

    lr1121_dio1_resume_isr();
    if (lr1121_dio1_read() != 0) {
      SIW917_DIO_STAT_INC(dio1_level_requeue_count);
      dio1_isr_pending = true;
      isr_1_pending = true;
      lr1121_dio1_pause_isr();
      dio1PendStageFromIsr();
    }
  }

#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  // Radio1 processing may already clear radio2's paired IRQ via
  // CheckForSecondPacket() or TX_DONE cleanup. Do not spend another SPI
  // transaction on a stale radio2 edge if its DIO line is no longer asserted.
  if (dio2Seen && lr1121_dio2_read() == 0) {
    dio2_isr_pending = false;
    isr_2_pending = false;
    dio2Seen = false;
  }

  if (dio2Seen) {
    dio2_isr_pending = false;
    isr_2_pending = false;
    SIW917_DIO_STAT_INC(dio2_direct_count);
    SIW917_DIO_TIMESTAMP(dio2_last_deferred_us);
    dio2_isr_processing = true;
    processDio2IrqNow();
    dio2_isr_processing = false;

    lr1121_dio2_resume_isr();
    if (lr1121_dio2_read() != 0) {
      SIW917_DIO_STAT_INC(dio2_level_requeue_count);
      dio2_isr_pending = true;
      isr_2_pending = true;
      lr1121_dio2_pause_isr();
      dio1PendStageFromIsr();
    }
  }
#endif
}

static bool dio1StageInit() {
  if (DIO1_STAGE_VECTOR_INDEX >= SI91X_VECTOR_TABLE_ENTRIES) {
    DBGLN("LR1121Hal DIO stage vector index %lu outside table size %lu",
          (unsigned long)DIO1_STAGE_VECTOR_INDEX,
          (unsigned long)SI91X_VECTOR_TABLE_ENTRIES);
    return false;
  }

  const uint32_t newVtor =
      (uint32_t)(uintptr_t)&dio1_stage_ram_vector_table[0];
  uint32_t oldVtor;
  uint32_t oldStageVector;
  const uint32_t primask = dio1StageEnterCritical();

  oldVtor = SCB->VTOR;
  if (oldVtor != newVtor) {
    memcpy(dio1_stage_ram_vector_table, (const void *)(uintptr_t)oldVtor,
           sizeof(dio1_stage_ram_vector_table));
  }

  oldStageVector = dio1_stage_ram_vector_table[DIO1_STAGE_VECTOR_INDEX];
  dio1_stage_ram_vector_table[DIO1_STAGE_VECTOR_INDEX] =
      (uint32_t)(uintptr_t)dio1StageIrqHandler;

  __DSB();
  __ISB();
  SCB->VTOR = newVtor;
  __DSB();
  __ISB();
  dio1StageExitCritical(primask);

  NVIC_ClearPendingIRQ((IRQn_Type)DIO1_STAGE_IRQ);
  NVIC_SetPriority((IRQn_Type)DIO1_STAGE_IRQ,
                   SIW917_ELRS_DIO_STAGE_IRQ_PRIORITY);
  NVIC_EnableIRQ((IRQn_Type)DIO1_STAGE_IRQ);
  dio1_stage_irq_installed = true;

#if SIW917_ELRS_DIO_INIT_VERBOSE
  DBGLN("LR1121Hal DIO stage vector installed irq=%d priority=%u "
        "oldVTOR=0x%08lX newVTOR=0x%08lX oldStage=0x%08lX newStage=0x%08lX",
        (int)DIO1_STAGE_IRQ, (unsigned)SIW917_ELRS_DIO_STAGE_IRQ_PRIORITY,
        (unsigned long)oldVtor, (unsigned long)newVtor,
        (unsigned long)oldStageVector,
        (unsigned long)(uintptr_t)dio1StageIrqHandler);
#else
  (void)oldStageVector;
#endif
  return true;
}
#else
[[maybe_unused]] static bool dio1StageInit() { return false; }
#endif

extern "C" uint32_t lr1121_hal_get_last_dio1_edge_us(void) {
  return dio1_last_edge_us;
}

extern "C" uint32_t lr1121_hal_get_last_deferred_us(void) {
  return dio1_last_deferred_us;
}

extern "C" bool lr1121_hal_has_pending_dio1(void) {
  bool pending = dio1_isr_pending || (lr1121_dio1_read() != 0);
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  pending = pending || dio2_isr_pending || (lr1121_dio2_read() != 0);
#endif
  return pending;
}

extern "C" uint32_t lr1121_hal_get_direct_dio_count(void) {
  return dio1_direct_count + dio2_direct_count;
}

extern "C" uint32_t lr1121_hal_get_direct_reentrant_count(void) {
  return dio1_direct_reentrant_count + dio2_direct_reentrant_count;
}

extern "C" uint32_t lr1121_hal_get_level_requeue_count(void) {
  return dio1_level_requeue_count + dio2_level_requeue_count;
}

extern "C" uint32_t lr1121_hal_get_stage_max_us(void) {
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  return dio1_stage_max_us;
#else
  return 0;
#endif
}

extern "C" uint32_t lr1121_hal_get_deferred_max_us(void) {
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  return dio1_deferred_max_us;
#else
  return 0;
#endif
}

void LR1121Hal::dioISR_1() {
  SIW917_DIO_TIMESTAMP(dio1_last_edge_us);
  SIW917_DIO_STAT_INC(isr_1_total_count);

#if SIW917_ELRS_TWO_STAGE_DIO_ISR
  dio1_isr_pending = true;
  isr_1_pending = true;

  lr1121_dio1_pause_isr();
#if SIW917_ELRS_DIRECT_GPIO_DIO_WHEN_LINKED
  if (dio1_stage_irq_installed && dio1GpioDirectPathAllowed()) {
    if (dio1_isr_processing) {
      SIW917_DIO_STAT_INC(dio1_direct_reentrant_count);
      dio1PendStageFromIsr();
      return;
    }

    dio1_isr_pending = false;
    isr_1_pending = false;
    SIW917_DIO_STAT_INC(dio1_direct_count);
    dio1_last_deferred_us = dio1_last_edge_us;
    dio1_isr_processing = true;
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
    const uint32_t processStartUs = micros();
#endif
    processDio1IrqNow();
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
    dio1UpdateMax(dio1_stage_max_us, micros() - processStartUs);
#endif
    dio1_isr_processing = false;

    lr1121_dio1_resume_isr();
    if (lr1121_dio1_read() != 0) {
      SIW917_DIO_STAT_INC(dio1_level_requeue_count);
      dio1_isr_pending = true;
      isr_1_pending = true;
      lr1121_dio1_pause_isr();
      dio1PendStageFromIsr();
    }
    return;
  }
#endif

  // Disconnected/scanning and recovery stay on the stage IRQ so the GPIO IRQ
  // never calls FreeRTOS APIs from its higher priority.
  if (dio1_stage_irq_installed) {
    dio1PendStageFromIsr();
  } else {
    elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
  }
  return;
#endif

  if (dio1GpioDirectPathAllowed()) {
    if (dio1_isr_processing) {
      SIW917_DIO_STAT_INC(dio1_direct_reentrant_count);
      dio1_isr_pending = true;
      isr_1_pending = true;
      lr1121_dio1_pause_isr();
      elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
      return;
    }

    SIW917_DIO_STAT_INC(dio1_direct_count);
    dio1_isr_processing = true;
    lr1121_dio1_pause_isr();
    processDio1IrqNow();
    dio1_isr_processing = false;

    lr1121_dio1_resume_isr();
    if (lr1121_dio1_read() != 0) {
      SIW917_DIO_STAT_INC(dio1_level_requeue_count);
      dio1_isr_pending = true;
      isr_1_pending = true;
      lr1121_dio1_pause_isr();
      elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
    }
    return;
  }

  dio1_isr_pending = true;
  isr_1_pending = true;

  // DIO1 stays high until the LR1121 IRQ is cleared over SPI. Mask it here so
  // the GPIO interrupt cannot repeatedly fire before the deferred handler runs.
  lr1121_dio1_pause_isr();
  elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
}

// Called from elrs_loop() to process deferred DIO1 interrupts safely
void LR1121Hal::handleDeferredISR() {
  const bool dio1High = lr1121_dio1_read() != 0;
  if (dio1_isr_pending || dio1High) {
    SIW917_DIO_TIMESTAMP(dio1_last_deferred_us);
  }

  if (!dio1_isr_pending && dio1High) {
    // DIO1 is level-high until the LR1121 IRQ is cleared. If a new IRQ arrives
    // while the GPIO edge is masked, there may be no fresh rising edge to wake
    // us, so synthesize one from the level.
    SIW917_DIO_STAT_INC(dio1_level_requeue_count);
    dio1_last_edge_us = dio1_last_deferred_us;
    dio1_isr_pending = true;
    isr_1_pending = true;
    lr1121_dio1_pause_isr();
  }

  if (dio1_isr_pending) {
    dio1_isr_pending = false;
    isr_1_pending = false;

    // Call the ISR callback from task context. On SiW917, explicitly clear and
    // re-arm after RX-side IRQs so DIO1 cannot remain asserted and starve the
    // ELRS loop after the first real packet.
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
    const uint32_t processStartUs = micros();
#endif
    processDio1IrqNow();
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
    dio1UpdateMax(dio1_deferred_max_us, micros() - processStartUs);
#endif

    lr1121_dio1_resume_isr();
    if (lr1121_dio1_read() != 0) {
      SIW917_DIO_STAT_INC(dio1_level_requeue_count);
      dio1_isr_pending = true;
      isr_1_pending = true;
      lr1121_dio1_pause_isr();
    }
  }

#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  const bool dio2High = lr1121_dio2_read() != 0;
  if (dio2_isr_pending || dio2High) {
    SIW917_DIO_TIMESTAMP(dio2_last_deferred_us);
  }

  if (!dio2_isr_pending && dio2High) {
    SIW917_DIO_STAT_INC(dio2_level_requeue_count);
    dio2_last_edge_us = dio2_last_deferred_us;
    dio2_isr_pending = true;
    isr_2_pending = true;
    lr1121_dio2_pause_isr();
  }

  if (dio2_isr_pending) {
    dio2_isr_pending = false;
    isr_2_pending = false;

    dio2_isr_processing = true;
    processDio2IrqNow();
    dio2_isr_processing = false;

    lr1121_dio2_resume_isr();
    if (lr1121_dio2_read() != 0) {
      SIW917_DIO_STAT_INC(dio2_level_requeue_count);
      dio2_isr_pending = true;
      isr_2_pending = true;
      lr1121_dio2_pause_isr();
    }
  }
#endif
}

void LR1121Hal::dioISR_2() {
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
  SIW917_DIO_TIMESTAMP(dio2_last_edge_us);
  SIW917_DIO_STAT_INC(isr_2_pending_count);

#if SIW917_ELRS_TWO_STAGE_DIO_ISR
  dio2_isr_pending = true;
  isr_2_pending = true;

  lr1121_dio2_pause_isr();
#if SIW917_ELRS_DIRECT_GPIO_DIO_WHEN_LINKED
  if (dio1_stage_irq_installed && dio1GpioDirectPathAllowed()) {
    if (dio2_isr_processing) {
      SIW917_DIO_STAT_INC(dio2_direct_reentrant_count);
      dio1PendStageFromIsr();
      return;
    }

    dio2_isr_pending = false;
    isr_2_pending = false;
    SIW917_DIO_STAT_INC(dio2_direct_count);
    dio2_last_deferred_us = dio2_last_edge_us;
    dio2_isr_processing = true;
    processDio2IrqNow();
    dio2_isr_processing = false;

    lr1121_dio2_resume_isr();
    if (lr1121_dio2_read() != 0) {
      SIW917_DIO_STAT_INC(dio2_level_requeue_count);
      dio2_isr_pending = true;
      isr_2_pending = true;
      lr1121_dio2_pause_isr();
      dio1PendStageFromIsr();
    }
    return;
  }
#endif

  if (dio1_stage_irq_installed) {
    dio1PendStageFromIsr();
  } else {
    elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
  }
  return;
#endif

  if (dio2_isr_processing) {
    SIW917_DIO_STAT_INC(dio2_direct_reentrant_count);
    dio2_isr_pending = true;
    isr_2_pending = true;
    lr1121_dio2_pause_isr();
    elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
    return;
  }

  dio2_isr_pending = true;
  isr_2_pending = true;
  lr1121_dio2_pause_isr();
  if (dio1GpioDirectPathAllowed()) {
    SIW917_DIO_STAT_INC(dio2_direct_count);
    dio2_isr_pending = false;
    isr_2_pending = false;
    dio2_isr_processing = true;
    processDio2IrqNow();
    dio2_isr_processing = false;

    lr1121_dio2_resume_isr();
    if (lr1121_dio2_read() != 0) {
      SIW917_DIO_STAT_INC(dio2_level_requeue_count);
      dio2_isr_pending = true;
      isr_2_pending = true;
      lr1121_dio2_pause_isr();
      elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
    }
    return;
  }

  elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_DIO1);
#endif
}
