/**
 * @file hardware.h
 * @brief Hardware pin definitions for SiW917 + LR2021 port seed
 * 
 * MCU pin mapping inherited from the working LR1121 Gemini/crossband probe.
 * Core2021-XF module-side pinout verified against Waveshare documentation.
 * 
 * GSPI (Radio SPI):
 *   GPIO_25 = SCK  (SPI Clock)
 *   GPIO_26 = MISO (SPI Data from radio)
 *   GPIO_27 = MOSI (SPI Data to radio)
 *   GPIO_28 = NSS  (Chip Select)
 *
 * Core2021-XF pins driven by LR2021 Control:
 *   GPIO_29 = BUSY
 *   GPIO_46 = DIO11 interrupt input
 *   GPIO_30 = NRESET
 *
 * RF front end:
 *   Core2021-XF exposes LORA_ANT and 2.4G_ANT separately. No sub-GHz
 *   external switch is programmed by this port.
 */

#pragma once

#include "Arduino.h"
#include "siw917_elrs_timing.h"

//=============================================================================
// SPI Pins - GSPI peripheral
//=============================================================================
#define GPIO_PIN_SCK    25
#define GPIO_PIN_MISO   26
#define GPIO_PIN_MOSI   27
#define GPIO_PIN_NSS    28

//=============================================================================
// LR2021 Control Pins
//=============================================================================
#define GPIO_PIN_BUSY   29   // BUSY pin (GPIO_29)
#define GPIO_PIN_DIO1   46   // Core2021-XF DIO11 interrupt (legacy ELRS name)
#define GPIO_PIN_RST    30   // Hardware reset (active low, GPIO_30)

// LR20xx DIO number connected to GPIO_PIN_DIO1. Waveshare Core2021-XF labels
// the interrupt pin as DIO11; keep the legacy GPIO_PIN_DIO1 software name so
// the copied ELRS HAL does not need a broad rename.
#ifndef LR2021_IRQ_DIO_NUM
#define LR2021_IRQ_DIO_NUM 11
#endif

//=============================================================================
// Secondary Radio (Gemini/Dual mode)
//=============================================================================
#if SIW917_ELRS_UPSTREAM_DUAL_RADIO
#define GPIO_PIN_NSS_2      SIW917_ELRS_RADIO2_NSS_PIN
#define GPIO_PIN_BUSY_2     SIW917_ELRS_RADIO2_BUSY_PIN
#define GPIO_PIN_DIO1_2     SIW917_ELRS_RADIO2_DIO_PIN
#define GPIO_PIN_RST_2      SIW917_ELRS_RADIO2_RST_PIN
#else
#define GPIO_PIN_NSS_2      UNDEF_PIN
#define GPIO_PIN_BUSY_2     UNDEF_PIN
#define GPIO_PIN_DIO1_2     UNDEF_PIN
#define GPIO_PIN_RST_2      UNDEF_PIN
#endif

//=============================================================================
// LED Pins
//=============================================================================
#define GPIO_PIN_LED        10    // SiW917 DevKit onboard LED
#define GPIO_PIN_LED_RED    UNDEF_PIN
#define GPIO_PIN_LED_GREEN  GPIO_PIN_LED
#define GPIO_PIN_LED_BLUE   UNDEF_PIN

//=============================================================================
// Button Pin
//=============================================================================
#define GPIO_PIN_BUTTON     11

//=============================================================================
// Serial Pins (CRSF output)
//=============================================================================
#define GPIO_PIN_RCSIGNAL_RX    UNDEF_PIN
#define GPIO_PIN_RCSIGNAL_TX    UNDEF_PIN

//=============================================================================
// PWM Outputs - Not used
//=============================================================================
#define GPIO_PIN_PWM_OUTPUTS_COUNT  0

//=============================================================================
// Antenna Switch - Not used by Core2021-XF
//=============================================================================
#define GPIO_PIN_ANT_CTRL       UNDEF_PIN
#define GPIO_PIN_ANT_CTRL_COMPL UNDEF_PIN

//=============================================================================
// Power Management
//=============================================================================
#define GPIO_PIN_PA_ENABLE      UNDEF_PIN
#define GPIO_PIN_RX_ENABLE      UNDEF_PIN
#define GPIO_PIN_TX_ENABLE      UNDEF_PIN
#define GPIO_PIN_TCXO_ENABLE    UNDEF_PIN

//=============================================================================
// Debug Settings
//=============================================================================
// Quiet timing build: keep DEBUG_LOG off for high-rate RF tests. BUSY-timeout
// and command-fail paths call DBGLN only when this is defined; serial load was
// observed to degrade 200/250/500/K1000 hold. Re-enable only for bring-up.
// #define DEBUG_LOG 1
// #define DEBUG_LOG_VERBOSE 1

//=============================================================================
// Feature Flags
//=============================================================================
#define OPT_HAS_VTX_SPI         false
#define OPT_HAS_SERVO_OUTPUT    false
#define OPT_HAS_GSENSOR         false
#define OPT_HAS_THERMAL         false
#define OPT_USE_BACKPACK        false

//=============================================================================
// LR2021/LR20xx Options
//=============================================================================
#define OPT_USE_HARDWARE_DCDC   false  // Use internal LDO, not external DC-DC
#define OPT_USE_SX1276_RFO_HF   false  // Use high-power PA, not low-power

#define SIW917_ELRS_LR2021_PORT 1

//=============================================================================
// RF Switch Control
//=============================================================================
#define LR2021_HAS_SUBGHZ_RF_SWITCH 0

// Bring-up fallback for LR2021 DIO11 routing. Keep this disabled in normal
// telemetry builds now that the physical DIO line is confirmed working; the
// poll path adds extra SPI reads in the connected hot path.
#ifndef SIW917_ELRS_LR2021_POLL_IRQ_WHEN_NO_DIO
#define SIW917_ELRS_LR2021_POLL_IRQ_WHEN_NO_DIO 0
#endif
#ifndef SIW917_ELRS_LR2021_POLL_IRQ_US
#define SIW917_ELRS_LR2021_POLL_IRQ_US 1000U
#endif

// Keep telemetry TX enabled in the LR2021 diagnostic build; this is the final
// receiver behavior we need to validate.
#ifndef SIW917_ELRS_DISABLE_DOWNLINK_TLM
#define SIW917_ELRS_DISABLE_DOWNLINK_TLM 0
#endif

// LR2021 leaves RX via the STDBY_RC fallback path more aggressively than the
// original LR1121 driver expected. Re-arm continuous RX after each FHSS retune.
#ifndef SIW917_ELRS_FHSS_SET_FREQ_RX
#define SIW917_ELRS_FHSS_SET_FREQ_RX 1
#endif

//=============================================================================
// Helper Functions
//=============================================================================

static inline bool isDualRadio() { return SIW917_ELRS_UPSTREAM_DUAL_RADIO != 0; }
