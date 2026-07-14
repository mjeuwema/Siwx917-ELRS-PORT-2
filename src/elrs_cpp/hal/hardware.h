/**
 * @file hardware.h
 * @brief Hardware pin definitions for SiW917 + LR1121 (Waveshare Core1121-HF)
 * 
 * Pin mapping for SiWx917-DK2605A with Waveshare/Core1121 LR1121 module:
 * 
 * GSPI (Radio SPI):
 *   GPIO_25 = SCK  (SPI Clock)
 *   GPIO_26 = MISO (SPI Data from LR1121)
 *   GPIO_27 = MOSI (SPI Data to LR1121)
 *   GPIO_28 = NSS  (Chip Select)
 *
 * LR1121 Control:
 *   GPIO_29 = BUSY   (LR1121 busy indicator)
 *   GPIO_12 = DIO9   (LR1121 IRQ line, breakout-accessible on DK2605A)
 *   GPIO_30 = NRESET (LR1121 reset)
 *
 * RF Switch (PE4259):
 *   DIO5 = Power enable (active high)
 *   DIO6 = TX/RX select (LOW=TX, HIGH=RX)
 */

#pragma once

#include "Arduino.h"

#if !defined(SIW917_ELRS_USE_UPSTREAM_TX_MAIN)

//=============================================================================
// SPI Pins - GSPI peripheral
//=============================================================================
#define GPIO_PIN_SCK    25
#define GPIO_PIN_MISO   26
#define GPIO_PIN_MOSI   27
#define GPIO_PIN_NSS    28

//=============================================================================
// LR1121 Control Pins
//=============================================================================
#define GPIO_PIN_BUSY   29   // BUSY pin (GPIO_29)
#define GPIO_PIN_DIO1   12   // LR1121 DIO9 IRQ on DK2605A breakout GPIO_12
#define GPIO_PIN_RST    30   // Hardware reset (active low, GPIO_30)

//=============================================================================
// Secondary Radio (Gemini/Dual mode) - Not used
//=============================================================================
#define GPIO_PIN_NSS_2      UNDEF_PIN
#define GPIO_PIN_BUSY_2     UNDEF_PIN
#define GPIO_PIN_DIO1_2     UNDEF_PIN
#define GPIO_PIN_RST_2      UNDEF_PIN

//=============================================================================
// I2C - Not used on the DK2605A TX bring-up
//=============================================================================
#define GPIO_PIN_SCL        UNDEF_PIN
#define GPIO_PIN_SDA        UNDEF_PIN

//=============================================================================
// LED Pins
//=============================================================================
#if defined(SIW917_ELRS_TARGET_TX)
#define GPIO_PIN_LED        51    // DK2605A LED0 green channel
#define GPIO_PIN_LED_RED    50    // DK2605A LED0 red channel
#define GPIO_PIN_LED_GREEN  51    // DK2605A LED0 green channel
#define GPIO_PIN_LED_BLUE   15    // DK2605A LED0 blue channel
#else
#define GPIO_PIN_LED        10    // BRD2708A onboard LED0
#define GPIO_PIN_LED_RED    UNDEF_PIN
#define GPIO_PIN_LED_GREEN  GPIO_PIN_LED
#define GPIO_PIN_LED_BLUE   UNDEF_PIN
#endif

//=============================================================================
// Button Pin
//=============================================================================
#if defined(SIW917_ELRS_TARGET_TX)
#define GPIO_PIN_BUTTON     49
#else
#define GPIO_PIN_BUTTON     11
#endif

//=============================================================================
// Serial Pins
//=============================================================================
#if defined(SIW917_ELRS_TARGET_TX)
#define GPIO_PIN_RCSIGNAL_RX    7     // Shared CRSF data, UART1 single-wire, DK2605A breakout pin 24
#define GPIO_PIN_RCSIGNAL_TX    7     // Shared CRSF data, UART1 single-wire, DK2605A breakout pin 24
#else
#define GPIO_PIN_RCSIGNAL_RX    UNDEF_PIN
#define GPIO_PIN_RCSIGNAL_TX    UNDEF_PIN
#endif

//=============================================================================
// PWM Outputs - Not used
//=============================================================================
#define GPIO_PIN_PWM_OUTPUTS_COUNT  0

//=============================================================================
// Antenna Switch - Not used (PE4259 controlled via DIO5/DIO6)
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
#define GPIO_PIN_RFamp_APC2     UNDEF_PIN
#define GPIO_PIN_FAN_EN         UNDEF_PIN
#define GPIO_PIN_FAN_PWM        UNDEF_PIN
#define GPIO_PIN_FAN_SPEEDS     nullptr

#define POWER_OUTPUT_DACWRITE false
#define MinPower PWR_100mW
#define MaxPower PWR_100mW
#define DefaultPower PWR_100mW
static const int8_t siw917_power_values[] = {20};
static const int8_t siw917_power_values_dual[] = {13};
#define POWER_OUTPUT_VALUES siw917_power_values
#define POWER_OUTPUT_VALUES_COUNT 1
#define POWER_OUTPUT_VALUES2 nullptr
#define POWER_OUTPUT_VALUES2_COUNT 0
#define POWER_OUTPUT_VALUES_DUAL siw917_power_values_dual
#define POWER_OUTPUT_VALUES_DUAL_COUNT 1

//=============================================================================
// Debug Settings
//=============================================================================
#define DEBUG_LOG 1
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
// LR1121 Options
//=============================================================================
#define OPT_USE_HARDWARE_DCDC   false  // Use internal LDO, not external DC-DC
#define OPT_USE_SX1276_RFO_HF   false  // Use high-power PA, not low-power

//=============================================================================
// RF Switch Control (LR1121 DIO pins for external switch)
// Waveshare Core1121-HF uses PE4259 RF switch controlled by DIO5/DIO6
// If LR1121_RFSW_CTRL_COUNT is not 8, default switch settings are used
//=============================================================================
#define LR1121_RFSW_CTRL_COUNT  0  // Use default RF switch settings

//=============================================================================
// Helper Functions
//=============================================================================

// Single radio configuration (no Gemini mode)
static inline bool isDualRadio() { return false; }

#endif
