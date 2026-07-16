#pragma once

#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "md5.h"

static inline const char *strchrnul(const char *s, int c)
{
    const char *match = strchr(s, c);
    return match ? match : s + strlen(s);
}

static inline void MD5Init(md5_context_t *ctx)
{
    md5_init(ctx);
}

static inline void MD5Update(md5_context_t *ctx, const uint8_t *data, size_t len)
{
    md5_update(ctx, data, len);
}

static inline void MD5Final(uint8_t digest[16], md5_context_t *ctx)
{
    md5_final(ctx, digest);
}

#ifdef __cplusplus
extern "C" {
#endif
void noInterrupts(void);
void interrupts(void);
#ifdef __cplusplus
}
#endif

#define PLATFORM_ESP8266 1

#ifndef USTXC
#define USTXC 0
#endif
#ifndef USS
#define USS(_uart) 0
#endif

#ifndef UNDEF_PIN
#define UNDEF_PIN (-1)
#endif

// Serial
#define GPIO_PIN_RCSIGNAL_RX 6
#define GPIO_PIN_RCSIGNAL_TX 7
#define GPIO_PIN_RCSIGNAL_TX_OE_N 9
#define GPIO_PIN_SERIAL1_RX UNDEF_PIN
#define GPIO_PIN_SERIAL1_TX UNDEF_PIN

// Radio
#define GPIO_PIN_BUSY 29
#define GPIO_PIN_BUSY_2 UNDEF_PIN
#define GPIO_PIN_DIO0 UNDEF_PIN
#define GPIO_PIN_DIO0_2 UNDEF_PIN
#define GPIO_PIN_DIO1 12
#define GPIO_PIN_DIO1_2 UNDEF_PIN
#define GPIO_PIN_MISO 26
#define GPIO_PIN_MOSI 27
#define GPIO_PIN_NSS 28
#define GPIO_PIN_NSS_2 UNDEF_PIN
#define GPIO_PIN_RST 30
#define GPIO_PIN_RST_2 UNDEF_PIN
#define GPIO_PIN_SCK 25
#define OPT_USE_HARDWARE_DCDC false
#define OPT_USE_SX1276_RFO_HF false
static const uint8_t siw917_lr1121_rfsw_ctrl[] = {0};
#define LR1121_RFSW_CTRL siw917_lr1121_rfsw_ctrl
#define LR1121_RFSW_CTRL_COUNT 0

// Antenna
#define GPIO_PIN_ANT_CTRL UNDEF_PIN
#define GPIO_PIN_ANT_CTRL_COMPL UNDEF_PIN

// Radio power
#define GPIO_PIN_PA_ENABLE UNDEF_PIN
#define GPIO_PIN_RFamp_APC2 UNDEF_PIN
#define GPIO_PIN_RX_ENABLE UNDEF_PIN
#define GPIO_PIN_TX_ENABLE UNDEF_PIN
#define GPIO_PIN_RX_ENABLE_2 UNDEF_PIN
#define GPIO_PIN_TX_ENABLE_2 UNDEF_PIN
#define GPIO_PIN_PA_PDET UNDEF_PIN
#define SKY85321_PDET_INTERCEPT 0.0f
#define SKY85321_PDET_SLOPE 0.0f
#define LBT_RSSI_THRESHOLD_OFFSET_DB 0
#define MinPower PWR_10mW
#define MaxPower PWR_100mW
#define DefaultPower PWR_100mW
#define POWER_OUTPUT_DACWRITE false
static const int16_t siw917_power_values_array[] = {10, 14, 17, 20};
static const int16_t siw917_power_values_dual_array[] = {10, 13, 13, 13};
static const int16_t *siw917_power_values = siw917_power_values_array;
static const int16_t *siw917_power_values2 = nullptr;
static const int16_t *siw917_power_values_dual = siw917_power_values_dual_array;
#define POWER_OUTPUT_VALUES siw917_power_values
#define POWER_OUTPUT_VALUES_COUNT 4
#define POWER_OUTPUT_VALUES2 siw917_power_values2
#define POWER_OUTPUT_VALUES2_COUNT 0
#define POWER_OUTPUT_VALUES_DUAL siw917_power_values_dual
#define POWER_OUTPUT_VALUES_DUAL_COUNT 4

// Buttons
#define GPIO_PIN_BUTTON 49
#define USER_BUTTON_LED UNDEF_PIN
#define GPIO_PIN_BUTTON2 UNDEF_PIN
#define USER_BUTTON2_LED UNDEF_PIN

// RGB LED0 on DK2605A, active-low
#define GPIO_PIN_LED 51
#define GPIO_PIN_LED_RED 50
#define GPIO_LED_RED_INVERTED true
#define GPIO_PIN_LED_GREEN 51
#define GPIO_LED_GREEN_INVERTED true
#define GPIO_PIN_LED_BLUE 15
#define GPIO_LED_BLUE_INVERTED true
#define GPIO_PIN_LED_WS2812 UNDEF_PIN
#define OPT_WS2812_IS_GRB false
#define WS2812_STATUS_LEDS nullptr
#define WS2812_STATUS_LEDS_COUNT 0
#define WS2812_VTX_STATUS_LEDS nullptr
#define WS2812_VTX_STATUS_LEDS_COUNT 0
#define WS2812_BOOT_LEDS nullptr
#define WS2812_BOOT_LEDS_COUNT 0

// Unsupported peripherals for this bring-up target
#define GPIO_PIN_SCL UNDEF_PIN
#define GPIO_PIN_SDA UNDEF_PIN
#define GPIO_PIN_PWM_OUTPUTS nullptr
#define GPIO_PIN_PWM_OUTPUTS_COUNT 0
#define OPT_PWM_OUT_ONLY false
#define OPT_HAS_SCREEN false
#define OPT_HAS_OLED_I2C false
#define OPT_HAS_OLED_SPI false
#define OPT_HAS_OLED_SPI_SMALL false
#define OPT_HAS_TFT_SCREEN false
#define OPT_HAS_GSENSOR false
#define OPT_HAS_GSENSOR_STK8xxx false
#define GPIO_PIN_GSENSOR_INT UNDEF_PIN
#define OPT_HAS_THERMAL false
#define OPT_HAS_THERMAL_LM75A false
#define GPIO_PIN_FAN_EN UNDEF_PIN
#define GPIO_PIN_FAN_PWM UNDEF_PIN
#define GPIO_PIN_FAN_TACHO UNDEF_PIN
#define GPIO_PIN_FAN_SPEEDS nullptr
#define GPIO_PIN_FAN_SPEEDS_COUNT 0

// Backpack/debug serial
#define OPT_USE_TX_BACKPACK false
#define BACKPACK_LOGGING_BAUD 0
#define GPIO_PIN_DEBUG_RX UNDEF_PIN
#define GPIO_PIN_DEBUG_TX UNDEF_PIN
#define GPIO_PIN_BACKPACK_BOOT UNDEF_PIN
#define GPIO_PIN_BACKPACK_EN UNDEF_PIN
#define PASSTHROUGH_BAUD 0

// VTX
#define OPT_HAS_VTX_SPI false
#define GPIO_PIN_SPI_VTX_NSS UNDEF_PIN
#define GPIO_PIN_SPI_VTX_MISO UNDEF_PIN
#define GPIO_PIN_SPI_VTX_MOSI UNDEF_PIN
#define GPIO_PIN_SPI_VTX_SCK UNDEF_PIN
