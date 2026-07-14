/**
 * @file status_led.c
 * @brief Status LED driver implementation for ELRS target boards
 *
 * BRD2708A RX path:
 *   - Uses LED0 on GPIO_10 (active high)
 *   - LED1 remains unused because it conflicts with the original LR1121 DIO1 path
 *
 * BRD2605A TX path:
 *   - Uses RGB LED0 channels on GPIO_50 (R), GPIO_51 (G), GPIO_15 (B)
 *   - All RGB channels are active low
 *
 * LED Modes:
 *   - OFF:          LED off
 *   - DISCONNECTED: Green slow blink
 *   - TENTATIVE:    Cyan medium blink
 *   - CONNECTED:    Solid green
 *   - BINDING:      Magenta fast blink
 *   - WIFI:         Blue double-blink pattern
 *   - ERROR:        Red rapid blink
 *
 * Register Configuration Citations:
 * - siw917x-family-rm.pdf Section 11.6.1 PAD_CONFIG_REG_x:
 *   Base address: 0x4600_4000
 *   Offset: 0x000 + (0x04 * pin_number)
 *   Bits 1:0 (E) = drive strength (01 = 4mA)
 *
 * - siw917x-family-rm.pdf Section 11.11 EGPIO Register Map:
 *   EGPIO base: 0x4613_0000
 *   GPIO_CONFIG_REG offset: 0x000 + (0x10 * pin)
 *   BIT_LOAD_REG offset: 0x004 + (0x10 * pin)
 */

#include "status_led.h"
#include <stdio.h>
#include <stdint.h>
#include "cmsis_os2.h"
#include "rsi_egpio.h"
#include "rsi_rom_egpio.h"
#include "rsi_rom_clks.h"
#include "sl_gpio_board.h"

/*******************************************************************************
 * Debug Output
 ******************************************************************************/
#ifndef DEBUGOUT
#define DEBUGOUT printf
#endif

#define LED_DBG(fmt, ...) DEBUGOUT("[STATUS_LED] " fmt, ##__VA_ARGS__)

#ifndef STATUS_LED_DEBUG_VERBOSE
#define STATUS_LED_DEBUG_VERBOSE 0
#endif

#if STATUS_LED_DEBUG_VERBOSE
#define LED_DBG_VERBOSE(fmt, ...) LED_DBG(fmt, ##__VA_ARGS__)
#else
#define LED_DBG_VERBOSE(fmt, ...) ((void)0)
#endif

/*******************************************************************************
 * Register Definitions
 ******************************************************************************/

/**
 * EGPIO clocks are disabled at reset on SiWx917. The radio driver also enables
 * them later for DIO1, but the status LED is initialized before the radio.
 */
#define M4CLK_BASE                  0x46000000UL
#define CLK_ENABLE_SET_REG2         (*(volatile uint32_t *)(M4CLK_BASE + 0x008))
#define CLK_ENABLE_SET_REG3         (*(volatile uint32_t *)(M4CLK_BASE + 0x010))
#define EGPIO_PCLK_ENABLE_BIT       (1UL << 21)
#define EGPIO_CLK_ENABLE_BIT        (1UL << 16)

#define STATUS_PAD_CONFIG_REG_BASE  0x46004000UL
#define STATUS_PAD_CONFIG_REG(pin)  (*(volatile uint32_t *)(STATUS_PAD_CONFIG_REG_BASE + (0x04UL * (pin))))

/**
 * Pad configuration bit definitions
 * Citation: siw917x-family-rm.pdf Section 11.6.1 "PAD_CONFIG_REG_x"
 */
#define PAD_E_4MA                   (1UL << 0)   /* 4mA drive strength */
#define PAD_SR_HIGH                 (1UL << 5)   /* High slew rate */

#define EGPIO_BASE                  0x46130000UL
#define GPIO_CONFIG_REG(pin)        (*(volatile uint32_t *)(EGPIO_BASE + 0x000 + (0x10UL * (pin))))
#define BIT_LOAD_REG(pin)           (*(volatile uint32_t *)(EGPIO_BASE + 0x004 + (0x10UL * (pin))))

/**
 * GPIO configuration bit definitions
 * Citation: siw917x-family-rm.pdf Section 11.12.9 "GPIO_CONFIG_REG_x"
 */
#define GPIO_MODE_MASK              0x3CUL       /* Bits 5:2 */
#define GPIO_MODE_GPIO              0x00UL       /* MODE = 0 for GPIO */
#define GPIO_DIRECTION_BIT          (1UL << 0)   /* 0=output, 1=input */

/*******************************************************************************
 * Blink Pattern Definitions (single LED patterns)
 ******************************************************************************/

/* Slow blink for disconnected: 500ms on, 500ms off (1Hz) */
#define BLINK_SLOW_MS               500

/* Medium blink for tentative: 250ms on, 250ms off (2Hz) */
#define BLINK_MEDIUM_MS             250

/* Fast blink for binding: 50ms on, 50ms off (10Hz) */
#define BLINK_FAST_MS               50

/* Double-blink pattern for WiFi mode */
#define WIFI_BLINK_ON_MS            100
#define WIFI_BLINK_OFF_MS           100
#define WIFI_PAUSE_MS               600

/* Rapid blink for error */
#define BLINK_RAPID_MS              75

/*******************************************************************************
 * Module State
 ******************************************************************************/

static status_led_mode_t current_mode = LED_MODE_OFF;
static uint32_t last_toggle_tick = 0;
static bool led_state = false;
static bool initialized = false;

/* State machine for complex patterns (WiFi double-blink) */
typedef enum {
    PATTERN_BLINK1_ON,
    PATTERN_BLINK1_OFF,
    PATTERN_BLINK2_ON,
    PATTERN_BLINK2_OFF,
    PATTERN_PAUSE
} pattern_state_t;

static pattern_state_t wifi_pattern_state = PATTERN_BLINK1_ON;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

#if defined(SIW917_ELRS_TARGET_TX)
#define STATUS_LED_RED_PAD_PIN      SL_SI91X_GPIO_50_PAD
#define STATUS_LED_GREEN_PAD_PIN    SL_SI91X_GPIO_51_PAD
#define STATUS_LED_BLUE_PAD_PIN     SL_SI91X_GPIO_15_PAD
static void write_gpio_level(uint8_t pin, bool high)
{
    BIT_LOAD_REG(pin) = high ? 1UL : 0UL;
}

static void write_rgb_led(bool red_on, bool green_on, bool blue_on)
{
    /* BRD2605A LED0 is active-low. */
    write_gpio_level(STATUS_LED_RED_GPIO_PIN, !red_on);
    write_gpio_level(STATUS_LED_GREEN_GPIO_PIN, !green_on);
    write_gpio_level(STATUS_LED_BLUE_GPIO_PIN, !blue_on);
}

static void init_led_gpio(uint8_t pin, uint8_t pad)
{
    RSI_EGPIO_PadSelectionEnable(pad);
    RSI_EGPIO_SetPinMux(EGPIO, 0, pin, EGPIO_PIN_MUX_MODE0);

    STATUS_PAD_CONFIG_REG(pin) = PAD_E_4MA | PAD_SR_HIGH;

    uint32_t cfg = GPIO_CONFIG_REG(pin);
    cfg &= ~GPIO_MODE_MASK;
    cfg &= ~GPIO_DIRECTION_BIT;
    GPIO_CONFIG_REG(pin) = cfg;
}
#endif

/**
 * @brief Write the current mode's LED output state
 */
static void write_led(bool on)
{
#if defined(SIW917_ELRS_TARGET_TX)
    if (!on) {
        write_rgb_led(false, false, false);
    } else {
        switch (current_mode) {
            case LED_MODE_WIFI:
                write_rgb_led(false, false, true);
                break;
            case LED_MODE_BINDING:
                write_rgb_led(true, false, true);
                break;
            case LED_MODE_ERROR:
                write_rgb_led(true, false, false);
                break;
            case LED_MODE_BOOT:
                write_rgb_led(true, true, true);
                break;
            case LED_MODE_TENTATIVE:
                write_rgb_led(false, true, true);
                break;
            case LED_MODE_CONNECTED:
            case LED_MODE_DISCONNECTED:
                write_rgb_led(false, true, false);
                break;
            case LED_MODE_OFF:
                write_rgb_led(false, false, false);
                break;
            default:
                write_rgb_led(false, true, false);
                break;
        }
    }
#else
    /* BRD2708A LED0 is active-high. */
    BIT_LOAD_REG(STATUS_LED0_GPIO_PIN) = on ? 1UL : 0UL;
#endif
    led_state = on;
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize the status LED
 */
int status_led_init(void)
{
    LED_DBG("Initializing status LED...\n");

    /* Enable EGPIO clocks before touching GPIO config/output registers. */
    CLK_ENABLE_SET_REG2 = EGPIO_PCLK_ENABLE_BIT;
    CLK_ENABLE_SET_REG3 = EGPIO_CLK_ENABLE_BIT;
    RSI_CLK_PeripheralClkEnable(M4CLK, EGPIO_CLK, ENABLE_STATIC_CLK);

#if defined(SIW917_ELRS_TARGET_TX)
    init_led_gpio(STATUS_LED_RED_GPIO_PIN, STATUS_LED_RED_PAD_PIN);
    init_led_gpio(STATUS_LED_GREEN_GPIO_PIN, STATUS_LED_GREEN_PAD_PIN);
    init_led_gpio(STATUS_LED_BLUE_GPIO_PIN, STATUS_LED_BLUE_PAD_PIN);
    LED_DBG_VERBOSE("  RGB LED0 configured: R=GPIO_%d (pad %u), G=GPIO_%d (pad %u), B=GPIO_%d (pad %u)\n",
                    STATUS_LED_RED_GPIO_PIN, (unsigned)STATUS_LED_RED_PAD_PIN,
                    STATUS_LED_GREEN_GPIO_PIN, (unsigned)STATUS_LED_GREEN_PAD_PIN,
                    STATUS_LED_BLUE_GPIO_PIN, (unsigned)STATUS_LED_BLUE_PAD_PIN);
    LED_DBG_VERBOSE("  LED polarity: active-low\n");
#else
    LED_DBG_VERBOSE("  NOTE: LED1 (ULP_GPIO_2) not used - conflicts with LR1121 DIO1\n");
    RSI_EGPIO_PadSelectionEnable(SL_SI91X_GPIO_10_PAD);
    RSI_EGPIO_SetPinMux(EGPIO, 0, STATUS_LED0_GPIO_PIN, EGPIO_PIN_MUX_MODE0);
    STATUS_PAD_CONFIG_REG(STATUS_LED0_GPIO_PIN) = PAD_E_4MA | PAD_SR_HIGH;
    uint32_t cfg = GPIO_CONFIG_REG(STATUS_LED0_GPIO_PIN);
    cfg &= ~GPIO_MODE_MASK;
    cfg &= ~GPIO_DIRECTION_BIT;
    GPIO_CONFIG_REG(STATUS_LED0_GPIO_PIN) = cfg;
    LED_DBG_VERBOSE("  LED0 configured: GPIO_%d (pad %u), active-high\n",
                    STATUS_LED0_GPIO_PIN, (unsigned)SL_SI91X_GPIO_10_PAD);
#endif

    write_led(false);
    
    current_mode = LED_MODE_OFF;
    last_toggle_tick = osKernelGetTickCount();
    initialized = true;
    
    LED_DBG("Status LED initialization complete\n");
    
    return 0;
}

/**
 * @brief Set the LED indication mode
 */
void status_led_set_mode(status_led_mode_t mode)
{
    if (!initialized) {
        return;
    }
    
    if (mode == current_mode) {
        return;  /* No change */
    }
    
    current_mode = mode;
    last_toggle_tick = osKernelGetTickCount();
    wifi_pattern_state = PATTERN_BLINK1_ON;  /* Reset pattern state */
    
    /* Set initial state for the new mode */
    switch (mode) {
        case LED_MODE_OFF:
            write_led(false);
            LED_DBG("Mode: OFF\n");
            break;
            
        case LED_MODE_DISCONNECTED:
            write_led(true);   /* Start ON for slow blink */
            LED_DBG("Mode: DISCONNECTED (slow blink 1Hz)\n");
            break;
            
        case LED_MODE_TENTATIVE:
            write_led(true);   /* Start ON for medium blink */
            LED_DBG("Mode: TENTATIVE (medium blink 2Hz)\n");
            break;
            
        case LED_MODE_CONNECTED:
            write_led(true);   /* Solid ON */
            LED_DBG("Mode: CONNECTED (solid ON)\n");
            break;
            
        case LED_MODE_BINDING:
            write_led(true);   /* Start fast blink */
            LED_DBG("Mode: BINDING (fast blink 10Hz)\n");
            break;
            
        case LED_MODE_WIFI:
            write_led(true);   /* Start double-blink pattern */
            LED_DBG("Mode: WIFI (double-blink pattern)\n");
            break;
            
        case LED_MODE_ERROR:
            write_led(true);   /* Rapid blink */
            LED_DBG("Mode: ERROR (rapid blink)\n");
            break;
            
        case LED_MODE_BOOT:
            write_led(false);
            LED_DBG("Mode: BOOT\n");
            break;
            
        default:
            break;
    }
}

/**
 * @brief Get the current LED mode
 */
status_led_mode_t status_led_get_mode(void)
{
    return current_mode;
}

/**
 * @brief Update LED state (call periodically from main loop)
 */
void status_led_update(void)
{
    if (!initialized) {
        return;
    }
    
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed = now - last_toggle_tick;
    
    switch (current_mode) {
        case LED_MODE_OFF:
        case LED_MODE_CONNECTED:
            /* No blinking needed - static states */
            return;
            
        case LED_MODE_DISCONNECTED:
            /* Slow blink: 500ms on, 500ms off */
            if (elapsed >= BLINK_SLOW_MS) {
                last_toggle_tick = now;
                led_state = !led_state;
                write_led(led_state);
            }
            break;
            
        case LED_MODE_TENTATIVE:
            /* Medium blink: 250ms on, 250ms off */
            if (elapsed >= BLINK_MEDIUM_MS) {
                last_toggle_tick = now;
                led_state = !led_state;
                write_led(led_state);
            }
            break;
            
        case LED_MODE_BINDING:
            /* Fast blink: 50ms on, 50ms off */
            if (elapsed >= BLINK_FAST_MS) {
                last_toggle_tick = now;
                led_state = !led_state;
                write_led(led_state);
            }
            break;
            
        case LED_MODE_WIFI:
            /* Double-blink pattern: blink-blink-pause */
            switch (wifi_pattern_state) {
                case PATTERN_BLINK1_ON:
                    if (elapsed >= WIFI_BLINK_ON_MS) {
                        last_toggle_tick = now;
                        write_led(false);
                        wifi_pattern_state = PATTERN_BLINK1_OFF;
                    }
                    break;
                case PATTERN_BLINK1_OFF:
                    if (elapsed >= WIFI_BLINK_OFF_MS) {
                        last_toggle_tick = now;
                        write_led(true);
                        wifi_pattern_state = PATTERN_BLINK2_ON;
                    }
                    break;
                case PATTERN_BLINK2_ON:
                    if (elapsed >= WIFI_BLINK_ON_MS) {
                        last_toggle_tick = now;
                        write_led(false);
                        wifi_pattern_state = PATTERN_BLINK2_OFF;
                    }
                    break;
                case PATTERN_BLINK2_OFF:
                    if (elapsed >= WIFI_PAUSE_MS) {
                        last_toggle_tick = now;
                        write_led(true);
                        wifi_pattern_state = PATTERN_BLINK1_ON;
                    }
                    break;
                default:
                    wifi_pattern_state = PATTERN_BLINK1_ON;
                    break;
            }
            break;
            
        case LED_MODE_ERROR:
            /* Rapid blink: 75ms on, 75ms off */
            if (elapsed >= BLINK_RAPID_MS) {
                last_toggle_tick = now;
                led_state = !led_state;
                write_led(led_state);
            }
            break;
            
        case LED_MODE_BOOT:
            /* Boot sequence handled separately */
            break;
            
        default:
            break;
    }
}

/**
 * @brief Directly control LED0
 */
void status_led0_set(bool on)
{
    if (!initialized) {
        return;
    }
    write_led(on);
}

/**
 * @brief LED1 control
 */
void status_led1_set(bool on)
{
#if defined(SIW917_ELRS_TARGET_TX)
    if (!initialized) {
        return;
    }
    write_rgb_led(false, false, on);
#else
    (void)on;  /* Unused - LED1 conflicts with LR1121 DIO1 */
#endif
}

/**
 * @brief Toggle LED0
 */
void status_led0_toggle(void)
{
    if (!initialized) {
        return;
    }
    led_state = !led_state;
    write_led(led_state);
}

/**
 * @brief Toggle LED1
 */
void status_led1_toggle(void)
{
#if defined(SIW917_ELRS_TARGET_TX)
    if (!initialized) {
        return;
    }
    const bool next_state = ((BIT_LOAD_REG(STATUS_LED_BLUE_GPIO_PIN) & 1U) != 0U);
    write_rgb_led(false, false, next_state);
#else
    /* Unused - LED1 conflicts with LR1121 DIO1 */
#endif
}

/**
 * @brief Run boot LED animation (single LED version)
 */
void status_led_boot_sequence(void)
{
    if (!initialized) {
        return;
    }
    
    LED_DBG("Running boot sequence...\n");
    
    /* Flash LED0 3 times rapidly */
    for (int i = 0; i < 3; i++) {
        write_led(true);
        osDelay(100);
        write_led(false);
        osDelay(100);
    }
    
    LED_DBG("Boot sequence complete\n");
}
