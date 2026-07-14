/**
 * @file bind_button.c
 * @brief Binding mode button driver implementation
 *
 * Configures the active board BTN1 pin for binding mode trigger.
 *
 * Register Configuration Citations:
 * - siw917x-family-rm.pdf Section 11.5/11.6.1 "PAD_CONFIG_REG_x":
 *   Base address: 0x4600_4000
 *   Offset: 0x000 + (0x04 * pin_number)
 *   Bits 7:6 (P) = 01 for pull-up
 *   Bit 5 (SR) = slew rate
 *   Bit 4 (REN) = 1 for receiver enable
 *   Bit 3 (SMT) = Schmitt trigger
 *
 * - siw917x-family-rm.pdf Section 11.12.9 "GPIO_CONFIG_REG_x":
 *   Base address: 0x4604_6000 (EGPIO)
 *   Offset: 0x000 + (0x10 * pin_number) for GPIO_CONFIG_REG_x
 *   Bit 0 (DIRECTION) = 1 for input
 */

#include "bind_button.h"
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

#define BTN_DBG(fmt, ...) DEBUGOUT("[BIND_BTN] " fmt, ##__VA_ARGS__)

#ifndef BIND_BUTTON_DEBUG_VERBOSE
#define BIND_BUTTON_DEBUG_VERBOSE 0
#endif

#if BIND_BUTTON_DEBUG_VERBOSE
#define BTN_DBG_VERBOSE(fmt, ...) BTN_DBG(fmt, ##__VA_ARGS__)
#else
#define BTN_DBG_VERBOSE(fmt, ...) ((void)0)
#endif

/*******************************************************************************
 * Register Definitions
 *
 * Citation: siw917x-family-rm.pdf Chapter 11 "GPIO"
 ******************************************************************************/

/**
 * Pad Configuration Register base
 * Citation: siw917x-family-rm.pdf Section 11.5
 */
/**
 * Pad configuration bit definitions
 * Citation: siw917x-family-rm.pdf Section 11.6.1 "PAD_CONFIG_REG_x"
 */
#define PAD_P_HIZ                   (0UL << 6)   /* High-Z */
#define PAD_P_PULLUP                (1UL << 6)   /* Pull-up */
#define PAD_P_PULLDOWN              (2UL << 6)   /* Pull-down */
#define PAD_REN_ENABLE              (1UL << 4)   /* Receiver enable */
#define PAD_SMT_ENABLE              (1UL << 3)   /* Schmitt trigger */

/**
 * EGPIO Register Base
 * Citation: siw917x-family-rm.pdf Section 11.11 "EGPIO Register Map"
 *   Base address for EGPIO instance: 0x4613_0000
 */
#define EGPIO_BASE                  0x46130000UL

#define BUTTON_GPIO_CONFIG_REG(pin) (*(volatile uint32_t *)(EGPIO_BASE + 0x000 + (0x10UL * (pin))))

/**
 * GPIO configuration bit definitions
 * Citation: siw917x-family-rm.pdf Section 11.12.1 "GPIO_CONFIG_REG_x"
 *   Bits 5:2 (MODE) = GPIO Pin Mode, selects muxing group/mode
 *   Bit 0 (DIRECTION) = 0 for output, 1 for input
 */
#define GPIO_MODE_MASK              (0x0FUL << 2)  /* Bits 5:2 */
#define GPIO_MODE_GPIO              (0x00UL << 2)  /* Direct GPIO mode (mode 0) */
#define GPIO_DIRECTION_BIT          (1UL << 0)     /* 0=output, 1=input */

#define BUTTON_BIT_LOAD_REG(pin)    (*(volatile uint32_t *)(EGPIO_BASE + 0x004 + (0x10UL * (pin))))

/**
 * EGPIO clocks are disabled at reset and must be enabled before direct
 * GPIO register access is reliable.
 */
#define M4CLK_BASE                  0x46000000UL
#define CLK_ENABLE_SET_REG2         (*(volatile uint32_t *)(M4CLK_BASE + 0x008))
#define CLK_ENABLE_SET_REG3         (*(volatile uint32_t *)(M4CLK_BASE + 0x010))
#define EGPIO_PCLK_ENABLE_BIT       (1UL << 21)
#define EGPIO_CLK_ENABLE_BIT        (1UL << 16)

#if defined(SIW917_ELRS_TARGET_TX)
#define BIND_BUTTON_PAD_NUM         SL_SI91X_GPIO_49_PAD
#else
#define BIND_BUTTON_PAD_NUM         SL_SI91X_GPIO_11_PAD
#endif

/*******************************************************************************
 * Module State
 ******************************************************************************/

static bind_button_callback_t button_callback = NULL;
static uint32_t press_start_tick = 0;
static bool was_pressed = false;
static bool initialized = false;
static uint32_t init_tick = 0;
static bool debounce_complete_logged = false;
static uint32_t last_raw_level = 0xFFFFFFFFUL;

/* Debounce time in ms after init before accepting button presses 
 * TX bring-up uses a shorter delay so BTN1 is usable right after boot.
 */
#if defined(SIW917_ELRS_TARGET_TX)
#define INIT_DEBOUNCE_MS  250
#else
#define INIT_DEBOUNCE_MS  2000
#endif

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

static inline uint32_t bind_button_read_raw_level(void)
{
    return BUTTON_BIT_LOAD_REG(BIND_BUTTON_GPIO_PIN) & 0x01U;
}

/**
 * @brief Initialize the binding button GPIO
 */
int bind_button_init(void)
{
    BTN_DBG("Initializing BTN1 (GPIO_%d) for binding mode...\n",
            BIND_BUTTON_GPIO_PIN);

    /* Enable EGPIO clocks and route the button pad into the GPIO controller. */
    CLK_ENABLE_SET_REG2 = EGPIO_PCLK_ENABLE_BIT;
    CLK_ENABLE_SET_REG3 = EGPIO_CLK_ENABLE_BIT;
    RSI_CLK_PeripheralClkEnable(M4CLK, EGPIO_CLK, ENABLE_STATIC_CLK);
    RSI_EGPIO_PadSelectionEnable(BIND_BUTTON_PAD_NUM);
    RSI_EGPIO_SetPinMux(EGPIO, 0, BIND_BUTTON_GPIO_PIN, EGPIO_PIN_MUX_MODE0);
    BTN_DBG_VERBOSE("  EGPIO clocks enabled, selected pad %u, mux=GPIO mode 0\n",
                    (unsigned)BIND_BUTTON_PAD_NUM);
    
    /* Step 2: Configure pad for input with pull-up
     * Citation: siw917x-family-rm.pdf Section 11.6.1
     * - Configure pull direction to match the board's button polarity
     * - REN = 1 (receiver enabled)
     * - SMT = 1 (Schmitt trigger for noise immunity)
     */
    const uint32_t pad_disable_state = PAD_P_PULLUP;
    PAD_CONFIG_REG(BIND_BUTTON_GPIO_PIN) =
        pad_disable_state | PAD_REN_ENABLE | PAD_SMT_ENABLE;
    BTN_DBG_VERBOSE("  PAD_CONFIG_REG_%d = 0x%08lX (%s, REN, SMT)\n",
                    BIND_BUTTON_GPIO_PIN,
                    (unsigned long)PAD_CONFIG_REG(BIND_BUTTON_GPIO_PIN),
                    (pad_disable_state == PAD_P_PULLDOWN) ? "pull-down" : "pull-up");
    
    /* Step 3: Configure GPIO for direct control, input direction
     * Citation: siw917x-family-rm.pdf Section 11.12.9
     * - MODE = 0 (GPIO mode)
     * - DIRECTION = 1 (input)
     */
    uint32_t cfg = BUTTON_GPIO_CONFIG_REG(BIND_BUTTON_GPIO_PIN);
    cfg &= ~GPIO_MODE_MASK;              /* Clear MODE bits */
    cfg |= GPIO_MODE_GPIO;               /* Set MODE = 0 (GPIO) */
    cfg |= GPIO_DIRECTION_BIT;           /* Set direction = input */
    BUTTON_GPIO_CONFIG_REG(BIND_BUTTON_GPIO_PIN) = cfg;
    BTN_DBG_VERBOSE("  GPIO_CONFIG_REG_%d = 0x%08lX (GPIO mode, input)\n",
                    BIND_BUTTON_GPIO_PIN,
                    (unsigned long)BUTTON_GPIO_CONFIG_REG(BIND_BUTTON_GPIO_PIN));
    
    /* Allow GPIO to settle before reading */
    for (volatile int i = 0; i < 100000; i++) { /* ~1ms delay */ }
    
    /* Read initial state and initialize was_pressed to match
     * This prevents false trigger on first poll if button reads as pressed
     */
    initialized = true;  /* Must set before calling is_pressed */
    last_raw_level = bind_button_read_raw_level();
    bool pressed = bind_button_is_pressed();
    was_pressed = pressed;  /* Initialize to current state to prevent false edge detection */
    BTN_DBG("  Initial state: %s\n", pressed ? "PRESSED" : "released");
    
    /* Record init time for debounce */
    init_tick = osKernelGetTickCount();
    debounce_complete_logged = false;
    
    BTN_DBG("BTN1 initialization complete\n");
    BTN_DBG("  Hold BTN1 for %d seconds to enter binding mode\n", 
            BIND_BUTTON_LONG_PRESS_MS / 1000);
    
    return 0;
}

/**
 * @brief Check if binding button is currently pressed
 */
bool bind_button_is_pressed(void)
{
    if (!initialized) {
        return false;
    }
    
    /* Read selected button GPIO state via BIT_LOAD register
     * Citation: siw917x-family-rm.pdf Section 11.12.4
     * Bit 0 contains the pin logic level
     *
     * Both supported board paths use active-low buttons:
     * - BRD2708A RX path: idle HIGH via pull-up, pressed LOW to GND
     * - BRD2605A TX path: idle HIGH via pull-up, pressed LOW to GND
     */
    uint32_t pin_state = bind_button_read_raw_level();
    return (pin_state == 0U);  /* Active low: pressed when LOW */
}

/**
 * @brief Print diagnostic information about button GPIO state
 * 
 * Call this to debug button issues - prints all relevant register values.
 */
void bind_button_print_diagnostics(void)
{
    BTN_DBG("=== BTN1 GPIO_%d Diagnostics ===\n", BIND_BUTTON_GPIO_PIN);
    BTN_DBG("  PAD_CONFIG_REG_%d     = 0x%08lX (expect board-matched pull + REN + SMT)\n",
            BIND_BUTTON_GPIO_PIN,
            (unsigned long)PAD_CONFIG_REG(BIND_BUTTON_GPIO_PIN));
    BTN_DBG("  GPIO_CONFIG_REG_%d    = 0x%08lX (expect MODE=0, DIR=input)\n",
            BIND_BUTTON_GPIO_PIN,
            (unsigned long)BUTTON_GPIO_CONFIG_REG(BIND_BUTTON_GPIO_PIN));
    BTN_DBG("  BIT_LOAD_REG_%d       = 0x%08lX (bit0 = pin state)\n",
            BIND_BUTTON_GPIO_PIN,
            (unsigned long)BUTTON_BIT_LOAD_REG(BIND_BUTTON_GPIO_PIN));
    BTN_DBG("  Button state: %s\n", bind_button_is_pressed() ? "PRESSED" : "RELEASED");
    BTN_DBG("  Callback registered: %s\n", button_callback ? "YES" : "NO");
    BTN_DBG("  Initialized: %s\n", initialized ? "YES" : "NO");
    if (was_pressed) {
        uint32_t duration = bind_button_get_press_duration();
        BTN_DBG("  Currently held for: %lu ms\n", (unsigned long)duration);
    }
}

/**
 * @brief Register callback for button events
 */
void bind_button_set_callback(bind_button_callback_t callback)
{
    button_callback = callback;
    BTN_DBG_VERBOSE("Callback %s\n", callback ? "registered" : "unregistered");
}

/**
 * @brief Poll button state and handle press detection
 */
void bind_button_poll(void)
{
    if (!initialized) {
        return;
    }
    
    uint32_t now = osKernelGetTickCount();
    
    /* Ignore button events during post-init debounce period
     * This prevents false triggers from GPIO settling noise
     */
    if ((now - init_tick) < INIT_DEBOUNCE_MS) {
        /* Still in debounce period - do not latch a press yet. */
        static uint32_t last_debounce_print = 0;
        if ((now - last_debounce_print) > 500) {
            last_debounce_print = now;
            BTN_DBG_VERBOSE("Debounce: %lu/%d ms, btn=%s\n",
                            (unsigned long)(now - init_tick), INIT_DEBOUNCE_MS,
                            bind_button_is_pressed() ? "PRESSED" : "released");
        }
        was_pressed = false;
        press_start_tick = 0;
        debounce_complete_logged = false;
        return;
    }
    
    uint32_t raw_level = bind_button_read_raw_level();
    if (raw_level != last_raw_level) {
        BTN_DBG_VERBOSE("Raw GPIO_%d level changed: %lu -> %lu\n",
                        BIND_BUTTON_GPIO_PIN,
                        (unsigned long)last_raw_level,
                        (unsigned long)raw_level);
        last_raw_level = raw_level;
    }

    bool is_pressed = (raw_level == 0U);

    if (!debounce_complete_logged) {
        debounce_complete_logged = true;
        BTN_DBG_VERBOSE("Debounce complete, btn=%s\n",
                        is_pressed ? "PRESSED" : "released");
        if (is_pressed) {
            press_start_tick = now;
            was_pressed = true;
            BTN_DBG_VERBOSE("Button held after debounce (starting timer)\n");
            return;
        }
    }
    
    if (is_pressed && !was_pressed) {
        /* Button just pressed - start timing */
        press_start_tick = now;
        was_pressed = true;
        BTN_DBG("Button pressed (starting timer)\n");
    }
    else if (!is_pressed && was_pressed) {
        /* Button just released - check duration */
        uint32_t duration = now - press_start_tick;
        was_pressed = false;
        
        bool is_long_press = (duration >= BIND_BUTTON_LONG_PRESS_MS);
        
        BTN_DBG("Button released after %lu ms (%s press)\n", 
                (unsigned long)duration,
                is_long_press ? "LONG" : "short");
        
        /* Invoke callback */
        if (button_callback != NULL) {
            button_callback(is_long_press);
        }
    }
}

/**
 * @brief Get press duration if button is currently held
 */
uint32_t bind_button_get_press_duration(void)
{
    if (!was_pressed) {
        return 0;
    }
    
    uint32_t now = osKernelGetTickCount();
    return now - press_start_tick;
}
