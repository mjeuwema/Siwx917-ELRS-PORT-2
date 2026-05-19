/**
 * @file Arduino.cpp
 * @brief Arduino API implementation for SiW917
 *
 * Implements Arduino-compatible functions using SiW917 SDK.
 */

#include "targets.h"
#include "siw917_elrs_timing.h"

// Suppress missing-field-initializers warnings from SDK headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

// Use the same GPIO headers as the working lr1121_driver.c
#include "rsi_egpio.h"
#include "rsi_rom_egpio.h"
#include "rsi_rom_clks.h"

#pragma GCC diagnostic pop

#include "cmsis_os2.h"
#include <cstdio>
#include <cstdlib>

// Include our C driver for GPIO operations
extern "C" {
#include "lr1121_driver.h"
}

//=============================================================================
// Serial Output
//=============================================================================

HardwareSerial Serial;
Stream *BackpackOrLogStrm = &Serial;

size_t HardwareSerial::write(uint8_t c) {
    // Use SiW917 debug UART via printf
    putchar(c);
    return 1;
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size) {
    for (size_t i = 0; i < size; i++) {
        putchar(buffer[i]);
    }
    return size;
}

//=============================================================================
// Timing Functions
//=============================================================================

extern "C" {

uint32_t millis(void) {
    return osKernelGetTickCount();
}

static uint32_t micros_systick_fallback(void) {
    // Combine the RTOS tick counter with the live SysTick down-counter to
    // preserve sub-millisecond timing. ELRS phase locking depends on this.
    uint32_t tick_before;
    uint32_t tick_after;
    uint32_t systick_val;
    const uint32_t systick_reload = SysTick->LOAD + 1U;

    do {
        tick_before = osKernelGetTickCount();
        systick_val = SysTick->VAL;
        tick_after = osKernelGetTickCount();
    } while (tick_before != tick_after);

    uint32_t elapsed_cycles = systick_reload - systick_val;
    uint32_t sub_ms_us =
        (uint32_t)(((uint64_t)elapsed_cycles * 1000ULL) / systick_reload);

    return (tick_before * 1000U) + sub_ms_us;
}

#if SIW917_ELRS_DWT_MICROS
extern uint32_t SystemCoreClock;

static bool micros_dwt_ready = false;
static bool micros_dwt_unavailable = false;
static uint32_t micros_dwt_cycles_per_us = 0;
static uint32_t micros_dwt_last_cycles = 0;
static uint64_t micros_dwt_accum_us = 0;
static uint32_t micros_dwt_remainder_cycles = 0;

static inline uint32_t micros_enter_critical(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void micros_exit_critical(uint32_t primask) {
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

static bool micros_dwt_init_locked(void) {
    if (micros_dwt_ready) {
        return true;
    }
    if (micros_dwt_unavailable) {
        return false;
    }

    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        micros_dwt_unavailable = true;
        return false;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();

    const uint32_t before = DWT->CYCCNT;
    for (volatile uint32_t spin = 0; spin < 64U; ++spin) {
        __NOP();
    }
    const uint32_t after = DWT->CYCCNT;
    if (after == before) {
        micros_dwt_unavailable = true;
        return false;
    }

    micros_dwt_cycles_per_us = cycles_per_us;
    micros_dwt_last_cycles = after;
    micros_dwt_accum_us = micros_systick_fallback();
    micros_dwt_remainder_cycles = 0U;
    micros_dwt_ready = true;
    return true;
}

static uint32_t micros_dwt_extended(void) {
    uint32_t primask = micros_enter_critical();
    if (!micros_dwt_init_locked()) {
        micros_exit_critical(primask);
        return micros_systick_fallback();
    }

    const uint32_t now_cycles = DWT->CYCCNT;
    const uint32_t delta_cycles = now_cycles - micros_dwt_last_cycles;
    micros_dwt_last_cycles = now_cycles;

    uint32_t elapsed_us;
    uint32_t remainder_cycles;
    const uint32_t cycles = delta_cycles + micros_dwt_remainder_cycles;
    if (cycles < delta_cycles) {
        const uint64_t wide_cycles =
            (uint64_t)delta_cycles + micros_dwt_remainder_cycles;
        elapsed_us = (uint32_t)(wide_cycles / micros_dwt_cycles_per_us);
        remainder_cycles = (uint32_t)(wide_cycles % micros_dwt_cycles_per_us);
    } else {
        elapsed_us = cycles / micros_dwt_cycles_per_us;
        remainder_cycles = cycles - (elapsed_us * micros_dwt_cycles_per_us);
    }

    micros_dwt_accum_us += elapsed_us;
    micros_dwt_remainder_cycles = remainder_cycles;
    const uint32_t result = (uint32_t)micros_dwt_accum_us;

    micros_exit_critical(primask);
    return result;
}

bool micros_uses_dwt(void) {
    return micros_dwt_ready && !micros_dwt_unavailable;
}
#endif

uint32_t micros(void) {
#if SIW917_ELRS_DWT_MICROS
    return micros_dwt_extended();
#else
    return micros_systick_fallback();
#endif
}

void delay(uint32_t ms) {
    osDelay(ms);
}

void delayMicroseconds(uint32_t us) {
    if (us == 0) {
        return;
    }

    uint32_t start = micros();
    while ((micros() - start) < us) {
        __NOP();
    }
}

void yield(void) {
    osThreadYield();
}

//=============================================================================
// GPIO Functions  
// Using RSI EGPIO API (same as working lr1121_driver.c)
//=============================================================================

void pinMode(int pin, int mode) {
    if (pin == UNDEF_PIN) return;
    
    // Enable EGPIO clock if not already enabled
    RSI_CLK_PeripheralClkEnable(M4CLK, EGPIO_CLK, ENABLE_STATIC_CLK);
    
    // Configure pad selection for the pin
    RSI_EGPIO_PadSelectionEnable(pin / 16 + 1);  // PAD 1-4 based on pin range
    
    // Set pin MUX to GPIO mode (mode 0)
    RSI_EGPIO_SetPinMux(EGPIO, 0, pin, 0);  // Port 0, GPIO mode
    
    if (mode == OUTPUT) {
        RSI_EGPIO_SetDir(EGPIO, 0, pin, 0);  // 0 = output
    } else {
        RSI_EGPIO_SetDir(EGPIO, 0, pin, 1);  // 1 = input
    }
}

void digitalWrite(int pin, int value) {
    if (pin == UNDEF_PIN) return;
    
    if (value) {
        RSI_EGPIO_SetPin(EGPIO, 0, pin, 1);
    } else {
        RSI_EGPIO_SetPin(EGPIO, 0, pin, 0);
    }
}

int digitalRead(int pin) {
    if (pin == UNDEF_PIN) return LOW;
    
    return RSI_EGPIO_GetPin(EGPIO, 0, pin) ? HIGH : LOW;
}

// Interrupt callback storage
static void (*gpio_callbacks[64])(void) = {nullptr};

void attachInterrupt(int pin, void (*callback)(void), int mode) {
    if (pin == UNDEF_PIN || pin >= 64) return;
    
    gpio_callbacks[pin] = callback;
    
    // Note: Actual interrupt setup is handled by lr1121_driver for DIO1
    // This Arduino shim just stores the callback
    // For DIO1, lr1121_driver.c configures the UULP GPIO interrupt directly
    (void)mode;  // Mode handled in driver setup
}

void detachInterrupt(int pin) {
    if (pin == UNDEF_PIN || pin >= 64) return;
    gpio_callbacks[pin] = nullptr;
}

} // extern "C"

//=============================================================================
// C++ Runtime Support (for baremetal embedded)
//=============================================================================

// These are required when using C++ classes with virtual destructors
// but not using the full C++ standard library
void* operator new(size_t size) {
    void* ptr = malloc(size);
    if (ptr == nullptr) {
        std::fputs("fatal: operator new failed\n", stderr);
        std::abort();
    }
    return ptr;
}

void* operator new[](size_t size) {
    void* ptr = malloc(size);
    if (ptr == nullptr) {
        std::fputs("fatal: operator new[] failed\n", stderr);
        std::abort();
    }
    return ptr;
}

void operator delete(void* ptr) noexcept {
    free(ptr);
}

void operator delete[](void* ptr) noexcept {
    free(ptr);
}

void operator delete(void* ptr, size_t) noexcept {
    free(ptr);
}

void operator delete[](void* ptr, size_t) noexcept {
    free(ptr);
}
