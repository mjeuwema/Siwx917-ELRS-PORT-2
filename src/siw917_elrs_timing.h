#pragma once

/*
 * Timing-first RX profile for SiW917 ELRS.
 *
 * Keep this enabled for high packet-rate work. It compiles out diagnostic
 * bookkeeping that was useful while debugging 50/150 Hz telemetry, but adds
 * jitter and serial load that work against 250/500 Hz operation.
 */
#ifndef SIW917_ELRS_TIMING_LEAN
#define SIW917_ELRS_TIMING_LEAN 1
#endif

/* Boot/init timer logs are useful for bring-up, but not for timing runs. */
#ifndef SIW917_ELRS_TIMER_VERBOSE_INIT
#define SIW917_ELRS_TIMER_VERBOSE_INIT (!SIW917_ELRS_TIMING_LEAN)
#endif

/*
 * The Silicon Labs config-timer ISR is generic: it checks every CT event type,
 * writes a callback flag, then indirect-calls the registered callback. ELRS only
 * needs Counter 0 peak interrupts, so in timing mode we vector CT_IRQn directly
 * to a tiny handler after the SDK has configured the timer and interrupt mask.
 */
#ifndef SIW917_ELRS_DIRECT_CT_IRQ
// Install a direct CT ISR through a RAM vector table. Do not use NVIC_SetVector
// against the boot VTOR: on this image the boot vector table is flash-backed.
#define SIW917_ELRS_DIRECT_CT_IRQ SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Run the mid-slot TICK directly from the CT interrupt. TICK only handles LQ
 * bookkeeping, so this removes one queued FreeRTOS wake per RF period without
 * issuing LR1121 SPI from CT IRQ context.
 */
#ifndef SIW917_ELRS_DIRECT_TIMER_TICK
#define SIW917_ELRS_DIRECT_TIMER_TICK SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Full direct Tick+Tock was flight-tested and rejected: TOCK performs FHSS and
 * telemetry radio commands, and the SiW917 GSPI/SDK transfer path times out
 * when those commands are issued from CT IRQ context.
 */
#ifndef SIW917_ELRS_DIRECT_TIMER_CALLBACKS
#define SIW917_ELRS_DIRECT_TIMER_CALLBACKS 0
#endif

/*
 * CT still wakes the ELRS task with osThreadFlagsSet(), so priority 5 is the
 * highest legal value with this FreeRTOS port. Going numerically lower would
 * require a second-stage low-priority wake interrupt.
 */
#ifndef SIW917_ELRS_CT_IRQ_PRIORITY
#define SIW917_ELRS_CT_IRQ_PRIORITY 5
#endif

/*
 * Prepare the next RF telemetry packet ahead of the TOCK edge. Flight testing
 * did not improve 150 Hz stability, so keep it off while isolating timing.
 */
#ifndef SIW917_ELRS_PREBUILD_TLM_PACKET
#define SIW917_ELRS_PREBUILD_TLM_PACKET 0
#endif

/*
 * Use ARM DWT->CYCCNT as the fast monotonic microsecond source. This avoids the
 * RTOS SysTick sampling jitter in packet-edge/PFD timestamps while preserving a
 * software-extended micros() counter instead of exposing raw 32-bit cycles.
 */
#ifndef SIW917_ELRS_DWT_MICROS
#define SIW917_ELRS_DWT_MICROS SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Keep LR1121 RX/TX-turnaround SPI transactions off the IRQ-driven GSPI wait
 * path. The pumped helper services the GSPI ISR synchronously, which removes
 * one more source of FreeRTOS/NVIC scheduling jitter without touching the
 * higher-risk direct register FIFO backend.
 */
#ifndef SIW917_ELRS_POLLED_HOT_SPI
#define SIW917_ELRS_POLLED_HOT_SPI SIW917_ELRS_TIMING_LEAN
#endif

/*
 * In the connected RF hot path, handle LR1121 DIO1 immediately instead of
 * bouncing through the RX task first. Startup/scanning still use the deferred
 * path because rate changes and bring-up are less timing-sensitive and easier
 * to recover if an IRQ arrives during radio reconfiguration.
 */
#ifndef SIW917_ELRS_DIRECT_DIO_ISR
#define SIW917_ELRS_DIRECT_DIO_ISR SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Bypass the Silicon Labs GPIO pin interrupt dispatcher for LR1121 DIO9
 * (legacy DIO1 naming). The SDK still configures the EGPIO channel, but the
 * NVIC vector points directly at our tiny clear-and-callback ISR.
 */
#ifndef SIW917_ELRS_DIRECT_DIO_VECTOR
#define SIW917_ELRS_DIRECT_DIO_VECTOR SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Split LR1121 DIO processing into two IRQ stages:
 *   - EGPIO pin IRQ: timestamp, mask DIO, pend software stage only.
 *   - Software-pended stage IRQ: perform LR1121 SPI/IRQ processing.
 *
 * Keep both priorities FreeRTOS-safe for the first flight test. Once this is
 * proven, the first stage can move above the FreeRTOS syscall ceiling because
 * it no longer calls OS APIs or touches GSPI.
 */
#ifndef SIW917_ELRS_TWO_STAGE_DIO_ISR
#define SIW917_ELRS_TWO_STAGE_DIO_ISR SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_DIO_IRQ_PRIORITY
#define SIW917_ELRS_DIO_IRQ_PRIORITY 4
#endif

#ifndef SIW917_ELRS_DIO_STAGE_IRQ_PRIORITY
#define SIW917_ELRS_DIO_STAGE_IRQ_PRIORITY 5
#endif

/*
 * Once connected, do not let a hot-path LR1121 command fall into long fallback
 * BUSY waits. If BUSY does not clear quickly, the slot is already lost; failing
 * fast is better than blocking the RF timing island for milliseconds.
 */
#ifndef SIW917_ELRS_BUSY_FAST_ONLY_WHEN_CONNECTED
#define SIW917_ELRS_BUSY_FAST_ONLY_WHEN_CONNECTED SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_BUSY_PORT_READ
#define SIW917_ELRS_BUSY_PORT_READ 0
#endif

#ifndef SIW917_ELRS_BUSY_FAST_ITERATIONS
#define SIW917_ELRS_BUSY_FAST_ITERATIONS 40000U
#endif

/*
 * Use a direct GSPI register/FIFO transfer for LR1121 GET_PACKET. The raw
 * helper mirrors the SiWx917 peripheral driver's FIFO sequence but bypasses
 * the CMSIS transfer state machine in the RX hot path. It stays scoped to the
 * packet-read primitive so fallback is one macro flip if the GSPI state machine
 * disagrees.
 */
#ifndef SIW917_ELRS_RAW_GSPI_GET_PACKET
#define SIW917_ELRS_RAW_GSPI_GET_PACKET SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Fused SetFrequency+SetRx is issued on every FHSS hop. Once raw GET_PACKET is
 * proven, move this retune command to the same direct GSPI path so 100 Hz and
 * faster rates do not pay the pumped CMSIS transfer cost on each slot.
 */
#ifndef SIW917_ELRS_RAW_GSPI_SET_FREQ_RX
#define SIW917_ELRS_RAW_GSPI_SET_FREQ_RX SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Non-upstream retune shortcut: after SetStandby, fold the next SetRfFrequency
 * into a custom SetFreq+SetRx helper. Keep this disabled by default so the ELRS
 * state machine behaves like ESP32/upstream; enable only for explicit A/B tests.
 */
#ifndef SIW917_ELRS_FUSED_RX_RETUNE
#define SIW917_ELRS_FUSED_RX_RETUNE 0
#endif

/*
 * Remaining connected telemetry hot-path commands:
 *   - ClearIrq/GetIrqStatus runs on every LR1121 DIO event.
 *   - WriteBuffer8_SetTx starts every telemetry response.
 *   - SetRx returns the radio to continuous RX after telemetry TX_DONE.
 *
 * Keep these separate so each can be backed out independently if a raw command
 * disagrees with the LR1121 firmware helper.
 */
#ifndef SIW917_ELRS_RAW_GSPI_CLEAR_IRQ
#define SIW917_ELRS_RAW_GSPI_CLEAR_IRQ SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_RAW_GSPI_TX
#define SIW917_ELRS_RAW_GSPI_TX SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_RAW_GSPI_SET_RX
#define SIW917_ELRS_RAW_GSPI_SET_RX SIW917_ELRS_TIMING_LEAN
#endif
