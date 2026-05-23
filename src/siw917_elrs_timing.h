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
 * Run TOCK directly from the CT interrupt, matching upstream's deterministic
 * timer model. This is only safe because the TOCK hot-path LR1121 commands
 * below are routed to raw/polled GSPI instead of the SDK/DMA transfer path.
 */
#ifndef SIW917_ELRS_DIRECT_TIMER_CALLBACKS
#define SIW917_ELRS_DIRECT_TIMER_CALLBACKS SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Keep CT below the LR1121 DIO stage IRQ. At 150 Hz+, TX_DONE/RX_DONE can land
 * close to the next TOCK; if CT wins that tie it may FHSS-retune while the radio
 * is still BUSY from telemetry TX. DIO-stage-first ordering lets the radio
 * clear TX_DONE and return to RX before the next timer-driven SetRfFrequency.
 *
 * Priority 6 remains FreeRTOS-safe and keeps the direct timer path deterministic
 * enough while avoiding the 0x020B BUSY timeout seen at 150 Hz.
 */
#ifndef SIW917_ELRS_CT_IRQ_PRIORITY
#define SIW917_ELRS_CT_IRQ_PRIORITY 6
#endif

/*
 * Keep telemetry packet construction in the timer/tock callback by default.
 * ESP32 builds and sends telemetry from HWtimerCallbackTock(); prebuilding is
 * useful as an experiment, but it is not upstream-equivalent behavior.
 */
#ifndef SIW917_ELRS_PREBUILD_TLM_PACKET
#define SIW917_ELRS_PREBUILD_TLM_PACKET 0
#endif

/*
 * Do not send telemetry from RX_DONE. Upstream ELRS sends telemetry only from
 * the timer/tock callback after OtaNonce++ and HandleFHSS(). Keep this disabled
 * unless deliberately running a non-upstream timing experiment.
 */
#ifndef SIW917_ELRS_EARLY_150HZ_TLM_TX
#define SIW917_ELRS_EARLY_150HZ_TLM_TX 0
#endif

/*
 * Experimental SiW917 ISR boundary: keep the upstream telemetry slot decision
 * at TOCK, but start the LR1121 TX command from the ELRS task.
 *
 * Flight testing rejected this as the default: the queue drains, but the actual
 * downlink leaves too late for the TX telemetry receive window at 150 Hz+ and
 * can overlap the next timer-driven FHSS retune. Keep it available only as an
 * explicit A/B diagnostic switch.
 */
#ifndef SIW917_ELRS_DEFER_TLM_TX_FROM_TIMER
#define SIW917_ELRS_DEFER_TLM_TX_FROM_TIMER 0
#endif

/*
 * Hot-path timing probes. These keep only high-water counters and print nothing
 * from ISR context; elrs_main reports them on disconnect. Disable after the
 * bottleneck is found to remove the timestamp-read overhead.
 */
#ifndef SIW917_ELRS_HOTPATH_TIMING_DIAG
#define SIW917_ELRS_HOTPATH_TIMING_DIAG 1
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
 * Once the RX has a live link candidate, handle LR1121 DIO1 immediately
 * instead of bouncing through the RX task first. Upstream ESP32 does this for
 * every radio IRQ; keeping disconnected scanning deferred is the SiW917-safe
 * compromise while matching ESP32 ordering for tentative/connected telemetry.
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
 * Direct GPIO-vector DIO processing is still too aggressive on SiW917: the
 * disconnected/scanning path can need a task wake from a GPIO IRQ priority that
 * is not FreeRTOS-safe, which wedges before link acquisition. Keep the proven
 * two-stage handoff and focus the bare-metal work on the LR1121 SPI commands.
 */
#ifndef SIW917_ELRS_TWO_STAGE_DIO_ISR
#define SIW917_ELRS_TWO_STAGE_DIO_ISR SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Direct GPIO-vector DIO remains an A/B-only experiment. It can connect, but
 * flight tests showed immediate telemetry loss even when gated until locked.
 * Keep the proven two-stage DIO path as the default on SiW917.
 */
#ifndef SIW917_ELRS_DIRECT_GPIO_DIO_WHEN_LINKED
#define SIW917_ELRS_DIRECT_GPIO_DIO_WHEN_LINKED 0
#endif

#ifndef SIW917_ELRS_DIO_IRQ_PRIORITY
#define SIW917_ELRS_DIO_IRQ_PRIORITY 4
#endif

#ifndef SIW917_ELRS_DIO_STAGE_IRQ_PRIORITY
#define SIW917_ELRS_DIO_STAGE_IRQ_PRIORITY 5
#endif

/*
 * Keep SDK GPIO setup for portability, but remove SDK helper calls from the
 * DIO1 hot path. DIO edge clear and DIO level reads happen on every packet, so
 * use the same direct EGPIO/register path we already use for LR1121 BUSY.
 */
#ifndef SIW917_ELRS_DIRECT_DIO_GPIO_REGS
#define SIW917_ELRS_DIRECT_DIO_GPIO_REGS SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Once tentatively/fully linked, do not let a hot-path LR1121 command fall into
 * long fallback BUSY waits. Upstream ESP32 polls BUSY for up to 2000 us in the
 * ISR path; match that budget with a DWT-backed microsecond wait instead of
 * entering the 100 ms bring-up fallback.
 */
#ifndef SIW917_ELRS_BUSY_FAST_ONLY_WHEN_CONNECTED
#define SIW917_ELRS_BUSY_FAST_ONLY_WHEN_CONNECTED SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_BUSY_PORT_READ
#define SIW917_ELRS_BUSY_PORT_READ 0
#endif

/*
 * Match upstream ESP32 LR1121_hal.cpp's 2000 us WaitOnBusy budget. Keep the
 * older iteration knob only for low-level A/B experiments; ELRS uses the
 * microsecond budget so the wait is stable across compiler/register changes.
 */
#ifndef SIW917_ELRS_BUSY_FAST_US
#define SIW917_ELRS_BUSY_FAST_US 2000U
#endif

/*
 * Upstream ESP32 LR1121_hal.cpp calls WaitOnBusy(), but it does not abort the
 * SPI command if that 2000 us wait times out. Match that behavior in timing
 * mode: skipping SetRx or ClearIrq after a telemetry TX can strand the radio
 * outside RX and looks exactly like the 150 Hz telemetry-loss failure.
 */
#ifndef SIW917_ELRS_CONTINUE_AFTER_BUSY_TIMEOUT
#define SIW917_ELRS_CONTINUE_AFTER_BUSY_TIMEOUT SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_BUSY_FAST_ITERATIONS
#define SIW917_ELRS_BUSY_FAST_ITERATIONS 4096U
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
 * Fixed-function bare-metal GetPacket command+response path.
 *
 * This removes the generic raw GSPI wrapper and the init-style 100 ms BUSY
 * waits from the RX_DONE hot path. Packet reads should use the same tiny,
 * time-bounded register loop as the clear/write hot commands.
 */
#ifndef SIW917_ELRS_FAST_GET_PACKET
#define SIW917_ELRS_FAST_GET_PACKET SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Keep upstream FHSS semantics: hops use SetRfFrequency only, not the custom
 * SetFreq+SetRx helper. Route that command through raw GSPI so direct timer
 * TOCK can still run without the SDK/DMA transfer path.
 */
#ifndef SIW917_ELRS_RAW_GSPI_SET_FREQ
#define SIW917_ELRS_RAW_GSPI_SET_FREQ SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Custom SetFrequency+SetRx helper. This is useful for explicit A/B tests but
 * is not used by the upstream FHSS path.
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

/*
 * Fixed-function bare-metal ClearIrq/GetIrqStatus transaction.
 *
 * Upstream LR1121 uses ClearIrq(0xFFFFFFFF) as an atomic get+clear. The generic
 * raw GSPI helper is still too much wrapper for 150 Hz telemetry, so this path
 * emits the six-byte transfer directly with a tiny GSPI register loop.
 */
#ifndef SIW917_ELRS_FAST_CLEAR_IRQ
#define SIW917_ELRS_FAST_CLEAR_IRQ SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Fixed-function bare-metal hot write commands.
 *
 * The generic raw GSPI helper is useful as a safe fallback, but the 100/150 Hz
 * telemetry window is dominated by a small set of short write-only commands:
 * WriteBuffer8_SetTx, SetRx, SetRfFrequency, and the optional fused
 * SetRfFrequency_SetRx. This path emits those commands with the same tiny
 * register loop used by FAST_CLEAR_IRQ.
 */
#ifndef SIW917_ELRS_FAST_HOT_COMMANDS
#define SIW917_ELRS_FAST_HOT_COMMANDS SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_RAW_GSPI_TX
#define SIW917_ELRS_RAW_GSPI_TX SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_RAW_GSPI_SET_RX
#define SIW917_ELRS_RAW_GSPI_SET_RX SIW917_ELRS_TIMING_LEAN
#endif

/*
 * 150 Hz telemetry leaves only a few hundred microseconds between TX_DONE and
 * the next uplink. ESP32 can clear/read IRQ status and re-arm RX inside that
 * window; SiW917 cannot always do both before the uplink starts. When the
 * driver knows it is in a TX, handle that DIO as TX_DONE, re-arm RX first, then
 * clear just the TX_DONE IRQ bit. RX_DONE that arrives during the clear remains
 * latched and is picked up by the level requeue path.
 */
#ifndef SIW917_ELRS_RX_FIRST_TXDONE
#define SIW917_ELRS_RX_FIRST_TXDONE SIW917_ELRS_TIMING_LEAN
#endif
