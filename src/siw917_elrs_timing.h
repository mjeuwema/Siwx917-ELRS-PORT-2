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

/*
 * Dual-LR1121 bring-up switches.
 *
 * PROBE initializes and version-checks a second LR1121 on the shared GSPI bus,
 * but keeps upstream ELRS in single-radio mode. Use this first to validate
 * wiring and chip-select/BUSY/reset timing without risking the working RX link.
 *
 * UPSTREAM_DUAL exposes GPIO_PIN_NSS_2 to ELRS, advertises true diversity, and
 * lets upstream dual-radio/Gemini packet paths run. Do not enable until PROBE
 * passes cleanly with both radios wired; the first Radio2 IRQ implementation is
 * intentionally deferred/safe rather than timing-optimized.
 */
#ifndef SIW917_ELRS_DUAL_RADIO_PROBE
#define SIW917_ELRS_DUAL_RADIO_PROBE 1
#endif

#ifndef SIW917_ELRS_UPSTREAM_DUAL_RADIO
#define SIW917_ELRS_UPSTREAM_DUAL_RADIO 1
#endif

#ifndef SIW917_ELRS_RADIO2_IRQ_READY
#define SIW917_ELRS_RADIO2_IRQ_READY 1
#endif

/*
 * Keep LR1121 dual-band/crossband air rates gated separately from same-band
 * Gemini/diversity bring-up. Enable this only when the port explicitly routes
 * radio1 on the primary sub-GHz FHSS domain and radio2 on the 2.4 GHz domain.
 */
#ifndef SIW917_ELRS_ENABLE_CROSSBAND_RATES
#define SIW917_ELRS_ENABLE_CROSSBAND_RATES 1
#endif

#ifndef SIW917_ELRS_CROSSBAND_TEST_LOG
#define SIW917_ELRS_CROSSBAND_TEST_LOG 0
#endif

/*
 * The 900 MHz SF5 acquisition rates are normal selectable TX modes. Keep this
 * as an emergency opt-out only; disabling it skips FCC915 250 Hz during scan.
 */
#ifndef SIW917_ELRS_SKIP_FAST_SF5_900_SCAN
#define SIW917_ELRS_SKIP_FAST_SF5_900_SCAN 0
#endif

#ifndef SIW917_ELRS_SKIP_900_200HZ_FULL_SCAN
#define SIW917_ELRS_SKIP_900_200HZ_FULL_SCAN 0
#endif

/*
 * 200 Hz Full on 900 MHz has a 5 ms RF interval with a long full-res packet.
 * Keep this as an emergency SiW917-only guardrail while timing is being
 * hardened. The upstream-equivalent default is 2, which does not clamp a TX
 * request for 1:2 telemetry.
 */
#ifndef SIW917_ELRS_MIN_TLM_DENOM_200HZ_FULL_GEMINI
#define SIW917_ELRS_MIN_TLM_DENOM_200HZ_FULL_GEMINI 2
#endif

/*
 * Matched TX power changes are not protocol-critical on every telemetry slot,
 * but committing them costs extra LR1121 commands. In the tightest SiW917 case
 * (900 MHz 200 Hz Full + Gemini + 1:2 telemetry), defer those commits until the
 * link moves to a less timing-sensitive mode.
 */
#ifndef SIW917_ELRS_DEFER_PWR_COMMIT_200HZ_FULL_GEMINI
#define SIW917_ELRS_DEFER_PWR_COMMIT_200HZ_FULL_GEMINI 1
#endif

#ifndef SIW917_ELRS_LOG_CONNECTED_PWR_UPDATES
#define SIW917_ELRS_LOG_CONNECTED_PWR_UPDATES 0
#endif

/*
 * The SiW917 HAL performs the Waveshare TCXO/XOSC bring-up and CalibImage for
 * each LR1121 before LR1121Driver::Begin() returns to the upstream-shaped
 * driver code. Re-running CalibImage here has proven intermittent after the
 * radios are already configured, so leave the runtime duplicate disabled.
 */
#ifndef SIW917_ELRS_RUNTIME_CALIB_IMAGE
#define SIW917_ELRS_RUNTIME_CALIB_IMAGE 0
#endif

#if SIW917_ELRS_UPSTREAM_DUAL_RADIO && !SIW917_ELRS_RADIO2_IRQ_READY
#error "Enable SIW917_ELRS_UPSTREAM_DUAL_RADIO only after Radio2 DIO9 IRQ support is compiled in."
#endif

#ifndef SIW917_ELRS_RADIO2_NSS_PIN
#define SIW917_ELRS_RADIO2_NSS_PIN 50
#endif

#ifndef SIW917_ELRS_RADIO2_BUSY_PIN
#define SIW917_ELRS_RADIO2_BUSY_PIN 51
#endif

#ifndef SIW917_ELRS_RADIO2_DIO_PIN
#define SIW917_ELRS_RADIO2_DIO_PIN 47
#endif

#ifndef SIW917_ELRS_RADIO2_RST_PIN
#define SIW917_ELRS_RADIO2_RST_PIN 49
#endif

/* Boot/init timer logs are useful for bring-up, but not for timing runs. */
#ifndef SIW917_ELRS_TIMER_VERBOSE_INIT
#define SIW917_ELRS_TIMER_VERBOSE_INIT (!SIW917_ELRS_TIMING_LEAN)
#endif

/* Full LR1121/TCXO bring-up traces are useful only when debugging radio init. */
#ifndef SIW917_ELRS_RADIO_INIT_VERBOSE
#define SIW917_ELRS_RADIO_INIT_VERBOSE 0
#endif

/* Radio2 pin-probe dumps are boot-only validation noise after wiring is proven. */
#ifndef SIW917_ELRS_RADIO2_GPIO_PROBE_DIAG
#define SIW917_ELRS_RADIO2_GPIO_PROBE_DIAG SIW917_ELRS_RADIO_INIT_VERBOSE
#endif

/* DIO init/vector details are useful during IRQ bring-up, not normal RF tests. */
#ifndef SIW917_ELRS_DIO_INIT_VERBOSE
#define SIW917_ELRS_DIO_INIT_VERBOSE SIW917_ELRS_RADIO_INIT_VERBOSE
#endif

/*
 * Timing-test profile: keep ELRS behavior upstream-shaped, but remove serial
 * and timestamp diagnostics that add avoidable jitter while chasing 150 Hz.
 * Flip this to 0 when you need the detailed HOTPATH/TLMFAST/RATECHG traces.
 */
#ifndef SIW917_ELRS_TIMING_TEST_BUILD
#define SIW917_ELRS_TIMING_TEST_BUILD SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Put ELRS hot functions marked ICACHE_RAM_ATTR/IRAM_ATTR into RAM. This gives
 * the SiW917 port the same "ISR/hot-path in internal RAM" intent that ESP32
 * targets get from IRAM_ATTR, without changing the ELRS call ordering.
 */
#ifndef SIW917_ELRS_RAM_HOTPATH_CODE
#define SIW917_ELRS_RAM_HOTPATH_CODE SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_RAMFUNC_ATTR
#if SIW917_ELRS_RAM_HOTPATH_CODE
#define SIW917_ELRS_RAMFUNC_ATTR __attribute__((section(".ramfunc")))
#else
#define SIW917_ELRS_RAMFUNC_ATTR
#endif
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
 * Diagnostic only: reserve telemetry slots but do not key the LR1121 TX path.
 * If Lua requests stop dropping the link with this enabled, the failure is in
 * the telemetry TX/TX_DONE/return-to-RX turnaround rather than Lua parsing.
 */
#ifndef SIW917_ELRS_DISABLE_DOWNLINK_TLM
#define SIW917_ELRS_DISABLE_DOWNLINK_TLM 0
#endif

/*
 * Flight-controller CRSF serial path. Upstream RX targets expose RC/link-stats
 * output and accept FC telemetry back into the OTA downlink. Set this to 1 only
 * for RF-only timing tests where USART0 should stay completely quiet.
 */
#ifndef SIW917_ELRS_DISABLE_CRSF_SERIAL
#define SIW917_ELRS_DISABLE_CRSF_SERIAL 0
#endif

/*
 * Parse CRSF frames received from the FC UART and enqueue them into the same
 * OTA telemetry path as RX Lua/device-management responses.
 */
#ifndef SIW917_ELRS_ENABLE_CRSF_FC_TELEMETRY
#define SIW917_ELRS_ENABLE_CRSF_FC_TELEMETRY (!SIW917_ELRS_DISABLE_CRSF_SERIAL)
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
#define SIW917_ELRS_HOTPATH_TIMING_DIAG (!SIW917_ELRS_TIMING_TEST_BUILD)
#endif

/*
 * High-rate safety margin trims. These counters/timestamps were useful while
 * bringing up DIO and CRC timing, but K1000-class modes pay for every volatile
 * write and timestamp read. Keep them compiled out in the lean timing build;
 * re-enable only when actively diagnosing that specific path.
 */
#ifndef SIW917_ELRS_DIO_EDGE_TIMESTAMPS
#define SIW917_ELRS_DIO_EDGE_TIMESTAMPS                                       \
  (SIW917_ELRS_HOTPATH_TIMING_DIAG || SIW917_ELRS_DIO_PFD_TIMESTAMP)
#endif

#ifndef SIW917_ELRS_DIO_STATS_DIAG
#define SIW917_ELRS_DIO_STATS_DIAG (!SIW917_ELRS_TIMING_TEST_BUILD)
#endif

#ifndef SIW917_ELRS_ISR_STATS_DIAG
#define SIW917_ELRS_ISR_STATS_DIAG (!SIW917_ELRS_TIMING_TEST_BUILD)
#endif

#ifndef SIW917_ELRS_PACKET_STATS_DIAG
#define SIW917_ELRS_PACKET_STATS_DIAG (!SIW917_ELRS_TIMING_TEST_BUILD)
#endif

#ifndef SIW917_ELRS_RAW_GSPI_STATS_DIAG
#define SIW917_ELRS_RAW_GSPI_STATS_DIAG (!SIW917_ELRS_TIMING_TEST_BUILD)
#endif

#ifndef SIW917_ELRS_DIRECT_DIO_HAL_IO
#define SIW917_ELRS_DIRECT_DIO_HAL_IO SIW917_ELRS_TIMING_LEAN
#endif

/*
 * More GSPI hot-path trimming. These keep the LR1121 command order unchanged
 * but remove small per-command costs that matter at 250 Hz/K1000: repeated
 * pin-register address calculation, an unconditional micros() read when BUSY
 * is already low, pre-filling response buffers, and staging hot command params
 * through a temporary tx array.
 */
#ifndef SIW917_ELRS_FAST_GSPI_CACHE_PIN_REGS
#define SIW917_ELRS_FAST_GSPI_CACHE_PIN_REGS SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_FAST_BUSY_READ_FIRST
#define SIW917_ELRS_FAST_BUSY_READ_FIRST SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_FAST_GSPI_SKIP_RESPONSE_PREFILL
#define SIW917_ELRS_FAST_GSPI_SKIP_RESPONSE_PREFILL SIW917_ELRS_TIMING_LEAN
#endif

#ifndef SIW917_ELRS_FAST_HOT_COMMAND_STREAM_PARAMS
#define SIW917_ELRS_FAST_HOT_COMMAND_STREAM_PARAMS SIW917_ELRS_TIMING_LEAN
#endif

/*
 * Low-rate Lua/downlink progress trace. This runs from the ELRS task only, not
 * from RF IRQ context, so it should not disturb the timer/DIO hot path while we
 * diagnose long Lua parameter downloads.
 */
#ifndef SIW917_ELRS_LUA_PROGRESS_DIAG
#define SIW917_ELRS_LUA_PROGRESS_DIAG 0
#endif

#ifndef SIW917_ELRS_LUA_PROGRESS_INTERVAL_MS
#define SIW917_ELRS_LUA_PROGRESS_INTERVAL_MS 1000U
#endif

/*
 * Low-rate connected-state heartbeat for "telemetry died but RC stayed linked"
 * tests. This is task-context only and should stay slow enough to avoid
 * changing RF timing while still showing whether downlink telemetry slots stop.
 */
#ifndef SIW917_ELRS_LINK_PROGRESS_DIAG
#define SIW917_ELRS_LINK_PROGRESS_DIAG 0
#endif

#ifndef SIW917_ELRS_LINK_PROGRESS_INTERVAL_MS
#define SIW917_ELRS_LINK_PROGRESS_INTERVAL_MS 5000U
#endif

/*
 * Disconnected scan diagnostics. Leave off for normal testing; scan mode can
 * print rapidly and the serial load can obscure the RF timing picture.
 */
#ifndef SIW917_ELRS_DISCONNECTED_SCAN_DIAG
#define SIW917_ELRS_DISCONNECTED_SCAN_DIAG 0
#endif

/*
 * Verbose RF-rate/radio reconfiguration trace. Leave this off for normal use:
 * scan mode can reconfigure rapidly, and these prints add serial load exactly
 * while the receiver is trying to catch SYNC.
 */
#ifndef SIW917_ELRS_RF_RATE_DIAG
#define SIW917_ELRS_RF_RATE_DIAG SIW917_ELRS_DISCONNECTED_SCAN_DIAG
#endif

/*
 * The upstream scan interval can be very short for K1000-class modes
 * (FCC915 index 0 computes to about 88 ms). On SiW917, rate reconfigure and
 * task/IRQ handoff jitter can make that too narrow to reliably catch a sync
 * packet, so keep high-rate scan dwell long enough to span multiple sync
 * opportunities without slowing lower-rate scans.
 */
#ifndef SIW917_ELRS_SCAN_MIN_DWELL_MS
#define SIW917_ELRS_SCAN_MIN_DWELL_MS 250U
#endif

/*
 * Keep PFD extEvent timestamping aligned with upstream RX: ProcessRFPacket()
 * samples beginProcessing after the radio IRQ path has read/handled the
 * packet, then adds PACKET_TO_TOCK_SLACK. The raw GPIO/DIO edge is useful for
 * latency diagnostics, but feeding it to PFD biases the loop early by the
 * LR1121 IRQ/SPI handling time and drives the timer off-frequency.
 */
#ifndef SIW917_ELRS_DIO_PFD_TIMESTAMP
#define SIW917_ELRS_DIO_PFD_TIMESTAMP 0
#endif

/*
 * Keep the timing build quiet when Lua parameter downloads request temporary
 * telemetry boost. These prints happen at the exact moment the link switches to
 * dense telemetry and can steal enough time to disturb 100/150 Hz operation.
 */
#ifndef SIW917_ELRS_TLM_RATE_DIAG
#define SIW917_ELRS_TLM_RATE_DIAG 0
#endif

/*
 * Dynamic-power SNR/RSSI diagnostics are useful when validating MatchTX /
 * dynamic power, but they run on the telemetry path. Keep them opt-in so the
 * normal RF acquisition and locked-link paths stay as quiet as upstream.
 */
#ifndef SIW917_ELRS_DYNPOWER_STATS_DIAG
#define SIW917_ELRS_DYNPOWER_STATS_DIAG 0
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
 * Connected radio hot-path profile. At 150 Hz the half-slot is only 3333 us.
 * A 2000 us BUSY wait is too large for the SiW917 timing island, but 250 us can
 * send commands while LR1121 BUSY is still asserted during dense telemetry. Cap
 * the bare-metal wait at a slot-budget-safe value that still respects BUSY.
 */
#ifndef SIW917_ELRS_BUSY_FAST_US
#define SIW917_ELRS_BUSY_FAST_US 500U
#endif

/*
 * SetRx is the one hot command we issue immediately after telemetry TX_DONE.
 * If LR1121 is still BUSY there, clocking SetRx too early can leave the radio
 * out of receive for several uplink slots. Give only SetRx a larger upstream-
 * style wait budget while keeping frequency/TX commands on the tighter budget.
 */
#ifndef SIW917_ELRS_SET_RX_BUSY_FAST_US
#define SIW917_ELRS_SET_RX_BUSY_FAST_US 1500U
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

/*
 * Strict timing profile: once linked, hot LR1121 commands must use the bounded
 * fast register path only. Falling back into generic SDK/raw helpers after a
 * fast-path miss can rescue a slow slot, but it makes 150 Hz noisy and cannot
 * scale to 1000 Hz FSK.
 */
#ifndef SIW917_ELRS_STRICT_BARE_METAL_HOTPATH
#define SIW917_ELRS_STRICT_BARE_METAL_HOTPATH SIW917_ELRS_TIMING_LEAN
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
 * A/B test for the 100/150 Hz RX_DONE collapse: force FHSS hops to use the
 * LR1121 helper that retunes and re-enters continuous RX in one command.
 */
#ifndef SIW917_ELRS_FHSS_SET_FREQ_RX
#define SIW917_ELRS_FHSS_SET_FREQ_RX 0
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
 * Keep TX_DONE IRQ ordering upstream-shaped by default: clear/read LR1121 IRQ
 * status before TXdoneCallback() re-arms RX. DIO is level-held until cleared,
 * so re-arming RX while TX_DONE is still asserted can starve later DIO edges.
 */
#ifndef SIW917_ELRS_RX_FIRST_TXDONE
#define SIW917_ELRS_RX_FIRST_TXDONE 0
#endif
