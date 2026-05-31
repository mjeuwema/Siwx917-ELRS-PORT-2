/**
 * @file hwTimer.cpp
 * @brief ELRS hwTimer C++ class implementation for SiW917
 *
 * This file provides the ELRS hwTimer C++ class interface by wrapping
 * the existing C implementation in hw_timer.c which uses the SiW917
 * Configurable Timer (CT) peripheral.
 *
 * The CT provides:
 * - SOC_PLL clock (180MHz, crystal accuracy) - ~5.5ns resolution
 * - 16-bit Counter 0 mode for ELRS half-intervals
 * - Buffer register for glitch-free interval updates
 *
 * Citation: ExpressLRS 4.0 lib/HWTIMER/hwTimer.h - Class interface
 * Citation: Our hw_timer.c - CT-based implementation
 */

#include "hwTimer.h"
#include "elrs_task_wakeup.h"
#include "logging.h"
#include "siw917_elrs_timing.h"

// Include our C timer implementation
extern "C" {
#include "hw_timer.h"
#if SIW917_ELRS_DWT_MICROS
bool micros_uses_dwt(void);
#endif
}

// Static member definitions
hwTimerCallback_t hwTimer::callbackTick = nullptr;
hwTimerCallback_t hwTimer::callbackTock = nullptr;

volatile bool hwTimer::running = false;
volatile bool hwTimer::isTick = false;

volatile uint32_t hwTimer::HWtimerInterval = 20000; // Default 20ms (50Hz)
volatile int32_t hwTimer::PhaseShift = 0;
volatile int32_t hwTimer::FreqOffset = 0;

static volatile bool immediateTockPending = false;
static volatile uint32_t immediateTockMicros = 0;
static volatile uint32_t activeEventMicros = 0;

enum TimerEventType : uint8_t {
  TIMER_EVENT_TICK = 1,
  TIMER_EVENT_TOCK = 2,
};

struct TimerEvent {
  uint8_t type;
  uint32_t timestampUs;
};

static constexpr uint16_t TIMER_EVENT_QUEUE_SIZE = 128;
static TimerEvent timerEventQueue[TIMER_EVENT_QUEUE_SIZE] = {};
static volatile uint16_t timerEventHead = 0;
static volatile uint16_t timerEventTail = 0;
static volatile uint32_t timerEventOverflowCount = 0;
static volatile uint32_t queuedTickCount = 0;
static volatile uint32_t queuedTockCount = 0;
static volatile uint32_t processedTickCount = 0;
static volatile uint32_t processedTockCount = 0;
static volatile uint32_t immediateTockDeliveredCount = 0;
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
static volatile uint32_t maxTickDurationUs = 0;
static volatile uint32_t maxTockDurationUs = 0;

static inline void updateMaxDuration(volatile uint32_t &maxValue,
                                     uint32_t durationUs) {
  if (durationUs > maxValue) {
    maxValue = durationUs;
  }
}
#endif

static void processTimerEvent(uint8_t type, uint32_t timestampUs) {
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  const uint32_t startUs = hw_timer_get_micros();
#endif
  activeEventMicros = timestampUs;

  if (type == TIMER_EVENT_TICK) {
    hwTimer::isTick = true;
    processedTickCount++;
    if (hwTimer::callbackTick) {
      hwTimer::callbackTick();
    }
    hwTimer::isTick = false;
  } else if (type == TIMER_EVENT_TOCK) {
    hwTimer::isTick = false;
    processedTockCount++;
    if (hwTimer::callbackTock) {
      hwTimer::callbackTock();
    }
    hwTimer::isTick = true;
  }

  activeEventMicros = 0;
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  const uint32_t durationUs = hw_timer_get_micros() - startUs;
  if (type == TIMER_EVENT_TICK) {
    updateMaxDuration(maxTickDurationUs, durationUs);
  } else if (type == TIMER_EVENT_TOCK) {
    updateMaxDuration(maxTockDurationUs, durationUs);
  }
#endif
}

static void resetTimerEventQueue() {
  immediateTockPending = false;
  immediateTockMicros = 0;
  timerEventHead = 0;
  timerEventTail = 0;
}

static void resetTimerEventStats() {
  timerEventOverflowCount = 0;
  queuedTickCount = 0;
  queuedTockCount = 0;
  processedTickCount = 0;
  processedTockCount = 0;
  immediateTockDeliveredCount = 0;
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  maxTickDurationUs = 0;
  maxTockDurationUs = 0;
#endif
}

static void __attribute__((unused))
enqueueTimerEvent(uint8_t type, uint32_t timestampUs) {
  const uint16_t head = timerEventHead;
  const uint16_t nextHead =
      (uint16_t)((head + 1U) % TIMER_EVENT_QUEUE_SIZE);

  if (nextHead == timerEventTail) {
    // Single-producer/single-consumer queue: the ISR owns head only, while the
    // task owns tail. Dropping the newest event is safer than moving tail here.
    timerEventOverflowCount++;
    return;
  }

  timerEventQueue[head].type = type;
  timerEventQueue[head].timestampUs = timestampUs;
  timerEventHead = nextHead;

  if (type == TIMER_EVENT_TICK) {
    queuedTickCount++;
  } else if (type == TIMER_EVENT_TOCK) {
    queuedTockCount++;
  }

  elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_TIMER);
}

static bool dequeueTimerEvent(TimerEvent *event) {
  if (timerEventTail == timerEventHead) {
    return false;
  }

  *event = timerEventQueue[timerEventTail];
  timerEventTail =
      (uint16_t)((timerEventTail + 1U) % TIMER_EVENT_QUEUE_SIZE);
  return true;
}

bool hwTimer::hasPendingEvent() {
  return running && (immediateTockPending || timerEventTail != timerEventHead);
}

extern "C" bool elrs_hw_timer_has_pending_event(void) {
  return hwTimer::hasPendingEvent();
}

//-----------------------------------------------------------------------------
// Internal C callback bridge
//-----------------------------------------------------------------------------

// These functions are called by hw_timer.c ISR. In timing mode, run the ELRS
// Tick/Tock callbacks directly from CT so RF hop/telemetry scheduling is not
// delayed by the FreeRTOS task wakeup path. The queued fallback is kept for
// bring-up and for quickly backing out if a platform command is not ISR-safe.
static inline uint32_t timerEventTimestampUs() {
  /*
   * Match upstream ESP32 RX behavior: HWtimerCallbackTock() calls
   * PFDloop.intEvent(micros()) from inside the timer ISR.  Using the scheduled
   * CT edge timestamp here hides any real CT-vs-micros drift from the PFD loop,
   * which can leave FHSS retunes phase-shifted even though the offset log looks
   * small.
   */
  return micros();
}

static void hwTimerTickBridge(void) {
  if (hwTimer::running) {
#if SIW917_ELRS_DIRECT_TIMER_CALLBACKS || SIW917_ELRS_DIRECT_TIMER_TICK
    processTimerEvent(TIMER_EVENT_TICK, timerEventTimestampUs());
#else
    enqueueTimerEvent(TIMER_EVENT_TICK, timerEventTimestampUs());
#endif
  }
}

static void hwTimerTockBridge(void) {
  if (hwTimer::running) {
#if SIW917_ELRS_DIRECT_TIMER_CALLBACKS
    processTimerEvent(TIMER_EVENT_TOCK, timerEventTimestampUs());
#else
    enqueueTimerEvent(TIMER_EVENT_TOCK, timerEventTimestampUs());
#endif
  }
}

//-----------------------------------------------------------------------------
// hwTimer Class Implementation
//-----------------------------------------------------------------------------

void hwTimer::init(hwTimerCallback_t cbTick, hwTimerCallback_t cbTock) {
#if SIW917_ELRS_TIMER_VERBOSE_INIT
  printf(">>> hwTimer::init ENTRY (interval=%lu) <<<\n",
         (unsigned long)HWtimerInterval);
#endif

  callbackTick = cbTick;
  callbackTock = cbTock;
  running = false;
  isTick = false;
  PhaseShift = 0;
  FreqOffset = 0;
  activeEventMicros = 0;
  resetTimerEventQueue();
  resetTimerEventStats();

  // Bypass removed - hwTimer now enabled

  // Initialize the underlying C timer with current interval
#if SIW917_ELRS_TIMER_VERBOSE_INIT
  printf("hwTimer::init - calling hw_timer_init...\n");
#endif
  sl_status_t status = hw_timer_init(HWtimerInterval);
#if SIW917_ELRS_TIMER_VERBOSE_INIT
  printf("hwTimer::init - hw_timer_init returned 0x%04lX\n",
         (unsigned long)status);
#endif

  if (status != SL_STATUS_OK) {
    printf("hwTimer::init FAILED!\n");
    DBGLN("hwTimer init failed: 0x%04X", (unsigned)status);
    return;
  }

  // Register our bridge callbacks with the C implementation
#if SIW917_ELRS_TIMER_VERBOSE_INIT
  printf("hwTimer::init - setting callbacks...\n");
#endif
  hw_timer_set_tick_callback(hwTimerTickBridge);
  hw_timer_set_tock_callback(hwTimerTockBridge);
#if SIW917_ELRS_DIRECT_TIMER_CALLBACKS
  const char *timerCallbackPath = "direct-isr";
#elif SIW917_ELRS_DIRECT_TIMER_TICK
  const char *timerCallbackPath = "tick-direct/tock-queued";
#else
  const char *timerCallbackPath = "queued-task";
#endif
#if SIW917_ELRS_TIMER_VERBOSE_INIT
  printf("hwTimer::init - timer callback path: %s\n", timerCallbackPath);
#else
  (void)timerCallbackPath;
#endif

#if SIW917_ELRS_DWT_MICROS
  (void)micros();
#if SIW917_ELRS_TIMER_VERBOSE_INIT
  printf("hwTimer::init - micros source: %s\n",
         micros_uses_dwt() ? "DWT CYCCNT" : "SysTick fallback");
#endif
#endif

#if SIW917_ELRS_TIMER_VERBOSE_INIT
  printf("hwTimer::init COMPLETE OK\n");
  DBGLN("hwTimer initialized (CT-based)");
#endif
}

void hwTimer::stop() {
  if (!running)
    return;

  running = false;
  resetTimerEventQueue();
  hw_timer_stop();
  DBGLN("hwTimer stopped");
}

void hwTimer::resume() {
  if (running)
    return;

  // Match upstream RX behavior: enabling the timer throws an immediate TOCK.
  // At K1000 a deferred nonce/FHSS advance can miss the first post-SYNC packet.
  isTick = false;
  PhaseShift = 0;
  activeEventMicros = 0;
  resetTimerEventQueue();
  resetTimerEventStats();

  const uint32_t resumeMicros = micros();
  hw_timer_set_event_epoch(resumeMicros);

  sl_status_t status = hw_timer_start();
  if (status != SL_STATUS_OK) {
    running = false;
    DBGLN("hwTimer resume failed: 0x%04X", (unsigned)status);
    return;
  }

  running = true;
  isTick = false;
  hw_timer_note_immediate_tock();
#if SIW917_ELRS_DIRECT_TIMER_CALLBACKS
  immediateTockDeliveredCount++;
  processTimerEvent(TIMER_EVENT_TOCK, resumeMicros);
#else
  immediateTockMicros = resumeMicros;
  immediateTockPending = true;
#endif

  DBGLN("hwTimer resumed, interval=%lu us", HWtimerInterval);
}

void hwTimer::service() {
  if (!running) {
    return;
  }

  if (immediateTockPending) {
    immediateTockPending = false;
    const uint32_t eventMicros =
        immediateTockMicros != 0 ? immediateTockMicros : micros();
    immediateTockMicros = 0;
    immediateTockDeliveredCount++;
    processTimerEvent(TIMER_EVENT_TOCK, eventMicros);
  }

  TimerEvent event;
  while (running && dequeueTimerEvent(&event)) {
    processTimerEvent(event.type, event.timestampUs);
  }
}

uint32_t hwTimer::eventMicros() {
  const uint32_t eventTime = activeEventMicros;
  return eventTime != 0 ? eventTime : micros();
}

void hwTimer::updateInterval(uint32_t newTimerInterval) {
  HWtimerInterval = newTimerInterval;

  // Update the C timer's interval
  hw_timer_set_interval(newTimerInterval);

#if SIW917_ELRS_RF_RATE_DIAG
  DBGLN("hwTimer interval: %lu us", (unsigned long)newTimerInterval);
#endif
}

void hwTimer::resetFreqOffset() {
  FreqOffset = 0;
  hw_timer_reset_freq_offset();
}

void hwTimer::phaseShift(int32_t newPhaseShift) {
  // Clamp to reasonable range (+/- 1/4 of interval)
  int32_t maxShift = (int32_t)(HWtimerInterval >> 2);
  if (newPhaseShift > maxShift)
    newPhaseShift = maxShift;
  if (newPhaseShift < -maxShift)
    newPhaseShift = -maxShift;

  PhaseShift = newPhaseShift;

  // Apply to underlying C timer
  hw_timer_phase_shift(newPhaseShift);
}

void hwTimer::incFreqOffset() {
  hw_timer_inc_freq_offset(1);
  FreqOffset = hw_timer_get_freq_offset();
}

void hwTimer::decFreqOffset() {
  hw_timer_inc_freq_offset(-1);
  FreqOffset = hw_timer_get_freq_offset();
}

// Note: handleISR() is not needed - the C timer calls our bridge functions
// directly
void hwTimer::handleISR() {
  // ISR handling is done by hw_timer.c callback mechanism
  // This function exists for API compatibility but is unused
}

uint32_t hwTimer::getHardwareHalfTicks() {
  return hw_timer_get_total_half_ticks();
}

uint32_t hwTimer::getHardwareCount() { return hw_timer_get_current_count(); }

uint32_t hwTimer::getHardwareMatch() { return hw_timer_get_match_value(); }

uint32_t hwTimer::getHardwareFreqHz() { return hw_timer_get_ct_freq_hz(); }

uint32_t hwTimer::getQueuedTickCount() { return queuedTickCount; }

uint32_t hwTimer::getQueuedTockCount() { return queuedTockCount; }

uint32_t hwTimer::getProcessedTickCount() { return processedTickCount; }

uint32_t hwTimer::getProcessedTockCount() { return processedTockCount; }

uint32_t hwTimer::getQueueOverflowCount() {
  return timerEventOverflowCount;
}

uint32_t hwTimer::getImmediateTockDeliveredCount() {
  return immediateTockDeliveredCount;
}

uint32_t hwTimer::getMaxTickDurationUs() {
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  return maxTickDurationUs;
#else
  return 0;
#endif
}

uint32_t hwTimer::getMaxTockDurationUs() {
#if SIW917_ELRS_HOTPATH_TIMING_DIAG
  return maxTockDurationUs;
#else
  return 0;
#endif
}
