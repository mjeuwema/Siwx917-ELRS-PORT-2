#include "elrs_task_wakeup.h"

#include "cmsis_os2.h"

static volatile osThreadId_t elrsTaskThread = nullptr;
static volatile uint32_t dioWakeCount = 0;
static volatile uint32_t timerWakeCount = 0;

extern "C" void elrs_task_wakeup_set_thread(void *thread_id) {
  elrsTaskThread = (osThreadId_t)thread_id;
}

extern "C" void elrs_task_wakeup_from_isr(uint32_t flags) {
  osThreadId_t thread = elrsTaskThread;
  if (thread == nullptr) {
    return;
  }

  if ((flags & ELRS_TASK_WAKE_DIO1) != 0) {
    dioWakeCount++;
  }
  if ((flags & ELRS_TASK_WAKE_TIMER) != 0) {
    timerWakeCount++;
  }

  (void)osThreadFlagsSet(thread, flags & ELRS_TASK_WAKE_ALL);
}

extern "C" void elrs_task_wakeup_signal(uint32_t flags) {
  elrs_task_wakeup_from_isr(flags);
}

extern "C" uint32_t elrs_task_wakeup_wait(uint32_t timeout_ticks) {
  uint32_t flags =
      osThreadFlagsWait(ELRS_TASK_WAKE_ALL, osFlagsWaitAny, timeout_ticks);
  if ((flags & osFlagsError) != 0) {
    return 0;
  }
  return flags & ELRS_TASK_WAKE_ALL;
}

extern "C" uint32_t elrs_task_wakeup_get_dio_count(void) {
  return dioWakeCount;
}

extern "C" uint32_t elrs_task_wakeup_get_timer_count(void) {
  return timerWakeCount;
}
