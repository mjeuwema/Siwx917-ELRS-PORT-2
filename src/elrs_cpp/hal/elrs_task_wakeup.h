#pragma once

#include <stdint.h>

#define ELRS_TASK_WAKE_DIO1  (1UL << 0)
#define ELRS_TASK_WAKE_TIMER (1UL << 1)
#define ELRS_TASK_WAKE_ALL   (ELRS_TASK_WAKE_DIO1 | ELRS_TASK_WAKE_TIMER)

#ifdef __cplusplus
extern "C" {
#endif

void elrs_task_wakeup_set_thread(void *thread_id);
void elrs_task_wakeup_from_isr(uint32_t flags);
void elrs_task_wakeup_signal(uint32_t flags);
uint32_t elrs_task_wakeup_wait(uint32_t timeout_ticks);
uint32_t elrs_task_wakeup_get_dio_count(void);
uint32_t elrs_task_wakeup_get_timer_count(void);

#ifdef __cplusplus
}
#endif
