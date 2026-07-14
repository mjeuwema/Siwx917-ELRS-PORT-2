/**
 * @file elrs_tx_main.h
 * @brief Main ELRS TX application interface for SiW917
 *
 * This is the parallel TX entry surface that will be wired into the
 * shared SiW917/LR1121 ELRS stack as the TX port comes online.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool elrs_tx_init(void);
void elrs_tx_start(void);
void elrs_tx_loop(void);
void elrs_tx_stop(void);

#ifdef __cplusplus
}
#endif
