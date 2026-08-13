#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void mlrs_mbridge_init(void);
void mlrs_mbridge_poll(void);
bool mlrs_mbridge_take_air(uint8_t *dst, uint8_t *len, uint8_t max);
void mlrs_mbridge_accept_downlink(const uint8_t *src, uint8_t len);

#ifdef __cplusplus
}
#endif
