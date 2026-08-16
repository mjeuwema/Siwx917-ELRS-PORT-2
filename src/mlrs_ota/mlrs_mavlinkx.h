#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MLRS_AIR_MAVLINKX
#define MLRS_AIR_MAVLINKX 0x22
#endif

void mlrs_mavlinkx_init(void);
void mlrs_mavlinkx_reset(void);
void mlrs_mavlinkx_air_lost(void);
void mlrs_mavlinkx_set_compression(bool enabled);

void mlrs_mavlinkx_ingest_mav(const uint8_t *data, uint16_t len);
uint8_t mlrs_mavlinkx_take_air(uint8_t *dst, uint8_t max);

void mlrs_mavlinkx_ingest_air(const uint8_t *data, uint16_t len);
uint8_t mlrs_mavlinkx_take_mav(uint8_t *dst, uint8_t max);

/* Local RADIO_STATUS (#109). Not sent over the air. */
uint8_t mlrs_pack_radio_status(uint8_t *dst, uint8_t max, uint8_t rssi,
                               uint8_t remrssi, uint8_t txbuf, uint8_t noise);
uint16_t mlrs_mavlinkx_air_pending(void);
uint32_t mlrs_mavlinkx_take_link_out_bytes(void);

#ifdef __cplusplus
}
#endif
