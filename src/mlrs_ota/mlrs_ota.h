#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELRS_AIR_PROTOCOL_ELRS 0
#define ELRS_AIR_PROTOCOL_MLRS 1
#ifndef MSP_ELRS_SET_AIR_PROTOCOL
#define MSP_ELRS_SET_AIR_PROTOCOL 0x2E
#endif
#define ELRS_RESERVED_AIR_PROTOCOL_OFFSET 26
#define ELRS_RESERVED_MLRS_RATE_OFFSET 25
#define ELRS_RESERVED_MLRS_BAND_OFFSET 24
#define ELRS_RESERVED_MLRS_GCM_SEND_OFFSET 16
#define ELRS_RESERVED_MLRS_GCM_RECV_OFFSET 20

#define MLRS_BAND_915 0
#define MLRS_BAND_24 1
#define MLRS_BAND_COUNT 2

#define MLRS_RATE_31HZ 0
#define MLRS_RATE_19HZ 1
#define MLRS_RATE_FSK50 2
#define MLRS_RATE_COUNT 3
#define MLRS_OTA_RATE_OPTIONS_915 "31Hz;19Hz;FSK50"
#define MLRS_OTA_RATE_OPTIONS_24 "50Hz;31Hz;19Hz"
#define MLRS_OTA_RATE_OPTIONS MLRS_OTA_RATE_OPTIONS_915
#define MLRS_AIR_MBRIDGE 0xA0
#define MLRS_AIR_RX_STATE 0x20
#define MLRS_AIR_HOPMASK 0x21
#define MLRS_AIR_MAVLINKX 0x22
#ifndef ELRS_SERIAL_PROTOCOL_LUA_SELECTION_MAVLINK
#define ELRS_SERIAL_PROTOCOL_LUA_SELECTION_MAVLINK 3
#endif

bool mlrs_ota_is_active(void);
bool mlrs_ota_is_connected(void);
void mlrs_ota_hop_skip_info(uint8_t *skip_count, uint8_t *hop_count,
                            uint32_t *mask);
void mlrs_ota_link_rate_info(uint8_t *rate, uint8_t *band, uint8_t *ul_plen,
                             uint8_t *dl_plen);
bool mlrs_ota_tlm_busy(void);
uint32_t mlrs_ota_tlm_busy_timeout_ms(void);
void mlrs_ota_rx_send_slot(void);
bool mlrs_ota_take_elrs_first_sync(void);
uint8_t mlrs_ota_get_protocol(void);
bool mlrs_ota_set_protocol(uint8_t protocol);
uint8_t mlrs_ota_get_rate(void);
bool mlrs_ota_set_rate(uint8_t rate);
uint8_t mlrs_ota_get_band(void);
bool mlrs_ota_set_band(uint8_t band);
const char *mlrs_ota_rate_name(void);
const char *mlrs_ota_rate_options(void);
void mlrs_ota_on_elrs_ready(void);
void mlrs_ota_loop(void);
bool mlrs_ota_handle_msp(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
void mlrs_elrs_rx_accept_uplink(const uint8_t *payload, uint8_t len);
void mlrs_elrs_rx_write_serial(const uint8_t *payload, uint8_t len);
uint8_t mlrs_elrs_rx_take_downlink(uint8_t *payload, uint8_t maxLen);
uint8_t mlrs_elrs_rx_take_serial(uint8_t *payload, uint8_t maxLen);
#endif
