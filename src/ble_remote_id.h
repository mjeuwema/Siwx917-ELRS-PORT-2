#ifndef BLE_REMOTE_ID_H
#define BLE_REMOTE_ID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_REMOTE_ID_UAS_ID_MAX 20U
#define BLE_REMOTE_ID_MESSAGE_SIZE 25U
#define BLE_REMOTE_ID_ADV_MAX 31U

typedef enum {
  BLE_REMOTE_ID_ADV_BASIC_ID = 0,
  BLE_REMOTE_ID_ADV_LOCATION = 1,
} ble_remote_id_adv_kind_t;

void ble_remote_id_init_from_config(void);
bool ble_remote_id_set_uas_id(const char *uas_id);
const char *ble_remote_id_get_uas_id(void);
bool ble_remote_id_set_location_e7(int32_t latitude_e7, int32_t longitude_e7, int16_t altitude_m);
void ble_remote_id_clear_location(void);
bool ble_remote_id_location_is_set(void);
bool ble_remote_id_location_is_fresh(uint32_t now_ms, uint32_t max_age_ms);
uint32_t ble_remote_id_location_sequence(void);
bool ble_remote_id_update_from_crsf_gps(int32_t latitude_e7,
                                        int32_t longitude_e7,
                                        int16_t altitude_m,
                                        uint16_t ground_speed_cms,
                                        uint16_t direction_cdeg,
                                        uint32_t now_ms);
uint16_t ble_remote_id_build_basic_id_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE]);
uint16_t ble_remote_id_build_location_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE]);
uint16_t ble_remote_id_build_basic_id_advertisement(uint8_t out[BLE_REMOTE_ID_ADV_MAX]);
uint16_t ble_remote_id_build_location_advertisement(uint8_t out[BLE_REMOTE_ID_ADV_MAX]);
uint16_t ble_remote_id_build_advertisement(uint8_t out[BLE_REMOTE_ID_ADV_MAX],
                                           ble_remote_id_adv_kind_t kind);
void ble_remote_id_format_status(char *out, size_t out_len, bool advertising);
void ble_remote_id_format_location(char *out, size_t out_len);
void ble_remote_id_format_adv_hex(char *out, size_t out_len, ble_remote_id_adv_kind_t kind);

#ifdef __cplusplus
}
#endif

#endif /* BLE_REMOTE_ID_H */
