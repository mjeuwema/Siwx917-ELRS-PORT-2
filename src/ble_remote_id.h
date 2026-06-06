#ifndef BLE_REMOTE_ID_H
#define BLE_REMOTE_ID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_REMOTE_ID_UAS_ID_MAX 20U
#define BLE_REMOTE_ID_SELF_ID_MAX 23U
#define BLE_REMOTE_ID_OPERATOR_ID_MAX 20U
#define BLE_REMOTE_ID_MESSAGE_SIZE 25U
#define BLE_REMOTE_ID_ADV_MAX 31U
#define BLE_REMOTE_ID_MESSAGE_PACK_HEADER_SIZE 3U
#define BLE_REMOTE_ID_MESSAGE_PACK_MAX_MESSAGES 5U
#define BLE_REMOTE_ID_MESSAGE_PACK_MAX \
  (BLE_REMOTE_ID_MESSAGE_PACK_HEADER_SIZE + \
   (BLE_REMOTE_ID_MESSAGE_PACK_MAX_MESSAGES * BLE_REMOTE_ID_MESSAGE_SIZE))
#define BLE_REMOTE_ID_BT5_ADV_MAX (1U + 1U + 2U + 1U + 1U + BLE_REMOTE_ID_MESSAGE_PACK_MAX)

typedef enum {
  BLE_REMOTE_ID_ADV_BASIC_ID = 0,
  BLE_REMOTE_ID_ADV_LOCATION = 1,
} ble_remote_id_adv_kind_t;

void ble_remote_id_init_from_config(void);
bool ble_remote_id_set_uas_id(const char *uas_id);
const char *ble_remote_id_get_uas_id(void);
bool ble_remote_id_set_self_id_text(const char *text);
const char *ble_remote_id_get_self_id_text(void);
void ble_remote_id_reset_self_id_text(void);
bool ble_remote_id_set_operator_id_text(const char *text);
const char *ble_remote_id_get_operator_id_text(void);
void ble_remote_id_clear_operator_id_text(void);
bool ble_remote_id_operator_id_is_set(void);
bool ble_remote_id_set_location_e7(int32_t latitude_e7, int32_t longitude_e7, int16_t altitude_m);
void ble_remote_id_clear_location(void);
bool ble_remote_id_location_is_set(void);
bool ble_remote_id_location_is_fresh(uint32_t now_ms, uint32_t max_age_ms);
uint32_t ble_remote_id_location_sequence(void);
void ble_remote_id_clear_takeoff_location(void);
bool ble_remote_id_takeoff_is_set(void);
bool ble_remote_id_update_from_crsf_gps(int32_t latitude_e7,
                                        int32_t longitude_e7,
                                        int16_t altitude_m,
                                        uint16_t ground_speed_cms,
                                        uint16_t direction_cdeg,
                                        uint32_t now_ms);
uint16_t ble_remote_id_build_basic_id_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE]);
uint16_t ble_remote_id_build_location_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE]);
uint16_t ble_remote_id_build_self_id_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE]);
uint16_t ble_remote_id_build_system_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE]);
uint16_t ble_remote_id_build_operator_id_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE]);
uint16_t ble_remote_id_build_basic_id_advertisement(uint8_t out[BLE_REMOTE_ID_ADV_MAX]);
uint16_t ble_remote_id_build_location_advertisement(uint8_t out[BLE_REMOTE_ID_ADV_MAX]);
uint16_t ble_remote_id_build_advertisement(uint8_t out[BLE_REMOTE_ID_ADV_MAX],
                                           ble_remote_id_adv_kind_t kind);
uint16_t ble_remote_id_build_message_pack(uint8_t *out, uint16_t max_len, bool include_location);
uint16_t ble_remote_id_build_bt5_advertisement(uint8_t *out,
                                               uint16_t max_len,
                                               bool include_location);
void ble_remote_id_format_status(char *out, size_t out_len, bool advertising);
void ble_remote_id_format_location(char *out, size_t out_len);
void ble_remote_id_format_takeoff(char *out, size_t out_len);
void ble_remote_id_format_adv_hex(char *out, size_t out_len, ble_remote_id_adv_kind_t kind);
void ble_remote_id_format_bt5_adv_hex(char *out, size_t out_len, bool include_location);

#ifdef __cplusplus
}
#endif

#endif /* BLE_REMOTE_ID_H */
