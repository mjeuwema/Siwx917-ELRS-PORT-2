#ifndef BLE_GATT_PROBE_H
#define BLE_GATT_PROBE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t ble_gatt_config_api_prepare_nwp(void);
int ble_gatt_config_api_start(void);
bool ble_gatt_config_api_is_running(void);
void ble_gatt_probe_start_task(void);
int ble_remote_id_service_prepare(void);
int ble_remote_id_service_start(bool enabled);
void ble_remote_id_service_set_enabled(bool enabled);
bool ble_remote_id_service_is_enabled(void);
bool ble_remote_id_service_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_GATT_PROBE_H */
