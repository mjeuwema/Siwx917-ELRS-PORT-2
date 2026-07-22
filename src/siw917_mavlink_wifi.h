#ifndef SIW917_MAVLINK_WIFI_H
#define SIW917_MAVLINK_WIFI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Internal TX backpack transport. The ELRS task only touches the bounded
 * queues exposed here; all WiFi and socket calls run in a low-priority task.
 */
bool siw917_mavlink_wifi_init(void);
void siw917_mavlink_wifi_set_enabled(bool enabled);
bool siw917_mavlink_wifi_is_enabled(void);
bool siw917_mavlink_wifi_is_running(void);

bool siw917_mavlink_wifi_enqueue_downlink(const uint8_t *data, size_t length);
size_t siw917_mavlink_wifi_uplink_available(void);
int siw917_mavlink_wifi_uplink_read(void);
size_t siw917_mavlink_wifi_uplink_read_bytes(uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
