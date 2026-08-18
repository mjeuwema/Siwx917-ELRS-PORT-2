#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Stock mLRS TX MAVLink component: sysid 51 / compid 68, local to GCS.
 * Not sent over the air. Lua "Tx Mav Component" enables this. MAVLink
 * WiFi On is still the UDP pipe. */

void mlrs_tx_mavcomp_set_enabled(bool enabled);
bool mlrs_tx_mavcomp_get_enabled(void);

/* Parse GCS uplink. Frames for 51/68 stay here; others go to the air. */
void mlrs_tx_mavcomp_ingest_uplink(const uint8_t *data, uint16_t len);

/* 1 Hz HEARTBEAT and paced PARAM_VALUE. Call from ota_loop, not RXdone. */
void mlrs_tx_mavcomp_service(void);

#ifdef __cplusplus
}
#endif
