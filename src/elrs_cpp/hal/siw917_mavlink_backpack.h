#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void siw917_mavlink_backpack_init(void);
void siw917_mavlink_backpack_service(void);
void siw917_mavlink_backpack_set_lua_enabled(bool enabled);
bool siw917_mavlink_backpack_get_lua_enabled(void);
bool siw917_mavlink_backpack_prepare_for_update(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
