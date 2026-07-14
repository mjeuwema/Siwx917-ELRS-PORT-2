#pragma once

#include <stdint.h>
#include <string.h>

#define STATION_IF 0

static inline bool wifi_get_macaddr(uint8_t if_index, uint8_t *macaddr) {
    (void)if_index;
    static const uint8_t siw917_default_uid[6] = {0xBE, 0x93, 0x67, 0x27, 0xD6, 0x9C};
    memcpy(macaddr, siw917_default_uid, sizeof(siw917_default_uid));
    return true;
}
