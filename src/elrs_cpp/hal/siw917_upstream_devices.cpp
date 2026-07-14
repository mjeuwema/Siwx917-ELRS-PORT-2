#include "device.h"

// Upstream TX expects a few optional UI devices to exist even when a target
// does not implement their ESP/WS2812-specific backends. Keep those as HAL
// stubs so the RF scheduler and CRSF/Lua path stay upstream-owned.

static bool initializeDisabledDevice()
{
    return false;
}

bool webserverPreventAutoStart = false;

device_t WIFI_device = {
    .initialize = initializeDisabledDevice,
    .start = nullptr,
    .event = nullptr,
    .timeout = nullptr,
    .subscribe = EVENT_NONE,
};

device_t RGB_device = {
    .initialize = initializeDisabledDevice,
    .start = nullptr,
    .event = nullptr,
    .timeout = nullptr,
    .subscribe = EVENT_NONE,
};
