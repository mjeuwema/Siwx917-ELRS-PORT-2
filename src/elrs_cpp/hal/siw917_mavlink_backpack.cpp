#include "Arduino.h"
#include "common.h"
#include "config.h"
#include "siw917_mavlink_wifi.h"

#include <stdio.h>

extern TxConfig config;

/* Upstream's Lua command sets this flag through handleSimpleSendCmd(). */
bool TxBackpackWiFiReadyToSend = false;

namespace {

class SiW917MavlinkWifiStream final : public Stream {
public:
    size_t write(uint8_t) override { return 1U; }
    size_t write(const uint8_t *, size_t size) override { return size; }

    int available() override
    {
        return (int)siw917_mavlink_wifi_uplink_available();
    }

    int read() override
    {
        return siw917_mavlink_wifi_uplink_read();
    }

    size_t readBytes(uint8_t *buffer, size_t length) override
    {
        return siw917_mavlink_wifi_uplink_read_bytes(buffer, length);
    }
};

SiW917MavlinkWifiStream mavlinkWifiStream;
bool lastMavlinkMode;
bool luaMavlinkWifiEnabled;
bool lastBridgeRequested;

} // namespace

extern "C" void siw917_mavlink_backpack_set_lua_enabled(bool enabled)
{
    luaMavlinkWifiEnabled = enabled;
}

extern "C" bool siw917_mavlink_backpack_get_lua_enabled(void)
{
    return luaMavlinkWifiEnabled;
}

extern "C" void siw917_mavlink_backpack_init(void)
{
    if (!siw917_mavlink_wifi_init()) {
        printf("[MAVWIFI] Ground-station transport unavailable\n");
        return;
    }

    /* Keep TxUSB as upstream's null stream and expose WiFi as its backpack. */
    BackpackOrLogStrm = &mavlinkWifiStream;
    lastMavlinkMode = config.GetLinkMode() == TX_MAVLINK_MODE;
    luaMavlinkWifiEnabled = false;
    lastBridgeRequested = false;
    siw917_mavlink_wifi_set_enabled(false);
    printf("[MAVWIFI] Internal MAVLink backpack attached (UDP 14550)\n");
    printf("[MAVWIFI] Use Lua WiFi > MAVLink WiFi Off/On\n");
}

extern "C" void siw917_mavlink_backpack_service(void)
{
    const bool mavlinkMode = config.GetLinkMode() == TX_MAVLINK_MODE;
    const bool bridgeRequested = mavlinkMode && luaMavlinkWifiEnabled;
    const bool linkModeChanged = mavlinkMode != lastMavlinkMode;
    if (!linkModeChanged && bridgeRequested == lastBridgeRequested) {
        return;
    }

    lastMavlinkMode = mavlinkMode;
    lastBridgeRequested = bridgeRequested;
    if (bridgeRequested) {
        siw917_mavlink_wifi_set_enabled(true);
    } else if (siw917_mavlink_wifi_is_enabled()) {
        siw917_mavlink_wifi_set_enabled(false);
    } else if (linkModeChanged && mavlinkMode) {
        printf("[MAVWIFI] Link Mode MAVLink; WiFi bridge remains Off\n");
    }
}

void checkBackpackUpdate()
{
    if (!TxBackpackWiFiReadyToSend) {
        return;
    }

    TxBackpackWiFiReadyToSend = false;
    if (config.GetLinkMode() != TX_MAVLINK_MODE) {
        printf("[MAVWIFI] Backpack WiFi request ignored: select Link Mode MAVLink first\n");
        return;
    }

    siw917_mavlink_backpack_set_lua_enabled(true);
}

void sendCRSFTelemetryToBackpack(uint8_t *)
{
}

void sendMAVLinkTelemetryToBackpack(uint8_t *frame)
{
    if (frame == nullptr) {
        return;
    }

    const uint8_t length = frame[CRSF_TELEMETRY_LENGTH_INDEX];
    (void)siw917_mavlink_wifi_enqueue_downlink(
      frame + CRSF_FRAME_NOT_COUNTED_BYTES,
      length);
}
