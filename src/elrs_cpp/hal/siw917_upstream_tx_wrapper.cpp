#include "Arduino.h"
#include "common.h"
#include "hwTimer.h"
#include "OTA.h"
#include "stubborn_sender.h"
#include "TXOTAConnector.h"
#include "../elrs_main.h"

#include <stdio.h>
#include <string.h>

extern "C" {
#include "elrs_config.h"
void elrs_cpp_request_wifi_mode(void);
void siw917_lr1121_handle_deferred_isr(void);
}

void setup();
void loop();

extern StubbornSender DataUlSender;
extern TXOTAConnector otaConnector;

RXtimerState_e RXtimerState = tim_disconnected;

static bool siw917_upstream_tx_initialized = false;
static bool siw917_upstream_tx_running = false;

extern "C" bool elrs_tx_init(void)
{
    if (siw917_upstream_tx_initialized)
    {
        return connectionState != radioFailed && connectionState != hardwareUndefined;
    }

    printf("[ELRS_CPP] initializing upstream TX with SiW917 HAL\n");
    if (elrs_config_init() != 0)
    {
        printf("[ELRS_CPP] WARNING: platform options storage init failed\n");
    }

    // Upstream owns RF rate, Lua parameters, handset processing, and model
    // persistence. The platform wrapper only enters the upstream lifecycle.
    setup();
    RXtimerState = tim_disconnected;
    siw917_upstream_tx_initialized =
        connectionState != radioFailed && connectionState != hardwareUndefined;
    printf("[ELRS_CPP] upstream TX setup complete state=%u initialized=%u\n",
           (unsigned)connectionState,
           siw917_upstream_tx_initialized ? 1U : 0U);
    return siw917_upstream_tx_initialized;
}

extern "C" void elrs_tx_start(void)
{
    if (!siw917_upstream_tx_initialized)
    {
        return;
    }

    siw917_upstream_tx_running = true;
    hwTimer::resume();
    printf("[ELRS_CPP] upstream TX scheduler started\n");
}

extern "C" void elrs_tx_loop(void)
{
    if (!siw917_upstream_tx_initialized)
    {
        return;
    }

    siw917_lr1121_handle_deferred_isr();
    loop();
    siw917_lr1121_handle_deferred_isr();
}

extern "C" void elrs_tx_stop(void)
{
    if (siw917_upstream_tx_running)
    {
        hwTimer::stop();
        printf("[ELRS_CPP] upstream TX scheduler stopped\n");
    }
    siw917_upstream_tx_running = false;
}

extern "C" void elrs_tx_abort_data_uplink(void)
{
    DataUlSender.ResetState();
    otaConnector.resetOutputQueue();
}

extern "C" bool elrs_is_connected(void)
{
    return connectionState == connected;
}

extern "C" elrs_connection_state_t elrs_get_connection_state(void)
{
    if (InBindingMode)
    {
        return ELRS_BINDING;
    }

    switch (connectionState)
    {
    case connected:
        return ELRS_CONNECTED;
    case tentative:
    case awaitingModelId:
        return ELRS_TENTATIVE;
    case radioFailed:
    case hardwareUndefined:
        return ELRS_RADIO_FAILED;
    default:
        return ELRS_DISCONNECTED;
    }
}

extern "C" uint8_t elrs_get_channels(uint32_t *channels)
{
    if (channels != nullptr)
    {
        memcpy(channels, ChannelData, sizeof(ChannelData));
    }
    return CRSF_NUM_CHANNELS;
}

extern "C" void elrs_get_link_stats(elrs_link_stats_t *stats)
{
    if (stats == nullptr)
    {
        return;
    }

    stats->rssi_1 = linkStats.uplink_RSSI_1 ? -(int8_t)linkStats.uplink_RSSI_1 : 0;
    stats->rssi_2 = linkStats.uplink_RSSI_2 ? -(int8_t)linkStats.uplink_RSSI_2 : 0;
    stats->lq = linkStats.uplink_Link_quality;
    stats->snr = linkStats.uplink_SNR;
    stats->rf_mode = linkStats.rf_Mode;
    stats->active_ant = linkStats.active_antenna;
}

extern "C" void elrs_set_channel_callback(elrs_channel_callback_t callback)
{
    (void)callback;
}

extern "C" void elrs_enter_binding_mode(void)
{
    InBindingMode = true;
}

extern "C" void elrs_exit_binding_mode(void)
{
    InBindingMode = false;
}

void setWifiUpdateMode()
{
    elrs_cpp_request_wifi_mode();
}

bool isThisAMavPacket(uint8_t *buffer, uint16_t bufferSize)
{
    (void)buffer;
    (void)bufferSize;
    return false;
}

void convert_mavlink_to_crsf_telem(crsf_addr_e destination, uint8_t *buffer, uint8_t count)
{
    (void)destination;
    (void)buffer;
    (void)count;
}
