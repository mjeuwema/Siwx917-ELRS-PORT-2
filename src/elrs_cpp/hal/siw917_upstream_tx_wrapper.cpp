#include "Arduino.h"
#include "common.h"
#include "hwTimer.h"
#include "OTA.h"
#include "stubborn_sender.h"
#include "TXOTAConnector.h"
#include "../elrs_main.h"
#include "siw917_mavlink_backpack.h"
#include "mlrs_ota.h"

#include <stdio.h>
#include <string.h>

extern "C" {
#include "elrs_config.h"
#include "crsf_serial.h"
void elrs_cpp_request_wifi_mode(void);
void siw917_lr1121_handle_deferred_isr(void);
void siw917_tx_get_link_diag(uint32_t *last_tlm_ms,
                             uint32_t *last_link_stats_ms,
                             uint8_t *telemetry_phase,
                             bool *radio_busy);
}

void setup();
void loop();

extern StubbornSender DataUlSender;
extern TXOTAConnector otaConnector;

RXtimerState_e RXtimerState = tim_disconnected;

static bool siw917_upstream_tx_initialized = false;
static bool siw917_upstream_tx_running = false;
static connectionState_e siw917_last_reported_link_state = hardwareUndefined;
static uint8_t siw917_last_reported_rate_index = 0xFFU;

static const char *siw917_link_state_name(connectionState_e state)
{
    switch (state)
    {
    case connected:
        return "connected";
    case tentative:
        return "tentative";
    case awaitingModelId:
        return "awaiting-model";
    case radioFailed:
        return "radio-failed";
    case hardwareUndefined:
        return "hardware-undefined";
    case wifiUpdate:
        return "wifi-update";
    default:
        return "disconnected";
    }
}

static void siw917_report_link_transition()
{
    const expresslrs_mod_settings_s *rate = ExpressLRS_currAirRate_Modparams;
    const uint8_t rateIndex = rate != nullptr ? rate->index : 0xFFU;
    if (connectionState == siw917_last_reported_link_state &&
        rateIndex == siw917_last_reported_rate_index)
    {
        return;
    }

    const connectionState_e previousState = siw917_last_reported_link_state;
    siw917_last_reported_link_state = connectionState;
    siw917_last_reported_rate_index = rateIndex;
    printf("[TXLINK] state=%s rate_index=%u rf_mode=%u interval_us=%lu "
           "tlm_denom=%u lq=%u\n",
           siw917_link_state_name(connectionState),
           rate != nullptr ? (unsigned)rate->index : 255U,
           rate != nullptr ? (unsigned)rate->enum_rate : 255U,
           rate != nullptr ? (unsigned long)rate->interval : 0UL,
           (unsigned)ExpressLRS_currTlmDenom,
           (unsigned)linkStats.uplink_Link_quality);

    if (connectionState == previousState)
    {
        return;
    }

    uint32_t lastTlmMs = 0U;
    uint32_t lastLinkStatsMs = 0U;
    uint8_t telemetryPhase = 0U;
    bool radioBusy = false;
    siw917_tx_get_link_diag(&lastTlmMs, &lastLinkStatsMs,
                            &telemetryPhase, &radioBusy);

    const uint32_t now = millis();
    const uint32_t telemetryAge = lastTlmMs != 0U ? now - lastTlmMs : UINT32_MAX;
    const uint32_t linkStatsAge =
        lastLinkStatsMs != 0U ? now - lastLinkStatsMs : UINT32_MAX;
    uint32_t lossTimeout = 514U;
    if (rate != nullptr)
    {
        const uint32_t calculated =
            ((uint32_t)ExpressLRS_currTlmDenom * rate->interval) / 200U + 2U;
        if (calculated > lossTimeout)
        {
            lossTimeout = calculated;
        }
    }

    crsf_serial_rx_diag_t rxDiag = {};
    crsf_serial_tx_diag_t txDiag = {};
    crsf_serial_get_rx_diag(&rxDiag);
    crsf_serial_get_tx_diag(&txDiag);
    printf("[TXLINK_DIAG] tlm_age_ms=%lu timeout_ms=%lu phase=%u "
           "radio_busy=%u linkstats_age_ms=%lu handset_rx_overrun=%lu "
           "handset_tx_q=%u/%u full=%lu errors=%lu temt=%lu\n",
           (unsigned long)telemetryAge,
           (unsigned long)lossTimeout,
           (unsigned)telemetryPhase,
           radioBusy ? 1U : 0U,
           (unsigned long)linkStatsAge,
           (unsigned long)rxDiag.rx_overrun_count,
           (unsigned)txDiag.queue_depth,
           (unsigned)txDiag.queue_high_water,
           (unsigned long)txDiag.queue_full_count,
           (unsigned long)txDiag.queue_error_count,
           (unsigned long)txDiag.temt_timeout_count);
}

extern "C" bool elrs_tx_init(void)
{
    if (siw917_upstream_tx_initialized)
    {
        return connectionState != radioFailed && connectionState != hardwareUndefined;
    }

    printf("[ELRS_CPP] initializing upstream TX with SiW917 HAL\n");
#if defined(SIW917_ELRS_CRSF_BENCH_2WIRE)
    printf("[ELRS_CPP] handset transport: EdgeTX two-wire electrical, paced CRSF replies\n");
#endif
    if (elrs_config_init() != 0)
    {
        printf("[ELRS_CPP] WARNING: platform options storage init failed\n");
    }

    // Upstream owns RF rate, Lua parameters, handset processing, and model
    // persistence. The platform wrapper only enters the upstream lifecycle.
    setup();
    siw917_mavlink_backpack_init();
    mlrs_ota_on_elrs_ready();
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
    hwTimer::service();
    mlrs_ota_loop();
    siw917_lr1121_handle_deferred_isr();
    /* 19 Hz SF6 UL is ~24 ms. loop() after TXnb delayed TXdone/RXnb
     * and clipped the downlink. Wait for air-done first. */
    if (mlrs_ota_is_active() && mlrs_ota_tx_air()) {
        const uint32_t t0 = millis();
        while (mlrs_ota_tx_air() &&
               (int32_t)(millis() - t0) < 40) {
            siw917_lr1121_handle_deferred_isr();
        }
    }
    loop();
    siw917_mavlink_backpack_service();
    siw917_lr1121_handle_deferred_isr();
    mlrs_ota_loop();
    if (!mlrs_ota_is_active()) {
        siw917_report_link_transition();
    }
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

extern "C" void elrs_tx_enter_wifi_update(void)
{
    InBindingMode = false;
    setConnectionState(wifiUpdate);
}

void setWifiUpdateMode()
{
    elrs_tx_enter_wifi_update();
    elrs_cpp_request_wifi_mode();
}
