#include "Arduino.h"
#include "OTA.h"
#include "config.h"
#include "common.h"
#include "handset.h"
#include "hwTimer.h"
#include "stubborn_sender.h"
#include "TXOTAConnector.h"
#include "../elrs_main.h"

#include <stdio.h>
#include <string.h>

extern "C" {
#include "crsf_serial.h"
#include "elrs_config.h"
void elrs_cpp_request_wifi_mode(void);
void siw917_lr1121_handle_deferred_isr(void);
}

void setup();
void loop();
void SetRFLinkRate(uint8_t index);

extern StubbornSender DataUlSender;
extern TXOTAConnector otaConnector;
extern Handset *handset;

RXtimerState_e RXtimerState = tim_disconnected;

static bool siw917_upstream_tx_initialized = false;
static bool siw917_upstream_tx_running = false;

static void service_handset_input(uint8_t maxPasses)
{
    if (handset == nullptr)
    {
        return;
    }

    for (uint8_t i = 0; i < maxPasses; ++i)
    {
        const uint32_t before = crsf_serial_rx_available();
        if (before == 0U && i != 0U)
        {
            break;
        }

        handset->handleInput();

        if (crsf_serial_rx_available() == 0U)
        {
            break;
        }
    }
}

static void apply_siw917_startup_config(void)
{
    elrs_config_t *cfg = elrs_config_get();
    if (cfg == nullptr)
    {
        printf("[ELRS_CPP] upstream startup config unavailable; keeping rate_index=%u\n",
               (unsigned)config.GetRate());
        return;
    }

    const uint8_t requestedRate = cfg->rate_index;
    printf("[ELRS_CPP] SiW917 NVM startup: rate_index=%u domain_low=%u domain_high=%u uid=%02X:%02X:%02X:%02X:%02X:%02X\n",
           (unsigned)requestedRate,
           (unsigned)cfg->reg_domain_low,
           (unsigned)cfg->reg_domain_high,
           cfg->uid[0], cfg->uid[1], cfg->uid[2],
           cfg->uid[3], cfg->uid[4], cfg->uid[5]);

    if (requestedRate >= RATE_MAX || !isSupportedRFRate(requestedRate))
    {
        printf("[ELRS_CPP] WARNING: requested rate_index=%u is not supported; keeping upstream rate_index=%u\n",
               (unsigned)requestedRate,
               (unsigned)config.GetRate());
        return;
    }

    if (config.GetRate() != requestedRate)
    {
#if defined(SIW917_ELRS_TX_PC_BENCH)
        // Bench mode owns startup config in SiW917 NVM. Avoid marking upstream
        // TxConfig dirty here because upstream will try to commit it later.
        printf("[ELRS_CPP] applying SiW917 runtime RF rate %u -> %u (no upstream config commit)\n",
               (unsigned)config.GetRate(),
               (unsigned)requestedRate);
#else
        printf("[ELRS_CPP] applying SiW917 startup rate %u -> %u\n",
               (unsigned)config.GetRate(),
               (unsigned)requestedRate);
        config.SetRate(requestedRate);
#endif
    }

    SetRFLinkRate(requestedRate);

    if (ExpressLRS_currAirRate_Modparams != nullptr)
    {
        printf("[ELRS_CPP] active RF rate after SiW917 config: rate_index=%u rf_mode=%u interval=%ld us\n",
               (unsigned)ExpressLRS_currAirRate_Modparams->index,
               (unsigned)ExpressLRS_currAirRate_Modparams->enum_rate,
               (long)ExpressLRS_currAirRate_Modparams->interval);
    }
}

extern "C" bool elrs_tx_init(void)
{
    if (siw917_upstream_tx_initialized)
    {
        return connectionState != radioFailed && connectionState != hardwareUndefined;
    }

    printf("[ELRS_CPP] upstream tx wrapper initializing config storage\n");
    printf("[ELRS_CPP] RADIO CRSF QUIET BUILD: five-second handset/bus summaries active\n");
#if defined(SIW917_ELRS_TX_PC_BENCH)
    printf("[ELRS_CPP] TX BENCH BUILD: quiet link diagnostics + acquisition sync boost\n");
#endif
    if (elrs_config_init() != 0)
    {
        printf("[ELRS_CPP] WARNING: config init failed, upstream setup will use defaults\n");
    }

    setup();
    apply_siw917_startup_config();
    RXtimerState = tim_disconnected;
    siw917_upstream_tx_initialized =
        connectionState != radioFailed && connectionState != hardwareUndefined;
    printf("[ELRS_CPP] upstream tx setup complete state=%u initialized=%u\n",
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

    printf("[ELRS_CPP] upstream tx scheduler start\n");
    siw917_upstream_tx_running = true;
    hwTimer::resume();
    printf("[ELRS_CPP] upstream tx scheduler started\n");
}

extern "C" void elrs_tx_loop(void)
{
    if (siw917_upstream_tx_initialized)
    {
        static uint32_t lastCrsfDebugMs = 0;
        const uint32_t now = millis();

        service_handset_input(96U);

        if ((uint32_t)(now - lastCrsfDebugMs) >= 5000U)
        {
            lastCrsfDebugMs = now;
            printf("[ELRS_CPP] tx_loop heartbeat running=%u state=%u handset=%u\n",
                   siw917_upstream_tx_running ? 1U : 0U,
                   (unsigned)connectionState,
                   handset != nullptr ? 1U : 0U);
            crsf_serial_debug_dump();
        }

        siw917_lr1121_handle_deferred_isr();
        loop();
        service_handset_input(32U);
        siw917_lr1121_handle_deferred_isr();
    }
}

extern "C" void elrs_tx_stop(void)
{
    if (siw917_upstream_tx_running)
    {
        hwTimer::stop();
        printf("[ELRS_CPP] upstream tx scheduler stopped\n");
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
