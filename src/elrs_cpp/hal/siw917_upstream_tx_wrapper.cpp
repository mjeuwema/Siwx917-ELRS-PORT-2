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
void siw917_txdbg_get(uint32_t *send_attempts,
                      uint32_t *send_sync,
                      uint32_t *send_rc,
                      uint32_t *send_data,
                      uint32_t *lbt_none,
                      uint32_t *tx_done_cb,
                      uint32_t *tx_done_ignored,
                      uint32_t *rx_window_req,
                      uint32_t *rx_expected,
                      uint32_t *rx_missed,
                      uint32_t *rx_done_cb,
                      uint32_t *rx_ignored,
                      uint32_t *rx_accept,
                      uint32_t *rx_reject,
                      uint8_t *nonce,
                      uint8_t *tlm_phase,
                      uint8_t *busy);
uint32_t lr1121_get_busy_fast_fail_count(void);
uint32_t lr1121_get_raw_gspi_fail_count(void);
uint32_t lr1121_get_rx_arm_max_us(void);
uint32_t lr1121_get_rx_arm_count(void);
uint32_t lr1121_get_rx_arm_fail_count(void);
void lr1121_reset_rx_arm_stats(void);
uint32_t lr1121_hal_get_direct_dio_count(void);
uint32_t lr1121_hal_get_direct_reentrant_count(void);
uint32_t lr1121_hal_get_level_requeue_count(void);
uint32_t lr1121_hal_get_stage_delay_max_us(void);
void lr1121_hal_reset_stage_delay_stats(void);
}

void setup();
void loop();
void SetRFLinkRate(uint8_t index);

extern StubbornSender DataUlSender;
extern TXOTAConnector otaConnector;
extern Handset *handset;
extern volatile uint32_t busy_timeout_count;

RXtimerState_e RXtimerState = tim_disconnected;

static bool siw917_upstream_tx_initialized = false;
static bool siw917_upstream_tx_running = false;
static uint8_t last_reported_rate = UINT8_MAX;
static int16_t last_reported_connection_state = -1;
static bool link_quality_state_initialized = false;
static bool link_quality_degraded = false;

static const char *rf_rate_name(uint8_t index)
{
    static const char *const names[RATE_MAX] = {
        "900 GFSK 1000 Full", "900 250Hz",      "900 200Hz Full",
        "900 200Hz",           "900 100Hz Full", "900 100Hz",
        "900 50Hz",            "900 25Hz",       "900 D50 DVDA",
        "2.4 GFSK 1000",       "2.4 D500 DVDA",  "2.4 D250 DVDA",
        "2.4 500Hz",           "2.4 333Hz Full", "2.4 250Hz",
        "2.4 150Hz",           "2.4 100Hz Full", "2.4 50Hz",
        "Dual 150Hz",          "Dual 100Hz Full",
    };

    return index < RATE_MAX ? names[index] : "unknown";
}

static const char *connection_state_name(connectionState_e state)
{
    switch (state)
    {
    case connected:
        return "connected";
    case tentative:
        return "tentative";
    case awaitingModelId:
        return "awaiting-model";
    case disconnected:
        return "disconnected";
    case noCrossfire:
        return "no-handset";
    case radioFailed:
        return "radio-failed";
    case hardwareUndefined:
        return "hardware-undefined";
    default:
        return "other";
    }
}

static void print_rf_counter_snapshot(const char *event)
{
    crsf_serial_rx_diag_t crsf_diag = {};
    uint32_t send_attempts = 0;
    uint32_t send_sync = 0;
    uint32_t send_rc = 0;
    uint32_t send_data = 0;
    uint32_t lbt_none = 0;
    uint32_t tx_done = 0;
    uint32_t tx_ignored = 0;
    uint32_t rx_windows = 0;
    uint32_t rx_expected = 0;
    uint32_t rx_missed = 0;
    uint32_t rx_done = 0;
    uint32_t rx_ignored = 0;
    uint32_t rx_accepted = 0;
    uint32_t rx_rejected = 0;
    uint8_t nonce = 0;
    uint8_t tlm_phase = 0;
    uint8_t busy = 0;
    siw917_txdbg_get(&send_attempts, &send_sync, &send_rc, &send_data,
                     &lbt_none, &tx_done, &tx_ignored, &rx_windows,
                     &rx_expected, &rx_missed, &rx_done, &rx_ignored,
                     &rx_accepted, &rx_rejected, &nonce, &tlm_phase, &busy);
    crsf_serial_get_rx_diag(&crsf_diag);

    printf("[RF_EVENT] %s busy_timeout=%lu busy_fast_fail=%lu "
           "gspi_fail=%lu dio=%lu reentrant=%lu requeue=%lu "
           "turnaround{dio_max=%luus arm_max=%luus arm=%lu fail=%lu} "
           "tx=%lu/%lu ignored=%lu tlm{win=%lu expect=%lu miss=%lu "
           "irq=%lu ok=%lu reject=%lu ignored=%lu} nonce=%u phase=%u busy=%u "
           "crsf{baud=%lu rx=%u/%u avail=%lu dma=%lu/%lu bytes=%lu "
           "busy=%lu fail=%lu timeout=%lu overrun=%lu}\n",
           event,
           (unsigned long)busy_timeout_count,
           (unsigned long)lr1121_get_busy_fast_fail_count(),
           (unsigned long)lr1121_get_raw_gspi_fail_count(),
           (unsigned long)lr1121_hal_get_direct_dio_count(),
           (unsigned long)lr1121_hal_get_direct_reentrant_count(),
           (unsigned long)lr1121_hal_get_level_requeue_count(),
           (unsigned long)lr1121_hal_get_stage_delay_max_us(),
           (unsigned long)lr1121_get_rx_arm_max_us(),
           (unsigned long)lr1121_get_rx_arm_count(),
           (unsigned long)lr1121_get_rx_arm_fail_count(),
           (unsigned long)tx_done,
           (unsigned long)send_attempts,
           (unsigned long)tx_ignored,
           (unsigned long)rx_windows,
           (unsigned long)rx_expected,
           (unsigned long)rx_missed,
           (unsigned long)rx_done,
           (unsigned long)rx_accepted,
           (unsigned long)rx_rejected,
           (unsigned long)rx_ignored,
           (unsigned)nonce,
           (unsigned)tlm_phase,
           (unsigned)busy,
           (unsigned long)crsf_diag.baud_rate,
           crsf_diag.rx_enabled ? 1U : 0U,
           crsf_diag.rx_armed ? 1U : 0U,
           (unsigned long)crsf_diag.available,
           (unsigned long)crsf_diag.dma_complete_count,
           (unsigned long)crsf_diag.dma_arm_count,
           (unsigned long)crsf_diag.dma_bytes_published,
           (unsigned long)crsf_diag.dma_rearm_busy_count,
           (unsigned long)crsf_diag.dma_rearm_fail_count,
           (unsigned long)crsf_diag.dma_timeout_count,
           (unsigned long)crsf_diag.rx_overrun_count);
}

static void report_rf_mode_if_changed(bool force)
{
    const expresslrs_mod_settings_s *const mod =
        ExpressLRS_currAirRate_Modparams;
    const expresslrs_rf_pref_params_s *const perf =
        ExpressLRS_currAirRate_RFperfParams;
    if (mod == nullptr || perf == nullptr ||
        (!force && mod->index == last_reported_rate))
    {
        return;
    }

    last_reported_rate = mod->index;
    link_quality_state_initialized = false;
    const uint32_t interval = (uint32_t)mod->interval;
    const uint32_t toa = perf->TOA;
    const uint32_t margin = interval > toa ? interval - toa : 0U;
    const char *const band = RadioBandMod::isB2G4(mod->radio_type)
                                 ? "2.4GHz"
                                 : (RadioBandMod::isBDUAL(mod->radio_type)
                                        ? "dual"
                                        : "900MHz");
    const char *const modulation =
        RadioBandMod::isGFSK(mod->radio_type) ? "GFSK" : "LoRa";

    printf("[RF_MODE] index=%u name=\"%s\" enum=%u band=%s mod=%s "
           "interval=%luus toa=%luus margin=%luus bw=%u sf=%u cr=%u "
           "preamble=%u payload=%u sends=%u hop=%u tlm_default=1/%u "
           "tlm_current=1/%u\n",
           (unsigned)mod->index, rf_rate_name(mod->index),
           (unsigned)mod->enum_rate, band, modulation,
           (unsigned long)interval, (unsigned long)toa,
           (unsigned long)margin, (unsigned)mod->bw, (unsigned)mod->sf,
           (unsigned)mod->cr, (unsigned)mod->PreambleLen,
           (unsigned)mod->PayloadLength, (unsigned)mod->numOfSends,
           (unsigned)mod->FHSShopInterval,
           (unsigned)TLMratioEnumToValue(mod->TLMinterval),
           (unsigned)ExpressLRS_currTlmDenom);
    print_rf_counter_snapshot("rate-change");
    lr1121_hal_reset_stage_delay_stats();
    lr1121_reset_rx_arm_stats();
}

static void report_link_events(void)
{
    if (last_reported_connection_state != (int16_t)connectionState)
    {
        printf("[RF_LINK] state=%s(%u) rate=%u name=\"%s\" lq=%u "
               "rssi=%u/%u snr=%d\n",
               connection_state_name(connectionState),
               (unsigned)connectionState,
               ExpressLRS_currAirRate_Modparams != nullptr
                   ? (unsigned)ExpressLRS_currAirRate_Modparams->index
                   : 255U,
               ExpressLRS_currAirRate_Modparams != nullptr
                   ? rf_rate_name(ExpressLRS_currAirRate_Modparams->index)
                   : "none",
               (unsigned)linkStats.uplink_Link_quality,
               (unsigned)linkStats.uplink_RSSI_1,
               (unsigned)linkStats.uplink_RSSI_2,
               (int)linkStats.uplink_SNR);
        print_rf_counter_snapshot("link-state");
        last_reported_connection_state = (int16_t)connectionState;
        link_quality_state_initialized = false;
    }

    if (connectionState != connected || linkStats.uplink_Link_quality == 0U)
    {
        return;
    }

    const bool degraded = link_quality_state_initialized
                              ? (link_quality_degraded
                                     ? linkStats.uplink_Link_quality < 90U
                                     : linkStats.uplink_Link_quality < 80U)
                              : linkStats.uplink_Link_quality < 80U;
    if (!link_quality_state_initialized || degraded != link_quality_degraded)
    {
        if (link_quality_state_initialized)
        {
            printf("[RF_LINK] telemetry-%s rate=%u lq=%u rssi=%u/%u snr=%d\n",
                   degraded ? "degraded" : "recovered",
                   ExpressLRS_currAirRate_Modparams != nullptr
                       ? (unsigned)ExpressLRS_currAirRate_Modparams->index
                       : 255U,
                   (unsigned)linkStats.uplink_Link_quality,
                   (unsigned)linkStats.uplink_RSSI_1,
                   (unsigned)linkStats.uplink_RSSI_2,
                   (int)linkStats.uplink_SNR);
            print_rf_counter_snapshot(degraded ? "telemetry-degraded"
                                               : "telemetry-recovered");
        }
        link_quality_degraded = degraded;
        link_quality_state_initialized = true;
    }
}

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

    report_rf_mode_if_changed(true);
}

extern "C" bool elrs_tx_init(void)
{
    if (siw917_upstream_tx_initialized)
    {
        return connectionState != radioFailed && connectionState != hardwareUndefined;
    }

    printf("[ELRS_CPP] upstream tx wrapper initializing config storage\n");
    printf("[ELRS_CPP] RADIO CRSF QUIET BUILD: periodic runtime diagnostics disabled\n");
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
        service_handset_input(96U);

        siw917_lr1121_handle_deferred_isr();
        loop();
        service_handset_input(32U);
        siw917_lr1121_handle_deferred_isr();
        report_rf_mode_if_changed(false);
        report_link_events();
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
