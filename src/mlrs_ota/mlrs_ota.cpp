#include "mlrs_ota.h"
#include "mlrs_aes_gcm.h"
#include "mlrs_mavlinkx.h"

#include "Arduino.h"
#include "FHSS.h"
#include "LR1121.h"
#include "LR1121_Regs.h"
#include "common.h"
#include "crsf_protocol.h"
#include "device.h"
#include "hwTimer.h"
#include "OTA.h"

#include <stdio.h>
#include <string.h>

extern "C" {
#include "elrs_config.h"
bool lr1121_elrs_calib_image(uint32_t freq_min, uint32_t freq_max);
}

extern LR1121Driver Radio;
extern uint8_t UID[UID_LEN];
extern uint32_t ChannelData[CRSF_NUM_CHANNELS];
extern expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;

#if defined(SIW917_ELRS_TARGET_TX) || defined(TARGET_TX)
#define MLRS_OTA_IS_TX 1
#include "CRSFRouter.h"
#include "TXOTAConnector.h"
#include "mlrs_mbridge.h"
#ifndef MinPower
#define MinPower PWR_10mW
#define MaxPower PWR_100mW
#endif
#include "POWERMGNT.h"
#include "config.h"
#include "stubborn_sender.h"
extern StubbornSender DataUlSender;
extern CRSFRouter crsfRouter;
extern TXOTAConnector otaConnector;
extern "C" {
#include "siw917_mavlink_wifi.h"
}
extern "C" void siw917_tx_note_mlrs_downlink(uint8_t lq, int8_t rssi, int8_t snr);
extern "C" void siw917_tx_note_mlrs_link_lost(void);
extern "C" void siw917_tx_publish_mlrs_linkstats(void);
#if defined(SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
// Same counters tx_main.cpp uses for a rate/model change. A cold RX only
// locks from PACKET_TYPE_SYNC on the FHSS sync channel.
extern volatile uint8_t syncSpamCounter;
extern volatile uint8_t syncSpamCounterAfterRateChange;
extern uint32_t SyncPacketLastSent;
extern volatile bool busyTransmitting;
#endif
#else
#define MLRS_OTA_IS_TX 0
#include "mlrs_mbridge_params.h"
extern "C" {
void elrs_cpp_request_wifi_mode(void);
void elrs_enter_binding_mode(void);
int elrs_config_save_with_rf_rearm(void);
int elrs_config_save(void);
void elrs_apply_bind_storage_change(uint8_t bindStorage);
void siw917_rx_set_model_match_id(uint8_t modelId);
void siw917_rx_set_force_telemetry_off(uint8_t forceOff);
void ble_remote_id_service_set_enabled(bool enabled);
int ble_remote_id_service_start(bool enabled);
}
#endif

extern elrsLinkStatistics_t linkStats;
extern bool connectionHasModelMatch;
#if !MLRS_OTA_IS_TX
extern uint8_t uplinkLQ;
#endif

#undef PACKED
#define PACKED(__Declaration__) __Declaration__ __attribute__((packed))

#define FRAME_TX_RX_LEN 99
#define FRAME_TX_RCDATA1_LEN 6
#define FRAME_TX_RCDATA2_LEN 10
#define FRAME_TX_PAYLOAD_LEN 52
#define FRAME_RX_PAYLOAD_LEN 70
#define FRAME_TX_AAD_LEN 13
#define FRAME_RX_AAD_LEN 11
#define FRAME_TX_SEAL_LEN (FRAME_TX_RCDATA1_LEN + FRAME_TX_RCDATA2_LEN + FRAME_TX_PAYLOAD_LEN)
#define FRAME_GCM_TAG_LEN 16
#define MLRS_OVERLAY_ID "20260813S"
#define MLRS_MUX_MAGIC 0x5A
#define MLRS_HOP_WIN 16
#define MLRS_HOP_FAIL_PCT 75
#define MLRS_HOP_CLEAR_OK 2
#define MLRS_HOP_MAX_SKIP 6
#define MLRS_GCM_CTR_BOOT_GAP 1024U
#define MLRS_GCM_CTR_RESERVE (1U << 20)
#define FHSS_MAX_HOPS 25
#define FHSS_NUM_915 25
#define FHSS_BIND_CH_915 19
#define FHSS_LIST_24_LEN 80
#define FHSS_BIND_CH_24 46
#define MLRS_SWITCH_CMD 0xE1
#define MLRS_RATE_CMD 0xE2
#define MLRS_BAND_CMD 0xE3
#define MLRS_ANNOUNCE_MIN_MS 4000
#define MLRS_RX_SCAN_MS 5000
#define MLRS_LOST_MS 2000

typedef struct {
  uint32_t interval_us;
  uint8_t bw;
  uint8_t sf;
  uint8_t cr;
  uint8_t preamble;
  uint8_t is_fsk;
  uint8_t hop_count;
  const char *name;
} mlrs_rate_cfg_t;

static const mlrs_rate_cfg_t kRates[MLRS_BAND_COUNT][MLRS_RATE_COUNT] = {
    {
        {32000, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF5,
         LR11XX_RADIO_LORA_CR_4_5, 12, 0, 25, "31Hz"},
        {53000, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF6,
         LR11XX_RADIO_LORA_CR_4_5, 12, 0, 25, "19Hz"},
        /* ELRS Config FSK: bw*10k=bitrate, sf=GFSK BW, cr*1k=fdev. */
        {20000, 10, LR11XX_RADIO_GFSK_BW_312000, 50, 16, 1, 25, "FSK50"},
    },
    {
        {20000, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF5,
         LR11XX_RADIO_LORA_CR_LI_4_5, 12, 0, 24, "50Hz"},
        {32000, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF6,
         LR11XX_RADIO_LORA_CR_LI_4_5, 12, 0, 18, "31Hz"},
        {53000, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF7,
         LR11XX_RADIO_LORA_CR_LI_4_5, 12, 0, 12, "19Hz"},
    },
};

enum {
  FRAME_TYPE_TX = 0x00,
  FRAME_TYPE_RX = 0x01,
  FRAME_TYPE_TX_RX_CMD = 0x02,
};

PACKED(typedef struct {
  uint32_t seq_no : 3;
  uint32_t ack : 1;
  uint32_t frame_type : 4;
  uint32_t antenna : 1;
  uint32_t rssi_u7 : 7;
  uint32_t fhss_index_band : 1;
  uint32_t fhss_index : 6;
  uint32_t spare : 2;
  uint32_t LQ_serial : 7;
  uint32_t transmit_antenna : 1;
  uint32_t payload_len : 7;
})
tTxFrameStatus;

PACKED(typedef struct {
  uint32_t seq_no : 3;
  uint32_t ack : 1;
  uint32_t frame_type : 4;
  uint32_t antenna : 1;
  uint32_t rssi_u7 : 7;
  uint32_t LQ_rc : 7;
  uint32_t LQ_serial : 7;
  uint32_t spare : 2;
  uint32_t transmit_antenna : 1;
  uint32_t payload_len : 7;
})
tRxFrameStatus;

PACKED(typedef struct {
  uint16_t ch0 : 11;
  uint16_t ch1 : 11;
  uint16_t ch2 : 11;
  uint16_t ch3 : 11;
  uint16_t ch12 : 2;
  uint16_t ch13 : 2;
})
tFrameRcData1;

PACKED(typedef struct {
  uint16_t ch4 : 11;
  uint16_t ch5 : 11;
  uint16_t ch6 : 11;
  uint16_t ch7 : 11;
  uint16_t ch14 : 2;
  uint16_t ch15 : 2;
  uint8_t ch8;
  uint8_t ch9;
  uint8_t ch10;
  uint8_t ch11;
})
tFrameRcData2;

PACKED(typedef struct {
  uint16_t sync_word;
  tTxFrameStatus status;
  uint32_t pkt_counter;
  uint16_t crc1;
  tFrameRcData1 rc1;
  tFrameRcData2 rc2;
  uint8_t payload[FRAME_TX_PAYLOAD_LEN];
  uint8_t tag[FRAME_GCM_TAG_LEN];
  uint16_t crc;
})
tTxFrame;

PACKED(typedef struct {
  uint16_t sync_word;
  tRxFrameStatus status;
  uint32_t pkt_counter;
  uint8_t payload[FRAME_RX_PAYLOAD_LEN];
  uint8_t tag[FRAME_GCM_TAG_LEN];
  uint16_t crc;
})
tRxFrame;

static_assert(sizeof(tTxFrameStatus) == 5, "tTxFrameStatus size");
static_assert(sizeof(tRxFrameStatus) == 5, "tRxFrameStatus size");
static_assert(sizeof(tFrameRcData2) == FRAME_TX_RCDATA2_LEN, "rc2 size");
static_assert(FRAME_TX_SEAL_LEN == 68, "tx gcm seal");
static_assert(sizeof(tTxFrame) == FRAME_TX_RX_LEN, "tTxFrame size");
static_assert(sizeof(tRxFrame) == FRAME_TX_RX_LEN, "tRxFrame size");

static const uint32_t kFhss915Hz[] = {
    902400000, 903000000, 903600000, 904200000, 904800000, 905400000,
    906000000, 906600000, 907200000, 907800000, 908400000, 909000000,
    909600000, 910200000, 910800000, 911400000, 912000000, 912600000,
    913200000, 913800000, 914400000, 915000000, 915600000, 916200000,
    916800000, 917400000, 918000000, 918600000, 919200000, 919800000,
    920400000, 921000000, 921600000, 922200000, 922800000, 923400000,
    924000000, 924600000, 925200000, 925800000, 926400000, 927000000,
    927600000,
};

static constexpr uint8_t kFhssListLen =
    (uint8_t)(sizeof(kFhss915Hz) / sizeof(kFhss915Hz[0]));

static uint8_t g_protocol = ELRS_AIR_PROTOCOL_ELRS;
static uint8_t g_band = MLRS_BAND_915;
static uint8_t g_rate = MLRS_RATE_31HZ;
static bool g_active = false;
static bool g_elrs_ready = false;
static bool g_lock_first_elrs_sync = false;
static uint32_t g_apply_at_ms = 0;
static uint8_t g_pending_protocol = 0xFF;
static volatile uint8_t g_pending_rate = 0xFF;
static volatile uint8_t g_pending_band = 0xFF;
static uint32_t g_apply_rate_at_ms = 0;
static uint32_t g_apply_band_at_ms = 0;
static uint8_t g_config_dirty = 0;
static uint32_t g_mlrs_started_ms = 0;
#if !MLRS_OTA_IS_TX
static uint32_t g_rate_scan_ms = 0;
#endif
static uint16_t g_sync_word = 0;
static uint32_t g_fhss_seed = 0;
static uint32_t g_fhss_list[FHSS_MAX_HOPS] = {};
static uint8_t g_fhss_ch[FHSS_MAX_HOPS] = {};
static uint8_t g_fhss_count = FHSS_NUM_915;
static uint8_t g_fhss_i = 0;
static uint32_t g_hop_mask = 0;
static uint8_t g_hop_gen = 0;
#if !MLRS_OTA_IS_TX
typedef struct {
  uint8_t vis;
  uint8_t fail;
  uint8_t probe_ok;
} hop_stat_t;
static hop_stat_t g_hop_stat[FHSS_MAX_HOPS] = {};
static uint32_t g_hop_proposed_mask = 0;
static uint8_t g_hop_proposed_gen = 0;
#endif
#if MLRS_OTA_IS_TX
static bool g_fhss_need_hop = false;
static bool g_logged_switch_cmd = false;
#else
static bool g_fhss_follow = false;
static volatile uint8_t g_fhss_arm_rx = 0;
static volatile uint8_t g_fhss_do_hop = 0;
static volatile uint8_t g_fhss_pending_hops = 0;
static volatile uint8_t g_slot_rx = 0;
static volatile uint8_t g_rx_need_rearm = 0;
static volatile uint8_t g_miss_streak = 0;
static bool g_had_link = false;
static uint32_t g_last_rf_ms = 0;
static uint8_t g_last_rx_spare = 0xFF;
#endif
#if MLRS_OTA_IS_TX
static uint8_t g_seq = 0;
#endif
static uint8_t g_lq = 0;
static volatile uint8_t g_valid_window = 0;
static volatile uint32_t g_last_rx_ms = 0;
static uint32_t g_last_hb_ms = 0;
static volatile uint8_t g_connected = 0;
static volatile uint8_t g_print_connected = 0;
static volatile uint8_t g_print_disconnected = 0;
static uint32_t g_tx_sent = 0;
static uint32_t g_rx_ok = 0;
static uint32_t g_rx_fail = 0;
static uint32_t g_rx_junk = 0;
static uint8_t g_last_rx_fail = 0;
#if MLRS_OTA_IS_TX
static volatile uint8_t g_tx_send_pending = 0;
static volatile uint8_t g_downlink_pending = 0;
#else
static volatile uint8_t g_uplink_pending = 0;
static volatile uint8_t g_tlm_ready = 0;
static volatile uint8_t g_tlm_busy = 0;
static volatile uint32_t g_tlm_busy_ms = 0;
#endif
static tTxFrame g_tx_frame = {};
static tRxFrame g_rx_frame = {};
static mlrs_gcm_ctx_t g_gcm_up = {};
static mlrs_gcm_ctx_t g_gcm_dn = {};
static uint32_t g_send_counter = 1;
static uint32_t g_recv_highest = 0;
static uint8_t g_gcm_ready = 0;
static uint8_t g_secret[MLRS_GCM_SECRET_LEN] = {};
static uint8_t g_secret_ok = 0;
static uint8_t g_last_ul_plen = 0;
static uint8_t g_last_dl_plen = 0;
#if MLRS_OTA_IS_TX
static uint8_t g_ul_hold[64] = {};
static uint8_t g_ul_hold_len = 0;
#else
static uint8_t g_dn_hold[64] = {};
static uint8_t g_dn_hold_len = 0;
static uint8_t g_rx_frame_valid = 0;
static tRxFrame g_rx_sent = {};
static uint8_t g_rx_sent_valid = 0;
static volatile uint8_t g_tlm_slot_pending = 0;
static volatile uint8_t g_tlm_slot_skip = 0;
#endif
static uint32_t g_arq_retry = 0;
static uint32_t g_arq_drop = 0;

/* Stock mLRS ARQ: downlink serial only. 3-bit seq, 1-bit ack, 1 retry
 * (live stock SetRetryCntAuto always ends at 1). Not compiled from Common/. */
#if MLRS_OTA_IS_TX
enum { ARQ_R_IDLE = 0, ARQ_R_MISSED, ARQ_R_WAS_IDLE, ARQ_R_RXED };
static uint8_t g_rarq_st;
static uint8_t g_rarq_seq;
static uint8_t g_rarq_last;
static uint8_t g_rarq_ack;
static uint8_t g_rarq_accept;
static uint8_t g_rarq_lost;

static void arq_init() {
  g_rarq_st = ARQ_R_IDLE;
  g_rarq_seq = 0;
  g_rarq_last = 0;
  g_rarq_ack = 0;
  g_rarq_accept = 0;
  g_rarq_lost = 0;
  g_arq_retry = 0;
  g_arq_drop = 0;
}

static void rarq_disconnected() { g_rarq_st = ARQ_R_IDLE; }

static void rarq_spin() {
  g_rarq_lost = 0;
  if (g_rarq_st == ARQ_R_WAS_IDLE) {
    g_rarq_accept = 1;
    g_rarq_lost = 1;
    g_rarq_last = g_rarq_seq;
    g_rarq_ack = g_rarq_seq;
  } else if (g_rarq_st == ARQ_R_RXED) {
    g_rarq_accept = (g_rarq_seq != g_rarq_last) ? 1 : 0;
    if (((g_rarq_seq - g_rarq_last) & 7U) > 1U) {
      g_rarq_lost = 1;
    }
    g_rarq_last = g_rarq_seq;
    g_rarq_ack = g_rarq_seq;
  } else {
    g_rarq_accept = 0;
  }
  if (g_rarq_lost) {
    ++g_arq_drop;
  }
}

static void rarq_missed() {
  g_rarq_st = ARQ_R_MISSED;
  rarq_spin();
}

static void rarq_received(uint8_t seq) {
  g_rarq_seq = seq & 7U;
  g_rarq_st = (g_rarq_st == ARQ_R_IDLE) ? ARQ_R_WAS_IDLE : ARQ_R_RXED;
  rarq_spin();
}
#else
enum { ARQ_T_IDLE = 0, ARQ_T_MISSED, ARQ_T_RXED };
static uint8_t g_tarq_st;
static uint8_t g_tarq_ack_bit;
static uint8_t g_tarq_seq;
static uint8_t g_tarq_retries;

static void arq_init() {
  g_tarq_st = ARQ_T_IDLE;
  g_tarq_ack_bit = 0;
  g_tarq_seq = 0;
  g_tarq_retries = 0;
  g_rx_sent_valid = 0;
  g_tlm_slot_pending = 0;
  g_tlm_slot_skip = 0;
  g_arq_retry = 0;
  g_arq_drop = 0;
}

static void tarq_disconnected() { g_tarq_st = ARQ_T_IDLE; }
static void tarq_missed() { g_tarq_st = ARQ_T_MISSED; }
static void tarq_ack(uint8_t ack_bit) {
  g_tarq_ack_bit = ack_bit & 1U;
  g_tarq_st = ARQ_T_RXED;
}

/* Compare ACK against last *sent* seq. Overlay sends in the RX ISR
 * before the next payload is packed, so ACK in this uplink is for the
 * previous downlink, not a not-yet-incremented next seq. */
static bool tarq_should_retry() {
  if (g_rx_sent_valid == 0) {
    return false;
  }
  const uint8_t lim = 1;
  bool nack = false;
  switch (g_tarq_st) {
  case ARQ_T_RXED:
    nack = ((g_tarq_ack_bit & 1U) != (g_tarq_seq & 1U));
    break;
  case ARQ_T_MISSED:
    nack = true;
    break;
  default:
    return false;
  }
  if (!nack) {
    return false;
  }
  if (g_tarq_retries >= lim) {
    return false;
  }
  ++g_tarq_retries;
  ++g_arq_retry;
  return true;
}

static uint8_t tarq_next_seq() {
  return (uint8_t)((g_tarq_seq + 1U) & 7U);
}

static void tarq_note_sent(uint8_t seq) {
  g_tarq_seq = seq & 7U;
  g_tarq_retries = 0;
}
#endif

static uint8_t mux_append(uint8_t *dst, uint8_t used, uint8_t max,
                         const uint8_t *frame, uint8_t flen) {
  if (dst == nullptr || frame == nullptr || flen == 0) {
    return used;
  }
  const uint8_t hdr = (used == 0) ? 2 : 0;
  if ((uint16_t)used + hdr + 1U + flen > max) {
    return used;
  }
  if (used == 0) {
    dst[0] = MLRS_MUX_MAGIC;
    dst[1] = 0;
    used = 2;
  }
  dst[used++] = flen;
  memcpy(dst + used, frame, flen);
  used = (uint8_t)(used + flen);
  dst[1]++;
  return used;
}

static hwTimerCallback_t g_saved_tick = nullptr;
static hwTimerCallback_t g_saved_tock = nullptr;
static bool (*g_saved_rx_cb)(SX12xxDriverCommon::rx_status) = nullptr;
static void (*g_saved_tx_cb)() = nullptr;
static uint32_t g_saved_interval_us = 0;

static void crc_init(uint16_t *crc) { *crc = 0xFFFF; }

static void crc_accumulate(uint8_t data, uint16_t *crcAccum) {
  uint8_t tmp = data ^ (uint8_t)(*crcAccum & 0xFF);
  tmp ^= (tmp << 4);
  *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

static void crc_accumulate_buf(uint16_t *crc, const uint8_t *buf, uint16_t len) {
  for (uint16_t i = 0; i < len; ++i) {
    crc_accumulate(buf[i], crc);
  }
}

static uint16_t crc_calculate(const uint8_t *buf, uint16_t len) {
  uint16_t crc;
  crc_init(&crc);
  crc_accumulate_buf(&crc, buf, len);
  return crc;
}

static uint32_t u32_le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void put_u32_le(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static void persist_gcm_counters() {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    return;
  }
  /* Reserve a block of IVs so a crash cannot reuse counters. Never call
   * elrs_config_save() from the hopping loop: it osDelay(50) + flash write. */
  uint32_t send_mark = g_send_counter + MLRS_GCM_CTR_RESERVE;
  if (send_mark < g_send_counter) {
    send_mark = 0xFFFFFFFFu;
  }
  put_u32_le(&cfg->reserved[ELRS_RESERVED_MLRS_GCM_SEND_OFFSET], send_mark);
  put_u32_le(&cfg->reserved[ELRS_RESERVED_MLRS_GCM_RECV_OFFSET], g_recv_highest);
  (void)elrs_config_save();
}

static void load_gcm_counters() {
  elrs_config_t *cfg = elrs_config_get();
  uint32_t send = 0;
  uint32_t recv = 0;
  if (cfg != nullptr) {
    send = u32_le(&cfg->reserved[ELRS_RESERVED_MLRS_GCM_SEND_OFFSET]);
    recv = u32_le(&cfg->reserved[ELRS_RESERVED_MLRS_GCM_RECV_OFFSET]);
  }
  g_send_counter = send + MLRS_GCM_CTR_BOOT_GAP;
  if (g_send_counter == 0) {
    g_send_counter = 1;
  }
  g_recv_highest = recv;
}

static bool replay_ok(uint32_t counter, bool allow_last) {
  if (allow_last) {
    if (counter < g_recv_highest) {
      return false;
    }
    if (counter > g_recv_highest) {
      g_recv_highest = counter;
    }
    return true;
  }
  if (counter <= g_recv_highest) {
    return false;
  }
  g_recv_highest = counter;
  return true;
}

static uint32_t next_send_counter() {
  const uint32_t c = g_send_counter++;
  if (g_send_counter == 0) {
    g_send_counter = 1;
  }
  return c;
}

static void load_or_make_secret() {
  memset(g_secret, 0, sizeof(g_secret));
  g_secret_ok = 0;
  if (elrs_config_get_mlrs_secret(g_secret) == 0) {
    g_secret_ok = 1;
    return;
  }
#if MLRS_OTA_IS_TX
  if (mlrs_gcm_random(g_secret, sizeof(g_secret)) &&
      elrs_config_set_mlrs_secret(g_secret) == 0) {
    (void)elrs_config_save();
    g_secret_ok = 1;
    printf("[mLRS] generated TRNG bind secret\n");
  }
#endif
}

static void derive_crypto_from_uid() {
  g_gcm_ready = mlrs_gcm_selftest() ? 1 : 0;
  load_or_make_secret();
  uint8_t zeros[MLRS_GCM_SECRET_LEN] = {};
  mlrs_gcm_derive(UID, g_secret_ok ? g_secret : zeros, &g_gcm_up, &g_gcm_dn);
}

#if MLRS_OTA_IS_TX
static uint16_t clip_rc(int32_t x) {
  if (x < 1) {
    return 1;
  }
  if (x > 2047) {
    return 2047;
  }
  return (uint16_t)x;
}

static uint16_t rc_from_crsf(uint16_t crsf_ch) {
  return clip_rc((((int32_t)crsf_ch - 992) * 2047) / 1966 + 1024);
}
#else
static uint16_t rc_to_crsf(uint16_t rc_ch) {
  return (uint16_t)((((int32_t)rc_ch - 1024) * 1920) / 2047 + 992);
}
#endif

static uint8_t rssi_u7_from_i8(int8_t rssi_i8) {
  if (rssi_i8 > -1) {
    return 1;
  }
  if (rssi_i8 < -127) {
    return 127;
  }
  return (uint8_t)(-rssi_i8);
}

static uint32_t fhss_prng_next(uint32_t *seed) {
  const uint32_t a = 214013;
  const uint32_t c = 2531011;
  const uint32_t m = 2147483648U;
  *seed = (a * (*seed) + c) % m;
  return *seed >> 16;
}

static uint8_t sanitize_band(uint8_t band) {
  if (band >= MLRS_BAND_COUNT) {
    return MLRS_BAND_915;
  }
  return band;
}

static uint8_t sanitize_rate(uint8_t rate) {
  /* EdgeTX native CRSF dialogs are 1-based; last option arrives as COUNT. */
  if (rate == MLRS_RATE_COUNT) {
    return (uint8_t)(MLRS_RATE_COUNT - 1U);
  }
  if (rate >= MLRS_RATE_COUNT) {
    return MLRS_RATE_31HZ;
  }
  return rate;
}

static void mavlinkx_sync_compression() {
  /* Stock mLRS enables X4 payload compression only in 19 Hz mode. */
  mlrs_mavlinkx_set_compression(sanitize_rate(g_rate) == MLRS_RATE_19HZ);
}

static uint8_t mavlinkx_fill_mux(uint8_t *payload, uint8_t payload_len,
                                 uint8_t max) {
  for (;;) {
    const uint8_t hdr = (payload_len == 0) ? 2 : 0;
    if ((uint16_t)payload_len + hdr + 1U + 2U > max) {
      break;
    }
    uint8_t tmp[64];
    tmp[0] = MLRS_AIR_MAVLINKX;
    uint8_t room = (uint8_t)(max - payload_len - hdr - 2U);
    if (room > 62U) {
      room = 62U;
    }
    const uint8_t n = mlrs_mavlinkx_take_air(tmp + 1, room);
    if (n == 0) {
      break;
    }
    const uint8_t next =
        mux_append(payload, payload_len, max, tmp, (uint8_t)(n + 1U));
    if (next == payload_len) {
      break;
    }
    payload_len = next;
  }
  return payload_len;
}

static uint8_t advertised_rate() {
  if (g_pending_rate != 0xFF) {
    return sanitize_rate(g_pending_rate);
  }
  return sanitize_rate(g_rate);
}

static uint8_t advertised_band() {
  if (g_pending_band != 0xFF) {
    return sanitize_band(g_pending_band);
  }
  return sanitize_band(g_band);
}

#if !MLRS_OTA_IS_TX
static uint8_t next_scan_rate(uint8_t rate) {
  return (uint8_t)((sanitize_rate(rate) + 1U) % MLRS_RATE_COUNT);
}
#endif

static const mlrs_rate_cfg_t *current_rate_cfg() {
  return &kRates[sanitize_band(g_band)][sanitize_rate(g_rate)];
}

#if MLRS_OTA_IS_TX
#define MLRS_DYN_LQ_BOOST_DIFF 20
#define MLRS_DYN_LQ_BOOST_MIN 50
#define MLRS_DYN_LQ_THRESH_UP 85
#define MLRS_DYN_LQ_THRESH_DN 95
#define MLRS_DYN_RSSI_CNT 5
#define MLRS_DYN_RSSI_THRESH_UP 15
#define MLRS_DYN_RSSI_THRESH_DN 21

static uint8_t g_dyn_lq_avg = 100;
static int16_t g_dyn_rssi_acc = 0;
static uint8_t g_dyn_rssi_n = 0;
static uint32_t g_dyn_last_tlm_ms = 0;
static uint8_t g_dyn_rate = 0xFF;

static int8_t rssi_i8_from_u7(uint8_t u7) {
  if (u7 == 0) {
    return -1;
  }
  return (int8_t)(-(int)u7);
}

static int8_t mlrs_rx_sensitivity() {
  const mlrs_rate_cfg_t *rate = current_rate_cfg();
  if (sanitize_band(g_band) == MLRS_BAND_24) {
    if (rate->is_fsk) {
      return -105;
    }
    if (sanitize_rate(g_rate) == 0) {
      return -105;
    }
    if (sanitize_rate(g_rate) == 1) {
      return -108;
    }
    return -112;
  }
  if (rate->is_fsk) {
    return -105;
  }
  if (sanitize_rate(g_rate) == 0) {
    return -112;
  }
  return -117;
}

extern bool isArmed;

static void mlrs_dynpower_set_config() {
  POWERMGNT::setPower((PowerLevels_e)config.GetPower());
}

static void mlrs_dynpower_reset() {
  g_dyn_lq_avg = 100;
  g_dyn_rssi_acc = 0;
  g_dyn_rssi_n = 0;
  g_dyn_last_tlm_ms = 0;
  g_dyn_rate = 0xFF;
}

static void mlrs_dynpower_begin() {
  mlrs_dynpower_reset();
  if (config.GetDynamicPower() && !isArmed) {
    POWERMGNT::setPower(MinPower);
  } else {
    mlrs_dynpower_set_config();
  }
}

static void mlrs_dynpower_end() { mlrs_dynpower_set_config(); }

static void mlrs_dynpower_on_event(bool tlm_ok, uint8_t rx_lq, int8_t rx_rssi) {
  const PowerLevels_e before = POWERMGNT::currPower();
  const char *why = nullptr;

  if (!config.GetDynamicPower()) {
    if (tlm_ok && rx_rssi <= -20 && before < (PowerLevels_e)config.GetPower()) {
      mlrs_dynpower_set_config();
      why = "restore";
    }
    if (why != nullptr && POWERMGNT::currPower() != before) {
      printf("[mLRS] dynpower %s -> %d dBm\n", why,
             (int)POWERMGNT::getPowerIndBm());
    }
    return;
  }

  const uint8_t boostChannel = config.GetBoostChannel();
  if ((connectionState == disconnected && isArmed) ||
      (boostChannel != 0 &&
       CRSF_to_BIT(ChannelData[AUX9 + boostChannel - 1]) == 0)) {
    mlrs_dynpower_set_config();
    if (POWERMGNT::currPower() != before) {
      printf("[mLRS] dynpower boost -> %d dBm\n",
             (int)POWERMGNT::getPowerIndBm());
    }
    return;
  }

  if (!isArmed) {
    if (before != MinPower) {
      POWERMGNT::setPower(MinPower);
      printf("[mLRS] dynpower disarm -> %d dBm\n",
             (int)POWERMGNT::getPowerIndBm());
    }
    return;
  }

  if (!tlm_ok) {
    if (before < (PowerLevels_e)config.GetPower() && g_dyn_last_tlm_ms != 0) {
      const uint32_t interval_ms = current_rate_cfg()->interval_us / 1000U;
      if ((millis() - g_dyn_last_tlm_ms) > (interval_ms + 8U)) {
        POWERMGNT::incPower();
        if (POWERMGNT::currPower() != before) {
          printf("[mLRS] dynpower tlm-miss -> %d dBm\n",
                 (int)POWERMGNT::getPowerIndBm());
        }
      }
    }
    return;
  }

  g_dyn_last_tlm_ms = millis();
  if (rx_rssi >= -5) {
    POWERMGNT::decPower();
    if (POWERMGNT::currPower() != before) {
      printf("[mLRS] dynpower overload -> %d dBm\n",
             (int)POWERMGNT::getPowerIndBm());
    }
  }

  if (g_dyn_rate != sanitize_rate(g_rate)) {
    g_dyn_rate = sanitize_rate(g_rate);
    g_dyn_lq_avg = rx_lq;
    g_dyn_rssi_n = 0;
    g_dyn_rssi_acc = 0;
  }

  const uint8_t lq_avg = g_dyn_lq_avg;
  const int32_t lq_diff = (int32_t)lq_avg - (int32_t)rx_lq;
  g_dyn_lq_avg = (uint8_t)(((uint16_t)g_dyn_lq_avg * 7U + rx_lq) / 8U);
  if (lq_diff >= MLRS_DYN_LQ_BOOST_DIFF || rx_lq <= MLRS_DYN_LQ_BOOST_MIN) {
    mlrs_dynpower_set_config();
    if (POWERMGNT::currPower() != before) {
      printf("[mLRS] dynpower lq-boost lq=%u -> %d dBm\n", (unsigned)rx_lq,
             (int)POWERMGNT::getPowerIndBm());
    }
    return;
  }

  const uint8_t configPower = (uint8_t)config.GetPower();
  const uint8_t currPower = (uint8_t)POWERMGNT::currPower();
  uint8_t headroom =
      (configPower > currPower) ? (uint8_t)(configPower - currPower) : 0;
  const PowerLevels_e start = POWERMGNT::currPower();
  const int8_t sens = mlrs_rx_sensitivity();
  g_dyn_rssi_acc = (int16_t)(g_dyn_rssi_acc + rx_rssi);
  ++g_dyn_rssi_n;
  if (g_dyn_rssi_n >= MLRS_DYN_RSSI_CNT) {
    const int8_t avg =
        (int8_t)(g_dyn_rssi_acc / (int16_t)g_dyn_rssi_n);
    g_dyn_rssi_acc = 0;
    g_dyn_rssi_n = 0;
    if ((avg < (int8_t)(sens + MLRS_DYN_RSSI_THRESH_UP)) && headroom > 0) {
      POWERMGNT::incPower();
      why = "rssi-up";
    } else if (avg > (int8_t)(sens + MLRS_DYN_RSSI_THRESH_DN) &&
               lq_avg >= MLRS_DYN_LQ_THRESH_DN) {
      POWERMGNT::decPower();
      why = "rssi-dn";
    }
  }

  headroom = ((uint8_t)config.GetPower() > (uint8_t)POWERMGNT::currPower())
                 ? (uint8_t)((uint8_t)config.GetPower() -
                             (uint8_t)POWERMGNT::currPower())
                 : 0;
  if (headroom > 0 && start == POWERMGNT::currPower() &&
      rx_lq <= MLRS_DYN_LQ_THRESH_UP) {
    POWERMGNT::incPower();
    why = "lq-up";
  }
  if (POWERMGNT::currPower() != before) {
    printf("[mLRS] dynpower %s rssi=%d lq=%u -> %d dBm\n",
           why != nullptr ? why : "adj", (int)rx_rssi, (unsigned)rx_lq,
           (int)POWERMGNT::getPowerIndBm());
  }
}
#endif

static bool tight_slot() {
  return current_rate_cfg()->interval_us <= 20000U;
}

#if MLRS_OTA_IS_TX
static uint32_t announce_ms() {
  const uint32_t slot_ms = current_rate_cfg()->interval_us / 1000U;
  uint32_t hold_ms =
      MLRS_LOST_MS + (uint32_t)g_fhss_count * slot_ms * 2U;
  if (hold_ms < MLRS_ANNOUNCE_MIN_MS) {
    hold_ms = MLRS_ANNOUNCE_MIN_MS;
  }
  return hold_ms;
}

static bool announcing() {
  return g_pending_protocol != 0xFF || g_pending_rate != 0xFF ||
         g_pending_band != 0xFF;
}
#endif

static void calib_image_for_freq(uint32_t freq) {
  uint32_t lo;
  uint32_t hi;
  if (freq >= 1000000000UL) {
    lo = 2400000000UL;
    hi = 2480000000UL;
  } else {
    lo = 900000000UL;
    hi = 931000000UL;
  }
  (void)lr1121_elrs_calib_image(lo, hi);
}

static void fhss_generate() {
  const uint8_t list_len =
      (g_band == MLRS_BAND_24) ? FHSS_LIST_24_LEN : kFhssListLen;
  const uint8_t bind_ch =
      (g_band == MLRS_BAND_24) ? FHSS_BIND_CH_24 : FHSS_BIND_CH_915;
  g_fhss_count = current_rate_cfg()->hop_count;
  if (g_fhss_count > FHSS_MAX_HOPS) {
    g_fhss_count = FHSS_MAX_HOPS;
  }
  uint32_t seed = g_fhss_seed;
  bool used[FHSS_LIST_24_LEN] = {};
  uint8_t k = 0;
  while (k < g_fhss_count) {
    uint8_t remaining = (uint8_t)(list_len - k);
    if (remaining == 0) {
      break;
    }
    uint8_t rn = (uint8_t)(fhss_prng_next(&seed) % remaining);
    uint8_t i = 0;
    uint8_t ch = 0;
    for (ch = 0; ch < list_len; ++ch) {
      if (used[ch]) {
        continue;
      }
      if (i == rn) {
        break;
      }
      ++i;
    }
    if (ch >= list_len) {
      ch = 0;
    }
    if (ch == bind_ch) {
      continue;
    }
    bool too_close = false;
    if (k > 0) {
      int8_t last = (int8_t)g_fhss_ch[k - 1];
      if (last == 0) {
        if (ch < 2) {
          too_close = true;
        }
      } else if ((ch >= last - 1) && (ch <= last + 1)) {
        too_close = true;
      }
    }
    if (too_close) {
      continue;
    }
    g_fhss_ch[k] = ch;
    if (g_band == MLRS_BAND_24) {
      g_fhss_list[k] = 2401000000UL + (uint32_t)ch * 1000000UL;
    } else {
      g_fhss_list[k] = kFhss915Hz[ch];
    }
    used[ch] = true;
    ++k;
  }
  if (k > 0) {
    g_fhss_count = k;
  }
  g_fhss_i = 0;
  g_hop_mask = 0;
  g_hop_gen = 0;
#if !MLRS_OTA_IS_TX
  memset(g_hop_stat, 0, sizeof(g_hop_stat));
  g_hop_proposed_mask = 0;
  g_hop_proposed_gen = 0;
#endif
#if MLRS_OTA_IS_TX
  g_fhss_need_hop = false;
#else
  g_fhss_follow = false;
  g_fhss_arm_rx = 0;
  g_fhss_do_hop = 0;
  g_fhss_pending_hops = 0;
  g_slot_rx = 0;
  g_rx_need_rearm = 0;
  g_miss_streak = 0;
#endif
}

static uint32_t fhss_curr() { return g_fhss_list[g_fhss_i]; }

static uint8_t hop_bitcount(uint32_t mask) {
  uint8_t n = 0;
  while (mask != 0) {
    mask &= mask - 1U;
    ++n;
  }
  return n;
}

static uint8_t hop_max_skip() {
  if (g_fhss_count < 8) {
    return 0;
  }
  uint8_t n = (uint8_t)(g_fhss_count / 4U);
  if (n > MLRS_HOP_MAX_SKIP) {
    n = MLRS_HOP_MAX_SKIP;
  }
  return n;
}

static uint32_t hop_sanitize_mask(uint32_t mask) {
  mask &= ~1U;
  if (g_fhss_count == 0) {
    return 0;
  }
  if (g_fhss_count < 32) {
    mask &= ((1UL << g_fhss_count) - 1UL);
  }
  const uint8_t max_skip = hop_max_skip();
  while (hop_bitcount(mask) > max_skip) {
    for (int i = (int)g_fhss_count - 1; i > 0; --i) {
      const uint32_t bit = 1UL << i;
      if ((mask & bit) != 0) {
        mask &= ~bit;
        break;
      }
    }
  }
  return mask;
}

static bool hop_is_skipped(uint8_t i) {
  if (i == 0 || i >= g_fhss_count) {
    return false;
  }
  return (g_hop_mask & (1UL << i)) != 0;
}

static uint8_t hop_probe_pick() {
  uint8_t skipped = 0;
  for (uint8_t i = 1; i < g_fhss_count; ++i) {
    if (hop_is_skipped(i)) {
      ++skipped;
    }
  }
  if (skipped == 0) {
    return 0xFF;
  }
#if MLRS_OTA_IS_TX
  const uint32_t ctr = (g_send_counter > 1) ? (g_send_counter - 1U) : 1U;
#else
  const uint32_t ctr = g_recv_highest;
#endif
  uint8_t n = (uint8_t)(ctr % skipped);
  for (uint8_t i = 1; i < g_fhss_count; ++i) {
    if (hop_is_skipped(i)) {
      if (n == 0) {
        return i;
      }
      --n;
    }
  }
  return 0xFF;
}

static void hopmask_apply(uint8_t gen, uint32_t mask) {
  mask = hop_sanitize_mask(mask);
  if (mask == g_hop_mask && gen == g_hop_gen) {
    return;
  }
  g_hop_mask = mask;
  g_hop_gen = gen;
  printf("[mLRS] hopmask gen=%u skip=%u/%u bits=0x%08lx\n", (unsigned)gen,
         (unsigned)hop_bitcount(mask), (unsigned)g_fhss_count,
         (unsigned long)mask);
}

static uint8_t hopmask_pack(uint8_t *dst, uint8_t gen, uint32_t mask) {
  dst[0] = MLRS_AIR_HOPMASK;
  dst[1] = gen;
  dst[2] = (uint8_t)mask;
  dst[3] = (uint8_t)(mask >> 8);
  dst[4] = (uint8_t)(mask >> 16);
  dst[5] = (uint8_t)(mask >> 24);
  return 6;
}

static bool hopmask_parse(const uint8_t *f, uint8_t n, uint8_t *gen,
                          uint32_t *mask) {
  if (f == nullptr || n < 6 || f[0] != MLRS_AIR_HOPMASK || gen == nullptr ||
      mask == nullptr) {
    return false;
  }
  *gen = f[1];
  *mask = (uint32_t)f[2] | ((uint32_t)f[3] << 8) | ((uint32_t)f[4] << 16) |
          ((uint32_t)f[5] << 24);
  return true;
}

static void fhss_hop() {
  if (g_fhss_count == 0) {
    return;
  }
  if (hop_is_skipped(g_fhss_i)) {
    uint8_t i = 0;
    for (uint8_t n = 0; n < g_fhss_count; ++n) {
      i = (uint8_t)((i + 1) % g_fhss_count);
      if (!hop_is_skipped(i)) {
        g_fhss_i = i;
        return;
      }
    }
    g_fhss_i = 0;
    return;
  }
  if (g_fhss_i == 0) {
    const uint8_t probe = hop_probe_pick();
    if (probe < g_fhss_count) {
      g_fhss_i = probe;
      return;
    }
  }
  uint8_t i = g_fhss_i;
  for (uint8_t n = 0; n < g_fhss_count; ++n) {
    i = (uint8_t)((i + 1) % g_fhss_count);
    if (!hop_is_skipped(i)) {
      g_fhss_i = i;
      return;
    }
  }
  g_fhss_i = 0;
}

#if !MLRS_OTA_IS_TX
static void fhss_set_index(uint8_t index) {
  if (index < g_fhss_count) {
    g_fhss_i = index;
  }
}

static void hop_note(uint8_t idx, bool ok) {
  if (idx >= g_fhss_count || idx >= FHSS_MAX_HOPS) {
    return;
  }
  hop_stat_t *s = &g_hop_stat[idx];
  if (s->vis < 254) {
    ++s->vis;
  }
  if (!ok && s->fail < 254) {
    ++s->fail;
  }
  if (ok && (g_hop_proposed_mask & (1UL << idx)) != 0) {
    if (s->probe_ok < 4) {
      ++s->probe_ok;
    }
  } else if (!ok) {
    s->probe_ok = 0;
  }
}

static void hop_rebuild_mask() {
  if (!g_connected || g_fhss_count < 8) {
    return;
  }
  uint32_t mask = g_hop_proposed_mask;
  const uint8_t max_skip = hop_max_skip();
  for (uint8_t i = 1; i < g_fhss_count; ++i) {
    hop_stat_t *s = &g_hop_stat[i];
    if (s->vis >= MLRS_HOP_WIN) {
      s->vis = (uint8_t)(s->vis / 2);
      s->fail = (uint8_t)(s->fail / 2);
    }
    if (s->vis < 8) {
      continue;
    }
    const uint16_t fail_pct = (uint16_t)((uint16_t)s->fail * 100U / s->vis);
    if ((mask & (1UL << i)) != 0) {
      if (s->probe_ok >= MLRS_HOP_CLEAR_OK) {
        mask &= ~(1UL << i);
        s->probe_ok = 0;
        s->vis = 0;
        s->fail = 0;
      }
    } else if (fail_pct >= MLRS_HOP_FAIL_PCT &&
               hop_bitcount(mask) < max_skip) {
      mask |= (1UL << i);
      s->probe_ok = 0;
    }
  }
  mask = hop_sanitize_mask(mask);
  if (mask == g_hop_proposed_mask) {
    return;
  }
  g_hop_proposed_mask = mask;
  g_hop_proposed_gen = (uint8_t)(g_hop_proposed_gen + 1U);
  if (g_hop_proposed_gen == 0) {
    g_hop_proposed_gen = 1;
  }
  printf("[mLRS] rx hopmask gen=%u skip=%u/%u bits=0x%08lx\n",
         (unsigned)g_hop_proposed_gen, (unsigned)hop_bitcount(mask),
         (unsigned)g_fhss_count, (unsigned long)mask);
}
#endif

static void derive_sync_from_uid() {
  uint32_t bind_dword = ((uint32_t)UID[0]) | ((uint32_t)UID[1] << 8) |
                        ((uint32_t)UID[2] << 16) | ((uint32_t)UID[3] << 24);
  g_sync_word = crc_calculate((const uint8_t *)&bind_dword, 4);
  g_fhss_seed = g_sync_word;
}

static uint8_t config_protocol() {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    return ELRS_AIR_PROTOCOL_ELRS;
  }
  uint8_t proto = cfg->reserved[ELRS_RESERVED_AIR_PROTOCOL_OFFSET];
  if (proto > ELRS_AIR_PROTOCOL_MLRS) {
    proto = ELRS_AIR_PROTOCOL_ELRS;
  }
  return proto;
}

static void save_protocol(uint8_t protocol) {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    return;
  }
  /* Do not persist mLRS across reboot until the overlay is stable. */
  (void)protocol;
  cfg->reserved[ELRS_RESERVED_AIR_PROTOCOL_OFFSET] = ELRS_AIR_PROTOCOL_ELRS;
  g_config_dirty = 1;
}

static uint8_t config_rate() {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    return 0;
  }
  return sanitize_rate(cfg->reserved[ELRS_RESERVED_MLRS_RATE_OFFSET]);
}

static void save_rate(uint8_t rate) {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    return;
  }
  cfg->reserved[ELRS_RESERVED_MLRS_RATE_OFFSET] = sanitize_rate(rate);
  g_config_dirty = 1;
}

static uint8_t config_band() {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    return MLRS_BAND_915;
  }
  return sanitize_band(cfg->reserved[ELRS_RESERVED_MLRS_BAND_OFFSET]);
}

static void save_band(uint8_t band) {
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    return;
  }
  cfg->reserved[ELRS_RESERVED_MLRS_BAND_OFFSET] = sanitize_band(band);
  g_config_dirty = 1;
}

static void flush_mlrs_config() {
  if (g_config_dirty == 0 || g_active) {
    return;
  }
  g_config_dirty = 0;
  (void)elrs_config_save();
}

static uint8_t remap_rate_for_band(uint8_t from_band, uint8_t to_band,
                                   uint8_t rate) {
  rate = sanitize_rate(rate);
  from_band = sanitize_band(from_band);
  to_band = sanitize_band(to_band);
  if (from_band == to_band) {
    return rate;
  }
  if (from_band == MLRS_BAND_915 && to_band == MLRS_BAND_24) {
    if (rate == 0) {
      return 1;
    }
    if (rate == 1) {
      return 2;
    }
    return 0;
  }
  if (rate == 0) {
    return 2;
  }
  if (rate == 1) {
    return 0;
  }
  return 1;
}

#if MLRS_OTA_IS_TX
static void pack_tx_frame(tTxFrame *frame, const uint16_t rc[16],
                          uint8_t fhss_index, uint8_t seq, uint8_t lq, bool ack,
                          const uint8_t *payload, uint8_t payload_len) {
  memset(frame, 0, sizeof(*frame));
  if (payload_len > FRAME_TX_PAYLOAD_LEN) {
    payload_len = FRAME_TX_PAYLOAD_LEN;
  }
  frame->sync_word = g_sync_word;
  frame->pkt_counter = next_send_counter();
  frame->status.seq_no = seq & 0x7;
  frame->status.ack = ack ? 1 : 0;
  frame->status.frame_type = (g_pending_protocol == ELRS_AIR_PROTOCOL_ELRS)
                                 ? FRAME_TYPE_TX_RX_CMD
                                 : FRAME_TYPE_TX;
  frame->status.antenna = 0;
  frame->status.transmit_antenna = 0;
  frame->status.rssi_u7 = rssi_u7_from_i8(Radio.LastPacketRSSI);
  frame->status.fhss_index_band = advertised_band() & 0x1;
  frame->status.fhss_index = fhss_index & 0x3F;
  frame->status.LQ_serial = lq;
  frame->status.spare = advertised_rate() & 0x3;
  frame->status.payload_len = payload_len;
  frame->rc1.ch0 = rc[0];
  frame->rc1.ch1 = rc[1];
  frame->rc1.ch2 = rc[2];
  frame->rc1.ch3 = rc[3];
  frame->rc2.ch4 = rc[4];
  frame->rc2.ch5 = rc[5];
  frame->rc2.ch6 = rc[6];
  frame->rc2.ch7 = rc[7];
  frame->rc2.ch8 = (uint8_t)(rc[8] / 8);
  frame->rc2.ch9 = (uint8_t)(rc[9] / 8);
  frame->rc2.ch10 = (uint8_t)(rc[10] / 8);
  frame->rc2.ch11 = (uint8_t)(rc[11] / 8);
  frame->rc1.ch12 = (rc[12] >= 1536) ? 2 : ((rc[12] <= 512) ? 0 : 1);
  frame->rc1.ch13 = (rc[13] >= 1536) ? 2 : ((rc[13] <= 512) ? 0 : 1);
  frame->rc2.ch14 = (rc[14] >= 1536) ? 2 : ((rc[14] <= 512) ? 0 : 1);
  frame->rc2.ch15 = (rc[15] >= 1536) ? 2 : ((rc[15] <= 512) ? 0 : 1);
  if (payload != nullptr && payload_len > 0) {
    memcpy(frame->payload, payload, payload_len);
  }
  g_last_ul_plen = payload_len;
  uint16_t crc;
  crc_init(&crc);
  crc_accumulate_buf(&crc, (uint8_t *)frame, 11);
  frame->crc1 = crc;
  uint8_t iv[MLRS_GCM_IV_LEN];
  mlrs_gcm_make_iv(&g_gcm_up, MLRS_GCM_DIR_UPLINK, frame->pkt_counter, iv);
  mlrs_gcm_seal(&g_gcm_up, iv, (uint8_t *)frame, FRAME_TX_AAD_LEN,
                (uint8_t *)&frame->rc1, FRAME_TX_SEAL_LEN, frame->tag,
                FRAME_GCM_TAG_LEN);
  crc_init(&crc);
  crc_accumulate_buf(&crc, (uint8_t *)frame, FRAME_TX_RX_LEN - 2);
  frame->crc = crc;
}
#else
static uint8_t check_tx_crc(const tTxFrame *frame) {
  if (frame->sync_word != g_sync_word) {
    return 1;
  }
  if ((frame->status.frame_type != FRAME_TYPE_TX) &&
      (frame->status.frame_type != FRAME_TYPE_TX_RX_CMD)) {
    return 2;
  }
  uint16_t crc;
  crc_init(&crc);
  crc_accumulate_buf(&crc, (const uint8_t *)frame, 11);
  if (crc != frame->crc1) {
    return 3;
  }
  crc_init(&crc);
  crc_accumulate_buf(&crc, (const uint8_t *)frame, FRAME_TX_RX_LEN - 2);
  if (crc != frame->crc) {
    return 4;
  }
  return 0;
}

static uint8_t open_tx_gcm(tTxFrame *frame) {
  uint8_t iv[MLRS_GCM_IV_LEN];
  mlrs_gcm_make_iv(&g_gcm_up, MLRS_GCM_DIR_UPLINK, frame->pkt_counter, iv);
  if (!mlrs_gcm_open(&g_gcm_up, iv, (const uint8_t *)frame, FRAME_TX_AAD_LEN,
                     (uint8_t *)&frame->rc1, FRAME_TX_SEAL_LEN, frame->tag,
                     FRAME_GCM_TAG_LEN)) {
    return 5;
  }
  if (!replay_ok(frame->pkt_counter, false)) {
    return 6;
  }
  return 0;
}

static void unpack_tx_rc(const tTxFrame *frame, uint16_t rc[16]) {
  rc[0] = frame->rc1.ch0;
  rc[1] = frame->rc1.ch1;
  rc[2] = frame->rc1.ch2;
  rc[3] = frame->rc1.ch3;
  rc[4] = frame->rc2.ch4;
  rc[5] = frame->rc2.ch5;
  rc[6] = frame->rc2.ch6;
  rc[7] = frame->rc2.ch7;
  rc[8] = (uint16_t)(frame->rc2.ch8 * 8);
  rc[9] = (uint16_t)(frame->rc2.ch9 * 8);
  rc[10] = (uint16_t)(frame->rc2.ch10 * 8);
  rc[11] = (uint16_t)(frame->rc2.ch11 * 8);
  rc[12] = (frame->rc1.ch12 > 1) ? 2047 : ((frame->rc1.ch12 < 1) ? 0 : 1024);
  rc[13] = (frame->rc1.ch13 > 1) ? 2047 : ((frame->rc1.ch13 < 1) ? 0 : 1024);
  rc[14] = (frame->rc2.ch14 > 1) ? 2047 : ((frame->rc2.ch14 < 1) ? 0 : 1024);
  rc[15] = (frame->rc2.ch15 > 1) ? 2047 : ((frame->rc2.ch15 < 1) ? 0 : 1024);
}

static void pack_rx_frame(tRxFrame *frame, uint8_t seq, uint8_t lq, bool ack,
                          const uint8_t *payload, uint8_t payload_len) {
  memset(frame, 0, sizeof(*frame));
  if (payload_len > FRAME_RX_PAYLOAD_LEN) {
    payload_len = FRAME_RX_PAYLOAD_LEN;
  }
  frame->sync_word = g_sync_word;
  frame->pkt_counter = next_send_counter();
  frame->status.seq_no = seq & 0x7;
  frame->status.ack = ack ? 1 : 0;
  frame->status.frame_type = FRAME_TYPE_RX;
  frame->status.antenna = 0;
  frame->status.transmit_antenna = 0;
  frame->status.rssi_u7 = rssi_u7_from_i8(Radio.LastPacketRSSI);
  frame->status.LQ_rc = lq;
  frame->status.LQ_serial = lq;
  frame->status.payload_len = payload_len;
  if (payload != nullptr && payload_len > 0) {
    memcpy(frame->payload, payload, payload_len);
  }
  g_last_dl_plen = payload_len;
  uint8_t iv[MLRS_GCM_IV_LEN];
  mlrs_gcm_make_iv(&g_gcm_dn, MLRS_GCM_DIR_DOWNLINK, frame->pkt_counter, iv);
  mlrs_gcm_seal(&g_gcm_dn, iv, (uint8_t *)frame, FRAME_RX_AAD_LEN, frame->payload,
                FRAME_RX_PAYLOAD_LEN, frame->tag, FRAME_GCM_TAG_LEN);
  uint16_t crc;
  crc_init(&crc);
  crc_accumulate_buf(&crc, (uint8_t *)frame, FRAME_TX_RX_LEN - 2);
  frame->crc = crc;
}

static void refresh_rx_frame(tRxFrame *frame, uint8_t lq) {
  frame->pkt_counter = next_send_counter();
  frame->status.rssi_u7 = rssi_u7_from_i8(Radio.LastPacketRSSI);
  frame->status.LQ_rc = lq;
  frame->status.LQ_serial = lq;
  uint8_t iv[MLRS_GCM_IV_LEN];
  mlrs_gcm_make_iv(&g_gcm_dn, MLRS_GCM_DIR_DOWNLINK, frame->pkt_counter, iv);
  mlrs_gcm_seal(&g_gcm_dn, iv, (uint8_t *)frame, FRAME_RX_AAD_LEN, frame->payload,
                FRAME_RX_PAYLOAD_LEN, frame->tag, FRAME_GCM_TAG_LEN);
  uint16_t crc;
  crc_init(&crc);
  crc_accumulate_buf(&crc, (uint8_t *)frame, FRAME_TX_RX_LEN - 2);
  frame->crc = crc;
}
#endif

#if MLRS_OTA_IS_TX
static uint8_t check_rx_crc(const tRxFrame *frame) {
  if (frame->sync_word != g_sync_word) {
    return 1;
  }
  if (frame->status.frame_type != FRAME_TYPE_RX) {
    return 2;
  }
  uint16_t crc;
  crc_init(&crc);
  crc_accumulate_buf(&crc, (const uint8_t *)frame, FRAME_TX_RX_LEN - 2);
  if (crc != frame->crc) {
    return 4;
  }
  return 0;
}

static uint8_t open_rx_gcm(tRxFrame *frame) {
  uint8_t iv[MLRS_GCM_IV_LEN];
  mlrs_gcm_make_iv(&g_gcm_dn, MLRS_GCM_DIR_DOWNLINK, frame->pkt_counter, iv);
  if (!mlrs_gcm_open(&g_gcm_dn, iv, (const uint8_t *)frame, FRAME_RX_AAD_LEN,
                     frame->payload, FRAME_RX_PAYLOAD_LEN, frame->tag,
                     FRAME_GCM_TAG_LEN)) {
    return 5;
  }
  if (!replay_ok(frame->pkt_counter, true)) {
    return 6;
  }
  return 0;
}
#endif

static void configure_mlrs_radio() {
  const mlrs_rate_cfg_t *rate = current_rate_cfg();
  const uint32_t freq = fhss_curr();
  const uint8_t sync1 = (uint8_t)(g_sync_word >> 8);
  const uint8_t sync2 = (uint8_t)(g_sync_word & 0xFF);
  if (rate->is_fsk) {
    delay(5);
  }
#if defined(SIW917_ELRS_USE_UPSTREAM_TX_MAIN) || defined(SIW917_ELRS_TARGET_TX)
  RadioBandMod::Combined modulation = RadioBandMod::Combined::LORA_900;
  if (rate->is_fsk) {
    modulation = RadioBandMod::Combined::GFSK_900;
  } else if (g_band == MLRS_BAND_24) {
    modulation = RadioBandMod::Combined::LORA_2G4;
  }
  Radio.Config(rate->bw, rate->sf, rate->cr, freq, rate->preamble, false,
               FRAME_TX_RX_LEN, modulation, sync1, sync2, SX12XX_Radio_All);
#else
  Radio.Config(rate->bw, rate->sf, rate->cr, freq, rate->preamble, false,
               FRAME_TX_RX_LEN, rate->is_fsk != 0, sync1, sync2,
               SX12XX_Radio_All);
#endif
  calib_image_for_freq(freq);
  Radio.SetFrequencyReg(freq, SX12XX_Radio_All, false, 0);
  if (rate->is_fsk) {
    delay(5);
    printf("[mLRS] GFSK 100kbps BW=312k fdev=50k plen=%u\n",
           (unsigned)FRAME_TX_RX_LEN);
  } else if (g_band == MLRS_BAND_24) {
    printf("[mLRS] 2.4G LoRa BW=800k SF=%u CR=LI4/5 plen=%u freq=%lu\n",
           (unsigned)rate->sf, (unsigned)FRAME_TX_RX_LEN,
           (unsigned long)freq);
  }
}

static void apply_mlrs_rate() {
  if (!g_active) {
    return;
  }
  hwTimer::stop();
  Radio.SetTxIdleMode();
  delay(5);
  fhss_generate();
  configure_mlrs_radio();
  hwTimer::updateInterval(current_rate_cfg()->interval_us);
  hwTimer::resume();
#if !MLRS_OTA_IS_TX
  g_valid_window = 0;
  g_lq = 0;
  g_connected = 0;
  g_miss_streak = 0;
  g_fhss_pending_hops = 0;
  Radio.RXnb();
#endif
  printf("[mLRS] %s / %s (%u us, %u hops%s)\n",
         g_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz", current_rate_cfg()->name,
         (unsigned)current_rate_cfg()->interval_us, (unsigned)g_fhss_count,
         tight_slot() ? ", tlm/2" : "");
  mavlinkx_sync_compression();
  printf("[mLRS] mavlinkx compress=%s\n",
         sanitize_rate(g_rate) == MLRS_RATE_19HZ ? "on" : "off");
}

static void restore_elrs_radio() {
  const expresslrs_mod_settings_s *mod = ExpressLRS_currAirRate_Modparams;
  if (mod == nullptr) {
    return;
  }
  const uint32_t freq = FHSSgetInitialFreq();
  // Match ELRS SetRFLinkRate: invertIQ from UID[5], sync bytes UID[5], UID[4].
  // Using UID[4], UID[5] here made a restored TX unhearable to a fresh RX.
  const bool invertIQ = (UID[5] & 0x01) != 0;
#if defined(SIW917_ELRS_USE_UPSTREAM_TX_MAIN) || defined(SIW917_ELRS_TARGET_TX)
  Radio.Config(mod->bw, mod->sf, mod->cr, freq, mod->PreambleLen, invertIQ,
               mod->PayloadLength,
               static_cast<RadioBandMod::Combined>(mod->radio_type),
               (uint8_t)UID[5], (uint8_t)UID[4], SX12XX_Radio_All);
#else
  const bool fsk = (mod->radio_type == RADIO_TYPE_LR1121_GFSK_900) ||
                   (mod->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
  Radio.Config(mod->bw, mod->sf, mod->cr, freq, mod->PreambleLen, invertIQ,
               mod->PayloadLength, fsk, (uint8_t)UID[5], (uint8_t)UID[4],
               SX12XX_Radio_All);
#endif
  calib_image_for_freq(freq);
  Radio.SetFrequencyReg(freq, SX12XX_Radio_All, false, 0);
}

static bool mlrs_rx_age_lost(uint32_t then_ms) {
  if (then_ms == 0) {
    return false;
  }
  return (int32_t)(millis() - then_ms) > (int32_t)MLRS_LOST_MS;
}

static void mlrs_declare_lost() {
  if (!g_connected) {
    return;
  }
  g_connected = 0;
  setConnectionState(disconnected);
  g_print_disconnected = 1;
#if MLRS_OTA_IS_TX
  rarq_disconnected();
  siw917_tx_note_mlrs_link_lost();
#else
  tarq_disconnected();
#endif
}

static void note_valid_rx() {
  g_last_rx_ms = millis();
  if (g_valid_window < 250) {
    ++g_valid_window;
  }
  g_lq = (uint8_t)((g_valid_window > 100) ? 100 : g_valid_window);
#if MLRS_OTA_IS_TX
  siw917_tx_note_mlrs_downlink(
      g_lq, Radio.LastPacketRSSI, (int8_t)SNR_DESCALE(Radio.LastPacketSNRRaw));
#else
  uplinkLQ = g_lq;
  linkStats.uplink_Link_quality = g_lq;
  linkStats.uplink_RSSI_1 = Radio.LastPacketRSSI;
  linkStats.uplink_SNR = (int8_t)SNR_DESCALE(Radio.LastPacketSNRRaw);
  connectionHasModelMatch = true;
  g_rate_scan_ms = g_last_rx_ms;
#endif
  if (!g_connected && g_valid_window >= 3) {
    g_connected = 1;
    setConnectionState(connected);
    g_print_connected = 1;
#if !MLRS_OTA_IS_TX
    g_had_link = true;
#endif
  } else if (g_connected) {
    connectionState = connected;
  }
}

static void note_missed_rx() {
  if (g_valid_window > 0) {
    --g_valid_window;
  }
  g_lq = (uint8_t)((g_valid_window > 100) ? 100 : g_valid_window);
#if MLRS_OTA_IS_TX
  mlrs_dynpower_on_event(false, g_lq, 0);
#endif
  /* LQ only. Declaring lost from the timer tock races millis() against
   * the last CRC and flaps connected while the RF path is still fine. */
}

#if MLRS_OTA_IS_TX
static void fill_rc_from_handset(uint16_t rc[16]) {
  for (uint8_t i = 0; i < 16; ++i) {
    rc[i] = rc_from_crsf((uint16_t)ChannelData[i]);
  }
}

static void mlrs_tx_send_frame() {
  if (announcing()) {
    g_fhss_i = 0;
    g_fhss_need_hop = false;
  } else if (g_fhss_need_hop) {
    fhss_hop();
    g_fhss_need_hop = false;
  }
  const uint32_t interval_ms = current_rate_cfg()->interval_us / 1000U;
  /* FSK50 / 2.4 50Hz send tlm every other slot. Scoring the skip
   * slot as a miss pins TX LQ at 50 with a perfect uplink. */
  uint32_t miss_ms = interval_ms + 8U;
  if (tight_slot()) {
    miss_ms = (interval_ms * 2U) + 8U;
  }
  if (g_last_rx_ms != 0 && (millis() - g_last_rx_ms) > miss_ms) {
    note_missed_rx();
    rarq_missed();
  }
  uint16_t rc[16];
  fill_rc_from_handset(rc);
  uint8_t payload[FRAME_TX_PAYLOAD_LEN] = {};
  uint8_t payload_len = 0;
  if (g_pending_protocol == ELRS_AIR_PROTOCOL_ELRS) {
    payload[0] = MLRS_SWITCH_CMD;
    payload_len = 1;
  } else if (g_pending_band != 0xFF) {
    payload[0] = MLRS_BAND_CMD;
    payload[1] = sanitize_band(g_pending_band);
    payload[2] = sanitize_rate(g_pending_rate != 0xFF ? g_pending_rate : g_rate);
    payload_len = 3;
  } else if (g_pending_rate != 0xFF) {
    payload[0] = MLRS_RATE_CMD;
    payload[1] = sanitize_rate(g_pending_rate);
    payload_len = 2;
  } else {
    {
      uint8_t hm[6];
      hopmask_pack(hm, g_hop_gen, g_hop_mask);
      payload_len =
          mux_append(payload, payload_len, FRAME_TX_PAYLOAD_LEN, hm, 6);
    }
    for (;;) {
      uint8_t tmp[64];
      uint8_t n = g_ul_hold_len;
      if (n != 0) {
        memcpy(tmp, g_ul_hold, n);
        g_ul_hold_len = 0;
      } else if (mlrs_mbridge_take_air(
                     tmp, &n, (uint8_t)(FRAME_TX_PAYLOAD_LEN - 3U))) {
        /* native mLRS Lua / MBridge, ahead of leftover ELRS CRSF */
      } else if (!otaConnector.takeQueuedPayload(
                     tmp, &n, (uint8_t)(FRAME_TX_PAYLOAD_LEN - 3U)) ||
                 n == 0) {
        break;
      }
      const uint8_t next =
          mux_append(payload, payload_len, FRAME_TX_PAYLOAD_LEN, tmp, n);
      if (next == payload_len) {
        memcpy(g_ul_hold, tmp, n);
        g_ul_hold_len = n;
        break;
      }
      payload_len = next;
    }
    {
      uint8_t mav[64];
      for (;;) {
        const uint8_t n =
            (uint8_t)siw917_mavlink_wifi_uplink_read_bytes(mav, sizeof(mav));
        if (n == 0) {
          break;
        }
        mlrs_mavlinkx_ingest_mav(mav, n);
      }
    }
    payload_len =
        mavlinkx_fill_mux(payload, payload_len, FRAME_TX_PAYLOAD_LEN);
  }
  pack_tx_frame(&g_tx_frame, rc, g_fhss_i, g_seq++, g_lq,
                (g_rarq_ack & 1U) != 0U, payload, payload_len);
  Radio.SetFrequencyReg(fhss_curr(), SX12XX_Radio_All, false, 0);
  Radio.TXnb((uint8_t *)&g_tx_frame, false, nullptr, SX12XX_Radio_All);
  ++g_tx_sent;
  g_fhss_need_hop = true;
  if (g_pending_protocol == ELRS_AIR_PROTOCOL_ELRS && !g_logged_switch_cmd) {
    g_logged_switch_cmd = true;
    printf("[mLRS] tx switch cmd hop=%u plen=%u\n", (unsigned)g_fhss_i,
           (unsigned)payload_len);
  }
  if (g_tx_sent == 1) {
    printf("[mLRS] first TX hop=%u freq=%lu Hz plen=%u\n", (unsigned)g_fhss_i,
           (unsigned long)fhss_curr(), (unsigned)FRAME_TX_RX_LEN);
  }
}

static void mlrs_tx_tock() { g_tx_send_pending = 1; }

static void mlrs_tx_done() { Radio.RXnb(); }

static bool mlrs_tx_rx_done(SX12xxDriverCommon::rx_status) {
  memcpy(&g_rx_frame, Radio.RXdataBuffer, sizeof(g_rx_frame));
  const uint8_t fail = check_rx_crc(&g_rx_frame);
  if (fail != 0) {
    g_last_rx_fail = fail;
    if (fail == 1) {
      ++g_rx_junk;
    } else {
      ++g_rx_fail;
      note_missed_rx();
      rarq_missed();
    }
    return true;
  }
  g_downlink_pending = 1;
  return true;
}

static void mlrs_tx_process_downlink() {
  const uint8_t fail = open_rx_gcm(&g_rx_frame);
  if (fail != 0) {
    g_last_rx_fail = fail;
    ++g_rx_fail;
    note_missed_rx();
    rarq_missed();
    return;
  }
  ++g_rx_ok;
  note_valid_rx();
  rarq_received((uint8_t)g_rx_frame.status.seq_no);
  if (g_rarq_lost) {
    mlrs_mavlinkx_air_lost();
  }
  g_last_dl_plen = (uint8_t)g_rx_frame.status.payload_len;
  mlrs_dynpower_on_event(true, (uint8_t)g_rx_frame.status.LQ_rc,
                         rssi_i8_from_u7((uint8_t)g_rx_frame.status.rssi_u7));
  if (g_rarq_accept && g_rx_frame.status.payload_len > 0) {
    const uint8_t *p = g_rx_frame.payload;
    uint8_t len = (uint8_t)g_rx_frame.status.payload_len;
    auto deliver = [](const uint8_t *f, uint8_t n) {
      if (f == nullptr || n == 0) {
        return;
      }
      if (f[0] == MLRS_AIR_HOPMASK) {
        uint8_t gen = 0;
        uint32_t mask = 0;
        if (hopmask_parse(f, n, &gen, &mask)) {
          hopmask_apply(gen, mask);
        }
      } else if (f[0] == MLRS_AIR_MBRIDGE) {
        mlrs_mbridge_accept_downlink(f, n);
      } else if (f[0] == MLRS_AIR_MAVLINKX) {
        mlrs_mavlinkx_ingest_air(f + 1, (uint8_t)(n - 1U));
        uint8_t mav[64];
        uint8_t k;
        while ((k = mlrs_mavlinkx_take_mav(mav, sizeof(mav))) != 0) {
          (void)siw917_mavlink_wifi_enqueue_downlink(mav, k);
        }
      } else if (f[0] == 0xFD || f[0] == 0xFE) {
        (void)siw917_mavlink_wifi_enqueue_downlink(f, n);
      } else {
        uint8_t tmp[64];
        if (n > sizeof(tmp)) {
          n = sizeof(tmp);
        }
        memcpy(tmp, f, n);
        crsfRouter.processMessage(&otaConnector, (crsf_header_t *)tmp);
      }
    };
    if (len >= 2 && p[0] == MLRS_MUX_MAGIC) {
      uint8_t n = p[1];
      uint8_t i = 2;
      while (n-- != 0 && i < len) {
        const uint8_t fl = p[i++];
        if (fl == 0 || (uint16_t)i + fl > len) {
          break;
        }
        deliver(p + i, fl);
        i = (uint8_t)(i + fl);
      }
    } else {
      deliver(p, len);
    }
  }
}

static void notify_rx_protocol(uint8_t protocol) {
  static uint8_t payload[4 + MLRS_GCM_SECRET_LEN];
  memset(payload, 0, sizeof(payload));
  payload[0] = MSP_ELRS_SET_AIR_PROTOCOL;
  payload[1] = protocol;
  payload[2] = advertised_rate();
  payload[3] = sanitize_band(g_band);
  uint8_t n = 4;
  if (protocol == ELRS_AIR_PROTOCOL_MLRS) {
    load_or_make_secret();
    if (g_secret_ok) {
      memcpy(payload + 4, g_secret, MLRS_GCM_SECRET_LEN);
      n = (uint8_t)(4 + MLRS_GCM_SECRET_LEN);
    }
  }
  DataUlSender.SetDataToTransmit(payload, n);
}
#else
static void apply_rc_to_elrs(const uint16_t rc[16]) {
  for (uint8_t i = 0; i < 16; ++i) {
    ChannelData[i] = rc_to_crsf(rc[i]);
  }
}

static void mlrs_rx_hop_listen() {
  if (g_fhss_follow) {
    fhss_hop();
  }
  Radio.SetFrequencyReg(fhss_curr(), SX12XX_Radio_All, true, 0);
}

static uint8_t mlrs_rx_pack_state(uint8_t *dst, uint8_t max) {
  const uint8_t nvals = (uint8_t)(MLRS_P_COUNT - MLRS_P_RX_PROTOCOL);
  const uint8_t n = (uint8_t)(2 + nvals);
  if (dst == nullptr || max < n) {
    return 0;
  }
  dst[0] = MLRS_AIR_MBRIDGE;
  dst[1] = MLRS_AIR_RX_STATE;
  memset(dst + 2, 0, nvals);
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    return n;
  }
  const uint8_t proto =
      elrs_serial_protocol_to_lua_selection(cfg->serial_protocol);
  dst[2 + (MLRS_P_RX_PROTOCOL - MLRS_P_RX_PROTOCOL)] = proto;
  dst[2 + (MLRS_P_RX_ACTIVE - MLRS_P_RX_PROTOCOL)] = proto;
  dst[2 + (MLRS_P_RX_SBUS_FS - MLRS_P_RX_PROTOCOL)] =
      (cfg->failsafe_mode > 1) ? 1 : cfg->failsafe_mode;
  dst[2 + (MLRS_P_RX_TGT_SYS - MLRS_P_RX_PROTOCOL)] =
      cfg->mavlink_target_sys_id;
  dst[2 + (MLRS_P_RX_SRC_SYS - MLRS_P_RX_PROTOCOL)] =
      cfg->mavlink_source_sys_id;
  dst[2 + (MLRS_P_RX_TLM_OFF - MLRS_P_RX_PROTOCOL)] = cfg->force_tlm ? 1 : 0;
  {
    uint8_t pwr = 4;
    if (cfg->tx_power != ELRS_TX_POWER_MATCH_TX_DBM) {
      static const int8_t kDbm[] = {10, 14, 17, 20};
      for (uint8_t i = 0; i < 4; ++i) {
        if (kDbm[i] == cfg->tx_power) {
          pwr = i;
          break;
        }
      }
    }
    dst[2 + (MLRS_P_RX_TLM_PWR - MLRS_P_RX_PROTOCOL)] = pwr;
  }
  dst[2 + (MLRS_P_RX_BLE_RID - MLRS_P_RX_PROTOCOL)] =
      elrs_config_get_ble_remote_id() ? 1 : 0;
  dst[2 + (MLRS_P_RX_WIFI - MLRS_P_RX_PROTOCOL)] = 0;
  dst[2 + (MLRS_P_RX_TEAM_CH - MLRS_P_RX_PROTOCOL)] =
      (cfg->teamrace_channel > 10) ? 10 : cfg->teamrace_channel;
  dst[2 + (MLRS_P_RX_TEAM_POS - MLRS_P_RX_PROTOCOL)] =
      (cfg->teamrace_position > 7) ? 7 : cfg->teamrace_position;
  dst[2 + (MLRS_P_RX_BIND_STOR - MLRS_P_RX_PROTOCOL)] =
      (cfg->bind_storage > 3) ? 0 : cfg->bind_storage;
  dst[2 + (MLRS_P_RX_BIND_MODE - MLRS_P_RX_PROTOCOL)] = 0;
  dst[2 + (MLRS_P_RX_MODEL_ID - MLRS_P_RX_PROTOCOL)] =
      (cfg->model_id == 0xFF) ? 0
      : (cfg->model_id >= 63) ? 64
                              : (uint8_t)(cfg->model_id + 1);
  return n;
}

static bool rx_serial_is_mavlink() {
  elrs_config_t *cfg = elrs_config_get();
  return cfg != nullptr &&
         elrs_serial_protocol_to_lua_selection(cfg->serial_protocol) ==
             ELRS_SERIAL_PROTOCOL_LUA_SELECTION_MAVLINK;
}

static bool mavlinkx_try_mux_item(uint8_t *payload, uint8_t *payload_len,
                                  const uint8_t *tmp, uint8_t n) {
  const uint8_t next =
      mux_append(payload, *payload_len, FRAME_RX_PAYLOAD_LEN, tmp, n);
  if (next == *payload_len) {
    memcpy(g_dn_hold, tmp, n);
    g_dn_hold_len = n;
    return false;
  }
  *payload_len = next;
  return true;
}

static void mlrs_rx_prepare_tlm() {
  if (g_tlm_ready || g_tlm_busy) {
    return;
  }
  const uint8_t seq = tarq_next_seq();
  uint8_t payload[FRAME_RX_PAYLOAD_LEN] = {};
  uint8_t payload_len = 0;
  hop_rebuild_mask();
  {
    uint8_t hm[6];
    hopmask_pack(hm, g_hop_proposed_gen, g_hop_proposed_mask);
    payload_len = mux_append(payload, payload_len, FRAME_RX_PAYLOAD_LEN, hm, 6);
  }
  uint8_t st[32];
  const uint8_t sn = mlrs_rx_pack_state(st, sizeof(st));
  if (sn != 0) {
    payload_len = mux_append(payload, payload_len, FRAME_RX_PAYLOAD_LEN, st, sn);
  }
  const bool mavlink_uart = rx_serial_is_mavlink();
  /* Held leftover is always a mux candidate (CRSF), never mid-MAVLink. */
  if (g_dn_hold_len != 0) {
    uint8_t tmp[64];
    const uint8_t n = g_dn_hold_len;
    memcpy(tmp, g_dn_hold, n);
    g_dn_hold_len = 0;
    if (tmp[0] == 0xFD || tmp[0] == 0xFE) {
      mlrs_mavlinkx_ingest_mav(tmp, n);
    } else if (!mavlinkx_try_mux_item(payload, &payload_len, tmp, n)) {
      payload_len =
          mavlinkx_fill_mux(payload, payload_len, FRAME_RX_PAYLOAD_LEN);
      pack_rx_frame(&g_rx_frame, seq, g_lq, false, payload, payload_len);
      g_rx_frame_valid = 1;
      g_tlm_ready = 1;
      return;
    }
  }
  for (;;) {
    uint8_t tmp[64];
    const uint8_t n = mlrs_elrs_rx_take_downlink(tmp, sizeof(tmp));
    if (n == 0) {
      break;
    }
    if (tmp[0] == 0xFD || tmp[0] == 0xFE) {
      mlrs_mavlinkx_ingest_mav(tmp, n);
      continue;
    }
    if (!mavlinkx_try_mux_item(payload, &payload_len, tmp, n)) {
      break;
    }
  }
  for (;;) {
    uint8_t tmp[64];
    const uint8_t n = mlrs_elrs_rx_take_serial(tmp, sizeof(tmp));
    if (n == 0) {
      break;
    }
    if (mavlink_uart || tmp[0] == 0xFD || tmp[0] == 0xFE) {
      mlrs_mavlinkx_ingest_mav(tmp, n);
      continue;
    }
    if (!mavlinkx_try_mux_item(payload, &payload_len, tmp, n)) {
      break;
    }
  }
  payload_len =
      mavlinkx_fill_mux(payload, payload_len, FRAME_RX_PAYLOAD_LEN);
  pack_rx_frame(&g_rx_frame, seq, g_lq, false, payload, payload_len);
  g_rx_frame_valid = 1;
  g_tlm_ready = 1;
}

static void mlrs_rx_air_tx(tRxFrame *frame) {
  if (g_tlm_busy) {
    return;
  }
  g_tlm_busy = 1;
  g_tlm_busy_ms = millis();
  Radio.TXnb((uint8_t *)frame, false, nullptr, SX12XX_Radio_All);
  ++g_tx_sent;
}

static void mlrs_rx_send_tlm() {
  if (g_tlm_busy) {
    /* Do not hop/listen over an in-flight telemetry TX. That aborts the
     * radio before TXdone, so g_tlm_busy never clears. */
    return;
  }
  if (tarq_should_retry()) {
    /* Stock update_rxframe_stats: same seq/payload, refresh LQ/CRC.
     * New pkt_counter is overlay GCM; run here after DIO/SPI finishes. */
    refresh_rx_frame(&g_rx_sent, g_lq);
    mlrs_rx_air_tx(&g_rx_sent);
    return;
  }
  mlrs_rx_prepare_tlm();
  if (!g_tlm_ready) {
    mlrs_rx_hop_listen();
    return;
  }
  memcpy(&g_rx_sent, &g_rx_frame, sizeof(g_rx_sent));
  g_rx_sent_valid = 1;
  tarq_note_sent((uint8_t)g_rx_frame.status.seq_no);
  g_tlm_ready = 0;
  mlrs_rx_air_tx(&g_rx_frame);
}

static void mlrs_rx_done_tx() {
  g_tlm_busy = 0;
  g_tlm_busy_ms = 0;
  g_fhss_do_hop = 1;
  g_fhss_arm_rx = 1;
}

static uint8_t mlrs_clamp_u8(uint8_t v, uint8_t lo, uint8_t hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

static int8_t mlrs_tlm_power_to_dbm(uint8_t sel) {
  static const int8_t kDbm[] = {10, 14, 17, 20};
  if (sel >= 4) {
    return ELRS_TX_POWER_MATCH_TX_DBM;
  }
  return kDbm[sel];
}

static void mlrs_rx_apply_mbridge(const uint8_t *f, uint8_t n) {
  if (f == nullptr || n < 2 || f[0] != MLRS_AIR_MBRIDGE) {
    return;
  }
  const uint8_t cmd = f[1];
  if (cmd == 13) {
    (void)elrs_config_save();
    printf("[mLRS] rx MBridge PARAM_STORE\n");
    return;
  }
  if (cmd != 12 || n < 4) {
    return;
  }
  const uint8_t idx = f[2];
  const uint8_t val = f[3];
  static uint8_t last_idx = 0xFF;
  static uint8_t last_val = 0xFF;
  const bool dup = (idx == last_idx && val == last_val);
  last_idx = idx;
  last_val = val;
  printf("[mLRS] rx MBridge PARAM_SET idx=%u val=%u%s\n", (unsigned)idx,
         (unsigned)val, dup ? " (repeat)" : "");
  if (dup && idx != MLRS_P_RX_BLE_RID && idx != MLRS_P_RX_WIFI &&
      idx != MLRS_P_RX_BIND_MODE) {
    return;
  }
  elrs_config_t *cfg = elrs_config_get();
  if (cfg == nullptr) {
    printf("[mLRS] rx MBridge PARAM_SET dropped, no config\n");
    return;
  }
  switch (idx) {
  case MLRS_P_RX_PROTOCOL:
    cfg->serial_protocol = elrs_serial_protocol_from_lua_selection(
        mlrs_clamp_u8(val, 0, ELRS_SERIAL_PROTOCOL_LUA_SELECTION_MAX));
    (void)elrs_config_save_with_rf_rearm();
    break;
  case MLRS_P_RX_SBUS_FS:
    cfg->failsafe_mode =
        mlrs_clamp_u8(val, ELRS_FAILSAFE_NO_PULSES, ELRS_FAILSAFE_LAST);
    break;
  case MLRS_P_RX_TGT_SYS:
    cfg->mavlink_target_sys_id = mlrs_clamp_u8(val, 1, 255);
    break;
  case MLRS_P_RX_SRC_SYS:
    cfg->mavlink_source_sys_id = mlrs_clamp_u8(val, 1, 255);
    break;
  case MLRS_P_RX_TLM_OFF:
    cfg->force_tlm = val != 0 ? 1 : 0;
    siw917_rx_set_force_telemetry_off(cfg->force_tlm);
    break;
  case MLRS_P_RX_TLM_PWR:
    cfg->tx_power = mlrs_tlm_power_to_dbm(val);
    break;
  case MLRS_P_RX_BLE_RID: {
    const bool enabled = val != 0;
    elrs_config_set_ble_remote_id(enabled);
    (void)ble_remote_id_service_start(enabled);
    printf("[mLRS] rx BLE RemoteID %s\n", enabled ? "On" : "Off");
    break;
  }
  case MLRS_P_RX_WIFI:
    if (val != 0) {
      elrs_cpp_request_wifi_mode();
    }
    break;
  case MLRS_P_RX_TEAM_CH:
    cfg->teamrace_channel = mlrs_clamp_u8(val, 0, 10);
    break;
  case MLRS_P_RX_TEAM_POS:
    cfg->teamrace_position = mlrs_clamp_u8(val, 0, 7);
    break;
  case MLRS_P_RX_BIND_STOR:
    elrs_apply_bind_storage_change(mlrs_clamp_u8(val, 0, 3));
    break;
  case MLRS_P_RX_BIND_MODE:
    if (val != 0 && cfg->bind_storage != ELRS_BIND_STORAGE_ADMINISTERED) {
      elrs_enter_binding_mode();
    }
    break;
  case MLRS_P_RX_MODEL_ID: {
    const uint8_t selection = mlrs_clamp_u8(val, 0, 64);
    cfg->model_id = (selection == 0) ? (uint8_t)0xFF : (uint8_t)(selection - 1);
    siw917_rx_set_model_match_id(cfg->model_id);
    break;
  }
  default:
    break;
  }
}

static void mlrs_rx_process_uplink() {
  const uint8_t fail = open_tx_gcm(&g_tx_frame);
  if (fail != 0) {
    g_last_rx_fail = fail;
    ++g_rx_fail;
    hop_note((uint8_t)g_tx_frame.status.fhss_index, false);
    return;
  }
  ++g_rx_ok;
  note_valid_rx();
  hop_note((uint8_t)g_tx_frame.status.fhss_index, true);
  if ((g_tx_frame.status.fhss_index_band != g_band) &&
      (g_tx_frame.status.fhss_index_band < MLRS_BAND_COUNT)) {
    g_pending_band = (uint8_t)g_tx_frame.status.fhss_index_band;
  }
  g_last_rx_spare = (uint8_t)g_tx_frame.status.spare;
  if (g_tx_frame.status.payload_len >= 2 &&
      g_tx_frame.payload[0] == MLRS_RATE_CMD &&
      g_tx_frame.payload[1] < MLRS_RATE_COUNT) {
    g_pending_rate = g_tx_frame.payload[1];
  } else if (g_tx_frame.status.payload_len >= 3 &&
             g_tx_frame.payload[0] == MLRS_BAND_CMD) {
    if (g_tx_frame.payload[1] < MLRS_BAND_COUNT) {
      g_pending_band = g_tx_frame.payload[1];
    }
    if (g_tx_frame.payload[2] < MLRS_RATE_COUNT) {
      g_pending_rate = g_tx_frame.payload[2];
    }
  } else if ((g_tx_frame.status.spare != g_rate) &&
             (g_tx_frame.status.spare < MLRS_RATE_COUNT)) {
    g_pending_rate = (uint8_t)g_tx_frame.status.spare;
  }
  uint16_t rc[16];
  unpack_tx_rc(&g_tx_frame, rc);
  apply_rc_to_elrs(rc);
  if (g_tx_frame.status.frame_type == FRAME_TYPE_TX_RX_CMD ||
      (g_tx_frame.status.payload_len == 1 &&
       g_tx_frame.payload[0] == MLRS_SWITCH_CMD)) {
    if (g_pending_protocol != ELRS_AIR_PROTOCOL_ELRS) {
      printf("[mLRS] rx switch cmd -> ELRS type=%u plen=%u\n",
             (unsigned)g_tx_frame.status.frame_type,
             (unsigned)g_tx_frame.status.payload_len);
      g_pending_protocol = ELRS_AIR_PROTOCOL_ELRS;
    }
    g_apply_at_ms = millis() + 100;
    g_fhss_follow = false;
  } else if (g_tx_frame.status.payload_len >= 3 &&
             g_tx_frame.payload[0] == MLRS_BAND_CMD) {
    static uint8_t logged_band = 0xFF;
    static uint8_t logged_band_rate = 0xFF;
    if (logged_band != g_tx_frame.payload[1] ||
        logged_band_rate != g_tx_frame.payload[2]) {
      logged_band = g_tx_frame.payload[1];
      logged_band_rate = g_tx_frame.payload[2];
      printf("[mLRS] rx band cmd -> %s / %s\n",
             g_tx_frame.payload[1] == MLRS_BAND_24 ? "2.4GHz" : "915MHz",
             kRates[sanitize_band(g_tx_frame.payload[1])]
                   [sanitize_rate(g_tx_frame.payload[2])]
                       .name);
    }
  } else if (g_tx_frame.status.payload_len >= 2 &&
             g_tx_frame.payload[0] == MLRS_RATE_CMD) {
    static uint8_t logged_rate = 0xFF;
    if (logged_rate != g_tx_frame.payload[1]) {
      logged_rate = g_tx_frame.payload[1];
      printf("[mLRS] rx rate cmd -> %s spare=%u\n",
             kRates[sanitize_band(g_band)][sanitize_rate(g_tx_frame.payload[1])]
                 .name,
             (unsigned)g_last_rx_spare);
    }
  } else if (g_tx_frame.status.payload_len > 0) {
    const uint8_t *p = g_tx_frame.payload;
    uint8_t len = (uint8_t)g_tx_frame.status.payload_len;
    auto deliver = [](const uint8_t *f, uint8_t n) {
      if (f == nullptr || n == 0) {
        return;
      }
      if (f[0] == MLRS_AIR_HOPMASK) {
        uint8_t gen = 0;
        uint32_t mask = 0;
        if (hopmask_parse(f, n, &gen, &mask)) {
          hopmask_apply(gen, mask);
        }
        return;
      }
      if (f[0] == MLRS_AIR_MBRIDGE) {
        mlrs_rx_apply_mbridge(f, n);
        return;
      }
      if (f[0] == MLRS_AIR_MAVLINKX) {
        mlrs_mavlinkx_ingest_air(f + 1, (uint8_t)(n - 1U));
        uint8_t mav[64];
        uint8_t k;
        while ((k = mlrs_mavlinkx_take_mav(mav, sizeof(mav))) != 0) {
          mlrs_elrs_rx_write_serial(mav, k);
        }
        return;
      }
      mlrs_elrs_rx_accept_uplink(f, n);
    };
    if (len >= 2 && p[0] == MLRS_MUX_MAGIC) {
      uint8_t n = p[1];
      uint8_t i = 2;
      while (n-- != 0 && i < len) {
        const uint8_t fl = p[i++];
        if (fl == 0 || (uint16_t)i + fl > len) {
          break;
        }
        deliver(p + i, fl);
        i = (uint8_t)(i + fl);
      }
    } else {
      deliver(p, len);
    }
  }
}

static bool mlrs_rx_rx_done(SX12xxDriverCommon::rx_status) {
  memcpy(&g_tx_frame, Radio.RXdataBuffer, sizeof(g_tx_frame));
  const uint8_t fail = check_tx_crc(&g_tx_frame);
  if (fail != 0 && fail != 4) {
    g_last_rf_ms = millis();
    g_last_rx_fail = fail;
    if (fail == 1) {
      ++g_rx_junk;
    } else {
      ++g_rx_fail;
      if (g_fhss_follow) {
        hop_note(g_fhss_i, false);
      }
      tarq_missed();
    }
    g_rx_need_rearm = 1;
    return false;
  }
  /* fail 0 = full frame, fail 4 = CRC1 ok / payload CRC bad (stock CRC1_VALID). */
  g_last_rf_ms = millis();
  g_last_rx_ms = millis();
  g_slot_rx = 1;
  g_rx_need_rearm = (fail == 4) ? 1 : 0;
  g_miss_streak = 0;
  fhss_set_index(g_tx_frame.status.fhss_index);
  g_fhss_follow = true;
  tarq_ack((uint8_t)g_tx_frame.status.ack);
  g_tlm_slot_skip =
      (tight_slot() && ((g_rx_ok & 1U) != 0U)) ? 1 : 0;
  g_tlm_slot_pending = 1;
  if (fail == 0) {
    g_last_rx_fail = 0;
    g_uplink_pending = 1;
  } else {
    g_last_rx_fail = fail;
    ++g_rx_fail;
  }
  return fail == 0;
}

static void mlrs_rx_tock() {
  if (g_tlm_busy) {
    /* Downlink TX occupies this slot. Lua/CRSF can delay TXdone past the
     * 32 ms interval; those tocks are not missed RX hops. Keep slot_rx
     * so the next tock does not count the packet we already got. */
    return;
  }
  if (!g_fhss_follow) {
    /* Search/listen: re-arm only after junk put the radio in FS. */
    if (g_rx_need_rearm) {
      g_fhss_do_hop = 0;
      g_fhss_arm_rx = 1;
    }
    return;
  }
  /* Hop once per missed slot on the same cadence as TX. Waiting
   * interval+8 ms hopped late (TX already on n+1) and desynced 50 Hz. */
  if (g_slot_rx) {
    g_slot_rx = 0;
    if (g_rx_need_rearm) {
      g_fhss_do_hop = 0;
      g_fhss_arm_rx = 1;
    }
  } else {
    note_missed_rx();
    tarq_missed();
    hop_note(g_fhss_i, false);
    if (g_miss_streak < 255) {
      ++g_miss_streak;
    }
    if (g_fhss_count > 0 && g_miss_streak >= g_fhss_count) {
      /* Full hop cycle with no lock: sit hop 0 and reacquire. */
      g_fhss_follow = false;
      g_fhss_i = 0;
      g_fhss_pending_hops = 0;
      g_miss_streak = 0;
      g_fhss_do_hop = 0;
      g_fhss_arm_rx = 1;
      return;
    }
    ++g_fhss_pending_hops;
    g_fhss_do_hop = 0;
    g_fhss_arm_rx = 1;
  }
}
#endif

static void start_mlrs() {
  if (g_active) {
    return;
  }
  derive_sync_from_uid();
  derive_crypto_from_uid();
  if (!g_gcm_ready) {
    printf("[mLRS] AES-GCM selftest failed, not starting\n");
    return;
  }
  load_gcm_counters();
  persist_gcm_counters();
  fhss_generate();
  g_saved_tick = hwTimer::callbackTick;
  g_saved_tock = hwTimer::callbackTock;
  g_saved_rx_cb = Radio.RXdoneCallback;
  g_saved_tx_cb = Radio.TXdoneCallback;
  g_saved_interval_us = hwTimer::getInterval();

  hwTimer::stop();
  Radio.SetTxIdleMode();
  delay(5);
  Radio.ClearIrqStatus(SX12XX_Radio_All);
  configure_mlrs_radio();
  delay(5);

#if MLRS_OTA_IS_TX
  Radio.TXdoneCallback = mlrs_tx_done;
  Radio.RXdoneCallback = mlrs_tx_rx_done;
  hwTimer::callbackTock = mlrs_tx_tock;
  busyTransmitting = false;
#else
  Radio.TXdoneCallback = mlrs_rx_done_tx;
  Radio.RXdoneCallback = mlrs_rx_rx_done;
  hwTimer::callbackTock = mlrs_rx_tock;
#endif
  hwTimer::callbackTick = nullptr;
  hwTimer::updateInterval(current_rate_cfg()->interval_us);
#if MLRS_OTA_IS_TX
  g_seq = 0;
#endif
  g_valid_window = 0;
  g_connected = 0;
  g_tx_sent = 0;
  g_rx_ok = 0;
  g_rx_fail = 0;
  g_rx_junk = 0;
  g_last_rx_fail = 0;
  g_mlrs_started_ms = millis();
  g_last_rx_ms = 0;
#if MLRS_OTA_IS_TX
  g_tx_send_pending = 0;
  g_downlink_pending = 0;
  g_logged_switch_cmd = false;
#else
  g_uplink_pending = 0;
  g_tlm_ready = 0;
  g_tlm_busy = 0;
  g_tlm_busy_ms = 0;
  g_had_link = false;
  g_last_rf_ms = 0;
  g_slot_rx = 0;
  g_rx_need_rearm = 0;
  g_fhss_do_hop = 0;
  g_fhss_pending_hops = 0;
  g_miss_streak = 0;
#endif
  g_active = true;
  g_protocol = ELRS_AIR_PROTOCOL_MLRS;
  mlrs_mavlinkx_init();
  mavlinkx_sync_compression();
  arq_init();
#if MLRS_OTA_IS_TX
  mlrs_dynpower_begin();
  /* Keep the last ELRS RQly during the first grace window. If no mLRS
   * downlink arrives, siw917_tx_note_mlrs_link_lost() sends LQ=0. */
  siw917_tx_publish_mlrs_linkstats();
#else
  setConnectionState(disconnected);
#endif
  hwTimer::resume();
#if MLRS_OTA_IS_TX
  if (g_tx_send_pending) {
    g_tx_send_pending = 0;
    mlrs_tx_send_frame();
  }
#endif
#if !MLRS_OTA_IS_TX
  g_rx_frame_valid = 0;
  g_tlm_slot_pending = 0;
  Radio.RXnb();
#endif
  printf("[mLRS] air protocol started (%s/%s, sync=0x%04X freq=%lu hop=0/%u)\n",
         g_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz", current_rate_cfg()->name,
         (unsigned)g_sync_word, (unsigned long)fhss_curr(),
         (unsigned)g_fhss_count);
  printf("[mLRS] overlay %s payload TX=%u RX=%u tag=%u ctr=%lu\n",
         MLRS_OVERLAY_ID, (unsigned)FRAME_TX_PAYLOAD_LEN,
         (unsigned)FRAME_RX_PAYLOAD_LEN, (unsigned)FRAME_GCM_TAG_LEN,
         (unsigned long)g_send_counter);
  printf("[mLRS] AES-GCM backend=%s secret=%s\n", mlrs_gcm_backend_name(),
         g_secret_ok ? "ok" : "MISSING");
  printf("[mLRS] mux MBridge Lua + CRSF + MAVLinkX in %u/%u-byte payloads\n",
         (unsigned)FRAME_TX_PAYLOAD_LEN, (unsigned)FRAME_RX_PAYLOAD_LEN);
  printf("[mLRS] mavlinkx compress=%s (19Hz only, stock X4)\n",
         sanitize_rate(g_rate) == MLRS_RATE_19HZ ? "on" : "off");
  printf("[mLRS] arq downlink retry=1 (stock)\n");
}

static void stop_mlrs() {
  if (!g_active) {
    return;
  }
  mlrs_mavlinkx_reset();
  arq_init();
  hwTimer::stop();
  persist_gcm_counters();
  Radio.SetTxIdleMode();
  Radio.ClearIrqStatus(SX12XX_Radio_All);
  Radio.RXdoneCallback = g_saved_rx_cb;
  Radio.TXdoneCallback = g_saved_tx_cb;
  hwTimer::callbackTick = g_saved_tick;
  hwTimer::callbackTock = g_saved_tock;
  if (ExpressLRS_currAirRate_Modparams != nullptr &&
      ExpressLRS_currAirRate_Modparams->interval != 0) {
    hwTimer::updateInterval(ExpressLRS_currAirRate_Modparams->interval);
  } else if (g_saved_interval_us != 0) {
    hwTimer::updateInterval(g_saved_interval_us);
  }
  restore_elrs_radio();
#if MLRS_OTA_IS_TX
  mlrs_dynpower_end();
  FHSSsetCurrIndex(0);
  OtaNonce = 0;
#if defined(SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
  // Same as an ELRS rate change: SYNC on nonce slots 1/2 even off hop 0
  // so a tentative RX keeps hearing packets at 1000 Hz.
  syncSpamCounter = 3;
  syncSpamCounterAfterRateChange = 10;
  SyncPacketLastSent = 0;
#endif
  // Drop mLRS downlink age so ELRS does not fake "connected" and skip SYNC.
  siw917_tx_note_mlrs_link_lost();
#endif
  delay(5);
  g_active = false;
  g_protocol = ELRS_AIR_PROTOCOL_ELRS;
  flush_mlrs_config();
  setConnectionState(disconnected);
#if MLRS_OTA_IS_TX
  hwTimer::resume();
#else
  // Stock ELRS keeps the RX timer stopped while disconnected so the next
  // SYNC can start it in phase. Resuming here hops on a leftover nonce and
  // the RX hears one SYNC, goes tentative, then misses every later packet.
  FHSSsetCurrIndex(0);
  OtaNonce = 0;
  g_lock_first_elrs_sync = true;
  Radio.RXnb();
#endif
  printf("[mLRS] restored ELRS air protocol\n");
}

static void apply_protocol(uint8_t protocol) {
  g_pending_protocol = 0xFF;
  g_apply_at_ms = 0;
  if (protocol == ELRS_AIR_PROTOCOL_MLRS) {
    start_mlrs();
  } else {
    stop_mlrs();
  }
  save_protocol(protocol);
}

extern "C" bool mlrs_ota_is_active(void) { return g_active; }

extern "C" bool mlrs_ota_tlm_busy(void) {
#if MLRS_OTA_IS_TX
  return false;
#else
  return g_tlm_busy != 0;
#endif
}

extern "C" uint32_t mlrs_ota_tlm_busy_timeout_ms(void) {
  /* Cap only. TXdone usually arrives in a few ms. 40 ms was sized for
   * 31 Hz; 19 Hz LoRa SF6 99-byte frames are still on air past that. */
  return current_rate_cfg()->interval_us / 1000U + 15U;
}

extern "C" void mlrs_ota_rx_send_slot(void) {
#if !MLRS_OTA_IS_TX
  if (!g_active || g_tlm_slot_pending == 0) {
    return;
  }
  g_tlm_slot_pending = 0;
  if (g_tlm_slot_skip) {
    if (!g_tlm_busy) {
      mlrs_rx_hop_listen();
    }
    return;
  }
  /* Stock order: ACK already applied in RXdone, then GetFreshPayload,
   * pack or update last frame, send. GCM runs after DIO/SPI returns. */
  mlrs_rx_send_tlm();
#endif
}

extern "C" bool mlrs_ota_take_elrs_first_sync(void) {
  if (!g_lock_first_elrs_sync) {
    return false;
  }
  g_lock_first_elrs_sync = false;
  return true;
}

extern "C" bool mlrs_ota_is_connected(void) { return g_active && g_connected; }

extern "C" void mlrs_ota_hop_skip_info(uint8_t *skip_count, uint8_t *hop_count,
                                       uint32_t *mask) {
  if (skip_count != nullptr) {
    *skip_count = hop_bitcount(g_hop_mask);
  }
  if (hop_count != nullptr) {
    *hop_count = g_fhss_count;
  }
  if (mask != nullptr) {
    *mask = g_hop_mask;
  }
}

extern "C" void mlrs_ota_link_rate_info(uint8_t *rate, uint8_t *band,
                                        uint8_t *ul_plen, uint8_t *dl_plen) {
  if (rate != nullptr) {
    *rate = sanitize_rate(g_rate);
  }
  if (band != nullptr) {
    *band = sanitize_band(g_band);
  }
  if (ul_plen != nullptr) {
    *ul_plen = g_last_ul_plen;
  }
  if (dl_plen != nullptr) {
    *dl_plen = g_last_dl_plen;
  }
}

extern "C" uint8_t mlrs_ota_get_protocol(void) {
  if (g_pending_protocol != 0xFF) {
    return g_pending_protocol;
  }
  return g_protocol;
}

extern "C" bool mlrs_ota_set_protocol(uint8_t protocol) {
  if (protocol > ELRS_AIR_PROTOCOL_MLRS) {
    return false;
  }
  if (protocol == g_protocol && g_pending_protocol == 0xFF) {
    return true;
  }
  printf("[mLRS] lua/air protocol -> %s\n",
         protocol == ELRS_AIR_PROTOCOL_MLRS ? "mLRS" : "ELRS");
#if MLRS_OTA_IS_TX
  if (protocol == ELRS_AIR_PROTOCOL_MLRS && g_active) {
    g_pending_protocol = 0xFF;
    g_apply_at_ms = 0;
    return true;
  }
  if (protocol == ELRS_AIR_PROTOCOL_MLRS && !g_active) {
    notify_rx_protocol(protocol);
    g_pending_protocol = protocol;
    g_apply_at_ms = millis() + 400;
    return true;
  }
  if (protocol == ELRS_AIR_PROTOCOL_ELRS && g_active) {
    g_pending_protocol = ELRS_AIR_PROTOCOL_ELRS;
    const uint32_t hold_ms = announce_ms();
    g_apply_at_ms = millis() + hold_ms;
    printf("[mLRS] announce ELRS for %u ms on hop 0\n", (unsigned)hold_ms);
    return true;
  }
#endif
  apply_protocol(protocol);
  return true;
}

extern "C" uint8_t mlrs_ota_get_rate(void) {
  if (g_pending_rate != 0xFF) {
    return g_pending_rate;
  }
  return sanitize_rate(g_rate);
}

extern "C" uint8_t mlrs_ota_get_band(void) {
  if (g_pending_band != 0xFF) {
    return g_pending_band;
  }
  return sanitize_band(g_band);
}

extern "C" const char *mlrs_ota_rate_name(void) {
  return kRates[mlrs_ota_get_band()][mlrs_ota_get_rate()].name;
}

extern "C" const char *mlrs_ota_rate_options(void) {
  return mlrs_ota_get_band() == MLRS_BAND_24 ? MLRS_OTA_RATE_OPTIONS_24
                                             : MLRS_OTA_RATE_OPTIONS_915;
}

extern "C" bool mlrs_ota_set_rate(uint8_t rate) {
  printf("[mLRS] lua/air rate arg=%u\n", (unsigned)rate);
  rate = sanitize_rate(rate);
  if (rate == g_rate && g_pending_rate == 0xFF) {
    return true;
  }
  printf("[mLRS] lua/air rate -> %s\n", kRates[sanitize_band(g_band)][rate].name);
#if MLRS_OTA_IS_TX
  if (g_pending_protocol == ELRS_AIR_PROTOCOL_MLRS && !g_active) {
    notify_rx_protocol(ELRS_AIR_PROTOCOL_MLRS);
  }
  if (g_active && rate != g_rate) {
    g_pending_rate = rate;
    const uint32_t hold_ms = announce_ms();
    g_apply_rate_at_ms = millis() + hold_ms;
    save_rate(rate);
    printf("[mLRS] announce rate %s for %u ms on hop 0\n",
           kRates[sanitize_band(g_band)][rate].name, (unsigned)hold_ms);
    return true;
  }
  if (g_active && rate == g_rate) {
    g_pending_rate = 0xFF;
    g_apply_rate_at_ms = 0;
    return true;
  }
#endif
  g_rate = rate;
  g_pending_rate = 0xFF;
  g_apply_rate_at_ms = 0;
  save_rate(rate);
  apply_mlrs_rate();
  return true;
}

extern "C" bool mlrs_ota_set_band(uint8_t band) {
  band = sanitize_band(band);
  if (band == g_band && g_pending_band == 0xFF) {
    return true;
  }
  const uint8_t new_rate = remap_rate_for_band(g_band, band, advertised_rate());
  printf("[mLRS] lua/air band -> %s / %s\n",
         band == MLRS_BAND_24 ? "2.4GHz" : "915MHz",
         kRates[band][new_rate].name);
  save_band(band);
  save_rate(new_rate);
#if MLRS_OTA_IS_TX
  if (g_pending_protocol == ELRS_AIR_PROTOCOL_MLRS && !g_active) {
    notify_rx_protocol(ELRS_AIR_PROTOCOL_MLRS);
  }
  if (g_active && band != g_band) {
    g_pending_band = band;
    g_pending_rate = new_rate;
    const uint32_t hold_ms = announce_ms();
    g_apply_band_at_ms = millis() + hold_ms;
    g_apply_rate_at_ms = 0;
    printf("[mLRS] announce band %s / %s for %u ms on hop 0\n",
           band == MLRS_BAND_24 ? "2.4GHz" : "915MHz",
           kRates[band][new_rate].name, (unsigned)hold_ms);
    return true;
  }
  if (g_active && band == g_band) {
    g_pending_band = 0xFF;
    g_apply_band_at_ms = 0;
    return true;
  }
#endif
  g_rate = new_rate;
  g_band = band;
  g_pending_band = 0xFF;
  g_pending_rate = 0xFF;
  g_apply_band_at_ms = 0;
  g_apply_rate_at_ms = 0;
  apply_mlrs_rate();
  return true;
}

extern "C" void mlrs_ota_on_elrs_ready(void) {
  g_elrs_ready = true;
#if MLRS_OTA_IS_TX
  mlrs_mbridge_init();
#endif
  g_band = config_band();
  g_rate = config_rate();
  g_protocol = ELRS_AIR_PROTOCOL_ELRS;
  if (config_protocol() == ELRS_AIR_PROTOCOL_MLRS) {
    save_protocol(ELRS_AIR_PROTOCOL_ELRS);
  }
  printf("[mLRS] boot air protocol=ELRS band=%s rate=%s (mLRS Lua-selected only)\n",
         g_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz",
         current_rate_cfg()->name);
}

extern "C" void mlrs_ota_loop(void) {
#if MLRS_OTA_IS_TX
  mlrs_mbridge_poll();
#endif
  if (g_print_connected) {
    g_print_connected = 0;
    printf("[mLRS] connected lq=%u\n", (unsigned)g_lq);
#if MLRS_OTA_IS_TX
    siw917_tx_publish_mlrs_linkstats();
#endif
  }
  if (g_print_disconnected) {
    g_print_disconnected = 0;
    printf("[mLRS] disconnected\n");
#if MLRS_OTA_IS_TX
    siw917_tx_note_mlrs_link_lost();
    siw917_tx_publish_mlrs_linkstats();
#endif
  }
#if MLRS_OTA_IS_TX
  if (g_active && g_tx_send_pending) {
    g_tx_send_pending = 0;
    mlrs_tx_send_frame();
  }
  if (g_active && g_downlink_pending) {
    g_downlink_pending = 0;
    mlrs_tx_process_downlink();
  }
  if (g_active) {
    static uint32_t g_last_handset_tlm_ms = 0;
    if (g_last_handset_tlm_ms == 0 ||
        (int32_t)(millis() - g_last_handset_tlm_ms) >= 200) {
      g_last_handset_tlm_ms = millis();
      const uint32_t ref =
          g_last_rx_ms != 0 ? g_last_rx_ms : g_mlrs_started_ms;
      if (!g_connected &&
          (int32_t)(millis() - ref) >= (int32_t)MLRS_LOST_MS) {
        siw917_tx_note_mlrs_link_lost();
      }
      siw917_tx_publish_mlrs_linkstats();
    }
  }
#else
  if (g_active && g_uplink_pending) {
    g_uplink_pending = 0;
    mlrs_rx_process_uplink();
  }
  if (g_active && g_tlm_busy && g_tlm_busy_ms != 0 &&
      (int32_t)(millis() - g_tlm_busy_ms) >=
          (int32_t)mlrs_ota_tlm_busy_timeout_ms()) {
    printf("[mLRS] tlm TX watchdog, re-arm RX\n");
    g_tlm_busy = 0;
    g_tlm_busy_ms = 0;
    g_tlm_ready = 0;
    g_fhss_arm_rx = 1;
  }
  if (g_active && g_fhss_arm_rx && !g_tlm_busy) {
    g_fhss_arm_rx = 0;
    g_rx_need_rearm = 0;
    uint8_t hops = g_fhss_pending_hops;
    g_fhss_pending_hops = 0;
    if (hops != 0) {
      while (hops-- != 0) {
        fhss_hop();
      }
    } else if (g_fhss_follow && g_fhss_do_hop) {
      fhss_hop();
    }
    g_fhss_do_hop = 0;
    Radio.SetFrequencyReg(fhss_curr(), SX12XX_Radio_All, true, 0);
  }
#endif
  if (g_pending_protocol != 0xFF && g_apply_at_ms != 0 &&
      (int32_t)(millis() - g_apply_at_ms) >= 0) {
    apply_protocol(g_pending_protocol);
  }
#if MLRS_OTA_IS_TX
  if (g_apply_band_at_ms != 0 &&
      (int32_t)(millis() - g_apply_band_at_ms) >= 0) {
    if (g_pending_band != 0xFF && g_pending_band != g_band) {
      g_rate = sanitize_rate(g_pending_rate != 0xFF ? g_pending_rate : g_rate);
      g_band = sanitize_band(g_pending_band);
      save_band(g_band);
      save_rate(g_rate);
      apply_mlrs_rate();
    }
    g_pending_band = 0xFF;
    g_pending_rate = 0xFF;
    g_apply_band_at_ms = 0;
    g_apply_rate_at_ms = 0;
  } else if (g_apply_rate_at_ms != 0 &&
             (int32_t)(millis() - g_apply_rate_at_ms) >= 0) {
    if (g_pending_rate != 0xFF && g_pending_rate != g_rate) {
      g_rate = sanitize_rate(g_pending_rate);
      save_rate(g_rate);
      apply_mlrs_rate();
    }
    g_pending_rate = 0xFF;
    g_apply_rate_at_ms = 0;
  }
#else
  bool config_changed = false;
  const uint8_t pending_band = g_pending_band;
  const uint8_t pending_rate = g_pending_rate;
  if (pending_band != 0xFF && pending_band != g_band &&
      pending_band < MLRS_BAND_COUNT) {
    printf("[mLRS] rx follow band %s -> %s\n",
           g_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz",
           pending_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz");
    g_band = pending_band;
    save_band(g_band);
    config_changed = true;
  }
  if (pending_band != 0xFF && g_pending_band == pending_band) {
    g_pending_band = 0xFF;
  }
  if (pending_rate != 0xFF && pending_rate != g_rate &&
      pending_rate < MLRS_RATE_COUNT) {
    printf("[mLRS] rx follow rate %s -> %s spare=%u\n",
           current_rate_cfg()->name,
           kRates[sanitize_band(g_band)][sanitize_rate(pending_rate)].name,
           (unsigned)g_last_rx_spare);
    g_rate = sanitize_rate(pending_rate);
    save_rate(g_rate);
    config_changed = true;
  }
  if (pending_rate != 0xFF && g_pending_rate == pending_rate) {
    g_pending_rate = 0xFF;
  }
  if (config_changed) {
    apply_mlrs_rate();
  }
  if (g_active && !g_connected) {
    const uint32_t heard_ms = g_last_rf_ms != 0 ? g_last_rf_ms : g_last_rx_ms;
    const bool heard_recently =
        (heard_ms != 0) &&
        ((int32_t)(millis() - heard_ms) < (int32_t)MLRS_RX_SCAN_MS);
    if (heard_recently) {
      g_rate_scan_ms = millis();
    } else if (g_had_link) {
      /* Stay on last rate/hop 0 so SWITCH_CMD and RATE_CMD can be heard. */
    } else {
      if (g_rate_scan_ms == 0) {
        g_rate_scan_ms = millis();
      }
      if ((int32_t)(millis() - g_rate_scan_ms) >= (int32_t)MLRS_RX_SCAN_MS) {
        g_rate_scan_ms = millis();
        const uint8_t next = next_scan_rate(g_rate);
        if (next != g_rate) {
          g_rate = next;
          printf("[mLRS] rx scan rate %s\n", current_rate_cfg()->name);
          apply_mlrs_rate();
        }
      }
    }
  } else {
    g_rate_scan_ms = 0;
  }
#endif
  if (g_active && g_connected && mlrs_rx_age_lost(g_last_rx_ms)) {
    mlrs_declare_lost();
  }
  if (g_active && (millis() - g_last_hb_ms) > 2000) {
    g_last_hb_ms = millis();
#if MLRS_OTA_IS_TX
    const int pwr_dbm = POWERMGNT::getPowerIndBm();
#else
    const int pwr_dbm = 0;
#endif
    printf("[mLRS] waiting lq=%u connected=%u sent=%u rxok=%u rxcrc=%u "
           "junk=%u last=%u rate=%s adv=%s freq=%lu hop=%u rssi=%d rqly=%u "
           "pwr=%d skip=%u/%u arq r=%lu d=%lu\n",
           (unsigned)g_lq, (unsigned)g_connected, (unsigned)g_tx_sent,
           (unsigned)g_rx_ok, (unsigned)g_rx_fail, (unsigned)g_rx_junk,
           (unsigned)g_last_rx_fail,
           current_rate_cfg()->name,
           kRates[advertised_band()][advertised_rate()].name,
           (unsigned long)fhss_curr(), (unsigned)g_fhss_i,
           (int)(int8_t)linkStats.uplink_RSSI_1,
           (unsigned)linkStats.uplink_Link_quality, pwr_dbm,
           (unsigned)hop_bitcount(g_hop_mask), (unsigned)g_fhss_count,
           (unsigned long)g_arq_retry, (unsigned long)g_arq_drop);
    g_arq_retry = 0;
    g_arq_drop = 0;
  }
  flush_mlrs_config();
}

extern "C" bool mlrs_ota_handle_msp(const uint8_t *data, uint8_t len) {
  if (data == nullptr || len < 2) {
    return false;
  }
  if (data[0] != MSP_ELRS_SET_AIR_PROTOCOL) {
    return false;
  }
  if (len >= 4) {
    g_band = sanitize_band(data[3]);
    save_band(g_band);
  }
  if (len >= 3) {
    g_rate = sanitize_rate(data[2]);
    save_rate(g_rate);
  }
  if (len >= (4 + MLRS_GCM_SECRET_LEN) &&
      data[1] == ELRS_AIR_PROTOCOL_MLRS) {
    if (elrs_config_set_mlrs_secret(data + 4) == 0) {
      (void)elrs_config_save();
      memcpy(g_secret, data + 4, MLRS_GCM_SECRET_LEN);
      g_secret_ok = 1;
      printf("[mLRS] stored bind secret from TX\n");
    }
  }
  (void)mlrs_ota_set_protocol(data[1]);
  return true;
}
