#include "mlrs_ota.h"

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
#include "stubborn_sender.h"
extern StubbornSender DataUlSender;
extern CRSFRouter crsfRouter;
extern TXOTAConnector otaConnector;
void sendCRSFTelemetryToBackpack(uint8_t *);
extern "C" void siw917_tx_note_mlrs_downlink(uint8_t lq, int8_t rssi, int8_t snr);
extern "C" void siw917_tx_note_mlrs_link_lost(void);
extern "C" void siw917_tx_publish_mlrs_linkstats(void);
#else
#define MLRS_OTA_IS_TX 0
#endif

extern elrsLinkStatistics_t linkStats;
extern bool connectionHasModelMatch;
#if !MLRS_OTA_IS_TX
extern uint8_t uplinkLQ;
#endif

#undef PACKED
#define PACKED(__Declaration__) __Declaration__ __attribute__((packed))

#define FRAME_TX_RX_LEN 91
#define FRAME_TX_RX_HEADER_LEN 7
#define FRAME_TX_RCDATA1_LEN 6
#define FRAME_TX_PAYLOAD_LEN 64
#define FRAME_RX_PAYLOAD_LEN 82
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
  tFrameRcData1 rc1;
  uint16_t crc1;
  tFrameRcData2 rc2;
  uint8_t payload[64];
  uint16_t crc;
})
tTxFrame;

PACKED(typedef struct {
  uint16_t sync_word;
  tRxFrameStatus status;
  uint8_t payload[82];
  uint16_t crc;
})
tRxFrame;

static_assert(sizeof(tTxFrameStatus) == 5, "tTxFrameStatus size");
static_assert(sizeof(tRxFrameStatus) == 5, "tRxFrameStatus size");
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
static uint8_t g_seq = 0;
static uint8_t g_lq = 0;
static uint8_t g_valid_window = 0;
static uint32_t g_last_rx_ms = 0;
static uint32_t g_last_hb_ms = 0;
static bool g_connected = false;
static volatile uint8_t g_print_connected = 0;
static volatile uint8_t g_print_disconnected = 0;
static uint32_t g_tx_sent = 0;
static uint32_t g_rx_ok = 0;
static uint32_t g_rx_fail = 0;
static uint32_t g_rx_junk = 0;
static uint8_t g_last_rx_fail = 0;
#if MLRS_OTA_IS_TX
static volatile uint8_t g_tx_send_pending = 0;
#endif
static tTxFrame g_tx_frame = {};
static tRxFrame g_rx_frame = {};

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

static void fhss_hop() {
  if (g_fhss_count == 0) {
    return;
  }
  g_fhss_i = (uint8_t)((g_fhss_i + 1) % g_fhss_count);
}

#if !MLRS_OTA_IS_TX
static void fhss_set_index(uint8_t index) {
  if (index < g_fhss_count) {
    g_fhss_i = index;
  }
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
  uint16_t crc;
  crc_init(&crc);
  crc_accumulate_buf(&crc, (uint8_t *)frame,
                     FRAME_TX_RX_HEADER_LEN + FRAME_TX_RCDATA1_LEN);
  frame->crc1 = crc;
  crc_accumulate_buf(&crc,
                     (uint8_t *)frame + FRAME_TX_RX_HEADER_LEN +
                         FRAME_TX_RCDATA1_LEN,
                     FRAME_TX_RX_LEN - FRAME_TX_RX_HEADER_LEN -
                         FRAME_TX_RCDATA1_LEN - 2);
  frame->crc = crc;
}
#else
static uint8_t check_tx_frame(const tTxFrame *frame) {
  if (frame->sync_word != g_sync_word) {
    return 1;
  }
  if ((frame->status.frame_type != FRAME_TYPE_TX) &&
      (frame->status.frame_type != FRAME_TYPE_TX_RX_CMD)) {
    return 2;
  }
  uint16_t crc;
  crc_init(&crc);
  crc_accumulate_buf(&crc, (const uint8_t *)frame,
                     FRAME_TX_RX_HEADER_LEN + FRAME_TX_RCDATA1_LEN);
  if (crc != frame->crc1) {
    return 3;
  }
  crc_accumulate_buf(&crc,
                     (const uint8_t *)frame + FRAME_TX_RX_HEADER_LEN +
                         FRAME_TX_RCDATA1_LEN,
                     FRAME_TX_RX_LEN - FRAME_TX_RX_HEADER_LEN -
                         FRAME_TX_RCDATA1_LEN - 2);
  if (crc != frame->crc) {
    return 4;
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
  uint16_t crc;
  crc_init(&crc);
  crc_accumulate_buf(&crc, (uint8_t *)frame, FRAME_TX_RX_LEN - 2);
  frame->crc = crc;
}
#endif

#if MLRS_OTA_IS_TX
static uint8_t check_rx_frame(const tRxFrame *frame) {
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
  g_connected = false;
  g_miss_streak = 0;
  g_fhss_pending_hops = 0;
  Radio.RXnb();
#endif
  printf("[mLRS] %s / %s (%u us, %u hops%s)\n",
         g_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz", current_rate_cfg()->name,
         (unsigned)current_rate_cfg()->interval_us, (unsigned)g_fhss_count,
         tight_slot() ? ", tlm/2" : "");
}

static void restore_elrs_radio() {
  const expresslrs_mod_settings_s *mod = ExpressLRS_currAirRate_Modparams;
  if (mod == nullptr) {
    return;
  }
  const uint32_t freq = FHSSgetInitialFreq();
#if defined(SIW917_ELRS_USE_UPSTREAM_TX_MAIN) || defined(SIW917_ELRS_TARGET_TX)
  Radio.Config(mod->bw, mod->sf, mod->cr, freq, mod->PreambleLen, false,
               mod->PayloadLength,
               static_cast<RadioBandMod::Combined>(mod->radio_type), UID[4],
               UID[5], SX12XX_Radio_All);
#else
  const bool fsk = (mod->radio_type == RADIO_TYPE_LR1121_GFSK_900) ||
                   (mod->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
  Radio.Config(mod->bw, mod->sf, mod->cr, freq, mod->PreambleLen, false,
               mod->PayloadLength, fsk, UID[4], UID[5], SX12XX_Radio_All);
#endif
  calib_image_for_freq(freq);
  Radio.SetFrequencyReg(freq, SX12XX_Radio_All, false, 0);
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
    g_connected = true;
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
  /* Keep hopping. Resetting hop/follow here desyncs 50 Hz FSK after a
   * short downlink gap and causes connect/disconnect flaps. */
  if (g_connected && (millis() - g_last_rx_ms) > MLRS_LOST_MS) {
    g_connected = false;
    setConnectionState(disconnected);
    g_print_disconnected = 1;
#if MLRS_OTA_IS_TX
    siw917_tx_note_mlrs_link_lost();
#endif
  }
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
  if (g_last_rx_ms != 0 &&
      (millis() - g_last_rx_ms) > (interval_ms + 8U)) {
    note_missed_rx();
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
    (void)otaConnector.takeQueuedPayload(payload, &payload_len,
                                         FRAME_TX_PAYLOAD_LEN);
  }
  pack_tx_frame(&g_tx_frame, rc, g_fhss_i, g_seq++, g_lq, false, payload,
                payload_len);
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
  const uint8_t fail = check_rx_frame(&g_rx_frame);
  if (fail == 0) {
    ++g_rx_ok;
    note_valid_rx();
    if (g_rx_frame.status.payload_len >= CRSF_MIN_PACKET_LEN) {
      crsfRouter.processMessage(&otaConnector,
                                (crsf_header_t *)g_rx_frame.payload);
      sendCRSFTelemetryToBackpack(g_rx_frame.payload);
    }
  } else {
    g_last_rx_fail = fail;
    if (fail == 1) {
      ++g_rx_junk;
    } else {
      ++g_rx_fail;
      note_missed_rx();
    }
  }
  return true;
}

static void notify_rx_protocol(uint8_t protocol) {
  static uint8_t payload[4];
  payload[0] = MSP_ELRS_SET_AIR_PROTOCOL;
  payload[1] = protocol;
  payload[2] = advertised_rate();
  payload[3] = sanitize_band(g_band);
  DataUlSender.SetDataToTransmit(payload, sizeof(payload));
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

static void mlrs_rx_send_tlm() {
  uint8_t payload[FRAME_RX_PAYLOAD_LEN] = {};
  const uint8_t payload_len =
      mlrs_elrs_rx_take_downlink(payload, FRAME_RX_PAYLOAD_LEN);
  pack_rx_frame(&g_rx_frame, g_seq++, g_lq, false, payload, payload_len);
  Radio.TXnb((uint8_t *)&g_rx_frame, false, nullptr, SX12XX_Radio_All);
  ++g_tx_sent;
}

static void mlrs_rx_done_tx() {
  g_fhss_do_hop = 1;
  g_fhss_arm_rx = 1;
}

static bool mlrs_rx_rx_done(SX12xxDriverCommon::rx_status) {
  memcpy(&g_tx_frame, Radio.RXdataBuffer, sizeof(g_tx_frame));
  const uint8_t fail = check_tx_frame(&g_tx_frame);
  if (fail != 0) {
    g_last_rf_ms = millis();
    g_last_rx_fail = fail;
    if (fail == 1) {
      ++g_rx_junk;
    } else {
      ++g_rx_fail;
    }
    /* Leave the radio in FS. Re-arming RX here lets SF5 2.4 false-locks
     * chain into a junk storm and miss the real uplink. Tock re-arms
     * (and hops once following). */
    g_rx_need_rearm = 1;
    return false;
  }
  ++g_rx_ok;
  note_valid_rx();
  g_last_rf_ms = g_last_rx_ms;
  g_slot_rx = 1;
  g_rx_need_rearm = 0;
  g_miss_streak = 0;
  fhss_set_index(g_tx_frame.status.fhss_index);
  g_fhss_follow = true;
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
  /* 20 ms slots (FSK50 and 2.4 50 Hz) cannot TX downlink every packet:
   * two-way LoRa ToA is ~16 ms, so CRSF after every TX overruns the next
   * uplink and desyncs hops. Skip tlm every other packet and hop now. */
  if (tight_slot() && ((g_rx_ok & 1U) != 0U)) {
    mlrs_rx_hop_listen();
  } else {
    mlrs_rx_send_tlm();
  }
  uint16_t rc[16];
  unpack_tx_rc(&g_tx_frame, rc);
  apply_rc_to_elrs(rc);
  if (g_tx_frame.status.frame_type == FRAME_TYPE_TX_RX_CMD ||
      (g_tx_frame.status.payload_len == 1 &&
       g_tx_frame.payload[0] == MLRS_SWITCH_CMD)) {
    printf("[mLRS] rx switch cmd -> ELRS type=%u plen=%u\n",
           (unsigned)g_tx_frame.status.frame_type,
           (unsigned)g_tx_frame.status.payload_len);
    g_pending_protocol = ELRS_AIR_PROTOCOL_ELRS;
    g_apply_at_ms = millis() + 50;
    g_fhss_follow = false;
  } else if (g_tx_frame.status.payload_len >= 3 &&
             g_tx_frame.payload[0] == MLRS_BAND_CMD) {
    printf("[mLRS] rx band cmd -> %s / %s\n",
           g_tx_frame.payload[1] == MLRS_BAND_24 ? "2.4GHz" : "915MHz",
           kRates[sanitize_band(g_tx_frame.payload[1])]
                 [sanitize_rate(g_tx_frame.payload[2])]
                     .name);
  } else if (g_tx_frame.status.payload_len >= 2 &&
             g_tx_frame.payload[0] == MLRS_RATE_CMD) {
    printf("[mLRS] rx rate cmd -> %s spare=%u\n",
           kRates[sanitize_band(g_band)][sanitize_rate(g_tx_frame.payload[1])]
               .name,
           (unsigned)g_last_rx_spare);
  } else if (g_tx_frame.status.payload_len > 0) {
    mlrs_elrs_rx_accept_uplink(g_tx_frame.payload,
                               (uint8_t)g_tx_frame.status.payload_len);
  }
  return true;
}

static void mlrs_rx_tock() {
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
#else
  Radio.TXdoneCallback = mlrs_rx_done_tx;
  Radio.RXdoneCallback = mlrs_rx_rx_done;
  hwTimer::callbackTock = mlrs_rx_tock;
#endif
  hwTimer::updateInterval(current_rate_cfg()->interval_us);
  g_seq = 0;
  g_valid_window = 0;
  g_connected = false;
  g_tx_sent = 0;
  g_rx_ok = 0;
  g_rx_fail = 0;
  g_rx_junk = 0;
  g_last_rx_fail = 0;
  g_mlrs_started_ms = millis();
  g_last_rx_ms = 0;
#if MLRS_OTA_IS_TX
  g_tx_send_pending = 0;
  g_logged_switch_cmd = false;
#else
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
#if MLRS_OTA_IS_TX
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
  Radio.RXnb();
#endif
  printf("[mLRS] air protocol started (%s/%s, sync=0x%04X freq=%lu hop=0/%u)\n",
         g_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz", current_rate_cfg()->name,
         (unsigned)g_sync_word, (unsigned long)fhss_curr(),
         (unsigned)g_fhss_count);
  printf("[mLRS] CRSF frames in payload (no stubborn)\n");
}

static void stop_mlrs() {
  if (!g_active) {
    return;
  }
  hwTimer::stop();
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
  FHSSsetCurrIndex(0);
  OtaNonce = 0;
#endif
  delay(5);
  g_active = false;
  g_protocol = ELRS_AIR_PROTOCOL_ELRS;
  flush_mlrs_config();
  setConnectionState(disconnected);
  hwTimer::resume();
#if !MLRS_OTA_IS_TX
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

extern "C" bool mlrs_ota_is_connected(void) { return g_active && g_connected; }

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
  if (g_active && g_fhss_arm_rx) {
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
  if (g_active && g_connected && (millis() - g_last_rx_ms) > MLRS_LOST_MS) {
    note_missed_rx();
  }
  if (g_active && (millis() - g_last_hb_ms) > 2000) {
    g_last_hb_ms = millis();
    printf("[mLRS] waiting lq=%u connected=%u sent=%u rxok=%u rxcrc=%u "
           "junk=%u last=%u rate=%s adv=%s freq=%lu hop=%u rssi=%d rqly=%u\n",
           (unsigned)g_lq, (unsigned)g_connected, (unsigned)g_tx_sent,
           (unsigned)g_rx_ok, (unsigned)g_rx_fail, (unsigned)g_rx_junk,
           (unsigned)g_last_rx_fail,
           current_rate_cfg()->name,
           kRates[advertised_band()][advertised_rate()].name,
           (unsigned long)fhss_curr(), (unsigned)g_fhss_i,
           (int)(int8_t)linkStats.uplink_RSSI_1,
           (unsigned)linkStats.uplink_Link_quality);
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
  (void)mlrs_ota_set_protocol(data[1]);
  return true;
}
