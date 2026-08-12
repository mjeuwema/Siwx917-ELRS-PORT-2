#include "mlrs_ota.h"

#include "Arduino.h"
#include "FHSS.h"
#include "LR1121.h"
#include "LR1121_Regs.h"
#include "common.h"
#include "device.h"
#include "hwTimer.h"

#include <stdio.h>
#include <string.h>

extern "C" {
#include "elrs_config.h"
}

extern LR1121Driver Radio;
extern uint8_t UID[UID_LEN];
extern uint32_t ChannelData[CRSF_NUM_CHANNELS];
extern expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;

#if defined(SIW917_ELRS_TARGET_TX) || defined(TARGET_TX)
#define MLRS_OTA_IS_TX 1
#include "stubborn_sender.h"
extern StubbornSender DataUlSender;
#else
#define MLRS_OTA_IS_TX 0
#endif

#ifndef PACKED
#define PACKED(__Declaration__) __Declaration__ __attribute__((packed))
#endif

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
        {20000, 10, LR11XX_RADIO_GFSK_BW_312000, 50, 16, 1, 25, "50Hz FSK"},
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
};

PACKED(typedef struct {
  uint32_t seq_no : 3;
  uint32_t ack : 1;
  uint32_t frame_type : 4;
  uint32_t antenna : 1;
  uint32_t rssi_u7 : 7;
  uint32_t fhss_index_band : 1;
  uint32_t fhss_index : 6;
  uint32_t LQ_serial : 7;
  uint32_t transmit_antenna : 1;
  uint32_t spare : 2;
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
  uint32_t transmit_antenna : 1;
  uint32_t spare : 2;
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
static uint8_t g_pending_rate = 0xFF;
static uint8_t g_pending_band = 0xFF;
static uint16_t g_sync_word = 0;
static uint32_t g_fhss_seed = 0;
static uint32_t g_fhss_list[FHSS_MAX_HOPS] = {};
static uint8_t g_fhss_ch[FHSS_MAX_HOPS] = {};
static uint8_t g_fhss_count = FHSS_NUM_915;
static uint8_t g_fhss_i = 0;
static uint8_t g_seq = 0;
static uint8_t g_lq = 0;
static uint8_t g_valid_window = 0;
static uint32_t g_last_rx_ms = 0;
static bool g_connected = false;
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
  if (rate >= MLRS_RATE_COUNT) {
    return 0;
  }
  return rate;
}

static const mlrs_rate_cfg_t *current_rate_cfg() {
  return &kRates[sanitize_band(g_band)][sanitize_rate(g_rate)];
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
  cfg->reserved[ELRS_RESERVED_AIR_PROTOCOL_OFFSET] = protocol;
  (void)elrs_config_save();
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
  (void)elrs_config_save();
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
                          uint8_t fhss_index, uint8_t seq, uint8_t lq,
                          const uint8_t *payload, uint8_t payload_len) {
  memset(frame, 0, sizeof(*frame));
  if (payload_len > FRAME_TX_PAYLOAD_LEN) {
    payload_len = FRAME_TX_PAYLOAD_LEN;
  }
  frame->sync_word = g_sync_word;
  frame->status.seq_no = seq & 0x7;
  frame->status.ack = 0;
  frame->status.frame_type = FRAME_TYPE_TX;
  frame->status.antenna = 0;
  frame->status.transmit_antenna = 0;
  frame->status.rssi_u7 = rssi_u7_from_i8(Radio.LastPacketRSSI);
  frame->status.fhss_index_band = sanitize_band(g_band) & 0x1;
  frame->status.fhss_index = fhss_index & 0x3F;
  frame->status.LQ_serial = lq;
  frame->status.spare = sanitize_rate(g_rate) & 0x3;
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
  if (frame->status.frame_type != FRAME_TYPE_TX) {
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

static void pack_rx_frame(tRxFrame *frame, uint8_t seq, uint8_t lq) {
  memset(frame, 0, sizeof(*frame));
  frame->sync_word = g_sync_word;
  frame->status.seq_no = seq & 0x7;
  frame->status.ack = 1;
  frame->status.frame_type = FRAME_TYPE_RX;
  frame->status.antenna = 0;
  frame->status.transmit_antenna = 0;
  frame->status.rssi_u7 = rssi_u7_from_i8(Radio.LastPacketRSSI);
  frame->status.LQ_rc = lq;
  frame->status.LQ_serial = lq;
  frame->status.payload_len = 0;
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
#if defined(SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
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
  Radio.SetFrequencyReg(freq, SX12XX_Radio_All, false, 0);
}

static void apply_mlrs_rate() {
  if (!g_active) {
    return;
  }
  hwTimer::stop();
  Radio.SetTxIdleMode();
  fhss_generate();
  configure_mlrs_radio();
  hwTimer::updateInterval(current_rate_cfg()->interval_us);
  hwTimer::resume();
#if !MLRS_OTA_IS_TX
  Radio.RXnb();
#endif
  printf("[mLRS] %s / %s (%u us, %u hops)\n",
         g_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz", current_rate_cfg()->name,
         (unsigned)current_rate_cfg()->interval_us, (unsigned)g_fhss_count);
}

static void restore_elrs_radio() {
  const expresslrs_mod_settings_s *mod = ExpressLRS_currAirRate_Modparams;
  if (mod == nullptr) {
    return;
  }
  const uint32_t freq = FHSSgetInitialFreq();
#if defined(SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
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
  Radio.SetFrequencyReg(freq, SX12XX_Radio_All, false, 0);
}

static void note_valid_rx() {
  g_last_rx_ms = millis();
  if (g_valid_window < 250) {
    ++g_valid_window;
  }
  g_lq = (uint8_t)((g_valid_window > 100) ? 100 : g_valid_window);
  if (!g_connected && g_valid_window >= 3) {
    g_connected = true;
    connectionState = connected;
    printf("[mLRS] connected lq=%u\n", (unsigned)g_lq);
  }
}

static void note_missed_rx() {
  if (g_valid_window > 0) {
    --g_valid_window;
  }
  g_lq = (uint8_t)((g_valid_window > 100) ? 100 : g_valid_window);
  if (g_connected && (millis() - g_last_rx_ms) > 1000) {
    g_connected = false;
    connectionState = disconnected;
    printf("[mLRS] disconnected\n");
  }
}

#if MLRS_OTA_IS_TX
static void fill_rc_from_handset(uint16_t rc[16]) {
  for (uint8_t i = 0; i < 16; ++i) {
    rc[i] = rc_from_crsf((uint16_t)ChannelData[i]);
  }
}

static void mlrs_tx_send_frame() {
  uint16_t rc[16];
  fill_rc_from_handset(rc);
  uint8_t payload[1] = {0};
  uint8_t payload_len = 0;
  if (g_pending_protocol == ELRS_AIR_PROTOCOL_ELRS) {
    payload[0] = MLRS_SWITCH_CMD;
    payload_len = 1;
  }
  fhss_hop();
  pack_tx_frame(&g_tx_frame, rc, g_fhss_i, g_seq++, g_lq, payload, payload_len);
  Radio.SetFrequencyReg(fhss_curr(), SX12XX_Radio_All, false, 0);
  Radio.TXnb((uint8_t *)&g_tx_frame, false, nullptr, SX12XX_Radio_All);
}

static void mlrs_tx_tock() { mlrs_tx_send_frame(); }

static void mlrs_tx_done() { Radio.RXnb(); }

static bool mlrs_tx_rx_done(SX12xxDriverCommon::rx_status) {
  memcpy(&g_rx_frame, Radio.GetRxPayloadBuffer(), sizeof(g_rx_frame));
  if (check_rx_frame(&g_rx_frame) == 0) {
    note_valid_rx();
  } else {
    note_missed_rx();
  }
  return true;
}

static void notify_rx_protocol(uint8_t protocol) {
  static uint8_t payload[4];
  payload[0] = MSP_ELRS_SET_AIR_PROTOCOL;
  payload[1] = protocol;
  payload[2] = sanitize_rate(g_rate);
  payload[3] = sanitize_band(g_band);
  DataUlSender.SetDataToTransmit(payload, sizeof(payload));
}
#else
static void apply_rc_to_elrs(const uint16_t rc[16]) {
  for (uint8_t i = 0; i < 16; ++i) {
    ChannelData[i] = rc_to_crsf(rc[i]);
  }
}

static void mlrs_rx_send_tlm() {
  pack_rx_frame(&g_rx_frame, g_seq++, g_lq);
  Radio.TXnb((uint8_t *)&g_rx_frame, false, nullptr, SX12XX_Radio_All);
}

static void mlrs_rx_done_tx() {
  fhss_hop();
  Radio.SetFrequencyReg(fhss_curr(), SX12XX_Radio_All, true, 0);
}

static bool mlrs_rx_rx_done(SX12xxDriverCommon::rx_status) {
  memcpy(&g_tx_frame, Radio.RXdataBuffer, sizeof(g_tx_frame));
  if (check_tx_frame(&g_tx_frame) != 0) {
    note_missed_rx();
    Radio.RXnb();
    return true;
  }
  note_valid_rx();
  fhss_set_index(g_tx_frame.status.fhss_index);
  if ((g_tx_frame.status.fhss_index_band != g_band) &&
      (g_tx_frame.status.fhss_index_band < MLRS_BAND_COUNT)) {
    g_pending_band = (uint8_t)g_tx_frame.status.fhss_index_band;
  }
  if ((g_tx_frame.status.spare != g_rate) &&
      (g_tx_frame.status.spare < MLRS_RATE_COUNT)) {
    g_pending_rate = (uint8_t)g_tx_frame.status.spare;
  }
  uint16_t rc[16];
  unpack_tx_rc(&g_tx_frame, rc);
  apply_rc_to_elrs(rc);
  if (g_tx_frame.status.payload_len > 0 &&
      g_tx_frame.payload[0] == MLRS_SWITCH_CMD) {
    g_pending_protocol = ELRS_AIR_PROTOCOL_ELRS;
    g_apply_at_ms = millis() + 50;
  }
  mlrs_rx_send_tlm();
  return true;
}

static void mlrs_rx_tock() {
  const uint32_t miss_ms = (current_rate_cfg()->interval_us / 1000U) + 8U;
  if ((millis() - g_last_rx_ms) > miss_ms) {
    note_missed_rx();
    fhss_hop();
    Radio.SetFrequencyReg(fhss_curr(), SX12XX_Radio_All, true, 0);
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
  configure_mlrs_radio();

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
  g_active = true;
  g_protocol = ELRS_AIR_PROTOCOL_MLRS;
  connectionState = disconnected;
  hwTimer::resume();
#if !MLRS_OTA_IS_TX
  Radio.RXnb();
#endif
  printf("[mLRS] air protocol started (%s/%s, sync=0x%04X)\n",
         g_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz", current_rate_cfg()->name,
         (unsigned)g_sync_word);
}

static void stop_mlrs() {
  if (!g_active) {
    return;
  }
  hwTimer::stop();
  Radio.SetTxIdleMode();
  Radio.RXdoneCallback = g_saved_rx_cb;
  Radio.TXdoneCallback = g_saved_tx_cb;
  hwTimer::callbackTick = g_saved_tick;
  hwTimer::callbackTock = g_saved_tock;
  if (g_saved_interval_us != 0) {
    hwTimer::updateInterval(g_saved_interval_us);
  }
  restore_elrs_radio();
  g_active = false;
  g_protocol = ELRS_AIR_PROTOCOL_ELRS;
  connectionState = disconnected;
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
  devicesTriggerEvent(EVENT_CONFIG_MAIN_CHANGED);
}

extern "C" bool mlrs_ota_is_active(void) { return g_active; }

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
  if (protocol == ELRS_AIR_PROTOCOL_MLRS && !g_active) {
    notify_rx_protocol(protocol);
    g_pending_protocol = protocol;
    g_apply_at_ms = millis() + 400;
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
  rate = sanitize_rate(rate);
  if (rate == g_rate && g_pending_rate == 0xFF) {
    return true;
  }
  printf("[mLRS] lua/air rate -> %s\n", kRates[sanitize_band(g_band)][rate].name);
  g_rate = rate;
  g_pending_rate = 0xFF;
  save_rate(rate);
#if MLRS_OTA_IS_TX
  if (g_pending_protocol == ELRS_AIR_PROTOCOL_MLRS && !g_active) {
    notify_rx_protocol(ELRS_AIR_PROTOCOL_MLRS);
  }
#endif
  apply_mlrs_rate();
  devicesTriggerEvent(EVENT_CONFIG_MAIN_CHANGED);
  return true;
}

extern "C" bool mlrs_ota_set_band(uint8_t band) {
  band = sanitize_band(band);
  if (band == g_band && g_pending_band == 0xFF) {
    return true;
  }
  g_rate = remap_rate_for_band(g_band, band, g_rate);
  g_band = band;
  g_pending_band = 0xFF;
  save_band(band);
  save_rate(g_rate);
  printf("[mLRS] lua/air band -> %s / %s\n",
         band == MLRS_BAND_24 ? "2.4GHz" : "915MHz", current_rate_cfg()->name);
#if MLRS_OTA_IS_TX
  if (g_pending_protocol == ELRS_AIR_PROTOCOL_MLRS && !g_active) {
    notify_rx_protocol(ELRS_AIR_PROTOCOL_MLRS);
  }
#endif
  apply_mlrs_rate();
  devicesTriggerEvent(EVENT_CONFIG_MAIN_CHANGED);
  return true;
}

extern "C" void mlrs_ota_on_elrs_ready(void) {
  g_elrs_ready = true;
  g_band = config_band();
  g_rate = config_rate();
  g_protocol = config_protocol();
  printf("[mLRS] boot air protocol=%s band=%s rate=%s\n",
         g_protocol == ELRS_AIR_PROTOCOL_MLRS ? "mLRS" : "ELRS",
         g_band == MLRS_BAND_24 ? "2.4GHz" : "915MHz",
         current_rate_cfg()->name);
  if (g_protocol == ELRS_AIR_PROTOCOL_MLRS) {
    apply_protocol(ELRS_AIR_PROTOCOL_MLRS);
  }
}

extern "C" void mlrs_ota_loop(void) {
  if (g_pending_protocol != 0xFF && g_apply_at_ms != 0 &&
      (int32_t)(millis() - g_apply_at_ms) >= 0) {
    apply_protocol(g_pending_protocol);
  }
  bool config_changed = false;
  if (g_pending_band != 0xFF && g_pending_band != g_band &&
      g_pending_band < MLRS_BAND_COUNT) {
    g_band = g_pending_band;
    save_band(g_band);
    config_changed = true;
  }
  g_pending_band = 0xFF;
  if (g_pending_rate != 0xFF && g_pending_rate != g_rate &&
      g_pending_rate < MLRS_RATE_COUNT) {
    g_rate = g_pending_rate;
    save_rate(g_rate);
    config_changed = true;
  }
  g_pending_rate = 0xFF;
  if (config_changed) {
    apply_mlrs_rate();
  }
  if (g_active && g_connected && (millis() - g_last_rx_ms) > 1500) {
    note_missed_rx();
  }
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
