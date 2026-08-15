#include "mlrs_mbridge.h"
#include "mlrs_mbridge_params.h"
#include "mlrs_ota.h"

#include "Arduino.h"
#include "targets.h"
#include "CRSFEndpoint.h"
#include "CRSFRouter.h"
#include "POWERMGNT.h"
#include "crsf_protocol.h"
#include "siw917_mavlink_backpack.h"

#include <new>
#include <stdio.h>
#include <string.h>

extern CRSFRouter crsfRouter;

#define MBRIDGE_STX1 'O'
#define MBRIDGE_STX2 'W'
#define MBRIDGE_COMMANDPACKET_STX 0xA0
#define MBRIDGE_COMMANDPACKET_MASK 0xE0

#define CRSF_FRAMETYPE_MBRIDGE_TO_MODULE 0x81
#define CRSF_FRAMETYPE_MBRIDGE_TO_RADIO 0x82

enum {
  MBRIDGE_CMD_TX_LINK_STATS = 2,
  MBRIDGE_CMD_REQUEST_INFO = 3,
  MBRIDGE_CMD_DEVICE_ITEM_TX = 4,
  MBRIDGE_CMD_DEVICE_ITEM_RX = 5,
  MBRIDGE_CMD_PARAM_ITEM = 7,
  MBRIDGE_CMD_PARAM_ITEM2 = 8,
  MBRIDGE_CMD_PARAM_ITEM3_4 = 9,
  MBRIDGE_CMD_REQUEST_CMD = 10,
  MBRIDGE_CMD_INFO = 11,
  MBRIDGE_CMD_PARAM_SET = 12,
  MBRIDGE_CMD_PARAM_STORE = 13,
  MBRIDGE_CMD_BIND_START = 14,
  MBRIDGE_CMD_BIND_STOP = 15,
  MBRIDGE_CMD_MODELID_SET = 16,
  MBRIDGE_CMD_SYSTEM_BOOTLOADER = 17,
  MBRIDGE_CMD_FLASH_ESP = 18,
};

enum {
  MBRIDGE_PARAM_TYPE_UINT8 = 0,
  MBRIDGE_PARAM_TYPE_INT8 = 1,
  MBRIDGE_PARAM_TYPE_LIST = 4,
  MBRIDGE_PARAM_TYPE_STR6 = 5,
};

#define MLRS_MB_VERSION_U16 0x1103 /* v1.4.03 */
#define MLRS_MB_LAYOUT_U16 0x1101  /* v1.4.01 */
#define MLRS_MB_OUT_Q 8
#define MLRS_MB_CMD_PAYLOAD_MAX 24

#define MSK_HIDE 0x0000
#define MSK_ALL 0xFFFF

struct ParamDef {
  uint8_t type;
  const char *name;
  const char *optstr;
  const char *unit;
  int16_t minv;
  int16_t maxv;
  uint16_t mask;
  int16_t def;
};

static const ParamDef kParams[MLRS_P_COUNT] = {
    {MBRIDGE_PARAM_TYPE_STR6, "Bind Phrase", "", "", 0, 0, MSK_ALL, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Mode", "50 Hz,31 Hz,19 Hz,FLRC,FSK,19 Hz 7x",
     "", 0, 5, MSK_ALL, 1},
    {MBRIDGE_PARAM_TYPE_LIST, "RF Band",
     "2.4,915 FCC,868,433,70,866 IN,915+2.4,868+2.4", "", 0, 7, 0x0003, 1},
    {MBRIDGE_PARAM_TYPE_LIST, "RF Ortho", "off,1/3,2/3,3/3", "", 0, 3, MSK_HIDE,
     0},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Power", "10 mW,25 mW,50 mW,100 mW", "", 0, 3,
     0x000F, 3},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Diversity",
     "enabled,antenna1,antenna2,r:en t:ant1,r:en t:ant2", "", 0, 4, 0x0002, 1},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Protocol", "mLRS,ELRS", "", 0, 1, 0x0003, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Ch Source", "none,crsf,in,mbridge", "", 0, 3,
     0x0002, 1},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Ch Order", "AETR,TAER,ETAR", "", 0, 2, 0x0007,
     0},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx In Mode", "sbus,sbus inv", "", 0, 1, MSK_HIDE,
     0},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Ser Port",
     "serial,wbridge,serial2,com,mbridge", "", 0, 4, 0x0002, 1},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Ser Baudrate", "57600,115200,230400", "", 0, 2,
     0x0007, 1},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Ser Port2", "none,serial,wbridge,serial2", "",
     0, 3, 0x0001, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Ser Baudrate2", "57600,115200,230400", "", 0,
     2, MSK_HIDE, 1},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Snd RadioStat", "off,1 Hz", "", 0, 1, 0x0003,
     1},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Mav Component", "off,enabled", "", 0, 1,
     0x0003, 1},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Power Sw Ch",
     "off,5,6,7,8,9,10,11,12,13,14,15,16", "", 0, 12, MSK_ALL, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Tx Buzzer", "off,LP,rxLQ", "", 0, 2, MSK_HIDE, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx Protocol", "CRSF,SBUS,SUMD,MAVLink", "", 0, 3,
     0x000F, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx Active Mode", "CRSF,SBUS,SUMD,MAVLink", "", 0,
     3, 0x0001, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx SBUS failsafe", "No Pulses,Last Pos", "", 0, 1,
     0x0003, 0},
    {MBRIDGE_PARAM_TYPE_UINT8, "Rx Target SysID", "", "", 1, 255, MSK_ALL, 1},
    {MBRIDGE_PARAM_TYPE_UINT8, "Rx Source SysID", "", "", 1, 255, MSK_ALL, 255},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx Tlm Off", "Off,On", "", 0, 1, 0x0003, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx Tlm Power", "10,25,50,100,MatchTX", "mW", 0, 4,
     0x001F, 4},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx BLE RemoteID", "Off,On", "", 0, 1, 0x0003, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx WiFi Mode", "Idle,Start", "", 0, 1, 0x0003, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx Team Ch",
     "AUX2,AUX3,AUX4,AUX5,AUX6,AUX7,AUX8,AUX9,AUX10,AUX11,AUX12", "", 0, 10,
     0x07FF, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx Team Pos",
     "Disabled,1/Low,2,3,Mid,4,5,6/High", "", 0, 7, 0x00FF, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx Bind Storage",
     "Persistent,Volatile,Returnable,Administered", "", 0, 3, 0x000F, 0},
    {MBRIDGE_PARAM_TYPE_LIST, "Rx Bind Mode", "Idle,Enter", "", 0, 1, 0x0003, 0},
    {MBRIDGE_PARAM_TYPE_UINT8, "Rx Model Id", "", "", 0, 64, MSK_ALL, 0},
};

static_assert(sizeof(kParams) / sizeof(kParams[0]) == MLRS_P_COUNT,
              "mLRS param table size");

static char g_bind[7] = "mlrs91";
static int16_t g_val[MLRS_P_COUNT];
static uint8_t g_out_cmd[MLRS_MB_OUT_Q];
static uint8_t g_out_len[MLRS_MB_OUT_Q];
static uint8_t g_out_pl[MLRS_MB_OUT_Q][MLRS_MB_CMD_PAYLOAD_MAX];
static uint8_t g_out_head = 0;
static uint8_t g_out_count = 0;
static uint32_t g_out_last_ms = 0;
static volatile uint8_t g_air[32];
static volatile uint8_t g_air_len = 0;
static volatile uint8_t g_air_repeat = 0;
static bool g_inited = false;
static bool g_logged = false;

static uint8_t cmd_payload_len(uint8_t cmd) {
  switch (cmd) {
  case MBRIDGE_CMD_TX_LINK_STATS:
    return 22;
  case MBRIDGE_CMD_DEVICE_ITEM_TX:
  case MBRIDGE_CMD_DEVICE_ITEM_RX:
  case MBRIDGE_CMD_PARAM_ITEM:
  case MBRIDGE_CMD_PARAM_ITEM2:
  case MBRIDGE_CMD_PARAM_ITEM3_4:
  case MBRIDGE_CMD_INFO:
    return 24;
  case MBRIDGE_CMD_REQUEST_CMD:
    return 18;
  case MBRIDGE_CMD_PARAM_SET:
    return 7;
  case MBRIDGE_CMD_MODELID_SET:
    return 3;
  default:
    return 0;
  }
}

static void copy_pad(char *dst, const char *src, uint8_t n) {
  memset(dst, 0, n);
  if (src == nullptr) {
    return;
  }
  strncpy(dst, src, n);
}

static uint8_t native_mode() {
  const uint8_t band = mlrs_ota_get_band();
  const uint8_t rate = mlrs_ota_get_rate();
  if (band == MLRS_BAND_24) {
    return rate;
  }
  if (rate == 0) {
    return 1;
  }
  if (rate == 1) {
    return 2;
  }
  return 4;
}

static uint8_t native_band() {
  return mlrs_ota_get_band() == MLRS_BAND_24 ? 0 : 1;
}

static uint8_t tx_power_idx() {
  const uint8_t pwr = (uint8_t)POWERMGNT::currPower();
  return (pwr > 3) ? 3 : pwr;
}

static uint8_t param_value(uint8_t idx) {
  switch (idx) {
  case MLRS_P_MODE:
    return native_mode();
  case MLRS_P_BAND:
    return native_band();
  case MLRS_P_TX_POWER:
    return tx_power_idx();
  case MLRS_P_TX_PROTO:
    return mlrs_ota_is_active() ? 0 : 1;
  case MLRS_P_TX_MAV_COMP:
    return siw917_mavlink_backpack_get_lua_enabled() ? 1 : 0;
  default:
    if (idx >= MLRS_P_COUNT) {
      return 0;
    }
    return (uint8_t)g_val[idx];
  }
}

static uint16_t param_mask(uint8_t idx) {
  if (idx >= MLRS_P_COUNT) {
    return 0;
  }
  switch (idx) {
  case MLRS_P_MODE:
    return (mlrs_ota_get_band() == MLRS_BAND_24) ? 0x0007 : 0x0016;
  case MLRS_P_RX_ACTIVE:
    return (uint16_t)(1u << (uint8_t)g_val[MLRS_P_RX_PROTOCOL]);
  default:
    return kParams[idx].mask;
  }
}

static void queue_air(uint8_t cmd, const uint8_t *payload, uint8_t plen) {
  if (plen > 24) {
    plen = 24;
  }
  g_air[0] = MLRS_AIR_MBRIDGE;
  g_air[1] = cmd;
  if (plen != 0 && payload != nullptr) {
    memcpy((void *)(g_air + 2), payload, plen);
  }
  g_air_len = (uint8_t)(2 + plen);
  g_air_repeat = 1;
  printf("[mLRS] MBridge air cmd=%u idx=%u val=%u len=%u\n", (unsigned)cmd,
         (payload != nullptr && plen > 0) ? (unsigned)payload[0] : 0,
         (payload != nullptr && plen > 1) ? (unsigned)payload[1] : 0,
         (unsigned)g_air_len);
}

static void queue_cmd(uint8_t cmd, const void *payload) {
  if (g_out_count >= MLRS_MB_OUT_Q) {
    return;
  }
  const uint8_t slot = (uint8_t)((g_out_head + g_out_count) % MLRS_MB_OUT_Q);
  const uint8_t plen = cmd_payload_len(cmd);
  g_out_cmd[slot] = cmd;
  g_out_len[slot] = plen;
  memset(g_out_pl[slot], 0, MLRS_MB_CMD_PAYLOAD_MAX);
  if (payload != nullptr && plen != 0) {
    memcpy(g_out_pl[slot], payload, plen);
  }
  ++g_out_count;
}

static void send_to_radio(uint8_t cmd, const uint8_t *payload, uint8_t plen) {
  uint8_t buf[64] = {};
  buf[3] =
      (uint8_t)(MBRIDGE_COMMANDPACKET_STX + (cmd & ~MBRIDGE_COMMANDPACKET_MASK));
  if (plen != 0 && payload != nullptr) {
    memcpy(buf + 4, payload, plen);
  }
  const uint8_t frameSize = (uint8_t)(plen + 3);
  crsfRouter.SetHeaderAndCrc(
      reinterpret_cast<crsf_header_t *>(buf),
      static_cast<crsf_frame_type_e>(CRSF_FRAMETYPE_MBRIDGE_TO_RADIO),
      frameSize);
  crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER,
                              reinterpret_cast<crsf_header_t *>(buf));
}

static void queue_device_tx() {
  uint8_t item[24] = {};
  item[0] = (uint8_t)(MLRS_MB_VERSION_U16 & 0xFF);
  item[1] = (uint8_t)(MLRS_MB_VERSION_U16 >> 8);
  item[2] = (uint8_t)(MLRS_MB_LAYOUT_U16 & 0xFF);
  item[3] = (uint8_t)(MLRS_MB_LAYOUT_U16 >> 8);
  copy_pad(reinterpret_cast<char *>(item + 4), "SiW917-TX", 20);
  queue_cmd(MBRIDGE_CMD_DEVICE_ITEM_TX, item);
}

static void queue_device_rx() {
  uint8_t item[24] = {};
  if (mlrs_ota_is_connected()) {
    item[0] = (uint8_t)(MLRS_MB_VERSION_U16 & 0xFF);
    item[1] = (uint8_t)(MLRS_MB_VERSION_U16 >> 8);
    item[2] = (uint8_t)(MLRS_MB_LAYOUT_U16 & 0xFF);
    item[3] = (uint8_t)(MLRS_MB_LAYOUT_U16 >> 8);
    copy_pad(reinterpret_cast<char *>(item + 4), "SiW917-RX", 20);
  }
  queue_cmd(MBRIDGE_CMD_DEVICE_ITEM_RX, item);
}

static void queue_info() {
  uint8_t info[24] = {};
  const int16_t sens =
      (mlrs_ota_get_band() == MLRS_BAND_24) ? (int16_t)-105 : (int16_t)-120;
  info[0] = (uint8_t)(sens & 0xFF);
  info[1] = (uint8_t)((uint16_t)sens >> 8);
  info[2] = 0x01; /* has_status, not binding */
  info[3] = (int8_t)POWERMGNT::getPowerIndBm();
  info[4] = info[3];
  info[5] = mlrs_ota_is_connected() ? 0x01 : 0x00;
  info[6] = 0;    /* tx_config_id */
  info[7] = 0x11; /* tx/rx diversity = antenna1 */
  info[8] = MLRS_P_COUNT;
  {
    uint8_t skip = 0;
    uint8_t hops = 0;
    uint32_t mask = 0;
    mlrs_ota_hop_skip_info(&skip, &hops, &mask);
    info[9] = skip;
    info[10] = hops;
    info[11] = (uint8_t)mask;
    info[12] = (uint8_t)(mask >> 8);
    info[13] = (uint8_t)(mask >> 16);
    info[14] = (uint8_t)(mask >> 24);
    uint8_t rate = 0;
    uint8_t band = 0;
    uint8_t ul_plen = 0;
    uint8_t dl_plen = 0;
    mlrs_ota_link_rate_info(&rate, &band, &ul_plen, &dl_plen);
    info[15] = rate;
    info[16] = band;
    info[17] = ul_plen;
    info[18] = dl_plen;
  }
  queue_cmd(MBRIDGE_CMD_INFO, info);
}

static void queue_param_items(uint8_t idx) {
  if (idx >= MLRS_P_COUNT) {
    uint8_t item[24] = {};
    item[0] = 255;
    queue_cmd(MBRIDGE_CMD_PARAM_ITEM, item);
    return;
  }

  const ParamDef *p = &kParams[idx];
  uint8_t item[24] = {};
  item[0] = idx;
  item[1] = p->type;
  copy_pad(reinterpret_cast<char *>(item + 2), p->name, 16);
  if (p->type == MBRIDGE_PARAM_TYPE_STR6) {
    copy_pad(reinterpret_cast<char *>(item + 18), g_bind, 6);
  } else {
    item[18] = param_value(idx);
  }
  queue_cmd(MBRIDGE_CMD_PARAM_ITEM, item);

  uint8_t item2[24] = {};
  item2[0] = idx;
  if (p->type == MBRIDGE_PARAM_TYPE_LIST) {
    const uint16_t mask = param_mask(idx);
    item2[1] = (uint8_t)(mask & 0xFF);
    item2[2] = (uint8_t)(mask >> 8);
    copy_pad(reinterpret_cast<char *>(item2 + 3), p->optstr, 21);
  } else if (p->type == MBRIDGE_PARAM_TYPE_INT8 ||
             p->type == MBRIDGE_PARAM_TYPE_UINT8) {
    item2[1] = (uint8_t)p->minv;
    item2[3] = (uint8_t)p->maxv;
    item2[5] = (uint8_t)p->def;
    copy_pad(reinterpret_cast<char *>(item2 + 7), p->unit, 6);
  }
  queue_cmd(MBRIDGE_CMD_PARAM_ITEM2, item2);

  if (p->type != MBRIDGE_PARAM_TYPE_LIST || p->optstr == nullptr) {
    return;
  }
  const size_t optlen = strlen(p->optstr);
  if (optlen < 21) {
    return;
  }
  uint8_t item3[24] = {};
  item3[0] = idx;
  copy_pad(reinterpret_cast<char *>(item3 + 1), p->optstr + 21, 23);
  queue_cmd(MBRIDGE_CMD_PARAM_ITEM3_4, item3);
  if (optlen < 21 + 23) {
    return;
  }
  uint8_t item4[24] = {};
  item4[0] = (uint8_t)(idx + 128);
  copy_pad(reinterpret_cast<char *>(item4 + 1), p->optstr + 21 + 23, 23);
  queue_cmd(MBRIDGE_CMD_PARAM_ITEM3_4, item4);
}

static bool apply_mode(uint8_t mode) {
  uint8_t rate;
  if (mlrs_ota_get_band() == MLRS_BAND_24) {
    if (mode > 2) {
      return false;
    }
    rate = mode;
  } else if (mode == 1) {
    rate = 0;
  } else if (mode == 2) {
    rate = 1;
  } else if (mode == 4) {
    rate = 2;
  } else {
    return false;
  }
  return mlrs_ota_set_rate(rate);
}

static bool apply_band(uint8_t nb) {
  if (nb > 1) {
    return false;
  }
  return mlrs_ota_set_band(nb == 0 ? MLRS_BAND_24 : MLRS_BAND_915);
}

static void apply_param_set(const uint8_t *pl, uint8_t len) {
  if (pl == nullptr || len < 2) {
    return;
  }
  const uint8_t idx = pl[0];
  if (idx >= MLRS_P_COUNT) {
    return;
  }
  if (idx == MLRS_P_BIND) {
    copy_pad(g_bind, reinterpret_cast<const char *>(pl + 1), 6);
    g_bind[6] = 0;
    return;
  }
  const uint8_t val = pl[1];
  if (kParams[idx].type == MBRIDGE_PARAM_TYPE_INT8) {
    g_val[idx] = (int8_t)val;
  } else {
    g_val[idx] = val;
  }

  switch (idx) {
  case MLRS_P_MODE:
    (void)apply_mode(val);
    break;
  case MLRS_P_BAND:
    (void)apply_band(val);
    break;
  case MLRS_P_TX_POWER:
    if (val <= 3) {
      POWERMGNT::setPower(static_cast<PowerLevels_e>(val));
    }
    break;
  case MLRS_P_TX_PROTO:
    if (val == 1) {
      (void)mlrs_ota_set_protocol(ELRS_AIR_PROTOCOL_ELRS);
    }
    break;
  case MLRS_P_TX_MAV_COMP:
    siw917_mavlink_backpack_set_lua_enabled(val != 0);
    break;
  case MLRS_P_RX_PROTOCOL:
    g_val[MLRS_P_RX_ACTIVE] = val;
    break;
  default:
    break;
  }

  if (idx >= MLRS_P_RX_PROTOCOL && idx != MLRS_P_RX_ACTIVE) {
    queue_air(MBRIDGE_CMD_PARAM_SET, pl, 7);
    if (idx == MLRS_P_RX_WIFI || idx == MLRS_P_RX_BIND_MODE) {
      g_val[idx] = 0;
    }
  }
}

static void handle_cmd(uint8_t cmd, const uint8_t *pl, uint8_t len) {
  if (!g_logged) {
    g_logged = true;
    printf("[mLRS] MBridge Lua on CRSF 129/130\n");
  }
  switch (cmd) {
  case MBRIDGE_CMD_REQUEST_INFO:
    queue_device_tx();
    queue_device_rx();
    queue_info();
    break;
  case MBRIDGE_CMD_REQUEST_CMD:
    if (len < 1) {
      break;
    }
    switch (pl[0]) {
    case MBRIDGE_CMD_DEVICE_ITEM_TX:
      queue_device_tx();
      break;
    case MBRIDGE_CMD_DEVICE_ITEM_RX:
      queue_device_rx();
      break;
    case MBRIDGE_CMD_INFO:
      queue_info();
      break;
    case MBRIDGE_CMD_REQUEST_INFO:
      queue_device_tx();
      queue_device_rx();
      queue_info();
      break;
    case MBRIDGE_CMD_PARAM_ITEM:
      queue_param_items(len > 1 ? pl[1] : 0);
      break;
    default:
      break;
    }
    break;
  case MBRIDGE_CMD_PARAM_SET:
    apply_param_set(pl, len);
    break;
  case MBRIDGE_CMD_PARAM_STORE:
    queue_air(MBRIDGE_CMD_PARAM_STORE, nullptr, 0);
    printf("[mLRS] MBridge PARAM_STORE\n");
    break;
  case MBRIDGE_CMD_BIND_START:
  case MBRIDGE_CMD_BIND_STOP:
  case MBRIDGE_CMD_MODELID_SET:
  case MBRIDGE_CMD_SYSTEM_BOOTLOADER:
  case MBRIDGE_CMD_FLASH_ESP:
    break;
  default:
    break;
  }
}

class MlrsMbridgeEndpoint final : public CRSFEndpoint {
public:
  MlrsMbridgeEndpoint() : CRSFEndpoint(CRSF_ADDRESS_RESERVED1) {}

  bool handleRaw(const crsf_header_t *message) override {
    if (message == nullptr || !mlrs_ota_is_active()) {
      return false;
    }
    if ((uint8_t)message->type != CRSF_FRAMETYPE_MBRIDGE_TO_MODULE) {
      return false;
    }
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(message);
    const uint8_t body = (uint8_t)(message->frame_size - 2);
    if (body < 3) {
      return true;
    }
    const uint8_t *p = raw + 3;
    if (p[0] != MBRIDGE_STX1 || p[1] != MBRIDGE_STX2) {
      return true;
    }
    const uint8_t cmd = (uint8_t)(p[2] & ~MBRIDGE_COMMANDPACKET_MASK);
    const uint8_t plen = (uint8_t)((body > 3) ? (body - 3) : 0);
    handle_cmd(cmd, p + 3, plen);
    mlrs_mbridge_poll();
    return true;
  }

  void handleMessage(const crsf_header_t *) override {}
};

alignas(MlrsMbridgeEndpoint) static uint8_t g_ep_mem[sizeof(MlrsMbridgeEndpoint)];
static MlrsMbridgeEndpoint *g_ep = nullptr;
static bool g_registered = false;

void mlrs_mbridge_init(void) {
  if (g_inited) {
    return;
  }
  /* SiW917 does not run C++ global constructors. A static CRSFEndpoint
   * would have a null vtable; broadcast DEVICE_PING then crashes the TX
   * task and ExpressLRS.lua never receives DEVICE_INFO. */
  g_ep = ::new (static_cast<void *>(g_ep_mem)) MlrsMbridgeEndpoint();
  for (uint8_t i = 0; i < MLRS_P_COUNT; ++i) {
    g_val[i] = kParams[i].def;
  }
  g_val[MLRS_P_TX_CH_SRC] = 1;
  g_val[MLRS_P_TX_SER_PORT] = 1;
  g_val[MLRS_P_TX_DIV] = 1;
  g_inited = true;
  printf("[mLRS] MBridge endpoint constructed\n");
}

void mlrs_mbridge_poll(void) {
  if (g_ep != nullptr && !g_registered && mlrs_ota_is_active()) {
    crsfRouter.addEndpoint(g_ep);
    g_registered = true;
    printf("[mLRS] MBridge attached to CRSF router\n");
  }
  if (g_out_count == 0) {
    return;
  }
  const uint32_t now = millis();
  if (g_out_last_ms != 0 && (int32_t)(now - g_out_last_ms) < 10) {
    return;
  }
  const uint8_t slot = g_out_head;
  send_to_radio(g_out_cmd[slot], g_out_pl[slot], g_out_len[slot]);
  g_out_head = (uint8_t)((g_out_head + 1) % MLRS_MB_OUT_Q);
  --g_out_count;
  g_out_last_ms = now;
}

bool mlrs_mbridge_take_air(uint8_t *dst, uint8_t *len, uint8_t max) {
  if (dst == nullptr || len == nullptr || g_air_len == 0 || g_air_len > max) {
    return false;
  }
  memcpy(dst, (const void *)g_air, g_air_len);
  *len = g_air_len;
  if (g_air_repeat > 1) {
    --g_air_repeat;
  } else {
    g_air_repeat = 0;
    g_air_len = 0;
  }
  return true;
}

void mlrs_mbridge_accept_downlink(const uint8_t *src, uint8_t len) {
  if (src == nullptr || len < 3 || src[0] != MLRS_AIR_MBRIDGE) {
    return;
  }
  if (src[1] != MLRS_AIR_RX_STATE) {
    return;
  }
  const uint8_t nvals = (uint8_t)(len - 2);
  static uint8_t last_ble = 0xFF;
  for (uint8_t i = 0; i < nvals; ++i) {
    const uint8_t idx = (uint8_t)(MLRS_P_RX_PROTOCOL + i);
    if (idx >= MLRS_P_COUNT) {
      break;
    }
    if (idx == MLRS_P_RX_WIFI || idx == MLRS_P_RX_BIND_MODE) {
      continue;
    }
    g_val[idx] = src[2 + i];
    if (idx == MLRS_P_RX_PROTOCOL) {
      g_val[MLRS_P_RX_ACTIVE] = src[2 + i];
    }
    if (idx == MLRS_P_RX_BLE_RID && src[2 + i] != last_ble) {
      last_ble = src[2 + i];
      printf("[mLRS] RX state BLE RemoteID %s\n", last_ble ? "On" : "Off");
    }
  }
}
