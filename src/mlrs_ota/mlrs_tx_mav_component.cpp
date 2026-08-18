#include "mlrs_tx_mav_component.h"

#include "Arduino.h"
#include "mlrs_mavlinkx.h"
#include "mlrs_mbridge.h"
#include "mlrs_mbridge_params.h"
#include "mlrs_ota.h"
#include "siw917_mavlink_wifi.h"

#include <stdio.h>
#include <string.h>

#define MAV_MAGIC_V1 0xFE
#define MAV_MAGIC_V2 0xFD
#define MAV_SYSID 51
#define MAV_COMPID 68
#define MAV_TYPE_GENERIC 0
#define MAV_AUTOPILOT_INVALID 8
#define MAV_MODE_FLAG_SAFETY_ARMED 128
#define MAV_STATE_ACTIVE 4
#define MAV_RESULT_ACCEPTED 0
#define MAV_RESULT_DENIED 2
#define MAV_PARAM_TYPE_UINT8 1
#define MAV_CAP_COMMAND_INT 8U
#define MAV_CAP_MAVLINK2 8192U
#define MAV_CMD_REQUEST_MESSAGE 512
#define MAV_CMD_REQUEST_PROTOCOL_VERSION 519
#define MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES 520
#define MSG_HEARTBEAT 0
#define MSG_PARAM_REQUEST_READ 20
#define MSG_PARAM_REQUEST_LIST 21
#define MSG_PARAM_VALUE 22
#define MSG_PARAM_SET 23
#define MSG_COMMAND_INT 75
#define MSG_COMMAND_LONG 76
#define MSG_COMMAND_ACK 77
#define MSG_AUTOPILOT_VERSION 148
#define CRC_HEARTBEAT 50
#define CRC_PARAM_VALUE 220
#define CRC_COMMAND_ACK 143
#define CRC_AUTOPILOT_VERSION 178
#define PARSE_MAX 288

enum {
  P_TX_POWER = 0,
  P_TX_MAV_COMP,
  P_MODE,
  P_RF_BAND,
  P_RX_PROTOCOL,
  P_RX_TLM_PWR,
  P_RX_TGT_SYS,
  P_RX_SRC_SYS,
  P_COUNT
};

static const uint8_t kIdx[P_COUNT] = {
    MLRS_P_TX_POWER,  MLRS_P_TX_MAV_COMP, MLRS_P_MODE,      MLRS_P_BAND,
    MLRS_P_RX_PROTOCOL, MLRS_P_RX_TLM_PWR, MLRS_P_RX_TGT_SYS, MLRS_P_RX_SRC_SYS,
};

static const char kName[P_COUNT][17] = {
    "TX_POWER",    "TX_MAV_COMP", "MODE",       "RF_BAND",
    "RX_PROTOCOL", "RX_TLM_PWR",  "RX_TGT_SYS", "RX_SRC_SYS",
};

static bool g_enabled = false;
static uint8_t g_seq = 0;
static uint32_t g_hb_ms = 0;
static bool g_list = false;
static uint8_t g_list_i = 0;
static uint32_t g_list_ms = 0;
static uint8_t g_send_idx = 0xFF;
static bool g_send_version = false;
static uint8_t g_parse[PARSE_MAX];
static uint16_t g_parse_n = 0;
static uint16_t g_need = 0;
static bool g_logged_wifi = false;

static uint16_t crc_acc(uint16_t crc, uint8_t b) {
  uint8_t tmp = b ^ (uint8_t)(crc & 0xFF);
  tmp ^= (uint8_t)(tmp << 4);
  return (uint16_t)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

static uint16_t crc_extra(uint16_t crc, uint8_t extra) {
  return crc_acc(crc, extra);
}

static uint8_t pack_v2(uint8_t *dst, uint8_t max, uint32_t msgid,
                       uint8_t extra, const uint8_t *payload, uint8_t plen) {
  const uint16_t n = (uint16_t)(10U + plen + 2U);
  if (dst == nullptr || payload == nullptr || max < n) {
    return 0;
  }
  dst[0] = MAV_MAGIC_V2;
  dst[1] = plen;
  dst[2] = 0;
  dst[3] = 0;
  dst[4] = g_seq++;
  dst[5] = MAV_SYSID;
  dst[6] = MAV_COMPID;
  dst[7] = (uint8_t)msgid;
  dst[8] = (uint8_t)(msgid >> 8);
  dst[9] = (uint8_t)(msgid >> 16);
  memcpy(dst + 10, payload, plen);
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 1; i < (uint16_t)(10U + plen); ++i) {
    crc = crc_acc(crc, dst[i]);
  }
  crc = crc_extra(crc, extra);
  dst[10U + plen] = (uint8_t)(crc & 0xFF);
  dst[11U + plen] = (uint8_t)(crc >> 8);
  return (uint8_t)n;
}

static void inject(const uint8_t *buf, uint8_t n) {
  if (buf == nullptr || n == 0 || !siw917_mavlink_wifi_is_enabled()) {
    return;
  }
  (void)siw917_mavlink_wifi_enqueue_downlink(buf, n);
}

static void send_heartbeat(void) {
  uint8_t p[9] = {};
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  p[3] = 0; /* custom_mode LE */
  p[4] = MAV_TYPE_GENERIC;
  p[5] = MAV_AUTOPILOT_INVALID;
  p[6] = MAV_MODE_FLAG_SAFETY_ARMED;
  p[7] = MAV_STATE_ACTIVE;
  p[8] = 3;
  uint8_t buf[32];
  const uint8_t n = pack_v2(buf, sizeof(buf), MSG_HEARTBEAT, CRC_HEARTBEAT, p, 9);
  inject(buf, n);
}

static void send_param_value(uint8_t i) {
  if (i >= P_COUNT) {
    return;
  }
  uint8_t p[25] = {};
  const float fv = (float)mlrs_mbridge_param_get(kIdx[i]);
  memcpy(p, &fv, 4);
  p[4] = (uint8_t)P_COUNT;
  p[5] = 0;
  p[6] = i;
  p[7] = 0;
  memcpy(p + 8, kName[i], 16);
  p[24] = MAV_PARAM_TYPE_UINT8;
  uint8_t buf[48];
  const uint8_t n =
      pack_v2(buf, sizeof(buf), MSG_PARAM_VALUE, CRC_PARAM_VALUE, p, 25);
  inject(buf, n);
}

static void send_cmd_ack(uint16_t cmd, uint8_t res, uint8_t tsys, uint8_t tcomp) {
  uint8_t p[10] = {};
  p[0] = (uint8_t)cmd;
  p[1] = (uint8_t)(cmd >> 8);
  p[2] = res;
  p[3] = 0;
  p[8] = tsys;
  p[9] = tcomp;
  uint8_t buf[32];
  const uint8_t n =
      pack_v2(buf, sizeof(buf), MSG_COMMAND_ACK, CRC_COMMAND_ACK, p, 10);
  inject(buf, n);
}

static void send_autopilot_version(void) {
  uint8_t p[78] = {};
  const uint64_t cap = (uint64_t)MAV_CAP_MAVLINK2 | (uint64_t)MAV_CAP_COMMAND_INT;
  memcpy(p, &cap, 8);
  const uint32_t ver = 20260813U;
  memcpy(p + 8, &ver, 4);
  uint8_t buf[96];
  const uint8_t n = pack_v2(buf, sizeof(buf), MSG_AUTOPILOT_VERSION,
                            CRC_AUTOPILOT_VERSION, p, 78);
  inject(buf, n);
}

static int find_param(const char *id, int16_t idx) {
  if (idx >= 0 && idx < (int16_t)P_COUNT) {
    return idx;
  }
  if (id == nullptr) {
    return -1;
  }
  char name[17];
  memset(name, 0, sizeof(name));
  memcpy(name, id, 16);
  for (uint8_t i = 0; i < 16; ++i) {
    if (name[i] == 0) {
      break;
    }
  }
  for (uint8_t i = 0; i < P_COUNT; ++i) {
    if (strncmp(name, kName[i], 16) == 0) {
      return (int)i;
    }
  }
  return -1;
}

static bool extract_targets(uint32_t msgid, const uint8_t *p, uint8_t plen,
                            uint8_t *tsys, uint8_t *tcomp) {
  if (p == nullptr || tsys == nullptr || tcomp == nullptr) {
    return false;
  }
  switch (msgid) {
  case MSG_PARAM_REQUEST_LIST:
    if (plen < 2) {
      return false;
    }
    *tsys = p[0];
    *tcomp = p[1];
    return true;
  case MSG_PARAM_REQUEST_READ:
    if (plen < 4) {
      return false;
    }
    *tsys = p[2];
    *tcomp = p[3];
    return true;
  case MSG_PARAM_SET:
    if (plen < 6) {
      return false;
    }
    *tsys = p[4];
    *tcomp = p[5];
    return true;
  case MSG_COMMAND_LONG:
    if (plen < 33) {
      return false;
    }
    *tsys = p[30];
    *tcomp = p[31];
    return true;
  case MSG_COMMAND_INT:
    if (plen < 34) {
      return false;
    }
    *tsys = p[32];
    *tcomp = p[33];
    return true;
  default:
    return false;
  }
}

static void handle_param_read(const uint8_t *p, uint8_t plen) {
  if (p == nullptr || plen < 20) {
    return;
  }
  int16_t idx = 0;
  memcpy(&idx, p, 2);
  char id[16];
  memcpy(id, p + 4, 16);
  const int found = find_param(id, idx);
  if (found >= 0) {
    g_send_idx = (uint8_t)found;
  }
}

static void handle_param_set(const uint8_t *p, uint8_t plen) {
  if (p == nullptr || plen < 23) {
    return;
  }
  float fv = 0;
  memcpy(&fv, p, 4);
  char id[16];
  memcpy(id, p + 6, 16);
  const int found = find_param(id, -1);
  if (found < 0) {
    return;
  }
  uint8_t val = 0;
  if (fv < 0.0f) {
    val = 0;
  } else if (fv > 255.0f) {
    val = 255;
  } else {
    val = (uint8_t)(fv + 0.5f);
  }
  /* Stock refuses to turn the component off from the GCS. */
  if (kIdx[found] == MLRS_P_TX_MAV_COMP) {
    val = 1;
  }
  mlrs_mbridge_param_set(kIdx[found], val);
  g_send_idx = (uint8_t)found;
}

static void handle_command(uint32_t msgid, const uint8_t *p, uint8_t plen,
                           uint8_t src_sys, uint8_t src_comp) {
  if (p == nullptr || plen < 32) {
    return;
  }
  uint16_t cmd = 0;
  float p1 = 0;
  memcpy(&p1, p, 4);
  if (msgid == MSG_COMMAND_LONG) {
    memcpy(&cmd, p + 28, 2);
  } else {
    memcpy(&cmd, p + 28, 2);
  }
  uint8_t res = MAV_RESULT_DENIED;
  const uint32_t want = (uint32_t)p1;
  if (cmd == MAV_CMD_REQUEST_MESSAGE) {
    if (want == MSG_AUTOPILOT_VERSION || want == 300) {
      g_send_version = true;
      res = MAV_RESULT_ACCEPTED;
    }
  } else if (cmd == MAV_CMD_REQUEST_PROTOCOL_VERSION ||
             cmd == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
    g_send_version = true;
    res = MAV_RESULT_ACCEPTED;
  }
  send_cmd_ack(cmd, res, src_sys, src_comp);
}

static void handle_frame(uint32_t msgid, const uint8_t *p, uint8_t plen,
                         uint8_t src_sys, uint8_t src_comp) {
  switch (msgid) {
  case MSG_PARAM_REQUEST_READ:
    handle_param_read(p, plen);
    break;
  case MSG_PARAM_REQUEST_LIST:
    g_list = true;
    g_list_i = 0;
    g_list_ms = millis() - 100U;
    break;
  case MSG_PARAM_SET:
    handle_param_set(p, plen);
    break;
  case MSG_COMMAND_LONG:
  case MSG_COMMAND_INT:
    handle_command(msgid, p, plen, src_sys, src_comp);
    break;
  default:
    break;
  }
}

static void finish_frame(void) {
  if (g_parse_n < 8) {
    g_parse_n = 0;
    g_need = 0;
    return;
  }
  const bool v2 = g_parse[0] == MAV_MAGIC_V2;
  const uint8_t plen = g_parse[1];
  const uint8_t src_sys = v2 ? g_parse[5] : g_parse[3];
  const uint8_t src_comp = v2 ? g_parse[6] : g_parse[4];
  uint32_t msgid = 0;
  const uint8_t *payload = nullptr;
  if (v2) {
    msgid = (uint32_t)g_parse[7] | ((uint32_t)g_parse[8] << 8) |
            ((uint32_t)g_parse[9] << 16);
    payload = g_parse + 10;
  } else {
    msgid = g_parse[5];
    payload = g_parse + 6;
  }
  uint8_t tsys = 0;
  uint8_t tcomp = 0;
  const bool has_tgt = extract_targets(msgid, payload, plen, &tsys, &tcomp);
  bool for_me = false;
  if (has_tgt) {
    for_me = (tsys == 0 || tsys == MAV_SYSID) &&
             (tcomp == 0 || tcomp == MAV_COMPID);
  }
  if (g_enabled && for_me) {
    handle_frame(msgid, payload, plen, src_sys, src_comp);
  }
  if (!g_enabled || !has_tgt || tsys == 0 || !for_me) {
    mlrs_mavlinkx_ingest_mav(g_parse, g_parse_n);
  }
  g_parse_n = 0;
  g_need = 0;
}

static uint16_t frame_need(void) {
  if (g_parse_n < 2) {
    return 2;
  }
  const bool v2 = g_parse[0] == MAV_MAGIC_V2;
  const uint8_t plen = g_parse[1];
  uint16_t n = (uint16_t)((v2 ? 10U : 6U) + plen + 2U);
  if (v2 && g_parse_n >= 3 && (g_parse[2] & 0x01U) != 0) {
    n = (uint16_t)(n + 13U);
  }
  return n;
}

static void parse_byte(uint8_t b) {
  if (g_parse_n == 0) {
    if (b != MAV_MAGIC_V2 && b != MAV_MAGIC_V1) {
      uint8_t one[1] = {b};
      mlrs_mavlinkx_ingest_mav(one, 1);
      return;
    }
  }
  if (g_parse_n >= PARSE_MAX) {
    g_parse_n = 0;
    g_need = 0;
    return;
  }
  g_parse[g_parse_n++] = b;
  if (g_parse_n == 1) {
    g_need = 2;
    return;
  }
  g_need = frame_need();
  if (g_need > PARSE_MAX) {
    g_parse_n = 0;
    g_need = 0;
    return;
  }
  if (g_parse_n >= g_need) {
    finish_frame();
  }
}

void mlrs_tx_mavcomp_set_enabled(bool enabled) {
  if (g_enabled == enabled) {
    return;
  }
  g_enabled = enabled;
  g_list = false;
  g_send_idx = 0xFF;
  g_logged_wifi = false;
  printf("[mLRS] mav component %s (51/68 local)\n", enabled ? "on" : "off");
}

bool mlrs_tx_mavcomp_get_enabled(void) { return g_enabled; }

void mlrs_tx_mavcomp_ingest_uplink(const uint8_t *data, uint16_t len) {
  if (data == nullptr || len == 0) {
    return;
  }
  if (!g_enabled) {
    mlrs_mavlinkx_ingest_mav(data, len);
    return;
  }
  for (uint16_t i = 0; i < len; ++i) {
    parse_byte(data[i]);
  }
}

void mlrs_tx_mavcomp_service(void) {
  if (!g_enabled || !mlrs_ota_is_active()) {
    return;
  }
  if (!siw917_mavlink_wifi_is_enabled()) {
    if (!g_logged_wifi) {
      g_logged_wifi = true;
      printf("[mLRS] mav component on; turn on MAVLink WiFi to see it on GCS\n");
    }
    return;
  }
  const uint32_t now = millis();
  if (g_hb_ms == 0 || (int32_t)(now - g_hb_ms) >= 1000) {
    g_hb_ms = now;
    send_heartbeat();
  }
  if (g_send_version) {
    g_send_version = false;
    send_autopilot_version();
    return;
  }
  if (g_send_idx != 0xFF) {
    const uint8_t i = g_send_idx;
    g_send_idx = 0xFF;
    send_param_value(i);
    return;
  }
  if (g_list && (g_list_ms == 0 || (int32_t)(now - g_list_ms) >= 50)) {
    g_list_ms = now;
    send_param_value(g_list_i);
    ++g_list_i;
    if (g_list_i >= P_COUNT) {
      g_list = false;
    }
  }
}
