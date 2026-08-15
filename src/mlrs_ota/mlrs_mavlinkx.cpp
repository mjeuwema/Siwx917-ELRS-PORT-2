//------------------------------
// Overlay MavlinkX: compact header + stock mLRS X4 payload compression.
// Compression scheme X4 Copyright (c) OlliW, OlliW42, www.olliw.eu
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies, and
// that the use of this software in a product is acknowledged in the product
// documentation or a location viewable to users.
//------------------------------

#include "mlrs_mavlinkx.h"

#include <string.h>

#define MAVLINKX_MAGIC_1 0x6F
#define MAVLINKX_MAGIC_2 0x77
#define MAVLINKX_CRC8_INIT 0xFF
#define MAV_MAGIC_V1 0xFE
#define MAV_MAGIC_V2 0xFD
#define MAV_V2_HEADER_LEN 10
#define MAV_V1_HEADER_LEN 6
#define MAV_CK_LEN 2
#define MAV_SIG_LEN 13
#define MAV_INCOMPAT_SIGNED 0x01
#define X_FIFO_LEN 512
#define X_FRAME_MAX 287
#define X_PAYLOAD_MAX 255

enum {
  X_FLAG_IS_V1 = 0x01,
  X_FLAG_HAS_MSGID16 = 0x02,
  X_FLAG_HAS_EXTENSION = 0x80,
  X_FLAG_IS_COMPRESSED = 0x40,
  X_FLAG_EXT_HAS_SIGNATURE = 0x01,
  X_FLAG_EXT_HAS_MSGID24 = 0x02,
};

struct Fifo {
  uint8_t buf[X_FIFO_LEN];
  uint16_t r;
  uint16_t w;
};

static Fifo g_air_fifo;
static Fifo g_mav_fifo;
static bool g_compress;

static uint16_t fifo_count(const Fifo *f) {
  return (uint16_t)((f->w - f->r) & (X_FIFO_LEN - 1));
}

static uint16_t fifo_free(const Fifo *f) {
  return (uint16_t)(X_FIFO_LEN - 1U - fifo_count(f));
}

static void fifo_reset(Fifo *f) { f->r = f->w = 0; }

static bool fifo_put_buf(Fifo *f, const uint8_t *p, uint16_t n) {
  if (fifo_free(f) < n) {
    return false;
  }
  for (uint16_t i = 0; i < n; ++i) {
    f->buf[f->w] = p[i];
    f->w = (uint16_t)((f->w + 1U) & (X_FIFO_LEN - 1));
  }
  return true;
}

static uint8_t fifo_get(Fifo *f) {
  uint8_t c = f->buf[f->r];
  f->r = (uint16_t)((f->r + 1U) & (X_FIFO_LEN - 1));
  return c;
}

static uint8_t fifo_take(Fifo *f, uint8_t *dst, uint8_t max) {
  uint16_t n = fifo_count(f);
  if (n > max) {
    n = max;
  }
  for (uint16_t i = 0; i < n; ++i) {
    dst[i] = fifo_get(f);
  }
  return (uint8_t)n;
}

static const uint8_t kCrc8[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3};

static uint8_t crc8_calc(const uint8_t *buf, uint16_t len) {
  uint8_t crc = MAVLINKX_CRC8_INIT;
  while (len--) {
    crc = kCrc8[crc ^ *buf++];
  }
  return crc;
}

struct BitSt {
  uint32_t bit_buf;
  uint8_t bit_cnt;
  uint16_t in_pos;
};

static void encode_put_bits(uint8_t *out, uint16_t *len_out, BitSt *st,
                            uint16_t code, uint8_t len) {
  st->bit_buf = (st->bit_buf << len) | code;
  st->bit_cnt = (uint8_t)(st->bit_cnt + len);
  while (st->bit_cnt >= 8) {
    st->bit_cnt = (uint8_t)(st->bit_cnt - 8);
    out[(*len_out)++] = (uint8_t)(st->bit_buf >> st->bit_cnt);
  }
}

static void encode_flush_bits(uint8_t *out, uint16_t *len_out, BitSt *st) {
  if (st->bit_cnt > 0) {
    uint32_t pad = (1U << (8U - st->bit_cnt)) - 1U;
    out[(*len_out)++] =
        (uint8_t)((st->bit_buf << (8U - st->bit_cnt)) | pad);
  }
}

static void encode_rle(uint8_t *out, uint16_t *len_out, BitSt *st, uint8_t c,
                       uint8_t rle) {
  uint16_t code;
  if (c == 0x00) {
    if (rle <= 4) {
      for (uint8_t i = 0; i < rle; ++i) {
        encode_put_bits(out, len_out, st, 0b000, 3);
      }
    } else {
      code = (uint16_t)((0b00110 << 8) + rle);
      encode_put_bits(out, len_out, st, code, 13);
    }
  } else if (c == 0xFF) {
    if (rle <= 3) {
      for (uint8_t i = 0; i < rle; ++i) {
        encode_put_bits(out, len_out, st, 0b0010, 4);
      }
    } else {
      code = (uint16_t)((0b00111 << 8) + rle);
      encode_put_bits(out, len_out, st, code, 13);
    }
  }
}

static uint8_t payload_compress(uint8_t *out, uint16_t *len_out,
                                const uint8_t *payload, uint16_t len) {
  BitSt st = {};
  uint8_t in_rle = 0;
  uint8_t rle_c = 0;
  uint8_t rle_n = 0;
  *len_out = 0;
  out[0] = 0xFF;
  for (uint16_t n = 0; n < len; ++n) {
    uint8_t c = payload[n];
    if (in_rle) {
      if (c == rle_c && rle_n < 255) {
        ++rle_n;
      } else {
        in_rle = 0;
        encode_rle(out, len_out, &st, rle_c, rle_n);
      }
    }
    if (!in_rle) {
      if (c == 0x00 || c == 0xFF) {
        in_rle = 1;
        rle_c = c;
        rle_n = 1;
      } else if (c >= 1 && c <= 64) {
        encode_put_bits(out, len_out, &st, (uint16_t)((0b10 << 6) + (c - 1)),
                        8);
      } else if (c >= 191 && c <= 254) {
        encode_put_bits(out, len_out, &st,
                        (uint16_t)((0b11 << 6) + (c - 191)), 8);
      } else {
        encode_put_bits(out, len_out, &st, (uint16_t)((0b01 << 7) + (c - 65)),
                        9);
      }
    }
    if (*len_out >= len) {
      return 0;
    }
  }
  if (in_rle) {
    encode_rle(out, len_out, &st, rle_c, rle_n);
  }
  encode_flush_bits(out, len_out, &st);
  if (*len_out >= len) {
    return 0;
  }
  return 1;
}

static uint8_t decode_get_bits(uint8_t *code, const uint8_t *in, uint16_t len,
                               BitSt *st, uint8_t bits) {
  while (st->bit_cnt < bits) {
    if (st->in_pos >= len) {
      return 0;
    }
    st->bit_buf = (st->bit_buf << 8) | in[st->in_pos++];
    st->bit_cnt = (uint8_t)(st->bit_cnt + 8);
  }
  uint32_t mask = ((uint32_t)1 << bits) - 1U;
  *code = (uint8_t)((st->bit_buf >> (st->bit_cnt - bits)) & mask);
  st->bit_cnt = (uint8_t)(st->bit_cnt - bits);
  return 1;
}

static void payload_decompress(uint8_t *payload, uint16_t *len_out,
                               uint16_t in_len) {
  uint8_t in[X_PAYLOAD_MAX + 4];
  BitSt st = {};
  if (in_len > X_PAYLOAD_MAX) {
    in_len = X_PAYLOAD_MAX;
  }
  memcpy(in, payload, in_len);
  *len_out = 0;
  while (1) {
    uint8_t c = 0;
    uint8_t kind = 0xFF;
    if (!decode_get_bits(&c, in, in_len, &st, 2)) {
      return;
    }
    if (c == 0b10) {
      kind = 0;
    } else if (c == 0b11) {
      kind = 1;
    } else if (c == 0b01) {
      kind = 2;
    } else if (decode_get_bits(&c, in, in_len, &st, 1)) {
      if (c == 0) {
        kind = 3;
      } else if (decode_get_bits(&c, in, in_len, &st, 1)) {
        if (c == 0) {
          kind = 4;
        } else if (decode_get_bits(&c, in, in_len, &st, 1)) {
          kind = (c == 0) ? 5 : 6;
        }
      }
    }
    if (kind == 0xFF) {
      return;
    }
    if (*len_out >= X_PAYLOAD_MAX) {
      return;
    }
    switch (kind) {
    case 0:
      if (!decode_get_bits(&c, in, in_len, &st, 6)) {
        return;
      }
      payload[(*len_out)++] = (uint8_t)(c + 1);
      break;
    case 1:
      if (!decode_get_bits(&c, in, in_len, &st, 6)) {
        return;
      }
      payload[(*len_out)++] = (uint8_t)(c + 191);
      break;
    case 2:
      if (!decode_get_bits(&c, in, in_len, &st, 7) || c > 125) {
        return;
      }
      payload[(*len_out)++] = (uint8_t)(c + 65);
      break;
    case 3:
      payload[(*len_out)++] = 0x00;
      break;
    case 4:
      payload[(*len_out)++] = 0xFF;
      break;
    case 5:
      if (!decode_get_bits(&c, in, in_len, &st, 8)) {
        return;
      }
      if ((uint16_t)*len_out + c > X_PAYLOAD_MAX) {
        return;
      }
      memset(payload + *len_out, 0, c);
      *len_out = (uint16_t)(*len_out + c);
      break;
    case 6:
      if (!decode_get_bits(&c, in, in_len, &st, 8)) {
        return;
      }
      if ((uint16_t)*len_out + c > X_PAYLOAD_MAX) {
        return;
      }
      memset(payload + *len_out, 0xFF, c);
      *len_out = (uint16_t)(*len_out + c);
      break;
    }
  }
}

struct MavMsg {
  uint8_t magic;
  uint8_t len;
  uint8_t incompat;
  uint8_t seq;
  uint8_t sysid;
  uint8_t compid;
  uint32_t msgid;
  uint16_t checksum;
  uint8_t payload[X_PAYLOAD_MAX];
  uint8_t signature[MAV_SIG_LEN];
  uint8_t has_sig;
};

static uint16_t encode_x(uint8_t *buf, const MavMsg *msg) {
  uint8_t flags_ext = 0;
  buf[0] = MAVLINKX_MAGIC_1;
  buf[1] = MAVLINKX_MAGIC_2;
  buf[2] = 0;
  if (msg->magic == MAV_MAGIC_V1) {
    buf[2] |= X_FLAG_IS_V1;
  }
  if (msg->has_sig) {
    buf[2] |= X_FLAG_HAS_EXTENSION;
    flags_ext |= X_FLAG_EXT_HAS_SIGNATURE;
  }
  if (msg->msgid >= 65536U) {
    buf[2] |= X_FLAG_HAS_EXTENSION;
    flags_ext |= X_FLAG_EXT_HAS_MSGID24;
  } else if (msg->msgid >= 256U) {
    buf[2] |= X_FLAG_HAS_MSGID16;
  }
  uint16_t pos = 3;
  if (buf[2] & X_FLAG_HAS_EXTENSION) {
    buf[pos++] = flags_ext;
  }
  const uint8_t pos_of_len = (uint8_t)pos;
  buf[pos++] = msg->len;
  buf[pos++] = msg->seq;
  buf[pos++] = msg->sysid;
  buf[pos++] = msg->compid;
  buf[pos++] = (uint8_t)msg->msgid;
  if (buf[2] & X_FLAG_HAS_MSGID16) {
    buf[pos++] = (uint8_t)(msg->msgid >> 8);
  }
  if (flags_ext & X_FLAG_EXT_HAS_MSGID24) {
    buf[pos++] = (uint8_t)(msg->msgid >> 16);
  }
  uint16_t plen = msg->len;
  if (g_compress &&
      payload_compress(&buf[pos + 1], &plen, msg->payload, msg->len)) {
    buf[2] |= X_FLAG_IS_COMPRESSED;
    buf[pos_of_len] = (uint8_t)plen;
  } else {
    plen = msg->len;
    memcpy(&buf[pos + 1], msg->payload, msg->len);
  }
  buf[pos] = crc8_calc(buf, pos);
  pos++;
  pos = (uint16_t)(pos + plen);
  buf[pos++] = (uint8_t)msg->checksum;
  buf[pos++] = (uint8_t)(msg->checksum >> 8);
  if (msg->has_sig) {
    memcpy(&buf[pos], msg->signature, MAV_SIG_LEN);
    pos = (uint16_t)(pos + MAV_SIG_LEN);
  }
  return pos;
}

static uint16_t mav_frame_from_msg(uint8_t *out, const MavMsg *msg) {
  uint16_t pos = 0;
  if (msg->magic == MAV_MAGIC_V1) {
    out[pos++] = MAV_MAGIC_V1;
    out[pos++] = msg->len;
    out[pos++] = msg->seq;
    out[pos++] = msg->sysid;
    out[pos++] = msg->compid;
    out[pos++] = (uint8_t)msg->msgid;
  } else {
    out[pos++] = MAV_MAGIC_V2;
    out[pos++] = msg->len;
    out[pos++] = msg->incompat;
    out[pos++] = 0;
    out[pos++] = msg->seq;
    out[pos++] = msg->sysid;
    out[pos++] = msg->compid;
    out[pos++] = (uint8_t)msg->msgid;
    out[pos++] = (uint8_t)(msg->msgid >> 8);
    out[pos++] = (uint8_t)(msg->msgid >> 16);
  }
  memcpy(out + pos, msg->payload, msg->len);
  pos = (uint16_t)(pos + msg->len);
  out[pos++] = (uint8_t)msg->checksum;
  out[pos++] = (uint8_t)(msg->checksum >> 8);
  if (msg->has_sig) {
    memcpy(out + pos, msg->signature, MAV_SIG_LEN);
    pos = (uint16_t)(pos + MAV_SIG_LEN);
  }
  return pos;
}

enum { MAV_IDLE, MAV_LEN, MAV_BODY };

static struct {
  uint8_t st;
  uint8_t buf[X_FRAME_MAX];
  uint16_t n;
  uint16_t need;
} g_mav_parse;

static void mav_parse_reset() {
  g_mav_parse.st = MAV_IDLE;
  g_mav_parse.n = 0;
  g_mav_parse.need = 0;
}

static void mav_parse_byte(uint8_t c) {
  if (g_mav_parse.st == MAV_IDLE) {
    if (c != MAV_MAGIC_V1 && c != MAV_MAGIC_V2) {
      return;
    }
    g_mav_parse.buf[0] = c;
    g_mav_parse.n = 1;
    g_mav_parse.st = MAV_LEN;
    return;
  }
  if (g_mav_parse.st == MAV_LEN) {
    g_mav_parse.buf[1] = c;
    g_mav_parse.n = 2;
    if (g_mav_parse.buf[0] == MAV_MAGIC_V1) {
      g_mav_parse.need =
          (uint16_t)(MAV_V1_HEADER_LEN + c + MAV_CK_LEN);
    } else {
      g_mav_parse.need =
          (uint16_t)(MAV_V2_HEADER_LEN + c + MAV_CK_LEN);
    }
    if (g_mav_parse.need > X_FRAME_MAX) {
      mav_parse_reset();
      return;
    }
    g_mav_parse.st = MAV_BODY;
    return;
  }
  if (g_mav_parse.n >= X_FRAME_MAX) {
    mav_parse_reset();
    return;
  }
  g_mav_parse.buf[g_mav_parse.n++] = c;
  if (g_mav_parse.buf[0] == MAV_MAGIC_V2 && g_mav_parse.n == 3 &&
      (g_mav_parse.buf[2] & MAV_INCOMPAT_SIGNED)) {
    g_mav_parse.need = (uint16_t)(g_mav_parse.need + MAV_SIG_LEN);
    if (g_mav_parse.need > X_FRAME_MAX) {
      mav_parse_reset();
      return;
    }
  }
  if (g_mav_parse.n < g_mav_parse.need) {
    return;
  }
  MavMsg msg = {};
  const uint8_t *b = g_mav_parse.buf;
  msg.magic = b[0];
  msg.len = b[1];
  uint16_t pay_off;
  if (msg.magic == MAV_MAGIC_V1) {
    msg.seq = b[2];
    msg.sysid = b[3];
    msg.compid = b[4];
    msg.msgid = b[5];
    pay_off = MAV_V1_HEADER_LEN;
  } else {
    msg.incompat = b[2];
    msg.seq = b[4];
    msg.sysid = b[5];
    msg.compid = b[6];
    msg.msgid = (uint32_t)b[7] | ((uint32_t)b[8] << 8) | ((uint32_t)b[9] << 16);
    pay_off = MAV_V2_HEADER_LEN;
  }
  memcpy(msg.payload, b + pay_off, msg.len);
  uint16_t ck_off = (uint16_t)(pay_off + msg.len);
  msg.checksum = (uint16_t)b[ck_off] | ((uint16_t)b[ck_off + 1] << 8);
  if (msg.magic == MAV_MAGIC_V2 && (msg.incompat & MAV_INCOMPAT_SIGNED)) {
    memcpy(msg.signature, b + ck_off + 2, MAV_SIG_LEN);
    msg.has_sig = 1;
  }
  uint8_t xbuf[X_FRAME_MAX];
  uint16_t xn = encode_x(xbuf, &msg);
  if (xn > 0 && xn <= X_FRAME_MAX) {
    (void)fifo_put_buf(&g_air_fifo, xbuf, xn);
  }
  mav_parse_reset();
}

enum {
  X_IDLE,
  X_MAGIC2,
  X_FLAGS,
  X_EXT,
  X_LEN,
  X_SEQ,
  X_SYS,
  X_COMP,
  X_MID1,
  X_MID2,
  X_MID3,
  X_CRC8,
  X_PAY,
  X_CK0,
  X_CK1,
  X_SIG
};

static struct {
  uint8_t st;
  uint8_t flags;
  uint8_t flags_ext;
  uint8_t hdr[20];
  uint8_t hdr_n;
  uint8_t plen;
  uint8_t pay[X_PAYLOAD_MAX];
  uint16_t pay_n;
  uint8_t seq;
  uint8_t sysid;
  uint8_t compid;
  uint32_t msgid;
  uint16_t checksum;
  uint8_t sig[MAV_SIG_LEN];
  uint8_t sig_n;
} g_x;

static void x_reset() {
  memset(&g_x, 0, sizeof(g_x));
  g_x.st = X_IDLE;
}

static void x_emit() {
  MavMsg msg = {};
  uint16_t plen = g_x.plen;
  if (g_x.flags & X_FLAG_IS_COMPRESSED) {
    payload_decompress(g_x.pay, &plen, g_x.plen);
  }
  msg.magic = (g_x.flags & X_FLAG_IS_V1) ? MAV_MAGIC_V1 : MAV_MAGIC_V2;
  msg.len = (uint8_t)plen;
  msg.seq = g_x.seq;
  msg.sysid = g_x.sysid;
  msg.compid = g_x.compid;
  msg.msgid = g_x.msgid;
  msg.checksum = g_x.checksum;
  memcpy(msg.payload, g_x.pay, msg.len);
  if (g_x.flags_ext & X_FLAG_EXT_HAS_SIGNATURE) {
    memcpy(msg.signature, g_x.sig, MAV_SIG_LEN);
    msg.has_sig = 1;
    msg.incompat = MAV_INCOMPAT_SIGNED;
  }
  uint8_t out[X_FRAME_MAX];
  uint16_t n = mav_frame_from_msg(out, &msg);
  (void)fifo_put_buf(&g_mav_fifo, out, n);
}

static void x_parse_byte(uint8_t c) {
  switch (g_x.st) {
  case X_IDLE:
    if (c == MAVLINKX_MAGIC_1) {
      g_x.hdr[0] = c;
      g_x.hdr_n = 1;
      g_x.st = X_MAGIC2;
    }
    break;
  case X_MAGIC2:
    if (c == MAVLINKX_MAGIC_2) {
      g_x.hdr[1] = c;
      g_x.hdr_n = 2;
      g_x.st = X_FLAGS;
    } else {
      x_reset();
    }
    break;
  case X_FLAGS:
    g_x.flags = c;
    g_x.hdr[g_x.hdr_n++] = c;
    g_x.st = (c & X_FLAG_HAS_EXTENSION) ? X_EXT : X_LEN;
    break;
  case X_EXT:
    g_x.flags_ext = c;
    g_x.hdr[g_x.hdr_n++] = c;
    g_x.st = X_LEN;
    break;
  case X_LEN:
    g_x.plen = c;
    g_x.hdr[g_x.hdr_n++] = c;
    g_x.st = X_SEQ;
    break;
  case X_SEQ:
    g_x.seq = c;
    g_x.hdr[g_x.hdr_n++] = c;
    g_x.st = X_SYS;
    break;
  case X_SYS:
    g_x.sysid = c;
    g_x.hdr[g_x.hdr_n++] = c;
    g_x.st = X_COMP;
    break;
  case X_COMP:
    g_x.compid = c;
    g_x.hdr[g_x.hdr_n++] = c;
    g_x.st = X_MID1;
    break;
  case X_MID1:
    g_x.msgid = c;
    g_x.hdr[g_x.hdr_n++] = c;
    if (g_x.flags & X_FLAG_HAS_MSGID16) {
      g_x.st = X_MID2;
    } else if (g_x.flags_ext & X_FLAG_EXT_HAS_MSGID24) {
      g_x.st = X_MID2;
    } else {
      g_x.st = X_CRC8;
    }
    break;
  case X_MID2:
    g_x.msgid |= (uint32_t)c << 8;
    g_x.hdr[g_x.hdr_n++] = c;
    g_x.st = (g_x.flags_ext & X_FLAG_EXT_HAS_MSGID24) ? X_MID3 : X_CRC8;
    break;
  case X_MID3:
    g_x.msgid |= (uint32_t)c << 16;
    g_x.hdr[g_x.hdr_n++] = c;
    g_x.st = X_CRC8;
    break;
  case X_CRC8:
    if (c != crc8_calc(g_x.hdr, g_x.hdr_n)) {
      x_reset();
      break;
    }
    g_x.pay_n = 0;
    g_x.st = (g_x.plen == 0) ? X_CK0 : X_PAY;
    break;
  case X_PAY:
    if (g_x.pay_n < X_PAYLOAD_MAX) {
      g_x.pay[g_x.pay_n++] = c;
    }
    if (g_x.pay_n >= g_x.plen) {
      g_x.st = X_CK0;
    }
    break;
  case X_CK0:
    g_x.checksum = c;
    g_x.st = X_CK1;
    break;
  case X_CK1:
    g_x.checksum |= (uint16_t)c << 8;
    if (g_x.flags_ext & X_FLAG_EXT_HAS_SIGNATURE) {
      g_x.sig_n = 0;
      g_x.st = X_SIG;
    } else {
      x_emit();
      x_reset();
    }
    break;
  case X_SIG:
    if (g_x.sig_n < MAV_SIG_LEN) {
      g_x.sig[g_x.sig_n++] = c;
    }
    if (g_x.sig_n >= MAV_SIG_LEN) {
      x_emit();
      x_reset();
    }
    break;
  default:
    x_reset();
    break;
  }
}

void mlrs_mavlinkx_init(void) {
  fifo_reset(&g_air_fifo);
  fifo_reset(&g_mav_fifo);
  mav_parse_reset();
  x_reset();
  g_compress = false;
}

void mlrs_mavlinkx_reset(void) { mlrs_mavlinkx_init(); }

void mlrs_mavlinkx_set_compression(bool enabled) { g_compress = enabled; }

void mlrs_mavlinkx_ingest_mav(const uint8_t *data, uint16_t len) {
  if (data == nullptr || len == 0) {
    return;
  }
  for (uint16_t i = 0; i < len; ++i) {
    if (g_mav_parse.st == MAV_IDLE && fifo_free(&g_air_fifo) < 290) {
      break;
    }
    mav_parse_byte(data[i]);
  }
}

uint8_t mlrs_mavlinkx_take_air(uint8_t *dst, uint8_t max) {
  if (dst == nullptr || max == 0) {
    return 0;
  }
  return fifo_take(&g_air_fifo, dst, max);
}

void mlrs_mavlinkx_ingest_air(const uint8_t *data, uint16_t len) {
  if (data == nullptr || len == 0) {
    return;
  }
  for (uint16_t i = 0; i < len; ++i) {
    x_parse_byte(data[i]);
  }
}

uint8_t mlrs_mavlinkx_take_mav(uint8_t *dst, uint8_t max) {
  if (dst == nullptr || max == 0) {
    return 0;
  }
  return fifo_take(&g_mav_fifo, dst, max);
}
