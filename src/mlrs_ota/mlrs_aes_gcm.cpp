#include "mlrs_aes_gcm.h"

#include <string.h>

#if defined(MLRS_HAVE_SI91X_GCM)
extern "C" {
#include "sl_si91x_gcm.h"
#include "sl_status.h"
}
#endif
#if defined(MLRS_HAVE_SI91X_HMAC)
extern "C" {
#include "sl_si91x_hmac.h"
}
#endif
#if defined(MLRS_HAVE_SI91X_TRNG)
extern "C" {
#include "sl_si91x_trng.h"
}
#endif
#if defined(MLRS_HAVE_SI91X_WRAP)
extern "C" {
#include "sl_si91x_wrap.h"
}
#endif

static bool g_hw_ready = false;
static bool g_wrap_ready = false;
static bool g_trng_ready = false;
static bool g_hmac_ready = false;

static const uint8_t kSbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b,
    0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
    0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26,
    0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2,
    0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
    0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed,
    0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f,
    0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
    0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec,
    0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14,
    0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
    0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
    0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f,
    0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
    0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11,
    0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f,
    0xb0, 0x54, 0xbb, 0x16};

static const uint8_t kRcon[10] = {0x01, 0x02, 0x04, 0x08, 0x10,
                                  0x20, 0x40, 0x80, 0x1b, 0x36};

static uint32_t load_be32(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) |
         (uint32_t)p[3];
}

static void store_be32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

static uint32_t rotr32(uint32_t x, uint32_t n) {
  return (x >> n) | (x << (32U - n));
}

static uint32_t sub_word(uint32_t w) {
  return ((uint32_t)kSbox[(w >> 24) & 0xFF] << 24) |
         ((uint32_t)kSbox[(w >> 16) & 0xFF] << 16) |
         ((uint32_t)kSbox[(w >> 8) & 0xFF] << 8) | (uint32_t)kSbox[w & 0xFF];
}

static void aes256_expand(const uint8_t key[32], uint32_t rk[60]) {
  for (int i = 0; i < 8; ++i) {
    rk[i] = load_be32(key + 4 * i);
  }
  for (int i = 8; i < 60; ++i) {
    uint32_t t = rk[i - 1];
    if ((i % 8) == 0) {
      t = sub_word((t << 8) | (t >> 24)) ^ ((uint32_t)kRcon[i / 8 - 1] << 24);
    } else if ((i % 8) == 4) {
      t = sub_word(t);
    }
    rk[i] = rk[i - 8] ^ t;
  }
}

static uint8_t xtime(uint8_t x) {
  return (uint8_t)((x << 1) ^ (((x >> 7) & 1U) * 0x1bU));
}

static void mix_columns(uint8_t *s) {
  for (int i = 0; i < 4; ++i) {
    uint8_t *c = s + 4 * i;
    const uint8_t a0 = c[0];
    const uint8_t a1 = c[1];
    const uint8_t a2 = c[2];
    const uint8_t a3 = c[3];
    const uint8_t t = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);
    c[0] ^= t ^ xtime((uint8_t)(a0 ^ a1));
    c[1] ^= t ^ xtime((uint8_t)(a1 ^ a2));
    c[2] ^= t ^ xtime((uint8_t)(a2 ^ a3));
    c[3] ^= t ^ xtime((uint8_t)(a3 ^ a0));
  }
}

static void shift_rows(uint8_t *s, const uint8_t *t) {
  s[0] = t[0];
  s[4] = t[4];
  s[8] = t[8];
  s[12] = t[12];
  s[1] = t[5];
  s[5] = t[9];
  s[9] = t[13];
  s[13] = t[1];
  s[2] = t[10];
  s[6] = t[14];
  s[10] = t[2];
  s[14] = t[6];
  s[3] = t[15];
  s[7] = t[3];
  s[11] = t[7];
  s[15] = t[11];
}

static void aes256_encrypt(const uint32_t rk[60], const uint8_t in[16],
                           uint8_t out[16]) {
  uint8_t s[16];
  memcpy(s, in, 16);
  for (int i = 0; i < 4; ++i) {
    store_be32(s + 4 * i, load_be32(s + 4 * i) ^ rk[i]);
  }
  for (int round = 1; round < 14; ++round) {
    uint8_t t[16];
    for (int i = 0; i < 16; ++i) {
      t[i] = kSbox[s[i]];
    }
    shift_rows(s, t);
    mix_columns(s);
    for (int i = 0; i < 4; ++i) {
      store_be32(s + 4 * i, load_be32(s + 4 * i) ^ rk[4 * round + i]);
    }
  }
  uint8_t t[16];
  for (int i = 0; i < 16; ++i) {
    t[i] = kSbox[s[i]];
  }
  shift_rows(s, t);
  for (int i = 0; i < 4; ++i) {
    store_be32(s + 4 * i, load_be32(s + 4 * i) ^ rk[56 + i]);
  }
  memcpy(out, s, 16);
}

static void xor16(uint8_t *d, const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 16; ++i) {
    d[i] = (uint8_t)(a[i] ^ b[i]);
  }
}

static void gf_mul(uint8_t z[16], const uint8_t x[16], const uint8_t y[16]) {
  uint8_t v[16];
  uint8_t r[16];
  uint8_t xin[16];
  memset(r, 0, 16);
  memcpy(v, y, 16);
  memcpy(xin, x, 16);
  for (int i = 0; i < 16; ++i) {
    for (int j = 7; j >= 0; --j) {
      if ((xin[i] >> j) & 1U) {
        for (int k = 0; k < 16; ++k) {
          r[k] ^= v[k];
        }
      }
      const uint8_t lsb = (uint8_t)(v[15] & 1U);
      for (int k = 15; k > 0; --k) {
        v[k] = (uint8_t)((v[k] >> 1) | (v[k - 1] << 7));
      }
      v[0] >>= 1;
      if (lsb) {
        v[0] ^= 0xe1;
      }
    }
  }
  memcpy(z, r, 16);
}

static void ghash(const uint8_t H[16], const uint8_t *aad, size_t aad_len,
                  const uint8_t *ct, size_t ct_len, uint8_t out[16]) {
  uint8_t y[16] = {};
  uint8_t block[16];
  size_t off = 0;
  while (off < aad_len) {
    memset(block, 0, 16);
    const size_t n = (aad_len - off) > 16 ? 16 : (aad_len - off);
    memcpy(block, aad + off, n);
    xor16(y, y, block);
    gf_mul(y, y, H);
    off += n;
  }
  off = 0;
  while (off < ct_len) {
    memset(block, 0, 16);
    const size_t n = (ct_len - off) > 16 ? 16 : (ct_len - off);
    memcpy(block, ct + off, n);
    xor16(y, y, block);
    gf_mul(y, y, H);
    off += n;
  }
  memset(block, 0, 16);
  store_be32(block + 4, (uint32_t)(aad_len * 8U));
  store_be32(block + 12, (uint32_t)(ct_len * 8U));
  xor16(y, y, block);
  gf_mul(y, y, H);
  memcpy(out, y, 16);
}

static void inc32(uint8_t ctr[16]) {
  for (int i = 15; i >= 12; --i) {
    if (++ctr[i] != 0) {
      break;
    }
  }
}

static void gctr(const uint32_t rk[60], const uint8_t icb[16], uint8_t *buf,
                 size_t len) {
  uint8_t ctr[16];
  uint8_t ks[16];
  memcpy(ctr, icb, 16);
  size_t off = 0;
  while (off < len) {
    aes256_encrypt(rk, ctr, ks);
    const size_t n = (len - off) > 16 ? 16 : (len - off);
    for (size_t i = 0; i < n; ++i) {
      buf[off + i] ^= ks[i];
    }
    inc32(ctr);
    off += n;
  }
}

static void gcm_crypt(const mlrs_gcm_ctx_t *ctx, const uint8_t iv[12],
                      const uint8_t *aad, size_t aad_len, uint8_t *data,
                      size_t data_len, uint8_t tag[16], bool encrypt) {
  uint8_t j0[16];
  memcpy(j0, iv, 12);
  j0[12] = 0;
  j0[13] = 0;
  j0[14] = 0;
  j0[15] = 1;

  uint8_t ctr[16];
  memcpy(ctr, j0, 16);
  inc32(ctr);

  if (encrypt) {
    gctr(ctx->round_key, ctr, data, data_len);
  }
  uint8_t s[16];
  ghash(ctx->H, aad, aad_len, data, data_len, s);
  if (!encrypt) {
    gctr(ctx->round_key, ctr, data, data_len);
  }

  uint8_t t[16];
  aes256_encrypt(ctx->round_key, j0, t);
  xor16(tag, t, s);
}

static const uint32_t kSha256K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
    0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

static void sha256_transform(uint32_t st[8], const uint8_t block[64]) {
  uint32_t w[64];
  for (int i = 0; i < 16; ++i) {
    w[i] = load_be32(block + 4 * i);
  }
  for (int i = 16; i < 64; ++i) {
    const uint32_t s0 =
        rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const uint32_t s1 =
        rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint32_t a = st[0], b = st[1], c = st[2], d = st[3];
  uint32_t e = st[4], f = st[5], g = st[6], h = st[7];
  for (int i = 0; i < 64; ++i) {
    const uint32_t S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
    const uint32_t ch = (e & f) ^ ((~e) & g);
    const uint32_t t1 = h + S1 + ch + kSha256K[i] + w[i];
    const uint32_t S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
    const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    const uint32_t t2 = S0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  st[0] += a;
  st[1] += b;
  st[2] += c;
  st[3] += d;
  st[4] += e;
  st[5] += f;
  st[6] += g;
  st[7] += h;
}

static void sha256(const uint8_t *msg, size_t len, uint8_t out[32]) {
  uint32_t st[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  uint8_t block[64];
  size_t off = 0;
  while (off + 64 <= len) {
    sha256_transform(st, msg + off);
    off += 64;
  }
  const size_t rem = len - off;
  memset(block, 0, 64);
  if (rem != 0) {
    memcpy(block, msg + off, rem);
  }
  block[rem] = 0x80;
  if (rem >= 56) {
    sha256_transform(st, block);
    memset(block, 0, 64);
  }
  const uint64_t bits = (uint64_t)len * 8U;
  store_be32(block + 56, (uint32_t)(bits >> 32));
  store_be32(block + 60, (uint32_t)bits);
  sha256_transform(st, block);
  for (int i = 0; i < 8; ++i) {
    store_be32(out + 4 * i, st[i]);
  }
}

static void hmac_sha256_sw(const uint8_t *key, size_t key_len, const uint8_t *msg,
                           size_t msg_len, uint8_t out[32]) {
  uint8_t k[64];
  memset(k, 0, sizeof(k));
  if (key_len > 64) {
    sha256(key, key_len, k);
  } else if (key_len != 0) {
    memcpy(k, key, key_len);
  }
  uint8_t ipad[64];
  uint8_t opad[64];
  for (int i = 0; i < 64; ++i) {
    ipad[i] = (uint8_t)(k[i] ^ 0x36);
    opad[i] = (uint8_t)(k[i] ^ 0x5c);
  }
  uint8_t inner[64 + 256];
  memcpy(inner, ipad, 64);
  if (msg_len > 256) {
    msg_len = 256;
  }
  if (msg != nullptr && msg_len != 0) {
    memcpy(inner + 64, msg, msg_len);
  }
  uint8_t ih[32];
  sha256(inner, 64 + msg_len, ih);
  uint8_t outer[96];
  memcpy(outer, opad, 64);
  memcpy(outer + 64, ih, 32);
  sha256(outer, 96, out);
  memset(k, 0, sizeof(k));
}

#if defined(MLRS_HAVE_SI91X_HMAC)
static bool hmac_sha256_hw(const uint8_t *key, size_t key_len, const uint8_t *msg,
                           size_t msg_len, uint8_t out[32]) {
  sl_si91x_hmac_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.hmac_mode = SL_SI91X_HMAC_SHA_256;
  cfg.msg = msg;
  cfg.msg_length = (uint32_t)msg_len;
  cfg.key_config.B0.key_type = SL_SI91X_TRANSPARENT_KEY;
  cfg.key_config.B0.key_size = (uint32_t)key_len;
  cfg.key_config.B0.key_slot = (sl_si91x_crypto_key_slot_t)0;
  cfg.key_config.B0.key = const_cast<uint8_t *>(key);
  return sl_si91x_hmac(&cfg, out) == SL_STATUS_OK;
}
#endif

static void hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg,
                        size_t msg_len, uint8_t out[32]) {
#if defined(MLRS_HAVE_SI91X_HMAC)
  if (g_hmac_ready && hmac_sha256_hw(key, key_len, msg, msg_len, out)) {
    return;
  }
#endif
  hmac_sha256_sw(key, key_len, msg, msg_len, out);
}

static void hkdf_sha256(const uint8_t *ikm, size_t ikm_len, const char *info,
                        uint8_t *okm, size_t okm_len) {
  uint8_t salt[32] = {};
  uint8_t prk[32];
  hmac_sha256(salt, sizeof(salt), ikm, ikm_len, prk);
  uint8_t t[32];
  uint8_t block[48];
  const size_t info_len = strlen(info);
  size_t off = 0;
  uint8_t counter = 1;
  memset(t, 0, sizeof(t));
  while (off < okm_len) {
    size_t n = 0;
    if (counter > 1) {
      memcpy(block, t, 32);
      n = 32;
    }
    memcpy(block + n, info, info_len);
    n += info_len;
    block[n++] = counter++;
    hmac_sha256(prk, 32, block, n, t);
    const size_t take = (okm_len - off) > 32 ? 32 : (okm_len - off);
    memcpy(okm + off, t, take);
    off += take;
  }
  memset(prk, 0, sizeof(prk));
  memset(t, 0, sizeof(t));
}

#if defined(MLRS_HAVE_SI91X_GCM)
static bool siw917_gcm(const mlrs_gcm_ctx_t *ctx, sl_si91x_gcm_type_t op,
                       const uint8_t iv[12], const uint8_t *aad, size_t aad_len,
                       const uint8_t *msg, size_t msg_len, uint8_t *out) {
  static const uint8_t kEmpty[1] = {0};
  sl_si91x_gcm_config_t cfg;
  if (msg_len > SL_SI91X_MAX_DATA_SIZE_IN_BYTES) {
    return false;
  }
  memset(&cfg, 0, sizeof(cfg));
  cfg.encrypt_decrypt = op;
  cfg.gcm_mode = SL_SI91X_GCM_MODE;
  cfg.dma_use = SL_SI91X_GCM_DMA_ENABLE;
  cfg.msg = msg;
  cfg.msg_length = (uint16_t)msg_len;
  cfg.nonce = iv;
  cfg.nonce_length = 12;
  cfg.ad = (aad != nullptr && aad_len != 0) ? aad : kEmpty;
  cfg.ad_length = (uint16_t)aad_len;
  cfg.key_config.b0.key_size = SL_SI91X_GCM_KEY_SIZE_256;
  cfg.key_config.b0.key_slot = (sl_si91x_crypto_key_slot_t)0;
  if (ctx->key_wrapped) {
    cfg.key_config.b0.key_type = SL_SI91X_WRAPPED_KEY;
    cfg.key_config.b0.wrap_iv_mode = SL_SI91X_WRAP_IV_CBC_MODE;
    memcpy(cfg.key_config.b0.wrap_iv, ctx->salt, MLRS_GCM_SALT_LEN);
    memcpy(cfg.key_config.b0.key_buffer, ctx->wrapped_key, MLRS_GCM_KEY_LEN);
  } else {
    cfg.key_config.b0.key_type = SL_SI91X_TRANSPARENT_KEY;
    memcpy(cfg.key_config.b0.key_buffer, ctx->key, MLRS_GCM_KEY_LEN);
  }
  return sl_si91x_gcm(&cfg, out) == SL_STATUS_OK;
}
#endif

#if defined(MLRS_HAVE_SI91X_WRAP)
static bool wrap_key_hw(const uint8_t key[32], const uint8_t salt[4],
                        uint8_t wrapped[32]) {
  static sl_si91x_wrap_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.key_type = SL_SI91X_TRANSPARENT_KEY;
  cfg.key_size = 32;
  cfg.wrap_iv_mode = SL_SI91X_WRAP_IV_CBC_MODE;
  memcpy(cfg.wrap_iv, "mLRSwrap", 8);
  memcpy(cfg.wrap_iv + 8, salt, 4);
  memcpy(cfg.key_buffer, key, 32);
  return sl_si91x_wrap(&cfg, wrapped) == SL_STATUS_OK;
}
#endif

static void ctx_init_with_key(mlrs_gcm_ctx_t *ctx, const uint8_t key[32],
                              const uint8_t salt[4]) {
  memset(ctx, 0, sizeof(*ctx));
  memcpy(ctx->key, key, MLRS_GCM_KEY_LEN);
  memcpy(ctx->salt, salt, MLRS_GCM_SALT_LEN);
  aes256_expand(key, ctx->round_key);
  uint8_t zero[16] = {};
  aes256_encrypt(ctx->round_key, zero, ctx->H);
#if defined(MLRS_HAVE_SI91X_WRAP)
  if (wrap_key_hw(key, salt, ctx->wrapped_key)) {
    ctx->key_wrapped = 1;
    g_wrap_ready = true;
  }
#endif
}

void mlrs_gcm_make_iv(const mlrs_gcm_ctx_t *ctx, uint8_t dir, uint32_t counter,
                      uint8_t iv[MLRS_GCM_IV_LEN]) {
  memcpy(iv, ctx->salt, MLRS_GCM_SALT_LEN);
  iv[4] = dir;
  iv[5] = 0;
  iv[6] = 0;
  iv[7] = 0;
  iv[8] = (uint8_t)counter;
  iv[9] = (uint8_t)(counter >> 8);
  iv[10] = (uint8_t)(counter >> 16);
  iv[11] = (uint8_t)(counter >> 24);
}

void mlrs_gcm_seal(const mlrs_gcm_ctx_t *ctx, const uint8_t iv[MLRS_GCM_IV_LEN],
                   const uint8_t *aad, size_t aad_len, uint8_t *data,
                   size_t data_len, uint8_t *tag, size_t tag_len) {
  uint8_t full[16];
  if (tag_len == 0 || tag_len > 16) {
    tag_len = 16;
  }
#if defined(MLRS_HAVE_SI91X_GCM)
  uint8_t hw_out[96];
  if (g_hw_ready && data_len + 16 <= sizeof(hw_out) &&
      siw917_gcm(ctx, SL_SI91X_GCM_ENCRYPT, iv, aad, aad_len, data, data_len,
                 hw_out)) {
    memcpy(data, hw_out, data_len);
    memcpy(tag, hw_out + data_len, tag_len);
    return;
  }
#endif
  gcm_crypt(ctx, iv, aad, aad_len, data, data_len, full, true);
  memcpy(tag, full, tag_len);
}

bool mlrs_gcm_open(const mlrs_gcm_ctx_t *ctx, const uint8_t iv[MLRS_GCM_IV_LEN],
                   const uint8_t *aad, size_t aad_len, uint8_t *data,
                   size_t data_len, const uint8_t *tag, size_t tag_len) {
  uint8_t got[16];
  if (tag_len == 0 || tag_len > 16) {
    tag_len = 16;
  }
#if defined(MLRS_HAVE_SI91X_GCM)
  uint8_t hw_out[96];
  if (g_hw_ready && data_len + 16 <= sizeof(hw_out) &&
      siw917_gcm(ctx, SL_SI91X_GCM_DECRYPT, iv, aad, aad_len, data, data_len,
                 hw_out)) {
    uint8_t hw_diff = 0;
    for (size_t i = 0; i < tag_len; ++i) {
      hw_diff |= (uint8_t)(hw_out[data_len + i] ^ tag[i]);
    }
    if (hw_diff != 0) {
      memset(data, 0, data_len);
      return false;
    }
    memcpy(data, hw_out, data_len);
    return true;
  }
#endif
  gcm_crypt(ctx, iv, aad, aad_len, data, data_len, got, false);
  uint8_t diff = 0;
  for (size_t i = 0; i < tag_len; ++i) {
    diff |= (uint8_t)(got[i] ^ tag[i]);
  }
  if (diff != 0) {
    memset(data, 0, data_len);
    return false;
  }
  return true;
}

void mlrs_gcm_derive(const uint8_t uid[6],
                     const uint8_t secret[MLRS_GCM_SECRET_LEN],
                     mlrs_gcm_ctx_t *uplink, mlrs_gcm_ctx_t *downlink) {
  uint8_t ikm[6 + MLRS_GCM_SECRET_LEN];
  memcpy(ikm, uid, 6);
  memcpy(ikm + 6, secret, MLRS_GCM_SECRET_LEN);
  uint8_t k_up[32];
  uint8_t k_dn[32];
  uint8_t salt_block[32];
  hkdf_sha256(ikm, sizeof(ikm), "mLRS-R2/uplink", k_up, sizeof(k_up));
  hkdf_sha256(ikm, sizeof(ikm), "mLRS-R2/dnlink", k_dn, sizeof(k_dn));
  hkdf_sha256(ikm, sizeof(ikm), "mLRS-R2/ivsalt", salt_block, sizeof(salt_block));
  ctx_init_with_key(uplink, k_up, salt_block);
  ctx_init_with_key(downlink, k_dn, salt_block);
  memset(ikm, 0, sizeof(ikm));
  memset(k_up, 0, sizeof(k_up));
  memset(k_dn, 0, sizeof(k_dn));
}

bool mlrs_gcm_random(uint8_t *out, size_t len) {
  if (out == nullptr || len == 0) {
    return false;
  }
#if defined(MLRS_HAVE_SI91X_TRNG)
  if (len <= 1024) {
    uint32_t words[256];
    if (sl_si91x_trng_get_random_num(words, (uint16_t)len) == SL_STATUS_OK) {
      memcpy(out, words, len);
      memset(words, 0, sizeof(words));
      uint8_t acc = 0;
      for (size_t i = 0; i < len; ++i) {
        acc |= out[i];
      }
      if (acc != 0) {
        g_trng_ready = true;
        return true;
      }
    }
  }
#endif
  uint8_t seed[48];
  memset(seed, 0, sizeof(seed));
  static uint32_t mix = 0xA5A5A5A5U;
  mix = mix * 1664525U + 1013904223U + (uint32_t)(uintptr_t)out;
  store_be32(seed, mix);
  store_be32(seed + 4, (uint32_t)len);
  store_be32(seed + 8, (uint32_t)(uintptr_t)&mix);
  sha256(seed, sizeof(seed), seed);
  size_t off = 0;
  uint32_t counter = 1;
  while (off < len) {
    uint8_t block[36];
    memcpy(block, seed, 32);
    store_be32(block + 32, counter++);
    uint8_t digest[32];
    sha256(block, sizeof(block), digest);
    const size_t n = (len - off) > 32 ? 32 : (len - off);
    memcpy(out + off, digest, n);
    off += n;
  }
  return true;
}

bool mlrs_gcm_selftest(void) {
  /* FIPS-197 C.3 AES-256 ECB. */
  const uint8_t aes_key[32] = {
      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
      0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
      0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};
  const uint8_t aes_pt[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                              0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
  const uint8_t aes_ct[16] = {0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf,
                              0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89};
  uint32_t rk[60];
  uint8_t got[16];
  aes256_expand(aes_key, rk);
  aes256_encrypt(rk, aes_pt, got);
  if (memcmp(got, aes_ct, 16) != 0) {
    return false;
  }

  /* RFC 4231 HMAC-SHA256 test case 1. */
  const uint8_t hmac_key[20] = {0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
                                0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
                                0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b};
  const uint8_t hmac_msg[8] = {'H', 'i', ' ', 'T', 'h', 'e', 'r', 'e'};
  const uint8_t hmac_exp[32] = {
      0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53, 0x5c, 0xa8, 0xaf,
      0xce, 0xaf, 0x0b, 0xf1, 0x2b, 0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83,
      0x3d, 0xa7, 0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7};
  uint8_t hmac_got[32];
  hmac_sha256_sw(hmac_key, sizeof(hmac_key), hmac_msg, sizeof(hmac_msg),
                 hmac_got);
  if (memcmp(hmac_got, hmac_exp, 32) != 0) {
    return false;
  }
#if defined(MLRS_HAVE_SI91X_HMAC)
  if (hmac_sha256_hw(hmac_key, sizeof(hmac_key), hmac_msg, sizeof(hmac_msg),
                     hmac_got) &&
      memcmp(hmac_got, hmac_exp, 32) == 0) {
    g_hmac_ready = true;
  }
#endif

  /* NIST SP 800-38D AES-256 GCM, Test Case 14. */
  const uint8_t key[32] = {};
  const uint8_t iv[12] = {};
  uint8_t pt[16] = {};
  const uint8_t ct_exp[16] = {0xce, 0xa7, 0x40, 0x3d, 0x4d, 0x60, 0x6b, 0x6e,
                              0x07, 0x4e, 0xc5, 0xd3, 0xba, 0xf3, 0x9d, 0x18};
  const uint8_t tag_exp[16] = {0xd0, 0xd1, 0xc8, 0xa7, 0x99, 0x99, 0x6b, 0xf0,
                               0x26, 0x5b, 0x98, 0xb5, 0xd4, 0x8a, 0xb9, 0x19};
  mlrs_gcm_ctx_t ctx;
  uint8_t salt[4] = {};
  g_hw_ready = false;
  ctx_init_with_key(&ctx, key, salt);
  uint8_t tag[16];
  mlrs_gcm_seal(&ctx, iv, nullptr, 0, pt, sizeof(pt), tag, sizeof(tag));
  if (memcmp(pt, ct_exp, sizeof(ct_exp)) != 0 ||
      memcmp(tag, tag_exp, sizeof(tag_exp)) != 0) {
    return false;
  }
  if (!mlrs_gcm_open(&ctx, iv, nullptr, 0, pt, sizeof(pt), tag_exp,
                     sizeof(tag_exp))) {
    return false;
  }

#if defined(MLRS_HAVE_SI91X_GCM)
  {
    uint8_t hw_pt[16] = {};
    uint8_t hw_out[32];
    mlrs_gcm_ctx_t hw_ctx;
    ctx_init_with_key(&hw_ctx, key, salt);
    hw_ctx.key_wrapped = 0;
    if (siw917_gcm(&hw_ctx, SL_SI91X_GCM_ENCRYPT, iv, nullptr, 0, hw_pt,
                   sizeof(hw_pt), hw_out) &&
        memcmp(hw_out, ct_exp, sizeof(ct_exp)) == 0 &&
        memcmp(hw_out + 16, tag_exp, sizeof(tag_exp)) == 0) {
      g_hw_ready = true;
    }
  }
#endif
  return true;
}

const char *mlrs_gcm_backend_name(void) {
#if defined(MLRS_HAVE_SI91X_GCM)
  if (g_hw_ready && g_wrap_ready) {
    return "siw917-hw-aes256-wrap";
  }
  if (g_hw_ready) {
    return g_hmac_ready ? "siw917-hw-aes256" : "siw917-hw-aes256 (sw-hkdf)";
  }
  return "software-aes256 (hw probe failed)";
#else
  return "software-aes256";
#endif
}
