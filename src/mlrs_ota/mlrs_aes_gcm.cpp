#include "mlrs_aes_gcm.h"

#include <string.h>

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

static uint32_t sub_word(uint32_t w) {
  return ((uint32_t)kSbox[(w >> 24) & 0xFF] << 24) |
         ((uint32_t)kSbox[(w >> 16) & 0xFF] << 16) |
         ((uint32_t)kSbox[(w >> 8) & 0xFF] << 8) | (uint32_t)kSbox[w & 0xFF];
}

static void aes_expand(const uint8_t key[16], uint32_t rk[44]) {
  for (int i = 0; i < 4; ++i) {
    rk[i] = load_be32(key + 4 * i);
  }
  for (int i = 4; i < 44; ++i) {
    uint32_t t = rk[i - 1];
    if ((i % 4) == 0) {
      t = sub_word((t << 8) | (t >> 24)) ^ ((uint32_t)kRcon[i / 4 - 1] << 24);
    }
    rk[i] = rk[i - 4] ^ t;
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

static void aes_encrypt(const uint32_t rk[44], const uint8_t in[16],
                        uint8_t out[16]) {
  uint8_t s[16];
  memcpy(s, in, 16);
  for (int i = 0; i < 4; ++i) {
    uint32_t w = load_be32(s + 4 * i) ^ rk[i];
    store_be32(s + 4 * i, w);
  }
  for (int round = 1; round < 10; ++round) {
    uint8_t t[16];
    for (int i = 0; i < 16; ++i) {
      t[i] = kSbox[s[i]];
    }
    /* Column-major ShiftRows: 4-byte columns, rows 1/2/3 rotate left. */
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
    mix_columns(s);
    for (int i = 0; i < 4; ++i) {
      uint32_t w = load_be32(s + 4 * i) ^ rk[4 * round + i];
      store_be32(s + 4 * i, w);
    }
  }
  uint8_t t[16];
  for (int i = 0; i < 16; ++i) {
    t[i] = kSbox[s[i]];
  }
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
  for (int i = 0; i < 4; ++i) {
    uint32_t w = load_be32(s + 4 * i) ^ rk[40 + i];
    store_be32(s + 4 * i, w);
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

static void gctr(const uint32_t rk[44], const uint8_t icb[16], uint8_t *buf,
                 size_t len) {
  uint8_t ctr[16];
  uint8_t ks[16];
  memcpy(ctr, icb, 16);
  size_t off = 0;
  while (off < len) {
    aes_encrypt(rk, ctr, ks);
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
  aes_encrypt(ctx->round_key, j0, t);
  xor16(tag, t, s);
}

static void ctx_init_with_key(mlrs_gcm_ctx_t *ctx, const uint8_t key[16],
                              const uint8_t salt[4]) {
  memset(ctx, 0, sizeof(*ctx));
  aes_expand(key, ctx->round_key);
  uint8_t zero[16] = {};
  aes_encrypt(ctx->round_key, zero, ctx->H);
  memcpy(ctx->salt, salt, MLRS_GCM_SALT_LEN);
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

void mlrs_gcm_derive_from_uid(const uint8_t uid[6], mlrs_gcm_ctx_t *uplink,
                              mlrs_gcm_ctx_t *downlink) {
  uint8_t root[16];
  memset(root, 0, sizeof(root));
  memcpy(root, uid, 6);
  root[6] = 'm';
  root[7] = 'L';
  root[8] = 'R';
  root[9] = 'S';
  root[10] = 'G';
  root[11] = 'C';
  root[12] = 'M';
  root[13] = '1';
  root[14] = 0x00;
  root[15] = 0x01;

  uint32_t rk[44];
  aes_expand(root, rk);

  uint8_t label_up[16] = {'m', 'L', 'R', 'S', '/', 'u', 'p', 'l',
                          'i', 'n', 'k', 0,   0,   0,   0,   1};
  uint8_t label_dn[16] = {'m', 'L', 'R', 'S', '/', 'd', 'n', 'l',
                          'i', 'n', 'k', 0,   0,   0,   0,   2};
  uint8_t label_iv[16] = {'m', 'L', 'R', 'S', '/', 'i', 'v', '-',
                          's', 'a', 'l', 't', 0,   0,   0,   3};
  uint8_t k_up[16];
  uint8_t k_dn[16];
  uint8_t salt_block[16];
  aes_encrypt(rk, label_up, k_up);
  aes_encrypt(rk, label_dn, k_dn);
  aes_encrypt(rk, label_iv, salt_block);
  ctx_init_with_key(uplink, k_up, salt_block);
  ctx_init_with_key(downlink, k_dn, salt_block);
  memset(root, 0, sizeof(root));
  memset(k_up, 0, sizeof(k_up));
  memset(k_dn, 0, sizeof(k_dn));
  memset(rk, 0, sizeof(rk));
}

bool mlrs_gcm_selftest(void) {
  /* NIST SP 800-38D Appendix D, Test Case 2. */
  const uint8_t key[16] = {};
  const uint8_t iv[12] = {};
  uint8_t pt[16] = {};
  const uint8_t ct_exp[16] = {0x03, 0x88, 0xda, 0xce, 0x60, 0xb6, 0xa3, 0x92,
                              0xf3, 0x28, 0xc2, 0xb9, 0x71, 0xb2, 0xfe, 0x78};
  const uint8_t tag_exp[16] = {0xab, 0x6e, 0x47, 0xd4, 0x2c, 0xec, 0x13, 0xbd,
                               0xf5, 0x3a, 0x67, 0xb2, 0x12, 0x57, 0xbd, 0xdf};
  mlrs_gcm_ctx_t ctx;
  uint8_t salt[4] = {};
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
  uint8_t zero[16] = {};
  if (memcmp(pt, zero, sizeof(zero)) != 0) {
    return false;
  }

  /* 48-byte payload + 20-byte AAD, cross-checked against Python cryptography. */
  const uint8_t key4[16] = {0xfe, 0xff, 0xe9, 0x92, 0x86, 0x65, 0x73, 0x1c,
                            0x6d, 0x6a, 0x8f, 0x94, 0x67, 0x30, 0x83, 0x08};
  const uint8_t iv4[12] = {0xca, 0xfe, 0xba, 0xbe, 0xfa, 0xce, 0xdb, 0xad,
                           0xde, 0xca, 0xf8, 0x88};
  const uint8_t aad4[20] = {0xfe, 0xed, 0xfa, 0xce, 0xde, 0xad, 0xbe, 0xef,
                            0xfe, 0xed, 0xfa, 0xce, 0xde, 0xad, 0xbe, 0xef,
                            0xab, 0xad, 0xda, 0xd2};
  uint8_t pt4[48] = {
      0xd9, 0x31, 0x32, 0x25, 0xf8, 0x84, 0x06, 0xe5, 0xa5, 0x59, 0x09, 0xc5,
      0xaf, 0xf5, 0x26, 0x9a, 0x86, 0xa7, 0xa9, 0x53, 0x15, 0x34, 0xf7, 0xda,
      0x2e, 0x4c, 0x30, 0x3d, 0x8a, 0x31, 0x8a, 0x72, 0x1e, 0x3c, 0x0c, 0x95,
      0xa9, 0x92, 0xd1, 0x8c, 0x68, 0x11, 0xce, 0x32, 0x64, 0xb0, 0x65, 0x29};
  const uint8_t ct4[48] = {
      0x42, 0x83, 0x1e, 0xc2, 0x21, 0x77, 0x74, 0x24, 0x4b, 0x72, 0x21, 0xb7,
      0x84, 0xd0, 0xd4, 0x9c, 0xe3, 0xaa, 0x21, 0x2f, 0x2c, 0x02, 0xa4, 0xe0,
      0x35, 0xc1, 0x7e, 0x23, 0x29, 0xac, 0xa1, 0x2e, 0x23, 0xd5, 0x14, 0xb2,
      0x68, 0x9c, 0x4b, 0xc3, 0x3a, 0x51, 0xaa, 0x4c, 0x81, 0x92, 0x7a, 0x09};
  const uint8_t tag4[16] = {0x6d, 0x15, 0x02, 0x5f, 0x13, 0x50, 0xfb, 0x85,
                            0xb4, 0xc0, 0xed, 0x6c, 0x05, 0xc1, 0xa0, 0x9e};
  ctx_init_with_key(&ctx, key4, salt);
  mlrs_gcm_seal(&ctx, iv4, aad4, sizeof(aad4), pt4, sizeof(pt4), tag,
                sizeof(tag));
  if (memcmp(pt4, ct4, sizeof(ct4)) != 0 ||
      memcmp(tag, tag4, sizeof(tag4)) != 0) {
    return false;
  }
  return mlrs_gcm_open(&ctx, iv4, aad4, sizeof(aad4), pt4, sizeof(pt4), tag4,
                       sizeof(tag4));
}
