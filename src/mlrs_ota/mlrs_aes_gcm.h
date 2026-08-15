#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MLRS_GCM_KEY_LEN 32
#define MLRS_GCM_SECRET_LEN 32
#define MLRS_GCM_IV_LEN 12
#define MLRS_GCM_TAG_LEN 16
#define MLRS_GCM_SALT_LEN 4
#define MLRS_GCM_DIR_UPLINK 0x01
#define MLRS_GCM_DIR_DOWNLINK 0x02

typedef struct {
  uint32_t round_key[60];
  uint8_t H[16];
  uint8_t salt[MLRS_GCM_SALT_LEN];
  uint8_t key[MLRS_GCM_KEY_LEN];
  uint8_t wrapped_key[MLRS_GCM_KEY_LEN];
  uint8_t key_wrapped;
} mlrs_gcm_ctx_t;

bool mlrs_gcm_selftest(void);
const char *mlrs_gcm_backend_name(void);
bool mlrs_gcm_random(uint8_t *out, size_t len);
void mlrs_gcm_derive(const uint8_t uid[6],
                     const uint8_t secret[MLRS_GCM_SECRET_LEN],
                     mlrs_gcm_ctx_t *uplink, mlrs_gcm_ctx_t *downlink);
void mlrs_gcm_make_iv(const mlrs_gcm_ctx_t *ctx, uint8_t dir, uint32_t counter,
                      uint8_t iv[MLRS_GCM_IV_LEN]);
void mlrs_gcm_seal(const mlrs_gcm_ctx_t *ctx, const uint8_t iv[MLRS_GCM_IV_LEN],
                   const uint8_t *aad, size_t aad_len, uint8_t *data,
                   size_t data_len, uint8_t *tag, size_t tag_len);
bool mlrs_gcm_open(const mlrs_gcm_ctx_t *ctx, const uint8_t iv[MLRS_GCM_IV_LEN],
                   const uint8_t *aad, size_t aad_len, uint8_t *data,
                   size_t data_len, const uint8_t *tag, size_t tag_len);

#ifdef __cplusplus
}
#endif
