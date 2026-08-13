#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MLRS_GCM_KEY_LEN 16
#define MLRS_GCM_IV_LEN 12
#define MLRS_GCM_TAG_LEN 16
#define MLRS_GCM_SALT_LEN 4
#define MLRS_GCM_DIR_UPLINK 0x01
#define MLRS_GCM_DIR_DOWNLINK 0x02

typedef struct {
  uint32_t round_key[44];
  uint8_t H[16];
  uint8_t salt[MLRS_GCM_SALT_LEN];
} mlrs_gcm_ctx_t;

bool mlrs_gcm_selftest(void);
void mlrs_gcm_derive_from_uid(const uint8_t uid[6], mlrs_gcm_ctx_t *uplink,
                              mlrs_gcm_ctx_t *downlink);
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
