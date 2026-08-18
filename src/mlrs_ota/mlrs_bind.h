#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELRS_RESERVED_BIND_PHRASE_OFFSET 0
#define ELRS_RESERVED_BIND_PHRASE_LEN 7

/* ELRS-compatible UID (MD5 of -DMY_BINDING_PHRASE="...") plus a
 * phrase-derived mLRS secret. Same 6-char phrase on TX and RX. */
void mlrs_bind_sanitize(char out[7], const char *in, uint8_t len);
bool mlrs_bind_apply(const char *phrase, uint8_t len);
void mlrs_bind_get_phrase(char out[7]);

#ifdef __cplusplus
}
#endif
