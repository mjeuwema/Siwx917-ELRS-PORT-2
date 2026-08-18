#include "mlrs_bind.h"

#include "common.h"
#include "elrs_config.h"
#include "md5.h"
#include "mlrs_ota.h"
#include "options.h"

#include <stdio.h>
#include <string.h>

#ifndef UID_LEN
#define UID_LEN 6
#endif

extern uint8_t UID[UID_LEN];

/* Official ELRS 4.1 SetBindPhrase: MD5 of -DMY_BINDING_PHRASE="..." */
static const uint8_t kElrsKey[] = "-DMY_BINDING_PHRASE=\"";

static void uid_from_elrs_phrase(const char *phrase, uint8_t uid[6]) {
  md5_context_t md5;
  uint8_t digest[16];
  const char quote = '"';
  const size_t plen = (phrase != nullptr) ? strlen(phrase) : 0;
  md5_init(&md5);
  md5_update(&md5, kElrsKey, sizeof(kElrsKey) - 1U);
  if (plen != 0) {
    md5_update(&md5, reinterpret_cast<const uint8_t *>(phrase), plen);
  }
  md5_update(&md5, reinterpret_cast<const uint8_t *>(&quote), 1);
  md5_final(&md5, digest);
  memcpy(uid, digest, 6);
}

static void secret_from_phrase(const char *phrase, uint8_t secret[32]) {
  uint8_t a[16];
  uint8_t b[16];
  const char *p = (phrase != nullptr) ? phrase : "";
  md5_context_t md5;
  md5_init(&md5);
  md5_update(&md5, reinterpret_cast<const uint8_t *>("mLRS-S1"), 7);
  md5_update(&md5, reinterpret_cast<const uint8_t *>(p), strlen(p));
  md5_final(&md5, a);
  md5_init(&md5);
  md5_update(&md5, reinterpret_cast<const uint8_t *>("mLRS-S2"), 7);
  md5_update(&md5, reinterpret_cast<const uint8_t *>(p), strlen(p));
  md5_final(&md5, b);
  memcpy(secret, a, 16);
  memcpy(secret + 16, b, 16);
}

void mlrs_bind_sanitize(char out[7], const char *in, uint8_t len) {
  static const char kChars[] = "abcdefghijklmnopqrstuvwxyz0123456789_#-.";
  memset(out, '_', 6);
  out[6] = 0;
  if (in == nullptr) {
    return;
  }
  uint8_t n = 0;
  for (uint8_t i = 0; i < len && n < 6; ++i) {
    char c = in[i];
    if (c == 0) {
      break;
    }
    if (c >= 'A' && c <= 'Z') {
      c = (char)(c - 'A' + 'a');
    }
    if (c == ' ') {
      c = '_';
    }
    if (strchr(kChars, c) == nullptr) {
      c = '_';
    }
    out[n++] = c;
  }
}

void mlrs_bind_get_phrase(char out[7]) {
  char full[ELRS_BIND_PHRASE_MAX + 1] = {};
  if (elrs_config_get_bind_phrase(full, sizeof(full)) != 0 || full[0] == 0) {
    memcpy(out, "------", 6);
    out[6] = 0;
    return;
  }
  mlrs_bind_sanitize(out, full, (uint8_t)strlen(full));
}

bool mlrs_bind_apply(const char *phrase, uint8_t len) {
  char full[ELRS_BIND_PHRASE_MAX + 1];
  memset(full, 0, sizeof(full));
  uint8_t n = 0;
  if (phrase != nullptr) {
    for (uint8_t i = 0; i < len && n < ELRS_BIND_PHRASE_MAX; ++i) {
      if (phrase[i] == 0) {
        break;
      }
      full[n++] = phrase[i];
    }
  }
  uint8_t uid[6];
  uint8_t secret[32];
  uid_from_elrs_phrase(full, uid);
  secret_from_phrase(full, secret);

  char prev[ELRS_BIND_PHRASE_MAX + 1] = {};
  (void)elrs_config_get_bind_phrase(prev, sizeof(prev));
  elrs_config_t *cfg = elrs_config_get();
  if (cfg != nullptr && memcmp(cfg->uid, uid, 6) == 0 &&
      strcmp(prev, full) == 0) {
    return false;
  }

  memcpy(UID, uid, 6);
  firmwareOptions.hasUID = (uid[0] | uid[1] | uid[2] | uid[3] | uid[4] |
                            uid[5]) != 0
                               ? 1
                               : 0;
  memcpy(firmwareOptions.uid, uid, 6);
  (void)elrs_config_set_uid(uid);
  (void)elrs_config_set_mlrs_secret(secret);
  (void)elrs_config_set_bind_phrase(full);
  (void)elrs_config_save();
  mlrs_ota_on_bind_changed();
  printf("[mLRS] bind phrase '%s' uid=%02X%02X%02X%02X%02X%02X\n", full, uid[0],
         uid[1], uid[2], uid[3], uid[4], uid[5]);
  return true;
}
