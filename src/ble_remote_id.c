#include "ble_remote_id.h"

#include "elrs_config.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

/*
 * This is an OpenDroneID Basic ID legacy advertisement helper. It is intended
 * for bench testing receivers, not as a complete Remote ID compliance module.
 */
#define ODID_PROTOCOL_VERSION 2U
#define ODID_MESSAGE_TYPE_BASIC_ID 0U
#define ODID_MESSAGE_TYPE_LOCATION 1U
#define ODID_ID_TYPE_SERIAL_NUMBER 1U
#define ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR 2U
#define ODID_STATUS_GROUND 1U
#define ODID_TIMESTAMP_INVALID 0xFFFFU
#define ODID_SERVICE_UUID 0xFFFAU
#define ODID_BLE_APPLICATION_CODE 0x0DU
#define ODID_LATITUDE_E7_MIN (-900000000)
#define ODID_LATITUDE_E7_MAX 900000000
#define ODID_LONGITUDE_E7_MIN (-1800000000)
#define ODID_LONGITUDE_E7_MAX 1800000000
#define ODID_ALTITUDE_M_MIN (-1000)
#define ODID_ALTITUDE_M_MAX 31767
#define ODID_DIRECTION_INVALID_CDEG 36100U

static char remote_id_uas_id[BLE_REMOTE_ID_UAS_ID_MAX + 1U];
static uint8_t remote_id_message_counter;
static volatile uint32_t remote_id_state_seq;
static volatile bool remote_id_location_valid;
static volatile int32_t remote_id_latitude_e7;
static volatile int32_t remote_id_longitude_e7;
static volatile int16_t remote_id_altitude_m;
static volatile uint16_t remote_id_ground_speed_cms;
static volatile uint16_t remote_id_direction_cdeg;
static volatile uint32_t remote_id_location_updated_ms;

typedef struct {
  bool valid;
  int32_t latitude_e7;
  int32_t longitude_e7;
  int16_t altitude_m;
  uint16_t ground_speed_cms;
  uint16_t direction_cdeg;
  uint32_t updated_ms;
  uint32_t seq;
} ble_remote_id_location_snapshot_t;

static ble_remote_id_location_snapshot_t remote_id_location_snapshot(void)
{
  ble_remote_id_location_snapshot_t snapshot;
  uint32_t start_seq;
  uint32_t end_seq;

  do {
    start_seq = remote_id_state_seq;
    if ((start_seq & 1U) != 0U) {
      continue;
    }

    snapshot.valid = remote_id_location_valid;
    snapshot.latitude_e7 = remote_id_latitude_e7;
    snapshot.longitude_e7 = remote_id_longitude_e7;
    snapshot.altitude_m = remote_id_altitude_m;
    snapshot.ground_speed_cms = remote_id_ground_speed_cms;
    snapshot.direction_cdeg = remote_id_direction_cdeg;
    snapshot.updated_ms = remote_id_location_updated_ms;
    snapshot.seq = start_seq;

    end_seq = remote_id_state_seq;
  } while (start_seq != end_seq || (end_seq & 1U) != 0U);

  return snapshot;
}

static void remote_id_location_write(bool valid,
                                     int32_t latitude_e7,
                                     int32_t longitude_e7,
                                     int16_t altitude_m,
                                     uint16_t ground_speed_cms,
                                     uint16_t direction_cdeg,
                                     uint32_t now_ms)
{
  remote_id_state_seq++;
  remote_id_location_valid = valid;
  remote_id_latitude_e7 = latitude_e7;
  remote_id_longitude_e7 = longitude_e7;
  remote_id_altitude_m = altitude_m;
  remote_id_ground_speed_cms = ground_speed_cms;
  remote_id_direction_cdeg = direction_cdeg;
  remote_id_location_updated_ms = now_ms;
  remote_id_state_seq++;
}

static bool is_allowed_uas_id_char(char c)
{
  return isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.';
}

static void make_default_uas_id(char *out, size_t out_len)
{
  uint8_t uid[6] = { 0 };
  if (out == NULL || out_len == 0U) {
    return;
  }

  if (elrs_config_get_uid(uid) == 0) {
    snprintf(out,
             out_len,
             "ELRS-%02X%02X%02X%02X%02X%02X",
             uid[0],
             uid[1],
             uid[2],
             uid[3],
             uid[4],
             uid[5]);
  } else {
    snprintf(out, out_len, "ELRS-SIW917-RX");
  }
}

void ble_remote_id_init_from_config(void)
{
  if (remote_id_uas_id[0] == '\0') {
    make_default_uas_id(remote_id_uas_id, sizeof(remote_id_uas_id));
  }
}

bool ble_remote_id_set_uas_id(const char *uas_id)
{
  size_t len;

  if (uas_id == NULL) {
    return false;
  }

  while (*uas_id == ' ' || *uas_id == '\t') {
    uas_id++;
  }

  len = strlen(uas_id);
  while (len > 0U &&
         (uas_id[len - 1U] == ' ' || uas_id[len - 1U] == '\t' ||
          uas_id[len - 1U] == '\r' || uas_id[len - 1U] == '\n')) {
    len--;
  }

  if (len == 0U || len > BLE_REMOTE_ID_UAS_ID_MAX) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    if (!is_allowed_uas_id_char(uas_id[i])) {
      return false;
    }
  }

  memset(remote_id_uas_id, 0, sizeof(remote_id_uas_id));
  memcpy(remote_id_uas_id, uas_id, len);
  return true;
}

const char *ble_remote_id_get_uas_id(void)
{
  ble_remote_id_init_from_config();
  return remote_id_uas_id;
}

bool ble_remote_id_set_location_e7(int32_t latitude_e7, int32_t longitude_e7, int16_t altitude_m)
{
  if (latitude_e7 < ODID_LATITUDE_E7_MIN || latitude_e7 > ODID_LATITUDE_E7_MAX ||
      longitude_e7 < ODID_LONGITUDE_E7_MIN || longitude_e7 > ODID_LONGITUDE_E7_MAX ||
      altitude_m < ODID_ALTITUDE_M_MIN || altitude_m > ODID_ALTITUDE_M_MAX) {
    return false;
  }

  /*
   * OpenDroneID treats both latitude and longitude set to zero as invalid or
   * unknown. Refuse that for the bench beacon so scanner apps do not plot us
   * near the Gulf of Guinea again.
   */
  if (latitude_e7 == 0 && longitude_e7 == 0) {
    return false;
  }

  remote_id_location_write(true,
                           latitude_e7,
                           longitude_e7,
                           altitude_m,
                           0U,
                           ODID_DIRECTION_INVALID_CDEG,
                           0U);
  return true;
}

void ble_remote_id_clear_location(void)
{
  remote_id_location_write(false, 0, 0, 0, 0U, ODID_DIRECTION_INVALID_CDEG, 0U);
}

bool ble_remote_id_location_is_set(void)
{
  return remote_id_location_valid;
}

bool ble_remote_id_location_is_fresh(uint32_t now_ms, uint32_t max_age_ms)
{
  ble_remote_id_location_snapshot_t snapshot = remote_id_location_snapshot();
  if (!snapshot.valid) {
    return false;
  }
  if (snapshot.updated_ms == 0U) {
    return true;
  }
  return (uint32_t)(now_ms - snapshot.updated_ms) <= max_age_ms;
}

uint32_t ble_remote_id_location_sequence(void)
{
  return remote_id_location_snapshot().seq;
}

bool ble_remote_id_update_from_crsf_gps(int32_t latitude_e7,
                                        int32_t longitude_e7,
                                        int16_t altitude_m,
                                        uint16_t ground_speed_cms,
                                        uint16_t direction_cdeg,
                                        uint32_t now_ms)
{
  if (latitude_e7 < ODID_LATITUDE_E7_MIN || latitude_e7 > ODID_LATITUDE_E7_MAX ||
      longitude_e7 < ODID_LONGITUDE_E7_MIN || longitude_e7 > ODID_LONGITUDE_E7_MAX ||
      altitude_m < ODID_ALTITUDE_M_MIN || altitude_m > ODID_ALTITUDE_M_MAX ||
      (latitude_e7 == 0 && longitude_e7 == 0)) {
    return false;
  }

  if (direction_cdeg > ODID_DIRECTION_INVALID_CDEG) {
    direction_cdeg = ODID_DIRECTION_INVALID_CDEG;
  }

  remote_id_location_write(true,
                           latitude_e7,
                           longitude_e7,
                           altitude_m,
                           ground_speed_cms,
                           direction_cdeg,
                           now_ms);
  return true;
}

static uint16_t encode_altitude_m(int16_t altitude_m)
{
  int32_t encoded = ((int32_t)altitude_m + 1000) * 2;
  if (encoded < 0) {
    encoded = 0;
  } else if (encoded > 65535) {
    encoded = 65535;
  }
  return (uint16_t)encoded;
}

static uint8_t encode_direction_byte(uint16_t direction_cdeg)
{
  if (direction_cdeg >= ODID_DIRECTION_INVALID_CDEG) {
    return 0U;
  }

  uint16_t degrees = (uint16_t)(direction_cdeg / 100U);
  if (degrees >= 360U) {
    degrees = 359U;
  }
  return (uint8_t)(degrees / 2U);
}

static uint8_t encode_horizontal_speed_byte(uint16_t ground_speed_cms)
{
  uint32_t encoded = ((uint32_t)ground_speed_cms + 12U) / 25U;
  if (encoded > 254U) {
    encoded = 254U;
  }
  return (uint8_t)encoded;
}

static void put_u16_le(uint8_t *out, uint16_t value)
{
  out[0] = (uint8_t)(value & 0xFFU);
  out[1] = (uint8_t)(value >> 8U);
}

static void put_i32_le(uint8_t *out, int32_t value)
{
  uint32_t raw = (uint32_t)value;
  out[0] = (uint8_t)(raw & 0xFFU);
  out[1] = (uint8_t)((raw >> 8U) & 0xFFU);
  out[2] = (uint8_t)((raw >> 16U) & 0xFFU);
  out[3] = (uint8_t)((raw >> 24U) & 0xFFU);
}

uint16_t ble_remote_id_build_basic_id_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE])
{
  const char *uas_id = ble_remote_id_get_uas_id();

  if (out == NULL) {
    return 0U;
  }

  memset(out, 0, BLE_REMOTE_ID_MESSAGE_SIZE);
  out[0] = (uint8_t)((ODID_MESSAGE_TYPE_BASIC_ID << 4U) | ODID_PROTOCOL_VERSION);
  out[1] = (uint8_t)((ODID_ID_TYPE_SERIAL_NUMBER << 4U) |
                     ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR);
  strncpy((char *)&out[2], uas_id, BLE_REMOTE_ID_UAS_ID_MAX);
  return BLE_REMOTE_ID_MESSAGE_SIZE;
}

uint16_t ble_remote_id_build_location_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE])
{
  ble_remote_id_location_snapshot_t snapshot = remote_id_location_snapshot();
  uint16_t altitude = encode_altitude_m(snapshot.altitude_m);

  if (out == NULL || !snapshot.valid) {
    return 0U;
  }

  memset(out, 0, BLE_REMOTE_ID_MESSAGE_SIZE);
  out[0] = (uint8_t)((ODID_MESSAGE_TYPE_LOCATION << 4U) | ODID_PROTOCOL_VERSION);
  out[1] = (uint8_t)(ODID_STATUS_GROUND << 4U);
  out[2] = encode_direction_byte(snapshot.direction_cdeg);
  out[3] = encode_horizontal_speed_byte(snapshot.ground_speed_cms);
  out[4] = 0U; /* Vertical speed 0 m/s. */
  put_i32_le(&out[5], snapshot.latitude_e7);
  put_i32_le(&out[9], snapshot.longitude_e7);
  put_u16_le(&out[13], altitude); /* Barometric altitude. */
  put_u16_le(&out[15], altitude); /* Geodetic altitude. */
  put_u16_le(&out[17], encode_altitude_m(0)); /* Height over takeoff. */
  out[19] = 0U; /* Unknown horizontal and vertical accuracy. */
  out[20] = 0U; /* Unknown baro and speed accuracy. */
  put_u16_le(&out[21], ODID_TIMESTAMP_INVALID);
  out[23] = 0U; /* Unknown timestamp accuracy. */
  out[24] = 0U;
  return BLE_REMOTE_ID_MESSAGE_SIZE;
}

uint16_t ble_remote_id_build_advertisement(uint8_t out[BLE_REMOTE_ID_ADV_MAX],
                                           ble_remote_id_adv_kind_t kind)
{
  uint16_t message_len;

  if (out == NULL) {
    return 0U;
  }

  memset(out, 0, BLE_REMOTE_ID_ADV_MAX);

  /*
   * Legacy advertising has only 31 bytes. For this bench beacon we omit Flags
   * and use one Service Data element:
   *   length=30, type=0x16, UUID=0xFFFA little-endian,
   *   ODID app code=0x0D, message counter, ODID message=25 bytes.
   */
  out[0] = 30U;
  out[1] = 0x16U; /* Service Data - 16-bit UUID */
  out[2] = (uint8_t)(ODID_SERVICE_UUID & 0xFFU);
  out[3] = (uint8_t)(ODID_SERVICE_UUID >> 8U);
  out[4] = ODID_BLE_APPLICATION_CODE;
  out[5] = remote_id_message_counter++;
  if (kind == BLE_REMOTE_ID_ADV_LOCATION) {
    message_len = ble_remote_id_build_location_message(&out[6]);
  } else {
    message_len = ble_remote_id_build_basic_id_message(&out[6]);
  }

  if (message_len != BLE_REMOTE_ID_MESSAGE_SIZE) {
    return 0U;
  }

  return 31U;
}

uint16_t ble_remote_id_build_basic_id_advertisement(uint8_t out[BLE_REMOTE_ID_ADV_MAX])
{
  return ble_remote_id_build_advertisement(out, BLE_REMOTE_ID_ADV_BASIC_ID);
}

uint16_t ble_remote_id_build_location_advertisement(uint8_t out[BLE_REMOTE_ID_ADV_MAX])
{
  return ble_remote_id_build_advertisement(out, BLE_REMOTE_ID_ADV_LOCATION);
}

void ble_remote_id_format_status(char *out, size_t out_len, bool advertising)
{
  ble_remote_id_location_snapshot_t snapshot = remote_id_location_snapshot();

  if (out == NULL || out_len == 0U) {
    return;
  }

  snprintf(out,
           out_len,
           "rid enabled=%u type=ble_legacy id=%s loc=%s seq=%lu uuid=0x%04X",
           advertising ? 1U : 0U,
           ble_remote_id_get_uas_id(),
           snapshot.valid ? "set" : "unset",
           (unsigned long)snapshot.seq,
           ODID_SERVICE_UUID);
}

void ble_remote_id_format_location(char *out, size_t out_len)
{
  ble_remote_id_location_snapshot_t snapshot = remote_id_location_snapshot();

  if (out == NULL || out_len == 0U) {
    return;
  }

  if (snapshot.valid) {
    snprintf(out,
             out_len,
             "rid loc=set lat_e7=%ld lon_e7=%ld alt_m=%d gs_cms=%u dir_cdeg=%u update_ms=%lu",
             (long)snapshot.latitude_e7,
             (long)snapshot.longitude_e7,
             (int)snapshot.altitude_m,
             (unsigned int)snapshot.ground_speed_cms,
             (unsigned int)snapshot.direction_cdeg,
             (unsigned long)snapshot.updated_ms);
  } else {
    snprintf(out, out_len, "rid loc=unset");
  }
}

void ble_remote_id_format_adv_hex(char *out, size_t out_len, ble_remote_id_adv_kind_t kind)
{
  uint8_t adv[BLE_REMOTE_ID_ADV_MAX] = { 0 };
  uint16_t adv_len = ble_remote_id_build_advertisement(adv, kind);
  size_t pos = 0;

  if (out == NULL || out_len == 0U) {
    return;
  }

  for (uint16_t i = 0; i < adv_len && pos + 3U < out_len; i++) {
    int wrote = snprintf(&out[pos], out_len - pos, "%02X", adv[i]);
    if (wrote <= 0) {
      break;
    }
    pos += (size_t)wrote;
  }
}
