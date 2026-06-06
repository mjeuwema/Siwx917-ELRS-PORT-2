#include "ble_remote_id.h"

#include "elrs_config.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

/*
 * OpenDroneID Bluetooth helpers. The legacy single-message advertisement is
 * kept for scanner compatibility tests; Bluetooth 5 Remote ID uses the
 * Message Pack container built below.
 */
#define ODID_PROTOCOL_VERSION 2U
#define ODID_MESSAGE_TYPE_BASIC_ID 0U
#define ODID_MESSAGE_TYPE_LOCATION 1U
#define ODID_MESSAGE_TYPE_SELF_ID 3U
#define ODID_MESSAGE_TYPE_SYSTEM 4U
#define ODID_MESSAGE_TYPE_OPERATOR_ID 5U
#define ODID_MESSAGE_TYPE_PACKED 0x0FU
#define ODID_ID_TYPE_SERIAL_NUMBER 1U
#define ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR 2U
#define ODID_STATUS_GROUND 1U
#define ODID_STATUS_AIRBORNE 2U
#define ODID_DESC_TYPE_TEXT 0U
#define ODID_OPERATOR_ID_TYPE_STANDARD 0U
#define ODID_OPERATOR_LOCATION_TYPE_TAKEOFF 0U
#define ODID_CLASSIFICATION_UNDECLARED 0U
#define ODID_CATEGORY_UNDECLARED 0U
#define ODID_CLASS_UNDECLARED 0U
#define ODID_TIMESTAMP_INVALID 0xFFFFU
#define ODID_SERVICE_UUID 0xFFFAU
#define ODID_BLE_APPLICATION_CODE 0x0DU
#define ODID_AD_TYPE_SERVICE_DATA_16 0x16U
#define ODID_LATITUDE_E7_MIN (-900000000)
#define ODID_LATITUDE_E7_MAX 900000000
#define ODID_LONGITUDE_E7_MIN (-1800000000)
#define ODID_LONGITUDE_E7_MAX 1800000000
#define ODID_ALTITUDE_M_MIN (-1000)
#define ODID_ALTITUDE_M_MAX 31767
#define ODID_DIRECTION_INVALID_CDEG 36100U
#define ODID_SPEED_HORIZONTAL_INVALID_CMS 25500U
#define ODID_SPEED_HORIZONTAL_LOW_SCALE_MAX_CMS 6375U
#define ODID_SPEED_HORIZONTAL_HIGH_SCALE_STEP_CMS 75U
#define ODID_HEIGHT_AIRBORNE_THRESHOLD_M 3
#define ODID_AIRBORNE_SPEED_THRESHOLD_CMS 200U
#define ODID_SYSTEM_TIMESTAMP_UNKNOWN 0U

static char remote_id_uas_id[BLE_REMOTE_ID_UAS_ID_MAX + 1U];
static char remote_id_self_id[BLE_REMOTE_ID_SELF_ID_MAX + 1U];
static char remote_id_operator_id[BLE_REMOTE_ID_OPERATOR_ID_MAX + 1U];
static uint8_t remote_id_message_counter;
static bool remote_id_operator_id_valid;
static volatile uint32_t remote_id_state_seq;
static volatile bool remote_id_location_valid;
static volatile int32_t remote_id_latitude_e7;
static volatile int32_t remote_id_longitude_e7;
static volatile int16_t remote_id_altitude_m;
static volatile uint16_t remote_id_ground_speed_cms;
static volatile uint16_t remote_id_direction_cdeg;
static volatile uint32_t remote_id_location_updated_ms;
static volatile bool remote_id_takeoff_valid;
static volatile int32_t remote_id_takeoff_latitude_e7;
static volatile int32_t remote_id_takeoff_longitude_e7;
static volatile int16_t remote_id_takeoff_altitude_m;

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

typedef struct {
  bool valid;
  int32_t latitude_e7;
  int32_t longitude_e7;
  int16_t altitude_m;
} ble_remote_id_takeoff_snapshot_t;

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
  if (valid && !remote_id_takeoff_valid) {
    remote_id_takeoff_valid = true;
    remote_id_takeoff_latitude_e7 = latitude_e7;
    remote_id_takeoff_longitude_e7 = longitude_e7;
    remote_id_takeoff_altitude_m = altitude_m;
  }
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

static bool is_ascii_printable_text_char(char c)
{
  unsigned char value = (unsigned char)c;
  return value >= 0x20U && value <= 0x7EU;
}

static bool set_trimmed_ascii_text(char *dst,
                                   size_t dst_size,
                                   const char *text,
                                   size_t max_len,
                                   bool (*char_check)(char))
{
  size_t len;

  if (dst == NULL || dst_size == 0U || text == NULL) {
    return false;
  }

  while (*text == ' ' || *text == '\t') {
    text++;
  }

  len = strlen(text);
  while (len > 0U &&
         (text[len - 1U] == ' ' || text[len - 1U] == '\t' ||
          text[len - 1U] == '\r' || text[len - 1U] == '\n')) {
    len--;
  }

  if (len == 0U || len > max_len || len >= dst_size) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    if (!char_check(text[i])) {
      return false;
    }
  }

  memset(dst, 0, dst_size);
  memcpy(dst, text, len);
  return true;
}

static ble_remote_id_takeoff_snapshot_t remote_id_takeoff_snapshot(void)
{
  ble_remote_id_takeoff_snapshot_t snapshot;
  uint32_t start_seq;
  uint32_t end_seq;

  do {
    start_seq = remote_id_state_seq;
    if ((start_seq & 1U) != 0U) {
      continue;
    }

    snapshot.valid = remote_id_takeoff_valid;
    snapshot.latitude_e7 = remote_id_takeoff_latitude_e7;
    snapshot.longitude_e7 = remote_id_takeoff_longitude_e7;
    snapshot.altitude_m = remote_id_takeoff_altitude_m;

    end_seq = remote_id_state_seq;
  } while (start_seq != end_seq || (end_seq & 1U) != 0U);

  return snapshot;
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

static void make_default_self_id(char *out, size_t out_len)
{
  if (out == NULL || out_len == 0U) {
    return;
  }

  snprintf(out, out_len, "ELRS Remote ID");
}

void ble_remote_id_init_from_config(void)
{
  if (remote_id_uas_id[0] == '\0') {
    make_default_uas_id(remote_id_uas_id, sizeof(remote_id_uas_id));
  }
  if (remote_id_self_id[0] == '\0') {
    make_default_self_id(remote_id_self_id, sizeof(remote_id_self_id));
  }
}

bool ble_remote_id_set_uas_id(const char *uas_id)
{
  return set_trimmed_ascii_text(remote_id_uas_id,
                                sizeof(remote_id_uas_id),
                                uas_id,
                                BLE_REMOTE_ID_UAS_ID_MAX,
                                is_allowed_uas_id_char);
}

const char *ble_remote_id_get_uas_id(void)
{
  ble_remote_id_init_from_config();
  return remote_id_uas_id;
}

bool ble_remote_id_set_self_id_text(const char *text)
{
  return set_trimmed_ascii_text(remote_id_self_id,
                                sizeof(remote_id_self_id),
                                text,
                                BLE_REMOTE_ID_SELF_ID_MAX,
                                is_ascii_printable_text_char);
}

const char *ble_remote_id_get_self_id_text(void)
{
  ble_remote_id_init_from_config();
  return remote_id_self_id;
}

void ble_remote_id_reset_self_id_text(void)
{
  memset(remote_id_self_id, 0, sizeof(remote_id_self_id));
  make_default_self_id(remote_id_self_id, sizeof(remote_id_self_id));
}

bool ble_remote_id_set_operator_id_text(const char *text)
{
  if (!set_trimmed_ascii_text(remote_id_operator_id,
                              sizeof(remote_id_operator_id),
                              text,
                              BLE_REMOTE_ID_OPERATOR_ID_MAX,
                              is_ascii_printable_text_char)) {
    return false;
  }

  remote_id_operator_id_valid = true;
  return true;
}

const char *ble_remote_id_get_operator_id_text(void)
{
  return remote_id_operator_id;
}

void ble_remote_id_clear_operator_id_text(void)
{
  memset(remote_id_operator_id, 0, sizeof(remote_id_operator_id));
  remote_id_operator_id_valid = false;
}

bool ble_remote_id_operator_id_is_set(void)
{
  return remote_id_operator_id_valid && remote_id_operator_id[0] != '\0';
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

void ble_remote_id_clear_takeoff_location(void)
{
  remote_id_state_seq++;
  remote_id_takeoff_valid = false;
  remote_id_takeoff_latitude_e7 = 0;
  remote_id_takeoff_longitude_e7 = 0;
  remote_id_takeoff_altitude_m = 0;
  remote_id_state_seq++;
}

bool ble_remote_id_takeoff_is_set(void)
{
  return remote_id_takeoff_snapshot().valid;
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

static uint8_t encode_direction_byte(uint16_t direction_cdeg, bool *westbound)
{
  if (westbound != NULL) {
    *westbound = false;
  }

  if (direction_cdeg >= ODID_DIRECTION_INVALID_CDEG) {
    return 0U;
  }

  uint16_t degrees = (uint16_t)((direction_cdeg + 50U) / 100U);
  if (degrees == 360U) {
    degrees = 0U;
  } else if (degrees > 360U) {
    degrees = 359U;
  }

  if (degrees >= 180U) {
    degrees = (uint16_t)(degrees - 180U);
    if (westbound != NULL) {
      *westbound = true;
    }
  }

  return (uint8_t)degrees;
}

static uint8_t encode_horizontal_speed_byte(uint16_t ground_speed_cms, bool *high_speed_scale)
{
  uint32_t encoded;

  if (high_speed_scale != NULL) {
    *high_speed_scale = false;
  }

  if (ground_speed_cms >= ODID_SPEED_HORIZONTAL_INVALID_CMS) {
    return 255U;
  }

  if (ground_speed_cms <= ODID_SPEED_HORIZONTAL_LOW_SCALE_MAX_CMS) {
    encoded = ((uint32_t)ground_speed_cms + 12U) / 25U;
  } else {
    if (high_speed_scale != NULL) {
      *high_speed_scale = true;
    }
    encoded = ((uint32_t)(ground_speed_cms - ODID_SPEED_HORIZONTAL_LOW_SCALE_MAX_CMS) +
               (ODID_SPEED_HORIZONTAL_HIGH_SCALE_STEP_CMS / 2U)) /
              ODID_SPEED_HORIZONTAL_HIGH_SCALE_STEP_CMS;
  }

  if (encoded > 255U) {
    encoded = 255U;
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

static uint8_t remote_id_location_status(const ble_remote_id_location_snapshot_t *location,
                                         const ble_remote_id_takeoff_snapshot_t *takeoff)
{
  if (location == NULL) {
    return ODID_STATUS_GROUND;
  }

  if (location->ground_speed_cms >= ODID_AIRBORNE_SPEED_THRESHOLD_CMS) {
    return ODID_STATUS_AIRBORNE;
  }

  if (takeoff != NULL && takeoff->valid) {
    int16_t height_m = (int16_t)(location->altitude_m - takeoff->altitude_m);
    if (height_m >= ODID_HEIGHT_AIRBORNE_THRESHOLD_M ||
        height_m <= -ODID_HEIGHT_AIRBORNE_THRESHOLD_M) {
      return ODID_STATUS_AIRBORNE;
    }
  }

  return ODID_STATUS_GROUND;
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
  ble_remote_id_takeoff_snapshot_t takeoff = remote_id_takeoff_snapshot();
  uint16_t altitude = encode_altitude_m(snapshot.altitude_m);
  int16_t height_over_takeoff_m = 0;
  bool westbound = false;
  bool high_speed_scale = false;
  uint8_t direction = encode_direction_byte(snapshot.direction_cdeg, &westbound);
  uint8_t horizontal_speed =
      encode_horizontal_speed_byte(snapshot.ground_speed_cms, &high_speed_scale);

  if (out == NULL || !snapshot.valid) {
    return 0U;
  }

  if (takeoff.valid) {
    height_over_takeoff_m = (int16_t)(snapshot.altitude_m - takeoff.altitude_m);
  }

  memset(out, 0, BLE_REMOTE_ID_MESSAGE_SIZE);
  out[0] = (uint8_t)((ODID_MESSAGE_TYPE_LOCATION << 4U) | ODID_PROTOCOL_VERSION);
  out[1] = (uint8_t)((remote_id_location_status(&snapshot, &takeoff) << 4U) |
                     (westbound ? 0x02U : 0U) |
                     (high_speed_scale ? 0x01U : 0U));
  out[2] = direction;
  out[3] = horizontal_speed;
  out[4] = 0U; /* Vertical speed 0 m/s. */
  put_i32_le(&out[5], snapshot.latitude_e7);
  put_i32_le(&out[9], snapshot.longitude_e7);
  put_u16_le(&out[13], altitude); /* Barometric altitude. */
  put_u16_le(&out[15], altitude); /* Geodetic altitude. */
  put_u16_le(&out[17], encode_altitude_m(height_over_takeoff_m));
  out[19] = 0U; /* Unknown horizontal and vertical accuracy. */
  out[20] = 0U; /* Unknown baro and speed accuracy. */
  put_u16_le(&out[21], ODID_TIMESTAMP_INVALID);
  out[23] = 0U; /* Unknown timestamp accuracy. */
  out[24] = 0U;
  return BLE_REMOTE_ID_MESSAGE_SIZE;
}

uint16_t ble_remote_id_build_self_id_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE])
{
  const char *self_id = ble_remote_id_get_self_id_text();

  if (out == NULL || self_id == NULL || self_id[0] == '\0') {
    return 0U;
  }

  memset(out, 0, BLE_REMOTE_ID_MESSAGE_SIZE);
  out[0] = (uint8_t)((ODID_MESSAGE_TYPE_SELF_ID << 4U) | ODID_PROTOCOL_VERSION);
  out[1] = ODID_DESC_TYPE_TEXT;
  strncpy((char *)&out[2], self_id, BLE_REMOTE_ID_SELF_ID_MAX);
  return BLE_REMOTE_ID_MESSAGE_SIZE;
}

uint16_t ble_remote_id_build_system_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE])
{
  ble_remote_id_takeoff_snapshot_t takeoff = remote_id_takeoff_snapshot();

  if (out == NULL || !takeoff.valid) {
    return 0U;
  }

  memset(out, 0, BLE_REMOTE_ID_MESSAGE_SIZE);
  out[0] = (uint8_t)((ODID_MESSAGE_TYPE_SYSTEM << 4U) | ODID_PROTOCOL_VERSION);
  out[1] = (uint8_t)(ODID_OPERATOR_LOCATION_TYPE_TAKEOFF |
                     (ODID_CLASSIFICATION_UNDECLARED << 2U));
  put_i32_le(&out[2], takeoff.latitude_e7);
  put_i32_le(&out[6], takeoff.longitude_e7);
  put_u16_le(&out[10], 1U);
  out[12] = 0U; /* Area radius 0 m. */
  put_u16_le(&out[13], encode_altitude_m(ODID_ALTITUDE_M_MIN)); /* Unknown ceiling. */
  put_u16_le(&out[15], encode_altitude_m(ODID_ALTITUDE_M_MIN)); /* Unknown floor. */
  out[17] = (uint8_t)(ODID_CLASS_UNDECLARED |
                      (ODID_CATEGORY_UNDECLARED << 4U));
  put_u16_le(&out[18], encode_altitude_m(takeoff.altitude_m));

  /*
   * We do not have a trusted UTC/GNSS epoch source on the RX yet, so keep the
   * System timestamp at zero until that data path exists.
   */
  out[20] = (uint8_t)(ODID_SYSTEM_TIMESTAMP_UNKNOWN & 0xFFU);
  out[21] = (uint8_t)((ODID_SYSTEM_TIMESTAMP_UNKNOWN >> 8U) & 0xFFU);
  out[22] = (uint8_t)((ODID_SYSTEM_TIMESTAMP_UNKNOWN >> 16U) & 0xFFU);
  out[23] = (uint8_t)((ODID_SYSTEM_TIMESTAMP_UNKNOWN >> 24U) & 0xFFU);
  out[24] = 0U;
  return BLE_REMOTE_ID_MESSAGE_SIZE;
}

uint16_t ble_remote_id_build_operator_id_message(uint8_t out[BLE_REMOTE_ID_MESSAGE_SIZE])
{
  const char *operator_id = ble_remote_id_get_operator_id_text();

  if (out == NULL || !ble_remote_id_operator_id_is_set()) {
    return 0U;
  }

  memset(out, 0, BLE_REMOTE_ID_MESSAGE_SIZE);
  out[0] = (uint8_t)((ODID_MESSAGE_TYPE_OPERATOR_ID << 4U) | ODID_PROTOCOL_VERSION);
  out[1] = ODID_OPERATOR_ID_TYPE_STANDARD;
  strncpy((char *)&out[2], operator_id, BLE_REMOTE_ID_OPERATOR_ID_MAX);
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

uint16_t ble_remote_id_build_message_pack(uint8_t *out, uint16_t max_len, bool include_location)
{
  uint16_t pos = BLE_REMOTE_ID_MESSAGE_PACK_HEADER_SIZE;
  uint8_t message_count = 0U;
  uint16_t message_len;

  if (out == NULL || max_len < BLE_REMOTE_ID_MESSAGE_PACK_HEADER_SIZE) {
    return 0U;
  }

  memset(out, 0, max_len);
  out[0] = (uint8_t)((ODID_MESSAGE_TYPE_PACKED << 4U) | ODID_PROTOCOL_VERSION);
  out[1] = BLE_REMOTE_ID_MESSAGE_SIZE;

  if ((uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE) > max_len ||
      ble_remote_id_build_basic_id_message(&out[pos]) != BLE_REMOTE_ID_MESSAGE_SIZE) {
    return 0U;
  }
  pos = (uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE);
  message_count++;

  if (include_location &&
      message_count < BLE_REMOTE_ID_MESSAGE_PACK_MAX_MESSAGES &&
      (uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE) <= max_len) {
    message_len = ble_remote_id_build_location_message(&out[pos]);
    if (message_len == BLE_REMOTE_ID_MESSAGE_SIZE) {
      pos = (uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE);
      message_count++;
    }
  }

  if (message_count < BLE_REMOTE_ID_MESSAGE_PACK_MAX_MESSAGES &&
      (uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE) <= max_len) {
    message_len = ble_remote_id_build_self_id_message(&out[pos]);
    if (message_len == BLE_REMOTE_ID_MESSAGE_SIZE) {
      pos = (uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE);
      message_count++;
    }
  }

  if (message_count < BLE_REMOTE_ID_MESSAGE_PACK_MAX_MESSAGES &&
      (uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE) <= max_len) {
    message_len = ble_remote_id_build_system_message(&out[pos]);
    if (message_len == BLE_REMOTE_ID_MESSAGE_SIZE) {
      pos = (uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE);
      message_count++;
    }
  }

  if (message_count < BLE_REMOTE_ID_MESSAGE_PACK_MAX_MESSAGES &&
      (uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE) <= max_len) {
    message_len = ble_remote_id_build_operator_id_message(&out[pos]);
    if (message_len == BLE_REMOTE_ID_MESSAGE_SIZE) {
      pos = (uint16_t)(pos + BLE_REMOTE_ID_MESSAGE_SIZE);
      message_count++;
    }
  }

  out[2] = message_count;
  return pos;
}

uint16_t ble_remote_id_build_bt5_advertisement(uint8_t *out,
                                               uint16_t max_len,
                                               bool include_location)
{
  uint16_t pack_len;
  uint16_t total_len;

  if (out == NULL || max_len < 6U) {
    return 0U;
  }

  memset(out, 0, max_len);
  out[1] = ODID_AD_TYPE_SERVICE_DATA_16;
  out[2] = (uint8_t)(ODID_SERVICE_UUID & 0xFFU);
  out[3] = (uint8_t)(ODID_SERVICE_UUID >> 8U);
  out[4] = ODID_BLE_APPLICATION_CODE;
  out[5] = remote_id_message_counter++;

  pack_len = ble_remote_id_build_message_pack(&out[6],
                                              (uint16_t)(max_len - 6U),
                                              include_location);
  if (pack_len == 0U) {
    return 0U;
  }

  total_len = (uint16_t)(6U + pack_len);
  if (total_len > max_len || total_len > 255U) {
    return 0U;
  }

  /*
   * AD length excludes this length byte and includes AD type + UUID +
   * OpenDroneID app code + counter + message-pack bytes.
   */
  out[0] = (uint8_t)(total_len - 1U);
  return total_len;
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
           "rid enabled=%u type=ble_bt5_msgpack id=%s self=%s loc=%s takeoff=%s opid=%s seq=%lu uuid=0x%04X",
           advertising ? 1U : 0U,
           ble_remote_id_get_uas_id(),
           ble_remote_id_get_self_id_text()[0] != '\0' ? "set" : "unset",
           snapshot.valid ? "set" : "unset",
           remote_id_takeoff_snapshot().valid ? "set" : "unset",
           ble_remote_id_operator_id_is_set() ? "set" : "unset",
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

void ble_remote_id_format_takeoff(char *out, size_t out_len)
{
  ble_remote_id_takeoff_snapshot_t snapshot = remote_id_takeoff_snapshot();

  if (out == NULL || out_len == 0U) {
    return;
  }

  if (snapshot.valid) {
    snprintf(out,
             out_len,
             "rid takeoff=set lat_e7=%ld lon_e7=%ld alt_m=%d",
             (long)snapshot.latitude_e7,
             (long)snapshot.longitude_e7,
             (int)snapshot.altitude_m);
  } else {
    snprintf(out, out_len, "rid takeoff=unset");
  }
}

void ble_remote_id_format_bt5_adv_hex(char *out, size_t out_len, bool include_location)
{
  uint8_t adv[BLE_REMOTE_ID_BT5_ADV_MAX] = { 0 };
  uint16_t adv_len = ble_remote_id_build_bt5_advertisement(adv,
                                                           sizeof(adv),
                                                           include_location);
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
