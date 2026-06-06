/*******************************************************************************
 * Standalone BLE GATT probe for SiWx917 ELRS RX.
 *
 * This intentionally does not touch the working WiFi Web UI path. It starts the
 * WiseConnect BLE stack as a peripheral, adds one small custom service, and
 * advertises so we can validate BLE before attempting WiFi/BLE coexistence.
 ******************************************************************************/

#include "ble_gatt_probe.h"

#include "ble_config.h"
#include "ble_remote_id.h"
#include "cmsis_os2.h"
#include "elrs_config.h"
#include "rsi_ble.h"
#include "rsi_ble_apis.h"
#include "rsi_ble_common_config.h"
#include "rsi_bt_common.h"
#include "rsi_bt_common_apis.h"
#include "rsi_common_apis.h"
#include "rsi_debug.h"
#include "rsi_utils.h"
#include "sl_constants.h"
#include "sl_utility.h"
#include "sl_wifi.h"
#include "sl_wifi_callback_framework.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern bool device_initialized;

#ifndef BLE_LOG_VERBOSE
#define BLE_LOG_VERBOSE 0
#endif

#if BLE_LOG_VERBOSE
#define BLE_LOG_DEBUGOUT(...) DEBUGOUT(__VA_ARGS__)
#else
#define BLE_LOG_DEBUGOUT(...) do { } while (0)
#endif

#define BLE_PROBE_NAME "ELRS-RX-BLE"

#define BLE_UUID_CHAR_DECL 0x2803
#define BLE_UUID_CCCD 0x2902

/* 16-bit vendor-style UUIDs keep this first probe close to SiLabs examples. */
#define BLE_UUID_ELRS_SERVICE 0xE7E0
#define BLE_UUID_ELRS_STATUS 0xE7E1
#define BLE_UUID_ELRS_COMMAND 0xE7E2

#define BLE_VALUE_MAX 320U
#define BLE_NOTIFY_CHUNK_MAX 20U
#define BLE_NOTIFY_INTER_CHUNK_DELAY_MS 15U
#define BLE_NOTIFY_RETRY_DELAY_MS 30U
#define BLE_NOTIFY_MAX_RETRIES 3U
#define BLE_STATUS_DEV_BUF_FULL ((int32_t)-31)
#define BLE_LAST_COMMAND_MAX 32U
/* BLE advertising intervals are 0.625 ms units: 0x0640 = exactly 1000 ms. */
#define BLE_REMOTE_ID_ADV_INTERVAL_1S 0x0640U

#define BLE_ATT_PROPERTY_READ 0x02
#define BLE_ATT_PROPERTY_WRITE_NO_RESPONSE 0x04
#define BLE_ATT_PROPERTY_WRITE 0x08
#define BLE_ATT_PROPERTY_NOTIFY 0x10

#define BLE_EVENT_CONNECTED 0
#define BLE_EVENT_DISCONNECTED 1
#define BLE_EVENT_WRITE 2
#define BLE_EVENT_READ 3
#define BLE_REMOTE_ID_DYNAMIC_UPDATE_SECONDS 1U
#define BLE_REMOTE_ID_LOCATION_STALE_MS 3000U
#define BLE_AE_ADV_HANDLE 0x00U
#define BLE_AE_ADV_INTERVAL_1S 0x0640U
#define BLE_AE_TEST_PAYLOAD_MAX 96U
#define BLE_REMOTE_ID_AE_PAYLOAD_MAX 64U

typedef enum {
  BLE_ADV_MODE_ELRS_CONFIG = 0,
  BLE_ADV_MODE_REMOTE_ID_BASIC = 1,
} ble_adv_mode_t;

static volatile uint32_t ble_event_map;
static osSemaphoreId_t ble_sem;
static rsi_ble_event_conn_status_t ble_conn;
static rsi_ble_event_disconnect_t ble_disconnect;
static rsi_ble_event_write_t ble_write;
static rsi_ble_read_req_t ble_read;
static uint16_t status_handle;
static uint16_t status_cccd_handle;
static uint16_t command_handle;
static uint16_t command_cccd_handle;
static bool is_connected;
static bool status_notifications_enabled;
static bool command_notifications_enabled;
static bool config_ready;
static bool config_dirty;
static volatile bool ble_task_started;
static volatile bool ble_service_ready;
static volatile bool ble_advertising;
static volatile bool ble_ae_advertising;
static volatile bool ble_ae_params_configured;
static volatile bool ble_config_advertise_when_remote_id_off = true;
static volatile bool ble_remote_id_requested_enabled;
static volatile bool ble_remote_id_request_pending;
static ble_adv_mode_t ble_adv_mode = BLE_ADV_MODE_ELRS_CONFIG;
static ble_remote_id_adv_kind_t ble_remote_id_next_adv_kind = BLE_REMOTE_ID_ADV_BASIC_ID;
static uint32_t ble_remote_id_restore_tick;
static uint32_t ble_remote_id_dynamic_tick;

static uint8_t status_value[BLE_VALUE_MAX] = "ready";
static uint16_t status_value_len = 5;
static uint16_t status_write_len = 5;
static uint8_t payload_value[BLE_VALUE_MAX] = "ready";
static uint16_t payload_value_len = 5;
static uint8_t command_value[BLE_VALUE_MAX] = "idle";
static uint16_t command_value_len = 4;
static char last_command_text[BLE_LAST_COMMAND_MAX];
static uint32_t response_seq;
static uint32_t connect_count;
static uint32_t disconnect_count;
static uint32_t write_count;
static uint32_t read_count;
static uint32_t notify_count;
static uint32_t notify_error_count;
static uint16_t last_write_handle;
static uint16_t last_read_handle;

static bool ascii_equal_ignore_case(const char *a, const char *b);
static char *trim_ascii(char *text);
static void ble_publish_ephemeral_response(const char *response);
static int32_t ble_ae_stop_advertising(void);
static int32_t ble_apply_remote_id_ae_advertisement(ble_remote_id_adv_kind_t kind);

static uint16_t ble_build_elrs_advertisement(uint8_t adv[31])
{
  uint16_t adv_len = 0;
  const uint8_t name_len = (uint8_t)strlen(BLE_PROBE_NAME);

  adv[adv_len++] = 2;
  adv[adv_len++] = 1;
  adv[adv_len++] = 6;
  adv[adv_len++] = 3;
  adv[adv_len++] = 3;
  adv[adv_len++] = (uint8_t)(BLE_UUID_ELRS_SERVICE & 0xFFU);
  adv[adv_len++] = (uint8_t)(BLE_UUID_ELRS_SERVICE >> 8);
  adv[adv_len++] = (uint8_t)(name_len + 1U);
  adv[adv_len++] = 9;
  memcpy(&adv[adv_len], BLE_PROBE_NAME, name_len);
  adv_len = (uint16_t)(adv_len + name_len);
  return adv_len;
}

static uint32_t ble_seconds_to_ticks(uint16_t seconds)
{
  uint32_t freq = osKernelGetTickFreq();
  if (freq == 0U) {
    freq = 1000U;
  }
  return (uint32_t)seconds * freq;
}

static uint32_t ble_ticks_to_ms(uint32_t ticks)
{
  uint32_t freq = osKernelGetTickFreq();
  if (freq == 0U) {
    freq = 1000U;
  }
  return (uint32_t)(((uint64_t)ticks * 1000U) / freq);
}

static bool ble_tick_reached(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0;
}

static const char *ble_remote_id_adv_kind_name(ble_remote_id_adv_kind_t kind)
{
  return (kind == BLE_REMOTE_ID_ADV_LOCATION) ? "Remote ID Location" : "Remote ID Basic ID";
}

static int32_t ble_apply_fixed_remote_id_power(void)
{
  int8_t sdk_index = RSI_BLE_PWR_INX;
  int32_t status = rsi_ble_set_ble_tx_power(sdk_index);
  if (status != RSI_SUCCESS) {
    DEBUGOUT("[BLE] Remote ID TX power setup failed index=%d status=0x%lX\n",
             (int)sdk_index,
             (unsigned long)status);
  } else {
    BLE_LOG_DEBUGOUT("[BLE] Remote ID TX power fixed 0 dBm mode -> SDK index %d\n",
                     (int)sdk_index);
  }
  return status;
}

static bool ble_should_advertise_current_mode(void)
{
  return ble_adv_mode == BLE_ADV_MODE_REMOTE_ID_BASIC ||
         ble_config_advertise_when_remote_id_off;
}

static int32_t ble_start_current_advertising(void)
{
  rsi_ble_req_adv_t adv = { 0 };

  adv.status = RSI_BLE_START_ADV;
  adv.adv_type =
      (ble_adv_mode == BLE_ADV_MODE_REMOTE_ID_BASIC) ? UNDIR_NON_CONN : UNDIR_CONN;
  adv.filter_type = RSI_BLE_ADV_FILTER_TYPE;
  adv.direct_addr_type = RSI_BLE_ADV_DIR_ADDR_TYPE;
  rsi_ascii_dev_address_to_6bytes_rev(adv.direct_addr,
                                      (int8_t *)RSI_BLE_ADV_DIR_ADDR);
  adv.adv_int_min = (ble_adv_mode == BLE_ADV_MODE_REMOTE_ID_BASIC)
                        ? BLE_REMOTE_ID_ADV_INTERVAL_1S
                        : RSI_BLE_ADV_INT_MIN;
  adv.adv_int_max = (ble_adv_mode == BLE_ADV_MODE_REMOTE_ID_BASIC)
                        ? BLE_REMOTE_ID_ADV_INTERVAL_1S
                        : RSI_BLE_ADV_INT_MAX;
  adv.own_addr_type = LE_PUBLIC_ADDRESS;
  adv.adv_channel_map = RSI_BLE_ADV_CHANNEL_MAP;

  return rsi_ble_start_advertising_with_values(&adv);
}

static int32_t ble_apply_current_advertisement(void)
{
  uint8_t adv[31] = { 0 };
  uint16_t adv_len;
  int32_t status;
  const char *adv_name;

  if (ble_adv_mode == BLE_ADV_MODE_ELRS_CONFIG &&
      !ble_config_advertise_when_remote_id_off) {
    if (ble_ae_advertising) {
      status = ble_ae_stop_advertising();
      if (status != RSI_SUCCESS) {
        DEBUGOUT("[BLE] stop AE advertising failed status=0x%lX\n",
                 (unsigned long)status);
        return status;
      }
    }

    if (!ble_advertising) {
      return RSI_SUCCESS;
    }

    status = rsi_ble_stop_advertising();
    if (status != RSI_SUCCESS) {
      DEBUGOUT("[BLE] stop advertising failed status=0x%lX\n",
               (unsigned long)status);
      return status;
    }
    ble_advertising = false;
    return RSI_SUCCESS;
  }

  if (ble_adv_mode == BLE_ADV_MODE_REMOTE_ID_BASIC) {
    ble_remote_id_adv_kind_t kind = ble_remote_id_next_adv_kind;
    if (kind == BLE_REMOTE_ID_ADV_LOCATION && !ble_remote_id_location_is_set()) {
      kind = BLE_REMOTE_ID_ADV_BASIC_ID;
    }

    return ble_apply_remote_id_ae_advertisement(kind);
  }

  if (ble_ae_advertising) {
    status = ble_ae_stop_advertising();
    if (status != RSI_SUCCESS) {
      DEBUGOUT("[BLE] stop AE advertising failed status=0x%lX\n",
               (unsigned long)status);
      return status;
    }
  }

  adv_len = ble_build_elrs_advertisement(adv);
  adv_name = "ELRS";

  if (adv_len == 0U) {
    return -1;
  }

  if (ble_advertising) {
    (void)rsi_ble_stop_advertising();
    ble_advertising = false;
  }

  status = rsi_ble_set_advertise_data(adv, adv_len);
  if (status != RSI_SUCCESS) {
    DEBUGOUT("[BLE] set %s advertise data failed len=%u status=0x%lX\n",
             adv_name,
             (unsigned int)adv_len,
             (unsigned long)status);
    return status;
  } else {
    BLE_LOG_DEBUGOUT("[BLE] set %s advertise data len=%u status=0x%lX\n",
                     adv_name,
                     (unsigned int)adv_len,
                     (unsigned long)status);
  }

  if (!is_connected && ble_should_advertise_current_mode()) {
    status = ble_start_current_advertising();
    if (status == RSI_SUCCESS) {
      ble_advertising = true;
      BLE_LOG_DEBUGOUT("[BLE] start %s advertising status=0x%lX\n",
                       adv_name,
                       (unsigned long)status);
    } else {
      DEBUGOUT("[BLE] start %s advertising failed status=0x%lX\n",
               adv_name,
               (unsigned long)status);
    }
  }

  return status;
}

static uint8_t ble_build_remote_id_ae_payload(uint8_t *payload,
                                              uint8_t max_len,
                                              ble_remote_id_adv_kind_t kind)
{
  uint8_t len = 0U;
  uint8_t legacy_adv[BLE_REMOTE_ID_ADV_MAX] = { 0 };
  uint16_t legacy_len;
  const char name[] = "ELRS-RID";
  const uint8_t name_len = (uint8_t)(sizeof(name) - 1U);

  if (payload == NULL || max_len == 0U) {
    return 0U;
  }

  legacy_len = ble_remote_id_build_advertisement(legacy_adv, kind);
  if (legacy_len == 0U) {
    return 0U;
  }

  if ((uint16_t)(legacy_len + 2U + name_len) > max_len) {
    return 0U;
  }

  /*
   * Keep the ODID service-data element byte-for-byte identical to the legacy
   * advertisement Drone Scanner already parsed correctly. The appended Complete
   * Local Name only proves the payload is travelling through the AE path.
   */
  memcpy(&payload[len], legacy_adv, legacy_len);
  len = (uint8_t)(len + legacy_len);

  payload[len++] = (uint8_t)(1U + name_len);
  payload[len++] = 0x09U;
  memcpy(&payload[len], name, name_len);
  len = (uint8_t)(len + name_len);

  return len;
}

static int32_t ble_apply_remote_id_ae_advertisement(ble_remote_id_adv_kind_t kind)
{
  uint8_t payload[BLE_REMOTE_ID_AE_PAYLOAD_MAX] = { 0 };
  uint8_t payload_len = ble_build_remote_id_ae_payload(payload, sizeof(payload), kind);
  const char *adv_name = ble_remote_id_adv_kind_name(kind);
  int32_t status = RSI_SUCCESS;

  if (payload_len == 0U || payload_len > sizeof(((rsi_ble_ae_data_t *)0)->data)) {
    return -1;
  }

  if (ble_advertising) {
    status = rsi_ble_stop_advertising();
    if (status != RSI_SUCCESS) {
      DEBUGOUT("[BLE] stop legacy advertising failed before AE status=0x%lX\n",
               (unsigned long)status);
      return status;
    }
    ble_advertising = false;
  }

  if (ble_ae_advertising) {
    status = ble_ae_stop_advertising();
    if (status != RSI_SUCCESS) {
      DEBUGOUT("[BLE] stop AE advertising failed status=0x%lX\n",
               (unsigned long)status);
      return status;
    }
  }

  if (!ble_ae_params_configured) {
    int8_t selected_tx_power = 0;
    rsi_ble_ae_adv_params_t params = { 0 };
    params.adv_handle = BLE_AE_ADV_HANDLE;
    params.adv_event_prop = 0x0000U;
    params.primary_adv_intterval_min = BLE_AE_ADV_INTERVAL_1S;
    params.primary_adv_intterval_max = BLE_AE_ADV_INTERVAL_1S;
    params.primary_adv_chnl_map = RSI_BLE_ADV_CHANNEL_MAP;
    params.own_addr_type = LE_PUBLIC_ADDRESS;
    params.peer_addr_type = LE_PUBLIC_ADDRESS;
    params.adv_filter_policy = ALLOW_SCAN_REQ_ANY_CONN_REQ_ANY;
    params.adv_tx_power = 0x7FU;
    params.primary_adv_phy = 0x01U;
    params.sec_adv_max_skip = 0x00U;
    params.sec_adv_phy = 0x01U;
    params.adv_sid = 0x00U;
    params.scan_req_notify_enable = 0x00U;

    status = rsi_ble_set_ae_params(&params, &selected_tx_power);
    if (status != RSI_SUCCESS) {
      DEBUGOUT("[BLE] set Remote ID AE params failed status=0x%lX\n",
               (unsigned long)status);
      return status;
    }
    ble_ae_params_configured = true;
    BLE_LOG_DEBUGOUT("[BLE] Remote ID AE params ready selected_tx_power=%d\n",
                     (int)selected_tx_power);
  }

  rsi_ble_ae_data_t data = { 0 };
  data.type = AE_ADV_DATA;
  data.adv_handle = BLE_AE_ADV_HANDLE;
  data.operation = 0x03U;
  data.frag_pref = 0x00U;
  data.data_len = payload_len;
  memcpy(data.data, payload, payload_len);

  status = rsi_ble_set_ae_data(&data);
  if (status != RSI_SUCCESS) {
    DEBUGOUT("[BLE] set %s AE data failed len=%u status=0x%lX\n",
             adv_name,
             (unsigned int)payload_len,
             (unsigned long)status);
    return status;
  }

  if (!is_connected && ble_should_advertise_current_mode()) {
    rsi_ble_ae_adv_enable_t enable = { 0 };
    enable.enable = RSI_BLE_START_ADV;
    enable.no_of_sets = 1U;
    enable.adv_handle = BLE_AE_ADV_HANDLE;
    enable.duration = 0U;
    enable.max_ae_events = 0U;

    status = rsi_ble_start_ae_advertising(&enable);
    if (status == RSI_SUCCESS) {
      ble_ae_advertising = true;
      BLE_LOG_DEBUGOUT("[BLE] start %s AE advertising len=%u status=0x%lX\n",
                       adv_name,
                       (unsigned int)payload_len,
                       (unsigned long)status);
    } else {
      DEBUGOUT("[BLE] start %s AE advertising failed status=0x%lX\n",
               adv_name,
               (unsigned long)status);
    }
  }

  return status;
}

static uint8_t ble_build_ae_test_payload(uint8_t *payload, uint8_t max_len)
{
  uint8_t len = 0U;
  const char name[] = "ELRS-AE-TEST";

  if (payload == NULL || max_len < 48U) {
    return 0U;
  }

  payload[len++] = 0x02U;
  payload[len++] = 0x01U;
  payload[len++] = 0x06U;

  payload[len++] = (uint8_t)(1U + sizeof(name) - 1U);
  payload[len++] = 0x08U;
  memcpy(&payload[len], name, sizeof(name) - 1U);
  len = (uint8_t)(len + sizeof(name) - 1U);

  /*
   * OpenDroneID service UUID with dummy bytes. This deliberately exceeds the
   * 31-byte legacy advertising limit without pretending to be valid RID data.
   */
  const uint8_t service_data_len = 44U;
  payload[len++] = (uint8_t)(1U + 2U + service_data_len);
  payload[len++] = 0x16U;
  payload[len++] = 0xFAU;
  payload[len++] = 0xFFU;
  for (uint8_t i = 0; i < service_data_len && len < max_len; i++) {
    payload[len++] = (uint8_t)(0xA0U + i);
  }

  return len;
}

static int32_t ble_ae_stop_advertising(void)
{
  rsi_ble_ae_adv_enable_t enable = { 0 };
  enable.enable = RSI_BLE_STOP_ADV;
  enable.no_of_sets = 1U;
  enable.adv_handle = BLE_AE_ADV_HANDLE;
  enable.duration = 0U;
  enable.max_ae_events = 0U;

  int32_t status = rsi_ble_start_ae_advertising(&enable);
  if (status == RSI_SUCCESS) {
    ble_ae_advertising = false;
  }
  return status;
}

static void ble_handle_ae_command(char *args)
{
  char response[128];
  args = trim_ascii(args);

  if (*args == '\0' || ascii_equal_ignore_case(args, "help")) {
    ble_publish_ephemeral_response("ae: caps,test,stop,status");
    return;
  }

  if (ascii_equal_ignore_case(args, "status")) {
    snprintf(response,
             sizeof(response),
             "ae advertising=%u handle=%u",
             ble_ae_advertising ? 1U : 0U,
             (unsigned int)BLE_AE_ADV_HANDLE);
    ble_publish_ephemeral_response(response);
    return;
  }

  if (ascii_equal_ignore_case(args, "caps")) {
    uint8_t max_len = 0U;
    uint8_t max_sets = 0U;
    int32_t len_status = rsi_ble_get_max_adv_data_len(&max_len);
    int32_t sets_status = rsi_ble_get_max_no_of_supp_adv_sets(&max_sets);
    snprintf(response,
             sizeof(response),
             "ae caps len_st=0x%lX max_len=%u sets_st=0x%lX max_sets=%u",
             (unsigned long)len_status,
             (unsigned int)max_len,
             (unsigned long)sets_status,
             (unsigned int)max_sets);
    ble_publish_ephemeral_response(response);
    return;
  }

  if (ascii_equal_ignore_case(args, "stop")) {
    int32_t status = ble_ae_stop_advertising();
    snprintf(response, sizeof(response), "ae stop status=0x%lX", (unsigned long)status);
    ble_publish_ephemeral_response(response);
    return;
  }

  if (ascii_equal_ignore_case(args, "test")) {
    uint8_t payload[BLE_AE_TEST_PAYLOAD_MAX] = { 0 };
    uint8_t payload_len = ble_build_ae_test_payload(payload, sizeof(payload));
    int8_t selected_tx_power = 0;
    int32_t status;

    if (payload_len == 0U || payload_len > sizeof(((rsi_ble_ae_data_t *)0)->data)) {
      ble_publish_ephemeral_response("ae test payload build failed");
      return;
    }

    if (ble_ae_advertising) {
      (void)ble_ae_stop_advertising();
    }

    rsi_ble_ae_adv_params_t params = { 0 };
    params.adv_handle = BLE_AE_ADV_HANDLE;
    params.adv_event_prop = 0x0000U;
    params.primary_adv_intterval_min = BLE_AE_ADV_INTERVAL_1S;
    params.primary_adv_intterval_max = BLE_AE_ADV_INTERVAL_1S;
    params.primary_adv_chnl_map = RSI_BLE_ADV_CHANNEL_MAP;
    params.own_addr_type = LE_PUBLIC_ADDRESS;
    params.peer_addr_type = LE_PUBLIC_ADDRESS;
    params.adv_filter_policy = ALLOW_SCAN_REQ_ANY_CONN_REQ_ANY;
    params.adv_tx_power = 0x7FU;
    params.primary_adv_phy = 0x01U;
    params.sec_adv_max_skip = 0x00U;
    params.sec_adv_phy = 0x01U;
    params.adv_sid = 0x00U;
    params.scan_req_notify_enable = 0x00U;

    status = rsi_ble_set_ae_params(&params, &selected_tx_power);
    if (status != RSI_SUCCESS) {
      snprintf(response, sizeof(response), "ae params failed status=0x%lX", (unsigned long)status);
      ble_publish_ephemeral_response(response);
      return;
    }

    rsi_ble_ae_data_t data = { 0 };
    data.type = AE_ADV_DATA;
    data.adv_handle = BLE_AE_ADV_HANDLE;
    data.operation = 0x03U;
    data.frag_pref = 0x00U;
    data.data_len = payload_len;
    memcpy(data.data, payload, payload_len);

    status = rsi_ble_set_ae_data(&data);
    if (status != RSI_SUCCESS) {
      snprintf(response, sizeof(response), "ae data failed len=%u status=0x%lX",
               (unsigned int)payload_len,
               (unsigned long)status);
      ble_publish_ephemeral_response(response);
      return;
    }

    rsi_ble_ae_adv_enable_t enable = { 0 };
    enable.enable = RSI_BLE_START_ADV;
    enable.no_of_sets = 1U;
    enable.adv_handle = BLE_AE_ADV_HANDLE;
    enable.duration = 0U;
    enable.max_ae_events = 0U;

    status = rsi_ble_start_ae_advertising(&enable);
    if (status == RSI_SUCCESS) {
      ble_ae_advertising = true;
    }
    snprintf(response,
             sizeof(response),
             "ae test len=%u start=0x%lX tx=%d",
             (unsigned int)payload_len,
             (unsigned long)status,
             (int)selected_tx_power);
    ble_publish_ephemeral_response(response);
    return;
  }

  ble_publish_ephemeral_response("err ae command; write ae help");
}

static void ble_restore_elrs_advertisement_if_due(void)
{
  if (ble_adv_mode != BLE_ADV_MODE_REMOTE_ID_BASIC ||
      ble_remote_id_restore_tick == 0U || is_connected) {
    return;
  }

  if (ble_tick_reached(osKernelGetTickCount(), ble_remote_id_restore_tick)) {
    BLE_LOG_DEBUGOUT("[BLE] Remote ID advertising timeout; restoring ELRS config advertisement\n");
    ble_adv_mode = BLE_ADV_MODE_ELRS_CONFIG;
    ble_remote_id_restore_tick = 0U;
    (void)ble_apply_current_advertisement();
  }
}

static void ble_update_remote_id_dynamic_advertisement_if_due(void)
{
  if (ble_adv_mode != BLE_ADV_MODE_REMOTE_ID_BASIC || is_connected) {
    ble_remote_id_dynamic_tick = 0U;
    return;
  }

  const uint32_t now = osKernelGetTickCount();
  if (ble_remote_id_dynamic_tick != 0U &&
      !ble_tick_reached(now, ble_remote_id_dynamic_tick)) {
    return;
  }

  ble_remote_id_dynamic_tick =
      now + ble_seconds_to_ticks(BLE_REMOTE_ID_DYNAMIC_UPDATE_SECONDS);

  if (!ble_remote_id_location_is_fresh(ble_ticks_to_ms(now),
                                       BLE_REMOTE_ID_LOCATION_STALE_MS)) {
    if (ble_remote_id_next_adv_kind != BLE_REMOTE_ID_ADV_BASIC_ID) {
      ble_remote_id_next_adv_kind = BLE_REMOTE_ID_ADV_BASIC_ID;
      (void)ble_apply_current_advertisement();
    }
    return;
  }

  ble_remote_id_next_adv_kind = BLE_REMOTE_ID_ADV_LOCATION;
  (void)ble_apply_current_advertisement();
}

static void ble_configure_remote_id_mode(bool enabled)
{
  ble_adv_mode = enabled ? BLE_ADV_MODE_REMOTE_ID_BASIC : BLE_ADV_MODE_ELRS_CONFIG;
  ble_remote_id_restore_tick = 0U;
  ble_remote_id_dynamic_tick = 0U;
  ble_remote_id_next_adv_kind = BLE_REMOTE_ID_ADV_BASIC_ID;
}

static void ble_apply_remote_id_request_if_pending(void)
{
  if (!ble_remote_id_request_pending) {
    return;
  }

  bool enabled = ble_remote_id_requested_enabled;
  ble_remote_id_request_pending = false;
  ble_configure_remote_id_mode(enabled);

  BLE_LOG_DEBUGOUT("[BLE] Remote ID runtime request: %s\n", enabled ? "enabled" : "disabled");
  if (status_handle == 0U) {
    return;
  }

  if (is_connected) {
    BLE_LOG_DEBUGOUT("[BLE] Remote ID advertising change queued until BLE disconnect\n");
    return;
  }

  int32_t status = ble_apply_current_advertisement();
  if (status != RSI_SUCCESS) {
    DEBUGOUT("[BLE] Remote ID advertising apply failed status=0x%lX\n",
             (unsigned long)status);
  } else {
    DEBUGOUT("[BLE] Remote ID advertising %s\n", enabled ? "enabled" : "disabled");
    BLE_LOG_DEBUGOUT("[BLE] Remote ID advertising apply status=0x%lX\n",
                     (unsigned long)status);
  }
}

static int32_t ble_notify_chunk(uint16_t handle,
                                const char *reason,
                                uint16_t offset,
                                const uint8_t *data,
                                uint16_t len)
{
  int32_t status = RSI_SUCCESS;

#if !BLE_LOG_VERBOSE
  (void)reason;
  (void)offset;
#endif

  for (uint32_t attempt = 0; attempt < BLE_NOTIFY_MAX_RETRIES; attempt++) {
    status = rsi_ble_notify_value(ble_conn.dev_addr, handle, len, data);
    BLE_LOG_DEBUGOUT("[BLE] notify %s handle=0x%04X offset=%u len=%u status=0x%lX\n",
                     reason,
                     (unsigned int)handle,
                     (unsigned int)offset,
                     (unsigned int)len,
                     (unsigned long)status);

    if (status != BLE_STATUS_DEV_BUF_FULL) {
      if (status == RSI_SUCCESS) {
        notify_count++;
      }
      return status;
    }

    osDelay(BLE_NOTIFY_RETRY_DELAY_MS);
  }

  return status;
}

static void ble_notify_response_on_handle(const char *reason, uint16_t handle)
{
  if (!is_connected || handle == 0) {
    return;
  }

  uint16_t chunk_len = status_value_len;
  if (chunk_len > BLE_NOTIFY_CHUNK_MAX) {
    chunk_len = BLE_NOTIFY_CHUNK_MAX;
  }

  int32_t status = ble_notify_chunk(handle,
                                    reason,
                                    0,
                                    status_value,
                                    chunk_len);
  if (status != RSI_SUCCESS) {
    notify_error_count++;
    DEBUGOUT("[BLE] notify %s failed status=0x%lX\n",
             reason,
             (unsigned long)status);
  }
}

static void ble_notify_response(const char *reason)
{
  if (status_notifications_enabled) {
    ble_notify_response_on_handle(reason, status_handle);
  }
  if (command_notifications_enabled) {
    ble_notify_response_on_handle(reason, command_handle);
  }
}

static const sl_wifi_device_configuration_t ble_wifi_config = {
    .boot_option = LOAD_NWP_FW,
    .mac_address = NULL,
    .band = SL_SI91X_WIFI_BAND_2_4GHZ,
    .region_code = US,
    .boot_config = {
        .oper_mode = SL_SI91X_ACCESS_POINT_MODE,
        .coex_mode = SL_SI91X_WLAN_BLE_MODE,
        .feature_bit_map = SL_WIFI_FEAT_SECURITY_OPEN,
        .tcp_ip_feature_bit_map =
            (SL_SI91X_TCP_IP_FEAT_DHCPV4_SERVER |
             SL_SI91X_TCP_IP_FEAT_EXTENSION_VALID),
        .custom_feature_bit_map = SL_SI91X_CUSTOM_FEAT_EXTENTION_VALID,
        .ext_custom_feature_bit_map =
            (SL_SI91X_EXT_FEAT_LOW_POWER_MODE | SL_SI91X_EXT_FEAT_XTAL_CLK |
             MEMORY_CONFIG |
             SL_SI91X_EXT_FEAT_FRONT_END_SWITCH_PINS_ULP_GPIO_4_5_0 |
             SL_SI91X_EXT_FEAT_BT_CUSTOM_FEAT_ENABLE),
        .ext_tcp_ip_feature_bit_map = SL_SI91X_CONFIG_FEAT_EXTENTION_VALID,
        .bt_feature_bit_map =
            (SL_SI91X_BT_RF_TYPE | SL_SI91X_ENABLE_BLE_PROTOCOL),
        .ble_feature_bit_map =
            ((SL_SI91X_BLE_MAX_NBR_PERIPHERALS(RSI_BLE_MAX_NBR_PERIPHERALS) |
              SL_SI91X_BLE_MAX_NBR_CENTRALS(RSI_BLE_MAX_NBR_CENTRALS) |
              SL_SI91X_BLE_MAX_NBR_ATT_SERV(RSI_BLE_MAX_NBR_ATT_SERV) |
              SL_SI91X_BLE_MAX_NBR_ATT_REC(RSI_BLE_MAX_NBR_ATT_REC)) |
             SL_SI91X_FEAT_BLE_CUSTOM_FEAT_EXTENTION_VALID |
             SL_SI91X_BLE_PWR_INX(RSI_BLE_PWR_INX) |
             SL_SI91X_BLE_PWR_SAVE_OPTIONS(RSI_BLE_PWR_SAVE_OPTIONS) |
             SL_SI91X_916_BLE_COMPATIBLE_FEAT_ENABLE
#if RSI_BLE_GATT_ASYNC_ENABLE
             | SL_SI91X_BLE_GATT_ASYNC_ENABLE
#endif
             ),
        .ble_ext_feature_bit_map =
            (SL_SI91X_BLE_NUM_CONN_EVENTS(RSI_BLE_NUM_CONN_EVENTS) |
             SL_SI91X_BLE_NUM_REC_BYTES(RSI_BLE_NUM_REC_BYTES)
#if RSI_BLE_INDICATE_CONFIRMATION_FROM_HOST
             | SL_SI91X_BLE_INDICATE_CONFIRMATION_FROM_HOST
#endif
#if RSI_BLE_MTU_EXCHANGE_FROM_HOST
             | SL_SI91X_BLE_MTU_EXCHANGE_FROM_HOST
#endif
#if RSI_BLE_SET_SCAN_RESP_DATA_FROM_HOST
             | SL_SI91X_BLE_SET_SCAN_RESP_DATA_FROM_HOST
#endif
#if RSI_BLE_DISABLE_CODED_PHY_FROM_HOST
             | SL_SI91X_BLE_DISABLE_CODED_PHY_FROM_HOST
#endif
#if BLE_SIMPLE_GATT
             | SL_SI91X_BLE_GATT_INIT
#endif
#if RSI_BLE_ENABLE_ADV_EXTN
             | SL_SI91X_BLE_ENABLE_ADV_EXTN
#endif
#if RSI_BLE_AE_MAX_ADV_SETS
             | SL_SI91X_BLE_AE_MAX_ADV_SETS(RSI_BLE_AE_MAX_ADV_SETS)
#endif
             ),
        .config_feature_bit_map = SL_SI91X_FEAT_SLEEP_GPIO_SEL_BITMAP,
    },
};

static void log_boot_config(void)
{
  const sl_wifi_system_boot_configuration_t *cfg = &ble_wifi_config.boot_config;

#if !BLE_LOG_VERBOSE
  (void)cfg;
#endif

  BLE_LOG_DEBUGOUT("[BLE] NWP cfg boot=%u oper=%lu coex=%lu band=%u region=%u\n",
                   (unsigned int)ble_wifi_config.boot_option,
                   (unsigned long)cfg->oper_mode,
                   (unsigned long)cfg->coex_mode,
                   (unsigned int)ble_wifi_config.band,
                   (unsigned int)ble_wifi_config.region_code);
  BLE_LOG_DEBUGOUT("[BLE] NWP cfg feat=0x%08lX tcp=0x%08lX custom=0x%08lX ext=0x%08lX\n",
                   (unsigned long)cfg->feature_bit_map,
                   (unsigned long)cfg->tcp_ip_feature_bit_map,
                   (unsigned long)cfg->custom_feature_bit_map,
                   (unsigned long)cfg->ext_custom_feature_bit_map);
  BLE_LOG_DEBUGOUT("[BLE] NWP cfg bt=0x%08lX ext_tcp=0x%08lX ble=0x%08lX ble_ext=0x%08lX cfg=0x%08lX\n",
                   (unsigned long)cfg->bt_feature_bit_map,
                   (unsigned long)cfg->ext_tcp_ip_feature_bit_map,
                   (unsigned long)cfg->ble_feature_bit_map,
                   (unsigned long)cfg->ble_ext_feature_bit_map,
                   (unsigned long)cfg->config_feature_bit_map);
}

static void ble_task_exit(void)
{
  ble_service_ready = false;
  ble_advertising = false;
  ble_task_started = false;
  osThreadExit();
}

uint32_t ble_gatt_config_api_prepare_nwp(void)
{
  if (device_initialized) {
    BLE_LOG_DEBUGOUT("[BLE] NWP already initialized before BLE-capable prepare\n");
    return SL_STATUS_OK;
  }

  BLE_LOG_DEBUGOUT("[BLE] Preparing NWP for WiFi AP + BLE config API...\n");
  log_boot_config();
  sl_status_t status = sl_wifi_init(&ble_wifi_config, NULL, sl_wifi_default_event_handler);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("[BLE] prepare sl_wifi_init failed status=0x%lX\n", (unsigned long)status);
  } else {
    BLE_LOG_DEBUGOUT("[BLE] prepare sl_wifi_init status=0x%lX\n", (unsigned long)status);
  }
  return (uint32_t)status;
}

static void ble_set_event(uint32_t event)
{
  ble_event_map |= (1UL << event);
  if (ble_sem != NULL) {
    osSemaphoreRelease(ble_sem);
  }
}

void ble_remote_id_service_set_enabled(bool enabled)
{
  ble_remote_id_requested_enabled = enabled;
  ble_remote_id_request_pending = true;
  if (ble_sem != NULL) {
    osSemaphoreRelease(ble_sem);
  }
}

bool ble_remote_id_service_is_enabled(void)
{
  return ble_remote_id_requested_enabled;
}

static int32_t ble_get_event(void)
{
  for (uint32_t event = 0; event < 32; event++) {
    if (ble_event_map & (1UL << event)) {
      return (int32_t)event;
    }
  }
  return -1;
}

static void ble_clear_event(uint32_t event)
{
  ble_event_map &= ~(1UL << event);
}

static void ble_note_peer(const uint8_t *dev_addr)
{
  memcpy(ble_conn.dev_addr, dev_addr, RSI_DEV_ADDR_LEN);
  is_connected = true;
}

static bool ascii_equal_ignore_case(const char *a, const char *b)
{
  while (*a != '\0' && *b != '\0') {
    char ca = *a++;
    char cb = *b++;

    if (ca >= 'A' && ca <= 'Z') {
      ca = (char)(ca - 'A' + 'a');
    }
    if (cb >= 'A' && cb <= 'Z') {
      cb = (char)(cb - 'A' + 'a');
    }
    if (ca != cb) {
      return false;
    }
  }

  return *a == '\0' && *b == '\0';
}

static bool ascii_starts_with_ignore_case(const char *text, const char *prefix)
{
  while (*prefix != '\0') {
    char ca = *text++;
    char cb = *prefix++;

    if (ca >= 'A' && ca <= 'Z') {
      ca = (char)(ca - 'A' + 'a');
    }
    if (cb >= 'A' && cb <= 'Z') {
      cb = (char)(cb - 'A' + 'a');
    }
    if (ca != cb) {
      return false;
    }
  }

  return true;
}

static char *trim_ascii(char *text)
{
  while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') {
    text++;
  }

  char *end = text + strlen(text);
  while (end > text) {
    char c = end[-1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
      break;
    }
    *--end = '\0';
  }

  return text;
}

static bool parse_u8_range(const char *text,
                           unsigned long min_value,
                           unsigned long max_value,
                           uint8_t *out)
{
  char *end = NULL;
  unsigned long value = strtoul(text, &end, 10);
  end = trim_ascii(end);
  if (end == text || *end != '\0' || value < min_value || value > max_value) {
    return false;
  }

  *out = (uint8_t)value;
  return true;
}

static bool parse_u16_range(const char *text,
                            unsigned long min_value,
                            unsigned long max_value,
                            uint16_t *out)
{
  char *end = NULL;
  unsigned long value = strtoul(text, &end, 10);
  end = trim_ascii(end);
  if (end == text || *end != '\0' || value < min_value || value > max_value) {
    return false;
  }

  *out = (uint16_t)value;
  return true;
}

static bool parse_u32_range(const char *text,
                            unsigned long min_value,
                            unsigned long max_value,
                            uint32_t *out)
{
  char *end = NULL;
  unsigned long value = strtoul(text, &end, 10);
  end = trim_ascii(end);
  if (end == text || *end != '\0' || value < min_value || value > max_value) {
    return false;
  }

  *out = (uint32_t)value;
  return true;
}

static bool parse_i32_range(const char *text,
                            long min_value,
                            long max_value,
                            int32_t *out)
{
  char *end = NULL;
  long value = strtol(text, &end, 10);
  end = trim_ascii(end);
  if (end == text || *end != '\0' || value < min_value || value > max_value) {
    return false;
  }

  *out = (int32_t)value;
  return true;
}

static char *skip_arg_separator(char *text)
{
  while (*text == ' ' || *text == '\t' || *text == ',') {
    text++;
  }
  return text;
}

static bool parse_double_arg(char **cursor, double min_value, double max_value, double *out)
{
  char *text = skip_arg_separator(*cursor);
  char *end = NULL;
  double value = strtod(text, &end);

  if (end == text || value < min_value || value > max_value) {
    return false;
  }

  *cursor = end;
  *out = value;
  return true;
}

static bool double_to_e7(double value, int32_t *out)
{
  double scaled = value * 10000000.0;
  if (scaled < -2147483648.0 || scaled > 2147483647.0) {
    return false;
  }

  *out = (int32_t)(scaled + ((scaled >= 0.0) ? 0.5 : -0.5));
  return true;
}

static void ble_note_last_command(const char *command)
{
  memset(last_command_text, 0, sizeof(last_command_text));
  strncpy(last_command_text, command, sizeof(last_command_text) - 1U);
}

static void ble_publish_response_internal(const char *response, bool remember_payload)
{
  size_t len = strlen(response);
  size_t write_len = len;
  int32_t local_status = RSI_SUCCESS;
  response_seq++;
  if (len > sizeof(status_value)) {
    len = sizeof(status_value);
  }
  if (write_len > sizeof(status_value)) {
    write_len = sizeof(status_value);
  }
  if (write_len < status_write_len) {
    write_len = status_write_len;
  }

  memset(status_value, ' ', sizeof(status_value));
  memcpy(status_value, response, len);
  status_value_len = (uint16_t)len;
  status_write_len = (uint16_t)write_len;
  if (remember_payload) {
    memcpy(payload_value, status_value, sizeof(payload_value));
    payload_value_len = status_value_len;
  }

  BLE_LOG_DEBUGOUT("[BLE] Response len=%u text='%.*s'\n",
                   (unsigned int)status_value_len,
                   (int)RSI_MIN(status_value_len, 80U),
                   (const char *)status_value);

  if (status_handle != 0) {
    local_status = rsi_ble_set_local_att_value(status_handle,
                                               BLE_NOTIFY_CHUNK_MAX,
                                               status_value);
    if (local_status != RSI_SUCCESS) {
      DEBUGOUT("[BLE] Local response update failed handle=0x%04X status=0x%lX\n",
               (unsigned int)status_handle,
               (unsigned long)local_status);
    } else {
      BLE_LOG_DEBUGOUT("[BLE] Local response update handle=0x%04X len=%u status=0x%lX\n",
                       (unsigned int)status_handle,
                       (unsigned int)BLE_NOTIFY_CHUNK_MAX,
                       (unsigned long)local_status);
    }
    if (command_handle != 0) {
      local_status = rsi_ble_set_local_att_value(command_handle,
                                                 BLE_NOTIFY_CHUNK_MAX,
                                                 status_value);
      if (local_status != RSI_SUCCESS) {
        DEBUGOUT("[BLE] Local command-read update failed handle=0x%04X status=0x%lX\n",
                 (unsigned int)command_handle,
                 (unsigned long)local_status);
      } else {
        BLE_LOG_DEBUGOUT("[BLE] Local command-read update handle=0x%04X len=%u status=0x%lX\n",
                         (unsigned int)command_handle,
                         (unsigned int)BLE_NOTIFY_CHUNK_MAX,
                         (unsigned long)local_status);
      }
    }
    ble_notify_response("response");
  }
}

static void ble_publish_response(const char *response)
{
  ble_publish_response_internal(response, true);
}

static void ble_publish_helper_response(const char *response)
{
  ble_publish_response_internal(response, false);
}

static void ble_publish_ephemeral_response(const char *response)
{
  /* Legacy name: command replies must be remembered so len/page works. */
  ble_publish_response(response);
}

static void ble_publish_payload_chunk(uint16_t offset)
{
  char response[BLE_NOTIFY_CHUNK_MAX + 1];
  uint16_t chunk_len = 0;

  if (offset >= payload_value_len) {
    ble_publish_helper_response("err chunk offset");
    return;
  }

  chunk_len = (uint16_t)(payload_value_len - offset);
  if (chunk_len > BLE_NOTIFY_CHUNK_MAX) {
    chunk_len = BLE_NOTIFY_CHUNK_MAX;
  }

  memset(response, 0, sizeof(response));
  memcpy(response, &payload_value[offset], chunk_len);
  ble_publish_helper_response(response);
}

static void ble_mark_dirty(void)
{
  config_dirty = true;
}

static void ble_json_escape(char *out, size_t out_size, const char *in)
{
  size_t written = 0;

  if (out_size == 0) {
    return;
  }

  while (*in != '\0' && written + 1U < out_size) {
    char c = *in++;
    if ((c == '"' || c == '\\') && written + 2U < out_size) {
      out[written++] = '\\';
      out[written++] = c;
    } else if ((uint8_t)c < 0x20U) {
      out[written++] = ' ';
    } else {
      out[written++] = c;
    }
  }

  out[written] = '\0';
}

static void ble_publish_json_error(const char *code, const char *message)
{
  char response[BLE_VALUE_MAX];
  snprintf(response,
           sizeof(response),
           "{\"ok\":false,\"err\":\"%s\",\"msg\":\"%s\"}",
           code,
           message);
  ble_publish_ephemeral_response(response);
}

static void ble_format_config_response(void)
{
  const elrs_config_t *cfg = elrs_config_get();
  char response[BLE_VALUE_MAX];
  const char *power = NULL;
  char power_buf[8];

  if (cfg->tx_power == ELRS_TX_POWER_MATCH_TX_DBM) {
    power = "match";
  } else {
    snprintf(power_buf, sizeof(power_buf), "%d", (int)cfg->tx_power);
    power = power_buf;
  }

  snprintf(response,
           sizeof(response),
           "{\"v\":%u,\"flags\":%u,\"uid\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
           "\"serial\":%u,\"failsafe\":%u,\"rate\":%u,\"model\":%u,"
           "\"tlmOff\":%u,\"tlmInt\":%u,\"power\":\"%s\","
           "\"domL\":%u,\"domH\":%u,\"webDomain\":%u,\"wifiCh\":%u,"
           "\"wifiInt\":%ld,\"baud\":%lu,\"lockFirst\":%u,"
           "\"airport\":%u,\"djiArmed\":%u,"
           "\"webCustom\":%u,\"ssid\":\"%s\"}",
           (unsigned int)cfg->version,
           (unsigned int)cfg->flags,
           (unsigned int)cfg->uid[0],
           (unsigned int)cfg->uid[1],
           (unsigned int)cfg->uid[2],
           (unsigned int)cfg->uid[3],
           (unsigned int)cfg->uid[4],
           (unsigned int)cfg->uid[5],
           (unsigned int)cfg->serial_protocol,
           (unsigned int)cfg->failsafe_mode,
           (unsigned int)cfg->rate_index,
           (unsigned int)cfg->model_id,
           (unsigned int)cfg->force_tlm,
           (unsigned int)cfg->tlm_interval,
           power,
           (unsigned int)cfg->reg_domain_low,
           (unsigned int)cfg->reg_domain_high,
           (unsigned int)elrs_config_get_web_domain(),
           (unsigned int)cfg->wifi_channel,
           (long)elrs_config_get_wifi_on_interval(),
           (unsigned long)elrs_config_get_uart_baud(),
           elrs_config_get_lock_on_first_connection() ? 1U : 0U,
           elrs_config_get_is_airport() ? 1U : 0U,
           elrs_config_get_dji_permanently_armed() ? 1U : 0U,
           elrs_config_web_options_customised() ? 1U : 0U,
           cfg->wifi_ssid);

  ble_publish_response(response);
}

static void ble_format_json_config_response(void)
{
  const elrs_config_t *cfg = elrs_config_get();
  char response[BLE_VALUE_MAX];
  const char *power = NULL;
  char power_buf[8];

  if (cfg == NULL || !config_ready) {
    ble_publish_json_error("config", "config not ready");
    return;
  }

  if (cfg->tx_power == ELRS_TX_POWER_MATCH_TX_DBM) {
    power = "match";
  } else {
    snprintf(power_buf, sizeof(power_buf), "%d", (int)cfg->tx_power);
    power = power_buf;
  }

  snprintf(response,
           sizeof(response),
           "{\"ok\":true,\"v\":%u,\"flags\":%u,\"uid\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
           "\"serial\":%u,\"failsafe\":%u,\"rate\":%u,\"model\":%u,"
           "\"tlmOff\":%u,\"tlmInt\":%u,\"power\":\"%s\","
           "\"domL\":%u,\"domH\":%u,\"webDomain\":%u,\"wifiCh\":%u,"
           "\"wifiInt\":%ld,\"baud\":%lu,\"wifiCustom\":%u,"
           "\"lockFirst\":%u,\"airport\":%u,\"djiArmed\":%u,"
           "\"bleRemoteId\":%u,"
           "\"webCustom\":%u,\"dirty\":%u}",
           (unsigned int)cfg->version,
           (unsigned int)cfg->flags,
           (unsigned int)cfg->uid[0],
           (unsigned int)cfg->uid[1],
           (unsigned int)cfg->uid[2],
           (unsigned int)cfg->uid[3],
           (unsigned int)cfg->uid[4],
           (unsigned int)cfg->uid[5],
           (unsigned int)cfg->serial_protocol,
           (unsigned int)cfg->failsafe_mode,
           (unsigned int)cfg->rate_index,
           (unsigned int)cfg->model_id,
           (unsigned int)cfg->force_tlm,
           (unsigned int)cfg->tlm_interval,
           power,
           (unsigned int)cfg->reg_domain_low,
           (unsigned int)cfg->reg_domain_high,
           (unsigned int)elrs_config_get_web_domain(),
           (unsigned int)cfg->wifi_channel,
           (long)elrs_config_get_wifi_on_interval(),
           (unsigned long)elrs_config_get_uart_baud(),
           (cfg->flags & ELRS_CONFIG_FLAG_WIFI_CUSTOM) ? 1U : 0U,
           elrs_config_get_lock_on_first_connection() ? 1U : 0U,
           elrs_config_get_is_airport() ? 1U : 0U,
           elrs_config_get_dji_permanently_armed() ? 1U : 0U,
           elrs_config_get_ble_remote_id() ? 1U : 0U,
           elrs_config_web_options_customised() ? 1U : 0U,
           config_dirty ? 1U : 0U);

  ble_publish_response(response);
}

static void ble_format_keys_response(void)
{
  ble_publish_response("keys=version,flags,uid,serial,failsafe,rate,model,tlm_off,tlm_interval,power,domL,domH,web_domain,wifi_interval,uart_baud,wifi_channel,wifi_ssid,wifi_custom,web_custom,lock_on_first,is_airport,dji_armed,ble_remote_id,mav_tgt,mav_src,team_ch,team_pos,bind,vbind,dirty");
}

static void ble_format_setkeys_response(void)
{
  ble_publish_response("setkeys=serial,failsafe,rate,model,tlm_off,tlm_interval,power,domL,domH,web_domain,wifi_interval,uart_baud,wifi_channel,wifi_ssid,wifi_password,wifi_custom,lock_on_first,is_airport,dji_armed,ble_remote_id,mav_tgt,mav_src,team_ch,team_pos,bind,vbind");
}

static void ble_format_ranges_response(void)
{
  ble_publish_response("ranges=serial:0,2,4,7;failsafe:0..2;rate:0..31;model:0..63,255;tlm_off:0..1;tlm_int:0..255;power:match,10,14,17,20;domL/H:0..7;web_domain:0..5;wifi_ch:1..13;wifi_int:-1..86400;baud:9600..2000000;bools:0..1;mav:1..255;team_ch:0..10;team_pos:0..7;bind:0..3;vbind:0..255;ble_remote_id:0..1");
}

static void ble_format_caps_response(void)
{
  ble_publish_response("{\"api\":\"elrs-ble-v1\",\"svc\":\"E7E0\",\"rsp\":\"E7E1\",\"cmd\":\"E7E2\",\"chunk\":20,\"max\":320,\"cmds\":[\"help\",\"status\",\"jstatus\",\"jget\",\"meta\",\"diag\",\"get\",\"keys\",\"setkeys\",\"ranges\",\"set\",\"save\",\"reload\",\"len\",\"chunk\",\"page\",\"rid\",\"ae\"]}");
}

static void ble_format_meta_response(void)
{
  char response[BLE_VALUE_MAX];
  snprintf(response,
           sizeof(response),
           "{\"api\":\"elrs-ble-v1\",\"name\":\"%s\",\"svc\":\"E7E0\","
           "\"rsp\":\"E7E1\",\"cmd\":\"E7E2\",\"rspH\":%u,\"cmdH\":%u,"
           "\"rspCccd\":%u,\"cmdCccd\":%u,\"max\":%u,\"page\":%u}",
           BLE_PROBE_NAME,
           (unsigned int)status_handle,
           (unsigned int)command_handle,
           (unsigned int)status_cccd_handle,
           (unsigned int)command_cccd_handle,
           (unsigned int)BLE_VALUE_MAX,
           (unsigned int)BLE_NOTIFY_CHUNK_MAX);
  ble_publish_response(response);
}

static void ble_format_diag_response(void)
{
  char response[BLE_VALUE_MAX];
  snprintf(response,
           sizeof(response),
           "diag seq=%lu conn=%lu disc=%lu wr=%lu rd=%lu ntf=%lu nerr=%lu "
           "lw=0x%04X lr=0x%04X sN=%u cN=%u last=%s",
           (unsigned long)response_seq,
           (unsigned long)connect_count,
           (unsigned long)disconnect_count,
           (unsigned long)write_count,
           (unsigned long)read_count,
           (unsigned long)notify_count,
           (unsigned long)notify_error_count,
           (unsigned int)last_write_handle,
           (unsigned int)last_read_handle,
           status_notifications_enabled ? 1U : 0U,
           command_notifications_enabled ? 1U : 0U,
           last_command_text[0] != '\0' ? last_command_text : "-");
  ble_publish_response(response);
}

static void ble_format_json_status_response(void)
{
  char response[BLE_VALUE_MAX];
  snprintf(response,
           sizeof(response),
           "{\"ok\":true,\"cfg\":%u,\"dirty\":%u,\"ntf\":%u,\"ntfS\":%u,"
           "\"ntfC\":%u,\"len\":%u,\"seq\":%lu,\"wr\":%lu,\"rd\":%lu}",
           config_ready ? 1U : 0U,
           config_dirty ? 1U : 0U,
           (status_notifications_enabled || command_notifications_enabled) ? 1U : 0U,
           status_notifications_enabled ? 1U : 0U,
           command_notifications_enabled ? 1U : 0U,
           (unsigned int)payload_value_len,
           (unsigned long)response_seq,
           (unsigned long)write_count,
           (unsigned long)read_count);
  ble_publish_response(response);
}

static void ble_format_key_response(const char *key)
{
  const elrs_config_t *cfg = elrs_config_get();
  char response[64];

  if (cfg == NULL || !config_ready) {
    ble_publish_ephemeral_response("err config not ready");
  } else if (ascii_equal_ignore_case(key, "version") ||
             ascii_equal_ignore_case(key, "v")) {
    snprintf(response, sizeof(response), "version=%u", (unsigned int)cfg->version);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "flags")) {
    snprintf(response, sizeof(response), "flags=0x%02X", (unsigned int)cfg->flags);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "uid")) {
    snprintf(response,
             sizeof(response),
             "uid=%02X%02X%02X%02X%02X%02X",
             (unsigned int)cfg->uid[0],
             (unsigned int)cfg->uid[1],
             (unsigned int)cfg->uid[2],
             (unsigned int)cfg->uid[3],
             (unsigned int)cfg->uid[4],
             (unsigned int)cfg->uid[5]);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "serial")) {
    snprintf(response, sizeof(response), "serial=%u", (unsigned int)cfg->serial_protocol);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "failsafe")) {
    snprintf(response, sizeof(response), "failsafe=%u", (unsigned int)cfg->failsafe_mode);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "rate")) {
    snprintf(response, sizeof(response), "rate=%u", (unsigned int)cfg->rate_index);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "model") ||
             ascii_equal_ignore_case(key, "model_id")) {
    snprintf(response, sizeof(response), "model=%u", (unsigned int)cfg->model_id);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "tlm_off") ||
             ascii_equal_ignore_case(key, "force_tlm")) {
    snprintf(response, sizeof(response), "tlm_off=%u", (unsigned int)cfg->force_tlm);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "tlm_interval") ||
             ascii_equal_ignore_case(key, "tlm_int")) {
    snprintf(response, sizeof(response), "tlm_interval=%u", (unsigned int)cfg->tlm_interval);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "power")) {
    if (cfg->tx_power == ELRS_TX_POWER_MATCH_TX_DBM) {
      ble_publish_ephemeral_response("power=match");
    } else {
      snprintf(response, sizeof(response), "power=%d", (int)cfg->tx_power);
      ble_publish_ephemeral_response(response);
    }
  } else if (ascii_equal_ignore_case(key, "domain_low") ||
             ascii_equal_ignore_case(key, "domL")) {
    snprintf(response, sizeof(response), "domL=%u", (unsigned int)cfg->reg_domain_low);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "domain_high") ||
             ascii_equal_ignore_case(key, "domH")) {
    snprintf(response, sizeof(response), "domH=%u", (unsigned int)cfg->reg_domain_high);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "web_domain")) {
    snprintf(response, sizeof(response), "web_domain=%u", (unsigned int)elrs_config_get_web_domain());
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "wifi_channel")) {
    snprintf(response, sizeof(response), "wifi_channel=%u", (unsigned int)cfg->wifi_channel);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "wifi_interval") ||
             ascii_equal_ignore_case(key, "wifi_int")) {
    snprintf(response, sizeof(response), "wifi_interval=%ld", (long)elrs_config_get_wifi_on_interval());
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "uart_baud") ||
             ascii_equal_ignore_case(key, "baud")) {
    snprintf(response, sizeof(response), "uart_baud=%lu", (unsigned long)elrs_config_get_uart_baud());
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "wifi_ssid")) {
    snprintf(response, sizeof(response), "ssid=%s", cfg->wifi_ssid);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "wifi_custom")) {
    snprintf(response,
             sizeof(response),
             "wifi_custom=%u",
             (cfg->flags & ELRS_CONFIG_FLAG_WIFI_CUSTOM) ? 1U : 0U);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "web_custom") ||
             ascii_equal_ignore_case(key, "web_customised") ||
             ascii_equal_ignore_case(key, "web_options_customised")) {
    snprintf(response,
             sizeof(response),
             "web_custom=%u",
             elrs_config_web_options_customised() ? 1U : 0U);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "lock_on_first") ||
             ascii_equal_ignore_case(key, "lock_first") ||
             ascii_equal_ignore_case(key, "lock")) {
    snprintf(response,
             sizeof(response),
             "lock_on_first=%u",
             elrs_config_get_lock_on_first_connection() ? 1U : 0U);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "is_airport") ||
             ascii_equal_ignore_case(key, "airport")) {
    snprintf(response,
             sizeof(response),
             "is_airport=%u",
             elrs_config_get_is_airport() ? 1U : 0U);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "dji_armed") ||
             ascii_equal_ignore_case(key, "dji") ||
             ascii_equal_ignore_case(key, "dji_permanently_armed")) {
    snprintf(response,
             sizeof(response),
             "dji_armed=%u",
             elrs_config_get_dji_permanently_armed() ? 1U : 0U);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "ble_remote_id") ||
             ascii_equal_ignore_case(key, "remote_id") ||
             ascii_equal_ignore_case(key, "rid_enabled")) {
    snprintf(response,
             sizeof(response),
             "ble_remote_id=%u",
             elrs_config_get_ble_remote_id() ? 1U : 0U);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "mav_tgt") ||
             ascii_equal_ignore_case(key, "mavlink_target")) {
    snprintf(response, sizeof(response), "mav_tgt=%u", (unsigned int)cfg->mavlink_target_sys_id);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "mav_src") ||
             ascii_equal_ignore_case(key, "mavlink_source")) {
    snprintf(response, sizeof(response), "mav_src=%u", (unsigned int)cfg->mavlink_source_sys_id);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "team_ch")) {
    snprintf(response, sizeof(response), "team_ch=%u", (unsigned int)cfg->teamrace_channel);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "team_pos")) {
    snprintf(response, sizeof(response), "team_pos=%u", (unsigned int)cfg->teamrace_position);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "bind") ||
             ascii_equal_ignore_case(key, "bind_storage")) {
    snprintf(response, sizeof(response), "bind=%u", (unsigned int)cfg->bind_storage);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "vbind")) {
    snprintf(response, sizeof(response), "vbind=%u", (unsigned int)cfg->vbind);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "dirty")) {
    snprintf(response, sizeof(response), "dirty=%u", config_dirty ? 1U : 0U);
    ble_publish_ephemeral_response(response);
  } else {
    ble_publish_ephemeral_response("err unknown key");
  }
}

static void ble_publish_json_key_u32(const char *key, unsigned int value)
{
  char response[BLE_VALUE_MAX];
  snprintf(response,
           sizeof(response),
           "{\"ok\":true,\"key\":\"%s\",\"value\":%u}",
           key,
           value);
  ble_publish_response(response);
}

static void ble_publish_json_key_i32(const char *key, int value)
{
  char response[BLE_VALUE_MAX];
  snprintf(response,
           sizeof(response),
           "{\"ok\":true,\"key\":\"%s\",\"value\":%d}",
           key,
           value);
  ble_publish_response(response);
}

static void ble_publish_json_key_string(const char *key, const char *value)
{
  char response[BLE_VALUE_MAX];
  char escaped[96];
  ble_json_escape(escaped, sizeof(escaped), value);
  snprintf(response,
           sizeof(response),
           "{\"ok\":true,\"key\":\"%s\",\"value\":\"%s\"}",
           key,
           escaped);
  ble_publish_response(response);
}

static void ble_format_json_key_response(const char *key)
{
  const elrs_config_t *cfg = elrs_config_get();
  char uid[18];

  if (cfg == NULL || !config_ready) {
    ble_publish_json_error("config", "config not ready");
  } else if (ascii_equal_ignore_case(key, "version") ||
             ascii_equal_ignore_case(key, "v")) {
    ble_publish_json_key_u32("version", (unsigned int)cfg->version);
  } else if (ascii_equal_ignore_case(key, "flags")) {
    ble_publish_json_key_u32("flags", (unsigned int)cfg->flags);
  } else if (ascii_equal_ignore_case(key, "uid")) {
    snprintf(uid,
             sizeof(uid),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned int)cfg->uid[0],
             (unsigned int)cfg->uid[1],
             (unsigned int)cfg->uid[2],
             (unsigned int)cfg->uid[3],
             (unsigned int)cfg->uid[4],
             (unsigned int)cfg->uid[5]);
    ble_publish_json_key_string("uid", uid);
  } else if (ascii_equal_ignore_case(key, "serial")) {
    ble_publish_json_key_u32("serial", (unsigned int)cfg->serial_protocol);
  } else if (ascii_equal_ignore_case(key, "failsafe")) {
    ble_publish_json_key_u32("failsafe", (unsigned int)cfg->failsafe_mode);
  } else if (ascii_equal_ignore_case(key, "rate")) {
    ble_publish_json_key_u32("rate", (unsigned int)cfg->rate_index);
  } else if (ascii_equal_ignore_case(key, "model") ||
             ascii_equal_ignore_case(key, "model_id")) {
    ble_publish_json_key_u32("model", (unsigned int)cfg->model_id);
  } else if (ascii_equal_ignore_case(key, "tlm_off") ||
             ascii_equal_ignore_case(key, "force_tlm")) {
    ble_publish_json_key_u32("tlm_off", (unsigned int)cfg->force_tlm);
  } else if (ascii_equal_ignore_case(key, "tlm_interval") ||
             ascii_equal_ignore_case(key, "tlm_int")) {
    ble_publish_json_key_u32("tlm_interval", (unsigned int)cfg->tlm_interval);
  } else if (ascii_equal_ignore_case(key, "power")) {
    if (cfg->tx_power == ELRS_TX_POWER_MATCH_TX_DBM) {
      ble_publish_json_key_string("power", "match");
    } else {
      ble_publish_json_key_i32("power", (int)cfg->tx_power);
    }
  } else if (ascii_equal_ignore_case(key, "domain_low") ||
             ascii_equal_ignore_case(key, "domL")) {
    ble_publish_json_key_u32("domL", (unsigned int)cfg->reg_domain_low);
  } else if (ascii_equal_ignore_case(key, "domain_high") ||
             ascii_equal_ignore_case(key, "domH")) {
    ble_publish_json_key_u32("domH", (unsigned int)cfg->reg_domain_high);
  } else if (ascii_equal_ignore_case(key, "web_domain")) {
    ble_publish_json_key_u32("web_domain", (unsigned int)elrs_config_get_web_domain());
  } else if (ascii_equal_ignore_case(key, "wifi_channel")) {
    ble_publish_json_key_u32("wifi_channel", (unsigned int)cfg->wifi_channel);
  } else if (ascii_equal_ignore_case(key, "wifi_interval") ||
             ascii_equal_ignore_case(key, "wifi_int")) {
    ble_publish_json_key_i32("wifi_interval", (int)elrs_config_get_wifi_on_interval());
  } else if (ascii_equal_ignore_case(key, "uart_baud") ||
             ascii_equal_ignore_case(key, "baud")) {
    ble_publish_json_key_u32("uart_baud", (unsigned int)elrs_config_get_uart_baud());
  } else if (ascii_equal_ignore_case(key, "wifi_ssid")) {
    ble_publish_json_key_string("wifi_ssid", cfg->wifi_ssid);
  } else if (ascii_equal_ignore_case(key, "wifi_custom")) {
    ble_publish_json_key_u32("wifi_custom",
                             (cfg->flags & ELRS_CONFIG_FLAG_WIFI_CUSTOM) ? 1U : 0U);
  } else if (ascii_equal_ignore_case(key, "web_custom") ||
             ascii_equal_ignore_case(key, "web_customised") ||
             ascii_equal_ignore_case(key, "web_options_customised")) {
    ble_publish_json_key_u32("web_custom",
                             elrs_config_web_options_customised() ? 1U : 0U);
  } else if (ascii_equal_ignore_case(key, "lock_on_first") ||
             ascii_equal_ignore_case(key, "lock_first") ||
             ascii_equal_ignore_case(key, "lock")) {
    ble_publish_json_key_u32("lock_on_first",
                             elrs_config_get_lock_on_first_connection() ? 1U : 0U);
  } else if (ascii_equal_ignore_case(key, "is_airport") ||
             ascii_equal_ignore_case(key, "airport")) {
    ble_publish_json_key_u32("is_airport", elrs_config_get_is_airport() ? 1U : 0U);
  } else if (ascii_equal_ignore_case(key, "dji_armed") ||
             ascii_equal_ignore_case(key, "dji") ||
             ascii_equal_ignore_case(key, "dji_permanently_armed")) {
    ble_publish_json_key_u32("dji_armed",
                             elrs_config_get_dji_permanently_armed() ? 1U : 0U);
  } else if (ascii_equal_ignore_case(key, "ble_remote_id") ||
             ascii_equal_ignore_case(key, "remote_id") ||
             ascii_equal_ignore_case(key, "rid_enabled")) {
    ble_publish_json_key_u32("ble_remote_id",
                             elrs_config_get_ble_remote_id() ? 1U : 0U);
  } else if (ascii_equal_ignore_case(key, "mav_tgt") ||
             ascii_equal_ignore_case(key, "mavlink_target")) {
    ble_publish_json_key_u32("mav_tgt", (unsigned int)cfg->mavlink_target_sys_id);
  } else if (ascii_equal_ignore_case(key, "mav_src") ||
             ascii_equal_ignore_case(key, "mavlink_source")) {
    ble_publish_json_key_u32("mav_src", (unsigned int)cfg->mavlink_source_sys_id);
  } else if (ascii_equal_ignore_case(key, "team_ch")) {
    ble_publish_json_key_u32("team_ch", (unsigned int)cfg->teamrace_channel);
  } else if (ascii_equal_ignore_case(key, "team_pos")) {
    ble_publish_json_key_u32("team_pos", (unsigned int)cfg->teamrace_position);
  } else if (ascii_equal_ignore_case(key, "bind") ||
             ascii_equal_ignore_case(key, "bind_storage")) {
    ble_publish_json_key_u32("bind", (unsigned int)cfg->bind_storage);
  } else if (ascii_equal_ignore_case(key, "vbind")) {
    ble_publish_json_key_u32("vbind", (unsigned int)cfg->vbind);
  } else if (ascii_equal_ignore_case(key, "dirty")) {
    ble_publish_json_key_u32("dirty", config_dirty ? 1U : 0U);
  } else {
    ble_publish_json_error("key", "unknown key");
  }
}

static void ble_reload_config_from_storage(void)
{
  int status = elrs_config_init();
  if (status == 0) {
    config_ready = true;
    config_dirty = false;
    ble_publish_ephemeral_response("ok reloaded");
  } else {
    config_ready = false;
    config_dirty = false;
    ble_publish_ephemeral_response("err reload failed");
  }
}

static void ble_send_read_response(const rsi_ble_read_req_t *read,
                                   const uint8_t *data,
                                   uint16_t data_len)
{
  uint16_t offset = read->offset;
  uint16_t remaining = 0;
  const uint8_t *chunk = data;

  if (offset < data_len) {
    remaining = (uint16_t)(data_len - offset);
    chunk = data + offset;
  }

  if (remaining > 20U) {
    remaining = 20U;
  }

  int32_t status = rsi_ble_gatt_read_response((uint8_t *)read->dev_addr,
                                              read->type,
                                              read->handle,
                                              offset,
                                              remaining,
                                              chunk);
  if (status != RSI_SUCCESS) {
    DEBUGOUT("[BLE] Read response failed handle=0x%04X status=0x%lX\n",
             (unsigned int)read->handle,
             (unsigned long)status);
  } else {
    BLE_LOG_DEBUGOUT("[BLE] Read response handle=0x%04X offset=%u len=%u status=0x%lX\n",
                     (unsigned int)read->handle,
                     (unsigned int)offset,
                     (unsigned int)remaining,
                     (unsigned long)status);
  }
}

static void ble_handle_set_command(char *args)
{
  char *separator = strchr(args, '=');
  char response[96];
  elrs_config_t *cfg = elrs_config_get();
  uint8_t value_u8 = 0;
  uint32_t value_u32 = 0;
  int32_t value_i32 = 0;

  if (separator == NULL) {
    ble_publish_ephemeral_response("err set syntax: set key=value");
    return;
  }

  *separator = '\0';
  char *key = trim_ascii(args);
  char *value = trim_ascii(separator + 1);

  if (!config_ready) {
    ble_publish_ephemeral_response("err config not ready");
  } else if (ascii_equal_ignore_case(key, "serial")) {
    if (!parse_u8_range(value, 0, 9, &value_u8) ||
        !elrs_serial_protocol_is_supported(value_u8)) {
      ble_publish_ephemeral_response("err serial use 0=CRSF 2=SBUS 4=SUMD 7=MAVLink");
      return;
    }
    cfg->serial_protocol = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok serial=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "failsafe")) {
    if (!parse_u8_range(value, 0, 2, &value_u8)) {
      ble_publish_ephemeral_response("err failsafe range 0..2");
      return;
    }
    cfg->failsafe_mode = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok failsafe=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "rate")) {
    if (!parse_u8_range(value, 0, 31, &value_u8)) {
      ble_publish_ephemeral_response("err rate range 0..31");
      return;
    }
    cfg->rate_index = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok rate=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "model") ||
             ascii_equal_ignore_case(key, "model_id")) {
    if (!parse_u8_range(value, 0, 255, &value_u8) ||
        (value_u8 > 63 && value_u8 != 255)) {
      ble_publish_ephemeral_response("err model range 0..63 or 255");
      return;
    }
    cfg->model_id = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok model=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "tlm_off") ||
             ascii_equal_ignore_case(key, "force_tlm")) {
    if (!parse_u8_range(value, 0, 1, &value_u8)) {
      ble_publish_ephemeral_response("err tlm_off range 0..1");
      return;
    }
    cfg->force_tlm = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok tlm_off=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "tlm_interval") ||
             ascii_equal_ignore_case(key, "tlm_int")) {
    if (!parse_u8_range(value, 0, 255, &value_u8)) {
      ble_publish_ephemeral_response("err tlm_interval range 0..255");
      return;
    }
    cfg->tlm_interval = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok tlm_interval=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "power")) {
    if (ascii_equal_ignore_case(value, "match")) {
      cfg->tx_power = ELRS_TX_POWER_MATCH_TX_DBM;
      ble_mark_dirty();
      ble_publish_ephemeral_response("ok power=match; save");
      return;
    }

    if (!parse_u8_range(value, 10, 20, &value_u8) ||
        !(value_u8 == 10 || value_u8 == 14 ||
          value_u8 == 17 || value_u8 == 20)) {
      ble_publish_ephemeral_response("err power use match,10,14,17,20");
      return;
    }
    cfg->tx_power = (int8_t)value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok power=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "domain_low") ||
             ascii_equal_ignore_case(key, "domL")) {
    if (!parse_u8_range(value, 0, 7, &value_u8)) {
      ble_publish_ephemeral_response("err domain range 0..7");
      return;
    }
    cfg->reg_domain_low = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok domain_low=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "domain_high") ||
             ascii_equal_ignore_case(key, "domH")) {
    if (!parse_u8_range(value, 0, 7, &value_u8)) {
      ble_publish_ephemeral_response("err domain range 0..7");
      return;
    }
    cfg->reg_domain_high = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok domain_high=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "web_domain")) {
    if (!parse_u8_range(value, 0, 5, &value_u8)) {
      ble_publish_ephemeral_response("err web_domain range 0..5");
      return;
    }
    elrs_config_set_web_domain(value_u8);
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok web_domain=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "wifi_channel")) {
    if (!parse_u8_range(value, 1, 13, &value_u8)) {
      ble_publish_ephemeral_response("err wifi_channel range 1..13");
      return;
    }
    cfg->wifi_channel = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok wifi_channel=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "wifi_interval") ||
             ascii_equal_ignore_case(key, "wifi_int")) {
    if (!parse_i32_range(value, -1, 86400, &value_i32)) {
      ble_publish_ephemeral_response("err wifi_interval range -1..86400");
      return;
    }
    elrs_config_set_wifi_on_interval(value_i32);
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok wifi_interval=%ld; save", (long)value_i32);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "uart_baud") ||
             ascii_equal_ignore_case(key, "baud")) {
    if (!parse_u32_range(value, 9600, 2000000, &value_u32)) {
      ble_publish_ephemeral_response("err uart_baud range 9600..2000000");
      return;
    }
    elrs_config_set_uart_baud(value_u32);
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok uart_baud=%lu; save", (unsigned long)value_u32);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "wifi_ssid")) {
    if (strlen(value) > 32) {
      ble_publish_ephemeral_response("err wifi_ssid max 32");
      return;
    }
    memset(cfg->wifi_ssid, 0, sizeof(cfg->wifi_ssid));
    strncpy(cfg->wifi_ssid, value, sizeof(cfg->wifi_ssid) - 1);
    cfg->flags |= ELRS_CONFIG_FLAG_WIFI_CUSTOM;
    ble_mark_dirty();
    ble_publish_ephemeral_response("ok wifi_ssid; save");
  } else if (ascii_equal_ignore_case(key, "wifi_password")) {
    if (strlen(value) > 64) {
      ble_publish_ephemeral_response("err wifi_password max 64");
      return;
    }
    memset(cfg->wifi_password, 0, sizeof(cfg->wifi_password));
    strncpy(cfg->wifi_password, value, sizeof(cfg->wifi_password) - 1);
    cfg->flags |= ELRS_CONFIG_FLAG_WIFI_CUSTOM;
    ble_mark_dirty();
    ble_publish_ephemeral_response("ok wifi_password; save");
  } else if (ascii_equal_ignore_case(key, "mav_tgt") ||
             ascii_equal_ignore_case(key, "mavlink_target")) {
    if (!parse_u8_range(value, 1, 255, &value_u8)) {
      ble_publish_ephemeral_response("err mav_tgt range 1..255");
      return;
    }
    cfg->mavlink_target_sys_id = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok mav_tgt=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "mav_src") ||
             ascii_equal_ignore_case(key, "mavlink_source")) {
    if (!parse_u8_range(value, 1, 255, &value_u8)) {
      ble_publish_ephemeral_response("err mav_src range 1..255");
      return;
    }
    cfg->mavlink_source_sys_id = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok mav_src=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "team_ch")) {
    if (!parse_u8_range(value, 0, 10, &value_u8)) {
      ble_publish_ephemeral_response("err team_ch range 0..10");
      return;
    }
    cfg->teamrace_channel = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok team_ch=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "team_pos")) {
    if (!parse_u8_range(value, 0, 7, &value_u8)) {
      ble_publish_ephemeral_response("err team_pos range 0..7");
      return;
    }
    cfg->teamrace_position = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok team_pos=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "bind") ||
             ascii_equal_ignore_case(key, "bind_storage")) {
    if (!parse_u8_range(value, 0, 3, &value_u8)) {
      ble_publish_ephemeral_response("err bind range 0..3");
      return;
    }
    cfg->bind_storage = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok bind=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "vbind")) {
    if (!parse_u8_range(value, 0, 255, &value_u8)) {
      ble_publish_ephemeral_response("err vbind range 0..255");
      return;
    }
    cfg->vbind = value_u8;
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok vbind=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "wifi_custom")) {
    if (!parse_u8_range(value, 0, 1, &value_u8)) {
      ble_publish_ephemeral_response("err wifi_custom range 0..1");
      return;
    }
    if (value_u8) {
      cfg->flags |= ELRS_CONFIG_FLAG_WIFI_CUSTOM;
    } else {
      cfg->flags &= (uint8_t)~ELRS_CONFIG_FLAG_WIFI_CUSTOM;
    }
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok wifi_custom=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "lock_on_first") ||
             ascii_equal_ignore_case(key, "lock_first") ||
             ascii_equal_ignore_case(key, "lock")) {
    if (!parse_u8_range(value, 0, 1, &value_u8)) {
      ble_publish_ephemeral_response("err lock_on_first range 0..1");
      return;
    }
    elrs_config_set_lock_on_first_connection(value_u8 != 0U);
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok lock_on_first=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "is_airport") ||
             ascii_equal_ignore_case(key, "airport")) {
    if (!parse_u8_range(value, 0, 1, &value_u8)) {
      ble_publish_ephemeral_response("err is_airport range 0..1");
      return;
    }
    elrs_config_set_is_airport(value_u8 != 0U);
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok is_airport=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "dji_armed") ||
             ascii_equal_ignore_case(key, "dji") ||
             ascii_equal_ignore_case(key, "dji_permanently_armed")) {
    if (!parse_u8_range(value, 0, 1, &value_u8)) {
      ble_publish_ephemeral_response("err dji_armed range 0..1");
      return;
    }
    elrs_config_set_dji_permanently_armed(value_u8 != 0U);
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok dji_armed=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(key, "ble_remote_id") ||
             ascii_equal_ignore_case(key, "remote_id") ||
             ascii_equal_ignore_case(key, "rid_enabled")) {
    if (!parse_u8_range(value, 0, 1, &value_u8)) {
      ble_publish_ephemeral_response("err ble_remote_id range 0..1");
      return;
    }
    elrs_config_set_ble_remote_id(value_u8 != 0U);
    ble_remote_id_service_set_enabled(value_u8 != 0U);
    ble_mark_dirty();
    snprintf(response, sizeof(response), "ok ble_remote_id=%u; save", (unsigned int)value_u8);
    ble_publish_ephemeral_response(response);
  } else {
    ble_publish_ephemeral_response("err unknown key");
  }
}

static void ble_handle_remote_id_location_command(char *args)
{
  char response[BLE_VALUE_MAX];
  char *cursor = trim_ascii(args);
  double latitude;
  double longitude;
  int32_t latitude_e7;
  int32_t longitude_e7;
  int32_t altitude_i32 = 0;

  if (*cursor == '\0' || ascii_equal_ignore_case(cursor, "status")) {
    ble_remote_id_format_location(response, sizeof(response));
    ble_publish_ephemeral_response(response);
    return;
  }

  if (ascii_equal_ignore_case(cursor, "clear") ||
      ascii_equal_ignore_case(cursor, "off") ||
      ascii_equal_ignore_case(cursor, "unset")) {
    ble_remote_id_clear_location();
    ble_remote_id_next_adv_kind = BLE_REMOTE_ID_ADV_BASIC_ID;
    ble_publish_ephemeral_response("ok rid loc cleared");
    return;
  }

  if (!parse_double_arg(&cursor, -90.0, 90.0, &latitude) ||
      !parse_double_arg(&cursor, -180.0, 180.0, &longitude) ||
      !double_to_e7(latitude, &latitude_e7) ||
      !double_to_e7(longitude, &longitude_e7)) {
    ble_publish_ephemeral_response("err rid loc use: rid loc <lat> <lon> [alt_m]");
    return;
  }

  cursor = skip_arg_separator(cursor);
  if (*cursor != '\0') {
    if (!parse_i32_range(cursor, -1000, 31767, &altitude_i32)) {
      ble_publish_ephemeral_response("err rid loc altitude -1000..31767 m");
      return;
    }
  }

  if (!ble_remote_id_set_location_e7(latitude_e7, longitude_e7, (int16_t)altitude_i32)) {
    ble_publish_ephemeral_response("err rid loc range or 0,0 invalid");
    return;
  }

  ble_remote_id_next_adv_kind = BLE_REMOTE_ID_ADV_LOCATION;
  if (ble_adv_mode == BLE_ADV_MODE_REMOTE_ID_BASIC && !is_connected) {
    int32_t adv_status = ble_apply_current_advertisement();
    snprintf(response,
             sizeof(response),
             "ok rid loc lat_e7=%ld lon_e7=%ld alt_m=%ld adv=0x%lX",
             (long)latitude_e7,
             (long)longitude_e7,
             (long)altitude_i32,
             (unsigned long)adv_status);
  } else {
    snprintf(response,
             sizeof(response),
             "ok rid loc lat_e7=%ld lon_e7=%ld alt_m=%ld queued",
             (long)latitude_e7,
             (long)longitude_e7,
             (long)altitude_i32);
  }
  ble_publish_ephemeral_response(response);
}

static void ble_handle_remote_id_command(char *args)
{
  char response[BLE_VALUE_MAX];

  args = trim_ascii(args);
  if (*args == '\0' || ascii_equal_ignore_case(args, "status")) {
    ble_remote_id_format_status(response,
                                sizeof(response),
                                ble_adv_mode == BLE_ADV_MODE_REMOTE_ID_BASIC);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(args, "help")) {
    ble_publish_ephemeral_response("rid: status,id <1..20>,loc <lat> <lon> [alt],preview [basic|loc],adv [sec],elrs");
  } else if (ascii_starts_with_ignore_case(args, "id ")) {
    char *id = trim_ascii(args + 3);
    if (ble_remote_id_set_uas_id(id)) {
      snprintf(response,
               sizeof(response),
               "ok rid id=%s",
               ble_remote_id_get_uas_id());
      ble_publish_ephemeral_response(response);
    } else {
      ble_publish_ephemeral_response("err rid id 1..20 chars A-Z 0-9 - _ .");
    }
  } else if (ascii_equal_ignore_case(args, "loc") ||
             ascii_starts_with_ignore_case(args, "loc ")) {
    ble_handle_remote_id_location_command((args[3] == '\0') ? "" : args + 4);
  } else if (ascii_starts_with_ignore_case(args, "preview") ||
             ascii_starts_with_ignore_case(args, "hex")) {
    char hex[96];
    bool is_hex = ascii_starts_with_ignore_case(args, "hex");
    char *kind_text = trim_ascii(args + (is_hex ? 3 : 7));
    ble_remote_id_adv_kind_t kind = BLE_REMOTE_ID_ADV_BASIC_ID;
    const char *name = "basic";

    if (ascii_equal_ignore_case(kind_text, "loc") ||
        ascii_equal_ignore_case(kind_text, "location")) {
      kind = BLE_REMOTE_ID_ADV_LOCATION;
      name = "loc";
      if (!ble_remote_id_location_is_set()) {
        ble_publish_ephemeral_response("err rid loc unset");
        return;
      }
    } else if (*kind_text != '\0' &&
               !ascii_equal_ignore_case(kind_text, "basic")) {
      ble_publish_ephemeral_response("err rid preview basic|loc");
      return;
    }

    ble_remote_id_format_adv_hex(hex, sizeof(hex), kind);
    snprintf(response, sizeof(response), "rid %s_advhex=%s", name, hex);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(args, "elrs") ||
             ascii_equal_ignore_case(args, "off")) {
    ble_adv_mode = BLE_ADV_MODE_ELRS_CONFIG;
    ble_remote_id_restore_tick = 0U;
    if (!is_connected) {
      int32_t adv_status = ble_apply_current_advertisement();
      snprintf(response,
               sizeof(response),
               "ok elrs advertising status=0x%lX",
               (unsigned long)adv_status);
    } else {
      snprintf(response, sizeof(response), "ok elrs advertising queued");
    }
    ble_publish_ephemeral_response(response);
  } else if (ascii_starts_with_ignore_case(args, "adv")) {
    uint16_t seconds = 60U;
    char *timeout_text = trim_ascii(args + 3);
    if (*timeout_text != '\0' &&
        !parse_u16_range(timeout_text, 5, 600, &seconds)) {
      ble_publish_ephemeral_response("err rid adv seconds 5..600");
      return;
    }

    ble_adv_mode = BLE_ADV_MODE_REMOTE_ID_BASIC;
    ble_remote_id_next_adv_kind = BLE_REMOTE_ID_ADV_BASIC_ID;
    ble_remote_id_restore_tick =
        osKernelGetTickCount() + ble_seconds_to_ticks(seconds);

    if (!is_connected) {
      int32_t adv_status = ble_apply_current_advertisement();
      snprintf(response,
               sizeof(response),
               "ok rid advertising %us status=0x%lX id=%s",
               (unsigned int)seconds,
               (unsigned long)adv_status,
               ble_remote_id_get_uas_id());
    } else {
      snprintf(response,
               sizeof(response),
               "ok rid queued %us; disconnect to advertise id=%s",
               (unsigned int)seconds,
               ble_remote_id_get_uas_id());
    }
    ble_publish_ephemeral_response(response);
  } else {
    ble_publish_ephemeral_response("err rid command; write rid help");
  }
}

static void ble_handle_command(const uint8_t *data, uint16_t len)
{
  char command[BLE_VALUE_MAX + 1];
  if (len > BLE_VALUE_MAX) {
    len = BLE_VALUE_MAX;
  }

  memcpy(command, data, len);
  command[len] = '\0';
  char *trimmed = trim_ascii(command);
  ble_note_last_command(trimmed);
  BLE_LOG_DEBUGOUT("[BLE] Command '%s'\n", trimmed);

  if (*trimmed == '\0') {
    ble_publish_ephemeral_response("err empty command");
  } else if (ascii_equal_ignore_case(trimmed, "help")) {
    ble_publish_ephemeral_response("cmds: ping,status,jstatus,jget,meta,diag,get,set,save,reload,len,page,rid,ae");
  } else if (ascii_equal_ignore_case(trimmed, "api")) {
    ble_publish_ephemeral_response("api=elrs-ble-v1");
  } else if (ascii_equal_ignore_case(trimmed, "caps")) {
    ble_format_caps_response();
  } else if (ascii_equal_ignore_case(trimmed, "meta")) {
    ble_format_meta_response();
  } else if (ascii_equal_ignore_case(trimmed, "diag")) {
    ble_format_diag_response();
  } else if (ascii_equal_ignore_case(trimmed, "keys")) {
    ble_format_keys_response();
  } else if (ascii_equal_ignore_case(trimmed, "setkeys")) {
    ble_format_setkeys_response();
  } else if (ascii_equal_ignore_case(trimmed, "ranges") ||
             ascii_equal_ignore_case(trimmed, "schema")) {
    ble_format_ranges_response();
  } else if (ascii_equal_ignore_case(trimmed, "ping")) {
    ble_publish_ephemeral_response("pong");
  } else if (ascii_equal_ignore_case(trimmed, "dirty")) {
    ble_publish_ephemeral_response(config_dirty ? "dirty=1" : "dirty=0");
  } else if (ascii_equal_ignore_case(trimmed, "rid")) {
    ble_handle_remote_id_command("");
  } else if (ascii_starts_with_ignore_case(trimmed, "rid ")) {
    ble_handle_remote_id_command(trimmed + 4);
  } else if (ascii_equal_ignore_case(trimmed, "ae")) {
    ble_handle_ae_command("");
  } else if (ascii_starts_with_ignore_case(trimmed, "ae ")) {
    ble_handle_ae_command(trimmed + 3);
  } else if (ascii_equal_ignore_case(trimmed, "jstatus")) {
    ble_format_json_status_response();
  } else if (ascii_equal_ignore_case(trimmed, "jget") ||
             ascii_equal_ignore_case(trimmed, "jget all")) {
    ble_format_json_config_response();
  } else if (ascii_starts_with_ignore_case(trimmed, "jget ")) {
    ble_format_json_key_response(trim_ascii(trimmed + 5));
  } else if (ascii_equal_ignore_case(trimmed, "status")) {
    char response[48];
    snprintf(response,
             sizeof(response),
             "ok cfg=%u dirty=%u ntf=%u len=%u",
             config_ready ? 1U : 0U,
             config_dirty ? 1U : 0U,
             (status_notifications_enabled || command_notifications_enabled) ? 1U : 0U,
             (unsigned int)payload_value_len);
    ble_publish_ephemeral_response(response);
  } else if (ascii_equal_ignore_case(trimmed, "len")) {
    char response[24];
    snprintf(response, sizeof(response), "len=%u", (unsigned int)payload_value_len);
    ble_publish_helper_response(response);
  } else if (ascii_starts_with_ignore_case(trimmed, "chunk ")) {
    uint16_t offset = 0;
    if (parse_u16_range(trimmed + 6, 0, BLE_VALUE_MAX - 1U, &offset)) {
      ble_publish_payload_chunk(offset);
    } else {
      ble_publish_helper_response("err chunk n");
    }
  } else if (ascii_starts_with_ignore_case(trimmed, "page ")) {
    uint16_t page = 0;
    if (parse_u16_range(trimmed + 5, 0, (BLE_VALUE_MAX - 1U) / BLE_NOTIFY_CHUNK_MAX, &page)) {
      ble_publish_payload_chunk((uint16_t)(page * BLE_NOTIFY_CHUNK_MAX));
    } else {
      ble_publish_helper_response("err page n");
    }
  } else if (ascii_equal_ignore_case(trimmed, "get") ||
             ascii_equal_ignore_case(trimmed, "get all")) {
    if (!config_ready) {
      ble_publish_ephemeral_response("err config not ready");
    } else {
      ble_format_config_response();
    }
  } else if (ascii_starts_with_ignore_case(trimmed, "get ")) {
    ble_format_key_response(trim_ascii(trimmed + 4));
  } else if (ascii_equal_ignore_case(trimmed, "save") ||
             ascii_equal_ignore_case(trimmed, "commit")) {
    if (!config_ready) {
      ble_publish_ephemeral_response("err config not ready");
    } else if (elrs_config_save() == 0) {
      config_dirty = false;
      ble_publish_ephemeral_response("ok saved");
    } else {
      ble_publish_ephemeral_response("err save failed");
    }
  } else if (ascii_equal_ignore_case(trimmed, "reload") ||
             ascii_equal_ignore_case(trimmed, "discard")) {
    ble_reload_config_from_storage();
  } else if (ascii_equal_ignore_case(trimmed, "reset")) {
    if (!config_ready) {
      ble_publish_ephemeral_response("err config not ready");
    } else if (elrs_config_reset() == 0) {
      config_dirty = false;
      ble_publish_ephemeral_response("ok reset");
    } else {
      ble_publish_ephemeral_response("err reset failed");
    }
  } else if (ascii_starts_with_ignore_case(trimmed, "set ")) {
    ble_handle_set_command(trimmed + 4);
  } else if (trimmed[0] == 'j' || trimmed[0] == 'J') {
    ble_publish_json_error("command", "unknown json command");
  } else {
    ble_publish_ephemeral_response("err unknown command; write help");
  }
}

static void ble_add_char_decl(void *service,
                              uint16_t handle,
                              uint8_t properties,
                              uint16_t value_handle,
                              uint16_t uuid16)
{
  rsi_ble_req_add_att_t att = { 0 };

  att.serv_handler = service;
  att.handle = handle;
  att.att_uuid.size = 2;
  att.att_uuid.val.val16 = BLE_UUID_CHAR_DECL;
  att.property = BLE_ATT_PROPERTY_READ;
  att.data_len = 6;
  att.data[0] = properties;
  rsi_uint16_to_2bytes(&att.data[2], value_handle);
  rsi_uint16_to_2bytes(&att.data[4], uuid16);

  (void)rsi_ble_add_attribute(&att);
}

static void ble_add_char_value(void *service,
                               uint16_t handle,
                               uint16_t uuid16,
                               uint8_t properties,
                               uint8_t *data,
                               uint16_t data_len)
{
  rsi_ble_req_add_att_t att = { 0 };

  att.serv_handler = service;
  att.handle = handle;
  att.att_uuid.size = 2;
  att.att_uuid.val.val16 = uuid16;
  att.property = properties;
  att.data_len = (uint16_t)RSI_MIN(sizeof(att.data), data_len);
  memcpy(att.data, data, att.data_len);
  (void)rsi_ble_add_attribute(&att);

  if (properties & BLE_ATT_PROPERTY_NOTIFY) {
    memset(&att, 0, sizeof(att));
    att.serv_handler = service;
    att.handle = handle + 1U;
    att.att_uuid.size = 2;
    att.att_uuid.val.val16 = BLE_UUID_CCCD;
    att.property = BLE_ATT_PROPERTY_READ | BLE_ATT_PROPERTY_WRITE;
    att.data_len = 2;
    (void)rsi_ble_add_attribute(&att);
  }
}

static int32_t ble_add_elrs_service(void)
{
  uuid_t uuid = { 0 };
  rsi_ble_resp_add_serv_t service = { 0 };
  int32_t status;

  uuid.size = 2;
  uuid.val.val16 = BLE_UUID_ELRS_SERVICE;
  status = rsi_ble_add_service(uuid, &service);
  BLE_LOG_DEBUGOUT("[BLE] add service status=0x%lX start=%u\n",
                   (unsigned long)status,
                   (unsigned int)service.start_handle);
  if (status != RSI_SUCCESS) {
    return status;
  }

  status_handle = service.start_handle + 2U;
  status_cccd_handle = status_handle + 1U;
  ble_add_char_decl(service.serv_handler,
                    service.start_handle + 1U,
                    BLE_ATT_PROPERTY_READ | BLE_ATT_PROPERTY_NOTIFY,
                    status_handle,
                    BLE_UUID_ELRS_STATUS);
  ble_add_char_value(service.serv_handler,
                     status_handle,
                     BLE_UUID_ELRS_STATUS,
                     BLE_ATT_PROPERTY_READ | BLE_ATT_PROPERTY_NOTIFY,
                     status_value,
                     BLE_NOTIFY_CHUNK_MAX);

  command_handle = service.start_handle + 5U;
  command_cccd_handle = command_handle + 1U;
  ble_add_char_decl(service.serv_handler,
                    service.start_handle + 4U,
                    BLE_ATT_PROPERTY_READ | BLE_ATT_PROPERTY_WRITE |
                      BLE_ATT_PROPERTY_WRITE_NO_RESPONSE |
                      BLE_ATT_PROPERTY_NOTIFY,
                    command_handle,
                    BLE_UUID_ELRS_COMMAND);
  ble_add_char_value(service.serv_handler,
                     command_handle,
                     BLE_UUID_ELRS_COMMAND,
                     BLE_ATT_PROPERTY_READ | BLE_ATT_PROPERTY_WRITE |
                       BLE_ATT_PROPERTY_WRITE_NO_RESPONSE |
                       BLE_ATT_PROPERTY_NOTIFY,
                     command_value,
                     BLE_NOTIFY_CHUNK_MAX);

  BLE_LOG_DEBUGOUT("[BLE] handles status=0x%04X status_cccd=0x%04X command=0x%04X command_cccd=0x%04X\n",
                   (unsigned int)status_handle,
                   (unsigned int)status_cccd_handle,
                   (unsigned int)command_handle,
                   (unsigned int)command_cccd_handle);

  return RSI_SUCCESS;
}

static void ble_on_connect(rsi_ble_event_conn_status_t *conn)
{
  memcpy(&ble_conn, conn, sizeof(ble_conn));
  ble_set_event(BLE_EVENT_CONNECTED);
}

static void ble_on_disconnect(rsi_ble_event_disconnect_t *disc, uint16_t reason)
{
  (void)reason;
  memcpy(&ble_disconnect, disc, sizeof(ble_disconnect));
  ble_set_event(BLE_EVENT_DISCONNECTED);
}

static void ble_on_write(uint16_t event_id, rsi_ble_event_write_t *write)
{
  (void)event_id;
  memcpy(&ble_write, write, sizeof(ble_write));
  ble_set_event(BLE_EVENT_WRITE);
}

static void ble_on_read_req(uint16_t event_id, rsi_ble_read_req_t *read)
{
  (void)event_id;
  memcpy(&ble_read, read, sizeof(ble_read));
  ble_set_event(BLE_EVENT_READ);
}

static void ble_task(void *argument)
{
  (void)argument;

  sl_status_t sl_status;
  int32_t status;
#if BLE_LOG_VERBOSE
  sl_wifi_firmware_version_t fw_version = { 0 };
  uint8_t local_addr[RSI_DEV_ADDR_LEN] = { 0 };
  uint8_t local_addr_text[18] = { 0 };
#endif

  DEBUGOUT("[BLE] Config API starting (%s)\n", BLE_PROBE_NAME);

  if (device_initialized) {
    BLE_LOG_DEBUGOUT("[BLE] NWP already initialized; attaching BLE service without sl_wifi_init()\n");
    BLE_LOG_DEBUGOUT("[BLE] If GATT setup fails, NWP was likely booted without BLE coex enabled.\n");
    sl_status = SL_STATUS_OK;
  } else {
    sl_status = (sl_status_t)ble_gatt_config_api_prepare_nwp();
    if (sl_status != SL_STATUS_OK) {
      DEBUGOUT("[BLE] init failed; BLE config API stopped\n");
      ble_task_exit();
    }
  }

#if BLE_LOG_VERBOSE
  sl_status = sl_wifi_get_firmware_version(&fw_version);
  if (sl_status == SL_STATUS_OK) {
    print_firmware_version(&fw_version);
  }
#endif

  int config_status = 0;
  if (!elrs_config_is_initialized()) {
    config_status = elrs_config_init();
  }
  config_ready = (config_status == 0);
  config_dirty = false;
  BLE_LOG_DEBUGOUT("[BLE] config %s status=%d\n",
                   elrs_config_is_initialized() ? "ready" : "init",
                   config_status);
  if (config_ready) {
    ble_remote_id_init_from_config();
    ble_remote_id_requested_enabled = elrs_config_get_ble_remote_id();
    ble_remote_id_request_pending = false;
    ble_configure_remote_id_mode(ble_remote_id_requested_enabled);
    ble_publish_response("ready config");
  } else {
    ble_remote_id_requested_enabled = false;
    ble_remote_id_request_pending = false;
    ble_configure_remote_id_mode(false);
    ble_publish_response("err config init");
  }

#if BLE_LOG_VERBOSE
  status = rsi_bt_get_local_device_address(local_addr);
  BLE_LOG_DEBUGOUT("[BLE] get local address status=0x%lX\n", (unsigned long)status);
  if (status == RSI_SUCCESS) {
    rsi_6byte_dev_address_to_ascii(local_addr_text, local_addr);
    BLE_LOG_DEBUGOUT("[BLE] local address %s\n", local_addr_text);
  }
#endif

  ble_sem = osSemaphoreNew(1, 0, NULL);
  if (ble_sem == NULL) {
    DEBUGOUT("[BLE] failed to create event semaphore\n");
    ble_task_exit();
  }

  status = ble_add_elrs_service();
  if (status != RSI_SUCCESS) {
    DEBUGOUT("[BLE] add ELRS service failed: 0x%lX\n", (unsigned long)status);
    ble_task_exit();
  }

  rsi_ble_gap_register_callbacks(NULL,
                                 ble_on_connect,
                                 ble_on_disconnect,
                                 NULL,
                                 NULL,
                                 NULL,
                                 NULL,
                                 NULL,
                                 NULL,
                                 NULL);
  rsi_ble_gatt_register_callbacks(NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  ble_on_write,
                                  NULL,
                                  NULL,
                                  ble_on_read_req,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL);

  status = rsi_bt_set_local_name((uint8_t *)BLE_PROBE_NAME);
  if (status != RSI_SUCCESS) {
    DEBUGOUT("[BLE] set local name failed status=0x%lX\n", (unsigned long)status);
    ble_task_exit();
  } else {
    BLE_LOG_DEBUGOUT("[BLE] set local name status=0x%lX\n", (unsigned long)status);
  }

  (void)ble_apply_fixed_remote_id_power();
  ble_service_ready = true;

  if (ble_remote_id_requested_enabled) {
    BLE_LOG_DEBUGOUT("[BLE] Remote ID enabled by RX config; initial advertisement is ODID Basic ID id=%s\n",
                     ble_remote_id_get_uas_id());
  }

  status = ble_apply_current_advertisement();
  if (status != RSI_SUCCESS) {
    DEBUGOUT("[BLE] initial advertising failed status=0x%lX\n", (unsigned long)status);
    ble_task_exit();
  } else {
    BLE_LOG_DEBUGOUT("[BLE] initial advertising status=0x%lX\n", (unsigned long)status);
  }

  if (ble_adv_mode == BLE_ADV_MODE_REMOTE_ID_BASIC) {
    DEBUGOUT("[BLE] Advertising Remote ID over BLE AE (fixed 0 dBm setting, id=%s)\n",
             ble_remote_id_get_uas_id());
    BLE_LOG_DEBUGOUT("[BLE] Remote ID AE payload includes ODID service data UUID 0xFFFA\n");
    BLE_LOG_DEBUGOUT("[BLE] AE test payload includes flags/name plus the ODID service data element\n");
    BLE_LOG_DEBUGOUT("[BLE] Use 'rid elrs' to restore normal %s advertising for config-app discovery\n",
                     BLE_PROBE_NAME);
  } else {
    if (ble_config_advertise_when_remote_id_off) {
      DEBUGOUT("[BLE] Advertising as %s\n", BLE_PROBE_NAME);
    } else {
      DEBUGOUT("[BLE] Remote ID standby ready (advertising off)\n");
    }
  }
  BLE_LOG_DEBUGOUT("[BLE] Read 0x%04X for response, write commands to 0x%04X\n",
                   BLE_UUID_ELRS_STATUS,
                   BLE_UUID_ELRS_COMMAND);
  BLE_LOG_DEBUGOUT("[BLE] Commands: help, api, caps, meta, diag, keys, setkeys, ranges, ping, status, jstatus, get [key], set key=value, save, reload, len, chunk n, rid, ae\n");

  while (1) {
    ble_apply_remote_id_request_if_pending();
    ble_restore_elrs_advertisement_if_due();
    ble_update_remote_id_dynamic_advertisement_if_due();

    int32_t event = ble_get_event();
    if (event < 0) {
      (void)osSemaphoreAcquire(ble_sem, 1000U);
      continue;
    }

    switch ((uint32_t)event) {
      case BLE_EVENT_CONNECTED:
        ble_clear_event(BLE_EVENT_CONNECTED);
        is_connected = true;
        connect_count++;
        DEBUGOUT("[BLE] Connected\n");
        ble_publish_helper_response("connected");
        break;

      case BLE_EVENT_DISCONNECTED:
        ble_clear_event(BLE_EVENT_DISCONNECTED);
        (void)ble_disconnect;
        is_connected = false;
        status_notifications_enabled = false;
        command_notifications_enabled = false;
        disconnect_count++;
        DEBUGOUT("[BLE] Disconnected; restarting advertising\n");
        ble_publish_helper_response("ready");
        status = ble_apply_current_advertisement();
        if (status != RSI_SUCCESS) {
          DEBUGOUT("[BLE] restart advertising failed status=0x%lX\n", (unsigned long)status);
        } else {
          BLE_LOG_DEBUGOUT("[BLE] restart advertising status=0x%lX\n", (unsigned long)status);
        }
        break;

      case BLE_EVENT_WRITE: {
        ble_clear_event(BLE_EVENT_WRITE);
        uint16_t handle = (uint16_t)(ble_write.handle[0] | (ble_write.handle[1] << 8));
        uint16_t len = ble_write.length;
        if (len > sizeof(command_value)) {
          len = sizeof(command_value);
        }

        ble_note_peer(ble_write.dev_addr);
        write_count++;
        last_write_handle = handle;
        BLE_LOG_DEBUGOUT("[BLE] Write handle=0x%04X len=%u\n",
                         (unsigned int)handle,
                         (unsigned int)ble_write.length);

        if (handle == command_handle) {
          memset(command_value, 0, sizeof(command_value));
          memcpy(command_value, ble_write.att_value, len);
          command_value_len = len;
          ble_handle_command(command_value, command_value_len);
        } else if ((handle == status_cccd_handle ||
                    handle == command_cccd_handle) &&
                   ble_write.length >= 2U) {
          uint16_t cccd = (uint16_t)(ble_write.att_value[0] |
                                     (ble_write.att_value[1] << 8));
          bool enabled = ((cccd & 0x0001U) != 0U);
          if (handle == status_cccd_handle) {
            status_notifications_enabled = enabled;
          } else {
            command_notifications_enabled = enabled;
          }
          BLE_LOG_DEBUGOUT("[BLE] Notifications %s on handle=0x%04X\n",
                           enabled ? "enabled" : "disabled",
                           (unsigned int)handle);
          if (enabled) {
            ble_notify_response("current");
          }
        }

        if (ble_write.pkt_type == RSI_BLE_WRITE_REQUEST_EVENT) {
          int32_t write_rsp = rsi_ble_gatt_write_response(ble_write.dev_addr, 0);
          if (write_rsp != RSI_SUCCESS) {
            DEBUGOUT("[BLE] Write response failed status=0x%lX\n", (unsigned long)write_rsp);
          } else {
            BLE_LOG_DEBUGOUT("[BLE] Write response status=0x%lX\n", (unsigned long)write_rsp);
          }
        }
        break;
      }

      case BLE_EVENT_READ: {
        ble_clear_event(BLE_EVENT_READ);
        ble_note_peer(ble_read.dev_addr);
        read_count++;
        last_read_handle = ble_read.handle;
        BLE_LOG_DEBUGOUT("[BLE] Read request handle=0x%04X type=%u offset=%u\n",
                         (unsigned int)ble_read.handle,
                         (unsigned int)ble_read.type,
                         (unsigned int)ble_read.offset);

        if (ble_read.handle == status_handle) {
          ble_send_read_response(&ble_read, status_value, status_value_len);
        } else if (ble_read.handle == command_handle) {
          ble_send_read_response(&ble_read, status_value, status_value_len);
        } else if (ble_read.handle == status_cccd_handle ||
                   ble_read.handle == command_cccd_handle) {
          bool enabled = (ble_read.handle == status_cccd_handle)
                             ? status_notifications_enabled
                             : command_notifications_enabled;
          uint8_t cccd[2] = {
              enabled ? 0x01U : 0x00U,
              0x00U,
          };
          ble_send_read_response(&ble_read, cccd, sizeof(cccd));
        } else {
          static const uint8_t empty[] = { 0 };
          ble_send_read_response(&ble_read, empty, 0);
        }
        break;
      }

      default:
        ble_clear_event((uint32_t)event);
        break;
    }
  }
}

bool ble_gatt_config_api_is_running(void)
{
  return ble_advertising;
}

bool ble_remote_id_service_is_ready(void)
{
  return ble_service_ready;
}

static int ble_start_task(bool advertise_config_when_remote_id_off)
{
  static const osThreadAttr_t ble_task_attributes = {
      .name = "ble_cfg",
      .attr_bits = 0,
      .cb_mem = 0,
      .cb_size = 0,
      .stack_mem = 0,
      .stack_size = 4096,
      .priority = osPriorityLow,
      .tz_module = 0,
      .reserved = 0,
  };

  ble_config_advertise_when_remote_id_off = advertise_config_when_remote_id_off;

  if (ble_task_started) {
    BLE_LOG_DEBUGOUT("[BLE] Config API task already requested\n");
    ble_remote_id_request_pending = true;
    if (ble_sem != NULL) {
      osSemaphoreRelease(ble_sem);
    }
    return 0;
  }

  osThreadId_t task = osThreadNew(ble_task, NULL, &ble_task_attributes);
  if (task == NULL) {
    DEBUGOUT("[BLE] Failed to create BLE config API task\n");
    return -1;
  }

  ble_task_started = true;
  BLE_LOG_DEBUGOUT("[BLE] BLE config API task created\n");
  return 0;
}

int ble_gatt_config_api_start(void)
{
  return ble_start_task(true);
}

int ble_remote_id_service_prepare(void)
{
  return ble_start_task(false);
}

int ble_remote_id_service_start(bool enabled)
{
  ble_remote_id_service_set_enabled(enabled);
  if (!enabled && !ble_task_started) {
    return 0;
  }
  return ble_start_task(false);
}

void ble_gatt_probe_start_task(void)
{
  (void)ble_gatt_config_api_start();
}
