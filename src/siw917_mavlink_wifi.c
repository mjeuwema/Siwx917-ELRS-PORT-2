#include "siw917_mavlink_wifi.h"

#include "wifi_http_test.h"

#include "cmsis_os2.h"
#include "em_device.h"
#include "rsi_debug.h"
#include "sl_net.h"
#include "sl_net_wifi_types.h"
#include "sl_si91x_socket.h"
#include "sl_wifi.h"
#include "sl_wifi_callback_framework.h"
#include "socket.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#define MAVLINK_UDP_PORT 14550U
#define MAVLINK_DOWNLINK_SLOT_COUNT 32U
#define MAVLINK_DOWNLINK_SLOT_SIZE 128U
#define MAVLINK_UPLINK_RING_SIZE 4096U
#define MAVLINK_SEND_BUDGET 8U

typedef struct {
  uint16_t length;
  uint8_t data[MAVLINK_DOWNLINK_SLOT_SIZE];
} mavlink_downlink_slot_t;

extern bool device_initialized;

static sl_net_wifi_psk_credential_entry_t mavlink_ap_credential = {
  .type        = SL_NET_WIFI_PSK,
  .data_length = sizeof(WIFI_TEST_AP_PASSWORD) - 1U,
  .data        = WIFI_TEST_AP_PASSWORD,
};

static sl_net_wifi_ap_profile_t mavlink_ap_profile = {
  .config = {
    .ssid.value  = WIFI_TEST_AP_SSID,
    .ssid.length = sizeof(WIFI_TEST_AP_SSID) - 1U,
    .channel = {
      .channel   = WIFI_TEST_AP_CHANNEL,
      .band      = SL_WIFI_BAND_2_4GHZ,
      .bandwidth = SL_WIFI_BANDWIDTH_20MHz,
    },
    .security            = SL_WIFI_WPA2,
    .encryption          = SL_WIFI_CCMP_ENCRYPTION,
    .rate_protocol       = SL_WIFI_RATE_PROTOCOL_AUTO,
    .options             = 0,
    .credential_id       = SL_NET_DEFAULT_WIFI_AP_CREDENTIAL_ID,
    .keepalive_type      = SL_SI91X_AP_NULL_BASED_KEEP_ALIVE,
    .beacon_interval     = 100,
    .client_idle_timeout = 0xFF,
    .dtim_beacon_count   = 3,
    .maximum_clients     = 4,
    .beacon_stop         = 0,
    .tdi_flags           = SL_WIFI_TDI_NONE,
    .is_11n_enabled      = 1,
  },
  .ip = {
    .mode      = SL_IP_MANAGEMENT_STATIC_IP,
    .type      = SL_IPV4,
    .host_name = NULL,
    .ip = {
      .v4.ip_address.value = WIFI_TEST_IP_ADDRESS,
      .v4.gateway.value    = WIFI_TEST_GATEWAY,
      .v4.netmask.value    = WIFI_TEST_SUBNET_MASK,
    },
  },
};

static mavlink_downlink_slot_t downlink_slots[MAVLINK_DOWNLINK_SLOT_COUNT];
static volatile uint32_t downlink_head;
static volatile uint32_t downlink_tail;
static uint8_t uplink_ring[MAVLINK_UPLINK_RING_SIZE];
static volatile uint32_t uplink_head;
static volatile uint32_t uplink_tail;

static volatile bool bridge_enabled;
static volatile bool bridge_running;
static volatile bool bridge_operation_active;
static volatile bool bridge_last_stop_ok = true;
static volatile bool peer_valid;
static volatile bool peer_announce_pending;
static volatile bool peer_disconnect_pending;
static volatile bool bridge_state_change_pending;
static volatile uint32_t wifi_client_count;
static uint8_t peer_ip[4];
static struct sockaddr_in peer_address;
static int udp_socket = -1;
static osThreadId_t bridge_task_id;

static volatile uint32_t downlink_drops;
static volatile uint32_t uplink_drops;
static volatile uint32_t socket_send_failures;

static sl_status_t mavlink_client_connected(sl_wifi_event_t event,
                                            void *data,
                                            uint32_t data_length,
                                            void *arg)
{
  (void)event;
  (void)data;
  (void)data_length;
  (void)arg;

  wifi_client_count++;
  __DMB();
  return SL_STATUS_OK;
}

static sl_status_t mavlink_client_disconnected(sl_wifi_event_t event,
                                               void *data,
                                               uint32_t data_length,
                                               void *arg)
{
  (void)event;
  (void)data;
  (void)data_length;
  (void)arg;

  if (wifi_client_count > 0U) {
    wifi_client_count--;
  }
  if (bridge_running && wifi_client_count == 0U) {
    peer_valid = false;
    __DMB();
    peer_disconnect_pending = true;
  }
  __DMB();
  return SL_STATUS_OK;
}

static void reset_queues(void)
{
  downlink_tail = downlink_head;
  uplink_tail   = uplink_head;
  __DMB();
}

static void remember_peer(const sl_si91x_socket_metadata_t *metadata)
{
  if (metadata == NULL || metadata->ip_version != 4U) {
    return;
  }

  const bool changed =
    !peer_valid || peer_address.sin_port != metadata->dest_port ||
    memcmp(peer_ip, metadata->dest_ip_addr.ipv4_address, sizeof(peer_ip)) != 0;

  struct sockaddr_in next_peer = { 0 };
  next_peer.sin_family = AF_INET;
  next_peer.sin_port   = metadata->dest_port;
  memcpy(&next_peer.sin_addr.s_addr,
         metadata->dest_ip_addr.ipv4_address,
         sizeof(next_peer.sin_addr.s_addr));

  memcpy(peer_ip, metadata->dest_ip_addr.ipv4_address, sizeof(peer_ip));
  peer_address = next_peer;
  __DMB();
  peer_valid = true;
  if (changed) {
    peer_announce_pending = true;
  }
}

static bool enqueue_uplink(const uint8_t *data, size_t length)
{
  if (data == NULL || length == 0U || length > MAVLINK_UPLINK_RING_SIZE) {
    return false;
  }

  const uint32_t head = uplink_head;
  const uint32_t tail = uplink_tail;
  if (length > (MAVLINK_UPLINK_RING_SIZE - (head - tail))) {
    uplink_drops++;
    return false;
  }

  for (size_t index = 0; index < length; ++index) {
    uplink_ring[(head + index) & (MAVLINK_UPLINK_RING_SIZE - 1U)] = data[index];
  }
  __DMB();
  uplink_head = head + (uint32_t)length;
  return true;
}

static void mavlink_socket_receive_callback(
  uint32_t socket,
  uint8_t *buffer,
  uint32_t length,
  const sl_si91x_socket_metadata_t *metadata)
{
  if (!bridge_enabled || !bridge_running || (int)socket != udp_socket) {
    return;
  }

  remember_peer(metadata);
  (void)enqueue_uplink(buffer, length);
}

static bool start_bridge(void)
{
  sl_status_t status;

  if (!device_initialized) {
    status = sl_net_init(SL_NET_WIFI_AP_INTERFACE, NULL, NULL, NULL);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("[MAVWIFI] NWP init failed: 0x%lX\n", (unsigned long)status);
      return false;
    }
  }

  status = sl_net_set_credential(SL_NET_DEFAULT_WIFI_AP_CREDENTIAL_ID,
                                 SL_NET_WIFI_PSK,
                                 mavlink_ap_credential.data,
                                 mavlink_ap_credential.data_length);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("[MAVWIFI] AP credential failed: 0x%lX\n", (unsigned long)status);
    return false;
  }

  status = sl_net_set_profile(SL_NET_WIFI_AP_INTERFACE,
                              SL_NET_DEFAULT_WIFI_AP_PROFILE_ID,
                              &mavlink_ap_profile);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("[MAVWIFI] AP profile failed: 0x%lX\n", (unsigned long)status);
    return false;
  }

  wifi_client_count = 0U;
  peer_disconnect_pending = false;
  status = sl_wifi_set_callback(SL_WIFI_CLIENT_CONNECTED_EVENTS,
                                mavlink_client_connected,
                                NULL);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("[MAVWIFI] AP connect callback failed: 0x%lX\n",
             (unsigned long)status);
    return false;
  }
  status = sl_wifi_set_callback(SL_WIFI_CLIENT_DISCONNECTED_EVENTS,
                                mavlink_client_disconnected,
                                NULL);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("[MAVWIFI] AP disconnect callback failed: 0x%lX\n",
             (unsigned long)status);
    return false;
  }

  status = sl_net_up(SL_NET_WIFI_AP_INTERFACE, SL_NET_DEFAULT_WIFI_AP_PROFILE_ID);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("[MAVWIFI] AP start failed: 0x%lX\n", (unsigned long)status);
    return false;
  }

  udp_socket = sl_si91x_socket_async(AF_INET,
                                     SOCK_DGRAM,
                                     IPPROTO_UDP,
                                     mavlink_socket_receive_callback);
  if (udp_socket < 0) {
    DEBUGOUT("[MAVWIFI] UDP socket failed: errno=%d\n", errno);
    (void)sl_net_down(SL_NET_WIFI_AP_INTERFACE);
    return false;
  }

  struct sockaddr_in local_address = { 0 };
  local_address.sin_family = AF_INET;
  local_address.sin_port   = MAVLINK_UDP_PORT;
  if (sl_si91x_bind(udp_socket,
                    (const struct sockaddr *)&local_address,
                    sizeof(local_address)) < 0) {
    DEBUGOUT("[MAVWIFI] UDP bind failed: errno=%d\n", errno);
    (void)sl_si91x_shutdown(udp_socket, 0);
    udp_socket = -1;
    (void)sl_net_down(SL_NET_WIFI_AP_INTERFACE);
    return false;
  }

  peer_valid    = false;
  bridge_running = true;
  reset_queues();
  DEBUGOUT("[MAVWIFI] Ground-station bridge ready\n");
  DEBUGOUT("[MAVWIFI] SSID=%s password=%s IP=192.168.10.10 UDP=%u\n",
           WIFI_TEST_AP_SSID,
           WIFI_TEST_AP_PASSWORD,
           (unsigned)MAVLINK_UDP_PORT);
  DEBUGOUT("[MAVWIFI] Waiting for the ground station's first UDP packet\n");
  return true;
}

static bool stop_bridge(void)
{
  const int socket_to_close = udp_socket;

  /* Prevent socket callbacks from accepting data before AP teardown starts. */
  peer_valid = false;
  peer_announce_pending = false;
  peer_disconnect_pending = false;
  wifi_client_count = 0U;
  udp_socket = -1;
  __DMB();
  reset_queues();

  if (socket_to_close >= 0) {
    (void)sl_si91x_shutdown(socket_to_close, 0);
  }

  /* Let any disconnect/socket callback already dispatched by the NWP finish. */
  osDelay(10U);

  const sl_status_t status = sl_wifi_stop_ap(SL_WIFI_AP_2_4GHZ_INTERFACE);
  const bool stopped =
    status == SL_STATUS_OK || status == SL_STATUS_WIFI_INTERFACE_NOT_UP;
  bridge_running = false;
  bridge_last_stop_ok = stopped;
  __DMB();
  if (stopped) {
    DEBUGOUT("[MAVWIFI] Bridge Off; WiFi AP stopped, NWP retained\n");
  } else {
    DEBUGOUT("[MAVWIFI] WiFi AP stop failed: 0x%lX\n",
             (unsigned long)status);
  }
  return stopped;
}

static bool pop_downlink(uint8_t *data, size_t *length)
{
  const uint32_t tail = downlink_tail;
  if (tail == downlink_head) {
    return false;
  }

  const mavlink_downlink_slot_t *slot =
    &downlink_slots[tail & (MAVLINK_DOWNLINK_SLOT_COUNT - 1U)];
  *length = slot->length;
  memcpy(data, slot->data, *length);
  __DMB();
  downlink_tail = tail + 1U;
  return true;
}

static void service_downlink(void)
{
  if (!peer_valid) {
    downlink_tail = downlink_head;
    return;
  }

  struct sockaddr_in destination = peer_address;
  uint8_t packet[MAVLINK_DOWNLINK_SLOT_SIZE];
  size_t length;

  for (uint32_t sent = 0; sent < MAVLINK_SEND_BUDGET; ++sent) {
    if (!pop_downlink(packet, &length)) {
      break;
    }

    if (sl_si91x_sendto(udp_socket,
                        packet,
                        length,
                        0,
                        (const struct sockaddr *)&destination,
                        sizeof(destination)) < 0) {
      socket_send_failures++;
      break;
    }
  }
}

static void mavlink_wifi_task(void *argument)
{
  (void)argument;
  uint32_t retry_at = 0U;
  uint32_t last_reported_downlink_drops = 0U;
  uint32_t last_reported_uplink_drops = 0U;
  uint32_t last_reported_send_failures = 0U;
  uint32_t last_stats_report = 0U;

  while (1) {
    const uint32_t now = osKernelGetTickCount();

    if (bridge_state_change_pending) {
      bridge_state_change_pending = false;
      peer_valid = false;
      peer_announce_pending = false;
      reset_queues();
      if (!bridge_enabled && bridge_running) {
        bridge_operation_active = true;
        __DMB();
        (void)stop_bridge();
        bridge_operation_active = false;
        __DMB();
      } else if (!bridge_enabled) {
        bridge_last_stop_ok = true;
      } else if (bridge_enabled && bridge_running) {
        DEBUGOUT("[MAVWIFI] Bridge resumed; waiting for a GCS UDP packet\n");
      }
    }

    if (bridge_running && peer_disconnect_pending) {
      peer_disconnect_pending = false;
      reset_queues();
      DEBUGOUT("[MAVWIFI] Last GCS station disconnected; stale UDP peer cleared\n");
    }

    if (bridge_enabled && !bridge_running && (int32_t)(now - retry_at) >= 0) {
      bridge_operation_active = true;
      __DMB();
      const bool started = start_bridge();
      bridge_operation_active = false;
      __DMB();
      if (!started) {
        retry_at = now + 5000U;
      }
    }

    if (bridge_running && bridge_enabled) {
      if (peer_announce_pending) {
        peer_announce_pending = false;
        DEBUGOUT("[MAVWIFI] GCS peer=%u.%u.%u.%u:%u\n",
                 (unsigned)peer_ip[0],
                 (unsigned)peer_ip[1],
                 (unsigned)peer_ip[2],
                 (unsigned)peer_ip[3],
                 (unsigned)peer_address.sin_port);
      }
      service_downlink();

      if ((now - last_stats_report) >= 10000U &&
          (downlink_drops != last_reported_downlink_drops ||
           uplink_drops != last_reported_uplink_drops ||
           socket_send_failures != last_reported_send_failures)) {
        last_stats_report = now;
        last_reported_downlink_drops = downlink_drops;
        last_reported_uplink_drops = uplink_drops;
        last_reported_send_failures = socket_send_failures;
        DEBUGOUT("[MAVWIFI] queue drops down=%lu up=%lu send_fail=%lu\n",
                 (unsigned long)downlink_drops,
                 (unsigned long)uplink_drops,
                 (unsigned long)socket_send_failures);
      }
    }

    osDelay((bridge_running && bridge_enabled) ? 2U : 50U);
  }
}

bool siw917_mavlink_wifi_init(void)
{
  if (bridge_task_id != NULL) {
    return true;
  }

  static const osThreadAttr_t task_attributes = {
    .name       = "mavlink_wifi",
    .stack_size = 4096,
    .priority   = osPriorityLow,
  };
  bridge_task_id = osThreadNew(mavlink_wifi_task, NULL, &task_attributes);
  if (bridge_task_id == NULL) {
    DEBUGOUT("[MAVWIFI] Failed to create ground-station task\n");
    return false;
  }
  return true;
}

void siw917_mavlink_wifi_set_enabled(bool enabled)
{
  if (bridge_enabled == enabled) {
    return;
  }

  bridge_enabled = enabled;
  if (!enabled) {
    peer_valid = false;
    if (bridge_running || bridge_operation_active) {
      bridge_last_stop_ok = false;
    }
  }
  bridge_state_change_pending = true;
  __DMB();
}

bool siw917_mavlink_wifi_is_enabled(void)
{
  return bridge_enabled;
}

bool siw917_mavlink_wifi_is_running(void)
{
  return bridge_running;
}

bool siw917_mavlink_wifi_wait_stopped(uint32_t timeout_ms)
{
  const uint32_t started_at = osKernelGetTickCount();

  while (1) {
    __DMB();
    if (!bridge_enabled && !bridge_running && !bridge_operation_active &&
        !bridge_state_change_pending && bridge_last_stop_ok) {
      return true;
    }

    if ((uint32_t)(osKernelGetTickCount() - started_at) >= timeout_ms) {
      return false;
    }
    osDelay(2U);
  }
}

bool siw917_mavlink_wifi_enqueue_downlink(const uint8_t *data, size_t length)
{
  if (!bridge_enabled || data == NULL || length == 0U ||
      length > MAVLINK_DOWNLINK_SLOT_SIZE) {
    return false;
  }

  const uint32_t head = downlink_head;
  if ((head - downlink_tail) >= MAVLINK_DOWNLINK_SLOT_COUNT) {
    downlink_drops++;
    return false;
  }

  mavlink_downlink_slot_t *slot =
    &downlink_slots[head & (MAVLINK_DOWNLINK_SLOT_COUNT - 1U)];
  slot->length = (uint16_t)length;
  memcpy(slot->data, data, length);
  __DMB();
  downlink_head = head + 1U;
  return true;
}

size_t siw917_mavlink_wifi_uplink_available(void)
{
  if (!bridge_enabled) {
    return 0U;
  }
  return (size_t)(uplink_head - uplink_tail);
}

int siw917_mavlink_wifi_uplink_read(void)
{
  const uint32_t tail = uplink_tail;
  if (!bridge_enabled || tail == uplink_head) {
    return -1;
  }

  const uint8_t value = uplink_ring[tail & (MAVLINK_UPLINK_RING_SIZE - 1U)];
  __DMB();
  uplink_tail = tail + 1U;
  return value;
}

size_t siw917_mavlink_wifi_uplink_read_bytes(uint8_t *data, size_t length)
{
  if (data == NULL || length == 0U || !bridge_enabled) {
    return 0U;
  }

  size_t available = siw917_mavlink_wifi_uplink_available();
  if (length > available) {
    length = available;
  }

  const uint32_t tail = uplink_tail;
  for (size_t index = 0; index < length; ++index) {
    data[index] = uplink_ring[(tail + index) & (MAVLINK_UPLINK_RING_SIZE - 1U)];
  }
  __DMB();
  uplink_tail = tail + (uint32_t)length;
  return length;
}
