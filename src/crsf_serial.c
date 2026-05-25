/**
 * @file crsf_serial.c
 * @brief CRSF Serial Output Implementation for SiW917
 *
 * Implements CRSF protocol output to flight controllers using the generated
 * CMSIS USART0 driver configuration already present in this project.
 *
 * Citation: TBS CRSF Protocol Specification
 * Citation: ExpressLRS src/lib/CrsfProtocol/crsf_protocol.h
 */

#include "crsf_serial.h"
#include "Driver_USART.h"
#include "RTE_Device_917.h"
#include "rsi_debug.h"
#include <string.h>

/*******************************************************************************
 * Module State
 ******************************************************************************/

static bool g_initialized = false;
static uint32_t g_tx_count = 0;
static volatile bool g_tx_in_progress = false;
static uint8_t g_tx_buffer[CRSF_SERIAL_MAX_FRAME_SIZE];
static ARM_DRIVER_USART *g_usart = NULL;

#define CRSF_SERIAL_RX_RING_SIZE 1024U
#define CRSF_SERIAL_RX_RING_MASK (CRSF_SERIAL_RX_RING_SIZE - 1U)

static uint8_t g_rx_ring[CRSF_SERIAL_RX_RING_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;
static volatile bool g_rx_enabled = false;
static volatile bool g_rx_armed = false;
static volatile uint32_t g_rx_overrun_count = 0;
static uint8_t g_rx_byte = 0;

/* CRC lookup table (polynomial 0xD5) */
static uint8_t crc8_table[256];
static bool crc_table_initialized = false;

extern ARM_DRIVER_USART Driver_USART0;

/*******************************************************************************
 * Debug Output
 ******************************************************************************/

#ifndef DEBUGOUT
#define DEBUGOUT printf
#endif

#define CRSF_DBG(fmt, ...) DEBUGOUT("[CRSF] " fmt, ##__VA_ARGS__)

#ifndef ARM_USART_EVENT_RX_TIMEOUT
#define ARM_USART_EVENT_RX_TIMEOUT 0U
#endif
#ifndef ARM_USART_EVENT_RX_OVERFLOW
#define ARM_USART_EVENT_RX_OVERFLOW 0U
#endif

/*******************************************************************************
 * CRC Calculation
 ******************************************************************************/

static void init_crc8_table(void)
{
    if (crc_table_initialized) return;
    
    for (uint16_t i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? CRSF_CRC_POLY : 0);
        }
        crc8_table[i] = crc;
    }
    crc_table_initialized = true;
}

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc = crc8_table[crc ^ *data++];
    }
    return crc;
}

/*******************************************************************************
 * RX Ring Buffer
 ******************************************************************************/

static void rx_ring_reset(void)
{
    g_rx_head = 0;
    g_rx_tail = 0;
    g_rx_overrun_count = 0;
}

static void rx_ring_push_from_isr(uint8_t byte)
{
    uint16_t next = (uint16_t)((g_rx_head + 1U) & CRSF_SERIAL_RX_RING_MASK);

    if (next == g_rx_tail) {
        g_rx_overrun_count++;
        g_rx_tail = (uint16_t)((g_rx_tail + 1U) & CRSF_SERIAL_RX_RING_MASK);
    }

    g_rx_ring[g_rx_head] = byte;
    g_rx_head = next;
}

static void rx_arm_receive_from_isr(void)
{
    if (!g_rx_enabled || g_usart == NULL) {
        g_rx_armed = false;
        return;
    }

    g_rx_armed = (g_usart->Receive(&g_rx_byte, 1) == ARM_DRIVER_OK);
}

/*******************************************************************************
 * USART Callback (required by driver)
 ******************************************************************************/

static void usart_callback(uint32_t event)
{
    if (event & (ARM_USART_EVENT_SEND_COMPLETE | ARM_USART_EVENT_TX_COMPLETE)) {
        g_tx_in_progress = false;
    }

    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
        g_rx_armed = false;
        rx_ring_push_from_isr(g_rx_byte);
        rx_arm_receive_from_isr();
    }

    if (event & (ARM_USART_EVENT_RX_OVERFLOW | ARM_USART_EVENT_RX_TIMEOUT)) {
        g_rx_overrun_count++;
        g_rx_armed = false;
        if (g_usart != NULL) {
            (void)g_usart->Control(ARM_USART_ABORT_RECEIVE, 0);
        }
        rx_arm_receive_from_isr();
    }
}

static int wait_for_tx_idle(void)
{
    ARM_USART_STATUS status = g_usart->GetStatus();

    /* Recover if the completion callback lagged but hardware is already idle. */
    if (!status.tx_busy) {
        g_tx_in_progress = false;
    }

    /*
     * Do not spin here. ELRS RX timing is more important than a CRSF output
     * frame, and a busy-wait in the FreeRTOS task can starve hwTimer::service()
     * long enough to drop Tick/Tock events.
     */
    return (!g_tx_in_progress && !status.tx_busy) ? 0 : -1;
}

static int transmit_frame(const uint8_t *frame, uint32_t frame_len)
{
    if ((frame == NULL) || (frame_len == 0) || (frame_len > sizeof(g_tx_buffer))) {
        return -1;
    }

    if (wait_for_tx_idle() != 0) {
        return -2;
    }

    memcpy(g_tx_buffer, frame, frame_len);
    g_tx_in_progress = true;

    if (g_usart->Send(g_tx_buffer, frame_len) != ARM_DRIVER_OK) {
        g_tx_in_progress = false;
        return -3;
    }

    return 0;
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

int crsf_serial_init(uint32_t baud_rate)
{
    if (g_initialized) {
        CRSF_DBG("Already initialized\n");
        return 0;
    }
    
    /* Initialize CRC table */
    init_crc8_table();
    
    /* Use default baud rate if not specified */
    if (baud_rate == 0) {
        baud_rate = CRSF_SERIAL_BAUDRATE_DEFAULT;
    }

    g_usart = &Driver_USART0;

    CRSF_DBG("Init USART0 on CLK GPIO_%d, TX GPIO_%d, RX GPIO_%d\n",
             RTE_USART0_CLK_PIN, RTE_USART0_TX_PIN, RTE_USART0_RX_PIN);

    if (g_usart->Initialize(usart_callback) != ARM_DRIVER_OK) {
        CRSF_DBG("USART init failed\n");
        g_usart = NULL;
        return -1;
    }

    if (g_usart->PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK) {
        CRSF_DBG("USART power-up failed\n");
        g_usart->Uninitialize();
        g_usart = NULL;
        return -2;
    }

    if (g_usart->Control(ARM_USART_MODE_ASYNCHRONOUS |
                             ARM_USART_DATA_BITS_8 |
                             ARM_USART_PARITY_NONE |
                             ARM_USART_STOP_BITS_1 |
                             ARM_USART_FLOW_CONTROL_NONE,
                         baud_rate) != ARM_DRIVER_OK) {
        CRSF_DBG("USART config failed\n");
        g_usart->PowerControl(ARM_POWER_OFF);
        g_usart->Uninitialize();
        g_usart = NULL;
        return -3;
    }

    /*
     * RX is enabled only by protocols that need an FC-to-radio byte stream
     * (currently MAVLink). Normal CRSF timing tests stay TX-only.
     */
    if (g_usart->Control(ARM_USART_CONTROL_TX, 1) != ARM_DRIVER_OK) {
        CRSF_DBG("USART TX enable failed\n");
        g_usart->PowerControl(ARM_POWER_OFF);
        g_usart->Uninitialize();
        g_usart = NULL;
        return -4;
    }

    g_tx_in_progress = false;
    g_rx_enabled = false;
    g_rx_armed = false;
    rx_ring_reset();
    CRSF_DBG("Initialized at %lu baud (USART0)\n", (unsigned long)baud_rate);
    
    g_initialized = true;
    g_tx_count = 0;
    return 0;
}

void crsf_serial_deinit(void)
{
    if (!g_initialized) return;

    if (g_usart != NULL) {
        (void)g_usart->Control(ARM_USART_ABORT_SEND, 0);
        (void)g_usart->Control(ARM_USART_ABORT_RECEIVE, 0);
        (void)g_usart->Control(ARM_USART_CONTROL_RX, 0);
        (void)g_usart->PowerControl(ARM_POWER_OFF);
        (void)g_usart->Uninitialize();
        g_usart = NULL;
    }
    
    g_initialized = false;
    g_tx_in_progress = false;
    g_rx_enabled = false;
    g_rx_armed = false;
    rx_ring_reset();
    CRSF_DBG("Deinitialized\n");
}

bool crsf_serial_is_ready(void)
{
    return g_initialized;
}

int crsf_serial_send_channels(const uint32_t *channels)
{
    if (!g_initialized || channels == NULL) {
        return -1;
    }
    
    /*
     * CRSF RC Channels Packed Frame Format:
     *   [0]    Sync byte (0xC8)
     *   [1]    Frame length (24 = payload + type + CRC)
     *   [2]    Frame type (0x16)
     *   [3-24] 16 channels packed as 11-bit values (22 bytes)
     *   [25]   CRC8
     */
    uint8_t frame[26];
    
    /* Header */
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 24;  /* Length: type + 22 bytes channels + CRC */
    frame[2] = CRSF_FRAMETYPE_RC;
    
    /* Pack 16 channels (11-bit each) into 22 bytes
     * Citation: CRSF spec - channels packed little-endian bit-wise
     */
    uint8_t *payload = &frame[3];
    memset(payload, 0, 22);
    
    uint32_t bits = 0;
    uint8_t bitsAvail = 0;
    uint8_t byteIdx = 0;
    
    for (int i = 0; i < CRSF_SERIAL_NUM_CHANNELS; i++) {
        /* Clamp channel value */
        uint32_t ch = channels[i];
        if (ch < CRSF_SERIAL_CHANNEL_MIN) ch = CRSF_SERIAL_CHANNEL_MIN;
        if (ch > CRSF_SERIAL_CHANNEL_MAX) ch = CRSF_SERIAL_CHANNEL_MAX;
        
        /* Add 11 bits to accumulator */
        bits |= (ch & 0x7FF) << bitsAvail;
        bitsAvail += 11;
        
        /* Output complete bytes */
        while (bitsAvail >= 8 && byteIdx < 22) {
            payload[byteIdx++] = bits & 0xFF;
            bits >>= 8;
            bitsAvail -= 8;
        }
    }
    
    /* CRC over type + payload (bytes 2-24) */
    frame[25] = crsf_crc8(&frame[2], 23);
    
    if (transmit_frame(frame, sizeof(frame)) != 0) {
        return -2;
    }
    
    g_tx_count++;
    return 0;
}

int crsf_serial_send_link_stats(const crsf_link_stats_t *stats)
{
    if (!g_initialized || stats == NULL) {
        return -1;
    }
    
    /*
     * CRSF Link Statistics Frame Format:
     *   [0]    Sync byte (0xC8)
     *   [1]    Frame length (12 = 10 payload + type + CRC)
     *   [2]    Frame type (0x14)
     *   [3-12] 10 bytes of link stats
     *   [13]   CRC8
     */
    uint8_t frame[14];
    
    /* Header */
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 12;
    frame[2] = CRSF_FRAMETYPE_LINK;
    
    /* Payload - matches crsfLinkStatistics_t exactly */
    frame[3]  = stats->uplink_rssi_1;
    frame[4]  = stats->uplink_rssi_2;
    frame[5]  = stats->uplink_lq;
    frame[6]  = stats->uplink_snr;
    frame[7]  = stats->active_antenna;
    frame[8]  = stats->rf_mode;
    frame[9]  = stats->uplink_tx_power;
    frame[10] = stats->downlink_rssi;
    frame[11] = stats->downlink_lq;
    frame[12] = stats->downlink_snr;
    
    /* CRC over type + payload */
    frame[13] = crsf_crc8(&frame[2], 11);
    
    if (transmit_frame(frame, sizeof(frame)) != 0) {
        return -2;
    }
    
    g_tx_count++;
    return 0;
}

uint32_t crsf_serial_get_tx_count(void)
{
    return g_tx_count;
}

int crsf_serial_send_frame(const uint8_t *frame, uint32_t frame_len)
{
    if (!g_initialized || frame == NULL) {
        return -1;
    }

    if (transmit_frame(frame, frame_len) != 0) {
        return -2;
    }

    g_tx_count++;
    return 0;
}

int crsf_serial_set_rx_enabled(bool enable)
{
    if (!g_initialized || g_usart == NULL) {
        return -1;
    }

    if (!enable) {
        g_rx_enabled = false;
        g_rx_armed = false;
        (void)g_usart->Control(ARM_USART_ABORT_RECEIVE, 0);
        (void)g_usart->Control(ARM_USART_CONTROL_RX, 0);
        rx_ring_reset();
        return 0;
    }

    if (g_rx_enabled) {
        return 0;
    }

    rx_ring_reset();
    if (g_usart->Control(ARM_USART_CONTROL_RX, 1) != ARM_DRIVER_OK) {
        CRSF_DBG("USART RX enable failed\n");
        return -2;
    }

    g_rx_enabled = true;
    rx_arm_receive_from_isr();
    if (!g_rx_armed) {
        CRSF_DBG("USART RX arm failed\n");
        g_rx_enabled = false;
        (void)g_usart->Control(ARM_USART_CONTROL_RX, 0);
        return -3;
    }

    CRSF_DBG("RX enabled\n");
    return 0;
}

uint32_t crsf_serial_rx_available(void)
{
    const uint16_t head = g_rx_head;
    const uint16_t tail = g_rx_tail;
    return (uint32_t)((head - tail) & CRSF_SERIAL_RX_RING_MASK);
}

uint32_t crsf_serial_read(uint8_t *out, uint32_t max_len)
{
    if (out == NULL || max_len == 0) {
        return 0;
    }

    uint32_t copied = 0;
    while (copied < max_len && g_rx_tail != g_rx_head) {
        out[copied++] = g_rx_ring[g_rx_tail];
        g_rx_tail = (uint16_t)((g_rx_tail + 1U) & CRSF_SERIAL_RX_RING_MASK);
    }

    return copied;
}

uint32_t crsf_serial_get_rx_overrun_count(void)
{
    return g_rx_overrun_count;
}
