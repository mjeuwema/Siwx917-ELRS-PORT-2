/**
 * @file crsf_serial.c
 * @brief CRSF Serial Output Implementation for SiW917
 *
 * Implements CRSF protocol output to flight controllers using the generated
 * CMSIS UART driver configuration already present in this project.
 *
 * Citation: TBS CRSF Protocol Specification
 * Citation: ExpressLRS src/lib/CrsfProtocol/crsf_protocol.h
 */

#include "crsf_serial.h"
#include "Driver_USART.h"
#include "RTE_Device_917.h"
#include "cmsis_gcc.h"
#include "rsi_egpio.h"
#include "rsi_debug.h"
#include "rsi_rom_egpio.h"
#include "rsi_udma.h"
#include <string.h>

/*******************************************************************************
 * Module State
 ******************************************************************************/

static bool g_initialized = false;
static uint32_t g_baud_rate = 0;
static crsf_serial_format_t g_serial_format = CRSF_SERIAL_FORMAT_8N1;
static uint32_t g_tx_count = 0;
#if CRSF_SERIAL_DIAG_LOGS
static uint32_t g_tx_diag_count = 0;
#endif
static uint32_t g_tx_stuck_recover_count = 0;
static uint8_t g_tx_busy_skip_count = 0;
static volatile bool g_tx_in_progress = false;
static ARM_DRIVER_USART *g_usart = NULL;

#if defined(__GNUC__)
#define CRSF_SERIAL_DMA_ALIGN __attribute__((aligned(16)))
#else
#define CRSF_SERIAL_DMA_ALIGN
#endif

static uint8_t g_tx_buffer[CRSF_SERIAL_MAX_FRAME_SIZE] CRSF_SERIAL_DMA_ALIGN;

#define CRSF_SERIAL_RX_RING_SIZE 1024U
#define CRSF_SERIAL_RX_RING_MASK (CRSF_SERIAL_RX_RING_SIZE - 1U)
#define CRSF_SERIAL_RX_DMA_CHUNK_SIZE 64U
#define CRSF_SERIAL_RX_DMA_CHUNK_COUNT 2U
#define CRSF_SERIAL_TX_STUCK_SKIP_LIMIT 8U

/*
 * BRD2708A's DEBUGOUT console uses the native ULP UART route. Keep CRSF on
 * USART0 so FC serial experiments cannot steal the USB debug console.
 */
#define CRSF_SERIAL_USE_ULP_UART 0

#if CRSF_SERIAL_USE_ULP_UART
#define CRSF_SERIAL_DRIVER_NAME "ULP_UART"
#define CRSF_SERIAL_ROUTE_NAME "BRD2708A mikroBUS native ULP UART"
#define CRSF_SERIAL_TX_PIN RTE_ULP_UART_TX_PIN
#define CRSF_SERIAL_TX_MUX RTE_ULP_UART_TX_MUX
#define CRSF_SERIAL_TX_PAD RTE_ULP_UART_TX_PAD
#define CRSF_SERIAL_RX_PIN RTE_ULP_UART_RX_PIN
#define CRSF_SERIAL_RX_MUX RTE_ULP_UART_RX_MUX
#define CRSF_SERIAL_RX_PAD RTE_ULP_UART_RX_PAD
#define CRSF_SERIAL_TX_DMA_CH RTE_ULPUART_CHNL_UDMA_TX_CH
#define CRSF_SERIAL_RX_DMA_CH RTE_ULPUART_CHNL_UDMA_RX_CH
#if defined(SL_ULPUART_DMA_CONFIG_ENABLE) && (SL_ULPUART_DMA_CONFIG_ENABLE == 1)
#define CRSF_SERIAL_DMA_ENABLED 1
#else
#define CRSF_SERIAL_DMA_ENABLED 0
#endif
#else
#define CRSF_SERIAL_DRIVER_NAME "USART0"
#define CRSF_SERIAL_ROUTE_NAME "BRD2708A mikroBUS via USART0 ULP pad route"
#define CRSF_SERIAL_TX_PIN RTE_USART0_TX_PIN
#define CRSF_SERIAL_TX_MUX RTE_USART0_TX_MUX
#define CRSF_SERIAL_TX_PAD RTE_USART0_TX_PAD
#define CRSF_SERIAL_RX_PIN RTE_USART0_RX_PIN
#define CRSF_SERIAL_RX_MUX RTE_USART0_RX_MUX
#define CRSF_SERIAL_RX_PAD RTE_USART0_RX_PAD
#define CRSF_SERIAL_TX_DMA_CH RTE_USART0_CHNL_UDMA_TX_CH
#define CRSF_SERIAL_RX_DMA_CH RTE_USART0_CHNL_UDMA_RX_CH
#if defined(SL_USART0_DMA_CONFIG_ENABLE) && (SL_USART0_DMA_CONFIG_ENABLE == 1)
#define CRSF_SERIAL_DMA_ENABLED 1
#else
#define CRSF_SERIAL_DMA_ENABLED 0
#endif
#endif

#if CRSF_SERIAL_DMA_ENABLED
#define CRSF_SERIAL_DMA_MODE_NAME "UDMA"
#else
#define CRSF_SERIAL_DMA_MODE_NAME "IRQ"
#endif

static uint8_t g_rx_ring[CRSF_SERIAL_RX_RING_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;
static volatile bool g_rx_enabled = false;
static volatile bool g_rx_armed = false;
static volatile uint32_t g_rx_overrun_count = 0;

#if CRSF_SERIAL_DMA_ENABLED
#if CRSF_SERIAL_USE_ULP_UART
extern RSI_UDMA_DESC_T UDMA1_Table[];
#define CRSF_SERIAL_UDMA_TABLE UDMA1_Table
#else
extern RSI_UDMA_DESC_T UDMA0_Table[];
#define CRSF_SERIAL_UDMA_TABLE UDMA0_Table
#endif
static volatile uint8_t g_rx_dma_buffer[CRSF_SERIAL_RX_DMA_CHUNK_COUNT][CRSF_SERIAL_RX_DMA_CHUNK_SIZE] CRSF_SERIAL_DMA_ALIGN;
static volatile uint8_t g_rx_dma_active_index = 0;
static volatile uint8_t g_rx_dma_next_index = 0;
static volatile uint32_t g_rx_dma_active_len = 0;
static volatile uint32_t g_rx_dma_pushed_len = 0;
#else
static uint8_t g_rx_byte = 0;
#endif

/* CRC lookup table (polynomial 0xD5) */
static uint8_t crc8_table[256];
static bool crc_table_initialized = false;

extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_ULP_UART;

#define CRSF_SERIAL_PAD_CONFIG_BASE 0x46004000UL
#define CRSF_SERIAL_PAD_CONFIG_REG(pin_) \
    (*(volatile uint32_t *)(CRSF_SERIAL_PAD_CONFIG_BASE + (4UL * (uint32_t)(pin_))))
#define CRSF_SERIAL_PAD_REN_ENABLE  (1UL << 4)
#define CRSF_SERIAL_PAD_SMT_ENABLE  (1UL << 3)
#define CRSF_SERIAL_PAD_PULL_MASK   (3UL << 6)
#define CRSF_SERIAL_PAD_PULLUP      (1UL << 6)

/*******************************************************************************
 * Debug Output
 ******************************************************************************/

#ifndef DEBUGOUT
#define DEBUGOUT printf
#endif

#define CRSF_DBG(fmt, ...) DEBUGOUT("[CRSF] " fmt, ##__VA_ARGS__)

#ifndef CRSF_SERIAL_DIAG_LOGS
#define CRSF_SERIAL_DIAG_LOGS 0
#endif

#ifndef CRSF_SERIAL_RECOVERY_LOGS
#define CRSF_SERIAL_RECOVERY_LOGS 0
#endif

#ifndef ARM_USART_EVENT_RX_TIMEOUT
#define ARM_USART_EVENT_RX_TIMEOUT 0U
#endif
#ifndef ARM_USART_EVENT_RX_OVERFLOW
#define ARM_USART_EVENT_RX_OVERFLOW 0U
#endif

#if CRSF_SERIAL_DIAG_LOGS
static const char *crsf_serial_pin_domain(uint32_t sdk_pin)
{
    return (sdk_pin >= GPIO_MAX_PIN) ? "ULP_GPIO" : "GPIO";
}

static uint32_t crsf_serial_module_pin(uint32_t sdk_pin)
{
    return (sdk_pin >= GPIO_MAX_PIN) ? (sdk_pin - GPIO_MAX_PIN) : sdk_pin;
}

static void crsf_serial_log_status(const char *stage)
{
    if (g_usart == NULL) {
        return;
    }

    ARM_USART_STATUS status = g_usart->GetStatus();
    CRSF_DBG("%s status: tx_busy=%u rx_busy=%u rx_overflow=%u framing=%u parity=%u\n",
             stage,
             (unsigned)status.tx_busy,
             (unsigned)status.rx_busy,
             (unsigned)status.rx_overflow,
             (unsigned)status.rx_framing_error,
             (unsigned)status.rx_parity_error);
}

static void crsf_serial_log_route(void)
{
    CRSF_DBG("%s route: mikroBUS TX=%s_%lu (SDK GPIO_%lu) -> FC RX\n",
             CRSF_SERIAL_ROUTE_NAME,
             crsf_serial_pin_domain(CRSF_SERIAL_TX_PIN),
             (unsigned long)crsf_serial_module_pin(CRSF_SERIAL_TX_PIN),
             (unsigned long)CRSF_SERIAL_TX_PIN);
    CRSF_DBG("%s route: mikroBUS RX=%s_%lu (SDK GPIO_%lu) <- FC TX\n",
             CRSF_SERIAL_ROUTE_NAME,
             crsf_serial_pin_domain(CRSF_SERIAL_RX_PIN),
             (unsigned long)crsf_serial_module_pin(CRSF_SERIAL_RX_PIN),
             (unsigned long)CRSF_SERIAL_RX_PIN);
#if CRSF_SERIAL_USE_ULP_UART
    CRSF_DBG("%s detail: TX GPIO_%lu mux=%lu pad=%lu; RX GPIO_%lu mux=%lu pad=%lu\n",
             CRSF_SERIAL_DRIVER_NAME,
             (unsigned long)CRSF_SERIAL_TX_PIN,
             (unsigned long)CRSF_SERIAL_TX_MUX,
             (unsigned long)CRSF_SERIAL_TX_PAD,
             (unsigned long)CRSF_SERIAL_RX_PIN,
             (unsigned long)CRSF_SERIAL_RX_MUX,
             (unsigned long)CRSF_SERIAL_RX_PAD);
#else
    CRSF_DBG("%s detail: CLK GPIO_%lu mux=%lu pad=%lu; TX GPIO_%lu mux=%lu pad=%lu; RX GPIO_%lu mux=%lu pad=%lu\n",
             CRSF_SERIAL_DRIVER_NAME,
             (unsigned long)RTE_USART0_CLK_PIN,
             (unsigned long)RTE_USART0_CLK_MUX,
             (unsigned long)RTE_USART0_CLK_PAD,
             (unsigned long)CRSF_SERIAL_TX_PIN,
             (unsigned long)CRSF_SERIAL_TX_MUX,
             (unsigned long)CRSF_SERIAL_TX_PAD,
             (unsigned long)CRSF_SERIAL_RX_PIN,
             (unsigned long)CRSF_SERIAL_RX_MUX,
             (unsigned long)CRSF_SERIAL_RX_PAD);
#endif
}
#else
#define crsf_serial_log_status(stage_) ((void)0)
#define crsf_serial_log_route() ((void)0)
#endif

static void crsf_serial_configure_rx_idle_bias(void)
{
    if (CRSF_SERIAL_RX_PIN >= GPIO_MAX_PIN) {
        const uint8_t ulp_pin = (uint8_t)(CRSF_SERIAL_RX_PIN - GPIO_MAX_PIN);
        RSI_EGPIO_UlpPadDriverDisableState(ulp_pin, ulp_Pullup);
        RSI_EGPIO_UlpPadReceiverEnable(ulp_pin);
#if CRSF_SERIAL_DIAG_LOGS
        CRSF_DBG("RX idle bias: ULP_GPIO_%u pull-up/receiver enabled\n",
                 (unsigned)ulp_pin);
#endif
        return;
    }

    uint32_t rx_pad = CRSF_SERIAL_PAD_CONFIG_REG(CRSF_SERIAL_RX_PIN);
    rx_pad &= ~CRSF_SERIAL_PAD_PULL_MASK;
    rx_pad |= CRSF_SERIAL_PAD_REN_ENABLE | CRSF_SERIAL_PAD_SMT_ENABLE |
              CRSF_SERIAL_PAD_PULLUP;
    CRSF_SERIAL_PAD_CONFIG_REG(CRSF_SERIAL_RX_PIN) = rx_pad;
#if CRSF_SERIAL_DIAG_LOGS
    CRSF_DBG("RX idle bias: GPIO_%lu PAD_CONFIG_REG=0x%08lX\n",
             (unsigned long)CRSF_SERIAL_RX_PIN,
             (unsigned long)CRSF_SERIAL_PAD_CONFIG_REG(CRSF_SERIAL_RX_PIN));
#endif
}

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

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0;

    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static uint16_t map_u16(uint16_t x,
                        uint16_t in_min,
                        uint16_t in_max,
                        uint16_t out_min,
                        uint16_t out_max)
{
    int32_t result = ((int32_t)(x - in_min) * (out_max - out_min) * 2 /
                          (in_max - in_min) +
                      out_min * 2 + 1) /
                     2;
    if (result < 0) {
        return 0;
    }
    if (result > 65535) {
        return 65535;
    }
    return (uint16_t)result;
}

static uint16_t crsf_channel_to_us(uint32_t value)
{
    if (value < CRSF_SERIAL_CHANNEL_MIN) {
        value = CRSF_SERIAL_CHANNEL_MIN;
    }
    if (value > CRSF_SERIAL_CHANNEL_MAX) {
        value = CRSF_SERIAL_CHANNEL_MAX;
    }

    return map_u16((uint16_t)value, CRSF_SERIAL_CHANNEL_MIN,
                   CRSF_SERIAL_CHANNEL_MAX, 988, 2012);
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

static void rx_ring_push_block_from_isr(const volatile uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        rx_ring_push_from_isr(data[i]);
    }
}

static void rx_dma_state_reset(void)
{
#if CRSF_SERIAL_DMA_ENABLED
    g_rx_dma_active_len = 0;
    g_rx_dma_pushed_len = 0;
    g_rx_dma_active_index = 0;
    g_rx_dma_next_index = 0;
#endif
}

#if CRSF_SERIAL_DMA_ENABLED
static uint32_t rx_dma_completed_len(void)
{
    if (!g_rx_armed || g_rx_dma_active_len == 0U) {
        return 0;
    }

    const RSI_UDMA_DESC_T *desc = &CRSF_SERIAL_UDMA_TABLE[CRSF_SERIAL_RX_DMA_CH];
    RSI_UDMA_CHA_CONFIG_DATA_T control = desc->vsUDMAChaConfigData1;

    if (control.transferType == UDMA_MODE_STOP) {
        return g_rx_dma_active_len;
    }

    uint32_t remaining = control.totalNumOfDMATrans + 1U;
    if (remaining > g_rx_dma_active_len) {
        remaining = g_rx_dma_active_len;
    }

    return g_rx_dma_active_len - remaining;
}

static void rx_dma_harvest_to_ring_unlocked(void)
{
    uint32_t completed = rx_dma_completed_len();
    uint32_t pushed = g_rx_dma_pushed_len;

    if (completed > g_rx_dma_active_len) {
        completed = g_rx_dma_active_len;
    }
    if (pushed > completed) {
        return;
    }
    if (completed == pushed) {
        return;
    }

    const uint8_t index = g_rx_dma_active_index;
    rx_ring_push_block_from_isr(&g_rx_dma_buffer[index][pushed], completed - pushed);
    g_rx_dma_pushed_len = completed;
}

static void rx_dma_harvest_to_ring(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rx_dma_harvest_to_ring_unlocked();
    if (primask == 0U) {
        __enable_irq();
    }
}
#endif

static void rx_arm_receive_from_isr(void)
{
    if (!g_rx_enabled || g_usart == NULL) {
        g_rx_armed = false;
        return;
    }

#if CRSF_SERIAL_DMA_ENABLED
    const uint8_t next = g_rx_dma_next_index;
    g_rx_dma_active_index = next;
    g_rx_dma_next_index = (uint8_t)((next + 1U) % CRSF_SERIAL_RX_DMA_CHUNK_COUNT);
    g_rx_dma_active_len = CRSF_SERIAL_RX_DMA_CHUNK_SIZE;
    g_rx_dma_pushed_len = 0;
    g_rx_armed = (g_usart->Receive((void *)g_rx_dma_buffer[next], CRSF_SERIAL_RX_DMA_CHUNK_SIZE) == ARM_DRIVER_OK);
    if (!g_rx_armed) {
        g_rx_dma_active_len = 0;
        g_rx_dma_pushed_len = 0;
    }
#else
    g_rx_armed = (g_usart->Receive(&g_rx_byte, 1) == ARM_DRIVER_OK);
#endif
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
#if CRSF_SERIAL_DMA_ENABLED
        rx_dma_harvest_to_ring_unlocked();
        if (g_rx_dma_pushed_len < g_rx_dma_active_len) {
            const uint8_t index = g_rx_dma_active_index;
            rx_ring_push_block_from_isr(&g_rx_dma_buffer[index][g_rx_dma_pushed_len],
                                        g_rx_dma_active_len - g_rx_dma_pushed_len);
        }
#else
        rx_ring_push_from_isr(g_rx_byte);
#endif
        g_rx_armed = false;
        rx_arm_receive_from_isr();
    }

    if (event & (ARM_USART_EVENT_RX_OVERFLOW | ARM_USART_EVENT_RX_TIMEOUT)) {
#if CRSF_SERIAL_DMA_ENABLED
        rx_dma_harvest_to_ring_unlocked();
#endif
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

    /*
     * The Si91x USART driver can report tx_busy immediately after TX enable,
     * before this module has queued any bytes. Do not let that stale hardware
     * bit starve the first CRSF frame forever.
     */
    if (!g_tx_in_progress) {
        g_tx_busy_skip_count = 0;
        return 0;
    }

    /* Recover if the completion callback lagged but hardware is already idle. */
    if (!status.tx_busy) {
        g_tx_in_progress = false;
        g_tx_busy_skip_count = 0;
        return 0;
    }

    if (++g_tx_busy_skip_count >= CRSF_SERIAL_TX_STUCK_SKIP_LIMIT) {
        (void)g_usart->Control(ARM_USART_ABORT_SEND, 0);
        g_tx_in_progress = false;
        g_tx_busy_skip_count = 0;
        g_tx_stuck_recover_count++;
#if CRSF_SERIAL_RECOVERY_LOGS
        if (g_tx_stuck_recover_count <= 4U) {
            CRSF_DBG("TX busy recovery #%lu\n",
                     (unsigned long)g_tx_stuck_recover_count);
        }
#endif
        return 0;
    }

    /*
     * Do not spin here. ELRS RX timing is more important than a CRSF output
     * frame, and a busy-wait in the FreeRTOS task can starve hwTimer::service()
     * long enough to drop Tick/Tock events.
     */
    return -1;
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
    g_tx_busy_skip_count = 0;

    if (g_usart->Send(g_tx_buffer, frame_len) != ARM_DRIVER_OK) {
        g_tx_in_progress = false;
        return -3;
    }

#if CRSF_SERIAL_DIAG_LOGS
    if (g_tx_diag_count < 8U) {
        const uint8_t addr = (frame_len > 0U) ? frame[0] : 0U;
        const uint8_t type = (frame_len > 2U) ? frame[2] : 0U;
        CRSF_DBG("TX frame OK #%lu len=%lu addr=0x%02X type=0x%02X\n",
                 (unsigned long)(g_tx_diag_count + 1U),
                 (unsigned long)frame_len,
                 addr,
                 type);
        g_tx_diag_count++;
    }
#endif

    return 0;
}

static int configure_usart(uint32_t baud_rate, crsf_serial_format_t format)
{
    uint32_t control = ARM_USART_MODE_ASYNCHRONOUS |
                       ARM_USART_DATA_BITS_8 |
                       ARM_USART_FLOW_CONTROL_NONE;

    if (format == CRSF_SERIAL_FORMAT_8E2) {
        control |= ARM_USART_PARITY_EVEN | ARM_USART_STOP_BITS_2;
    } else {
        control |= ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1;
    }

    if (g_usart->Control(control,
                         baud_rate) != ARM_DRIVER_OK) {
        CRSF_DBG("USART config failed\n");
        return -1;
    }

    if (g_usart->Control(ARM_USART_CONTROL_TX, 1) != ARM_DRIVER_OK) {
        CRSF_DBG("USART TX enable failed\n");
        return -2;
    }
    crsf_serial_log_status("After TX enable");

    /* UART RX idles high. Keep the configured RX pad pulled up so an
     * unplugged FC does not feed continuous noise into the DMA receiver.
     */
    crsf_serial_configure_rx_idle_bias();

    g_baud_rate = baud_rate;
    g_serial_format = format;
    return 0;
}

static int reconfigure_usart(uint32_t baud_rate, crsf_serial_format_t format)
{
    if (g_usart == NULL) {
        return -1;
    }

    g_rx_enabled = false;
    g_rx_armed = false;
    (void)g_usart->Control(ARM_USART_ABORT_RECEIVE, 0);
    (void)g_usart->Control(ARM_USART_CONTROL_RX, 0);
    (void)g_usart->Control(ARM_USART_ABORT_SEND, 0);

    g_tx_in_progress = false;
    rx_dma_state_reset();
    rx_ring_reset();

    if (configure_usart(baud_rate, format) != 0) {
        return -2;
    }

    CRSF_DBG("Reconfigured %s to %lu baud (%s)\n",
             CRSF_SERIAL_DRIVER_NAME,
             (unsigned long)baud_rate,
             format == CRSF_SERIAL_FORMAT_8E2 ? "8E2" : "8N1");
    return 0;
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

int crsf_serial_init(uint32_t baud_rate)
{
    return crsf_serial_init_ex(baud_rate, CRSF_SERIAL_FORMAT_8N1);
}

int crsf_serial_init_ex(uint32_t baud_rate, crsf_serial_format_t format)
{
    /* Use default baud rate if not specified */
    if (baud_rate == 0) {
        baud_rate = CRSF_SERIAL_BAUDRATE_DEFAULT;
    }

    if (g_initialized) {
        if (g_baud_rate == baud_rate && g_serial_format == format) {
            return 0;
        }
        return reconfigure_usart(baud_rate, format);
    }
    
    /* Initialize CRC table */
    init_crc8_table();

    g_usart = CRSF_SERIAL_USE_ULP_UART ? &Driver_ULP_UART : &Driver_USART0;

    CRSF_DBG("Init %s on TX GPIO_%lu, RX GPIO_%lu\n",
             CRSF_SERIAL_DRIVER_NAME,
             (unsigned long)CRSF_SERIAL_TX_PIN,
             (unsigned long)CRSF_SERIAL_RX_PIN);
    crsf_serial_log_route();

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

    if (configure_usart(baud_rate, format) != 0) {
        g_usart->PowerControl(ARM_POWER_OFF);
        g_usart->Uninitialize();
        g_usart = NULL;
        return -3;
    }

    /*
     * RX is enabled later by the active serial protocol. CRSF uses it for
     * FC telemetry passthrough; MAVLink uses it for byte-stream OTA transport.
     */
    g_tx_in_progress = false;
    g_rx_enabled = false;
    g_rx_armed = false;
    rx_dma_state_reset();
    rx_ring_reset();
#if CRSF_SERIAL_DMA_ENABLED
    CRSF_DBG("Initialized at %lu baud %s (%s %s TX ch %u RX ch %u)\n",
             (unsigned long)baud_rate,
             format == CRSF_SERIAL_FORMAT_8E2 ? "8E2" : "8N1",
             CRSF_SERIAL_DRIVER_NAME,
             CRSF_SERIAL_DMA_MODE_NAME,
             (unsigned)CRSF_SERIAL_TX_DMA_CH,
             (unsigned)CRSF_SERIAL_RX_DMA_CH);
#else
    CRSF_DBG("Initialized at %lu baud %s (%s IRQ RX/TX)\n",
             (unsigned long)baud_rate,
             format == CRSF_SERIAL_FORMAT_8E2 ? "8E2" : "8N1",
             CRSF_SERIAL_DRIVER_NAME);
#endif
    
    g_initialized = true;
    g_tx_count = 0;
#if CRSF_SERIAL_DIAG_LOGS
    g_tx_diag_count = 0;
#endif
    return 0;
}

void crsf_serial_deinit(void)
{
    if (!g_initialized) return;

    if (g_usart != NULL) {
        g_rx_enabled = false;
        g_rx_armed = false;
        (void)g_usart->Control(ARM_USART_ABORT_SEND, 0);
        (void)g_usart->Control(ARM_USART_ABORT_RECEIVE, 0);
        (void)g_usart->Control(ARM_USART_CONTROL_RX, 0);
        (void)g_usart->PowerControl(ARM_POWER_OFF);
        (void)g_usart->Uninitialize();
        g_usart = NULL;
    }
    
    g_initialized = false;
    g_baud_rate = 0;
    g_tx_in_progress = false;
    g_rx_enabled = false;
    g_rx_armed = false;
    rx_dma_state_reset();
    rx_ring_reset();
    CRSF_DBG("Deinitialized\n");
}

static void pack_16x11_channels(uint8_t *payload, uint32_t payload_len,
                                const uint32_t *channels)
{
    uint32_t bits = 0;
    uint8_t bitsAvail = 0;
    uint32_t byteIdx = 0;

    memset(payload, 0, payload_len);

    for (int i = 0; i < CRSF_SERIAL_NUM_CHANNELS; i++) {
        uint32_t ch = channels[i];
        if (ch < CRSF_SERIAL_CHANNEL_MIN) ch = CRSF_SERIAL_CHANNEL_MIN;
        if (ch > CRSF_SERIAL_CHANNEL_MAX) ch = CRSF_SERIAL_CHANNEL_MAX;

        bits |= (ch & 0x7FFU) << bitsAvail;
        bitsAvail += 11U;

        while (bitsAvail >= 8U && byteIdx < payload_len) {
            payload[byteIdx++] = (uint8_t)(bits & 0xFFU);
            bits >>= 8U;
            bitsAvail -= 8U;
        }
    }
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
    pack_16x11_channels(payload, 22, channels);
    
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

int crsf_serial_send_sbus_channels(const uint32_t *channels,
                                   bool failsafe_active,
                                   bool frame_lost)
{
    if (!g_initialized || channels == NULL) {
        return -1;
    }

    uint8_t frame[25];
    frame[0] = 0x0F;
    pack_16x11_channels(&frame[1], 22, channels);
    frame[23] = (frame_lost ? (1U << 2) : 0U) |
                (failsafe_active ? (1U << 3) : 0U);
    frame[24] = 0x00;

    if (transmit_frame(frame, sizeof(frame)) != 0) {
        return -2;
    }

    g_tx_count++;
    return 0;
}

int crsf_serial_send_sumd_channels(const uint32_t *channels)
{
    static const uint8_t sumd_channel_map[CRSF_SERIAL_NUM_CHANNELS] = {
        0, 1, 2, 3, 7, 5, 6, 4, 8, 9, 10, 11, 12, 13, 14, 15,
    };

    if (!g_initialized || channels == NULL) {
        return -1;
    }

    uint8_t frame[37];
    frame[0] = 0xA8;
    frame[1] = 0x01;
    frame[2] = 0x10;

    for (uint8_t i = 0; i < CRSF_SERIAL_NUM_CHANNELS; i++) {
        const uint8_t source = sumd_channel_map[i];
        const uint16_t value = (uint16_t)(crsf_channel_to_us(channels[source]) << 3);
        const uint8_t offset = (uint8_t)(3U + (i * 2U));
        frame[offset] = (uint8_t)(value >> 8);
        frame[offset + 1U] = (uint8_t)(value & 0xFFU);
    }

    const uint16_t crc = crc16_ccitt(frame, 35);
    frame[35] = (uint8_t)(crc >> 8);
    frame[36] = (uint8_t)(crc & 0xFFU);

    if (transmit_frame(frame, sizeof(frame)) != 0) {
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
        rx_dma_state_reset();
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
    crsf_serial_log_status("After RX enable");
    return 0;
}

uint32_t crsf_serial_rx_available(void)
{
#if CRSF_SERIAL_DMA_ENABLED
    rx_dma_harvest_to_ring();
#endif
    const uint16_t head = g_rx_head;
    const uint16_t tail = g_rx_tail;
    return (uint32_t)((head - tail) & CRSF_SERIAL_RX_RING_MASK);
}

uint32_t crsf_serial_read(uint8_t *out, uint32_t max_len)
{
    if (out == NULL || max_len == 0) {
        return 0;
    }

#if CRSF_SERIAL_DMA_ENABLED
    rx_dma_harvest_to_ring();
#endif
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
