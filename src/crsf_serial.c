/**
 * @file crsf_serial.c
 * @brief CRSF Serial Output Implementation for SiW917
 *
 * Implements the generated CMSIS UART driver path used by both receiver
 * flight-controller CRSF and transmitter handset-module CRSF.
 *
 * Citation: TBS CRSF Protocol Specification
 * Citation: ExpressLRS src/lib/CrsfProtocol/crsf_protocol.h
 */

#include "crsf_serial.h"
#include "Driver_USART.h"
#include "RTE_Device_917.h"
#include "rsi_egpio.h"
#include "rsi_debug.h"
#include "rsi_rom_egpio.h"
#include "rsi_rom_clks.h"
#include "cmsis_os2.h"
#include <string.h>

#if defined(SIW917_ELRS_TARGET_TX)
#include "elrs_cpp/hal/elrs_task_wakeup.h"
extern void delayMicroseconds(uint32_t us);
extern uint32_t SystemCoreClock;
#endif

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

#define CRSF_SERIAL_RX_RING_SIZE 4096U
#define CRSF_SERIAL_RX_RING_MASK (CRSF_SERIAL_RX_RING_SIZE - 1U)
#define CRSF_SERIAL_RX_DMA_CHUNK_SIZE 64U
#define CRSF_SERIAL_RX_DMA_CHUNK_COUNT 2U
#define CRSF_SERIAL_TX_STUCK_SKIP_LIMIT 8U

/*
 * RX builds keep the original receiver-to-flight-controller USART0 route.
 * TX builds use UART1 as a normal, non-inverted SiW-side UART. The radio-side
 * inverted single-wire DATA bus is handled by an external inverter/tri-state
 * adapter whose active-low TX output-enable is controlled below.
 */
#define CRSF_SERIAL_USE_ULP_UART 0

#if defined(SIW917_ELRS_TARGET_TX)
#define CRSF_SERIAL_DRIVER_NAME "UART1"
#if defined(SIW917_ELRS_CRSF_BENCH_2WIRE)
#define CRSF_SERIAL_ROUTE_NAME "DK2605A breakout bench UART1 2-wire"
#else
#define CRSF_SERIAL_ROUTE_NAME "DK2605A handset CRSF adapter UART1 2-wire + TX_OE_N"
#endif
#define CRSF_SERIAL_TX_PIN RTE_UART1_TX_PIN
#define CRSF_SERIAL_TX_MUX RTE_UART1_TX_MUX
#define CRSF_SERIAL_TX_PAD RTE_UART1_TX_PAD
#define CRSF_SERIAL_RX_PIN RTE_UART1_RX_PIN
#define CRSF_SERIAL_RX_MUX RTE_UART1_RX_MUX
#define CRSF_SERIAL_RX_PAD RTE_UART1_RX_PAD
#define CRSF_SERIAL_SHARED_DATA_PIN RTE_UART1_TX_PIN
#define CRSF_SERIAL_SHARED_DATA_MUX RTE_UART1_TX_MUX
#define CRSF_SERIAL_SHARED_DATA_PAD RTE_UART1_TX_PAD
#define CRSF_SERIAL_TX_DMA_CH RTE_UART1_CHNL_UDMA_TX_CH
#define CRSF_SERIAL_RX_DMA_CH RTE_UART1_CHNL_UDMA_RX_CH
#ifndef CRSF_SERIAL_TX_OE_N_PIN
#define CRSF_SERIAL_TX_OE_N_PIN RTE_UART1_RS485_DE_PIN
#endif
#ifndef CRSF_SERIAL_TX_OE_N_PAD
#define CRSF_SERIAL_TX_OE_N_PAD RTE_UART1_RS485_DE_PAD
#endif
#define CRSF_SERIAL_HAS_TX_OE_N 1
#define CRSF_SERIAL_UART_REGS UART1
#if defined(RTE_UART1_DMA_MODE1_EN) && (RTE_UART1_DMA_MODE1_EN == 1)
#define CRSF_SERIAL_DMA_ENABLED 1
#else
#define CRSF_SERIAL_DMA_ENABLED 0
#endif
#elif CRSF_SERIAL_USE_ULP_UART
#define CRSF_SERIAL_DRIVER_NAME "ULP_UART"
#define CRSF_SERIAL_ROUTE_NAME "BRD2708A mikroBUS native ULP UART"
#define CRSF_SERIAL_TX_PIN RTE_ULP_UART_TX_PIN
#define CRSF_SERIAL_TX_MUX RTE_ULP_UART_TX_MUX
#define CRSF_SERIAL_TX_PAD RTE_ULP_UART_TX_PAD
#define CRSF_SERIAL_RX_PIN RTE_ULP_UART_RX_PIN
#define CRSF_SERIAL_RX_MUX RTE_ULP_UART_RX_MUX
#define CRSF_SERIAL_RX_PAD RTE_ULP_UART_RX_PAD
#define CRSF_SERIAL_SHARED_DATA_PIN 0U
#define CRSF_SERIAL_SHARED_DATA_MUX 0U
#define CRSF_SERIAL_SHARED_DATA_PAD 0U
#define CRSF_SERIAL_TX_DMA_CH RTE_ULPUART_CHNL_UDMA_TX_CH
#define CRSF_SERIAL_RX_DMA_CH RTE_ULPUART_CHNL_UDMA_RX_CH
#define CRSF_SERIAL_HAS_TX_OE_N 0
#define CRSF_SERIAL_UART_REGS ULP_UART
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
#define CRSF_SERIAL_SHARED_DATA_PIN 0U
#define CRSF_SERIAL_SHARED_DATA_MUX 0U
#define CRSF_SERIAL_SHARED_DATA_PAD 0U
#define CRSF_SERIAL_TX_DMA_CH RTE_USART0_CHNL_UDMA_TX_CH
#define CRSF_SERIAL_RX_DMA_CH RTE_USART0_CHNL_UDMA_RX_CH
#define CRSF_SERIAL_HAS_TX_OE_N 0
#define CRSF_SERIAL_UART_REGS USART0
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

#if defined(SIW917_ELRS_TARGET_TX)
#define CRSF_SERIAL_SUPPRESS_TX_ECHO 1
#else
#define CRSF_SERIAL_SUPPRESS_TX_ECHO 0
#endif

#if defined(SIW917_ELRS_TARGET_TX) && \
    !defined(SIW917_ELRS_CRSF_BENCH_2WIRE) && CRSF_SERIAL_DMA_ENABLED
#define CRSF_SERIAL_TX_ECHO_PROBE 1
#else
#define CRSF_SERIAL_TX_ECHO_PROBE 0
#endif

#if defined(SIW917_ELRS_TARGET_TX) && CRSF_SERIAL_DMA_ENABLED
#define CRSF_SERIAL_RX_FRAME_AWARE_DMA 1
#else
#define CRSF_SERIAL_RX_FRAME_AWARE_DMA 0
#endif

#define CRSF_SERIAL_TX_OE_SETUP_US       5U
#define CRSF_SERIAL_TX_ECHO_SETTLE_US    5U
#define CRSF_SERIAL_DEVICE_INFO_TYPE     0x29U

#if defined(SIW917_CRSF_RADIO_RETURN_DIAG)
static volatile uint32_t g_return_diag_reply_count = 0;
static bool g_return_diag_frame_dumped = false;
#define CRSF_RETURN_DIAG_DETAIL_LIMIT 4U
#endif

static uint8_t g_rx_ring[CRSF_SERIAL_RX_RING_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;
static volatile bool g_rx_enabled = false;
static volatile bool g_rx_armed = false;
static volatile bool g_rx_paused_for_tx = false;
static volatile uint32_t g_rx_overrun_count = 0;
static volatile uint32_t g_rx_echo_pause_count = 0;
static volatile uint32_t g_tx_temt_timeout_count = 0;

#if CRSF_SERIAL_TX_ECHO_PROBE
static volatile uint32_t g_tx_echo_probe_count = 0;
static volatile uint32_t g_tx_echo_match_count = 0;
static volatile uint32_t g_tx_echo_miss_count = 0;
static volatile uint32_t g_tx_echo_last_prefix = 0;
#endif

#if defined(SIW917_ELRS_TARGET_TX)
static volatile uint32_t g_device_info_timing_sample_count = 0;
static volatile uint32_t g_device_info_ping_to_oe_last_us = 0;
static volatile uint32_t g_device_info_ping_to_oe_max_us = 0;
static volatile uint32_t g_device_info_wire_last_us = 0;
static volatile uint32_t g_device_info_wire_max_us = 0;
static volatile uint32_t g_device_info_total_last_us = 0;
static volatile uint32_t g_device_info_total_max_us = 0;
#endif

#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
static volatile uint32_t g_last_ping_rx_cycles = 0;
static volatile uint32_t g_last_ping_rx_sequence = 0;
#endif

#if CRSF_SERIAL_HAS_TX_OE_N
static volatile bool g_tx_oe_n_configured = false;
static volatile bool g_tx_oe_n_high = true;
static volatile uint32_t g_tx_oe_n_assert_count = 0;
static volatile uint32_t g_tx_oe_n_release_count = 0;
#endif

#if CRSF_SERIAL_DMA_ENABLED
static volatile uint8_t g_rx_dma_buffer[CRSF_SERIAL_RX_DMA_CHUNK_COUNT][CRSF_SERIAL_RX_DMA_CHUNK_SIZE] CRSF_SERIAL_DMA_ALIGN;
static volatile uint8_t g_rx_dma_active_index = 0;
static volatile uint8_t g_rx_dma_next_index = 0;
static volatile uint32_t g_rx_dma_active_len = 0;
static volatile uint32_t g_rx_dma_pushed_len = 0;
#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
typedef enum {
    CRSF_RX_DMA_SEEK_SYNC = 0,
    CRSF_RX_DMA_READ_LENGTH,
    CRSF_RX_DMA_READ_BODY
} crsf_rx_dma_state_t;
static volatile crsf_rx_dma_state_t g_rx_dma_frame_state = CRSF_RX_DMA_SEEK_SYNC;
static volatile uint32_t g_rx_dma_request_len = 1U;
#endif
#else
static uint8_t g_rx_byte = 0;
#endif

/* CRC lookup table (polynomial 0xD5) */
static uint8_t crc8_table[256];
static bool crc_table_initialized = false;

#if defined(SIW917_ELRS_TARGET_TX)
extern ARM_DRIVER_USART Driver_UART1;
static ARM_DRIVER_USART *crsf_serial_select_driver(void)
{
    return &Driver_UART1;
}
#else
extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_ULP_UART;
static ARM_DRIVER_USART *crsf_serial_select_driver(void)
{
#if CRSF_SERIAL_USE_ULP_UART
    return &Driver_ULP_UART;
#else
    return &Driver_USART0;
#endif
}
#endif

#define CRSF_SERIAL_PAD_CONFIG_BASE 0x46004000UL
#define CRSF_SERIAL_PAD_CONFIG_REG(pin_) \
    (*(volatile uint32_t *)(CRSF_SERIAL_PAD_CONFIG_BASE + (4UL * (uint32_t)(pin_))))
#define CRSF_SERIAL_PAD_REN_ENABLE  (1UL << 4)
#define CRSF_SERIAL_PAD_SMT_ENABLE  (1UL << 3)
#define CRSF_SERIAL_PAD_PULL_MASK   (3UL << 6)
#define CRSF_SERIAL_PAD_PULLUP      (1UL << 6)
#define CRSF_SERIAL_PAD_DRIVE_4MA   (1UL << 0)
#define CRSF_SERIAL_PAD_SLEW_HIGH   (1UL << 5)

#define CRSF_SERIAL_M4CLK_BASE              0x46000000UL
#define CRSF_SERIAL_CLK_ENABLE_SET_REG2     (*(volatile uint32_t *)(CRSF_SERIAL_M4CLK_BASE + 0x008UL))
#define CRSF_SERIAL_CLK_ENABLE_SET_REG3     (*(volatile uint32_t *)(CRSF_SERIAL_M4CLK_BASE + 0x010UL))
#define CRSF_SERIAL_EGPIO_PCLK_ENABLE_BIT   (1UL << 21)
#define CRSF_SERIAL_EGPIO_CLK_ENABLE_BIT    (1UL << 16)
#define CRSF_SERIAL_EGPIO_BASE              0x46130000UL
#define CRSF_SERIAL_GPIO_CONFIG_REG(pin_) \
    (*(volatile uint32_t *)(CRSF_SERIAL_EGPIO_BASE + 0x000UL + (0x10UL * (uint32_t)(pin_))))
#define CRSF_SERIAL_BIT_LOAD_REG(pin_) \
    (*(volatile uint32_t *)(CRSF_SERIAL_EGPIO_BASE + 0x004UL + (0x10UL * (uint32_t)(pin_))))
#define CRSF_SERIAL_GPIO_MODE_MASK      0x3CUL
#define CRSF_SERIAL_GPIO_MODE_GPIO      0x00UL
#define CRSF_SERIAL_GPIO_DIRECTION_BIT  (1UL << 0)
#define CRSF_SERIAL_TX_TEMT_TIMEOUT_MS  10U

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
#if defined(SIW917_ELRS_TARGET_TX) && defined(SIW917_ELRS_CRSF_HANDSET_SINGLEWIRE)
    CRSF_DBG("%s route: shared DATA=%s_%lu (SDK GPIO_%lu)\n",
             CRSF_SERIAL_ROUTE_NAME,
             crsf_serial_pin_domain(CRSF_SERIAL_SHARED_DATA_PIN),
             (unsigned long)crsf_serial_module_pin(CRSF_SERIAL_SHARED_DATA_PIN),
             (unsigned long)CRSF_SERIAL_SHARED_DATA_PIN);
    CRSF_DBG("%s detail: DATA GPIO_%lu mux=%lu pad=%lu; logical RX follows same shared line\n",
             CRSF_SERIAL_DRIVER_NAME,
             (unsigned long)CRSF_SERIAL_SHARED_DATA_PIN,
             (unsigned long)CRSF_SERIAL_SHARED_DATA_MUX,
             (unsigned long)CRSF_SERIAL_SHARED_DATA_PAD);
#elif defined(SIW917_ELRS_TARGET_TX)
    CRSF_DBG("%s route: handset DATA adapter -> GPIO_%lu (SiW RX), GPIO_%lu (SiW TX) -> adapter\n",
             CRSF_SERIAL_ROUTE_NAME,
             (unsigned long)CRSF_SERIAL_RX_PIN,
             (unsigned long)CRSF_SERIAL_TX_PIN);
    CRSF_DBG("%s detail: normal non-inverted 8N1; TX GPIO_%lu mux=%lu pad=%lu; RX GPIO_%lu mux=%lu pad=%lu; TX_OE_N GPIO_%lu pad=%lu\n",
             CRSF_SERIAL_DRIVER_NAME,
             (unsigned long)CRSF_SERIAL_TX_PIN,
             (unsigned long)CRSF_SERIAL_TX_MUX,
             (unsigned long)CRSF_SERIAL_TX_PAD,
             (unsigned long)CRSF_SERIAL_RX_PIN,
             (unsigned long)CRSF_SERIAL_RX_MUX,
             (unsigned long)CRSF_SERIAL_RX_PAD,
             (unsigned long)CRSF_SERIAL_TX_OE_N_PIN,
             (unsigned long)CRSF_SERIAL_TX_OE_N_PAD);
#else
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

static bool crsf_serial_uart_temt(void)
{
    return (CRSF_SERIAL_UART_REGS->LSR_b.TEMT != 0U);
}

#if CRSF_SERIAL_HAS_TX_OE_N
static void crsf_serial_tx_oe_n_write(bool high)
{
    if (CRSF_SERIAL_TX_OE_N_PIN >= GPIO_MAX_PIN) {
        return;
    }

    CRSF_SERIAL_BIT_LOAD_REG(CRSF_SERIAL_TX_OE_N_PIN) = high ? 1UL : 0UL;

    if (g_tx_oe_n_high != high) {
        if (high) {
            g_tx_oe_n_release_count++;
        } else {
            g_tx_oe_n_assert_count++;
        }
    }
    g_tx_oe_n_high = high;
}

static void crsf_serial_configure_tx_oe_n_idle(void)
{
    if (CRSF_SERIAL_TX_OE_N_PIN >= GPIO_MAX_PIN) {
        CRSF_DBG("Handset TX_OE_N GPIO_%lu is not an HP GPIO; OE disabled\n",
                 (unsigned long)CRSF_SERIAL_TX_OE_N_PIN);
        return;
    }

    CRSF_SERIAL_CLK_ENABLE_SET_REG2 = CRSF_SERIAL_EGPIO_PCLK_ENABLE_BIT;
    CRSF_SERIAL_CLK_ENABLE_SET_REG3 = CRSF_SERIAL_EGPIO_CLK_ENABLE_BIT;
    RSI_CLK_PeripheralClkEnable(M4CLK, EGPIO_CLK, ENABLE_STATIC_CLK);

    RSI_EGPIO_PadSelectionEnable(CRSF_SERIAL_TX_OE_N_PAD);
    RSI_EGPIO_SetPinMux(EGPIO,
                        0,
                        CRSF_SERIAL_TX_OE_N_PIN,
                        EGPIO_PIN_MUX_MODE0);

    CRSF_SERIAL_PAD_CONFIG_REG(CRSF_SERIAL_TX_OE_N_PIN) =
        CRSF_SERIAL_PAD_DRIVE_4MA | CRSF_SERIAL_PAD_SLEW_HIGH;

    CRSF_SERIAL_BIT_LOAD_REG(CRSF_SERIAL_TX_OE_N_PIN) = 1UL;
    g_tx_oe_n_high = true;

    uint32_t cfg = CRSF_SERIAL_GPIO_CONFIG_REG(CRSF_SERIAL_TX_OE_N_PIN);
    cfg &= ~CRSF_SERIAL_GPIO_MODE_MASK;
    cfg |= CRSF_SERIAL_GPIO_MODE_GPIO;
    cfg &= ~CRSF_SERIAL_GPIO_DIRECTION_BIT;
    CRSF_SERIAL_GPIO_CONFIG_REG(CRSF_SERIAL_TX_OE_N_PIN) = cfg;

    crsf_serial_tx_oe_n_write(true);
    g_tx_oe_n_configured = true;
    CRSF_DBG("Handset TX_OE_N idle HIGH on GPIO_%lu pad=%lu (adapter output high-Z)\n",
             (unsigned long)CRSF_SERIAL_TX_OE_N_PIN,
             (unsigned long)CRSF_SERIAL_TX_OE_N_PAD);
}
#else
#define crsf_serial_configure_tx_oe_n_idle() ((void)0)
#define crsf_serial_tx_oe_n_write(high_) ((void)0)
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

static void rx_dma_state_reset(void)
{
#if CRSF_SERIAL_DMA_ENABLED
    g_rx_dma_active_len = 0;
    g_rx_dma_pushed_len = 0;
    g_rx_dma_active_index = 0;
    g_rx_dma_next_index = 0;
#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
    g_rx_dma_frame_state = CRSF_RX_DMA_SEEK_SYNC;
    g_rx_dma_request_len = 1U;
#endif
#endif
}

#if CRSF_SERIAL_DMA_ENABLED
static void rx_ring_push_block_from_isr(const volatile uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        rx_ring_push_from_isr(data[i]);
    }
}

static void rx_dma_harvest_to_ring_unlocked(void)
{
    /*
     * Do not peek at an active Si91x UDMA descriptor as if it were a stable
     * byte-stream cursor. On UART1 this produced phantom progress and pushed
     * zeros/stale bytes into the CRSF parser. Completed chunks are published
     * only from the DMA completion callback below.
     */
    (void)0;
}

static void rx_dma_harvest_to_ring(void)
{
    rx_dma_harvest_to_ring_unlocked();
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
    uint32_t request_len = CRSF_SERIAL_RX_DMA_CHUNK_SIZE;
#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
    request_len = g_rx_dma_request_len;
    if (request_len == 0U || request_len > CRSF_SERIAL_RX_DMA_CHUNK_SIZE) {
        request_len = 1U;
        g_rx_dma_frame_state = CRSF_RX_DMA_SEEK_SYNC;
        g_rx_dma_request_len = request_len;
    }
#endif
    const int32_t rc = g_usart->Receive((void *)g_rx_dma_buffer[next],
                                        request_len);
    if (rc == ARM_DRIVER_OK) {
        g_rx_dma_active_index = next;
        g_rx_dma_next_index = (uint8_t)((next + 1U) % CRSF_SERIAL_RX_DMA_CHUNK_COUNT);
        g_rx_dma_active_len = request_len;
        g_rx_dma_pushed_len = 0;
        g_rx_armed = true;
        return;
    }

    /*
     * If the SDK reports BUSY, leave the existing active buffer metadata alone;
     * the hardware may still be filling it. Rotating software state here would
     * desynchronize the ring from the real DMA target.
     */
    ARM_USART_STATUS status = g_usart->GetStatus();
    g_rx_armed = status.rx_busy ? true : false;
    if (!g_rx_armed) {
        g_rx_dma_active_len = 0;
        g_rx_dma_pushed_len = 0;
    }
#else
    g_rx_armed = (g_usart->Receive(&g_rx_byte, 1) == ARM_DRIVER_OK);
#endif
}

#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
static void rx_dma_advance_frame_state(const volatile uint8_t *data, uint32_t len)
{
    if (len == 0U) {
        g_rx_dma_frame_state = CRSF_RX_DMA_SEEK_SYNC;
        g_rx_dma_request_len = 1U;
        return;
    }

    switch (g_rx_dma_frame_state) {
        case CRSF_RX_DMA_SEEK_SYNC:
            if (data[0] == 0xEEU || data[0] == CRSF_SYNC_BYTE) {
                g_rx_dma_frame_state = CRSF_RX_DMA_READ_LENGTH;
            }
            g_rx_dma_request_len = 1U;
            break;

        case CRSF_RX_DMA_READ_LENGTH: {
            const uint32_t body_len = data[0];
            if (body_len >= 2U &&
                body_len <= (CRSF_SERIAL_MAX_FRAME_SIZE - 2U)) {
                g_rx_dma_frame_state = CRSF_RX_DMA_READ_BODY;
                g_rx_dma_request_len = body_len;
            } else {
                g_rx_dma_frame_state = CRSF_RX_DMA_SEEK_SYNC;
                g_rx_dma_request_len = 1U;
            }
            break;
        }

        case CRSF_RX_DMA_READ_BODY:
        default:
            g_rx_dma_frame_state = CRSF_RX_DMA_SEEK_SYNC;
            g_rx_dma_request_len = 1U;
            break;
    }
}
#endif

#if defined(SIW917_ELRS_TARGET_TX)
static uint32_t crsf_serial_cycles_to_us(uint32_t cycles)
{
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    return cycles_per_us != 0U ? cycles / cycles_per_us : 0U;
}
#endif

static void rx_pause_for_tx_echo_suppression(void)
{
#if CRSF_SERIAL_SUPPRESS_TX_ECHO
    if (!g_rx_enabled || g_usart == NULL || g_rx_paused_for_tx) {
        return;
    }

    g_rx_paused_for_tx = true;
    g_rx_armed = false;
    g_rx_echo_pause_count++;
    (void)g_usart->Control(ARM_USART_ABORT_RECEIVE, 0);
    (void)g_usart->Control(ARM_USART_CONTROL_RX, 0);
    rx_dma_state_reset();
    rx_ring_reset();

#if CRSF_SERIAL_TX_ECHO_PROBE
    /*
     * Arm a clean DMA buffer only for the duration of our own transmission.
     * The adapter's always-on RX path should reflect the shared DATA waveform
     * back to SIW_RX. The bytes are inspected and discarded before normal CRSF
     * parsing resumes, so local echo cannot reach the upstream router.
     */
    memset((void *)g_rx_dma_buffer, 0, sizeof(g_rx_dma_buffer));
#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
    g_rx_dma_request_len = CRSF_SERIAL_RX_DMA_CHUNK_SIZE;
#endif
    if (g_usart->Control(ARM_USART_CONTROL_RX, 1) == ARM_DRIVER_OK) {
        rx_arm_receive_from_isr();
    }
#endif
#endif
}

static void rx_resume_after_tx_echo_suppression_from_isr(void)
{
#if CRSF_SERIAL_SUPPRESS_TX_ECHO
    if (!g_rx_paused_for_tx || !g_rx_enabled || g_usart == NULL) {
        return;
    }

    (void)g_usart->Control(ARM_USART_ABORT_RECEIVE, 0);
    (void)g_usart->Control(ARM_USART_CONTROL_RX, 0);
    g_rx_paused_for_tx = false;
    rx_dma_state_reset();
    rx_ring_reset();
    if (g_usart->Control(ARM_USART_CONTROL_RX, 1) == ARM_DRIVER_OK) {
        rx_arm_receive_from_isr();
    } else {
        g_rx_armed = false;
    }
#endif
}

static void rx_probe_tx_echo_and_resume(const uint8_t *frame, uint32_t frame_len)
{
#if CRSF_SERIAL_TX_ECHO_PROBE
    delayMicroseconds(CRSF_SERIAL_TX_ECHO_SETTLE_US);

    const uint8_t buffer_index = g_rx_dma_active_index;
    const volatile uint8_t *captured = g_rx_dma_buffer[buffer_index];
    uint32_t prefix = 0;
    while (prefix < frame_len && captured[prefix] == frame[prefix]) {
        prefix++;
    }

    g_tx_echo_probe_count++;
    g_tx_echo_last_prefix = prefix;
    const bool matched = prefix == frame_len;
    if (matched) {
        g_tx_echo_match_count++;
    } else {
        g_tx_echo_miss_count++;
    }
#else
    (void)frame;
    (void)frame_len;
#endif

    rx_resume_after_tx_echo_suppression_from_isr();
}

/*******************************************************************************
 * USART Callback (required by driver)
 ******************************************************************************/

static void usart_callback(uint32_t event)
{
    if (event & (ARM_USART_EVENT_SEND_COMPLETE | ARM_USART_EVENT_TX_COMPLETE)) {
        g_tx_in_progress = false;
#if !defined(SIW917_ELRS_TARGET_TX)
        rx_resume_after_tx_echo_suppression_from_isr();
#endif
    }

    /*
     * In IRQ 1-byte RX mode the Si91x driver calls cb_event(RECEIVE_COMPLETE),
     * then continues inside the same ISR. Because we re-arm RX immediately, the
     * driver's later timeout check can call cb_event(RECEIVE_COMPLETE|RX_TIMEOUT)
     * for the freshly armed transfer while g_rx_byte still holds the old byte.
     * Treat timeout-bearing IRQ callbacks as timeout only, otherwise every byte
     * is pushed twice and CRSF frames become EE EE 18 18 16 16 ...
     */
    const bool rx_timeout = (event & ARM_USART_EVENT_RX_TIMEOUT) != 0U;
    const bool rx_complete =
        ((event & ARM_USART_EVENT_RECEIVE_COMPLETE) != 0U) && !rx_timeout;

    if (rx_complete) {
#if CRSF_SERIAL_TX_ECHO_PROBE
        if (g_rx_paused_for_tx) {
            /* Keep the completed probe buffer intact for the task to inspect. */
            g_rx_armed = false;
        } else
#endif
        {
#if CRSF_SERIAL_DMA_ENABLED
#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
            bool completed_frame = false;
#endif
            if (g_rx_dma_active_len != 0U) {
                const uint8_t index = g_rx_dma_active_index;
#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
                completed_frame = g_rx_dma_frame_state == CRSF_RX_DMA_READ_BODY;
                if (completed_frame &&
                    g_rx_dma_buffer[index][0] == 0x28U) {
                    g_last_ping_rx_cycles = DWT->CYCCNT;
                    g_last_ping_rx_sequence++;
                }
#endif
                rx_ring_push_block_from_isr(g_rx_dma_buffer[index], g_rx_dma_active_len);
                g_rx_dma_pushed_len = g_rx_dma_active_len;
#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
                rx_dma_advance_frame_state(g_rx_dma_buffer[index], g_rx_dma_active_len);
#endif
            }
#else
            rx_ring_push_from_isr(g_rx_byte);
#endif
            g_rx_armed = false;
            rx_arm_receive_from_isr();
#if CRSF_SERIAL_RX_FRAME_AWARE_DMA && defined(SIW917_ELRS_TARGET_TX)
            if (completed_frame) {
                elrs_task_wakeup_from_isr(ELRS_TASK_WAKE_UART);
            }
#endif
        }
    }

    if (event & ARM_USART_EVENT_RX_OVERFLOW) {
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

#if CRSF_SERIAL_DMA_ENABLED
    if (rx_timeout) {
        /*
         * The DMA transfer is still owned by the driver on timeout. Publishing
         * a partial active buffer here corrupts CRSF frame alignment, so wait
         * for the next UDMA_EVENT_XFER_DONE.
         */
    }
#else
    (void)rx_timeout;
#endif
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

static int wait_for_tx_complete_and_temt(uint32_t frame_len)
{
    (void)frame_len;

#if defined(SIW917_ELRS_TARGET_TX)
    const uint32_t start_cycles = DWT->CYCCNT;
    const uint32_t timeout_cycles =
        (SystemCoreClock / 1000U) * CRSF_SERIAL_TX_TEMT_TIMEOUT_MS;
#else
    uint32_t spin_count = 0;
    const osKernelState_t kernel_state = osKernelGetState();
    const bool kernel_running = (kernel_state == osKernelRunning);
    const uint32_t start_tick = kernel_running ? osKernelGetTickCount() : 0U;
#endif

    while (true) {
        /*
         * On the Si91x CMSIS UART1 DMA path, status.tx_busy can remain set
         * after the send callback fires even though the UART TEMT bit says the
         * shift register is empty. For the handset half-duplex adapter, TEMT is
         * the electrical condition we must honor before releasing TX_OE_N.
         */
        if (!g_tx_in_progress && crsf_serial_uart_temt()) {
            g_tx_busy_skip_count = 0;
            return 0;
        }

#if defined(SIW917_ELRS_TARGET_TX)
        /*
         * The handset owns a short half-duplex response window. A one-tick
         * RTOS sleep can be longer than the complete CRSF reply, so keep
         * interrupts enabled and poll TEMT until the final stop bit leaves
         * the UART. The longest legal CRSF frame is under 2 ms at 400 kbaud.
         */
        if ((uint32_t)(DWT->CYCCNT - start_cycles) > timeout_cycles) {
            break;
        }
        __NOP();
#else
        if (kernel_running) {
            if ((uint32_t)(osKernelGetTickCount() - start_tick) > CRSF_SERIAL_TX_TEMT_TIMEOUT_MS) {
                break;
            }
            if ((spin_count++ & 0x3FU) == 0U) {
                osDelay(1);
            }
        } else if (++spin_count > 1000000U) {
            break;
        }
#endif
    }

    g_tx_temt_timeout_count++;
    if (g_tx_temt_timeout_count <= 4U) {
        ARM_USART_STATUS status = g_usart->GetStatus();
        CRSF_DBG("TX TEMT timeout #%lu tx_busy=%u in_progress=%u temt=%u len=%lu\n",
                 (unsigned long)g_tx_temt_timeout_count,
                 (unsigned)status.tx_busy,
                 g_tx_in_progress ? 1U : 0U,
                 crsf_serial_uart_temt() ? 1U : 0U,
                 (unsigned long)frame_len);
    }
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
#if defined(SIW917_ELRS_TARGET_TX)
    const uint8_t frame_type = frame_len > 2U ? frame[2] : 0U;
    const bool measure_device_info =
        frame_type == CRSF_SERIAL_DEVICE_INFO_TYPE &&
        g_last_ping_rx_sequence != 0U;
    const uint32_t ping_rx_cycles = g_last_ping_rx_cycles;
#endif
    rx_pause_for_tx_echo_suppression();
#if defined(SIW917_ELRS_TARGET_TX)
    const uint32_t oe_assert_cycles = DWT->CYCCNT;
#endif
    crsf_serial_tx_oe_n_write(false);
#if defined(SIW917_ELRS_TARGET_TX)
    delayMicroseconds(CRSF_SERIAL_TX_OE_SETUP_US);
    const uint32_t send_start_cycles = DWT->CYCCNT;
#endif
    g_tx_in_progress = true;
    g_tx_busy_skip_count = 0;

    if (g_usart->Send(g_tx_buffer, frame_len) != ARM_DRIVER_OK) {
        g_tx_in_progress = false;
        crsf_serial_tx_oe_n_write(true);
        rx_resume_after_tx_echo_suppression_from_isr();
        return -3;
    }

    if (wait_for_tx_complete_and_temt(frame_len) != 0) {
        (void)g_usart->Control(ARM_USART_ABORT_SEND, 0);
        g_tx_in_progress = false;
        crsf_serial_tx_oe_n_write(true);
        rx_resume_after_tx_echo_suppression_from_isr();
        return -4;
    }

#if defined(SIW917_ELRS_TARGET_TX)
    const uint32_t temt_cycles = DWT->CYCCNT;
#endif
    crsf_serial_tx_oe_n_write(true);
#if defined(SIW917_ELRS_TARGET_TX)
    const uint32_t oe_release_cycles = DWT->CYCCNT;
    if (measure_device_info) {
        const uint32_t ping_to_oe_us =
            crsf_serial_cycles_to_us(oe_assert_cycles - ping_rx_cycles);
        const uint32_t wire_us =
            crsf_serial_cycles_to_us(temt_cycles - send_start_cycles);
        const uint32_t total_us =
            crsf_serial_cycles_to_us(oe_release_cycles - ping_rx_cycles);

        g_device_info_timing_sample_count++;
        g_device_info_ping_to_oe_last_us = ping_to_oe_us;
        g_device_info_wire_last_us = wire_us;
        g_device_info_total_last_us = total_us;
        if (ping_to_oe_us > g_device_info_ping_to_oe_max_us) {
            g_device_info_ping_to_oe_max_us = ping_to_oe_us;
        }
        if (wire_us > g_device_info_wire_max_us) {
            g_device_info_wire_max_us = wire_us;
        }
        if (total_us > g_device_info_total_max_us) {
            g_device_info_total_max_us = total_us;
        }
    }
#endif
    rx_probe_tx_echo_and_resume(g_tx_buffer, frame_len);

#if defined(SIW917_CRSF_RADIO_RETURN_DIAG)
    if (measure_device_info) {
        if (!g_return_diag_frame_dumped) {
            const uint8_t calculated_crc =
                frame_len >= 3U
                    ? crsf_crc8(g_tx_buffer + 2U, (uint8_t)(frame_len - 3U))
                    : 0U;
            printf("[CRSF_RETURN_DIAG] upstream-only sync=0x%02X dest=0x%02X\n",
                   g_tx_buffer[0],
                   frame_len > 3U ? g_tx_buffer[3] : 0U);
            printf("[CRSF_RETURN_DIAG] device_info len=%lu crc=0x%02X wire_crc=0x%02X bytes=",
                   (unsigned long)frame_len,
                   calculated_crc,
                   g_tx_buffer[frame_len - 1U]);
            for (uint32_t i = 0U; i < frame_len; i++) {
                printf("%02X%s", g_tx_buffer[i], (i + 1U < frame_len) ? " " : "\n");
            }
            g_return_diag_frame_dumped = true;
        }
#if CRSF_SERIAL_TX_ECHO_PROBE
        /*
         * EdgeTX sends an external-module frame every 4 ms at 400 kbaud.
         * Printing every reply can consume the following response slot and
         * turn the diagnostic itself into the timing failure under test.
         * Keep only enough detail to validate the waveform; the normal
         * five-second CRSF BUS summary carries the cumulative counters.
         */
        if (g_return_diag_reply_count < CRSF_RETURN_DIAG_DETAIL_LIMIT) {
            printf("[CRSF_RETURN_DIAG] reply=%lu actual_ping_to_oe=%lu us total=%lu us echo=%s prefix=%lu/%lu\n",
                   (unsigned long)(g_return_diag_reply_count + 1U),
                   (unsigned long)g_device_info_ping_to_oe_last_us,
                   (unsigned long)g_device_info_total_last_us,
                   g_tx_echo_last_prefix == frame_len ? "MATCH" : "MISS",
                   (unsigned long)g_tx_echo_last_prefix,
                   (unsigned long)frame_len);
        }
#endif
        g_return_diag_reply_count++;
    }
#endif

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
    uint32_t control =
#if defined(SIW917_ELRS_TARGET_TX) && defined(SIW917_ELRS_CRSF_HANDSET_SINGLEWIRE)
                       ARM_USART_MODE_SINGLE_WIRE |
#else
                       ARM_USART_MODE_ASYNCHRONOUS |
#endif
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
    crsf_serial_tx_oe_n_write(true);
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

    g_usart = crsf_serial_select_driver();

#if defined(SIW917_ELRS_TARGET_TX)
    crsf_serial_configure_tx_oe_n_idle();
    CRSF_DBG("Init %s handset CRSF normal UART on TX GPIO_%lu, RX GPIO_%lu, TX_OE_N GPIO_%lu\n",
             CRSF_SERIAL_DRIVER_NAME,
             (unsigned long)CRSF_SERIAL_TX_PIN,
             (unsigned long)CRSF_SERIAL_RX_PIN,
             (unsigned long)CRSF_SERIAL_TX_OE_N_PIN);
#if CRSF_SERIAL_TX_ECHO_PROBE
    CRSF_DBG("TXBUS echo probe enabled; OE setup=%u us, echo settle=%u us\n",
             (unsigned)CRSF_SERIAL_TX_OE_SETUP_US,
             (unsigned)CRSF_SERIAL_TX_ECHO_SETTLE_US);
#endif
#if CRSF_SERIAL_RX_FRAME_AWARE_DMA
    CRSF_DBG("Handset RX frame-aware DMA enabled (sync 1 + length 1 + exact body + task wake)\n");
#endif
#else
    CRSF_DBG("Init %s on TX GPIO_%lu, RX GPIO_%lu\n",
             CRSF_SERIAL_DRIVER_NAME,
             (unsigned long)CRSF_SERIAL_TX_PIN,
             (unsigned long)CRSF_SERIAL_RX_PIN);
#endif
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
        crsf_serial_tx_oe_n_write(true);
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

void crsf_serial_debug_dump(void)
{
#if CRSF_SERIAL_DMA_ENABLED
    rx_dma_harvest_to_ring();
#endif
    const uint16_t head = g_rx_head;
    const uint16_t tail = g_rx_tail;
    const uint32_t available = (uint32_t)((head - tail) & CRSF_SERIAL_RX_RING_MASK);
    unsigned oe_configured = 0U;
    unsigned oe_high = 1U;
    unsigned long oe_assert = 0UL;
    unsigned long oe_release = 0UL;
#if CRSF_SERIAL_HAS_TX_OE_N
    oe_configured = g_tx_oe_n_configured ? 1U : 0U;
    oe_high = g_tx_oe_n_high ? 1U : 0U;
    oe_assert = (unsigned long)g_tx_oe_n_assert_count;
    oe_release = (unsigned long)g_tx_oe_n_release_count;
#endif

    unsigned long echo_probes = 0UL;
    unsigned long echo_matches = 0UL;
    unsigned long echo_misses = 0UL;
#if CRSF_SERIAL_TX_ECHO_PROBE
    echo_probes = (unsigned long)g_tx_echo_probe_count;
    echo_matches = (unsigned long)g_tx_echo_match_count;
    echo_misses = (unsigned long)g_tx_echo_miss_count;
#endif

    unsigned long timing_samples = 0UL;
    unsigned long ping_to_oe_last_us = 0UL;
    unsigned long ping_to_oe_max_us = 0UL;
    unsigned long wire_last_us = 0UL;
    unsigned long wire_max_us = 0UL;
    unsigned long total_last_us = 0UL;
    unsigned long total_max_us = 0UL;
#if defined(SIW917_ELRS_TARGET_TX)
    timing_samples = (unsigned long)g_device_info_timing_sample_count;
    ping_to_oe_last_us = (unsigned long)g_device_info_ping_to_oe_last_us;
    ping_to_oe_max_us = (unsigned long)g_device_info_ping_to_oe_max_us;
    wire_last_us = (unsigned long)g_device_info_wire_last_us;
    wire_max_us = (unsigned long)g_device_info_wire_max_us;
    total_last_us = (unsigned long)g_device_info_total_last_us;
    total_max_us = (unsigned long)g_device_info_total_max_us;
#endif

    CRSF_DBG("BUS driver=%s route=%s baud=%lu rx=%u/%u paused=%u avail=%lu overrun=%lu "
             "tx=%lu temt_timeout=%lu oe=%u/%u pulses=%lu/%lu "
             "echo=%lu/%lu/%lu devinfo=%lu ping_to_oe=%lu/%lu us "
             "wire=%lu/%lu us total=%lu/%lu us\n",
             CRSF_SERIAL_DRIVER_NAME,
             CRSF_SERIAL_ROUTE_NAME,
             (unsigned long)g_baud_rate,
             g_rx_enabled ? 1U : 0U,
             g_rx_armed ? 1U : 0U,
             g_rx_paused_for_tx ? 1U : 0U,
             (unsigned long)available,
             (unsigned long)g_rx_overrun_count,
             (unsigned long)g_tx_count,
             (unsigned long)g_tx_temt_timeout_count,
             oe_configured,
             oe_high,
             oe_assert,
             oe_release,
             echo_probes,
             echo_matches,
             echo_misses,
             timing_samples,
             ping_to_oe_last_us,
             ping_to_oe_max_us,
             wire_last_us,
             wire_max_us,
             total_last_us,
             total_max_us);
}
