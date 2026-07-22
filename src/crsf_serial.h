/**
 * @file crsf_serial.h
 * @brief Board-specific CRSF serial transport for SiW917
 *
 * Outputs CRSF bytes over a board-specific UART path.
 *
 * - RX builds keep the existing USART0 receiver-to-flight-controller path.
 * - TX builds use UART1 as a normal, non-inverted 8N1 UART on the SiW side.
 *   An external inverter/tri-state adapter owns the radio-side inverted
 *   single-wire DATA line.
 *
 * Pin Configuration:
 *   RX build:
 *     Uses generated `Driver_USART0` routing from `config/RTE_Device_917.h`.
 *
 *   TX build on DK2605A:
 *     Uses generated `Driver_UART1` routing.
 *     TX = GPIO_7, RX = GPIO_6, TX_OE_N = GPIO_9 by default.
 *
 * Citation: TBS CRSF Protocol Specification
 * Citation: ExpressLRS src/lib/CrsfProtocol/crsf_protocol.h
 */

#ifndef CRSF_SERIAL_H
#define CRSF_SERIAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

#if defined(SIW917_ELRS_TARGET_TX)
#define CRSF_SERIAL_BAUDRATE_DEFAULT   400000  /* EdgeTX external CRSF */
#else
#define CRSF_SERIAL_BAUDRATE_DEFAULT   420000  /* ELRS RX-to-FC default */
#endif
#define SBUS_SERIAL_BAUDRATE           100000
#define SUMD_SERIAL_BAUDRATE           115200

/* Use existing CRSF definitions from crsf_protocol.h if available */
#ifndef CRSF_SYNC_BYTE
#define CRSF_SYNC_BYTE          0xC8
#endif
#ifndef CRSF_FRAMETYPE_RC_CHANNELS_PACKED
#define CRSF_FRAMETYPE_RC       0x16    /* RC channels packed */
#else
#define CRSF_FRAMETYPE_RC       CRSF_FRAMETYPE_RC_CHANNELS_PACKED
#endif
#ifndef CRSF_FRAMETYPE_LINK_STATISTICS
#define CRSF_FRAMETYPE_LINK     0x14    /* Link statistics */
#else
#define CRSF_FRAMETYPE_LINK     CRSF_FRAMETYPE_LINK_STATISTICS
#endif
#ifndef CRSF_CRC_POLY
#define CRSF_CRC_POLY           0xD5
#endif

#define CRSF_SERIAL_NUM_CHANNELS       16
#define CRSF_SERIAL_CHANNEL_MIN        172     /* 988us */
#define CRSF_SERIAL_CHANNEL_MID        992     /* 1500us */
#define CRSF_SERIAL_CHANNEL_MAX        1811    /* 2012us */
#define CRSF_SERIAL_MAX_FRAME_SIZE     64

typedef enum {
    CRSF_SERIAL_FORMAT_8N1 = 0,
    CRSF_SERIAL_FORMAT_8E2 = 1,
} crsf_serial_format_t;

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

typedef struct {
    uint32_t baud_rate;
    uint32_t available;
    uint32_t dma_arm_count;
    uint32_t dma_complete_count;
    uint32_t dma_bytes_published;
    uint32_t dma_rearm_busy_count;
    uint32_t dma_rearm_fail_count;
    uint32_t dma_timeout_count;
    uint32_t rx_overrun_count;
    bool rx_enabled;
    bool rx_armed;
} crsf_serial_rx_diag_t;

/**
 * @brief Link statistics for CRSF output
 */
typedef struct {
    uint8_t uplink_rssi_1;      /* RSSI antenna 1 (dBm * -1) */
    uint8_t uplink_rssi_2;      /* RSSI antenna 2 (dBm * -1) */
    uint8_t uplink_lq;          /* Link quality 0-100% */
    int8_t  uplink_snr;         /* SNR in dB */
    uint8_t active_antenna;     /* 0 = ant1, 1 = ant2 */
    uint8_t rf_mode;            /* RF mode index */
    uint8_t uplink_tx_power;    /* TX power index */
    uint8_t downlink_rssi;      /* Downlink RSSI (dBm * -1) */
    uint8_t downlink_lq;        /* Downlink LQ 0-100% */
    int8_t  downlink_snr;       /* Downlink SNR in dB */
} crsf_link_stats_t;

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize CRSF serial output
 *
 * Configures the selected UART for CRSF output at the specified baud rate.
 *
 * @param baud_rate Baud rate (0 = build-specific default)
 * @return 0 on success, negative on error
 */
int crsf_serial_init(uint32_t baud_rate);

/**
 * @brief Initialize/reconfigure the selected UART with an explicit frame format
 *
 * This keeps CRSF/MAVLink on 8N1 while allowing SBUS to use upstream's 8E2
 * framing without maintaining a second serial driver path.
 */
int crsf_serial_init_ex(uint32_t baud_rate, crsf_serial_format_t format);

/**
 * @brief Deinitialize CRSF serial output
 */
void crsf_serial_deinit(void);

/**
 * @brief Check if CRSF serial is initialized
 * @return true if ready
 */
bool crsf_serial_is_ready(void);

/**
 * @brief Send RC channels to flight controller
 *
 * Packs and transmits 16 channels in CRSF format.
 *
 * @param channels Array of 16 channel values (CRSF format: 172-1811)
 * @return 0 on success, negative on error
 */
int crsf_serial_send_channels(const uint32_t *channels);

/**
 * @brief Send RC channels in SBUS format
 *
 * @param channels Array of 16 channel values in CRSF channel units
 * @param failsafe_active Set SBUS failsafe flag
 * @param frame_lost Set SBUS frame lost flag
 */
int crsf_serial_send_sbus_channels(const uint32_t *channels,
                                   bool failsafe_active,
                                   bool frame_lost);

/**
 * @brief Send RC channels in SUMD format
 *
 * @param channels Array of 16 channel values in CRSF channel units
 */
int crsf_serial_send_sumd_channels(const uint32_t *channels);

/**
 * @brief Send link statistics to flight controller
 *
 * @param stats Link statistics structure
 * @return 0 on success, negative on error
 */
int crsf_serial_send_link_stats(const crsf_link_stats_t *stats);

/**
 * @brief Send a prebuilt CRSF frame to the flight controller
 *
 * @param frame Complete CRSF frame including sync byte, length, type and CRC
 * @param frame_len Total frame length in bytes
 * @return 0 on success, negative on error
 */
int crsf_serial_send_frame(const uint8_t *frame, uint32_t frame_len);

/** Wait until all queued bytes and the UART shift register are empty. */
int crsf_serial_flush(void);

/**
 * @brief Enable or disable USART RX byte capture
 *
 * RX is optional because normal standalone timing tests only need serial TX.
 *
 * @param enable true to receive bytes into the internal ring buffer
 * @return 0 on success, negative on error
 */
int crsf_serial_set_rx_enabled(bool enable);

/** Service deferred handset UART work from the platform task. */
void crsf_serial_service(void);

/** Atomically discard bytes captured before or during a baud-rate change. */
void crsf_serial_discard_rx(void);

/**
 * @brief Return the number of bytes currently buffered from USART RX
 */
uint32_t crsf_serial_rx_available(void);

/**
 * @brief Read buffered USART RX bytes without blocking
 *
 * @param out Destination buffer
 * @param max_len Maximum bytes to copy
 * @return Number of bytes copied
 */
uint32_t crsf_serial_read(uint8_t *out, uint32_t max_len);

/**
 * @brief RX bytes dropped because the software ring was full
 */
uint32_t crsf_serial_get_rx_overrun_count(void);

/**
 * @brief Snapshot low-overhead handset RX/DMA diagnostics
 */
void crsf_serial_get_rx_diag(crsf_serial_rx_diag_t *diag);

/**
 * @brief Print raw UART RX/TX diagnostic state without consuming buffered bytes
 */
void crsf_serial_debug_dump(void);

/**
 * @brief Get number of frames sent
 * @return Total frames transmitted
 */
uint32_t crsf_serial_get_tx_count(void);

#ifdef __cplusplus
}
#endif

#endif /* CRSF_SERIAL_H */
