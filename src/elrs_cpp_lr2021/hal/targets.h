/**
 * @file targets.h
 * @brief ELRS target definitions for SiW917 platform
 * 
 * This file defines the platform and includes all necessary headers
 * for the ELRS C++ code to compile on SiW917.
 */

#pragma once

// Define our platform
#define PLATFORM_SIW917 1

// This is a receiver target
#define TARGET_RX 1

// Radio type - keep RADIO_LR1121 for upstream ELRS table compatibility until
// the copied driver is fully migrated to the LR20xx command set.
#define RADIO_LR1121 1
#define RADIO_LR2021 1

// Regulatory domain (can be overridden)
#ifndef Regulatory_Domain_FCC_915
#define Regulatory_Domain_FCC_915 1
#endif

// OTA Version ID - must match TX for compatibility
#define OTA_VERSION_ID 4

// LR2021 diagnostic defaults must be visible before siw917_elrs_timing.h sets
// its shared fallbacks.
#ifndef SIW917_ELRS_DISABLE_DOWNLINK_TLM
#define SIW917_ELRS_DISABLE_DOWNLINK_TLM 0
#endif
#ifndef SIW917_ELRS_FHSS_SET_FREQ_RX
#define SIW917_ELRS_FHSS_SET_FREQ_RX 1
#endif
#ifndef SIW917_ELRS_LR2021_MIN_TLM_DENOM
#define SIW917_ELRS_LR2021_MIN_TLM_DENOM 2
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_MIN_TLM_DENOM
// Honor the TX-selected telemetry ratio for GFSK. Forcing GFSK downlink to
// 1:128 makes Lua/device telemetry crawl and causes the TX to report missing
// telemetry whenever it is listening for a faster ratio.
#define SIW917_ELRS_LR2021_GFSK_MIN_TLM_DENOM SIW917_ELRS_LR2021_MIN_TLM_DENOM
#endif
#ifndef SIW917_ELRS_LR2021_TXDONE_WATCHDOG
#define SIW917_ELRS_LR2021_TXDONE_WATCHDOG 1
#endif
#ifndef SIW917_ELRS_LR2021_TXDONE_WATCHDOG_PEEK_IRQ
#define SIW917_ELRS_LR2021_TXDONE_WATCHDOG_PEEK_IRQ 0
#endif
#ifndef SIW917_ELRS_LR2021_FORCE_RF_TLM
#define SIW917_ELRS_LR2021_FORCE_RF_TLM 1
#endif
#ifndef SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX
#define SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX 0
#endif
#ifndef SIW917_ELRS_LR2021_TLM_AFTER_TIMER_LOCK
// Match upstream RX behavior: telemetry slots are valid once the receiver is
// out of disconnected state. Gating LR2021 downlink until tim_locked can leave
// the TX with no telemetry during long tentative/settling periods.
#define SIW917_ELRS_LR2021_TLM_AFTER_TIMER_LOCK 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_TLM_BEFORE_TIMER_LOCK
#define SIW917_ELRS_LR2021_GFSK_TLM_BEFORE_TIMER_LOCK 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_RX_LOCK_TIMEOUT_MS
#define SIW917_ELRS_LR2021_GFSK_RX_LOCK_TIMEOUT_MS 6500
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_RELAX_TENTATIVE_LQ
#define SIW917_ELRS_LR2021_GFSK_RELAX_TENTATIVE_LQ 1
#endif
#ifndef SIW917_ELRS_LR2021_RUNTIME_CALIB_FE
#define SIW917_ELRS_LR2021_RUNTIME_CALIB_FE 1
#endif
#ifndef SIW917_ELRS_LR2021_RX_FE_CAL_RETRY
// Quiet hot path: FE calibration is seeded for the FHSS range during Begin().
// Polling GetErrors() after every SetRx costs two extra SPI transactions per
// RX hop and can collide with telemetry/next-preamble timing.
#define SIW917_ELRS_LR2021_RX_FE_CAL_RETRY 0
#endif
#ifndef SIW917_ELRS_LR2021_CALIB_FE_POST_DELAY_US
#define SIW917_ELRS_LR2021_CALIB_FE_POST_DELAY_US 250
#endif
#ifndef SIW917_ELRS_LR2021_SKIP_PACKET_STATUS
#define SIW917_ELRS_LR2021_SKIP_PACKET_STATUS 1
#endif
#ifndef SIW917_ELRS_LR2021_FAST_SET_TX
#define SIW917_ELRS_LR2021_FAST_SET_TX 1
#endif
#ifndef SIW917_ELRS_LR2021_LINK_PROGRESS_STATUS_SPI
#define SIW917_ELRS_LR2021_LINK_PROGRESS_STATUS_SPI 0
#endif
#ifndef SIW917_ELRS_LR2021_RX_FIFO_AS_RX_DONE
#define SIW917_ELRS_LR2021_RX_FIFO_AS_RX_DONE 0
#endif
#ifndef SIW917_ELRS_LR2021_SCAN_TRACE
// Quiet timing: scan serial competes with SYNC acquisition at high rates.
#define SIW917_ELRS_LR2021_SCAN_TRACE 0
#endif
#ifndef SIW917_ELRS_LR2021_ROUTE_SCAN_DIAG_IRQS
#define SIW917_ELRS_LR2021_ROUTE_SCAN_DIAG_IRQS 0
#endif
#ifndef SIW917_ELRS_LR2021_LORA_SF5_SF6_MIN_PREAMBLE
#define SIW917_ELRS_LR2021_LORA_SF5_SF6_MIN_PREAMBLE 0
#endif
#ifndef SIW917_ELRS_LR2021_LORA_SF5_MIN_PREAMBLE
#define SIW917_ELRS_LR2021_LORA_SF5_MIN_PREAMBLE                              \
  SIW917_ELRS_LR2021_LORA_SF5_SF6_MIN_PREAMBLE
#endif
#ifndef SIW917_ELRS_LR2021_LORA_SF5_FREQ_RANGE
// LR20xx LoRa detector frequency-error range for SF5 only:
// 0 = +/-BW/4, 1 = +/-BW/3, 2 = +/-BW/2.
#define SIW917_ELRS_LR2021_LORA_SF5_FREQ_RANGE 1
#endif
#ifndef SIW917_ELRS_LR2021_LORA_SF5_SX1276_COMPAT
#define SIW917_ELRS_LR2021_LORA_SF5_SX1276_COMPAT 0
#endif
#ifndef SIW917_ELRS_LR2021_LORA_COMPAT_EXT_SYNCWORD
#define SIW917_ELRS_LR2021_LORA_COMPAT_EXT_SYNCWORD 0
#endif
#ifndef SIW917_ELRS_LR2021_SF5_OTA8_FORCE_CR_LI_4_8
#define SIW917_ELRS_LR2021_SF5_OTA8_FORCE_CR_LI_4_8 0
#endif
#ifndef SIW917_ELRS_LR2021_FORCE_2G4_DOMAIN
#define SIW917_ELRS_LR2021_FORCE_2G4_DOMAIN 1
#endif
#ifndef SIW917_ELRS_LR2021_FREQ_OFFSET_PPM_SWEEP
#define SIW917_ELRS_LR2021_FREQ_OFFSET_PPM_SWEEP 0
#endif
#ifndef SIW917_ELRS_LR2021_FREQ_OFFSET_SWEEP_DWELL_MS
#define SIW917_ELRS_LR2021_FREQ_OFFSET_SWEEP_DWELL_MS 2500
#endif
#ifndef SIW917_ELRS_LR2021_FREQ_OFFSET_FIXED_PPM
#define SIW917_ELRS_LR2021_FREQ_OFFSET_FIXED_PPM 0
#endif
#ifndef SIW917_ELRS_LR2021_FORCE_LORA_DETECTOR_DISABLE
#define SIW917_ELRS_LR2021_FORCE_LORA_DETECTOR_DISABLE 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_DETECT_ON_SYNCWORD
#define SIW917_ELRS_LR2021_GFSK_DETECT_ON_SYNCWORD 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_SET_WHITENING_PARAMS
#define SIW917_ELRS_LR2021_GFSK_SET_WHITENING_PARAMS 1
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_SKIP_IMMEDIATE_TOCK
#define SIW917_ELRS_LR2021_GFSK_SKIP_IMMEDIATE_TOCK 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_TENTATIVE_NONCE_RESYNC
#define SIW917_ELRS_LR2021_GFSK_TENTATIVE_NONCE_RESYNC 1
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_NONCE_RESYNC_MAX_DELTA
#define SIW917_ELRS_LR2021_GFSK_NONCE_RESYNC_MAX_DELTA 8
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_CONNECTED_NONCE_RESYNC
#define SIW917_ELRS_LR2021_GFSK_CONNECTED_NONCE_RESYNC 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_CONNECTED_NONCE_RESYNC_MAX_DELTA
#define SIW917_ELRS_LR2021_GFSK_CONNECTED_NONCE_RESYNC_MAX_DELTA 16
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_SOFT_SYNC_RESYNC
#define SIW917_ELRS_LR2021_GFSK_SOFT_SYNC_RESYNC 1
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_RESYNC_ADJUST_FHSS
#define SIW917_ELRS_LR2021_GFSK_RESYNC_ADJUST_FHSS 1
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_RESYNC_SKIP_PFD
#define SIW917_ELRS_LR2021_GFSK_RESYNC_SKIP_PFD 1
#endif
#ifndef SIW917_ELRS_LR2021_PERIOD_NORMALIZE_PFD
#define SIW917_ELRS_LR2021_PERIOD_NORMALIZE_PFD 1
#endif
#ifndef SIW917_ELRS_LR2021_LORA_FAIL_DUMP
#define SIW917_ELRS_LR2021_LORA_FAIL_DUMP 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_FAIL_DUMP
#define SIW917_ELRS_LR2021_GFSK_FAIL_DUMP 0
#endif
#ifndef SIW917_ELRS_DIO_EDGE_TIMESTAMPS
// Quiet timing: do not sample micros() on every DIO edge during RF runs.
#define SIW917_ELRS_DIO_EDGE_TIMESTAMPS 0
#endif
#ifndef SIW917_ELRS_DIO_PFD_TIMESTAMP
// Keep PFD aligned with packet processing, not raw GPIO edge.
#define SIW917_ELRS_DIO_PFD_TIMESTAMP 0
#endif
#ifndef SIW917_ELRS_DIO_STATS_DIAG
#define SIW917_ELRS_DIO_STATS_DIAG 0
#endif
#ifndef SIW917_ELRS_LUA_PROGRESS_DIAG
#define SIW917_ELRS_LUA_PROGRESS_DIAG 0
#endif
#ifndef SIW917_ELRS_LINK_PROGRESS_DIAG
#define SIW917_ELRS_LINK_PROGRESS_DIAG 0
#endif

// Include Arduino compatibility layer
#include "Arduino.h"

// Include SiW917 timing profile before defining hot-path placement attributes.
#include "siw917_elrs_timing.h"

// Include hardware pin definitions
#include "hardware.h"

// Word alignment for SPI buffers
#ifndef WORD_ALIGNED_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#endif

#ifndef WORD_PADDED
#define WORD_PADDED(size) (((size)+3) & ~3)
#endif

// Keep upstream ELRS annotations parse-compatible. SiW917 RAM placement is
// applied with SIW917_ELRS_RAMFUNC_ATTR on targeted non-inline hot functions;
// using ICACHE_RAM_ATTR globally also marks inline header helpers and triggers
// C++ section conflicts.
#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// PROGMEM (no-op on SiW917 - unified memory space)
#define PROGMEM

// General features
#define LED_MAX_BRIGHTNESS 50

// LR1121 uses Hz directly for frequency (no frequency step conversion)
// Note: FREQ_STEP is also defined in LR1121_Regs.h with a formula, 
// so we don't define it here to avoid conflicts
