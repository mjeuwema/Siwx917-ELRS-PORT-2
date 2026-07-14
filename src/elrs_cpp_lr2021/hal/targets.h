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
#define SIW917_ELRS_FHSS_SET_FREQ_RX 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_HOP_SET_RX
// LR20xx does not have LR1121's combined SetRfFrequency_SetRx command. Its
// SetRfFrequency command leaves a GFSK receiver out of continuous RX, so
// explicitly re-arm after each GFSK FHSS retune.
#define SIW917_ELRS_LR2021_GFSK_HOP_SET_RX 1
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
#define SIW917_ELRS_LR2021_TXDONE_WATCHDOG_PEEK_IRQ 1
#endif
#ifndef SIW917_ELRS_LR2021_TXDONE_WATCHDOG_EVENT_LOG
#define SIW917_ELRS_LR2021_TXDONE_WATCHDOG_EVENT_LOG 1
#endif
#ifndef SIW917_ELRS_LR2021_TXDONE_LORA_GUARD_US
#define SIW917_ELRS_LR2021_TXDONE_LORA_GUARD_US 350U
#endif
#ifndef SIW917_ELRS_TLM_GAP_DIAG
#define SIW917_ELRS_TLM_GAP_DIAG 1
#endif
#ifndef SIW917_ELRS_TLM_TURNAROUND_TRACE
// Keep the healthy telemetry turnaround free of timestamp reads and volatile
// bookkeeping. Re-enable only for a focused telemetry-loss investigation.
#define SIW917_ELRS_TLM_TURNAROUND_TRACE 0
#endif
#ifndef SIW917_ELRS_TLM_MISS_IRQ_TRACE
// This trace keeps per-slot state even when it never prints. Keep it out of
// normal RF runs; it can be re-enabled for a focused loss investigation.
#define SIW917_ELRS_TLM_MISS_IRQ_TRACE 0
#endif
#ifndef SIW917_ELRS_TLM_GAP_PERIOD_MS
// Keep RF timing quiet by default. TLMGAP still prints on actual local
// telemetry stalls; periodic TLMSTAT lines are too expensive at high rates.
#define SIW917_ELRS_TLM_GAP_PERIOD_MS 0
#endif
#ifndef SIW917_ELRS_PREBUILD_TLM_PACKET
// Keep telemetry construction in the ELRS tock path. This matches upstream
// and avoids publishing a nonce-specific DVDA packet from main-loop context.
#define SIW917_ELRS_PREBUILD_TLM_PACKET 0
#endif
#ifndef SIW917_ELRS_LR2021_PREENCODE_TLM
// Upstream encodes the final packet in TXnb. Keep DK500 on that same path so
// the encoded bytes and the live OtaNonce can never come from different slots.
#define SIW917_ELRS_LR2021_PREENCODE_TLM 0
#endif
#ifndef SIW917_ELRS_LR2021_DVDA_PREHOP_TLM
// Pre-hopping after RX_DONE made the previously working DK250 downlink
// invisible in v207. Preserve upstream's at-tock FHSS ordering.
#define SIW917_ELRS_LR2021_DVDA_PREHOP_TLM 0
#endif
#ifndef SIW917_ELRS_LR2021_DVDA_NONCE_FREE_TLM_PROBE
// Diagnostic only: classify DVDA downlinks as SYNC so the OTA CRC does not
// include OtaNonce. The TX records a valid downlink before payload dispatch,
// cleanly separating nonce-phase failures from RF/PHY failures.
#define SIW917_ELRS_LR2021_DVDA_NONCE_FREE_TLM_PROBE 0
#endif
#ifndef SIW917_ELRS_LR2021_DVDA_DEFER_TLM_HOP_PROBE
// Diagnostic only: send DVDA telemetry on the completed uplink frequency,
// then advance FHSS at TX_DONE before returning to RX.
#define SIW917_ELRS_LR2021_DVDA_DEFER_TLM_HOP_PROBE 0
#endif
#ifndef SIW917_ELRS_LR2021_FORCE_RF_TLM
#define SIW917_ELRS_LR2021_FORCE_RF_TLM 1
#endif
#ifndef SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX
// Hardware AutoRxTx does not preserve the LR2021 continuous-RX/FHSS flow:
// v192 repeatedly relocked with zero LQ. Keep explicit TX_DONE -> SetRx.
#define SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX 0
#endif
#ifndef SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX_LOCKED_ONLY
// Keep acquisition on the established manual TX -> RX path. Once the ELRS
// timer is locked, use the LR20xx hardware handoff for the tight downlink
// turnaround.
#define SIW917_ELRS_LR2021_AUTO_RX_AFTER_TX_LOCKED_ONLY 1
#endif
#ifndef SIW917_ELRS_LR2021_RX_TX_FALLBACK_FS
// Preserve upstream ELRS's TX-to-RX turnaround contract. FS leaves the
// synthesizer running after a telemetry TX, so the following SetRx does not
// have to restart from RC standby inside the next uplink window.
#define SIW917_ELRS_LR2021_RX_TX_FALLBACK_FS 1
#endif
#ifndef SIW917_ELRS_LR2021_TLM_AFTER_TIMER_LOCK
// The shared telemetry-slot predicate now matches the working LR1121 path and
// requires tim_locked. Keep this legacy secondary gate disabled.
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
// CalibFE is invalid in RX/TX. Band configuration seeds all three on-chip
// calibration slots from FS, so FHSS retunes must remain calibration-free.
#define SIW917_ELRS_LR2021_RUNTIME_CALIB_FE 0
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
#ifndef SIW917_ELRS_LR2021_SF5_SCAN_DIAG
// Keep acquisition quiet; enable only for a targeted SF5 capture.
#define SIW917_ELRS_LR2021_SF5_SCAN_DIAG 0
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
#ifndef SIW917_ELRS_LR2021_LORA_SF5_SX1276_COMPAT
// The ExpressLRS transmitter is LR1121-based, so preserve the native state
// established by SetModulationParams. Enable only for an SX1276-generation
// peer that requires Semtech's compatibility workaround.
#define SIW917_ELRS_LR2021_LORA_SF5_SX1276_COMPAT 0
#endif
#ifndef SIW917_ELRS_LR2021_LORA_COMPAT_EXT_SYNCWORD
#define SIW917_ELRS_LR2021_LORA_COMPAT_EXT_SYNCWORD 0
#endif
#ifndef SIW917_ELRS_LR2021_SUBGHZ_SF5_EXT_SYNCWORD
// Semtech's LR20xx driver uses the standard one-byte 0x12 private-network
// syncword with the mandatory SF5/SF6 compatibility field.
#define SIW917_ELRS_LR2021_SUBGHZ_SF5_EXT_SYNCWORD 0
#endif
#ifndef SIW917_ELRS_LR2021_SUBGHZ_SF5_FIFO_HANDOFF
// The normal LR20xx RX_DONE path owns packet handoff. The FIFO-threshold mode
// is diagnostic-only and is not part of Semtech's LoRa setup sequence.
#define SIW917_ELRS_LR2021_SUBGHZ_SF5_FIFO_HANDOFF 0
#endif
#ifndef SIW917_ELRS_LR2021_FORCE_2G4_DOMAIN
// Use the configured low-band regulatory domain for sub-GHz testing. The
// diagnostic 2.4GHz override bypasses FCC915 rate scanning entirely.
#define SIW917_ELRS_LR2021_FORCE_2G4_DOMAIN 0
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
// K1000 acquisition was proven with the LR11xx-compatible 8-bit detector in
// v105-v142. Syncword-only detection produced no tentative lock in v104/v187.
#define SIW917_ELRS_LR2021_GFSK_DETECT_ON_SYNCWORD 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_SET_WHITENING_PARAMS
// Preserve the LR2021 reset/default whitening parameters. Explicitly writing
// type=LR11xx, seed=0x01FF eliminated K1000 acquisition in v103 and v122.
#define SIW917_ELRS_LR2021_GFSK_SET_WHITENING_PARAMS 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_TXDONE_FAST_PATH
// GFSK needs the shortest TX-to-RX handoff at dense telemetry ratios. TX state
// is armed immediately before SetTx so an early TX_DONE cannot race the launch
// bookkeeping, and the ISR clears the IRQ before returning the radio to RX.
#define SIW917_ELRS_LR2021_GFSK_TXDONE_FAST_PATH 1
#endif
#ifndef SIW917_ELRS_LR2021_LORA_TXDONE_FAST_PATH
// SF5/BW500 leaves less than one millisecond after downlink airtime at 250 Hz.
// Use the same fresh-edge, clear-before-RX handoff already proven for GFSK.
#define SIW917_ELRS_LR2021_LORA_TXDONE_FAST_PATH 1
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
// Match upstream ELRS: a mismatched connected SYNC must pass through
// TentativeConnection() so nonce/FHSS and the receiver timer are realigned
// together. Updating nonce/FHSS alone can leave downlink slots out of phase.
#define SIW917_ELRS_LR2021_GFSK_SOFT_SYNC_RESYNC 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_RESYNC_ADJUST_FHSS
#define SIW917_ELRS_LR2021_GFSK_RESYNC_ADJUST_FHSS 1
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_RESYNC_SKIP_PFD
#define SIW917_ELRS_LR2021_GFSK_RESYNC_SKIP_PFD 1
#endif
#ifndef SIW917_ELRS_LR2021_PERIOD_NORMALIZE_PFD
#define SIW917_ELRS_LR2021_PERIOD_NORMALIZE_PFD 0
#endif
#ifndef SIW917_ELRS_LR2021_LORA_FAIL_DUMP
#define SIW917_ELRS_LR2021_LORA_FAIL_DUMP 0
#endif
#ifndef SIW917_ELRS_LR2021_GFSK_FAIL_DUMP
#define SIW917_ELRS_LR2021_GFSK_FAIL_DUMP 0
#endif
#ifndef SIW917_ELRS_DIO_EDGE_TIMESTAMPS
// Keep the locked RF path quiet; PFD uses the upstream processing timestamp.
#define SIW917_ELRS_DIO_EDGE_TIMESTAMPS 0
#endif
#ifndef SIW917_ELRS_DIO_PFD_TIMESTAMP
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
