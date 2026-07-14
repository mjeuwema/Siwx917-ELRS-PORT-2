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

// Select RX or TX role at build time.
#if defined(SIW917_ELRS_TARGET_TX)
#define TARGET_TX 1
#else
#define TARGET_RX 1
#endif

// Radio type - LR1121 sub-GHz + 2.4GHz dual-band
#define RADIO_LR1121 1

// Regulatory domain (can be overridden)
#ifndef Regulatory_Domain_FCC_915
#define Regulatory_Domain_FCC_915 1
#endif

// OTA Version ID - must match TX for compatibility
#define OTA_VERSION_ID 4

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
