/**
 * @file SPIEx.cpp
 * @brief ELRS SPIEx implementation for SiW917
 *
 * Uses the lr1121_driver.c SPI functions for actual hardware access.
 */

#include "SPIEx.h"
#include "logging.h"
#include <string.h>

// Include our C SPI driver
extern "C" {
#include "lr1121_driver.h"
}

// Global instances
SPIClass SPI;
SPIExClass SPIEx;

namespace {
constexpr size_t kSpiScratchSize = 512;
uint8_t gTransferWriteDummy[kSpiScratchSize];
uint8_t gTransferReadBuf[kSpiScratchSize];
uint8_t gSpiexWriteDummy[kSpiScratchSize];

static inline uint8_t lr1121RadioForMask(uint8_t cs_mask) {
  return (cs_mask & SX12XX_Radio_2) ? LR1121_RADIO_2 : LR1121_RADIO_1;
}

static void transferSelectedRadio(uint8_t radio_mask, uint8_t *data,
                                  uint32_t size, bool reading) {
  lr1121_select_radio(radio_mask);
  lr1121_cs_assert();

  if (reading) {
    if (size <= sizeof(gTransferReadBuf)) {
      lr1121_spi_transfer(data, gTransferReadBuf, size);
      memcpy(data, gTransferReadBuf, size);
    } else {
      for (uint32_t i = 0; i < size; i++) {
        uint8_t rx = 0;
        lr1121_spi_transfer(&data[i], &rx, 1);
        data[i] = rx;
      }
    }
  } else {
    if (size <= sizeof(gSpiexWriteDummy)) {
      lr1121_spi_transfer(data, gSpiexWriteDummy, size);
    } else {
      for (uint32_t i = 0; i < size; i++) {
        uint8_t rx = 0;
        lr1121_spi_transfer(&data[i], &rx, 1);
      }
    }
  }

  lr1121_cs_deassert();
}
} // namespace

//-----------------------------------------------------------------------------
// SPIClass Implementation
//-----------------------------------------------------------------------------

uint8_t SPIClass::transfer(uint8_t data) {
  uint8_t rx = 0;
  lr1121_spi_transfer(&data, &rx, 1);
  return rx;
}

void SPIClass::transfer(void *buf, size_t count) {
  // In-place transfer
  uint8_t *data = (uint8_t *)buf;
  lr1121_spi_transfer(data, data, count);
}

void SPIClass::transferBytes(const uint8_t *tx, uint8_t *rx, size_t count) {
  if (count == 0)
    return;

  if (rx != nullptr) {
    // Full duplex: separate tx and rx buffers
    lr1121_spi_transfer(tx, rx, count);
  } else {
    // Write-only: use a shared dummy buffer so tx data is NOT overwritten
    // without putting a large scratch buffer on the task stack.
    if (count <= sizeof(gTransferWriteDummy)) {
      lr1121_spi_transfer(tx, gTransferWriteDummy, count);
    } else {
      for (size_t i = 0; i < count; i++) {
        uint8_t d = 0;
        lr1121_spi_transfer(&tx[i], &d, 1);
      }
    }
  }
}

//-----------------------------------------------------------------------------
// SPIExClass Implementation
//-----------------------------------------------------------------------------

void SPIExClass::write(uint8_t cs_mask, uint8_t *data, uint32_t size) {
  _transfer(cs_mask, data, size, false);
}

void SPIExClass::read(uint8_t cs_mask, uint8_t *data, uint32_t size) {
  _transfer(cs_mask, data, size, true);
}

void SPIExClass::_transfer(uint8_t cs_mask, uint8_t *data, uint32_t size,
                           bool reading) {
  if (size == 0)
    return;

  if (!reading && cs_mask == SX12XX_Radio_All &&
      SIW917_ELRS_UPSTREAM_DUAL_RADIO) {
    transferSelectedRadio(LR1121_RADIO_1, data, size, false);
    transferSelectedRadio(LR1121_RADIO_2, data, size, false);
    lr1121_select_radio(LR1121_RADIO_1);
    return;
  }

  transferSelectedRadio(lr1121RadioForMask(cs_mask), data, size, reading);
  lr1121_select_radio(LR1121_RADIO_1);
}
