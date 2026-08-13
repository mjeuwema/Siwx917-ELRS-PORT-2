#pragma once

#include "CRSFConnector.h"
#include "FIFO.h"
#include "telemetry_protocol.h"

class TXOTAConnector final : public CRSFConnector {
public:
  TXOTAConnector();

  void forwardMessage(const crsf_header_t *message) override;
  bool queuePayload(const uint8_t *payload, uint8_t length);

  void resetOutputQueue();
  void pumpSender();
  bool takeQueuedPayload(uint8_t *out, uint8_t *len, uint8_t maxLen);

private:
  void unlockMessage();

  static constexpr auto MSP_SERIAL_OUT_FIFO_SIZE = 256U;
  FIFO<MSP_SERIAL_OUT_FIFO_SIZE> outputQueue;
  uint8_t currentTransmissionBuffer[ELRS_DATA_UL_BUFFER] = {};
  uint8_t currentTransmissionLength = 0;
};
