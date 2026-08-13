#pragma once

#include "CRSFConnector.h"
#include "FIFO.h"

#define TELEMETRY_FIFO_SIZE 2048
using TelemetryFifo = FIFO<TELEMETRY_FIFO_SIZE>;

class RXOTAConnector : public CRSFConnector {
public:
  RXOTAConnector();

  void forwardMessage(const crsf_header_t *message) override;
  bool GetNextPayload(uint8_t *nextPayloadSize, uint8_t *payloadData);
  uint8_t GetFifoFullPct() const {
    return (TELEMETRY_FIFO_SIZE - messagePayloads.free()) * 100 /
           TELEMETRY_FIFO_SIZE;
  }
  bool IsEmpty() const { return messagePayloads.size() == 0; }

private:
  TelemetryFifo messagePayloads;
  uint8_t prioritizedCount = 0;
};
