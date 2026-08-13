#include "TXOTAConnector.h"

#include "common.h"
#include "stubborn_sender.h"

#include <cstring>

extern StubbornSender DataUlSender;

TXOTAConnector::TXOTAConnector() {
  addDevice(CRSF_ADDRESS_CRSF_RECEIVER);
  addDevice(CRSF_ADDRESS_FLIGHT_CONTROLLER);
}

bool TXOTAConnector::queuePayload(const uint8_t *payload, uint8_t length) {
  if (payload == nullptr || length == 0U || length > ELRS_DATA_UL_BUFFER) {
    return false;
  }

  if (currentTransmissionLength == 0U) {
    for (uint8_t i = 0; i < length; ++i) {
      currentTransmissionBuffer[i] = payload[i];
    }
    currentTransmissionLength = length;
    return true;
  }

  bool queued = false;
  outputQueue.lock();
  if (outputQueue.ensure((uint8_t)(length + 1U))) {
    outputQueue.push(length);
    outputQueue.pushBytes(payload, length);
    queued = true;
  }
  outputQueue.unlock();
  return queued;
}

void TXOTAConnector::pumpSender() {
  static bool transferActive = false;

  if (transferActive) {
    unlockMessage();
    transferActive = false;
  }

  if (!transferActive && currentTransmissionLength > 0U) {
    DataUlSender.SetDataToTransmit(currentTransmissionBuffer,
                                   currentTransmissionLength);
    transferActive = true;
  }
}

bool TXOTAConnector::takeQueuedPayload(uint8_t *out, uint8_t *len,
                                       uint8_t maxLen) {
  if (out == nullptr || len == nullptr) {
    if (len != nullptr) {
      *len = 0;
    }
    return false;
  }

  while (currentTransmissionLength > 0U) {
    if (currentTransmissionLength <= maxLen) {
      memcpy(out, currentTransmissionBuffer, currentTransmissionLength);
      *len = currentTransmissionLength;
      unlockMessage();
      return true;
    }
    /* Oversized CRSF (Lua param dump) must not wedge the queue. */
    unlockMessage();
  }

  *len = 0;
  return false;
}

void TXOTAConnector::resetOutputQueue() {
  outputQueue.flush();
  currentTransmissionLength = 0;
}

void TXOTAConnector::unlockMessage() {
  if (outputQueue.size() > 0U) {
    outputQueue.lock();
    currentTransmissionLength = outputQueue.pop();
    outputQueue.popBytes(currentTransmissionBuffer, currentTransmissionLength);
    outputQueue.unlock();
  } else {
    currentTransmissionLength = 0;
  }
}

void TXOTAConnector::forwardMessage(const crsf_header_t *message) {
  if (connectionState != connected) {
    return;
  }

  const uint8_t length =
      (uint8_t)(message->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES);
  (void)queuePayload(reinterpret_cast<const uint8_t *>(message), length);
}
