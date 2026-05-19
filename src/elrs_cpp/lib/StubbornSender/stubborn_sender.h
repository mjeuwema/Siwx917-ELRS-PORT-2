#pragma once

#include <cstdint>

// The number of times to resend the same package index before going to RESYNC
#define SSENDER_MAX_MISSED_PACKETS 20

typedef enum {
    SENDER_IDLE = 0,
    SEND_PENDING,
    SENDING,
    WAIT_UNTIL_NEXT_CONFIRM,
    RESYNC,
    RESYNC_THEN_SEND, // perform a RESYNC then go to SENDING
} stubborn_sender_state_e;

struct StubbornSenderPreparedPayload
{
    const uint8_t *sourceData;
    uint8_t sourceLength;
    uint8_t sourceOffset;
    uint8_t sourcePackage;
    uint8_t packageIndex;
    uint8_t bytesLastPayload;
    stubborn_sender_state_e sourceState;
    stubborn_sender_state_e committedState;
};

class StubbornSender
{
public:
    StubbornSender();
    void setMaxPackageIndex(uint8_t maxPackageIndex);
    void ResetState();
    void UpdateTelemetryRate(uint16_t airRate, uint8_t tlmRatio, uint8_t tlmBurst);
    void SetDataToTransmit(uint8_t* dataToTransmit, uint8_t lengthToTransmit);
    uint8_t GetCurrentPayload(uint8_t *outData, uint8_t maxLen);
    uint8_t PrepareCurrentPayload(uint8_t *outData, uint8_t maxLen, StubbornSenderPreparedPayload *prepared) const;
    bool CommitPreparedPayload(const StubbornSenderPreparedPayload &prepared);
    void ConfirmCurrentPayload(bool telemetryConfirmValue);
    bool IsActive() const { return senderState != SENDER_IDLE; }
    uint16_t GetMaxPacketsBeforeResync() const { return maxWaitCount; }
    uint8_t GetState() const { return (uint8_t)senderState; }
    uint8_t GetCurrentPackage() const { return currentPackage; }
    uint8_t GetCurrentOffset() const { return currentOffset; }
    uint8_t GetBytesLastPayload() const { return bytesLastPayload; }
    uint16_t GetWaitCount() const { return waitCount; }
    bool GetExpectedAck() const { return telemetryConfirmExpectedValue; }
private:
    uint8_t *data;
    uint8_t length;
    uint8_t currentOffset;
    uint8_t bytesLastPayload;
    uint8_t currentPackage;
    bool telemetryConfirmExpectedValue;
    uint16_t waitCount;
    uint16_t maxWaitCount;
    uint8_t maxPackageIndex;
    stubborn_sender_state_e senderState;
};
