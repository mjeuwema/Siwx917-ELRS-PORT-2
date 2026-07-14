#pragma once

#include "CRSFConnector.h"
#include "crsf_protocol.h"
#include "handset.h"

class CRSFHandset final : public Handset, public CRSFConnector {
public:
  static uint32_t GoodPktsCountResult;
  static uint32_t BadPktsCountResult;

  void Begin() override;
  void End() override;

  void forwardMessage(const crsf_header_t *message) override;

  void handleInput() override;
  void handleOutput(uint32_t receivedBytes);

  void setPacketInterval(int32_t PacketInterval) override;
  void JustSentRFpacket() override;

  uint8_t GetMaxPacketBytes() const override { return maxPacketBytes; }
  int getMinPacketInterval() const override;

private:
  uint8_t inBuffer[CRSF_MAX_PACKET_LEN] = {};

  volatile uint32_t dataLastRecv = 0;
  volatile int32_t OpenTXsyncOffset = 0;
  volatile int32_t OpenTXsyncWindow = 0;
  volatile int32_t OpenTXsyncWindowSize = 0;
  uint32_t OpenTXsyncLastSent = 0;

  uint8_t SerialInPacketPtr = 0;
  bool transmitting = false;
  uint32_t GoodPktsCount = 0;
  uint32_t BadPktsCount = 0;
  uint32_t RxBytesObserved = 0;
  uint32_t UARTwdtLastChecked = 0;
  bool UARTidleReported = false;
  uint8_t maxPacketBytes = CRSF_MAX_PACKET_LEN;
  uint8_t maxPeriodBytes = CRSF_MAX_PACKET_LEN;

  static uint8_t UARTcurrentBaudIdx;
  static uint32_t UARTrequestedBaud;

  void sendSyncPacketToTX();
  void adjustMaxPacketSize();
  void alignBufferToSync(uint8_t startIdx);
  bool ProcessPacket();
  bool UARTwdt();
  uint32_t autobaud();
  void flush_port_input();
};
