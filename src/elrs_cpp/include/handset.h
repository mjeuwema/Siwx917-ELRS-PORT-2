#pragma once

#include "common.h"

typedef void (*RcChannelsOverrideCallback_fn)(uint32_t channels[],
                                              size_t channelCnt);

extern bool isArmed;

class Handset {
public:
  virtual void Begin() = 0;
  virtual void End() = 0;

  void setRcChannelsOverrideCallback(RcChannelsOverrideCallback_fn callback) {
    RcChannelsOverrideCallback = callback;
  }

  void setRCDataCallback(void (*callback)()) { RCdataCallback = callback; }

  void registerCallbacks(void (*connectedCallback)(),
                         void (*disconnectedCallback)()) {
    connected = connectedCallback;
    disconnected = disconnectedCallback;
  }

  virtual void handleInput() = 0;

  virtual void setPacketInterval(int32_t PacketInterval) {
    RequestedRCpacketInterval = PacketInterval;
  }

  virtual uint8_t GetMaxPacketBytes() const { return 255; }
  virtual int getMinPacketInterval() const { return 1; }
  virtual bool IsArmed() const { return isArmed; }
  virtual void JustSentRFpacket() {}

  void PerformChannelOverrides(uint32_t channels[], size_t channelCount) {
    if (RcChannelsOverrideCallback) {
      RcChannelsOverrideCallback(channels, channelCount);
    }
  }

  void RCDataReceived(uint32_t channels[], size_t channelCount) {
    RCdataLastRecv = micros();
    for (unsigned ch = 0; ch < channelCount; ++ch) {
      ChannelData[ch] = channels[ch];
    }
    if (RCdataCallback) {
      RCdataCallback();
    }
  }

  uint32_t GetRCdataLastRecv() const { return RCdataLastRecv; }

protected:
  virtual ~Handset() = default;

  bool controllerConnected = false;
  RcChannelsOverrideCallback_fn RcChannelsOverrideCallback = nullptr;
  void (*RCdataCallback)() = nullptr;
  void (*disconnected)() = nullptr;
  void (*connected)() = nullptr;

  int32_t RequestedRCpacketInterval = 5000;

private:
  volatile uint32_t RCdataLastRecv = 0;
};

#ifdef TARGET_TX
extern Handset *handset;
#endif
