#pragma once

#include "crsf_protocol.h"

class CRSFConnector {
public:
  CRSFConnector() = default;
  virtual ~CRSFConnector() = default;

  void addDevice(crsf_addr_e device_id);
  bool forwardsTo(crsf_addr_e device_id);

  virtual void forwardMessage(const crsf_header_t *message) = 0;
  virtual uint8_t GetMaxPacketBytes() const { return CRSF_MAX_PACKET_LEN; }

private:
  crsf_addr_e devices[8] = {};
  uint8_t deviceCount = 0;
};
