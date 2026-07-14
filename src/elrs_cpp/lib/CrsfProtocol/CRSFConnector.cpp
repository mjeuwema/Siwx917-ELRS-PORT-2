#include "CRSFConnector.h"

void CRSFConnector::addDevice(crsf_addr_e device_id) {
  if (device_id == 0) {
    return;
  }

  for (uint8_t i = 0; i < deviceCount; ++i) {
    if (devices[i] == device_id) {
      return;
    }
  }

  if (deviceCount < sizeof(devices) / sizeof(devices[0])) {
    devices[deviceCount++] = device_id;
  }
}

bool CRSFConnector::forwardsTo(crsf_addr_e device_id) {
  for (uint8_t i = 0; i < deviceCount; ++i) {
    if (devices[i] == device_id) {
      return true;
    }
  }
  return false;
}
