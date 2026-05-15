#include "CRSFRouter.h"

#include "logging.h"

#define CRSF_ROUTER_DIAG 0

void CRSFRouter::addConnector(CRSFConnector *connector) {
  if (connector == nullptr) {
    return;
  }
  for (uint8_t i = 0; i < connectorCount; ++i) {
    if (connectors[i] == connector) {
      return;
    }
  }
  if (connectorCount < sizeof(connectors) / sizeof(connectors[0])) {
    connectors[connectorCount++] = connector;
  }
}

void CRSFRouter::removeConnector(CRSFConnector *connector) {
  for (uint8_t i = 0; i < connectorCount; ++i) {
    if (connectors[i] == connector) {
      for (uint8_t j = i; j + 1 < connectorCount; ++j) {
        connectors[j] = connectors[j + 1];
      }
      connectors[--connectorCount] = nullptr;
      return;
    }
  }
}

void CRSFRouter::addEndpoint(CRSFEndpoint *endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  for (uint8_t i = 0; i < endpointCount; ++i) {
    if (endpoints[i] == endpoint) {
      return;
    }
  }
  if (endpointCount < sizeof(endpoints) / sizeof(endpoints[0])) {
    endpoints[endpointCount++] = endpoint;
  }
}

void CRSFRouter::processMessage(CRSFConnector *connector,
                                const crsf_header_t *message) const {
  if (message->type == CRSF_FRAMETYPE_HEARTBEAT) {
    if (connector != nullptr) {
      connector->addDevice((crsf_addr_e)((uint8_t *)message)[sizeof(crsf_header_t) + 1]);
    }
    return;
  }

  const auto *extMessage = (const crsf_ext_header_t *)message;
  if (connector != nullptr && message->type >= CRSF_FRAMETYPE_DEVICE_PING) {
    connector->addDevice(extMessage->orig_addr);
  }

#if CRSF_ROUTER_DIAG
  if (message->type >= CRSF_FRAMETYPE_DEVICE_PING) {
    DBGLN("[CRSF_ROUTER] UL type=0x%02X len=%u dest=0x%02X orig=0x%02X endpoints=%u connectors=%u",
          message->type, message->frame_size, extMessage->dest_addr,
          extMessage->orig_addr, endpointCount, connectorCount);
  }
#endif

  for (uint8_t i = 0; i < endpointCount; ++i) {
    auto *endpoint = endpoints[i];
    if (endpoint->handleRaw(message)) {
#if CRSF_ROUTER_DIAG
      DBGLN("[CRSF_ROUTER] handled raw type=0x%02X", message->type);
#endif
      return;
    }

    const crsf_addr_e deviceId = endpoint->getDeviceId();
    const bool shouldHandle =
        message->type < CRSF_FRAMETYPE_DEVICE_PING ||
        extMessage->dest_addr == deviceId ||
        extMessage->dest_addr == CRSF_ADDRESS_BROADCAST;
#if CRSF_ROUTER_DIAG
    if (message->type >= CRSF_FRAMETYPE_DEVICE_PING) {
      DBGLN("[CRSF_ROUTER] endpoint check idx=%u dev=0x%02X pass=%u",
            i, deviceId, shouldHandle ? 1 : 0);
    }
#endif
    if (shouldHandle) {
      endpoint->handleMessage(message);
      if (message->type >= CRSF_FRAMETYPE_DEVICE_PING &&
          extMessage->dest_addr == deviceId) {
#if CRSF_ROUTER_DIAG
        DBGLN("[CRSF_ROUTER] endpoint consumed type=0x%02X dev=0x%02X",
              message->type, deviceId);
#endif
        return;
      }
    }
  }

#if CRSF_ROUTER_DIAG
  if (message->type >= CRSF_FRAMETYPE_DEVICE_PING) {
    DBGLN("[CRSF_ROUTER] fallthrough deliver type=0x%02X dest=0x%02X",
          message->type, extMessage->dest_addr);
  }
#endif
  deliverMessage(connector, message);
}

void CRSFRouter::deliverMessage(const CRSFConnector *connector,
                                const crsf_header_t *message) const {
  const auto *extMessage = (const crsf_ext_header_t *)message;

  if (message->type >= CRSF_FRAMETYPE_DEVICE_PING &&
      extMessage->dest_addr != CRSF_ADDRESS_BROADCAST) {
    for (uint8_t i = 0; i < connectorCount; ++i) {
      auto *other = connectors[i];
      if (other != connector && other->forwardsTo(extMessage->dest_addr)) {
        other->forwardMessage(message);
        return;
      }
    }
  }

  for (uint8_t i = 0; i < connectorCount; ++i) {
    auto *other = connectors[i];
    if (other != connector) {
      other->forwardMessage(message);
    }
  }
}

void CRSFRouter::deliverMessageTo(crsf_addr_e destination,
                                  const crsf_header_t *message) const {
  for (uint8_t i = 0; i < connectorCount; ++i) {
    auto *connector = connectors[i];
    if (destination == CRSF_ADDRESS_BROADCAST ||
        connector->forwardsTo(destination)) {
#if CRSF_ROUTER_DIAG
      DBGLN("[CRSF_ROUTER] DL type=0x%02X len=%u to=0x%02X via=%u",
            message->type, message->frame_size, destination, i);
#endif
      connector->forwardMessage(message);
      if (destination != CRSF_ADDRESS_BROADCAST) {
        return;
      }
    }
  }
}

void CRSFRouter::SetHeaderAndCrc(crsf_header_t *frame,
                                 crsf_frame_type_e frameType,
                                 uint8_t frameSize) {
  frame->sync_byte = CRSF_SYNC_BYTE;
  frame->frame_size = frameSize;
  frame->type = frameType;
  frame->payload[frameSize - CRSF_FRAME_NOT_COUNTED_BYTES] =
      crsf_crc.calc((uint8_t *)&frame->type, frameSize - 1);
}

void CRSFRouter::SetExtendedHeaderAndCrc(crsf_ext_header_t *frame,
                                         crsf_frame_type_e frameType,
                                         uint8_t frameSize,
                                         crsf_addr_e destAddr,
                                         crsf_addr_e origAddr) {
  frame->dest_addr = destAddr;
  frame->orig_addr = origAddr;
  SetHeaderAndCrc((crsf_header_t *)frame, frameType, frameSize);
}

uint8_t CRSFRouter::getConnectorMaxPacketSize(crsf_addr_e origin) const {
  for (uint8_t i = 0; i < connectorCount; ++i) {
    auto *connector = connectors[i];
    if (connector->forwardsTo(origin)) {
      return connector->GetMaxPacketBytes();
    }
  }
  return CRSF_MAX_PACKET_LEN;
}
