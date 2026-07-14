#pragma once

#include "CRSFConnector.h"
#include "CRSFEndpoint.h"
#include "crc.h"

class CRSFRouter final {
public:
  CRSFRouter() = default;

  void addConnector(CRSFConnector *connector);
  void removeConnector(CRSFConnector *connector);
  void addEndpoint(CRSFEndpoint *endpoint);

  void processMessage(CRSFConnector *connector,
                      const crsf_header_t *message) const;
  void deliverMessage(const CRSFConnector *connector,
                      const crsf_header_t *message) const;
  void deliverMessageTo(crsf_addr_e destination,
                        const crsf_header_t *message) const;

  void SetHeaderAndCrc(crsf_header_t *frame, crsf_frame_type_e frameType,
                       uint8_t frameSize);
  void SetExtendedHeaderAndCrc(crsf_ext_header_t *frame,
                               crsf_frame_type_e frameType,
                               uint8_t frameSize, crsf_addr_e destAddr,
                               crsf_addr_e origAddr);
  uint8_t getConnectorMaxPacketSize(crsf_addr_e origin) const;

  GENERIC_CRC8 crsf_crc = GENERIC_CRC8(CRSF_CRC_POLY);

private:
  CRSFConnector *connectors[4] = {};
  CRSFEndpoint *endpoints[4] = {};
  uint8_t connectorCount = 0;
  uint8_t endpointCount = 0;
};

extern CRSFRouter crsfRouter;
extern elrsLinkStatistics_t linkStats;
