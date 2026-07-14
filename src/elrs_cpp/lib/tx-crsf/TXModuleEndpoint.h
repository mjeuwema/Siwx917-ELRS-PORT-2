#pragma once

#include "CRSFEndpoint.h"

enum warningFlags {
  LUA_FLAG_CONNECTED = 0,
  LUA_FLAG_STATUS1,
  LUA_FLAG_MODEL_MATCH,
  LUA_FLAG_ISARMED,
  LUA_FLAG_WARNING1,
  LUA_FLAG_ERROR_CONNECTED,
  LUA_FLAG_ERROR_BAUDRATE,
  LUA_FLAG_CRITICAL_WARNING2,
};

class TXModuleEndpoint final : public CRSFEndpoint {
public:
  TXModuleEndpoint() : CRSFEndpoint(CRSF_ADDRESS_CRSF_TRANSMITTER) {}
  ~TXModuleEndpoint() override = default;

  void begin();

  bool handleRaw(const crsf_header_t *message) override;
  void handleMessage(const crsf_header_t *message) override;
  void RcPacketToChannelsData(const crsf_header_t *message);

  void updateFolderNamesAndVisibility();
  void registerParameters() override;
  void updateParameters() override;

  uint8_t getSyncUID5() const;
  void flagConnectedWarning();

  uint8_t modelId = 0xFF;
  bool modelMatchEnabled = false;

protected:
  void devicePingCalled() override;

  void supressCriticalErrors();
  void setWarningFlag(warningFlags flag, bool value);
  void sendELRSstatus(crsf_addr_e origin);

private:
  uint8_t getModelIdForSync() const;
  void scheduleModelIdSync();
  void updateModelID();
  void handleSimpleSendCmd(propertiesCommon *item, int32_t arg);

  bool parametersRegistered = false;
  uint32_t bindCommandStartedAtMs = 0;
  uint8_t luaWarningFlags = 0;
};

extern TXModuleEndpoint crsfTransmitter;
