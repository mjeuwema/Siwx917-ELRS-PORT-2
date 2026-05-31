#pragma once

#include "CRSFEndpoint.h"

class SiW917RXEndpoint final : public CRSFEndpoint {
public:
  SiW917RXEndpoint();

  bool handleRaw(const crsf_header_t *message) override;
  void handleMessage(const crsf_header_t *message) override;
  void registerParameters() override;
  void updateParameters() override;

  void processPending(bool telemetryBusy);
  bool consumeSerialApplyRequest();
  void requestActiveModeRefresh();

protected:
  void devicePingCalled() override { updateParameters(); }

private:
  void requestConfigSave(bool applySerialAfterSave = false);
  void handleWiFiCommand(propertiesCommon *item, int32_t arg);
  void handleBindCommand(propertiesCommon *item, int32_t arg);

  bool configSavePending = false;
  bool serialApplyPending = false;
  bool serialApplyRequested = false;
  bool activeModeRefreshPending = false;
  bool wifiPending = false;
  bool bindPending = false;
  uint32_t configSaveAtMs = 0;
  uint32_t pendingActionAtMs = 0;
};
