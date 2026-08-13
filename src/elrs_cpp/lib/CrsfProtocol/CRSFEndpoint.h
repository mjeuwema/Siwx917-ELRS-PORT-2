#pragma once

#include "CRSFParameters.h"

#define MAX_CRSF_PARAMETERS 64

class CRSFEndpoint {
public:
  explicit CRSFEndpoint(crsf_addr_e device_id) : device_id(device_id) {}
  virtual ~CRSFEndpoint() = default;

  crsf_addr_e getDeviceId() const { return device_id; }
  virtual bool handleRaw(const crsf_header_t *message) {
    (void)message;
    return false;
  }
  virtual void handleMessage(const crsf_header_t *message) = 0;
  virtual void registerParameters() {}
  virtual void updateParameters() {}

protected:
  virtual void devicePingCalled() {}

  void registerParameter(void *definition,
                         const parameterHandlerCallback &callback = nullptr,
                         uint8_t parent = 0);
  void sendDeviceInformationPacket();
  void parameterUpdateReq(crsf_addr_e origin, bool isElrs,
                          uint8_t parameterType, uint8_t parameterIndex,
                          void *payload);
  void sendParameterUpdate(uint8_t parameterIndex, uint8_t fieldChunk = 0,
                           bool isElrs = false);
  void sendCommandResponse(commandParameter *cmd, commandStep_e step,
                           const char *message);
  void sendAllParameters();

  static void setTextSelectionValue(selectionParameter *parameter,
                                    uint8_t newValue) {
    parameter->value = newValue;
  }
  static void setUint8Value(int8Parameter *parameter, uint8_t newValue) {
    parameter->properties.u.value = newValue;
  }
  static void setStringValue(stringParameter *parameter, const char *newValue) {
    parameter->value = newValue;
  }

private:
  crsf_addr_e device_id;
  crsf_addr_e requestOrigin = CRSF_ADDRESS_BROADCAST;

  propertiesCommon *paramDefinitions[MAX_CRSF_PARAMETERS] = {};
  parameterHandlerCallback paramCallbacks[MAX_CRSF_PARAMETERS] = {};

  uint8_t lastParameter = 0;
  uint8_t nextStatusChunk = 0;

  static uint8_t *textSelectionParameterToArray(const selectionParameter *parameter,
                                                uint8_t *next);
  static uint8_t *commandParameterToArray(const commandParameter *parameter,
                                          uint8_t *next);
  static uint8_t *int8ParameterToArray(const int8Parameter *parameter,
                                       uint8_t *next);
  static uint8_t *stringParameterToArray(const stringParameter *parameter,
                                         uint8_t *next);
  uint8_t *folderParameterToArray(const folderParameter *parameter,
                                  uint8_t *next) const;

  uint8_t sendParameter(crsf_addr_e origin, bool isElrs,
                        crsf_frame_type_e frameType, uint8_t fieldChunk,
                        const propertiesCommon *parameter);
  void pushResponseChunk(commandParameter *cmd, bool isElrs);
};
