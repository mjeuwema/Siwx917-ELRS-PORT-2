#include "CRSFEndpoint.h"

#include "CRSFRouter.h"
#include "logging.h"
#include "options.h"

#include <cstring>

#define CRSF_PARAM_DIAG 0

static folderParameter paramRootFolder = {
    {"HooJ", CRSF_FOLDER, 0, 0},
    nullptr,
};

static char *copyString(char *dst, const char *src) {
  if (src == nullptr) {
    src = "";
  }
  while ((*dst = *src) != '\0') {
    ++dst;
    ++src;
  }
  return dst;
}

static uint8_t selectionOptionMax(const char *strOptions) {
  uint8_t retVal = 0;
  while (strOptions != nullptr && *strOptions != '\0') {
    if (*strOptions++ == ';') {
      ++retVal;
    }
  }
  return retVal;
}

uint8_t findSelectionLabel(const selectionParameter *parameter, char *outArray,
                           uint8_t value) {
  const char *cursor = parameter->options;
  uint8_t index = 0;
  while (cursor != nullptr && *cursor != '\0') {
    const char *start = cursor;
    while (*cursor != ';' && *cursor != '\0') {
      ++cursor;
    }
    if (index == value) {
      uint8_t labelLen = (uint8_t)(cursor - start);
      memcpy(outArray, start, labelLen);
      outArray[labelLen] = '\0';
      if (parameter->units != nullptr) {
        strcat(outArray, parameter->units);
      }
      return (uint8_t)strlen(outArray);
    }
    if (*cursor == ';') {
      ++cursor;
      ++index;
    }
  }
  return 0;
}

uint8_t *CRSFEndpoint::textSelectionParameterToArray(
    const selectionParameter *parameter, uint8_t *next) {
  next = (uint8_t *)copyString((char *)next, parameter->options) + 1;
  *next++ = parameter->value;
  *next++ = 0;
  *next++ = selectionOptionMax(parameter->options);
  *next++ = 0;
  return (uint8_t *)copyString((char *)next, parameter->units);
}

uint8_t *CRSFEndpoint::commandParameterToArray(const commandParameter *parameter,
                                               uint8_t *next) {
  *next++ = parameter->step;
  *next++ = 200;
  return (uint8_t *)copyString((char *)next, parameter->info);
}

uint8_t *CRSFEndpoint::int8ParameterToArray(const int8Parameter *parameter,
                                            uint8_t *next) {
  memcpy(next, &parameter->properties, sizeof(parameter->properties));
  next += sizeof(parameter->properties);
  *next++ = 0;
  return (uint8_t *)copyString((char *)next, parameter->units);
}

uint8_t *CRSFEndpoint::stringParameterToArray(const stringParameter *parameter,
                                              uint8_t *next) {
  return (uint8_t *)copyString((char *)next, parameter->value);
}

uint8_t *CRSFEndpoint::folderParameterToArray(const folderParameter *parameter,
                                              uint8_t *next) const {
  uint8_t *children = (uint8_t *)copyString(
                          (char *)next,
                          parameter->dyn_name ? parameter->dyn_name
                                              : parameter->common.name) +
                      1;
  for (uint8_t i = 1; i <= lastParameter; ++i) {
    if (paramDefinitions[i]->parent == parameter->common.id) {
      *children++ = i;
    }
  }
  *children = 0xFF;
  return children;
}

uint8_t CRSFEndpoint::sendParameter(crsf_addr_e origin, bool isElrs,
                                    crsf_frame_type_e frameType,
                                    uint8_t fieldChunk,
                                    const propertiesCommon *parameter) {
  const uint8_t dataType = parameter->type & CRSF_FIELD_TYPE_MASK;

  uint8_t chunkBuffer[256 + 4] = {};
  chunkBuffer[2] = parameter->parent;
  chunkBuffer[3] = dataType;
  if ((parameter->type & CRSF_FIELD_HIDDEN) != 0 ||
      (isElrs && (parameter->type & CRSF_FIELD_ELRS_HIDDEN) != 0)) {
    chunkBuffer[3] |= 0x80;
  }

  uint8_t *chunkStart =
      (uint8_t *)copyString((char *)&chunkBuffer[4], parameter->name) + 1;
  uint8_t *dataEnd = nullptr;

  switch (dataType) {
  case CRSF_TEXT_SELECTION:
    dataEnd =
        textSelectionParameterToArray((const selectionParameter *)parameter,
                                      chunkStart);
    break;
  case CRSF_COMMAND:
    dataEnd =
        commandParameterToArray((const commandParameter *)parameter, chunkStart);
    break;
  case CRSF_INT8:
  case CRSF_UINT8:
    dataEnd = int8ParameterToArray((const int8Parameter *)parameter, chunkStart);
    break;
  case CRSF_STRING:
  case CRSF_INFO:
    dataEnd =
        stringParameterToArray((const stringParameter *)parameter, chunkStart);
    break;
  case CRSF_FOLDER:
    dataEnd =
        folderParameterToArray((const folderParameter *)parameter,
                               &chunkBuffer[4]);
    break;
  default:
    return 0;
  }

  const uint16_t dataSize = (uint16_t)((dataEnd - chunkBuffer) - 2 + 1);
  const uint8_t connectorMax = crsfRouter.getConnectorMaxPacketSize(origin);
  const uint8_t chunkMax = connectorMax > 8 ? (uint8_t)(connectorMax - 6 - 2) : 1;
  const uint8_t chunkCnt = (uint8_t)((dataSize + chunkMax - 1) / chunkMax);
  if (fieldChunk >= chunkCnt) {
#if CRSF_PARAM_DIAG
    DBGLN("[CRSF_PARAM] skip id=%u chunk=%u chunks=%u name=%s",
          parameter->id, fieldChunk, chunkCnt, parameter->name);
#endif
    return 0;
  }

  const uint16_t chunkOffset = (uint16_t)fieldChunk * chunkMax;
  const uint8_t chunkSize =
      (uint8_t)min((uint16_t)(dataSize - chunkOffset), (uint16_t)chunkMax);

  uint8_t paramInformation[CRSF_FRAME_SIZE_MAX] = {};
  chunkStart = &chunkBuffer[chunkOffset];
  chunkStart[0] = parameter->id;
  chunkStart[1] = (uint8_t)(chunkCnt - (fieldChunk + 1));
  memcpy(paramInformation + sizeof(crsf_ext_header_t), chunkStart,
         chunkSize + 2);
  crsfRouter.SetExtendedHeaderAndCrc(
      (crsf_ext_header_t *)paramInformation, frameType,
      CRSF_EXT_FRAME_SIZE(chunkSize + 2), origin, device_id);
#if CRSF_PARAM_DIAG
  DBGLN("[CRSF_PARAM] send type=0x%02X id=%u chunk=%u remain=%u frame=%u data=%u to=0x%02X name=%s",
        frameType, parameter->id, fieldChunk,
        (uint8_t)(chunkCnt - (fieldChunk + 1)),
        ((crsf_header_t *)paramInformation)->frame_size, chunkSize, origin,
        parameter->name);
#endif
  crsfRouter.deliverMessageTo(origin, (crsf_header_t *)paramInformation);
  return (uint8_t)(chunkCnt - (fieldChunk + 1));
}

void CRSFEndpoint::pushResponseChunk(commandParameter *cmd, bool isElrs) {
  if (sendParameter(requestOrigin, isElrs,
                    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, nextStatusChunk,
                    (propertiesCommon *)cmd) == 0) {
    nextStatusChunk = 0;
  } else {
    ++nextStatusChunk;
  }
}

void CRSFEndpoint::sendParameterUpdate(uint8_t parameterIndex,
                                       uint8_t fieldChunk, bool isElrs) {
  if (parameterIndex >= MAX_CRSF_PARAMETERS ||
      paramDefinitions[parameterIndex] == nullptr) {
    return;
  }

  (void)sendParameter(requestOrigin, isElrs,
                      CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, fieldChunk,
                      paramDefinitions[parameterIndex]);
}

void CRSFEndpoint::sendCommandResponse(commandParameter *cmd,
                                       commandStep_e step,
                                       const char *message) {
  cmd->step = step;
  cmd->info = message;
  nextStatusChunk = 0;
  pushResponseChunk(cmd, false);
}

void CRSFEndpoint::registerParameter(void *definition,
                                     const parameterHandlerCallback &callback,
                                     uint8_t parent) {
  if (lastParameter == 0) {
    paramDefinitions[0] = (propertiesCommon *)&paramRootFolder;
    paramCallbacks[0] = nullptr;
  }

  auto *parameter = (propertiesCommon *)definition;
  ++lastParameter;
  parameter->id = lastParameter;
  parameter->parent = parent;
  paramDefinitions[lastParameter] = parameter;
  paramCallbacks[lastParameter] = callback;
}

void CRSFEndpoint::parameterUpdateReq(crsf_addr_e origin, bool isElrs,
                                      uint8_t parameterType,
                                      uint8_t parameterIndex,
                                      void *payload) {
  requestOrigin = origin;

  if (parameterType == CRSF_FRAMETYPE_DEVICE_PING) {
#if CRSF_PARAM_DIAG
    DBGLN("[CRSF_PARAM] device ping from=0x%02X", origin);
#endif
    devicePingCalled();
    sendDeviceInformationPacket();
    return;
  }

  if (parameterIndex >= MAX_CRSF_PARAMETERS ||
      paramDefinitions[parameterIndex] == nullptr) {
#if CRSF_PARAM_DIAG
    DBGLN("[CRSF_PARAM] invalid req type=0x%02X idx=%u from=0x%02X",
          parameterType, parameterIndex, origin);
#endif
    return;
  }

  propertiesCommon *parameter = paramDefinitions[parameterIndex];
  uint8_t parameterArg = payload != nullptr ? *((uint8_t *)payload) : 0;
#if CRSF_PARAM_DIAG
  DBGLN("[CRSF_PARAM] req type=0x%02X idx=%u arg=%u from=0x%02X name=%s",
        parameterType, parameterIndex, parameterArg, origin, parameter->name);
#endif

  switch (parameterType) {
  case CRSF_FRAMETYPE_PARAMETER_WRITE:
    if (paramCallbacks[parameterIndex]) {
      auto *argBytes = (uint8_t *)payload;
      int32_t arg = 0;
      switch (parameter->type & CRSF_FIELD_TYPE_MASK) {
      case CRSF_UINT16:
      case CRSF_INT16:
        arg = (argBytes[0] << 8) | argBytes[1];
        if ((parameter->type & CRSF_FIELD_TYPE_MASK) == CRSF_INT16) {
          arg = (int16_t)arg;
        }
        break;
      case CRSF_INT8:
        arg = (int8_t)argBytes[0];
        break;
      default:
        arg = argBytes[0];
        break;
      }

      if (parameterArg == lcsQuery && nextStatusChunk != 0) {
        pushResponseChunk((commandParameter *)parameter, isElrs);
      } else {
        paramCallbacks[parameterIndex](parameter, arg);
      }
    }
    break;

  case CRSF_FRAMETYPE_PARAMETER_READ: {
    auto *field = (commandParameter *)parameter;
    const uint8_t dataType = field->common.type & CRSF_FIELD_TYPE_MASK;
    if (dataType == CRSF_COMMAND && parameterArg == 0) {
      field->step = lcsIdle;
      field->info = "";
    }
    sendParameter(origin, isElrs, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY,
                  parameterArg, &field->common);
    break;
  }

  default:
    break;
  }
}

static uint32_t VersionStrToU32(const char *verStr) {
  uint32_t retVal = 0;
  uint8_t accumulator = 0;
  bool trailingData = false;
  while (verStr != nullptr && *verStr != '\0') {
    const char c = *verStr++;
    if (c == '.') {
      retVal = (retVal << 8) | accumulator;
      accumulator = 0;
      trailingData = false;
    } else if (c >= '0' && c <= '9') {
      accumulator = (uint8_t)((accumulator * 10) + (c - '0'));
      trailingData = true;
    } else {
      break;
    }
  }
  if (trailingData) {
    retVal = (retVal << 8) | accumulator;
  }
  return retVal < 0x010000 ? ((uint32_t)OTA_VERSION_ID << 16) : retVal;
}

void CRSFEndpoint::sendDeviceInformationPacket() {
  uint8_t deviceInformation[DEVICE_INFORMATION_LENGTH] = {};
  const uint8_t nameSize = (uint8_t)strlen(device_name) + 1;
  auto *device = (deviceInformationPacket_t *)(deviceInformation +
                                               sizeof(crsf_ext_header_t) +
                                               nameSize);
  memcpy(deviceInformation + sizeof(crsf_ext_header_t), device_name, nameSize);
  device->serialNo = htobe32(0x454C5253);
  device->hardwareVer = 0;
  device->softwareVer = htobe32(VersionStrToU32(version));
  device->fieldCnt = lastParameter;
  device->parameterVersion = 0;
  crsfRouter.SetExtendedHeaderAndCrc(
      (crsf_ext_header_t *)deviceInformation, CRSF_FRAMETYPE_DEVICE_INFO,
      DEVICE_INFORMATION_FRAME_SIZE, requestOrigin, device_id);
#if CRSF_PARAM_DIAG
  DBGLN("[CRSF_PARAM] send device-info fields=%u frame=%u to=0x%02X name=%s",
        lastParameter, ((crsf_header_t *)deviceInformation)->frame_size,
        requestOrigin, device_name);
#endif
  crsfRouter.deliverMessageTo(requestOrigin,
                              (crsf_header_t *)deviceInformation);
}
