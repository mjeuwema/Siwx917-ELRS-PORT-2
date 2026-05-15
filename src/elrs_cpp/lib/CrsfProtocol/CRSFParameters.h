#pragma once

#include "crsf_protocol.h"

#include <functional>

struct propertiesCommon {
  const char *name;
  crsf_value_type_e type;
  uint8_t id;
  uint8_t parent;
} PACKED;

struct selectionParameter {
  propertiesCommon common;
  uint8_t value;
  const char *options;
  const char *units;
} PACKED;

enum commandStep_e : uint8_t {
  lcsIdle = 0,
  lcsClick = 1,
  lcsExecuting = 2,
  lcsAskConfirm = 3,
  lcsConfirmed = 4,
  lcsCancel = 5,
  lcsQuery = 6,
};

struct commandParameter {
  propertiesCommon common;
  commandStep_e step;
  const char *info;
} PACKED;

struct int8Parameter {
  propertiesCommon common;
  union {
    struct {
      uint8_t value;
      const uint8_t min;
      const uint8_t max;
    } u;
    struct {
      int8_t value;
      const int8_t min;
      const int8_t max;
    } s;
  } PACKED properties;
  const char *const units;
} PACKED;

struct int16Parameter {
  propertiesCommon common;
  union {
    struct {
      uint16_t value;
      const uint16_t min;
      const uint16_t max;
    } u;
    struct {
      int16_t value;
      const int16_t min;
      const int16_t max;
    } s;
  } PACKED properties;
  const char *const units;
} PACKED;

struct floatParameter {
  propertiesCommon common;
  struct {
    uint32_t value;
    const uint32_t min;
    const uint32_t max;
    const uint32_t def;
    const uint8_t precision;
    const uint32_t step;
  } PACKED properties;
  const char *const units;
} PACKED;

struct stringParameter {
  propertiesCommon common;
  const char *value;
} PACKED;

struct folderParameter {
  propertiesCommon common;
  char *dyn_name;
} PACKED;

#define LUA_FIELD_HIDE(fld)                                                      \
  do {                                                                           \
    (fld).common.type =                                                          \
        (crsf_value_type_e)((uint8_t)(fld).common.type | CRSF_FIELD_HIDDEN);     \
  } while (0)

#define LUA_FIELD_SHOW(fld)                                                      \
  do {                                                                           \
    (fld).common.type =                                                          \
        (crsf_value_type_e)((uint8_t)(fld).common.type & ~CRSF_FIELD_HIDDEN);    \
  } while (0)

#define LUA_FIELD_VISIBLE(fld, cond)                                             \
  do {                                                                           \
    if (cond) {                                                                  \
      LUA_FIELD_SHOW(fld);                                                       \
    } else {                                                                     \
      LUA_FIELD_HIDE(fld);                                                       \
    }                                                                            \
  } while (0)

using parameterHandlerCallback = std::function<void(propertiesCommon *item, int32_t arg)>;

uint8_t findSelectionLabel(const selectionParameter *parameter, char *outArray,
                           uint8_t value);

constexpr char STR_EMPTYSPACE[1] = {};
