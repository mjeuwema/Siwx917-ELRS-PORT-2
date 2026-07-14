#pragma once

#include <stdint.h>

// Duration constants used by the upstream device framework.
#define DURATION_IGNORE -2
#define DURATION_NEVER -1
#define DURATION_IMMEDIATELY 0

enum deviceEvent_t {
  EVENT_NONE = 0,
  EVENT_ARM_FLAG_CHANGED = 1 << 0,
  EVENT_POWER_CHANGED = 1 << 1,
  EVENT_VTX_CHANGE = 1 << 2,
  EVENT_ENTER_BIND_MODE = 1 << 3,
  EVENT_EXIT_BIND_MODE = 1 << 4,
  EVENT_MODEL_SELECTED = 1 << 5,
  EVENT_CONNECTION_CHANGED = 1 << 6,
  EVENT_CONFIG_MODEL_CHANGED = 1 << 8,
  EVENT_CONFIG_VTX_CHANGED = 1 << 9,
  EVENT_CONFIG_MAIN_CHANGED = 1 << 10,
  EVENT_CONFIG_FAN_CHANGED = 1 << 11,
  EVENT_CONFIG_MOTION_CHANGED = 1 << 12,
  EVENT_CONFIG_BUTTON_CHANGED = 1 << 13,
  EVENT_CONFIG_UID_CHANGED = 1 << 14,
  EVENT_CONFIG_POWER_COUNT_CHANGED = 1 << 15,
  EVENT_CONFIG_PWM_CHANGE = 1 << 16,
  EVENT_CONFIG_SERIAL_CHANGE = 1 << 17,
  EVENT_CONFIG_VERSION_CHANGED = 1 << 18,
  EVENT_ALL = 0xFFFFFFFF
};

typedef struct {
  bool (*initialize)();
  int (*start)();
  int (*event)();
  int (*timeout)();
  uint32_t subscribe;
} device_t;

typedef struct {
  device_t *device;
  int8_t core;
} device_affinity_t;

void devicesRegister(device_affinity_t *devices, uint8_t count);
void devicesInit(void);
void devicesStart(void);
void devicesUpdate(unsigned long now);
void devicesTriggerEvent(uint32_t events);
void devicesStop(void);
