/*******************************************************************************
 * Compatibility glue for compiling WiseConnect BLE SDK sources in this project.
 *
 * The Silicon Labs BLE examples generate a few logging/config defines through
 * their component graph. This standalone probe imports only the BLE source files,
 * so provide the missing no-op defaults without changing the WiFi/RF app path.
 ******************************************************************************/

#ifndef BLE_SDK_COMPAT_H
#define BLE_SDK_COMPAT_H

#include "rsi_debug.h"

#ifndef BLE
#define BLE 0
#endif

#ifndef LOG_INFO
#define LOG_INFO 0
#endif

#ifndef LOG_WARN
#define LOG_WARN 0
#endif

#ifndef LOG_ERROR
#define LOG_ERROR 0
#endif

#ifndef LOG_TRACE
#define LOG_TRACE 0
#endif

#ifndef SL_PRINTF
#define SL_PRINTF(...)
#endif

#ifndef ALL_PHYS
#define ALL_PHYS 0x00
#endif

#endif /* BLE_SDK_COMPAT_H */
