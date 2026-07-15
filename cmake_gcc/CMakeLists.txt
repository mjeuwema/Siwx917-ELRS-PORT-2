# Define minimal required version of CMake.
cmake_minimum_required(VERSION "3.20")

# Include toolchain before project() call
include(toolchain.cmake)

# Project definition
project(
	wifi_gspi_merged
	VERSION 1.0
	LANGUAGES C CXX ASM
)

option(SIW917_ELRS_TARGET_TX "Build the SiW917 ExpressLRS transmitter role" OFF)
option(SIW917_ELRS_CRSF_BENCH_2WIRE
    "Use separate UART1 RX/TX pins for bench CRSF testing instead of module-bay single-wire"
    ON
)
option(SIW917_ELRS_TX_PC_BENCH
    "Run the TX with safe internal RC channels so RF link testing does not require a radio handset"
    ON
)
option(SIW917_ELRS_USE_UPSTREAM_TX_MAIN
    "Build official ExpressLRS src/src/tx_main.cpp instead of the SiW917 adapter entrypoint"
    OFF
)
option(SIW917_CRSF_RADIO_RETURN_DIAG
    "Sweep device-info reply turnaround timing while diagnosing an EdgeTX return path"
    OFF
)
set(SIW917_ELRS_UPSTREAM_DIR "" CACHE PATH
    "Path to an official ExpressLRS src directory containing src/tx_main.cpp"
)

# Include the definition of the slc_wifi_gspi_merged target,
# which contains the content of the SLC project
include(wifi_gspi_merged.cmake)

#=============================================================================
# ELRS C++ Platform Abstraction Layer for SiW917
#=============================================================================

if(NOT SIW917_ELRS_UPSTREAM_DIR)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../ExpressLRS/src/src/tx_main.cpp")
        set(SIW917_ELRS_UPSTREAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ExpressLRS/src")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../upstream_elrs/src/src/tx_main.cpp")
        set(SIW917_ELRS_UPSTREAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../upstream_elrs/src")
    endif()
endif()
if(SIW917_ELRS_UPSTREAM_DIR)
    set(ELRS_UPSTREAM_DIR "${SIW917_ELRS_UPSTREAM_DIR}")
else()
    set(ELRS_UPSTREAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ExpressLRS/src")
endif()

set(ELRS_UPSTREAM_INCLUDE_DIRS
    "${ELRS_UPSTREAM_DIR}/src"
    "${ELRS_UPSTREAM_DIR}/include"
    "${ELRS_UPSTREAM_DIR}/lib/DEVICE"
    "${ELRS_UPSTREAM_DIR}/lib/FEC"
    "${ELRS_UPSTREAM_DIR}/lib/CrsfProtocol"
    "${ELRS_UPSTREAM_DIR}/lib/TelemetryProtocol"
    "${ELRS_UPSTREAM_DIR}/lib/LQCALC"
    "${ELRS_UPSTREAM_DIR}/lib/LBT"
    "${ELRS_UPSTREAM_DIR}/lib/PFD"
    "${ELRS_UPSTREAM_DIR}/lib/POWERMGNT"
    "${ELRS_UPSTREAM_DIR}/lib/FIFO"
    "${ELRS_UPSTREAM_DIR}/lib/LR1121Driver"
    "${ELRS_UPSTREAM_DIR}/lib/SX12xxDriverCommon"
    "${ELRS_UPSTREAM_DIR}/lib/OPTIONS"
    "${ELRS_UPSTREAM_DIR}/lib/OTA"
    "${ELRS_UPSTREAM_DIR}/lib/CRC"
    "${ELRS_UPSTREAM_DIR}/lib/FHSS"
    "${ELRS_UPSTREAM_DIR}/lib/logging"
    "${ELRS_UPSTREAM_DIR}/lib/MeanAccumulator"
    "${ELRS_UPSTREAM_DIR}/lib/rx-crsf"
    "${ELRS_UPSTREAM_DIR}/lib/CONFIG"
    "${ELRS_UPSTREAM_DIR}/lib/elrs_eeprom"
    "${ELRS_UPSTREAM_DIR}/lib/SPIEx"
    "${ELRS_UPSTREAM_DIR}/lib/HWTIMER"
    "${ELRS_UPSTREAM_DIR}/lib/MSP"
    "${ELRS_UPSTREAM_DIR}/lib/CRSF2MSP"
    "${ELRS_UPSTREAM_DIR}/lib/StubbornReceiver"
    "${ELRS_UPSTREAM_DIR}/lib/StubbornSender"
    "${ELRS_UPSTREAM_DIR}/lib/Handset"
    "${ELRS_UPSTREAM_DIR}/lib/ADC"
    "${ELRS_UPSTREAM_DIR}/lib/BUTTON"
    "${ELRS_UPSTREAM_DIR}/lib/LED"
    "${ELRS_UPSTREAM_DIR}/lib/VTX"
    "${ELRS_UPSTREAM_DIR}/lib/WIFI"
    "${ELRS_UPSTREAM_DIR}/lib/MAVLink"
    "${ELRS_UPSTREAM_DIR}/lib/tx-crsf"
)

set(ELRS_UPSTREAM_SOURCE_INCLUDE_DIRS ${ELRS_UPSTREAM_INCLUDE_DIRS})

# SiW917 platform abstraction files (Arduino compatibility layer)
# Prefer the SiW917 adapter until the upstream tx_main canary compiles cleanly.
if(SIW917_ELRS_TARGET_TX AND SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
    if(NOT EXISTS "${ELRS_UPSTREAM_DIR}/src/tx_main.cpp")
        message(FATAL_ERROR
            "SIW917_ELRS_USE_UPSTREAM_TX_MAIN=ON requires an official "
            "ExpressLRS src directory. Set SIW917_ELRS_UPSTREAM_DIR to a "
            "path containing src/tx_main.cpp."
        )
    endif()
    set(ELRS_MAIN_SOURCE "${ELRS_UPSTREAM_DIR}/src/tx_main.cpp")
    set(ELRS_COMMON_SOURCE "${ELRS_UPSTREAM_DIR}/src/common.cpp")
    set(ELRS_RXTX_COMMON_SOURCE "${ELRS_UPSTREAM_DIR}/src/rxtx_common.cpp")
    set(ELRS_LR1121_SOURCE "${ELRS_UPSTREAM_DIR}/lib/LR1121Driver/LR1121.cpp")
    set(ELRS_DEVICE_SOURCE "${ELRS_UPSTREAM_DIR}/lib/DEVICE/device.cpp")
    set(ELRS_TX_UPSTREAM_EXTRA_SOURCES
        "${ELRS_UPSTREAM_DIR}/src/dynpower.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/ADC/devADC.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/BUTTON/devButton.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/Handset/CRSFHandset.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/LED/devLED.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/MSP/msp.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/POWERMGNT/POWERMGNT.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/devTXLUA.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/TXModuleEndpoint.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/TXModuleParameters.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/TXOTAConnector.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/TXUSBConnector.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/VTX/devVTX.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/elrs_eeprom/elrs_eeprom.cpp"
        "../src/elrs_cpp/hal/siw917_upstream_devices.cpp"
        "../src/elrs_cpp/hal/siw917_upstream_tx_wrapper.cpp"
    )
    if(SIW917_ELRS_TX_PC_BENCH)
        list(APPEND ELRS_TX_UPSTREAM_EXTRA_SOURCES
            "../src/elrs_cpp/hal/siw917_upstream_bench_handset.cpp"
        )
    else()
        list(APPEND ELRS_TX_UPSTREAM_EXTRA_SOURCES
            "${ELRS_UPSTREAM_DIR}/lib/Handset/devHandset.cpp"
        )
    endif()
elseif(SIW917_ELRS_TARGET_TX)
    set(ELRS_MAIN_SOURCE "../src/elrs_cpp/elrs_tx_main.cpp")
    set(ELRS_COMMON_SOURCE "../src/elrs_cpp/common.cpp")
    set(ELRS_LR1121_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../src/elrs_cpp/lib/LR1121Driver/LR1121.cpp")
    set(ELRS_DEVICE_SOURCE "../src/elrs_cpp/hal/device.cpp")
else()
    set(ELRS_MAIN_SOURCE "../src/elrs_cpp/elrs_main.cpp")
    set(ELRS_COMMON_SOURCE "../src/elrs_cpp/common.cpp")
    set(ELRS_LR1121_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../src/elrs_cpp/lib/LR1121Driver/LR1121.cpp")
    set(ELRS_DEVICE_SOURCE "../src/elrs_cpp/hal/device.cpp")
endif()

set(ELRS_PLATFORM_SOURCES
    "../src/elrs_cpp/hal/Arduino.cpp"
    "../src/elrs_cpp/hal/SPIEx.cpp"
    "../src/elrs_cpp/hal/hwTimer.cpp"
    "../src/elrs_cpp/hal/LR1121_hal.cpp"
    "../src/elrs_cpp/hal/elrs_task_wakeup.cpp"
    ${ELRS_MAIN_SOURCE}
    ${ELRS_RXTX_COMMON_SOURCE}
    ${ELRS_COMMON_SOURCE}
    ${ELRS_TX_UPSTREAM_EXTRA_SOURCES}
    # CT-based hardware timer (C implementation wrapped by hwTimer.cpp)
    "../src/hw_timer.c"
    # Hardware timer test suite
    "../src/hw_timer_test.c"
)

# ELRS C++ library sources from our elrs_cpp folder.
# The upstream TX canary replaces TX scheduler/device sources with official
# ExpressLRS sources and keeps only the SiW917 HAL/shims local.
set(ELRS_CPP_LIB_DIR "../src/elrs_cpp/lib")
if(SIW917_ELRS_TARGET_TX AND SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
    set(ELRS_PROTOCOL_LIB_DIR "${ELRS_UPSTREAM_DIR}/lib")
    set(ELRS_CONFIG_SOURCE "${ELRS_UPSTREAM_DIR}/lib/CONFIG/config.cpp")
    set(ELRS_OPTIONS_SOURCE "${ELRS_CPP_LIB_DIR}/OPTIONS/options.cpp")
    set(ELRS_CRSF_RXTX_ENDPOINT_SOURCE "${ELRS_UPSTREAM_DIR}/lib/CrsfProtocol/RxTxEndpoint.cpp")
else()
    set(ELRS_PROTOCOL_LIB_DIR "${ELRS_CPP_LIB_DIR}")
    set(ELRS_CONFIG_SOURCE "")
    set(ELRS_OPTIONS_SOURCE "${ELRS_CPP_LIB_DIR}/OPTIONS/options.cpp")
    set(ELRS_CRSF_RXTX_ENDPOINT_SOURCE "")
endif()

set(ELRS_CPP_SOURCES
    # CRC
    "${ELRS_PROTOCOL_LIB_DIR}/CRC/crc.cpp"
    # FHSS
    "${ELRS_PROTOCOL_LIB_DIR}/FHSS/FHSS.cpp"
    "${ELRS_PROTOCOL_LIB_DIR}/FHSS/random.cpp"
    # OTA
    "${ELRS_PROTOCOL_LIB_DIR}/OTA/OTA.cpp"
    # LR1121 driver core. The upstream TX canary uses official LR1121.cpp
    # with the SiW917 LR1121_hal.cpp underneath.
    ${ELRS_LR1121_SOURCE}
    # OPTIONS
    ${ELRS_OPTIONS_SOURCE}
    ${ELRS_CONFIG_SOURCE}
    # FEC - Forward Error Correction (Hamming codes)
    "${ELRS_PROTOCOL_LIB_DIR}/FEC/FEC.cpp"
    "${ELRS_PROTOCOL_LIB_DIR}/FEC/hamming.cpp"
    # Device framework/stubs
    ${ELRS_DEVICE_SOURCE}
    # StubbornSender (for telemetry TX)
    "${ELRS_PROTOCOL_LIB_DIR}/StubbornSender/stubborn_sender.cpp"
    # StubbornReceiver (for telemetry RX / downlink data)
    "${ELRS_PROTOCOL_LIB_DIR}/StubbornReceiver/stubborn_receiver.cpp"
    # CRSF routing and RX Lua parameters
    "${ELRS_PROTOCOL_LIB_DIR}/CrsfProtocol/CRSFConnector.cpp"
    "${ELRS_PROTOCOL_LIB_DIR}/CrsfProtocol/CRSFRouter.cpp"
    "${ELRS_PROTOCOL_LIB_DIR}/CrsfProtocol/CRSFEndpoint.cpp"
    "${ELRS_PROTOCOL_LIB_DIR}/CrsfProtocol/CRSFParser.cpp"
    ${ELRS_CRSF_RXTX_ENDPOINT_SOURCE}
)

if(SIW917_ELRS_TARGET_TX AND NOT SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
    list(APPEND ELRS_CPP_SOURCES
    "${ELRS_CPP_LIB_DIR}/Handset/CRSFHandset.cpp"
    "${ELRS_CPP_LIB_DIR}/tx-crsf/TXOTAConnector.cpp"
    "${ELRS_CPP_LIB_DIR}/tx-crsf/TXModuleEndpoint.cpp"
    "${ELRS_CPP_LIB_DIR}/tx-crsf/TXModuleParameters.cpp"
    )
elseif(NOT SIW917_ELRS_TARGET_TX)
    list(APPEND ELRS_CPP_SOURCES
    "${ELRS_CPP_LIB_DIR}/rx-crsf/RXOTAConnector.cpp"
    "${ELRS_CPP_LIB_DIR}/rx-crsf/SiW917RXEndpoint.cpp"
    )
endif()

#=============================================================================
# Main executable
#=============================================================================

add_executable(wifi_gspi_merged
    # SiW917 platform layer
    ${ELRS_PLATFORM_SOURCES}
    # ELRS C++ libraries (upstream)
    ${ELRS_CPP_SOURCES}
)

#=============================================================================
# Include directories for ELRS
#=============================================================================

# Get the include directories from slc and apply them to our executable
# This is needed because C++ source files need SDK headers during compilation
get_target_property(SLC_INCLUDE_DIRS slc INTERFACE_INCLUDE_DIRECTORIES)
if(SLC_INCLUDE_DIRS)
    target_include_directories(wifi_gspi_merged PUBLIC ${SLC_INCLUDE_DIRS})
endif()

# gspi_example.c and other generated C sources are built via the managed `slc`
# target, so they need the same role define as the ELRS executable.
if(SIW917_ELRS_TARGET_TX)
    target_compile_definitions(slc PUBLIC
        SIW917_ELRS_TARGET_TX=1
    )
    if(SIW917_ELRS_CRSF_BENCH_2WIRE)
        target_compile_definitions(slc PUBLIC
            SIW917_ELRS_CRSF_BENCH_2WIRE=1
        )
    endif()
    if(SIW917_ELRS_TX_PC_BENCH)
        target_compile_definitions(slc PUBLIC
            SIW917_ELRS_TX_PC_BENCH=1
            SLI_SI91X_MCU_INTR_BASED_RX_ON_UART=1
        )
    endif()
    if(SIW917_CRSF_RADIO_RETURN_DIAG)
        target_compile_definitions(slc PUBLIC
            SIW917_CRSF_RADIO_RETURN_DIAG=1
        )
    endif()
endif()

target_include_directories(wifi_gspi_merged PUBLIC
    # Our SiW917 platform headers (must come FIRST to override upstream)
    "../src"
    "../src/elrs_cpp/hal"
    "../src/elrs_cpp/include"
    "../src/elrs_cpp/lib/CRC"
    "../src/elrs_cpp/lib/FHSS"
    "../src/elrs_cpp/lib/OTA"
    "../src/elrs_cpp/lib/LR1121Driver"
    "../src/elrs_cpp/lib/SX12xxDriverCommon"
    "../src/elrs_cpp/lib/CrsfProtocol"
    "../src/elrs_cpp/lib/Handset"
    "../src/elrs_cpp/lib/tx-crsf"
    "../src/elrs_cpp/lib/rx-crsf"
    "../src/elrs_cpp/lib/TelemetryProtocol"
    "../src/elrs_cpp/lib/LQCALC"
    "../src/elrs_cpp/lib/PFD"
    "../src/elrs_cpp/lib/FIFO"
    "../src/elrs_cpp/lib/LowPassFilter"
    "../src/elrs_cpp/lib/OPTIONS"
    "../src/elrs_cpp/lib/FEC"
    "../src/elrs_cpp/lib/StubbornSender"
    "../src/elrs_cpp/lib/StubbornReceiver"
    
    # Upstream ELRS include directories (for missing headers)
    ${ELRS_UPSTREAM_INCLUDE_DIRS}
)

if(SIW917_ELRS_TARGET_TX AND SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
    target_include_directories(wifi_gspi_merged BEFORE PUBLIC
        "${ELRS_UPSTREAM_DIR}/lib/Handset"
    )
    set(ELRS_UPSTREAM_SOURCE_INCLUDE_OPTIONS "")
    foreach(dir IN LISTS ELRS_UPSTREAM_SOURCE_INCLUDE_DIRS)
        list(APPEND ELRS_UPSTREAM_SOURCE_INCLUDE_OPTIONS -iquote "${dir}")
    endforeach()
    list(APPEND ELRS_UPSTREAM_SOURCE_INCLUDE_OPTIONS
        -include "${CMAKE_CURRENT_SOURCE_DIR}/../src/elrs_cpp/hal/siw917_upstream_target.h"
        -Wno-error=extra
        -Wno-error=implicit-fallthrough
        -Wno-error=missing-field-initializers
        -Wno-error=sign-compare
        -Wno-error=type-limits
    )
    set_property(SOURCE
        "${ELRS_MAIN_SOURCE}"
        "${ELRS_COMMON_SOURCE}"
        "${ELRS_RXTX_COMMON_SOURCE}"
        "${ELRS_LR1121_SOURCE}"
        "${ELRS_DEVICE_SOURCE}"
        ${ELRS_TX_UPSTREAM_EXTRA_SOURCES}
        ${ELRS_CPP_SOURCES}
        APPEND PROPERTY COMPILE_OPTIONS ${ELRS_UPSTREAM_SOURCE_INCLUDE_OPTIONS}
    )
endif()

#=============================================================================
# Compile definitions
#=============================================================================

target_compile_definitions(wifi_gspi_merged PUBLIC
    # Platform identification - don't redefine if already set
    # PLATFORM_SIW917=1  # Set in targets.h with guards
    # TARGET_RX=1        # Set in targets.h with guards
    
    # Radio type - LR1121 dual-band
    # RADIO_LR1121=1     # Set in hardware.h with guards
    
    # Regulatory domain (can be overridden)
    # Regulatory_Domain_FCC_915=1  # Set in hardware.h with guards
    
    # Debug logging
    # DEBUG_LOG=1        # Set in hardware.h with guards
)

if(SIW917_ELRS_TARGET_TX)
    target_compile_definitions(wifi_gspi_merged PUBLIC
        SIW917_ELRS_TARGET_TX=1
    )
    if(SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
        target_compile_definitions(wifi_gspi_merged PUBLIC
            SIW917_ELRS_USE_UPSTREAM_TX_MAIN=1
            TARGET_TX=1
            PLATFORM_SIW917=1
            RADIO_LR1121=1
            Regulatory_Domain_FCC_915=1
            # Linked TX telemetry needs the shortest TX_DONE/RX_DONE path.
            # Counters are event-reported only; they do not print in an ISR.
            SIW917_ELRS_DIRECT_GPIO_DIO_WHEN_LINKED=1
            SIW917_ELRS_DIO_STATS_DIAG=1
            SIW917_ELRS_DIO_EDGE_TIMESTAMPS=1
            SIW917_ELRS_TX_RF_STATS_DIAG=1
            IRAM_ATTR=
        )
    endif()
    if(SIW917_ELRS_CRSF_BENCH_2WIRE)
        target_compile_definitions(wifi_gspi_merged PUBLIC
            SIW917_ELRS_CRSF_BENCH_2WIRE=1
        )
    endif()
    if(SIW917_ELRS_TX_PC_BENCH)
        target_compile_definitions(wifi_gspi_merged PUBLIC
            SIW917_ELRS_TX_PC_BENCH=1
            SLI_SI91X_MCU_INTR_BASED_RX_ON_UART=1
        )
    endif()
    if(SIW917_CRSF_RADIO_RETURN_DIAG)
        target_compile_definitions(wifi_gspi_merged PUBLIC
            SIW917_CRSF_RADIO_RETURN_DIAG=1
        )
    endif()
endif()

#=============================================================================
# Compile options
#=============================================================================

target_compile_options(wifi_gspi_merged PUBLIC
    # C++ specific flags (already defined in slc but ensure they apply)
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    # Suppress some warnings from upstream ELRS code
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-variable>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-ignored-qualifiers>
    # Don't treat warnings as errors for ELRS code
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-error>
)

target_link_options(wifi_gspi_merged PUBLIC
    # No additional link options needed
)

# Link with the content defined in the SLC project
# Using PUBLIC so our C++ files get the include directories from slc
target_link_libraries(wifi_gspi_merged PUBLIC
    slc
)

if(SIW917_ELRS_TARGET_TX AND SIW917_ELRS_USE_UPSTREAM_TX_MAIN)
    target_link_libraries(wifi_gspi_merged PUBLIC
        stdc++
    )
endif()

# Include managed project content if available
include(wifi_gspi_merged_project.cmake OPTIONAL RESULT_VARIABLE managed_project)
if(managed_project)
    message(STATUS "Using managed project content from ${managed_project}")
endif()

# Force the gcc linker command
set_target_properties(wifi_gspi_merged PROPERTIES LINKER_LANGUAGE C)

# Create .bin, .hex and .s37 artifacts after building the project
add_custom_command(TARGET wifi_gspi_merged
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_SREC_CMD} "$<TARGET_FILE:wifi_gspi_merged>" "$<TARGET_FILE_DIR:wifi_gspi_merged>/$<TARGET_FILE_BASE_NAME:wifi_gspi_merged>.s37"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_IHEX_CMD} "$<TARGET_FILE:wifi_gspi_merged>" "$<TARGET_FILE_DIR:wifi_gspi_merged>/$<TARGET_FILE_BASE_NAME:wifi_gspi_merged>.hex"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_BIN_CMD}  "$<TARGET_FILE:wifi_gspi_merged>" "$<TARGET_FILE_DIR:wifi_gspi_merged>/$<TARGET_FILE_BASE_NAME:wifi_gspi_merged>.bin" 
)

# Run post-build pipeline to perform additional post-processing
if(post_build_command)
add_custom_command(TARGET wifi_gspi_merged
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMAND ${post_build_command}
)
endif()
