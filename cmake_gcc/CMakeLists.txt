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
option(SIW917_ELRS_EVENT_COUNTERS
    "Collect low-overhead RF counters for event-only TX diagnostics"
    OFF
)
set(SIW917_ELRS_UPSTREAM_DIR "" CACHE PATH
    "Path to an official ExpressLRS src directory containing src/tx_main.cpp"
)
set(SIW917_ELRS_MAVLINK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/mavlink_c_library_v2"
    CACHE PATH "Path to the MAVLink c_library_v2 root used by upstream ELRS"
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

if(SIW917_ELRS_TARGET_TX AND SIW917_ELRS_USE_UPSTREAM_TX_MAIN AND
   NOT EXISTS "${SIW917_ELRS_MAVLINK_DIR}/common/mavlink.h")
    message(FATAL_ERROR
        "Missing upstream MAVLink headers at ${SIW917_ELRS_MAVLINK_DIR}. "
        "Use c_library_v2 commit e54a8d2e8cf7985e689ad1c8c8f37dc0800ea87b."
    )
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
    "${SIW917_ELRS_MAVLINK_DIR}"
    "${ELRS_UPSTREAM_DIR}/lib/tx-crsf"
)

set(ELRS_UPSTREAM_SOURCE_INCLUDE_DIRS ${ELRS_UPSTREAM_INCLUDE_DIRS})

# Keep target-specific transport handling out of the vendored ELRS source.
# The custom EdgeTX firmware exposes independent module-bay RX/TX wires, but
# CRSF replies still need upstream's half-duplex period budgeting. Generate a
# build-only source that keeps electrical full-duplex for startup/autobaud,
# applies the period budget independently, and never flushes the live RX wire
# after a response.
function(siw917_generate_two_wire_handset_source input_file output_var)
    set(old_duplex_block
"#if defined(PLATFORM_SIW917)
    // The SiW917 pins are still split before the external inverter/combiner,
    // but the RadioMaster module bay side is a single half-duplex CRSF bus.
    // Use upstream's half-duplex pacing so Lua/device replies fit the handset
    // receive windows instead of colliding with the radio's RC stream.
    halfDuplex = true;
    DBGLN(\"[CRSFHandset] SiW917 radio adapter: forcing half-duplex handset pacing\");
#endif")
    set(new_duplex_block
"#if defined(PLATFORM_SIW917) && !defined(SIW917_ELRS_CRSF_BENCH_2WIRE)
    // Adapter builds expose a shared half-duplex module-bay bus.
    halfDuplex = true;
    DBGLN(\"[CRSFHandset] SiW917 radio adapter: forcing half-duplex handset pacing\");
#endif")

    set(old_pacing_block
"        uint8_t periodBytesRemaining = HANDSET_TELEMETRY_FIFO_SIZE;
        if (halfDuplex)
        {")
    set(new_pacing_block
"        uint8_t periodBytesRemaining = HANDSET_TELEMETRY_FIFO_SIZE;
#if defined(PLATFORM_SIW917) && defined(SIW917_ELRS_CRSF_BENCH_2WIRE)
        const bool paceHandsetReplies = true;
#else
        const bool paceHandsetReplies = halfDuplex;
#endif
        if (paceHandsetReplies)
        {")

    set(old_block
"    if (transmitting)
    {
        // if currently transmitting in half-duplex mode then check if the TX buffers are empty.
        // If there is still data in the transmit buffers then exit, and we'll check next go round.
#if defined(PLATFORM_ESP32)
        if (!uart_ll_is_tx_idle(UART_LL_GET_HW(0)))
        {
            return;
        }
#elif defined(PLATFORM_ESP8266)
        if (((USS(0) >> USTXC) & 0xff) > 0)
        {
            return;
        }
#endif
        // All done transmitting; go back to receive mode
        transmitting = false;
        duplex_set_RX();
        flush_port_input();
    }")
    set(new_block
"    if (transmitting)
    {
#if defined(PLATFORM_SIW917) && defined(SIW917_ELRS_CRSF_BENCH_2WIRE)
        // EdgeTX uses independent RX/TX wires on this target. Keep logical
        // half-duplex reply pacing, but never block or discard live RX bytes.
        transmitting = false;
#else
        // if currently transmitting in half-duplex mode then check if the TX buffers are empty.
        // If there is still data in the transmit buffers then exit, and we'll check next go round.
#if defined(PLATFORM_ESP32)
        if (!uart_ll_is_tx_idle(UART_LL_GET_HW(0)))
        {
            return;
        }
#elif defined(PLATFORM_ESP8266)
        if (((USS(0) >> USTXC) & 0xff) > 0)
        {
            return;
        }
#endif
        // All done transmitting; go back to receive mode
        transmitting = false;
        duplex_set_RX();
        flush_port_input();
#endif
    }")

    file(READ "${input_file}" source_text)
    string(FIND "${source_text}" "${old_duplex_block}" duplex_offset)
    if(duplex_offset EQUAL -1)
        message(FATAL_ERROR
            "SiW917 handset duplex patch no longer matches ${input_file}"
        )
    endif()
    string(REPLACE "${old_duplex_block}" "${new_duplex_block}" source_text "${source_text}")

    string(FIND "${source_text}" "${old_pacing_block}" pacing_offset)
    if(pacing_offset EQUAL -1)
        message(FATAL_ERROR
            "SiW917 handset pacing patch no longer matches ${input_file}"
        )
    endif()
    string(REPLACE "${old_pacing_block}" "${new_pacing_block}" source_text "${source_text}")

    string(FIND "${source_text}" "${old_block}" block_offset)
    if(block_offset EQUAL -1)
        message(FATAL_ERROR
            "SiW917 handset transport patch no longer matches ${input_file}"
        )
    endif()
    string(REPLACE "${old_block}" "${new_block}" source_text "${source_text}")

    set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/siw917_upstream_transport")
    set(output_file "${output_dir}/CRSFHandset.cpp")
    file(MAKE_DIRECTORY "${output_dir}")
    file(WRITE "${output_file}" "${source_text}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${input_file}")
    set(${output_var} "${output_file}" PARENT_SCOPE)
endfunction()

# Upstream keeps the backpack hooks as local no-op functions on non-ESP
# targets. Generate a build-only copy that declares SiW917's internal WiFi
# backpack hooks instead, leaving the vendored upstream source unchanged.
function(siw917_generate_mavlink_backpack_source input_file output_var)
    set(old_block
"#if defined(PLATFORM_ESP32)
#include \"devScreen.h\"
#include \"devBLE.h\"
#include \"devGsensor.h\"
#include \"devThermal.h\"
#include \"devPDET.h\"
#include \"devBackpack.h\"
#else
// Fake functions for 8285
void checkBackpackUpdate() {}
void sendCRSFTelemetryToBackpack(uint8_t *) {}
void sendMAVLinkTelemetryToBackpack(uint8_t *) {}
#endif")
    set(new_block
"#if defined(PLATFORM_ESP32)
#include \"devScreen.h\"
#include \"devBLE.h\"
#include \"devGsensor.h\"
#include \"devThermal.h\"
#include \"devPDET.h\"
#include \"devBackpack.h\"
#elif defined(PLATFORM_SIW917)
void checkBackpackUpdate();
void sendCRSFTelemetryToBackpack(uint8_t *);
void sendMAVLinkTelemetryToBackpack(uint8_t *);
#else
// Fake functions for 8285
void checkBackpackUpdate() {}
void sendCRSFTelemetryToBackpack(uint8_t *) {}
void sendMAVLinkTelemetryToBackpack(uint8_t *) {}
#endif")

    set(old_mlrs_include
"#include \"stubborn_sender.h\"")
    set(new_mlrs_include
"#include \"stubborn_sender.h\"
#include \"mlrs_ota.h\"")
    set(old_link_diag_marker
"static void UpdateConnectDisconnectStatus()
{")
    set(new_link_diag_marker
"extern \"C\" void siw917_tx_get_link_diag(uint32_t *last_tlm_ms,
                                              uint32_t *last_link_stats_ms,
                                              uint8_t *telemetry_phase,
                                              bool *radio_busy)
{
  if (last_tlm_ms)
  {
    *last_tlm_ms = LastTLMpacketRecv_Ms;
  }
  if (last_link_stats_ms)
  {
    *last_link_stats_ms = LinkStatsLastReported_Ms;
  }
  if (telemetry_phase)
  {
    *telemetry_phase = (uint8_t)TelemetryRcvPhase;
  }
  if (radio_busy)
  {
    *radio_busy = busyTransmitting;
  }
}

extern \"C\" void siw917_tx_note_mlrs_downlink(uint8_t lq, int8_t rssi, int8_t snr)
{
  LastTLMpacketRecv_Ms = millis();
  RxDisconnected_Ms = 0;
  // Floor handset RQly at 50 while a real downlink is arriving so the
  // overlay acquire ramp does not trip EdgeTX Telemetry Critical. Real
  // loss is reported by siw917_tx_note_mlrs_link_lost().
  uint8_t reported = lq;
  if (reported > 0 && reported < 50)
  {
    reported = 50;
  }
  linkStats.uplink_Link_quality = reported;
  linkStats.downlink_Link_quality = reported;
  if (rssi < 0)
  {
    linkStats.uplink_RSSI_1 = rssi;
    linkStats.downlink_RSSI_1 = rssi;
  }
  linkStats.uplink_SNR = snr;
  linkStats.downlink_SNR = snr;
  if (connectionState != connected && connectionState < MODE_STATES)
  {
    setConnectionState(connected);
  }
}

extern \"C\" void siw917_tx_note_mlrs_link_lost(void)
{
  LastTLMpacketRecv_Ms = 0;
  linkStats.uplink_Link_quality = 0;
  linkStats.downlink_Link_quality = 0;
  RxDisconnected_Ms = millis();
  if (connectionState == connected)
  {
    setConnectionState(disconnected);
  }
}

static void UpdateConnectDisconnectStatus()
{
  if (mlrs_ota_is_active())
  {
    return;
  }")

    file(READ "${input_file}" source_text)
    string(FIND "${source_text}" "${old_block}" block_offset)
    if(block_offset EQUAL -1)
        message(FATAL_ERROR
            "SiW917 MAVLink backpack hook patch no longer matches ${input_file}"
        )
    endif()
    string(REPLACE "${old_block}" "${new_block}" source_text "${source_text}")

    string(FIND "${source_text}" "${old_mlrs_include}" mlrs_include_offset)
    if(mlrs_include_offset EQUAL -1)
        message(FATAL_ERROR
            "SiW917 TX mLRS include patch no longer matches ${input_file}"
        )
    endif()
    string(REPLACE "${old_mlrs_include}" "${new_mlrs_include}"
        source_text "${source_text}")

    string(FIND "${source_text}" "${old_link_diag_marker}" link_diag_offset)
    if(link_diag_offset EQUAL -1)
        message(FATAL_ERROR
            "SiW917 TX link diagnostic patch no longer matches ${input_file}"
        )
    endif()
    string(REPLACE "${old_link_diag_marker}" "${new_link_diag_marker}"
        source_text "${source_text}")

    set(old_pump_sender
"  else if (!DataUlSender.IsActive())
  {
    otaConnector.pumpSender();
  }")
    set(new_pump_sender
"  else if (!mlrs_ota_is_active() && !DataUlSender.IsActive())
  {
    otaConnector.pumpSender();
  }")
    string(FIND "${source_text}" "${old_pump_sender}" pump_offset)
    if(pump_offset EQUAL -1)
        message(FATAL_ERROR
            "SiW917 TX mLRS pumpSender patch no longer matches ${input_file}"
        )
    endif()
    string(REPLACE "${old_pump_sender}" "${new_pump_sender}"
        source_text "${source_text}")

    set(old_linkstats_send
"static void checkSendLinkStatsToHandset(uint32_t now)
{
  if ((now - LinkStatsLastReported_Ms) > firmwareOptions.tlm_report_interval)
  {
    // If we have gone to disconnected state, keep sending the ghost linkstats for some time
    if (RxDisconnected_Ms)
    {
      constexpr uint32_t LOST_NOTIFICATION_MAX_DELAY_MS = 3000U;
      // Delay a variable amount based on the LQ. Lower LQ, lower time before we tell the handset what's up
      uint32_t TelemetryLostCalloutDelay_Ms = map(constrain(linkStats.uplink_Link_quality, 50, 100), 50, 100, 0, LOST_NOTIFICATION_MAX_DELAY_MS);
      if (now - RxDisconnected_Ms > TelemetryLostCalloutDelay_Ms)
      {
        RxDisconnected_Ms = 0;
        linkStats.uplink_Link_quality = 0;
      }
    }

    CRSF_MK_FRAME_T(crsfLinkStatistics_t) linkStatisticsFrame;
    crsfRouter.makeLinkStatisticsPacket(&linkStatisticsFrame.h);
    // the linkStats originates from the OTA connector so we don't send it back there.
    crsfRouter.deliverMessage(&otaConnector, &linkStatisticsFrame.h);
    sendCRSFTelemetryToBackpack((uint8_t *)&linkStatisticsFrame);
    LinkStatsLastReported_Ms = now;
  }
}")
    set(new_linkstats_send
"static void sendLinkStatsToHandsetNow()
{
  CRSF_MK_FRAME_T(crsfLinkStatistics_t) linkStatisticsFrame;
  crsfRouter.makeLinkStatisticsPacket(&linkStatisticsFrame.h);
  // the linkStats originates from the OTA connector so we don't send it back there.
  crsfRouter.deliverMessage(&otaConnector, &linkStatisticsFrame.h);
  sendCRSFTelemetryToBackpack((uint8_t *)&linkStatisticsFrame);
  LinkStatsLastReported_Ms = millis();
}

static void checkSendLinkStatsToHandset(uint32_t now)
{
  if ((now - LinkStatsLastReported_Ms) > firmwareOptions.tlm_report_interval)
  {
    // If we have gone to disconnected state, keep sending the ghost linkstats for some time
    if (!mlrs_ota_is_active() && RxDisconnected_Ms)
    {
      constexpr uint32_t LOST_NOTIFICATION_MAX_DELAY_MS = 3000U;
      // Delay a variable amount based on the LQ. Lower LQ, lower time before we tell the handset what's up
      uint32_t TelemetryLostCalloutDelay_Ms = map(constrain(linkStats.uplink_Link_quality, 50, 100), 50, 100, 0, LOST_NOTIFICATION_MAX_DELAY_MS);
      if (now - RxDisconnected_Ms > TelemetryLostCalloutDelay_Ms)
      {
        RxDisconnected_Ms = 0;
        linkStats.uplink_Link_quality = 0;
      }
    }

    sendLinkStatsToHandsetNow();
  }
}

extern \"C\" void siw917_tx_publish_mlrs_linkstats(void)
{
  sendLinkStatsToHandsetNow();
}")
    string(FIND "${source_text}" "${old_linkstats_send}" linkstats_offset)
    if(linkstats_offset EQUAL -1)
        message(FATAL_ERROR
            "SiW917 TX mLRS handset linkstats patch no longer matches ${input_file}"
        )
    endif()
    string(REPLACE "${old_linkstats_send}" "${new_linkstats_send}"
        source_text "${source_text}")

    set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/siw917_upstream_transport")
    set(output_file "${output_dir}/tx_main.cpp")
    file(MAKE_DIRECTORY "${output_dir}")
    file(WRITE "${output_file}" "${source_text}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${input_file}")
    set(${output_var} "${output_file}" PARENT_SCOPE)
endfunction()

# Upstream gates the Backpack controls together with every feature of an
# external ESP backpack. Generate a build-only parameter source that exposes a
# dedicated runtime Off/On control for SiW917's internal WiFi transport.
function(siw917_generate_internal_backpack_parameters_source input_file output_var)
    set(old_parameter
"static commandParameter luaTxBackpackUpdate = {
    {\"Enable Backpack WiFi\", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};")
    set(new_parameter
"${old_parameter}

static selectionParameter luaSiW917MavlinkWifi = {
    {\"MAVLink WiFi\", CRSF_TEXT_SELECTION},
    0, // value
    luastrOffOn,
    STR_EMPTYSPACE};")
    set(old_declarations
"extern bool BackpackTelemReadyToSend;")
    set(new_declarations
"extern \"C\" void siw917_mavlink_backpack_set_lua_enabled(bool enabled);
extern \"C\" bool siw917_mavlink_backpack_get_lua_enabled(void);

extern bool BackpackTelemReadyToSend;")
    set(old_registration
"    if (OPT_USE_TX_BACKPACK) {
      registerParameter(&luaTxBackpackUpdate, sendCallback, luaWiFiFolder.common.id);")
    set(new_registration
"    if (OPT_USE_SIW917_INTERNAL_BACKPACK) {
      registerParameter(
          &luaSiW917MavlinkWifi, [](propertiesCommon *item, uint8_t arg) {
              const bool enabled = arg != 0;
              siw917_mavlink_backpack_set_lua_enabled(enabled);
              setTextSelectionValue((selectionParameter *)item, enabled ? 1 : 0);
          }, luaWiFiFolder.common.id);
    }

    if (OPT_USE_TX_BACKPACK) {
      registerParameter(&luaTxBackpackUpdate, sendCallback, luaWiFiFolder.common.id);")
    set(old_state_update
"  if (OPT_USE_TX_BACKPACK)
  {")
    set(new_state_update
"  if (OPT_USE_SIW917_INTERNAL_BACKPACK)
  {
    setTextSelectionValue(&luaSiW917MavlinkWifi,
                          siw917_mavlink_backpack_get_lua_enabled() ? 1 : 0);
  }
  if (OPT_USE_TX_BACKPACK)
  {")

    set(old_mlrs_include
"#include \"msptypes.h\"")
    set(new_mlrs_include
"#include \"msptypes.h\"
#include \"mlrs_ota.h\"")
    set(old_mlrs_parameter
"static selectionParameter luaAirRate = {
    {\"Packet Rate\", CRSF_TEXT_SELECTION},
    0, // value
    luastrPacketRates,
    STR_EMPTYSPACE
};")
    set(new_mlrs_parameter
"${old_mlrs_parameter}

static constexpr char luastrAirProtocol[] = \"ELRS;mLRS\";
static constexpr char luastrMlrsBand[] = \"915MHz;2.4GHz\";
static selectionParameter luaAirProtocol = {
    {\"Air Protocol\", CRSF_TEXT_SELECTION},
    0, // value
    luastrAirProtocol,
    STR_EMPTYSPACE
};
static selectionParameter luaMlrsRate = {
    {\"mLRS Rate\", CRSF_TEXT_SELECTION},
    0, // value
    MLRS_OTA_RATE_OPTIONS_915,
    STR_EMPTYSPACE
};
static selectionParameter luaMlrsBand = {
    {\"mLRS Band\", CRSF_TEXT_SELECTION},
    0, // value
    luastrMlrsBand,
    STR_EMPTYSPACE
};")
    set(old_mlrs_registration
"    registerParameter(&luaAirRate, [this](propertiesCommon *item, uint8_t arg) {
      uint8_t selectedRate = RATE_MAX - 1 - arg;
      SetPacketRateIdx(selectedRate, true);
    });")
    set(new_mlrs_registration
"    registerParameter(&luaAirRate, [this](propertiesCommon *item, uint8_t arg) {
      if (mlrs_ota_is_active()) {
        return;
      }
      uint8_t selectedRate = RATE_MAX - 1 - arg;
      SetPacketRateIdx(selectedRate, true);
    });
    registerParameter(&luaAirProtocol, [this](propertiesCommon *item, uint8_t arg) {
      (void)item;
      mlrs_ota_set_protocol(arg);
      updateParameters();
    });
    registerParameter(&luaMlrsRate, [this](propertiesCommon *item, uint8_t arg) {
      (void)item;
      mlrs_ota_set_rate(arg);
      luaMlrsRate.options = mlrs_ota_rate_options();
      setTextSelectionValue(&luaMlrsRate, mlrs_ota_get_rate());
    });
    registerParameter(&luaMlrsBand, [this](propertiesCommon *item, uint8_t arg) {
      (void)item;
      mlrs_ota_set_band(arg);
      updateParameters();
    });")
    set(old_mlrs_state
"  setTextSelectionValue(&luaAirRate, RATE_MAX - 1 - currentRate);")
    set(new_mlrs_state
"  setTextSelectionValue(&luaAirRate, RATE_MAX - 1 - currentRate);
  luaMlrsRate.options = mlrs_ota_rate_options();
  setTextSelectionValue(&luaAirProtocol, mlrs_ota_get_protocol());
  setTextSelectionValue(&luaMlrsRate, mlrs_ota_get_rate());
  setTextSelectionValue(&luaMlrsBand, mlrs_ota_get_band());")

    file(READ "${input_file}" source_text)
    foreach(patch_name IN ITEMS parameter declarations registration state_update mlrs_include mlrs_parameter mlrs_registration mlrs_state)
        set(old_text "${old_${patch_name}}")
        string(FIND "${source_text}" "${old_text}" patch_offset)
        if(patch_offset EQUAL -1)
            message(FATAL_ERROR
                "SiW917 internal backpack ${patch_name} patch no longer matches ${input_file}"
            )
        endif()
        string(REPLACE "${old_text}" "${new_${patch_name}}" source_text "${source_text}")
    endforeach()

    set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/siw917_upstream_transport")
    set(output_file "${output_dir}/TXModuleParameters.cpp")
    file(MAKE_DIRECTORY "${output_dir}")
    file(WRITE "${output_file}" "${source_text}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${input_file}")
    set(${output_var} "${output_file}" PARENT_SCOPE)
endfunction()

function(siw917_generate_lr1121_config_source input_file output_var)
    set(old_stdby
"    SetMode(LR1121_MODE_STDBY_RC, radioNumber);

    this->modulation = _modulation;

    // 8.1.1 SetPacketType")
    set(new_stdby
"    SetMode(LR1121_MODE_STDBY_XOSC, radioNumber);
    delay(5);

    this->modulation = _modulation;

    // 8.1.1 SetPacketType")
    file(READ "${input_file}" source_text)
    string(FIND "${source_text}" "${old_stdby}" patch_offset)
    if(patch_offset EQUAL -1)
        message(FATAL_ERROR
            "SiW917 LR1121 Config standby patch no longer matches ${input_file}"
        )
    endif()
    string(REPLACE "${old_stdby}" "${new_stdby}" source_text "${source_text}")
    set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/siw917_upstream_transport")
    set(output_file "${output_dir}/LR1121.cpp")
    file(MAKE_DIRECTORY "${output_dir}")
    file(WRITE "${output_file}" "${source_text}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${input_file}")
    set(${output_var} "${output_file}" PARENT_SCOPE)
endfunction()

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
    siw917_generate_mavlink_backpack_source(
        "${ELRS_MAIN_SOURCE}"
        ELRS_MAIN_SOURCE
    )
    set(ELRS_COMMON_SOURCE "${ELRS_UPSTREAM_DIR}/src/common.cpp")
    set(ELRS_RXTX_COMMON_SOURCE "${ELRS_UPSTREAM_DIR}/src/rxtx_common.cpp")
    set(ELRS_LR1121_SOURCE "${ELRS_UPSTREAM_DIR}/lib/LR1121Driver/LR1121.cpp")
    siw917_generate_lr1121_config_source(
        "${ELRS_LR1121_SOURCE}"
        ELRS_LR1121_SOURCE
    )
    set(ELRS_HANDSET_SOURCE "${ELRS_UPSTREAM_DIR}/lib/Handset/CRSFHandset.cpp")
    set(ELRS_TX_MODULE_PARAMETERS_SOURCE
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/TXModuleParameters.cpp")
    siw917_generate_internal_backpack_parameters_source(
        "${ELRS_TX_MODULE_PARAMETERS_SOURCE}"
        ELRS_TX_MODULE_PARAMETERS_SOURCE
    )
    if(SIW917_ELRS_CRSF_BENCH_2WIRE)
        siw917_generate_two_wire_handset_source(
            "${ELRS_HANDSET_SOURCE}"
            ELRS_HANDSET_SOURCE
        )
    endif()
    set(ELRS_DEVICE_SOURCE "${ELRS_UPSTREAM_DIR}/lib/DEVICE/device.cpp")
    set(ELRS_TX_UPSTREAM_EXTRA_SOURCES
        "${ELRS_UPSTREAM_DIR}/src/dynpower.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/ADC/devADC.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/BUTTON/devButton.cpp"
        ${ELRS_HANDSET_SOURCE}
        "${ELRS_UPSTREAM_DIR}/lib/LED/devLED.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/MAVLink/MAVLink.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/MAVLink/ardupilot_custom_telemetry.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/MSP/msp.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/POWERMGNT/POWERMGNT.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/devTXLUA.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/TXModuleEndpoint.cpp"
        ${ELRS_TX_MODULE_PARAMETERS_SOURCE}
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/TXOTAConnector.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/tx-crsf/TXUSBConnector.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/VTX/devVTX.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/elrs_eeprom/elrs_eeprom.cpp"
        "../src/elrs_cpp/hal/siw917_upstream_devices.cpp"
        "../src/elrs_cpp/hal/siw917_mavlink_backpack.cpp"
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
    "../src/siw917_mavlink_wifi.c"
    "../src/mlrs_ota/mlrs_ota.cpp"
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
    if(SIW917_ELRS_EVENT_COUNTERS)
        target_compile_definitions(slc PUBLIC
            SIW917_ELRS_EVENT_COUNTERS=1
            SIW917_ELRS_ISR_STATS_DIAG=1
            SIW917_ELRS_RAW_GSPI_STATS_DIAG=1
        )
    endif()
endif()

target_include_directories(wifi_gspi_merged PUBLIC
    # Our SiW917 platform headers (must come FIRST to override upstream)
    "../src"
    "../src/mlrs_ota"
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
    set_property(SOURCE
        "${ELRS_UPSTREAM_DIR}/lib/MAVLink/MAVLink.cpp"
        "${ELRS_UPSTREAM_DIR}/lib/MAVLink/ardupilot_custom_telemetry.cpp"
        APPEND PROPERTY COMPILE_OPTIONS
            -include ctime
            -Wno-error=address-of-packed-member
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
    if(SIW917_ELRS_EVENT_COUNTERS)
        target_compile_definitions(wifi_gspi_merged PUBLIC
            SIW917_ELRS_EVENT_COUNTERS=1
            SIW917_ELRS_TX_RF_STATS_DIAG=1
            SIW917_ELRS_DIO_STATS_DIAG=1
            SIW917_ELRS_ISR_STATS_DIAG=1
            SIW917_ELRS_RAW_GSPI_STATS_DIAG=1
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
