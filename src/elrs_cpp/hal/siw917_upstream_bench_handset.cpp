#include "Arduino.h"
#include "config.h"
#include "FHSS.h"
#include "OTA.h"
#include "common.h"
#include "device.h"
#include "elrs_task_wakeup.h"
#include "handset.h"
#include "CRSFConnector.h"
#include "CRSFRouter.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(TARGET_TX) && defined(SIW917_ELRS_TX_PC_BENCH)

Handset *handset;

extern "C" void lr1121_get_isr_stats(uint32_t *isr_count,
                                      uint32_t *rx_count,
                                      uint32_t *tx_count,
                                      uint32_t *other_count,
                                      uint32_t *last_irq);

extern "C" void siw917_txdbg_get(uint32_t *send_attempts,
                                  uint32_t *send_sync,
                                  uint32_t *send_rc,
                                  uint32_t *send_data,
                                  uint32_t *lbt_none,
                                  uint32_t *tx_done_cb,
                                  uint32_t *tx_done_ignored,
                                  uint32_t *rx_window_req,
                                  uint32_t *rx_expected,
                                  uint32_t *rx_missed,
                                  uint32_t *rx_done_cb,
                                  uint32_t *rx_ignored,
                                  uint32_t *rx_accept,
                                  uint32_t *rx_reject,
                                  uint8_t *nonce,
                                  uint8_t *tlm_phase,
                                  uint8_t *busy);

extern "C" void elrs_tx_abort_data_uplink(void);

namespace {

constexpr uint32_t BENCH_STATUS_INTERVAL_MS = 5000;
constexpr uint32_t RXLUA_LINK_STABLE_MS = 5000;
constexpr uint32_t RXLUA_REQUEST_SPACING_MS = 500;
constexpr uint32_t RXLUA_RESPONSE_TIMEOUT_MS = 1000;
constexpr uint8_t RXLUA_MAX_RETRIES = 1;
constexpr uint16_t DEBUG_RX_RING_SIZE = 256;
constexpr uint16_t DEBUG_RX_RING_MASK = DEBUG_RX_RING_SIZE - 1;
static_assert((DEBUG_RX_RING_SIZE & DEBUG_RX_RING_MASK) == 0,
              "debug RX ring must be a power of two");

volatile uint8_t debugRxRing[DEBUG_RX_RING_SIZE];
volatile uint16_t debugRxHead = 0;
volatile uint16_t debugRxTail = 0;
volatile uint32_t debugRxOverflow = 0;

uint16_t boundedStringLength(const uint8_t *data, uint16_t maxLen)
{
    uint16_t len = 0;
    while (len < maxLen && data[len] != 0U) {
        ++len;
    }
    return len;
}

void copyPrintable(char *out, size_t outSize, const uint8_t *data,
                   uint16_t len)
{
    if (outSize == 0U) {
        return;
    }
    size_t used = 0;
    for (uint16_t i = 0; i < len && used + 1U < outSize; ++i) {
        const uint8_t c = data[i];
        if (c == '"') {
            out[used++] = '\'';
        } else if (c >= 32U && c < 127U) {
            out[used++] = (char)c;
        } else {
            out[used++] = '?';
        }
    }
    out[used] = '\0';
}

bool debugReadByte(uint8_t *out)
{
    if (debugRxTail == debugRxHead) {
        return false;
    }
    *out = debugRxRing[debugRxTail];
    debugRxTail = (uint16_t)((debugRxTail + 1U) & DEBUG_RX_RING_MASK);
    return true;
}

const char *paramTypeName(uint8_t type)
{
    switch ((crsf_value_type_e)type) {
    case CRSF_UINT8:
        return "uint8";
    case CRSF_INT8:
        return "int8";
    case CRSF_UINT16:
        return "uint16";
    case CRSF_INT16:
        return "int16";
    case CRSF_UINT32:
        return "uint32";
    case CRSF_INT32:
        return "int32";
    case CRSF_FLOAT:
        return "float";
    case CRSF_TEXT_SELECTION:
        return "select";
    case CRSF_STRING:
        return "string";
    case CRSF_FOLDER:
        return "folder";
    case CRSF_INFO:
        return "info";
    case CRSF_COMMAND:
        return "command";
    default:
        return "unknown";
    }
}

void copySelectionLabel(char *out, size_t outSize, const uint8_t *options,
                        uint16_t optionsLen, uint8_t selected)
{
    if (outSize == 0U) {
        return;
    }
    out[0] = '\0';
    uint16_t start = 0;
    uint8_t index = 0;
    for (uint16_t i = 0; i <= optionsLen; ++i) {
        if (i == optionsLen || options[i] == ';') {
            if (index == selected) {
                copyPrintable(out, outSize, options + start, i - start);
                return;
            }
            start = i + 1U;
            ++index;
        }
    }
}

uint16_t readLe16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

class SiW917BenchLuaConnector final : public CRSFConnector {
public:
    SiW917BenchLuaConnector()
    {
        addDevice(CRSF_ADDRESS_RADIO_TRANSMITTER);
        addDevice(CRSF_ADDRESS_ELRS_LUA);
    }

    void Begin()
    {
        crsfRouter.addConnector(this);
    }

    void StartPull()
    {
        if (connectionState != ::connected) {
            printf("[RXLUA] ERROR need_connected_link state=%u\n",
                   (unsigned)connectionState);
            return;
        }
        updateLinkStableTime();
        const uint32_t now = millis();
        const uint32_t stableMs = now - connectedSinceMs;
        if (stableMs < RXLUA_LINK_STABLE_MS) {
            printf("[RXLUA] ERROR wait_stable_link stable_ms=%lu need_ms=%lu\n",
                   (unsigned long)stableMs,
                   (unsigned long)RXLUA_LINK_STABLE_MS);
            return;
        }
        if (active) {
            printf("[RXLUA] WARN pull_already_active field=%u count=%u\n",
                   (unsigned)requestedField,
                   (unsigned)paramsPrinted);
            return;
        }

        // The RF control link is healthy, but current RX builds lose lock when
        // TX data-uplink carries Lua requests. Keep this GUI path safe until
        // the RX data-uplink handler is instrumented/fixed.
        elrs_tx_abort_data_uplink();
        active = false;
        waiting = false;
        printf("[RXLUA] ERROR disabled_rf_uplink_unstable "
               "state=connected lq=%u rssi=%d/%d snr=%d rf_mode=%u\n",
               (unsigned)linkStats.uplink_Link_quality,
               linkStats.uplink_RSSI_1 ? -(int)linkStats.uplink_RSSI_1 : 0,
               linkStats.uplink_RSSI_2 ? -(int)linkStats.uplink_RSSI_2 : 0,
               (int)linkStats.uplink_SNR,
               (unsigned)linkStats.rf_Mode);
        printf("[RXLUA] INFO normal_rf_link_ok; RX Lua over RF disabled to "
               "avoid packet-timeout disconnects\n");
        return;

        active = true;
        gotDeviceInfo = false;
        waiting = false;
        requestedField = 0;
        fieldCount = 0;
        currentChunk = 0;
        paramBufferLen = 0;
        paramsPrinted = 0;
        retries = 0;
        elrs_tx_abort_data_uplink();
        printf("[RXLUA] START requesting RX Lua parameters over RF link\n");
        sendDevicePing();
    }

    void Service()
    {
        updateLinkStableTime();
        if (!active) {
            return;
        }
        if (connectionState != ::connected) {
            printf("[RXLUA] ABORT link_lost state=%u params=%u\n",
                   (unsigned)connectionState,
                   (unsigned)paramsPrinted);
            elrs_tx_abort_data_uplink();
            active = false;
            return;
        }

        const uint32_t now = millis();
        if (waiting) {
            if ((uint32_t)(now - requestSentMs) < RXLUA_RESPONSE_TIMEOUT_MS) {
                return;
            }
            if (++retries > RXLUA_MAX_RETRIES) {
                printf("[RXLUA] WARN timeout field=%u chunk=%u\n",
                       (unsigned)requestedField,
                       (unsigned)currentChunk);
                elrs_tx_abort_data_uplink();
                waiting = false;
                retries = 0;
                if (gotDeviceInfo) {
                    ++requestedField;
                    nextRequestMs = now + RXLUA_REQUEST_SPACING_MS;
                } else {
                    active = false;
                }
            } else {
                resendCurrentRequest();
            }
            return;
        }

        if (!gotDeviceInfo || (int32_t)(now - nextRequestMs) < 0) {
            return;
        }

        if (requestedField > fieldCount) {
            printf("[RXLUA] DONE count=%u fields=%u device=\"%s\"\n",
                   (unsigned)paramsPrinted,
                   (unsigned)fieldCount,
                   deviceName);
            active = false;
            return;
        }

        currentChunk = 0;
        paramBufferLen = 0;
        sendParameterRead(requestedField, currentChunk);
    }

    void forwardMessage(const crsf_header_t *message) override
    {
        if (message == nullptr || message->type < CRSF_FRAMETYPE_DEVICE_PING) {
            return;
        }

        const auto *ext = (const crsf_ext_header_t *)message;
        if (ext->orig_addr != CRSF_ADDRESS_CRSF_RECEIVER) {
            return;
        }

        const uint8_t payloadLen =
            message->frame_size >= CRSF_FRAME_LENGTH_EXT_TYPE_CRC
                ? (uint8_t)(message->frame_size -
                            CRSF_FRAME_LENGTH_EXT_TYPE_CRC)
                : 0U;

        switch (message->type) {
        case CRSF_FRAMETYPE_DEVICE_INFO:
            handleDeviceInfo(ext->payload, payloadLen);
            break;
        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
            handleParameterEntry(ext->payload, payloadLen);
            break;
        default:
            break;
        }
    }

private:
    bool active = false;
    bool gotDeviceInfo = false;
    bool waiting = false;
    bool waitingDeviceInfo = false;
    bool linkWasConnected = false;
    uint8_t fieldCount = 0;
    uint8_t requestedField = 0;
    uint8_t currentChunk = 0;
    uint8_t retries = 0;
    uint8_t paramsPrinted = 0;
    uint16_t paramBufferLen = 0;
    uint32_t requestSentMs = 0;
    uint32_t nextRequestMs = 0;
    uint32_t connectedSinceMs = 0;
    uint8_t paramBuffer[256] = {};
    char deviceName[40] = {};

    void updateLinkStableTime()
    {
        const bool isConnected = connectionState == ::connected;
        if (isConnected && !linkWasConnected) {
            connectedSinceMs = millis();
        } else if (!isConnected) {
            connectedSinceMs = millis();
        }
        linkWasConnected = isConnected;
    }

    void sendExtended(crsf_frame_type_e type, const uint8_t *payload,
                      uint8_t payloadLen)
    {
        uint8_t frame[CRSF_MAX_PACKET_LEN] = {};
        auto *ext = (crsf_ext_header_t *)frame;
        if (payloadLen > 0U && payload != nullptr) {
            memcpy(ext->payload, payload, payloadLen);
        }
        crsfRouter.SetExtendedHeaderAndCrc(
            ext, type, CRSF_EXT_FRAME_SIZE(payloadLen),
            CRSF_ADDRESS_CRSF_RECEIVER, CRSF_ADDRESS_ELRS_LUA);
        crsfRouter.processMessage(this, (crsf_header_t *)frame);
        waiting = true;
        requestSentMs = millis();
    }

    void sendDevicePing()
    {
        waitingDeviceInfo = true;
        retries = 0;
        sendExtended(CRSF_FRAMETYPE_DEVICE_PING, nullptr, 0);
    }

    void sendParameterRead(uint8_t field, uint8_t chunk)
    {
        uint8_t payload[2] = {field, chunk};
        waitingDeviceInfo = false;
        retries = 0;
        sendExtended(CRSF_FRAMETYPE_PARAMETER_READ, payload, sizeof(payload));
    }

    void resendCurrentRequest()
    {
        printf("[RXLUA] RETRY field=%u chunk=%u attempt=%u\n",
               waitingDeviceInfo ? 255U : (unsigned)requestedField,
               waitingDeviceInfo ? 0U : (unsigned)currentChunk,
               (unsigned)retries);
        if (waitingDeviceInfo) {
            sendExtended(CRSF_FRAMETYPE_DEVICE_PING, nullptr, 0);
        } else {
            uint8_t payload[2] = {requestedField, currentChunk};
            sendExtended(CRSF_FRAMETYPE_PARAMETER_READ, payload,
                         sizeof(payload));
        }
    }

    void handleDeviceInfo(const uint8_t *payload, uint8_t payloadLen)
    {
        const uint16_t nameLen = boundedStringLength(payload, payloadLen);
        if (nameLen >= payloadLen ||
            payloadLen < nameLen + 1U + sizeof(deviceInformationPacket_t)) {
            printf("[RXLUA] ERROR malformed_device_info len=%u\n",
                   (unsigned)payloadLen);
            waiting = false;
            active = false;
            return;
        }

        copyPrintable(deviceName, sizeof(deviceName), payload, nameLen);
        const uint8_t *info = payload + nameLen + 1U;
        fieldCount = info[12];
        gotDeviceInfo = true;
        waiting = false;
        requestedField = 0;
        nextRequestMs = millis() + RXLUA_REQUEST_SPACING_MS;
        printf("[RXLUA] DEVICE name=\"%s\" fields=%u parameter_version=%u\n",
               deviceName,
               (unsigned)fieldCount,
               (unsigned)info[13]);
    }

    void handleParameterEntry(const uint8_t *payload, uint8_t payloadLen)
    {
        if (payloadLen < 2U) {
            return;
        }
        const uint8_t field = payload[0];
        const uint8_t chunksRemain = payload[1];
        const uint8_t *chunk = payload + 2U;
        const uint8_t chunkLen = payloadLen - 2U;

        if (field != requestedField) {
            printf("[RXLUA] WARN unexpected_field got=%u expected=%u\n",
                   (unsigned)field,
                   (unsigned)requestedField);
            return;
        }
        if ((uint16_t)paramBufferLen + chunkLen > sizeof(paramBuffer)) {
            printf("[RXLUA] WARN field=%u too_large\n", (unsigned)field);
            waiting = false;
            ++requestedField;
            nextRequestMs = millis() + RXLUA_REQUEST_SPACING_MS;
            return;
        }

        memcpy(paramBuffer + paramBufferLen, chunk, chunkLen);
        paramBufferLen = (uint16_t)(paramBufferLen + chunkLen);

        if (chunksRemain > 0U) {
            ++currentChunk;
            sendParameterRead(field, currentChunk);
            return;
        }

        printParameter(field, paramBuffer, paramBufferLen);
        ++paramsPrinted;
        waiting = false;
        retries = 0;
        ++requestedField;
        nextRequestMs = millis() + RXLUA_REQUEST_SPACING_MS;
    }

    void printParameter(uint8_t field, const uint8_t *data, uint16_t len)
    {
        if (len < 3U) {
            printf("[RXLUA] PARAM id=%u malformed len=%u\n",
                   (unsigned)field,
                   (unsigned)len);
            return;
        }

        const uint8_t parent = data[0];
        const uint8_t rawType = data[1];
        const uint8_t dataType = (uint8_t)(rawType & CRSF_FIELD_TYPE_MASK);
        const bool hidden = (rawType & CRSF_FIELD_HIDDEN) != 0U;
        const uint16_t nameLen = boundedStringLength(data + 2U, len - 2U);
        if (2U + nameLen >= len) {
            printf("[RXLUA] PARAM id=%u parent=%u type=%s hidden=%u "
                   "name=\"\" value=\"\" detail=\"missing_name_terminator\"\n",
                   (unsigned)field,
                   (unsigned)parent,
                   paramTypeName(dataType),
                   hidden ? 1U : 0U);
            return;
        }

        char name[48] = {};
        char value[96] = {};
        char detail[256] = {};
        copyPrintable(name, sizeof(name), data + 2U, nameLen);
        uint16_t offset = (uint16_t)(2U + nameLen + 1U);

        switch ((crsf_value_type_e)dataType) {
        case CRSF_TEXT_SELECTION: {
            const uint16_t optionsLen =
                boundedStringLength(data + offset, len - offset);
            if (offset + optionsLen + 5U <= len) {
                char options[96] = {};
                char selected[48] = {};
                copyPrintable(options, sizeof(options), data + offset,
                              optionsLen);
                const uint16_t meta = (uint16_t)(offset + optionsLen + 1U);
                const uint8_t selectedValue = data[meta];
                const uint8_t minValue = data[meta + 1U];
                const uint8_t maxValue = data[meta + 2U];
                const uint8_t defaultValue = data[meta + 3U];
                const uint16_t unitsOffset = (uint16_t)(meta + 4U);
                const uint16_t unitsLen =
                    boundedStringLength(data + unitsOffset, len - unitsOffset);
                char units[24] = {};
                copyPrintable(units, sizeof(units), data + unitsOffset,
                              unitsLen);
                copySelectionLabel(selected, sizeof(selected), data + offset,
                                   optionsLen, selectedValue);
                snprintf(value, sizeof(value), "%u:%s%s",
                         (unsigned)selectedValue, selected, units);
                snprintf(detail, sizeof(detail),
                         "options=%s min=%u max=%u default=%u units=%s",
                         options, (unsigned)minValue, (unsigned)maxValue,
                         (unsigned)defaultValue, units);
            }
            break;
        }
        case CRSF_UINT8:
        case CRSF_INT8:
            if (offset + 4U <= len) {
                const int valueSigned = dataType == CRSF_INT8
                    ? (int)(int8_t)data[offset]
                    : (int)data[offset];
                const int minSigned = dataType == CRSF_INT8
                    ? (int)(int8_t)data[offset + 1U]
                    : (int)data[offset + 1U];
                const int maxSigned = dataType == CRSF_INT8
                    ? (int)(int8_t)data[offset + 2U]
                    : (int)data[offset + 2U];
                const uint16_t unitsOffset = (uint16_t)(offset + 4U);
                const uint16_t unitsLen =
                    boundedStringLength(data + unitsOffset, len - unitsOffset);
                char units[24] = {};
                copyPrintable(units, sizeof(units), data + unitsOffset,
                              unitsLen);
                snprintf(value, sizeof(value), "%d%s", valueSigned, units);
                snprintf(detail, sizeof(detail), "min=%d max=%d units=%s",
                         minSigned, maxSigned, units);
            }
            break;
        case CRSF_UINT16:
        case CRSF_INT16:
            if (offset + 8U <= len) {
                const uint16_t rawValue = readLe16(data + offset);
                const uint16_t rawMin = readLe16(data + offset + 2U);
                const uint16_t rawMax = readLe16(data + offset + 4U);
                const int valueSigned = dataType == CRSF_INT16
                    ? (int)(int16_t)rawValue
                    : (int)rawValue;
                const int minSigned = dataType == CRSF_INT16
                    ? (int)(int16_t)rawMin
                    : (int)rawMin;
                const int maxSigned = dataType == CRSF_INT16
                    ? (int)(int16_t)rawMax
                    : (int)rawMax;
                const uint16_t unitsOffset = (uint16_t)(offset + 8U);
                const uint16_t unitsLen =
                    boundedStringLength(data + unitsOffset, len - unitsOffset);
                char units[24] = {};
                copyPrintable(units, sizeof(units), data + unitsOffset,
                              unitsLen);
                snprintf(value, sizeof(value), "%d%s", valueSigned, units);
                snprintf(detail, sizeof(detail), "min=%d max=%d units=%s",
                         minSigned, maxSigned, units);
            }
            break;
        case CRSF_STRING:
        case CRSF_INFO: {
            const uint16_t strLen =
                boundedStringLength(data + offset, len - offset);
            copyPrintable(value, sizeof(value), data + offset, strLen);
            break;
        }
        case CRSF_FOLDER:
            snprintf(value, sizeof(value), "<folder>");
            snprintf(detail, sizeof(detail), "children=");
            for (uint16_t i = offset; i < len && data[i] != 0xFFU; ++i) {
                const size_t used = strlen(detail);
                if (used + 5U < sizeof(detail)) {
                    snprintf(detail + used, sizeof(detail) - used, "%u,",
                             (unsigned)data[i]);
                }
            }
            break;
        case CRSF_COMMAND:
            if (offset + 2U <= len) {
                const uint8_t step = data[offset];
                const uint8_t timeout = data[offset + 1U];
                const uint16_t infoOffset = (uint16_t)(offset + 2U);
                const uint16_t infoLen =
                    boundedStringLength(data + infoOffset, len - infoOffset);
                copyPrintable(value, sizeof(value), data + infoOffset,
                              infoLen);
                snprintf(detail, sizeof(detail), "step=%u timeout=%u",
                         (unsigned)step, (unsigned)timeout);
            }
            break;
        default:
            snprintf(value, sizeof(value), "<raw>");
            snprintf(detail, sizeof(detail), "payload_len=%u",
                     (unsigned)len);
            break;
        }

        printf("[RXLUA] PARAM id=%u parent=%u type=%s hidden=%u "
               "name=\"%s\" value=\"%s\" detail=\"%s\"\n",
               (unsigned)field,
               (unsigned)parent,
               paramTypeName(dataType),
               hidden ? 1U : 0U,
               name,
               value,
               detail);
    }
};

SiW917BenchLuaConnector benchLuaConnector;

class SiW917BenchHandset final : public Handset {
public:
    void Begin() override
    {
        seedSafeChannels();
        benchLuaConnector.Begin();
        controllerConnected = true;
        printf("[TXBENCH] Upstream bench handset active - no radio required\n");
        printf("[TXLINK] Link debug active - watch for CONNECTED when an RX locks\n");
        printf("[PCBENCH] Debug serial commands ready: PCBENCH HELP\n");
        if (connected) {
            connected();
        }
        setConnectionState(::disconnected);
        feedChannels();
        lastFeedUs = micros();
        nextStatusMs = millis() + BENCH_STATUS_INTERVAL_MS;
    }

    void End() override
    {
        controllerConnected = false;
        if (disconnected) {
            disconnected();
        }
    }

    void handleInput() override
    {
        drainDebugCommands();
        benchLuaConnector.Service();

        const uint32_t nowUs = micros();
        if ((int32_t)(nowUs - lastFeedUs) >= RequestedRCpacketInterval) {
            feedChannels();
            lastFeedUs = nowUs;
        }

        const uint32_t nowMs = millis();
        const bool stateChanged =
            lastConnectionState != (uint8_t)connectionState;
        if (stateChanged || (int32_t)(nowMs - nextStatusMs) >= 0) {
            logFixedRate();
            logStatus();
            nextStatusMs = nowMs + BENCH_STATUS_INTERVAL_MS;
        }
    }

    uint8_t GetMaxPacketBytes() const override
    {
        return CRSF_MAX_PACKET_LEN;
    }

    int getMinPacketInterval() const override
    {
        return 1;
    }

    void CaptureCrsfChannels(const uint32_t *newChannels,
                             uint8_t channelCount,
                             bool armed)
    {
        const uint8_t limit = channelCount < (uint8_t)CRSF_NUM_CHANNELS
            ? channelCount
            : (uint8_t)CRSF_NUM_CHANNELS;
        for (uint8_t i = 0; i < limit; ++i) {
            channels[i] = newChannels[i] > CRSF_CHANNEL_VALUE_EXT_MAX
                ? CRSF_CHANNEL_VALUE_EXT_MAX
                : newChannels[i];
        }
        isArmed = armed;

        const uint16_t controlUs[] = {
            crsfToUs(channels[0]),
            crsfToUs(channels[1]),
            crsfToUs(channels[2]),
            crsfToUs(channels[3]),
            crsfToUs(channels[AUX1]),
        };
        bool shouldLog = !controlLogValid || lastControlArm != armed;
        for (uint8_t i = 0; i < 5U && !shouldLog; ++i) {
            const uint16_t current = controlUs[i];
            const uint16_t previous = lastControlUs[i];
            const uint16_t delta = current > previous
                ? current - previous
                : previous - current;
            shouldLog = delta >= 8U;
        }

        const uint32_t nowMs = millis();
        if (shouldLog && (uint32_t)(nowMs - lastControlLogMs) >= 100U) {
            printf("[PCBENCH] RC control accepted: roll=%u pitch=%u "
                   "throttle=%u yaw=%u aux1=%u armed=%u\n",
                   (unsigned)controlUs[0],
                   (unsigned)controlUs[1],
                   (unsigned)controlUs[2],
                   (unsigned)controlUs[3],
                   (unsigned)controlUs[4],
                   armed ? 1U : 0U);
            for (uint8_t i = 0; i < 5U; ++i) {
                lastControlUs[i] = controlUs[i];
            }
            lastControlArm = armed;
            controlLogValid = true;
            lastControlLogMs = nowMs;
        }
    }

private:
    uint32_t channels[CRSF_NUM_CHANNELS] = {};
    char commandLine[96] = {};
    uint8_t commandLineLen = 0;
    uint32_t lastFeedUs = 0;
    uint32_t nextStatusMs = 0;
    uint32_t lastWakeTimer = 0;
    uint32_t lastWakeDio = 0;
    uint32_t lastIrqTxDone = 0;
    uint32_t lastIrqRxDone = 0;
    uint32_t lastSendAttempts = 0;
    uint32_t lastTxDoneCb = 0;
    uint32_t lastRxWindowReq = 0;
    uint32_t lastRxMissed = 0;
    uint32_t lastRxDoneCb = 0;
    uint32_t lastRxAccept = 0;
    uint32_t lastRxReject = 0;
    uint8_t lastConnectionState = 0xFF;
    uint32_t connectedSinceMs = 0;
    uint32_t lastControlLogMs = 0;
    uint16_t lastControlUs[5] = {};
    bool fixedRateLogged = false;
    bool controlLogValid = false;
    bool lastControlArm = false;

    uint16_t crsfToUs(uint32_t value) const
    {
        constexpr uint32_t kCrsfMin = 172U;
        constexpr uint32_t kCrsfMax = 1811U;
        if (value < kCrsfMin) {
            value = kCrsfMin;
        } else if (value > kCrsfMax) {
            value = kCrsfMax;
        }
        return (uint16_t)(988U + ((value - kCrsfMin) * 1024U) /
                                   (kCrsfMax - kCrsfMin));
    }

    uint32_t usToCrsf(uint32_t value) const
    {
        if (value < US_CHANNEL_VALUE_STD_MIN) {
            value = US_CHANNEL_VALUE_STD_MIN;
        } else if (value > US_CHANNEL_VALUE_STD_MAX) {
            value = US_CHANNEL_VALUE_STD_MAX;
        }
        return CRSF_CHANNEL_VALUE_STD_MIN +
               ((value - US_CHANNEL_VALUE_STD_MIN) *
                (CRSF_CHANNEL_VALUE_STD_MAX - CRSF_CHANNEL_VALUE_STD_MIN)) /
                   (US_CHANNEL_VALUE_STD_MAX - US_CHANNEL_VALUE_STD_MIN);
    }

    void seedSafeChannels()
    {
        for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
            channels[i] = CRSF_CHANNEL_VALUE_MID;
        }
        channels[2] = CRSF_CHANNEL_VALUE_1000; // throttle low
        channels[AUX1] = CRSF_CHANNEL_VALUE_1000; // disarmed
    }

    void feedChannels()
    {
        PerformChannelOverrides(channels, CRSF_NUM_CHANNELS);
        RCDataReceived(channels, CRSF_NUM_CHANNELS);
    }

    void applyControlUs(uint32_t rollUs, uint32_t pitchUs,
                        uint32_t throttleUs, uint32_t yawUs,
                        uint32_t aux1Us)
    {
        uint32_t updated[CRSF_NUM_CHANNELS] = {};
        for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
            updated[i] = channels[i];
        }
        updated[0] = usToCrsf(rollUs);
        updated[1] = usToCrsf(pitchUs);
        updated[2] = usToCrsf(throttleUs);
        updated[3] = usToCrsf(yawUs);
        updated[AUX1] = usToCrsf(aux1Us);
        CaptureCrsfChannels(updated, CRSF_NUM_CHANNELS, aux1Us >= 1500U);
    }

    bool parseFiveUInts(char *text, uint32_t *values)
    {
        char *cursor = text;
        for (uint8_t i = 0; i < 5U; ++i) {
            while (*cursor == ' ' || *cursor == '\t') {
                ++cursor;
            }
            if (*cursor == '\0') {
                return false;
            }
            char *end = nullptr;
            const unsigned long parsed = strtoul(cursor, &end, 10);
            if (end == cursor) {
                return false;
            }
            values[i] = (uint32_t)parsed;
            cursor = end;
        }
        return true;
    }

    void handleDebugCommand(char *line)
    {
        if (strncmp(line, "PCBENCH ", 8) != 0) {
            return;
        }

        char *command = line + 8;
        if (strcmp(command, "HELP") == 0) {
            printf("[PCBENCH] commands: PCBENCH RC <roll_us> <pitch_us> "
                   "<thr_us> <yaw_us> <aux1_us> | PCBENCH SAFE | "
                   "PCBENCH RXLUA PULL\n");
        } else if (strcmp(command, "SAFE") == 0) {
            seedSafeChannels();
            isArmed = false;
            printf("[PCBENCH] safe RC applied: throttle=988 aux1=disarmed\n");
        } else if (strncmp(command, "RC ", 3) == 0) {
            uint32_t values[5] = {};
            if (parseFiveUInts(command + 3, values)) {
                applyControlUs(values[0], values[1], values[2], values[3],
                               values[4]);
            } else {
                printf("[PCBENCH] ERROR bad RC command\n");
            }
        } else if (strcmp(command, "RXLUA PULL") == 0) {
            benchLuaConnector.StartPull();
        } else {
            printf("[PCBENCH] ERROR unknown command: %s\n", command);
        }
    }

    void drainDebugCommands()
    {
        uint8_t byte = 0;
        while (debugReadByte(&byte)) {
            if (byte == '\r' || byte == '\n') {
                if (commandLineLen > 0U) {
                    commandLine[commandLineLen] = '\0';
                    handleDebugCommand(commandLine);
                    commandLineLen = 0;
                }
            } else if (commandLineLen + 1U < sizeof(commandLine)) {
                commandLine[commandLineLen++] = (char)byte;
            } else {
                commandLineLen = 0;
                printf("[PCBENCH] ERROR command too long\n");
            }
        }
    }

    void logFixedRate()
    {
        if (fixedRateLogged || ExpressLRS_currAirRate_Modparams == nullptr) {
            return;
        }

        const expresslrs_mod_settings_s *const params =
            ExpressLRS_currAirRate_Modparams;
        printf("[TXBENCH] upstream fixed RF rate - no automatic packet-rate cycling "
               "(rate_index=%u rf_mode=%u interval=%ld us)\n",
               (unsigned)params->index,
               (unsigned)params->enum_rate,
               (long)params->interval);
        printf("[TXBENCH] PC bench sync packets advertise runtime_rate=%u "
               "(upstream_config_rate=%u)\n",
               (unsigned)params->index,
               (unsigned)config.GetRate());
        printf("[TXBENCH] PC bench disconnected acquisition sync boost enabled\n");
        printf("[TXBENCH] active OTA uid=%02X:%02X:%02X:%02X:%02X:%02X "
               "crc_init=0x%04X uid_seed=0x%08lX model_match=%u\n",
               UID[0], UID[1], UID[2], UID[3], UID[4], UID[5],
               (unsigned)OtaCrcInitializer,
               (unsigned long)OtaGetUidSeed(),
               config.GetModelMatch() ? 1U : 0U);
        fixedRateLogged = true;
    }

    const char *stateName(connectionState_e state) const
    {
        switch (state) {
        case ::connected:
            return "connected";
        case ::tentative:
            return "tentative";
        case ::awaitingModelId:
            return "awaiting-model";
        case ::disconnected:
            return "disconnected";
        case ::noCrossfire:
            return "no-crossfire";
        case ::bleJoystick:
            return "ble-joystick";
        case ::wifiUpdate:
            return "wifi";
        case ::serialUpdate:
            return "serial-update";
        case ::radioFailed:
            return "radio-failed";
        case ::hardwareUndefined:
            return "hardware-undefined";
        default:
            return "other";
        }
    }

    void logStatus()
    {
        const expresslrs_mod_settings_s *const params =
            ExpressLRS_currAirRate_Modparams;
        const uint32_t wakeTimer = elrs_task_wakeup_get_timer_count();
        const uint32_t wakeDio = elrs_task_wakeup_get_dio_count();
        uint32_t irqCount = 0;
        uint32_t irqRxDone = 0;
        uint32_t irqTxDone = 0;
        uint32_t irqOther = 0;
        uint32_t irqLast = 0;
        uint32_t sendAttempts = 0;
        uint32_t sendSync = 0;
        uint32_t sendRc = 0;
        uint32_t sendData = 0;
        uint32_t lbtNone = 0;
        uint32_t txDoneCb = 0;
        uint32_t txDoneIgnored = 0;
        uint32_t rxWindowReq = 0;
        uint32_t rxExpected = 0;
        uint32_t rxMissed = 0;
        uint32_t rxDoneCb = 0;
        uint32_t rxIgnored = 0;
        uint32_t rxAccept = 0;
        uint32_t rxReject = 0;
        uint8_t nonce = 0;
        uint8_t tlmPhase = 0;
        uint8_t busy = 0;

        lr1121_get_isr_stats(&irqCount, &irqRxDone, &irqTxDone, &irqOther,
                             &irqLast);
        siw917_txdbg_get(&sendAttempts, &sendSync, &sendRc, &sendData,
                         &lbtNone, &txDoneCb, &txDoneIgnored, &rxWindowReq,
                         &rxExpected, &rxMissed, &rxDoneCb, &rxIgnored,
                         &rxAccept, &rxReject, &nonce, &tlmPhase, &busy);

        const uint8_t fhssPtr = FHSSgetCurrIndex();
        const uint16_t fhssSeqCount = FHSSgetSequenceCount();
        const uint32_t fhssChanCount = FHSSgetChannelCount();
        const uint8_t fhssChan = FHSSusePrimaryFreqBand
            ? FHSSsequence[fhssPtr]
            : FHSSsequence_DualBand[fhssPtr];
        const uint32_t deltaAttempts = sendAttempts - lastSendAttempts;
        const uint32_t deltaTxDone = irqTxDone - lastIrqTxDone;
        const uint32_t deltaRxWin = rxWindowReq - lastRxWindowReq;
        const uint32_t deltaRxMiss = rxMissed - lastRxMissed;
        const uint32_t deltaRxOk = rxAccept - lastRxAccept;
        const uint32_t deltaRxReject = rxReject - lastRxReject;
        const int rssi1Dbm = linkStats.uplink_RSSI_1
            ? -(int)linkStats.uplink_RSSI_1
            : 0;
        const int rssi2Dbm = linkStats.uplink_RSSI_2
            ? -(int)linkStats.uplink_RSSI_2
            : 0;

        bool printedTransition = false;
        if (lastConnectionState != (uint8_t)connectionState) {
            if (connectionState == ::connected) {
                connectedSinceMs = millis();
                printf("[TXLINK] CONNECTED: RX telemetry locked "
                       "lq=%u rssi=%d/%d dBm snr=%d rate_index=%u "
                       "rf_mode=%u tlmDenom=%u fhss=%u/%lu rxOk=%lu\n",
                       (unsigned)linkStats.uplink_Link_quality,
                       rssi1Dbm,
                       rssi2Dbm,
                       (int)linkStats.uplink_SNR,
                       params != nullptr ? (unsigned)params->index : 255U,
                       params != nullptr ? (unsigned)params->enum_rate : 255U,
                       (unsigned)ExpressLRS_currTlmDenom,
                       (unsigned)fhssChan,
                       (unsigned long)fhssChanCount,
                       (unsigned long)rxAccept);
                printedTransition = true;
            } else if (lastConnectionState == (uint8_t)::connected) {
                printf("[TXLINK] DISCONNECTED: lost RX telemetry "
                       "new_state=%s txDone_delta=%lu rxWin_delta=%lu "
                       "rxOk_delta=%lu rxMiss_delta=%lu\n",
                       stateName(connectionState),
                       (unsigned long)deltaTxDone,
                       (unsigned long)deltaRxWin,
                       (unsigned long)deltaRxOk,
                       (unsigned long)deltaRxMiss);
                printedTransition = true;
            } else {
                printf("[TXLINK] STATE: %s (%u) waiting for RX "
                       "rate_index=%u rf_mode=%u fhss=%u/%lu\n",
                       stateName(connectionState),
                       (unsigned)connectionState,
                       params != nullptr ? (unsigned)params->index : 255U,
                       params != nullptr ? (unsigned)params->enum_rate : 255U,
                       (unsigned)fhssChan,
                       (unsigned long)fhssChanCount);
                printedTransition = true;
            }
            lastConnectionState = (uint8_t)connectionState;
        }

        if (printedTransition) {
            // The transition line is the important event; leave steady-state
            // detail to the next heartbeat so the bench log stays readable.
        } else if (connectionState == ::connected) {
            const uint32_t connectedForMs = millis() - connectedSinceMs;
            printf("[TXLINK] CONNECTED status: up=%lu ms lq=%u "
                   "rssi=%d/%d dBm snr=%d rate_index=%u rf_mode=%u "
                   "tlmDenom=%u rxOk_delta=%lu rxMiss_delta=%lu "
                   "rxReject_delta=%lu\n",
                   (unsigned long)connectedForMs,
                   (unsigned)linkStats.uplink_Link_quality,
                   rssi1Dbm,
                   rssi2Dbm,
                   (int)linkStats.uplink_SNR,
                   params != nullptr ? (unsigned)params->index : 255U,
                   params != nullptr ? (unsigned)params->enum_rate : 255U,
                   (unsigned)ExpressLRS_currTlmDenom,
                   (unsigned long)deltaRxOk,
                   (unsigned long)deltaRxMiss,
                   (unsigned long)deltaRxReject);
        } else {
            printf("[TXLINK] SEARCHING status: state=%s txAttempts_delta=%lu "
                   "txDone_delta=%lu rxWin_delta=%lu rxOk_delta=%lu "
                   "rxMiss_delta=%lu\n",
                   stateName(connectionState),
                   (unsigned long)deltaAttempts,
                   (unsigned long)deltaTxDone,
                   (unsigned long)deltaRxWin,
                   (unsigned long)deltaRxOk,
                   (unsigned long)deltaRxMiss);
        }

#if defined(SIW917_ELRS_TX_RF_VERBOSE)
        printf("[TXBENCH] upstream state=%u rate_index=%u rf_mode=%u "
               "interval=%ld tlmDenom=%u fhss_ptr=%u/%u fhss_chan=%u/%lu domain=%s "
               "lq=%u rssi1=%d rssi2=%d snr=%d pc_bench=1\n",
               (unsigned)connectionState,
               params != nullptr ? (unsigned)params->index : 255U,
               params != nullptr ? (unsigned)params->enum_rate : 255U,
               params != nullptr ? (long)params->interval : 0L,
               (unsigned)ExpressLRS_currTlmDenom,
               (unsigned)fhssPtr,
               (unsigned)fhssSeqCount,
               (unsigned)fhssChan,
               (unsigned long)fhssChanCount,
               FHSSgetRegulatoryDomain(),
               (unsigned)linkStats.uplink_Link_quality,
               (int)linkStats.uplink_RSSI_1,
               (int)linkStats.uplink_RSSI_2,
               (int)linkStats.uplink_SNR);
        printf("[TXRFDBG] wake timer=%lu dio=%lu irq isr=%lu txDone=%lu "
               "rxDone=%lu other=%lu last=0x%08lX\n",
               (unsigned long)wakeTimer,
               (unsigned long)wakeDio,
               (unsigned long)irqCount,
               (unsigned long)irqTxDone,
               (unsigned long)irqRxDone,
               (unsigned long)irqOther,
               (unsigned long)irqLast);
        printf("[TXRFDBG] tx attempts=%lu sync=%lu rc=%lu data=%lu "
               "lbtNone=%lu txCb=%lu txIgnored=%lu rxWin=%lu rxExpect=%lu "
               "rxMiss=%lu rxCb=%lu rxIgnored=%lu rxOk=%lu rxReject=%lu "
               "nonce=%u tlmPhase=%u busy=%u\n",
               (unsigned long)sendAttempts,
               (unsigned long)sendSync,
               (unsigned long)sendRc,
               (unsigned long)sendData,
               (unsigned long)lbtNone,
               (unsigned long)txDoneCb,
               (unsigned long)txDoneIgnored,
               (unsigned long)rxWindowReq,
               (unsigned long)rxExpected,
               (unsigned long)rxMissed,
               (unsigned long)rxDoneCb,
               (unsigned long)rxIgnored,
               (unsigned long)rxAccept,
               (unsigned long)rxReject,
               (unsigned)nonce,
               (unsigned)tlmPhase,
               (unsigned)busy);
        printf("[TXRFDBG] delta timer=%lu dio=%lu attempts=%lu txDone=%lu "
               "txCb=%lu rxWin=%lu rxMiss=%lu rxCb=%lu rxOk=%lu "
               "rxReject=%lu\n",
               (unsigned long)(wakeTimer - lastWakeTimer),
               (unsigned long)(wakeDio - lastWakeDio),
               (unsigned long)deltaAttempts,
               (unsigned long)deltaTxDone,
               (unsigned long)(txDoneCb - lastTxDoneCb),
               (unsigned long)deltaRxWin,
               (unsigned long)deltaRxMiss,
               (unsigned long)(rxDoneCb - lastRxDoneCb),
               (unsigned long)deltaRxOk,
               (unsigned long)deltaRxReject);
#endif

        lastWakeTimer = wakeTimer;
        lastWakeDio = wakeDio;
        lastIrqTxDone = irqTxDone;
        lastIrqRxDone = irqRxDone;
        lastSendAttempts = sendAttempts;
        lastTxDoneCb = txDoneCb;
        lastRxWindowReq = rxWindowReq;
        lastRxMissed = rxMissed;
        lastRxDoneCb = rxDoneCb;
        lastRxAccept = rxAccept;
        lastRxReject = rxReject;
    }
};

SiW917BenchHandset benchHandset;

bool initialize()
{
    handset = &benchHandset;
    return true;
}

int start()
{
    handset->Begin();
    return DURATION_IMMEDIATELY;
}

int timeout()
{
    handset->handleInput();
    return DURATION_IMMEDIATELY;
}

} // namespace

extern "C" void cache_uart_rx_data(const char character)
{
    const uint16_t next = (uint16_t)((debugRxHead + 1U) & DEBUG_RX_RING_MASK);
    if (next == debugRxTail) {
        debugRxOverflow++;
        debugRxTail = (uint16_t)((debugRxTail + 1U) & DEBUG_RX_RING_MASK);
    }
    debugRxRing[debugRxHead] = (uint8_t)character;
    debugRxHead = next;
}

extern "C" void siw917_pcbench_capture_crsf_channels(const uint32_t *channels,
                                                      uint8_t channelCount,
                                                      uint8_t armed)
{
    benchHandset.CaptureCrsfChannels(channels, channelCount, armed != 0U);
}

device_t Handset_device = {
    .initialize = initialize,
    .start = start,
    .event = nullptr,
    .timeout = timeout,
    .subscribe = EVENT_NONE,
};

#endif
