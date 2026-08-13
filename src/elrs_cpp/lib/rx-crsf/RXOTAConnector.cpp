#include "RXOTAConnector.h"

#include "logging.h"

#define RXOTA_DEL_BIT (1U << 7)
#define IS_DEL(size) (((size) & RXOTA_DEL_BIT) != 0)
#define SET_DEL(size) ((size) | RXOTA_DEL_BIT)
#define SIZE(size) ((size) & ~RXOTA_DEL_BIT)
#define RXOTA_DIAG 0

enum action_e {
  ACTION_NEXT,
  ACTION_IGNORE,
  ACTION_OVERWRITE,
  ACTION_APPEND,
};

static bool isPrioritised(crsf_frame_type_e frameType) {
  return frameType >= CRSF_FRAMETYPE_DEVICE_PING &&
         frameType <= CRSF_FRAMETYPE_PARAMETER_WRITE;
}

static action_e compareQueuedMessage(const crsf_header_t *newMessage,
                                     const TelemetryFifo &payloads,
                                     uint16_t queuePosition) {
  const auto frameType = (crsf_frame_type_e)newMessage->type;

  if (frameType == CRSF_FRAMETYPE_DEVICE_INFO) {
    const auto *ext = (const crsf_ext_header_t *)newMessage;
    if (payloads[queuePosition + 3] == ext->dest_addr &&
        payloads[queuePosition + 4] == ext->orig_addr) {
      return ACTION_OVERWRITE;
    }
    return ACTION_NEXT;
  }

  if (frameType == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY) {
    return ACTION_NEXT;
  }

  if (frameType < CRSF_FRAMETYPE_DEVICE_PING) {
    return ACTION_OVERWRITE;
  }

  return ACTION_NEXT;
}

RXOTAConnector::RXOTAConnector() {
  addDevice(CRSF_ADDRESS_RADIO_TRANSMITTER);
  addDevice(CRSF_ADDRESS_CRSF_TRANSMITTER);
}

bool RXOTAConnector::GetNextPayload(uint8_t *nextPayloadSize,
                                    uint8_t *payloadData) {
  if (prioritizedCount != 0) {
    for (uint16_t i = 0; i < messagePayloads.size();) {
      const auto size = messagePayloads[i];
      if (isPrioritised(
              (crsf_frame_type_e)messagePayloads[i + 1 + CRSF_TELEMETRY_TYPE_INDEX])) {
        if (!IS_DEL(size)) {
          --prioritizedCount;
          if (i == 0) {
            messagePayloads.pop();
            messagePayloads.popBytes(payloadData, size);
          } else {
            for (uint16_t pos = 0; pos < size; ++pos) {
              payloadData[pos] = messagePayloads[i + 1 + pos];
            }
            messagePayloads.set(i, SET_DEL(size));
          }
          *nextPayloadSize =
              CRSF_FRAME_SIZE(payloadData[CRSF_TELEMETRY_LENGTH_INDEX]);
#if RXOTA_DIAG
          DBGLN("[RXOTA] dequeue-pri type=0x%02X len=%u fifo=%u pri=%u",
                payloadData[CRSF_TELEMETRY_TYPE_INDEX], *nextPayloadSize,
                messagePayloads.size(), prioritizedCount);
#endif
          return true;
        }
      }
      i += 1 + SIZE(size);
    }
    prioritizedCount = 0;
  }

  while (messagePayloads.size() > 0) {
    const auto size = messagePayloads.pop();
    if (IS_DEL(size)) {
      messagePayloads.skip(SIZE(size));
      continue;
    }
    messagePayloads.popBytes(payloadData, size);
    *nextPayloadSize =
        CRSF_FRAME_SIZE(payloadData[CRSF_TELEMETRY_LENGTH_INDEX]);
#if RXOTA_DIAG
    DBGLN("[RXOTA] dequeue type=0x%02X len=%u fifo=%u pri=%u",
          payloadData[CRSF_TELEMETRY_TYPE_INDEX], *nextPayloadSize,
          messagePayloads.size(), prioritizedCount);
#endif
    return true;
  }

  *nextPayloadSize = 0;
  return false;
}

void RXOTAConnector::forwardMessage(const crsf_header_t *message) {
  const uint8_t messageSize =
      CRSF_FRAME_SIZE(((const uint8_t *)message)[CRSF_TELEMETRY_LENGTH_INDEX]);
  action_e action = ACTION_APPEND;
  uint16_t overwritePosition = 0;

  if (message->type == CRSF_FRAMETYPE_DEVICE_INFO ||
      message->type == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY ||
      message->type < CRSF_FRAMETYPE_DEVICE_PING) {
    for (uint16_t i = 0; i < messagePayloads.size();) {
      const auto size = messagePayloads[i];
      if (!IS_DEL(size) &&
          messagePayloads[i + 1 + CRSF_TELEMETRY_TYPE_INDEX] == message->type) {
        const auto compare =
            compareQueuedMessage(message, messagePayloads, i + 1);
        if (compare != ACTION_NEXT) {
          overwritePosition = i;
          action = compare;
          break;
        }
      }
      i += 1 + SIZE(size);
    }
  }

  if (isPrioritised(message->type)) {
    ++prioritizedCount;
  }

  switch (action) {
  case ACTION_IGNORE:
#if RXOTA_DIAG
    DBGLN("[RXOTA] ignore type=0x%02X len=%u fifo=%u", message->type,
          messageSize, messagePayloads.size());
#endif
    break;
  case ACTION_OVERWRITE:
    if (!IS_DEL(messagePayloads[overwritePosition])) {
      if (messagePayloads[overwritePosition] >= messageSize) {
        for (uint16_t i = 0; i < messageSize; ++i) {
          messagePayloads.set(overwritePosition + i + 1,
                              ((const uint8_t *)message)[i]);
        }
        break;
      }
      messagePayloads.set(overwritePosition,
                          SET_DEL(messagePayloads[overwritePosition]));
    }
    [[fallthrough]];
  case ACTION_APPEND:
  case ACTION_NEXT:
  default:
    while (!messagePayloads.available(messageSize + 1)) {
      const uint8_t sz = SIZE(messagePayloads.pop());
      messagePayloads.skip(sz);
#if RXOTA_DIAG
      DBGLN("[RXOTA] drop-old size=%u need=%u fifo=%u", sz, messageSize,
            messagePayloads.size());
#endif
    }
    messagePayloads.push(messageSize);
    messagePayloads.pushBytes((const uint8_t *)message, messageSize);
#if RXOTA_DIAG
    DBGLN("[RXOTA] enqueue type=0x%02X len=%u fifo=%u pri=%u action=%u",
          message->type, messageSize, messagePayloads.size(), prioritizedCount,
          action);
#endif
    break;
  }
}
