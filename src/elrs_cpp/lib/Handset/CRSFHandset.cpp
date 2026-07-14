#include "CRSFHandset.h"

#include <cstring>

#include "CRSFRouter.h"
#include "FIFO.h"
#include "common.h"
#include "crsf_serial.h"
#include "logging.h"
#include "options.h"

static constexpr int HANDSET_TELEMETRY_FIFO_SIZE = 128;
static constexpr auto CRSF_SERIAL_OUT_FIFO_SIZE = 256U;
static FIFO<CRSF_SERIAL_OUT_FIFO_SIZE> SerialOutFIFO;

static constexpr int32_t OpenTXsyncPacketInterval = 200;
static constexpr int32_t OpenTXsyncOffsetSafeMargin = 1000;
static constexpr uint32_t UARTwdtInterval = 1000;

static const uint32_t TxToHandsetBauds[] = {400000U, 115200U, 5250000U,
                                            3750000U, 1870000U, 921600U,
                                            2250000U};

uint32_t CRSFHandset::GoodPktsCountResult = 0;
uint32_t CRSFHandset::BadPktsCountResult = 0;
uint8_t CRSFHandset::UARTcurrentBaudIdx = 6;
uint32_t CRSFHandset::UARTrequestedBaud = 400000U;

void CRSFHandset::Begin() {
  DBGLN("About to start CRSF task...");

  addDevice(CRSF_ADDRESS_ELRS_LUA);
  addDevice(CRSF_ADDRESS_RADIO_TRANSMITTER);
  crsfRouter.addConnector(this);

  UARTwdtLastChecked = millis() + UARTwdtInterval;
  controllerConnected = false;
  SerialInPacketPtr = 0;
  GoodPktsCount = 0;
  BadPktsCount = 0;
  RxBytesObserved = 0;
  UARTidleReported = false;
  transmitting = false;

  if (firmwareOptions.uart_baud != 0U) {
    UARTrequestedBaud = firmwareOptions.uart_baud;
  }

  adjustMaxPacketSize();

  if (crsf_serial_init(UARTrequestedBaud) != 0) {
    ERRLN("CRSF serial init failed");
    return;
  }

  if (crsf_serial_set_rx_enabled(true) != 0) {
    ERRLN("CRSF serial RX enable failed");
  }

  flush_port_input();
}

void CRSFHandset::End() {
  uint32_t startTime = millis();
  while (SerialOutFIFO.peek() > 0) {
    handleInput();
    if (millis() - startTime > 1000U) {
      break;
    }
  }

  (void)crsf_serial_set_rx_enabled(false);
  crsf_serial_deinit();
  SerialOutFIFO.flush();
  controllerConnected = false;
  DBGLN("CRSF UART END");
}

void CRSFHandset::flush_port_input() {
  uint8_t discard[32];
  while (crsf_serial_rx_available() > 0U) {
    (void)crsf_serial_read(discard, sizeof(discard));
  }
}

void CRSFHandset::forwardMessage(const crsf_header_t *message) {
  if (!controllerConnected) {
    return;
  }

  const uint8_t size = CRSF_FRAME_SIZE(message->frame_size);
  if (size > CRSF_MAX_PACKET_LEN) {
    ERRLN("too large");
    return;
  }

  uint8_t frame[CRSF_MAX_PACKET_LEN] = {};
  memcpy(frame, message, size);
  frame[0] = CRSF_SYNC_BYTE;

  SerialOutFIFO.lock();
  if (SerialOutFIFO.ensure(size + 1U)) {
    SerialOutFIFO.push(size);
    SerialOutFIFO.pushBytes(frame, size);
  }
  SerialOutFIFO.unlock();
}

void ICACHE_RAM_ATTR CRSFHandset::setPacketInterval(int32_t PacketInterval) {
  RequestedRCpacketInterval = PacketInterval;
  OpenTXsyncOffset = 0;
  OpenTXsyncWindow = 0;
  OpenTXsyncWindowSize =
      std::max((int32_t)1, (int32_t)(20000 / RequestedRCpacketInterval));
  OpenTXsyncLastSent -= OpenTXsyncPacketInterval;
  adjustMaxPacketSize();
}

void ICACHE_RAM_ATTR CRSFHandset::JustSentRFpacket() {
  uint32_t last = dataLastRecv;
  uint32_t now = micros();
  auto delta = (int32_t)(now - last);

  if (delta >= RequestedRCpacketInterval) {
    OpenTXsyncOffset = -(delta % RequestedRCpacketInterval) * 10;
    OpenTXsyncWindow = 0;
    OpenTXsyncLastSent -= OpenTXsyncPacketInterval;
  } else {
    OpenTXsyncWindow = std::min<int32_t>(
        (int32_t)(OpenTXsyncWindow + 1), (int32_t)OpenTXsyncWindowSize);
    OpenTXsyncOffset =
        ((OpenTXsyncOffset * (OpenTXsyncWindow - 1)) + delta * 10) /
        OpenTXsyncWindow;
  }
}

void CRSFHandset::sendSyncPacketToTX() {
  const uint32_t now = millis();
  if (now - OpenTXsyncLastSent < OpenTXsyncPacketInterval) {
    return;
  }

  int32_t packetRate = RequestedRCpacketInterval * 10;
  int32_t offset = OpenTXsyncOffset - OpenTXsyncOffsetSafeMargin;

  CRSF_MK_EXT_FRAME_T(crsf_sync_packet_t) syncPacket = {};
  syncPacket.h.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
  syncPacket.h.frame_size = CRSF_EXT_FRAME_SIZE(sizeof(crsf_sync_packet_t));
  syncPacket.h.type = CRSF_FRAMETYPE_HANDSET;
  syncPacket.h.dest_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
  syncPacket.h.orig_addr = CRSF_ADDRESS_CRSF_TRANSMITTER;
  syncPacket.p.subType = CRSF_HANDSET_SUBCMD_TIMING;
  syncPacket.p.rate = htobe32(packetRate);
  syncPacket.p.offset = htobe32(offset);
  syncPacket.crc = crsfRouter.crsf_crc.calc(
      (uint8_t *)&syncPacket + CRSF_TELEMETRY_TYPE_INDEX,
      sizeof(syncPacket) - 3);
  crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER,
                              (crsf_header_t *)&syncPacket);

  OpenTXsyncLastSent = now;
}

bool CRSFHandset::ProcessPacket() {
  dataLastRecv = micros();

  if (!controllerConnected) {
    controllerConnected = true;
    UARTidleReported = false;
    DBGLN("CRSF UART Connected");
    if (connected) {
      connected();
    }
  }

  const auto *message = (const crsf_header_t *)&inBuffer[0];
  crsfRouter.processMessage(this, message);
  return true;
}

void CRSFHandset::alignBufferToSync(uint8_t startIdx) {
  for (unsigned i = startIdx; i < SerialInPacketPtr; ++i) {
    if (inBuffer[i] == CRSF_ADDRESS_CRSF_TRANSMITTER ||
        inBuffer[i] == CRSF_SYNC_BYTE) {
      SerialInPacketPtr -= i;
      memmove(inBuffer, &inBuffer[i], SerialInPacketPtr);
      return;
    }
  }

  SerialInPacketPtr = 0;
}

void CRSFHandset::handleInput() {
  if (UARTwdt()) {
    return;
  }

  const uint32_t available = crsf_serial_rx_available();
  if (available == 0U) {
    handleOutput(0);
    return;
  }

  UARTidleReported = false;

  const uint32_t toRead =
      std::min<uint32_t>(available, CRSF_MAX_PACKET_LEN - SerialInPacketPtr);
  const uint32_t bytesRead =
      crsf_serial_read(&inBuffer[SerialInPacketPtr], toRead);
  SerialInPacketPtr += (uint8_t)bytesRead;
  RxBytesObserved += bytesRead;
  alignBufferToSync(0);

  if (SerialInPacketPtr < 3U) {
    handleOutput(0);
    return;
  }

  const uint32_t totalLen = inBuffer[1] + 2U;
  if (totalLen < 4U || totalLen > CRSF_MAX_PACKET_LEN) {
    alignBufferToSync(1);
    handleOutput(0);
    return;
  }

  if (SerialInPacketPtr < totalLen) {
    handleOutput(0);
    return;
  }

  const uint8_t calculatedCrc =
      crsfRouter.crsf_crc.calc(&inBuffer[2], totalLen - 3U);
  if (calculatedCrc == inBuffer[totalLen - 1U]) {
    GoodPktsCount++;
    ProcessPacket();
    handleOutput(totalLen);
  } else {
    DBGLN("UART CRC failure");
    BadPktsCount++;
    handleOutput(0);
  }

  SerialInPacketPtr -= (uint8_t)totalLen;
  memmove(inBuffer, &inBuffer[totalLen], SerialInPacketPtr);
}

void CRSFHandset::handleOutput(uint32_t receivedBytes) {
  (void)receivedBytes;

  if (!controllerConnected) {
    SerialOutFIFO.lock();
    SerialOutFIFO.flush();
    SerialOutFIFO.unlock();
    return;
  }

  if (SerialOutFIFO.size() == 0U) {
    sendSyncPacketToTX();
  }

  if (SerialOutFIFO.size() == 0U) {
    return;
  }

  uint8_t frame[CRSF_MAX_PACKET_LEN] = {};
  uint8_t packetLength = 0;

  SerialOutFIFO.lock();
  packetLength = SerialOutFIFO.peek();
  if (packetLength == 0U || SerialOutFIFO.size() < (uint16_t)(packetLength + 1U)) {
    SerialOutFIFO.unlock();
    return;
  }
  for (uint8_t i = 0; i < packetLength; ++i) {
    frame[i] = SerialOutFIFO[i + 1U];
  }
  SerialOutFIFO.unlock();

  if (crsf_serial_send_frame(frame, packetLength) == 0) {
    SerialOutFIFO.lock();
    SerialOutFIFO.skip(packetLength + 1U);
    SerialOutFIFO.unlock();
  }
}

int CRSFHandset::getMinPacketInterval() const {
  if (UARTrequestedBaud <= 115200U) {
    return 4000;
  }
  if (UARTrequestedBaud <= 420000U) {
    return 2000;
  }
  return 1;
}

void ICACHE_RAM_ATTR CRSFHandset::adjustMaxPacketSize() {
  const int luaChunkQuerySize = 26;
  const int32_t packetsPerSecond =
      std::max<int32_t>(1, 1000000 / RequestedRCpacketInterval);
  const int32_t calculatedPeriodBytes =
      (int32_t)((UARTrequestedBaud / 10U) / packetsPerSecond * 87 / 100);

  maxPeriodBytes = (uint8_t)std::max<int32_t>(
      15, std::min<int32_t>(calculatedPeriodBytes,
                            HANDSET_TELEMETRY_FIFO_SIZE));

  const int32_t packetBudget =
      maxPeriodBytes - std::max<int32_t>(maxPeriodBytes / 2, luaChunkQuerySize);
  maxPacketBytes = (uint8_t)std::max<int32_t>(
      15, std::min<int32_t>(packetBudget, CRSF_MAX_PACKET_LEN));
  DBGLN("Adjusted max packet size %u-%u", maxPacketBytes, maxPeriodBytes);
}

uint32_t CRSFHandset::autobaud() {
  UARTcurrentBaudIdx =
      (uint8_t)((UARTcurrentBaudIdx + 1U) %
                (sizeof(TxToHandsetBauds) / sizeof(TxToHandsetBauds[0])));
  return TxToHandsetBauds[UARTcurrentBaudIdx];
}

bool CRSFHandset::UARTwdt() {
  bool retval = false;

  const uint32_t now = millis();
  if (now - UARTwdtLastChecked <= UARTwdtInterval) {
    return false;
  }

  const bool idleNoHandset =
      !controllerConnected && GoodPktsCount == 0U && BadPktsCount == 0U &&
      RxBytesObserved == 0U && SerialInPacketPtr == 0U;

  if (connectionState != wifiUpdate && idleNoHandset) {
    if (!UARTidleReported) {
#if defined(SIW917_ELRS_TX_PC_BENCH)
      DBGLN("CRSF UART idle - PC bench is using internal RC channels");
#else
      DBGLN("CRSF UART idle - waiting for handset data");
#endif
      UARTidleReported = true;
    }
  } else if (connectionState != wifiUpdate &&
             (BadPktsCount >= GoodPktsCount || !controllerConnected)) {
    UARTidleReported = false;
    DBGLN("Too many bad UART RX packets!");

    if (controllerConnected) {
      DBGLN("CRSF UART Disconnected");
      if (disconnected) {
        disconnected();
      }
      controllerConnected = false;
    }

    UARTrequestedBaud = autobaud();
    DBGLN("UART WDT: Switch to: %lu baud",
          (unsigned long)UARTrequestedBaud);

    adjustMaxPacketSize();

    SerialOutFIFO.flush();
    (void)crsf_serial_set_rx_enabled(false);
    crsf_serial_deinit();

    if (crsf_serial_init(UARTrequestedBaud) == 0) {
      if (crsf_serial_set_rx_enabled(true) != 0) {
        ERRLN("CRSF serial RX enable failed");
      }
      flush_port_input();
    } else {
      ERRLN("CRSF serial init failed");
    }

    retval = true;
  }

  DBGVLN("UART STATS Bad:Good = %lu:%lu", (unsigned long)BadPktsCount,
         (unsigned long)GoodPktsCount);

  UARTwdtLastChecked = now;
  if (retval) {
    UARTwdtLastChecked -= 3U * (UARTwdtInterval >> 2);
  }

  GoodPktsCountResult = GoodPktsCount;
  BadPktsCountResult = BadPktsCount;
  RxBytesObserved = 0;
  BadPktsCount = 0;
  GoodPktsCount = 0;
  return retval;
}
