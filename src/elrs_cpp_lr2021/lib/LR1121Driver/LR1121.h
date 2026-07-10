#pragma once

#include "LR1121_Regs.h"
#include "SX12xxDriverCommon.h"
#include "targets.h"

#ifdef PLATFORM_ESP8266
#include <cstdint>
#endif

#define RADIO_SNR_SCALE 4

typedef struct {
  uint8_t hardware;
  uint8_t type;
  uint16_t version;
} __attribute__((packed)) firmware_version_t;

class BufferCodec {
public:
  virtual ~BufferCodec() {}
  virtual void encode(uint8_t *out, uint8_t *in, uint32_t len);
  virtual void decode(uint8_t *out, uint8_t *in, uint32_t len);
};

class LR1121Driver : public SX12xxDriverCommon {
public:
  static LR1121Driver *instance;

  ////////////////Configuration Functions/////////////
  LR1121Driver();
  bool Begin(uint32_t minimumFrequency, uint32_t maximumFrequency);
  void End();
  void SetTxIdleMode() {
    SetMode(LR1121_MODE_FS, SX12XX_Radio_All);
  }; // set Idle mode used when switching from RX to TX
  void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
              uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength,
              bool setFSKModulation, uint8_t fskSyncWord1, uint8_t fskSyncWord2,
              SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_All);
  void SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber,
                       bool doRx = false, uint32_t rxTime = 0);
  void SetOutputPower(int8_t power, bool isSubGHz = true);
  bool HasPendingOutputPower() const;
  void CommitOutputPowerForNextTx();
  void startCWTest(uint32_t freq, SX12XX_Radio_Number_t radioNumber);

  bool GetFrequencyErrorbool(SX12XX_Radio_Number_t radioNumber);
  // bool FrequencyErrorAvailable() const { return modeSupportsFei &&
  // (LastPacketSNRRaw > 0); }
  bool FrequencyErrorAvailable() const { return false; }

  void TXnb(uint8_t *data, bool sendGeminiBuffer, uint8_t *dataGemini,
            SX12XX_Radio_Number_t radioNumber);
  bool LastTxStartSuccessful() const { return lastTxStartSuccessful; }
  void RXnb();
  void RXnbFromTxDone();
  bool TakeAutoRxAfterTxArmed();

  uint32_t GetIrqStatus(SX12XX_Radio_Number_t radioNumber);
  uint32_t PeekIrqStatus(SX12XX_Radio_Number_t radioNumber);
  void ClearIrqStatus(SX12XX_Radio_Number_t radioNumber);
  void ClearRxFifo(SX12XX_Radio_Number_t radioNumber);

  static void IsrCallback_1();
  static void IsrCallback_2();

  // For polling mode - call with pre-read IRQ status to avoid double-read
  static void IsrCallbackWithStatus(SX12XX_Radio_Number_t radioNumber,
                                    uint32_t irqStatus);

  void StartRssiInst(SX12XX_Radio_Number_t radioNumber);
  int8_t GetRssiInst(SX12XX_Radio_Number_t radioNumber);
  uint16_t GetErrors(SX12XX_Radio_Number_t radioNumber);
  uint8_t GetPacketType(SX12XX_Radio_Number_t radioNumber);
  void GetLoRaRxStats(SX12XX_Radio_Number_t radioNumber, uint16_t *pktRxTotal,
                      uint16_t *pktCrcError, uint16_t *headerCrcError,
                      uint16_t *falseSync);
  void GetGfskRxStats(SX12XX_Radio_Number_t radioNumber,
                      uint16_t *pktRxTotal, uint16_t *pktCrcError,
                      uint16_t *pktLenError, uint16_t *preambleDetected,
                      uint16_t *syncOk, uint16_t *syncFail,
                      uint16_t *timeout);
  void GetLastPacketStats();
  void CheckForSecondPacket();

  // Firmware update methods
  firmware_version_t
  GetFirmwareVersion(SX12XX_Radio_Number_t radioNumber,
                     uint16_t command = LR20XX_SYSTEM_GET_VERSION);
  int BeginUpdate(SX12XX_Radio_Number_t radioNumber, uint32_t expectedSize);
  int WriteUpdateBytes(const uint8_t *bytes, uint32_t size);
  int EndUpdate();

private:
  // constant used for no power change pending
  // must not be a valid power register value
  static const uint8_t PWRPENDING_NONE = 0x7f;

  // LR1121_RadioOperatingModes_t currOpmode;
  bool useFSK;
  bool rxContinuousActive;
  volatile bool txInProgress;
  volatile bool lastTxStartSuccessful;
  volatile bool autoRxAfterTxArmed;
  bool modeSupportsFei;
  uint8_t pwrCurrentLF;
  uint8_t pwrPendingLF;
  uint8_t pwrCurrentHF; // HF = High Frequency
  uint8_t pwrPendingHF;
  bool pwrForceUpdate;
  bool radio1isSubGHz;
  bool radio2isSubGHz;
  uint32_t feCalFreqRadio1;
  uint32_t feCalFreqRadio2;
  enum { FE_CAL_CACHE_SIZE = 3 };
  uint16_t feCalWordsRadio1[FE_CAL_CACHE_SIZE];
  uint16_t feCalWordsRadio2[FE_CAL_CACHE_SIZE];
  uint8_t feCalWordCountRadio1;
  uint8_t feCalWordCountRadio2;
  lr11xx_RadioOperatingModes_t fallBackMode;
  BufferCodec *codec;

  WORD_ALIGNED_ATTR uint8_t rx_buf[32] = {};
  WORD_ALIGNED_ATTR uint8_t rx2_buf[32] = {};

  bool CheckVersion(SX12XX_Radio_Number_t radioNumber);

  void SetMode(lr11xx_RadioOperatingModes_t OPmode,
               SX12XX_Radio_Number_t radioNumber);

  // LoRa functions
  void ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr,
                           SX12XX_Radio_Number_t radioNumber);
  void SetPacketParamsLoRa(uint8_t PreambleLength,
                           lr11xx_RadioLoRaPacketLengthsModes_t HeaderType,
                           uint8_t PayloadLength, uint8_t InvertIQ,
                           SX12XX_Radio_Number_t radioNumber);
  void SetLoRaSyncWord(uint8_t syncWord, uint8_t sf,
                       SX12XX_Radio_Number_t radioNumber);
  void ConfigureLoRaRxDetector(uint8_t sf, uint8_t InvertIQ,
                               SX12XX_Radio_Number_t radioNumber);
  // FSK functions
  void ConfigModParamsFSK(uint32_t Bitrate, uint8_t BWF, uint32_t Fdev,
                          SX12XX_Radio_Number_t radioNumber);
  void SetPacketParamsFSK(uint8_t PreambleLength, uint8_t PayloadLength,
                          SX12XX_Radio_Number_t radioNumber);
  void SetFSKWhiteningParams(SX12XX_Radio_Number_t radioNumber);
  void SetFSKSyncWord(uint8_t fskSyncWord1, uint8_t fskSyncWord2,
                      SX12XX_Radio_Number_t radioNumber);

  void SetDioIrqParams();
  void SetDioFunctionIrq(SX12XX_Radio_Number_t radioNumber);
  void SetRxTimeoutStopOnPreamble(bool stopOnPreamble,
                                  SX12XX_Radio_Number_t radioNumber);
  void SetRxPath(bool isSubGHz, SX12XX_Radio_Number_t radioNumber);
  void ConfigureRegulatorMode(SX12XX_Radio_Number_t radioNumber);
  void CalibrateAll(SX12XX_Radio_Number_t radioNumber);
  bool CalibrateFrontEndDefaultSet(SX12XX_Radio_Number_t radioNumber,
                                   bool force = false);
  bool CalibrateFrontEndWords(const uint16_t *calWords, uint8_t count,
                              SX12XX_Radio_Number_t radioNumber,
                              bool force = false);
  bool CalibrateFrontEndForFrequency(uint32_t freqHz,
                                     SX12XX_Radio_Number_t radioNumber,
                                     bool force = false);
  bool CalibrateFrontEnd24GSet(SX12XX_Radio_Number_t radioNumber,
                               bool force = false);
  static uint16_t FrontEndCalWordForFrequency(uint32_t freqHz);
  bool HasFrontEndCalWord(uint16_t calWord,
                          SX12XX_Radio_Number_t radioNumber) const;
  bool HasFrontEndCalCoverage(uint16_t calWord,
                              SX12XX_Radio_Number_t radioNumber) const;
  void MarkFrontEndCalWord(uint16_t calWord,
                           SX12XX_Radio_Number_t radioNumber);
  void ResetFrontEndCalCache();
  void SeedFrontEndCalibrationRange(uint32_t minimumFrequency,
                                    uint32_t maximumFrequency,
                                    SX12XX_Radio_Number_t radioNumber);
  void ClearIrqStatusMask(uint32_t irqMask, SX12XX_Radio_Number_t radioNumber);
  void WriteRegMem32(uint32_t addr, uint32_t data,
                     SX12XX_Radio_Number_t radioNumber);
  uint32_t ReadRegMem32(uint32_t addr, SX12XX_Radio_Number_t radioNumber);
  void WriteRegMemMask32(uint32_t addr, uint32_t mask, uint32_t data,
                         SX12XX_Radio_Number_t radioNumber);
  void ConfigureLoraSx1276Compatibility(uint8_t sf,
                                        SX12XX_Radio_Number_t radioNumber);
  void ConfigureLoraFrequencyRange(uint8_t sf,
                                   SX12XX_Radio_Number_t radioNumber);
  void ApplyDcdcReset(SX12XX_Radio_Number_t radioNumber);
  void ApplyDcdcConfigure(SX12XX_Radio_Number_t radioNumber);
  void SetDcdcFrequency(uint32_t frequencyHz,
                        SX12XX_Radio_Number_t radioNumber);

  static void IsrCallback(SX12XX_Radio_Number_t radioNumber);

  void DecodeRssiSnr(SX12XX_Radio_Number_t radioNumber, const uint8_t *buf);

  bool
  RXnbISR(SX12XX_Radio_Number_t radioNumber); // ISR for non-blocking RX routine
  void TXnbISR();                             // ISR for non-blocking TX routine
  void CommitOutputPower();
  void WriteOutputPower(uint8_t pwr, bool isSubGHz,
                        SX12XX_Radio_Number_t radioNumber);
  void SetPaConfig(bool isSubGHz, SX12XX_Radio_Number_t radioNumber);
};
