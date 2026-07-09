# LR2021 Migration Checklist

This checklist is scoped to the copied Gemini/crossband tree in this folder.
It should be completed before wiring `src/elrs_cpp_lr2021` into the main CMake
target.

## Driver Surface

- `lib/LR1121Driver/LR1121.cpp`
  - Done: LR1121 firmware update entry points are stubbed out. The bundled
    `F30104` LR1121 image and LR11xx bootloader commands are not valid LR2021
    firmware handling.
  - Done for the active C++ runtime path: replaced the main system/radio
    command opcodes with `LR20XX_*` equivalents.
  - Done: replaced the old disabled `CalibrateImage` breadcrumb with LR20xx
    `CalibFE` helpers based on Waveshare's RadioLib LR2021 implementation.
    Bring-up seeds the configured band edges; rate changes refresh `CalibFE`
    when the requested frequency moves by at least 20 MHz. This deliberately
    stays out of the FHSS per-packet hot path.
  - Done: corrected LR2021 packet type values from the Waveshare RadioLib
    reference: LoRa is `0x00`, GFSK is `0x02`.
  - Done: `SetDioFunction` now configures the selected DIO as an IRQ pin before
    `SetDioIrqConfig` routes `TxDone`/`RxDone`.
  - Done: LR2021 `SetRegMode` and general `Calibrate(0x6F)` are now sent
    during `Begin()`.
  - Done: removed the `SetDioAsRfSwitch` path. Waveshare's Core2021-XF pinout
    exposes DIO11 as the interrupt pin plus DIO8/DIO7 GPIO, but no sub-GHz
    RF-switch control line for this port to program.
  - Done: `SetDioIrqParams` now emits LR20xx `SetDioIrqConfig`.
  - Done: updated IRQ bit masks:
    - `RxDone` is bit 18.
    - `TxDone` is bit 19.
    - `Timeout` is bit 21.
    - `CrcError` is bit 22.
  - Done: replaced LR1121 experimental combined commands:
    - `LR11XX_RADIO_SET_FREQ_SET_RX`
    - `LR11XX_RADIO_GET_PACKET`
    - `LR11XX_RADIO_WRITE_BUFFER8_SET_TX`
    These are not present in the LR20xx datasheet command table.
  - Done: replaced RX FIFO reads with `ReadRadioRxFifo` (`0x0001`) and TX FIFO
    writes with `WriteRadioTxFifo` (`0x0002`).
  - Done: replaced shared LR1121 modulation/packet commands with:
    - LoRa: `SetLoraModulationParams`, `SetLoraPacketParams`,
      `SetLoraSyncword`, `GetLoraPacketStatus`.
    - FSK: `SetFskModulationParams`, `SetFskPacketParams`,
      `SetFskWhiteningParams`, `SetFskCrcParams`, `SetFskSyncword`,
      `GetFskPacketStatus`.
  - Done: replaced PA control with LR20xx `SetPaConfig`, `SelPa`, and
    `SetTxParams`. The current values are reference-design placeholders pending
    Core2021-XF RF matching confirmation.

- `hal/LR1121_hal.cpp`
  - Keep the SiW917 GSPI/BUSY/chip-select structure.
  - Done for packet RX: `ReadRadioRxFifo` is handled as a direct full-duplex
    FIFO read.
  - Done: removed `lr1121_waveshare_init()` from the LR2021 HAL init/reset
    path so the old LR1121 switch/calibration sequence is not sent.
  - Still needed: fork/replace the low-level C helper layer currently included
    as `src/lr1121_driver.*`.
  - Still needed: verify LR2021-specific XTAL/XOSC and `CalibFE` bring-up on
    the actual Core2021-XF hardware.
  - Replace any remaining LR1121-specific reset, bootloader, or oscillator
    assumptions with the LR20xx timing from the datasheet.

- `hal/hardware.h`
  - Done: Core2021-XF module-side pinout confirmed from Waveshare docs:
    `DIO11` is the interrupt pin; `DIO8` and `DIO7` are GPIO pins; antenna pins
    are separate `LORA_ANT` and `2.4G_ANT`.
  - Still needed: confirm the SiW917 carrier-board wiring maps Core2021-XF
    DIO11 to GPIO_46 as inherited from the working LR1121 probe.

## ELRS Layer

- Keep `RADIO_LR1121` defined until the ELRS rate tables are generalized,
  because `FHSS`, `common.cpp`, and the OTA/rate code key off LR1121 dual-band
  radio types.
- Add a new radio type only after the LR2021 driver can provide the same ELRS
  semantics as the LR1121 path.
- Preserve the crossband gating from `siw917_elrs_timing.h`:
  `SIW917_ELRS_ENABLE_CROSSBAND_RATES` should remain the switch for enabling
  rates 18/19.

## First Bring-Up Target

1. Single LR2021 version/status probe over GSPI.
2. Standby XTAL/XOSC bring-up.
3. Sub-GHz LoRa RX only.
4. Sub-GHz telemetry TX return-to-RX.
5. 2.4 GHz LoRa RX only.
6. Dual-radio same-band Gemini.
7. LR2021 crossband rates.

That order keeps basic host-command compatibility ahead of the tight ELRS
timing path.
