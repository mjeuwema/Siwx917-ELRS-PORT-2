# SiW917 ELRS LR2021 Port Seed

This folder is a copy of the working Gemini/crossband probe port from
`codex/gemini-crossband-probe:src/elrs_cpp`. The existing `src/elrs_cpp`
tree is intentionally untouched.

## Current State

This is an in-progress LR2021 hardware port with a working CMake build variant,
but it still needs hardware bring-up on the Core2021-XF module. The
LR2021/LR2022/LR2012 datasheet uses the LR20xx host command set, which is not
opcode-compatible with the copied LR1121 driver:

- LR20xx FIFO access is `ReadRadioRxFifo` / `WriteRadioTxFifo`
  (`0x0001` / `0x0002`) instead of LR11xx buffer commands.
- LR20xx radio commands move common operations to `0x0200` and up:
  `SetRfFrequency=0x0200`, `SetRxPath=0x0201`, `SetPaConfig=0x0202`,
  `SetTxParams=0x0203`, `SetPacketType=0x0207`, `SetRx=0x020C`,
  `SetTx=0x020D`, and `SelPa=0x020F`.
- LR20xx LoRa/GFSK packet commands are split into `SetLora*` and `SetFsk*`
  command families rather than LR1121's shared packet/modulation commands.
- LR20xx IRQ routing uses `SetDioIrqConfig` and `GetAndClearIrqStatus`, not
  the LR1121 `SetDioIrqParams` / `ClearIrq` behavior currently used here.

`lib/LR1121Driver/LR20xx_Regs.h` contains the LR20xx command map extracted
from the LR20xx datasheet V2.1 linked in the task and cross-checked against
Waveshare's Core2021-XF RadioLib LR2021 implementation.

The active C++ runtime path now uses LR20xx opcodes and values for packet type,
LoRa/FSK modulation and packet params, RF frequency, RX/TX, RSSI, IRQ routing,
FIFO read/write, packet status, PA config, and RX path selection. The inherited
LR1121 RF-switch programming path has been removed from this copy because the
Core2021-XF does not expose a sub-GHz RF switch control path.

Waveshare's RadioLib reference and Arduino examples added these important
bring-up details:

- LR2021 packet types are not the same values as LR1121; LoRa is `0x00` and
  GFSK is `0x02`.
- The selected interrupt DIO must first be configured with `SetDioFunction`
  before `SetDioIrqConfig` routes IRQ bits to it.
- Core2021-XF examples set `radio.irqDioNum = 11` before `begin()`, matching
  this port's default `LR2021_IRQ_DIO_NUM=11`.
- Core2021-XF examples set `radio.XTAL = true`, so this port keeps the LR2021
  on the external 32 MHz crystal/XOSC path and does not enable `SetTcxoMode`.
- The LR2021 path needs `SetRegMode`, `Calibrate`, and `CalibFE`. This copy
  now seeds `CalibFE` at the runtime band edges and refreshes it when a rate
  changes frequency by at least 20 MHz, while keeping calibration out of the
  FHSS per-packet hot path.

Remaining hardware work:

- Verify the LR2021 CMake variant on Core2021-XF hardware using the generated
  `cmake_gcc/build_lr2021/base/wifi_gspi_merged.rps` image.
- Eventually replace or fork the lower-level `src/lr1121_driver.*` helper layer;
  the
  copied HAL still includes it for GSPI, reset, BUSY, chip-select, and DIO
  plumbing. The LR1121-specific `lr1121_elrs_init.*` helper is no longer
  included by this LR2021 copy.
- Verify the SiW917 carrier-board wiring still connects the Core2021-XF DIO11
  module pin to GPIO_46. The LR20xx IRQ route now defaults to
  `LR2021_IRQ_DIO_NUM=11`.
- Verify LR2021 XTAL/XOSC and `CalibFE` bring-up on hardware. The old
  `lr1121_waveshare_init()` sequence is no longer called because it programs
  LR1121-specific calibration/switch state.
- Tune the PA tables for the actual RF matching network rather than assuming
  Semtech reference-design values.

## Hardware Notes

The LR2021 is the full-featured dual-band LR20xx part: sub-GHz plus
1.5-2.5 GHz operation. That makes it the LR20xx-family candidate for an ELRS
crossband/Gemini experiment. LR2012 is sub-GHz only and would not support the
2.4 GHz half of this port.

Waveshare's Core2021-XF pinout labels the module-side interrupt pin as DIO11,
with DIO8 and DIO7 available as GPIO pins. The antenna pins are separated as
LORA_ANT and 2.4G_ANT, so this port uses LR20xx `SetRxPath` instead of the
old LR1121 `SetDioAsRfSwitch` command.

## Build Wiring

The default build still points at `src/elrs_cpp`. The LR2021 variant is selected
with the CMake preset `project_lr2021`, which sets
`ELRS_CPP_DIR=../src/elrs_cpp_lr2021` and writes outputs under
`cmake_gcc/build_lr2021`.
