# Working TX Checkpoint

Date: 2026-07-21

Git tag: `checkpoint/tx-lua-atomic-dma-2026-07-21`

This checkpoint records the confirmed working SiW917 ExpressLRS transmitter
state after restoring atomic UART baud-change RX rearming and verifying the
complete EdgeTX Lua request/reply path. High-volume CRSF protocol diagnostics
are disabled in the checkpoint firmware.

## Firmware Configuration

- Role: ExpressLRS transmitter
- ELRS core: upstream `tx_main.cpp`
- Radio: single LR1121, FCC 915 MHz plus 2.4 GHz modes
- Handset transport: direct two-wire UART1 for the custom TX16S EdgeTX build
- UART: normal polarity, 8N1, upstream baud scan with 1.87 Mbaud preferred
- SiW917 RX: GPIO_6, BRD2605A breakout pin 21
- SiW917 TX: GPIO_7, BRD2605A breakout pin 24
- Adapter/TX_OE_N: not used by the two-wire build
- PC bench mode: disabled
- CRSF protocol diagnostics: disabled
- Radio return diagnostic sweep: disabled

Build options:

```text
SIW917_ELRS_TARGET_TX=ON
SIW917_ELRS_CRSF_BENCH_2WIRE=ON
SIW917_ELRS_TX_PC_BENCH=OFF
SIW917_ELRS_USE_UPSTREAM_TX_MAIN=ON
SIW917_CRSF_RADIO_RETURN_DIAG=OFF
SIW917_ELRS_EVENT_COUNTERS=OFF
```

## Checkpoint Firmware

Canonical build output:

```text
C:\Users\mjeuw\OneDrive\Documents\ELRS TX\cmake_gcc\build-tx-clean-port\base\wifi_gspi_merged.rps
```

Stable local copy:

```text
C:\Users\mjeuw\OneDrive\Documents\ELRS TX\firmware\SiW917_ELRS_TX_TwoWire_Checkpoint_9C66E512.rps
```

```text
Size:   363348 bytes
SHA256: 9C66E512B28D6B35DBACA5660C3BBBEE017F95A442F4179EB6BC164D036D0257
```

## Upstream Source Identity

The build uses the local ExpressLRS checkout at:

```text
C:\Users\mjeuw\OneDrive\Documents\ELRS TX\upstream_expresslrs_master
```

Base commit:

```text
cfa88c0bb3e686e104a237d7239bdbbeb3ed4e9e
```

The exact four-file upstream working-tree delta is captured by:

```text
patches\siw917-tx-upstream-cfa88c0.patch
SHA256: AC4C341ACA8781DB3A4694234B302A0066A2C1C3F0E3CB7D65FE076B9BE244BF
Patch ID: 570e0b4d5d4fdde8a2194e017bed3bb3532570bd
```

The patch was verified by applying it to a temporary clean worktree at the
base commit.

## Build

```powershell
cd "C:\Users\mjeuw\OneDrive\Documents\ELRS TX\cmake_gcc"
$env:ELRS_UPSTREAM_DIR = "C:\Users\mjeuw\OneDrive\Documents\ELRS TX\upstream_expresslrs_master\src"
cmake --workflow --preset tx-clean-port
```

## Radio Wiring

```text
TX16S pin 1 PPM       -> SiW917 GPIO_6 / UART1 RX
TX16S pin 2 HEARTBEAT <- SiW917 GPIO_7 / UART1 TX
TX16S pin 4 GND       -> SiW917 GND
TX16S pin 3 VBAT      -> regulator only; not connected directly to SiW917
TX16S pin 5 S.Port    -> disconnected
```

## Confirmed Behavior

- EdgeTX detects the SiW917 TX module over direct two-wire UART.
- The ExpressLRS Lua interface loads and exchanges all TX parameter frames.
- The diagnostic run received more than 1000 valid handset frames with zero
  invalid frames and confirmed device ping, device info, reads, and writes.
- UART baud changes return with RX DMA armed.
- Handset TX uses a queued DMA path with lossless backpressure.
- The TX connects to an RX and exchanges RF telemetry.
- Temporary `[CRSF_UART]` and `[CRSF_LUA]` serial tracing is not compiled into
  this checkpoint firmware.
