# Working TX Checkpoint

Date: 2026-07-23

Git tag: `checkpoint/tx-http-ota-2026-07-23`

This checkpoint records the confirmed working SiW917 ExpressLRS transmitter
state with the EdgeTX Lua request/reply path, MAVLink WiFi bridge, clean
bridge-to-OTA handoff, streamed HTTP web assets, and TX-specific Web UI
identity. High-volume CRSF protocol diagnostics are disabled in the checkpoint
firmware.

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
C:\Users\mjeuw\SimplicityStudio\TEST\wifi_gspi_tx_clean\cmake_gcc\build-tx-clean-port\base\wifi_gspi_merged.rps
```

Stable local copy:

```text
C:\Users\mjeuw\SimplicityStudio\TEST\wifi_gspi_tx_clean\firmware\SiW917_ELRS_TX_HTTP_OTA_TX_UI_2026-07-23_A6B85DFC.rps
```

```text
Size:   378604 bytes
SHA256: A6B85DFCA3BC2629A5F9294C0FF10A1E09F435B80B5AB13B1C07B6B3FAF1C5E2
```

## Upstream Source Identity

The build uses the local ExpressLRS checkout at:

```text
C:\Users\mjeuw\SimplicityStudio\TEST\wifi_gspi_tx_clean\upstream_expresslrs_master
```

Base commit:

```text
cfa88c0bb3e686e104a237d7239bdbbeb3ed4e9e
```

The exact four-file upstream working-tree delta is captured by:

```text
patches\siw917-tx-upstream-cfa88c0.patch
SHA256: 5013E2A10408A036B72B9295CAD4F0DC7C4E5FA0F14B455C6D6776AB0B0AD7E9
Patch ID: 570e0b4d5d4fdde8a2194e017bed3bb3532570bd
```

The patch was verified by applying it to a temporary clean worktree at the
base commit.

## Build

```powershell
cd "C:\Users\mjeuw\SimplicityStudio\TEST\wifi_gspi_tx_clean\cmake_gcc"
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
- The internal MAVLink WiFi bridge can release the AP cleanly before OTA mode.
- The Web UI loads its compressed HTML, JavaScript, and CSS assets using the
  SiWx917 HTTP service's streamed response API.
- Web UI information identifies the product, Lua device, and module type as TX.
- The SiWx917 RPS and LR1121 firmware-update endpoints are present.
- Temporary `[CRSF_UART]` and `[CRSF_LUA]` serial tracing is not compiled into
  this checkpoint firmware.
