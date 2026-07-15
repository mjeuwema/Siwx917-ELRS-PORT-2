# Working TX Checkpoint

Date: 2026-07-15

This checkpoint records the first confirmed working RadioMaster TX16S Mark 3
connection using the custom EdgeTX two-wire external-module transport.

## SiW917 firmware

- Role: ExpressLRS transmitter
- ELRS core: upstream `tx_main.cpp`
- Handset transport: direct two-wire UART1
- UART: normal polarity, 400000 baud, 8N1
- SiW917 RX: GPIO_6, BRD2605A breakout pin 21
- SiW917 TX: GPIO_7, BRD2605A breakout pin 24
- Adapter/TX_OE_N: not used
- PC bench mode: disabled
- Radio return diagnostic sweep: disabled

Build options:

```text
SIW917_ELRS_TARGET_TX=ON
SIW917_ELRS_CRSF_BENCH_2WIRE=ON
SIW917_ELRS_TX_PC_BENCH=OFF
SIW917_ELRS_USE_UPSTREAM_TX_MAIN=ON
SIW917_CRSF_RADIO_RETURN_DIAG=OFF
```

Current timing-stable artifact:

```text
C:\Users\mjeuw\OneDrive\Documents\ELRS_TX_AI_Handoff_Build\tx_module_2wire\base\SiW917_ELRS_TX_TwoWire_Stable_81B714C4.rps
SHA256: 81B714C45A98ED55770A7B376F106C66F65154F4A4E0DA8A0449C65765F1AD31
```

## Radio wiring

```text
TX16S pin 1 PPM       -> SiW917 GPIO_6 / UART1 RX
TX16S pin 2 HEARTBEAT <- SiW917 GPIO_7 / UART1 TX
TX16S pin 4 GND       -> SiW917 GND
TX16S pin 3 VBAT      -> regulator only; not connected directly to SiW917
TX16S pin 5 S.Port    -> disconnected
```

## Upstream source identity

The build uses the upstream source snapshot at:

```text
C:\Users\mjeuw\OneDrive\Documents\ELRS TX AI Handoff\ELRS_TX_workspace_refs\upstream_expresslrs_master\src
```

Key source hashes:

```text
src/tx_main.cpp
26AB4A305E2E99BC9B57C9B6D71179F09E961C2E0E2F782EC5E9E022F82399EC

lib/tx-crsf/TXModuleParameters.cpp
77D83A7C086A91AAC9894FB97CDCF053E35E4C29F3A75B67AF3518D1DB7F2DD4
```

## Confirmed behavior

- EdgeTX detects the SiW917 TX module over direct two-wire UART.
- The ExpressLRS Lua interface loads on the radio.
- Lua parameter loading is responsive after repeated module reboots.
- RF band selection persists when switching between FCC 915 MHz and 2.4 GHz.
- The TX connects to the RX and maintains telemetry at the tested packet rates.
- No inverter, line combiner, S.Port connection, or TX output-enable signal is
  required with the custom EdgeTX firmware.
