# Working TX Checkpoint

Date: 2026-07-14

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

Known-good artifact:

```text
C:\Users\mjeuw\OneDrive\Documents\ELRS_TX_AI_Handoff_Build\tx_module_2wire\base\SiW917_ELRS_TX_TwoWire_400k.rps
SHA256: 8662FAE6E8C2CA50C8855591F166C0F66D38023E14E05C3AC7C165505ED77607
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
- No inverter, line combiner, S.Port connection, or TX output-enable signal is
  required with the custom EdgeTX firmware.
