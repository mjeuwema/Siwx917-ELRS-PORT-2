# SiW917 LR1121 Port Status

This is the current release-candidate checklist for the SiW917 dual-LR1121
ExpressLRS RX port.

## Proven RF Modes

| Area | Status | Notes |
| --- | --- | --- |
| FCC915 normal | Passing | Standard single-band LR1121 rates tested by serial logs and connection holds. |
| FCC915 same-band Gemini | Passing | Dual LR1121 shared-band operation holds after timing/hot-path cleanup. |
| LR1121 crossband | Passing | Crossband 100 Hz and 150 Hz hold with radio1 on FCC915 and radio2 on ISM2G4. |
| Rate switching | Passing for tested modes | Tested across normal, Gemini, and crossband bring-up flow. Continue soak testing when changing timing code. |
| Lua/device telemetry | Passing in basic discovery | TX Lua parameter/device discovery works; re-test after any telemetry scheduler change. |
| FC CRSF serial | Build-ready, needs bench test | CRSF RC/link-stats output is enabled by default, and FC telemetry frames are parsed from USART0 RX and forwarded over OTA. |

## Intentional Scope

- This port targets LR1121 hardware, not SX128x/FLRC parity.
- FCC915 builds intentionally skip 2.4 GHz-only modes while still allowing
  LR1121 crossband rates.
- 2.4 GHz-only domain support is present in the rate table but should be tested
  separately before claiming production parity.

## Port Guardrails

- Keep `SIW917_ELRS_TIMING_LEAN` enabled for high packet-rate operation.
- Use `SIW917_ELRS_DISABLE_CRSF_SERIAL=1` only for RF-only timing tests. The
  upstream-parity default keeps FC CRSF serial enabled.
- Keep noisy diagnostics off by default; use the flags in
  `src/siw917_elrs_timing.h` for targeted A/B testing.
- Keep `SIW917_ELRS_DIRECT_GPIO_DIO_WHEN_LINKED` disabled unless explicitly
  testing that path. Prior flight testing showed immediate telemetry loss.
- Keep crossband radio routing pinned: radio1 uses the primary FCC915 FHSS
  sequence, radio2 uses the ISM2G4 FHSS sequence. Same-band Gemini can still
  alternate physical radios.

## Before Shipping A Build

1. Build `cmake_gcc/build/base/wifi_gspi_merged.rps`.
2. Boot and verify both LR1121 chips report ready.
3. Soak FCC915 normal, FCC915 Gemini, crossband 100 Hz, and crossband 150 Hz.
4. Open TX Lua and confirm the RX device appears and parameters load.
5. Connect an FC on USART0 and confirm RC channels, link stats, and at least one
   FC telemetry sensor frame reach the TX.
6. Watch serial for `LostConnection`, image-calibration warnings, or repeated
   serial protocol transitions.
