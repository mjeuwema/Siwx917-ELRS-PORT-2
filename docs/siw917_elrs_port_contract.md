# SiW917 ExpressLRS Port Contract

This note captures the upstream behavior the SiW917/LR1121 port needs to
preserve. The goal is to make the platform layer emulate ExpressLRS timing and
radio semantics, instead of tuning ELRS core behavior around SiW917 artifacts.

## Upstream Sources Checked

- ExpressLRS RX core: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/src/rx_main.cpp
- ExpressLRS LR1121 driver: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/LR1121Driver/LR1121.cpp
- ExpressLRS LR1121 HAL contract: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/LR1121Driver/LR1121_hal.h
- ExpressLRS ESP32 timer behavior: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/HWTIMER/ESP32_hwTimer.cpp

## Contract Summary

| Area | Upstream expectation | SiW917 port requirement |
| --- | --- | --- |
| Packet timestamp / PFD | RX core calls the PFD from the packet callback using `micros()` taken after the radio ISR has read/decoded the packet, then adds `max(interval - 2 * TOA, PACKET_TO_TOCK_SLACK)`. | The SiW917 deferred ISR must make packet-callback timing look like ESP32 as closely as possible. DIO-edge timestamps are useful diagnostics, but they are earlier than the downstream HAL contract and should not drive PFD by default. |
| Timer resume | ESP32 schedules an immediate TOCK when the timer is resumed, but that interrupt is delivered after the current radio ISR/callback unwinds. | `hwTimer::resume()` should queue the synthetic TOCK and `hwTimer::service()` should run it after `LR1121Hal::handleDeferredISR()` returns. |
| Timer frequency offset | ESP32 RX applies `FreqOffset` every half interval in timer ticks. With `HWTIMER_TICKS_PER_US=5`, one `incFreqOffset()` step is 0.2 us, not 1 us. | Keep the C++/PFD unit as the upstream timer tick and translate fractional microseconds inside the SiW917 CT wrapper. |
| LR1121 packet read | Upstream `RXnbISR()` performs GET_PACKET in the radio callback and then invokes the packet callback. | SiW917 can implement special GSPI/C-driver handling for GET_PACKET in the HAL, but the returned packet semantics must match upstream. |
| DIO interrupt | Upstream assumes a tiny, low-latency DIO path into the radio IRQ callback. | SiW917 GPIO ISR should only capture edge time and defer work; the task must service DIO immediately without millisecond-scale delay. |
| FHSS retune | Upstream RX core calls `SetFrequencyReg(..., false)` for single-radio FHSS and relies on the radio/HAL behavior for continued RX. | If LR1121 on SiW917 needs fused retune+RX or explicit rearm, keep that adaptation in the LR1121 driver/HAL, not scattered through RX core policy. |
| Initial RF rate / scan cycle | Upstream starts scanning from the configured initial rate, gives it a slow first window, resets LQ when changing scan modes, and then cycles through supported rates while disconnected. | Use the persisted SiW917 `rate_index` when it is valid for the active band; fall back only when unsupported, and preserve upstream scan cadence/reset semantics. |
| Loss/reacquire cooldown | Upstream resets sync/scan timers after a bad tentative lock or requested RF-rate change so the RX does not immediately cycle away again. | After `LostConnection(true)` from those paths, refresh `LastSyncPacket` and `RFmodeLastCycled` before resuming scan/RX. |
| OTA serializer side effects | Upstream updates OTA serializers and stubborn telemetry package limits together whenever the RF link rate changes. | Keep `OtaUpdateSerializers()` paired with `DataUlReceiver` and downlink `StubbornSender` package-limit updates so 4-byte and 8-byte OTA modes stay coherent. |
| `micros()` | Upstream expects a monotonic microsecond clock suitable for PFD math. | A DWT cycle-counter-backed `micros()` is the right SiW917 equivalent; RTOS tick timing is not enough for packet phase work. |

## Current Deviations To Retire Or Contain

- `PACKET_TO_TOCK_SLACK` remains the upstream 200 us floor, but RX PFD scheduling
  must still use the downstream expression `max(interval - 2 * TOA,
  PACKET_TO_TOCK_SLACK)`. Do not replace that with a SiW917-only constant or a
  DIO-edge timestamp.
- RX core diagnostics are useful while bringing up the port, but they should be
  guarded as port diagnostics and not become protocol behavior.
- Timer-frequency correction is now adapted at the SiW917 CT boundary. Keep
  `incFreqOffset()`/`decFreqOffset()` in upstream units; do not convert those
  calls to whole microseconds in ELRS core code.
- `HandleFHSS()` is restored to the upstream single-radio call shape:
  `SetFrequencyReg(..., false)`. The SiW917 fused retune+RX workaround is now
  contained behind the LR1121 SiW917 HAL boundary.
- Startup RF-rate selection now honors the persisted SiW917 `rate_index` when
  that rate is valid for the active band. Disconnected scan cycling now follows
  upstream cadence more closely: slow first window, current scan slot applied
  before advancing, unsupported next slots skipped, and LQ reset between scan
  modes.
- `SetRFLinkRate()` now keeps OTA serializer changes paired with the upstream
  stubborn telemetry package-limit updates, including full-resolution packet
  mode.
- Bad tentative-lock and RF-rate-change reconnect paths now reset the scan
  cooldown timers like upstream, instead of immediately giving the just-selected
  rate almost no reacquire window.
- `LR1121Driver::RXnbISR()` currently explicitly rearms RX after packet
  processing. Upstream does not do this; keep it only if measurements show the
  LR1121/SiW917 HAL loses continuous RX without it.
- `LR1121Driver::RXnb()` and `SetFrequencyReg()` currently force
  `STDBY_XOSC` before RX/frequency commands. Upstream does not do this; either
  prove it is required by LR1121/SiW917 sequencing or move it behind a narrower
  SiW917 HAL adaptation.

## Next Refactor Order

1. Re-test the restored FHSS call shape and configured-rate startup path with
   serial logs and, if needed, a short Saleae DIO/SPI capture.
2. Move any remaining required retune+RX/rearm behavior into the SiW917 LR1121 layer and
   restore RX core call sites toward upstream shape.
3. Keep only diagnostics that answer a specific port question, and compile-gate
   them behind a single port-debug define.
