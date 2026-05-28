# Gemini / Diversity / Crossband Bring-Up Plan

This RX already has a validated single-LR1121 timing path on SiW917. For Gemini
and crossband work, preserve that path first: use one shared GSPI bus and add
separate chip-select, BUSY, IRQ, and optionally reset lines for the second radio.

## Why One Shared GSPI Bus First

- The current ELRS hot path is tuned around the SiW917 GSPI peripheral at 16 MHz.
- The generated SiW917 config exposes one GSPI master with multiple chip-select
  options; SSI/ULP_SSI also exist, but they would need a separate driver path.
- ELRS dual-radio transactions are selected by radio mask, not truly concurrent
  SPI transfers, so sharing SCK/MOSI/MISO is the lowest-risk hardware topology.
- A second SPI peripheral would add pin mux, IRQ/DMA, and timing risk before we
  even prove that two LR1121s can be initialized and serviced.

## Prototype Wiring

| Signal | Radio 1 | Radio 2 | Requirement |
|--------|---------|---------|-------------|
| SCK | GPIO_25 | GPIO_25 | Shared GSPI clock |
| MISO | GPIO_26 | GPIO_26 | Shared, both radios must release MISO when not selected |
| MOSI | GPIO_27 | GPIO_27 | Shared GSPI MOSI |
| NSS | GPIO_28 | GPIO_50 | Unique active-low chip-select |
| BUSY | GPIO_29 | GPIO_51 | Unique busy input |
| DIO9 | GPIO_46 | GPIO_47 | Unique rising-edge IRQ |
| NRESET | GPIO_30 | GPIO_49 | Separate reset for debugging |

Avoid the known occupied pins:

- GPIO_10: status LED0
- GPIO_11: bind button BTN1
- GPIO_25/26/27/28/29/30: radio 1 bus/control
- GPIO_46: radio 1 DIO9 IRQ
- GPIO_52/54/55: USART0 serial output
- ULP/UULP GPIOs: avoid for radio IRQ until the HP GPIO path is proven

The recommended Radio 2 prototype map is:

- GPIO_50: NSS2
- GPIO_51: BUSY2
- GPIO_47: DIO9_2 / ELRS DIO1_2 IRQ
- GPIO_49: NRESET2

## Build Switches

The firmware keeps the validated single-radio RX path as the default.

- `SIW917_ELRS_DUAL_RADIO_PROBE=1`: initializes Radio 2 on the shared GSPI bus,
  selects its NSS/BUSY/RST lines, and prints its firmware version, but still
  advertises a single-radio receiver to ELRS.
- `SIW917_ELRS_UPSTREAM_DUAL_RADIO=1`: reserved for the next stage after Radio
  2 DIO9 interrupt handling is implemented. It is intentionally compile-guarded
  so a partial Gemini path cannot accidentally replace the proven RX link.

UG590 for BRD2708A lists GPIO_50 and GPIO_51 on the left-side breakout pads, and
GPIO_48, GPIO_47, GPIO_49, GPIO_46, and GPIO_15 on the right-side breakout pads.
The current RX already uses GPIO_46 for Radio 1 DIO9, so leave it alone. GPIO_48
or GPIO_15 can be used as fallback Radio 2 control pins if the recommended
layout is awkward.

## Code Milestones

1. Add compile-time pin definitions for `GPIO_PIN_NSS_2`, `GPIO_PIN_BUSY_2`,
   `GPIO_PIN_DIO1_2`, and optionally `GPIO_PIN_RST_2`.
2. Teach the SiW917 SPI wrapper to honor `SX12XX_Radio_1`,
   `SX12XX_Radio_2`, and `SX12XX_Radio_All` by asserting the matching NSS line.
3. Extend the low-level BUSY wait to read the requested radio's BUSY pin.
4. Add a dual-radio probe mode that prints firmware version for radio 1 and
   radio 2 independently before enabling Gemini receive logic.
5. Add a second DIO9 interrupt channel and route it to `LR1121Hal::dioISR_2`.
6. Enable upstream ELRS dual-radio behavior only after both radios initialize,
   clear IRQs, and survive packet acquisition without timing regressions.

## First Success Criteria

- Boot log shows both LR1121 chips return firmware `0x0104`.
- Radio 1 still connects exactly like the current checkpoint with radio 2 wired.
- Radio 2 can be selected and queried without disturbing radio 1.
- No packet-timeout regressions appear during a normal single-radio connection.
