# LR1121 ELRS Saleae Extension

High-level analyzer for LR1121 SPI traffic used by this SiW917 ExpressLRS RX port.

Load `extension.json` in Logic 2 via:

1. Extensions panel
2. Three-dot menu
3. Load Existing Extension...
4. Select this folder's `extension.json`

Add Saleae's built-in SPI analyzer first, then add `LR1121 ELRS SPI` as a high-level analyzer using the SPI analyzer as input.

Suggested capture channels:

- CS / NSS
- SCK
- MOSI
- MISO
- BUSY
- DIO1
- Firmware debug GPIO for hwTimer/FHSS events if available

