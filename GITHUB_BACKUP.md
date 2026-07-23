# Canonical SiW917 ExpressLRS TX Backup

Date: 2026-07-23

Repository: `https://github.com/mjeuwema/Siwx917-ELRS-PORT-2.git`

Branch: `codex/tx-canonical-backup-2026-07-23`

This branch preserves the canonical SiW917 ExpressLRS transmitter workspace.
It includes the direct two-wire EdgeTX handset transport, upstream ELRS TX
integration, LR1121 support, Lua parameters, MAVLink WiFi bridge, and the
bridge-to-OTA WiFi handoff.

The RX and LR2021 projects are not part of this branch.

## Restore

```powershell
git clone --branch codex/tx-canonical-backup-2026-07-23 --recurse-submodules `
  https://github.com/mjeuwema/Siwx917-ELRS-PORT-2.git wifi_gspi_tx_clean
cd wifi_gspi_tx_clean

git -C upstream_expresslrs_master apply --check `
  ..\patches\siw917-tx-upstream-cfa88c0.patch
git -C upstream_expresslrs_master apply `
  ..\patches\siw917-tx-upstream-cfa88c0.patch
```

Pinned dependencies:

- ExpressLRS: `cfa88c0bb3e686e104a237d7239bdbbeb3ed4e9e`
- MAVLink C library: `e54a8d2e8cf7985e689ad1c8c8f37dc0800ea87b`
- SiW917 upstream patch SHA-256:
  `5013E2A10408A036B72B9295CAD4F0DC7C4E5FA0F14B455C6D6776AB0B0AD7E9`
- SiW917 upstream patch ID:
  `570e0b4d5d4fdde8a2194e017bed3bb3532570bd`

## Build

The Silicon Labs SDK and toolchain must be installed on the machine.

```powershell
cd cmake_gcc
cmake --workflow --preset tx-clean-port
```

Build output:

```text
cmake_gcc\build-tx-clean-port\base\wifi_gspi_merged.rps
```

## Verified Firmware

```text
firmware\SiW917_ELRS_TX_HTTP_OTA_TX_UI_2026-07-23_A6B85DFC.rps
SHA256: A6B85DFCA3BC2629A5F9294C0FF10A1E09F435B80B5AB13B1C07B6B3FAF1C5E2
Size: 378604 bytes
```
