# SiW917 ExpressLRS TX Laptop Setup

This repository is the SiW917 TX port. ExpressLRS itself is kept as a separate,
pristine repository so platform work does not modify upstream ELRS core code.

## Required versions

- Simplicity SDK Suite 2025.6.2
- WiseConnect 3 SDK 3.5.2
- Arm GNU Toolchain 12.2.Rel1 (GCC 12.2.1)
- Ninja 1.12.1 or compatible
- CMake 3.25 or newer
- Simplicity Commander 1.20.5 or compatible
- Git and PowerShell 7 or Windows PowerShell 5.1

Install Simplicity Studio and use Package Manager to install the two SDKs,
the Arm GNU toolchain, and Simplicity Commander. The build script discovers
their per-user Conan paths automatically.

## Restore the repositories

Use the complete handoff package's restore script:

```powershell
.\restore_workspaces.ps1
```

The included ExpressLRS source snapshot is pinned to official commit
`a9d4a9cb5b5687c4c9d7e9e7fbdf44ad93651da6`.

## Build the radio-connected TX firmware

```powershell
cd .\workspace\wifi_gspi_tx_clean
git switch codex/tx-clean-hal-port
.\scripts\build_tx_clean_port.ps1
```

The script builds the dedicated configuration:

- TX role enabled
- pristine upstream `tx_main.cpp` enabled
- direct two-wire handset UART enabled
- PC bench mode disabled
- normal, non-inverted UART on SiW GPIO6/GPIO7

Output:

`cmake_gcc\build-tx-clean-port\base\wifi_gspi_merged.rps`

## Flash

```powershell
commander flash .\cmake_gcc\build-tx-clean-port\base\wifi_gspi_merged.rps
```

The handoff also includes the proven checkpoint and the latest clean-port test
image under `firmware`. Do not overwrite or delete those reference images.

## Regeneration warning

The checked-in generated CMake files contain the TX port integration. Do not
Force Generate the project as a routine setup step because regeneration can
replace `cmake_gcc\CMakeLists.txt`. If regeneration is required after changing
SDK versions, preserve that file and reapply the TX integration afterward.
