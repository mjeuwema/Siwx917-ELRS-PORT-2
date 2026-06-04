# ELRS BLE Companion

Native Android test/utility app for the SiWx917 ExpressLRS BLE config API.

## What It Does

- Scans for the receiver advertising as `ELRS-RX-BLE`.
- Connects to BLE service `0xE7E0`.
- Writes commands to characteristic `0xE7E2`.
- Reads responses from characteristic `0xE7E1`.
- Uses the firmware paging protocol: write command, write `len`, read `len=N`, then read `page 0..N`.
- Loads editable config fields with short `jget <field>` commands instead of long metadata reads.
- Applies changes with `set key=value`, then can `save`, `reload`, or `reset`.

## Build

This project intentionally avoids Compose, AndroidX, and external app dependencies so it can build as a small native Android app.

Install the Android SDK first:

1. Open Android Studio.
2. Go to `Settings > Languages & Frameworks > Android SDK`.
3. Install Android SDK Platform 34 and Android SDK Build-Tools 34.x.
4. Copy `local.properties.example` to `local.properties` if Gradle does not auto-detect the SDK.

Then build:

```powershell
.\gradlew.bat :app:assembleDebug
```

The debug APK will be created at:

```text
app\build\outputs\apk\debug\app-debug.apk
```

## Expected Firmware

The receiver firmware should expose:

- Device name: `ELRS-RX-BLE`
- Service UUID: `0000E7E0-0000-1000-8000-00805F9B34FB`
- Response characteristic: `0000E7E1-0000-1000-8000-00805F9B34FB`
- Command characteristic: `0000E7E2-0000-1000-8000-00805F9B34FB`

The app assumes all meaningful command responses are remembered by firmware and can be read back through `len` and `page`.
