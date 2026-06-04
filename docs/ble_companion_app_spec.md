# ELRS BLE Companion App Spec

This document describes the app we need for the SiWx917 ExpressLRS RX BLE
configuration API. BLE transport is proven working; the app work is now about
making a reliable command client and a safe configuration UI.

## Scope

The app is a BLE configuration companion for the receiver.

It should:

- Connect to the RX over BLE without joining the RX WiFi AP.
- Read receiver configuration.
- Edit supported config fields.
- Track unsaved changes.
- Save changes to NVM3 when the user confirms.
- Discard unsaved changes by reloading from NVM3.
- Expose a raw command/debug panel for development.

It should not:

- Flash MCU firmware.
- Upload LR1121 firmware.
- Host or mirror the Web UI.
- Replace the WiFi OTA path.
- Modify RF timing while the RX is actively connected unless we explicitly add a safe gate later.

## BLE Target

Device name:

```text
ELRS-RX-BLE
```

16-bit UUIDs:

```text
Service:  0xE7E0
Response: 0xE7E1
Command:  0xE7E2
```

128-bit UUID form:

```text
0000E7E0-0000-1000-8000-00805F9B34FB
0000E7E1-0000-1000-8000-00805F9B34FB
0000E7E2-0000-1000-8000-00805F9B34FB
```

Known handles from current firmware are useful for scanner diagnostics, but the
app must discover by UUID instead of hard-coding handles:

```text
Response value handle: 0x000C
Response CCCD handle:  0x000D
Command value handle:  0x000F
Command CCCD handle:   0x0010
```

## Core Protocol

Commands are UTF-8 strings written to `0xE7E2`.

Responses are UTF-8 strings exposed through `0xE7E1`. The firmware also mirrors
the latest response into `0xE7E2` for compatibility with simple BLE tools.

The BLE read path often exposes only 20 bytes at a time. Therefore the app must
treat every meaningful command response as potentially chunked.

The firmware currently remembers up to 320 bytes of response payload. Always
use `len` and `page <n>` instead of assuming a response fits in one BLE read.

Notifications are optional and should be treated as a response-ready hint or
first-chunk preview only. The app must not depend on notifications to stream a
complete long response.

Robust transaction flow:

1. Write command to `0xE7E2`.
2. Wait briefly, such as 80-150 ms.
3. Write `len` to `0xE7E2`.
4. Read the response and parse `len=N`.
5. Write `page 0`, read 20 bytes.
6. Write `page 1`, read 20 bytes.
7. Continue until collected bytes are at least `N`.
8. Trim the assembled string to `N` bytes/chars.
9. Parse the assembled string.

Important detail:

`len` and `page <n>` are ephemeral responses. They do not replace the remembered
payload from the previous meaningful command. This is why `jget`, then `len`,
then `page 0..n` works.

For manual scanner use, short commands can be read directly. For the app, always
use the robust transaction flow for consistency.

## Startup Flow

On app launch:

1. Request BLE permissions.
2. Scan for devices named `ELRS-RX-BLE` or advertising service `0xE7E0`.
3. Show discovered receivers.
4. Connect to selected receiver.
5. Discover services.
6. Verify service `0xE7E0`.
7. Verify command characteristic `0xE7E2`.
8. Verify response characteristic `0xE7E1`.
9. Optionally enable notifications on `0xE7E1`, but do not depend on them.
10. Run only the minimal editor handshake first:

```text
api
jstatus
jget
```

Do not block the edit screen on `meta`, `caps`, `keys`, `setkeys`, or `ranges`.
Those responses are useful for a protocol/debug screen, but they are longer than
the config-ready checks and make fragile Android BLE clients more likely to time
out or disconnect before the editor has any data.

After the editor is populated, the app may fetch optional protocol metadata in
the background or on the debug screen:

```text
meta
caps
keys
setkeys
ranges
```

If full `jget` paging is unreliable on a given phone/app stack, load the editor
from single-field commands instead. Each field response is short and exercises
the same command queue without requiring many pages:

```text
jget serial
jget failsafe
jget rate
jget model
jget tlm_off
jget tlm_interval
jget power
jget domL
jget domH
jget web_domain
jget wifi_interval
jget uart_baud
jget wifi_channel
jget wifi_ssid
jget wifi_custom
jget lock_on_first
jget is_airport
jget dji_armed
jget mav_tgt
jget mav_src
jget team_ch
jget team_pos
jget bind
jget vbind
```

The app should remain usable if notifications fail, as long as write/read
transactions work.

## Command Queue

The app must use a serialized command queue.

Rules:

- Never send multiple commands concurrently.
- Do not send `len` until the previous command write is acknowledged or enough delay has elapsed.
- Do not send `page n` until `len` was parsed.
- Timeout each BLE operation.
- Fail the current command cleanly if disconnected.
- After reconnect, rediscover services and rerun handshake.

Recommended timing:

```text
Write timeout: 2 seconds
Read timeout:  2 seconds
Post-write delay before len: 80-150 ms
Retries per read: 1-2
```

## Existing Commands

Human/debug commands:

```text
help
api
caps
meta
diag
keys
setkeys
ranges
schema
ping
status
dirty
get
get <key>
set key=value
save
commit
reload
discard
reset
len
page <n>
chunk <offset>
```

App-preferred JSON commands:

```text
jstatus
jget
jget <key>
```

## App-Preferred Commands

Use `jstatus` for state:

```json
{"ok":true,"cfg":1,"dirty":0,"ntf":0,"ntfS":0,"ntfC":0,"len":142,"seq":8,"wr":6,"rd":2}
```

Use `jget` for compact config:

```json
{"ok":true,"v":17,"flags":3,"uid":"BE:93:67:27:D6:9C","serial":0,"failsafe":0,"rate":8,"model":255,"tlmOff":0,"tlmInt":0,"power":"match","domL":6,"domH":6,"webDomain":1,"wifiCh":6,"wifiInt":60,"baud":420000,"wifiCustom":0,"lockFirst":1,"airport":0,"djiArmed":0,"webCustom":0,"dirty":0}
```

Use `jget <key>` for single fields and long strings:

```json
{"ok":true,"key":"serial","value":0}
{"ok":true,"key":"wifi_ssid","value":""}
{"ok":true,"key":"power","value":"match"}
```

JSON error shape:

```json
{"ok":false,"err":"key","msg":"unknown key"}
```

## Config Fields

Readable fields:

```text
version
flags
uid
serial
failsafe
rate
model
tlm_off
tlm_interval
power
domL
domH
web_domain
wifi_interval
uart_baud
wifi_channel
wifi_ssid
wifi_custom
web_custom
lock_on_first
is_airport
dji_armed
mav_tgt
mav_src
team_ch
team_pos
bind
vbind
dirty
```

Writable fields:

```text
serial
failsafe
rate
model
tlm_off
tlm_interval
power
domL
domH
web_domain
wifi_interval
uart_baud
wifi_channel
wifi_ssid
wifi_password
wifi_custom
lock_on_first
is_airport
dji_armed
mav_tgt
mav_src
team_ch
team_pos
bind
vbind
```

Value ranges:

```text
serial: 0,2,4,7
failsafe: 0..2
rate: 0..31
model: 0..63 or 255
tlm_off: 0..1
tlm_interval: 0..255
power: match,10,14,17,20
domL: 0..7
domH: 0..7
web_domain: 0..5
wifi_interval: -1..86400
uart_baud: 9600..2000000
wifi_channel: 1..13
mav_tgt: 1..255
mav_src: 1..255
team_ch: 0..10
team_pos: 0..7
bind: 0..3
vbind: 0..255
wifi_custom,lock_on_first,is_airport,dji_armed: 0..1
```

Serial values:

```text
0 = CRSF
2 = SBUS
4 = SUMD
7 = MAVLink
```

Model match:

```text
255 = disabled
0..63 = enabled model ID
```

## Editing Model

The app should use a simple staged-save UX.

When user changes a field:

1. Send `set key=value`.
2. If response starts with `ok`, mark app state as dirty.
3. Refresh `jstatus`.
4. Do not assume the change survives reboot until `save` succeeds.

Save button:

```text
save
jstatus
jget
```

Discard button:

```text
reload
jstatus
jget
```

Factory reset button:

```text
reset
jstatus
jget
```

Factory reset must require confirmation.

## UI Screens

Recommended screens:

- Scan/connect screen.
- Receiver dashboard.
- Config editor.
- Advanced/raw command screen.
- BLE debug log screen.
- About/protocol screen.

Dashboard should show:

- Connection state.
- Device name.
- API version.
- Config ready.
- Dirty state.
- Notifications enabled or read-only mode.
- Last command.
- Last response.

Config editor should show:

- Serial protocol dropdown.
- Failsafe dropdown.
- Model match toggle and model ID input.
- Force telemetry off toggle.
- Telemetry interval input.
- RX telemetry power dropdown.
- Low/high domain selectors.
- Web domain selector for the upstream WebUI-style domain field.
- WiFi-on interval field, including `-1` for disabled.
- UART baud field.
- WiFi custom toggle, SSID, password, channel.
- Lock on first connection toggle.
- Airport mode toggle.
- DJI permanently armed toggle.
- MAVLink target/source IDs.
- Team race channel/position.
- Bind storage selector.
- Voltage bind input.

Advanced/raw command screen should:

- Let the user type any command.
- Show assembled full response.
- Show raw first read if useful.
- Show `diag` output.
- Copy response to clipboard.

## Safety Rules

- Show a persistent unsaved-changes banner when dirty.
- Warn before model match is enabled.
- Warn before serial protocol changes.
- Confirm before `reset`.
- Prefer batching user edits visually, but send `set` immediately so firmware dirty state matches app dirty state.
- Do not automatically call `save` without an explicit user action.
- On disconnect with dirty state, warn that unsaved changes may exist on the receiver.

## Android Implementation Notes

Use native Android BLE APIs or a proven BLE library, but the app must serialize
operations. Android BLE is fragile if writes, reads, and service discovery are
overlapped.

Required Android 12+ permissions:

```text
BLUETOOTH_SCAN
BLUETOOTH_CONNECT
```

Older Android may require location permission for scanning:

```text
ACCESS_FINE_LOCATION
```

State machine:

```text
Idle -> Scanning -> Connecting -> Discovering -> Ready -> Busy
Ready -> Disconnected on GATT disconnect/error
Busy -> Ready after command completes
Busy -> Error after timeout
```

The app should close the GATT object on disconnect and create a new connection
object on reconnect.

## Firmware Diagnostics For App Debugging

Use `diag` when app behavior is confusing.

Example:

```text
diag seq=6 conn=0 disc=0 wr=6 rd=0 ntf=0 nerr=0 lw=0x000F lr=0x0000 sN=0 cN=0 last=diag
```

Meaning:

- `wr`: command writes seen by firmware.
- `rd`: read callbacks seen by firmware.
- `ntf`: notifications sent by firmware.
- `lw`: last write handle.
- `lr`: last read handle.
- `sN`: response/status characteristic notification enabled.
- `cN`: command characteristic notification enabled.
- `last`: last command text.

Note: the NWP may serve local attribute reads without an M4 read callback, so
`rd=0` does not automatically mean reads are failing. If the app sees responses,
the read path is fine.

## Minimum Viable App

The first useful app version only needs:

- Scan/connect.
- Handshake with `api`, `meta`, `jstatus`, `jget`.
- Display config.
- Raw command screen.
- Edit serial, failsafe, model, telemetry off, telemetry interval, power.
- Edit WebUI-style advanced fields: web domain, WiFi interval, UART baud, lock on first, airport, DJI armed.
- Save and reload buttons.
- Debug log.

After that, add the rest of the fields and nicer UI.
