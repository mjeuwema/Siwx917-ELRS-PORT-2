# SiWx917 ELRS BLE Config API

This BLE API is a small companion-app control plane for the SiWx917 ELRS RX. It
does not host the Web UI over BLE. Instead, an app connects to the RX and sends
short text commands over a custom GATT service.

## GATT Layout

- Device name: `ELRS-RX-BLE`
- Service UUID: `0xE7E0`
- Response/status characteristic UUID: `0xE7E1`
- Command characteristic UUID: `0xE7E2`
- Max response payload remembered by firmware: 320 bytes
- Safe app read page size: 20 bytes

Most scanner apps should write commands to `0xE7E2`, then read either `0xE7E1`
or `0xE7E2`. The firmware mirrors the latest response into both characteristic
values because some generic BLE apps make one easier to read than the other.

Both `0xE7E1` and `0xE7E2` support notifications for app compatibility. A
companion app can either:

- Write command to `0xE7E2`, then explicitly read `0xE7E1` or `0xE7E2`.
- Subscribe to notifications on `0xE7E1` and treat the notification as a response-ready hint.
- Subscribe to notifications on `0xE7E2` if the app architecture expects command/response on one characteristic.

Notifications intentionally carry only the first 20-byte chunk. Use `len` and
`page <n>` to assemble full responses.

The receiver serial log is the fastest way to diagnose app bugs. After a write,
a working app should cause at least one of these logs:

```text
[BLE] Read request handle=...
[BLE] Notifications enabled on handle=...
```

If writes appear in the receiver log but neither of those messages appears, the
app is sending commands but never asking the receiver for the response.

## Basic Flow

1. Connect to `ELRS-RX-BLE`.
2. Discover service `0xE7E0`.
3. Write a UTF-8 command to characteristic `0xE7E2`.
4. Read characteristic `0xE7E1` for the response, or receive notifications from `0xE7E1`.
5. For long responses, write `len`, then read pages with `page 0`, `page 1`,
   and so on.

Example:

```text
write: ping
read:  pong

write: meta
read:  {"api":"elrs-ble-v1","name":"ELRS-RX-BLE",...

write: diag
read:  diag seq=12 conn=1 disc=0 wr=4 rd=3 ntf=0 nerr=0 lw=0x000F lr=0x000C sN=0 cN=0 last=diag

write: jstatus
read:  {"ok":true,"cfg":1,"dirty":0,"ntf":0,"ntfS":0,"ntfC":0,"len":142,"seq":8,"wr":6,"rd":2}

write: jget
read:  {"ok":true,"v":17,"flags":3,"uid":"BE:93:67:27:D6:9C",...

write: jget serial
read:  {"ok":true,"key":"serial","value":0}

write: get
read:  {"v":17,"flags":3,...

write: len
read:  len=142

write: page 0
read:  {"v":17,"flags":3,"u

write: page 1
read:  id":"BE:93:67:27:D
```

## Commands

- `help`: short command summary.
- `api`: returns `api=elrs-ble-v1`.
- `caps`: chunkable JSON-ish capability summary.
- `meta`: chunkable JSON-ish UUID, handle, max-size, and page-size summary.
- `diag`: chunkable receiver-side BLE counters for debugging app behavior.
- `keys`: chunkable list of readable keys.
- `setkeys`: chunkable list of writable keys.
- `ranges` or `schema`: chunkable compact value limits for writable keys.
- `ping`: returns `pong`.
- `status`: returns config-ready, dirty, notifications, and remembered payload length.
- `jstatus`: chunkable JSON status with config, notification, sequence, read, and write counters.
- `jget`: chunkable JSON config snapshot with an `ok` field. The compact snapshot includes common Lua/WebUI-style fields but omits long strings; use `jget wifi_ssid` for SSID.
- `jget <key>`: chunkable JSON single-key response like `{"ok":true,"key":"serial","value":0}`.
- `dirty`: returns only the unsaved-change state.
- `get`: returns a compact JSON config snapshot.
- `get <key>`: returns one key.
- `set key=value`: updates RAM config only and marks it dirty.
- `save` or `commit`: writes RAM config to NVM3 and clears dirty.
- `reload` or `discard`: reloads config from NVM3 and clears unsaved BLE edits.
- `reset`: factory-resets config and saves defaults to NVM3.
- `len`: returns remembered payload length.
- `page <n>`: returns 20 bytes from remembered payload at `n * 20`.
- `chunk <offset>`: returns 20 bytes from remembered payload at byte offset.
- `rid`: Remote ID test beacon status.
- `rid id <value>`: set the in-RAM Remote ID Basic ID UAS ID, 1-20 safe ASCII chars.
- `rid preview`: return the BLE legacy advertisement payload as hex.
- `rid adv [seconds]`: temporarily switch to BLE legacy OpenDroneID Basic ID advertising.
- `rid elrs`: restore normal `ELRS-RX-BLE` advertising.

JSON command errors use this shape:

```json
{"ok":false,"err":"key","msg":"unknown key"}
```

## Writable Keys

Current writable keys:

```text
serial,failsafe,rate,model,tlm_off,tlm_interval,power,domL,domH,
web_domain,wifi_interval,uart_baud,wifi_channel,wifi_ssid,wifi_password,
wifi_custom,lock_on_first,is_airport,dji_armed,mav_tgt,mav_src,
team_ch,team_pos,bind,vbind
```

Readable-only keys also include:

```text
version,flags,uid,web_custom,dirty
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

Useful examples:

```text
set serial=0
set serial=2
set serial=4
set serial=7
set model=255
set failsafe=0
set tlm_interval=0
set power=match
set web_domain=1
set wifi_interval=60
set uart_baud=420000
set wifi_ssid=MyNetwork
set wifi_password=MyPassword
save
reload
```

`set` does not persist by itself. The app should call `save` after a batch of
changes, or `reload` to discard unsaved edits.
