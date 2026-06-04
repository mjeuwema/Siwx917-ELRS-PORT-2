# BLE Remote ID Test Beacon

This branch has a first-pass BLE Remote ID helper for bench testing.

It is intentionally not a compliance claim. Full Remote ID needs the complete
required message set for the operating region, including live aircraft location,
takeoff/operator/system data as applicable, update timing, and any required
authentication/registration data.

## What Is Implemented

- Builds an OpenDroneID-style Basic ID message.
- Builds an OpenDroneID-style Location message when a test coordinate is set.
- Wraps that 25-byte message in BLE legacy Service Data using UUID `0xFFFA`.
- Exposes control through the existing ELRS BLE config GATT command channel.
- Starts BLE advertising as Remote ID automatically.
- Rotates Basic ID and Location adverts once per second when a location is set.
- Keeps commands for switching back to normal `ELRS-RX-BLE` config advertising
  while testing.

The Basic ID defaults to:

```text
ELRS-<six byte UID as hex>
```

Example:

```text
ELRS-BE936727D69C
```

## Commands

Use the BLE companion app raw command box or any BLE scanner that can write to
the ELRS command characteristic.

```text
rid
rid status
rid help
rid id ELRS-TEST123
rid loc
rid loc 42.1234567 -71.1234567 25
rid loc clear
rid preview
rid preview loc
rid adv
rid adv 120
rid elrs
```

Command behavior:

- `rid`: show current Remote ID status.
- `rid id <value>`: set the in-RAM UAS ID, 1-20 chars, using `A-Z`, `a-z`,
  `0-9`, `-`, `_`, or `.`.
- `rid loc`: show the current test location state.
- `rid loc <lat> <lon> [alt_m]`: set the in-RAM test Location message.
  Latitude and longitude are decimal degrees. Altitude defaults to `0` meters.
  `0,0` is refused because OpenDroneID treats both coordinates as unknown and
  scanner apps may plot that near the Gulf of Guinea.
- `rid loc clear`: stop rotating Location messages.
- `rid preview`: show the Basic ID BLE advertisement payload as hex.
- `rid preview loc`: show the Location BLE advertisement payload as hex.
- `rid adv [seconds]`: queue/switch to temporary Basic ID advertising. This is
  mostly useful after using `rid elrs`. The default is 60 seconds. Allowed range
  is 5-600 seconds.
- `rid elrs`: restore normal `ELRS-RX-BLE` advertising.

On startup, the RX advertises Remote ID immediately and does not automatically
time out back to `ELRS-RX-BLE`. This gives scanner apps priority. The Remote ID
payload fills all 31 bytes of legacy BLE advertising data, so the normal device
name may not appear while Remote ID is active.

Until `rid loc <lat> <lon> [alt_m]` is set, the RX only advertises Basic ID. Once
a location is set, the RX alternates Basic ID and Location adverts once per
second. This is still a bench/test beacon; it does not yet pull live GPS.

If `rid adv` or `rid elrs` is sent while the app is connected, the advertisement
change is queued until the app disconnects, because a connected BLE peripheral
is not advertising in the normal way.

## Advertisement Payload

The BLE legacy advertisement payload is:

```text
Len   0x1E
Type  0x16  Service Data - 16-bit UUID
UUID  0xFFFA little-endian, bytes FA FF
App   0x0D  OpenDroneID application code
Count 0xXX  Message counter
Data  25-byte OpenDroneID Basic ID message
```

This is 31 bytes total and fills the full legacy advertising payload. It omits
BLE Flags so that the required OpenDroneID transport bytes and full 25-byte ODID
message fit.

`rid preview` should now start with:

```text
1E16FAFF0D
```

`rid preview loc` should also start with `1E16FAFF0D`, but the ODID message byte
after the transport counter will be `0x12`, meaning message type Location and
protocol version 2.

## Next Steps For Real Remote ID

- Feed live position from MAVLink/CRSF GPS telemetry.
- Add Location, System, Operator ID, and optional Self ID messages.
- Decide whether SiWx917 supports the required extended advertising/message-pack
  path for the target jurisdiction, or whether Wi-Fi Beacon/NAN is a better fit.
- Add a safety gate so Remote ID only runs when the aircraft is armed/in flight,
  if that matches the intended product behavior.
- Validate with OpenDroneID receiver apps and a real sniffer.
