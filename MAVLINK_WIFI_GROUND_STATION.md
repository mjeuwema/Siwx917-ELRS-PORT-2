# SiW917 MAVLink WiFi Ground-Station Bench Test

This build uses the SiW917 as an internal ExpressLRS MAVLink backpack. The
official upstream TX core still owns MAVLink RF uplink/downlink handling; the
SiW917 HAL only moves raw MAVLink bytes between upstream's backpack stream and
a UDP socket.

## Firmware

Flash the canonical build output:

```text
C:\Users\mjeuw\OneDrive\Documents\ELRS TX\cmake_gcc\build-tx-clean-port\base\wifi_gspi_merged.rps
```

## Start The Bridge

1. Power the TX module, radio, RX, and flight controller normally.
2. While the RF link is disconnected, open the ExpressLRS Lua interface and
   set `Link Mode` to `MAVLink`. This changes the RF transport but does not
   start WiFi.
3. Configure the RX serial protocol and flight-controller UART for MAVLink.
4. In the Lua `WiFi` folder, set `MAVLink WiFi` to `On`. The TX module then
   starts this access point:

```text
SSID:     ELRS_TEST_AP
Password: elrs1234
Address:  192.168.10.10
UDP port: 14550
```

5. Connect the ground-station computer to `ELRS_TEST_AP`.
6. Configure the ground station as a UDP client targeting
   `192.168.10.10:14550`. The module learns the return IP and port from the
   first UDP packet sent by the ground station, so the ground station must
   transmit at least one MAVLink packet before downlink forwarding begins.

## Expected Serial Markers

At boot:

```text
[MAVWIFI] Internal MAVLink backpack attached (UDP 14550)
```

When MAVLink mode is selected while the bridge is Off:

```text
[MAVWIFI] Link Mode MAVLink; WiFi bridge remains Off
```

After setting `WiFi > MAVLink WiFi` to `On`:

```text
[MAVWIFI] Ground-station bridge ready
[MAVWIFI] SSID=ELRS_TEST_AP password=elrs1234 IP=192.168.10.10 UDP=14550
```

After setting it back to `Off`:

```text
[MAVWIFI] Bridge Off; WiFi AP stopped, NWP retained
```

After the ground station sends its first packet:

```text
[MAVWIFI] GCS peer=<computer-ip>:<source-port>
```

Queue-drop or socket-failure counters are only printed when nonzero and are
rate-limited to once every ten seconds.

## Architecture And Scope

- The ELRS task performs no WiFi or socket calls.
- RF downlink data is copied into fixed-size queue slots.
- UDP uplink data is copied by the asynchronous socket callback into a ring.
- A dedicated `osPriorityLow` task owns AP startup and UDP sends.
- `Link Mode` selects upstream MAVLink RF transport; the Lua `MAVLink WiFi`
  Off/On selection controls the internal bridge separately.
- `Off` immediately blocks bridge traffic, clears the learned UDP peer and both
  queues, closes the UDP socket, and then stops AP broadcasting. The NWP stays
  initialized so a later `On` transition does not reboot the network processor.
- A WiFi client disconnect only clears stale peer state; it does not stop the
  AP. Only Lua `Off` or leaving MAVLink mode performs the ordered AP shutdown.
- Leaving MAVLink mode turns the bridge Off. Rebooting leaves it Off until it
  is enabled from Lua again.
- Disconnecting the last WiFi station clears the learned UDP peer and queued
  data without tearing down the NWP during a live RF session. Reconnect to the
  existing AP and send a packet to establish the peer again.
- OTA/HTTP integration is intentionally unchanged in this phase. Do not enter
  OTA configuration mode while the MAVLink bridge is active; coordinated
  handoff between these two WiFi users is the next WiFi/OTA task.
