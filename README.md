# kk-link

ESP32-S3 firmware that bridges iOS packet/KISS apps over Bluetooth Low Energy
to a Kenwood TM-D750 mobile radio over wired USB.

The TM-D750 only accepts incoming Bluetooth connections while it is in Pairing
Mode, so a Bluetooth SPP adapter can never reconnect to it. Its main-unit USB
port is a standard CDC-ACM serial port, so kk-link talks to the radio over USB
and keeps BLE only on the app side.

kk-link was inspired by Island Magic's [B.B. Link](https://getbblink.com) project, which connects iOS
packet apps to Kenwood radios over Bluetooth. kk-link seeks to continue its
legacy with new hardware for newly released radios such as the TM-D750. It is
an independent implementation and shares no code with B.B. Link.

Status: **milestone 2**, a working BLE-KISS bridge. Apps that speak the
[BLE-KISS API](https://github.com/hessu/aprs-specs/blob/master/BLE-KISS-API.md)
find the adapter as **"K.K. Link"**, and KISS frames flow both ways between the
app and the radio's built-in TNC. Tested with RadioMail on iOS and from a Mac
BLE client.

## Download

Ready-to-flash firmware is on the
**[latest release page](https://github.com/jonnyborbs/kk-link/releases/latest)**,
with step-by-step instructions for flashing it from a web browser. No
development tools are needed.

## How it works

- **Idle:** the adapter advertises the BLE-KISS service and keeps the radio's
  USB serial port in CAT mode. It reads the radio's TNC mode (`TN`) whenever
  the radio connects.
- **App subscribes** to the RX characteristic: the adapter saves the current
  TNC mode in flash, switches the TNC to KISS on the same band
  (`TN 2,<band>`), and passes bytes through unchanged in both directions.
- **App unsubscribes or disconnects:** the adapter sends the KISS exit frame
  `C0 FF C0`, waits for CAT to answer again and restores the saved TNC mode.
- **Recovery:** KISS mode survives DTR drops and USB replugs. If the adapter
  resets or the cable drops mid-session, the next connection finds the radio
  silent on CAT, sends `C0 FF C0`, and restores the TNC mode saved in flash.

| Characteristic | UUID | Direction |
| --- | --- | --- |
| Service | `00000001-ba2a-46c9-ae49-01b0961f68bb` | |
| TX | `00000002-ba2a-46c9-ae49-01b0961f68bb` | app → radio, write / write without response |
| RX | `00000003-ba2a-46c9-ae49-01b0961f68bb` | radio → app, notify |

The adapter name "K.K. Link" is a placeholder. RadioMail enables frequency
control (QSY) only for adapters named "B.B. Link"; that name is not used
without coordinating with Island Magic.

### Experimental: frequency control (QSY)

When RadioMail believes it is talking to a B.B. Link adapter, it sends its
frequency changes in-band as KISS SetHardware (`0x06`) frames:

| Frame | Observed meaning |
| --- | --- |
| `C0 06 EA f3 f2 f1 f0 C0` | set the TNC band to frequency `f` (Hz, big-endian) |
| `C0 06 EB C0` | sent before disconnecting; treated as "restore frequency" |
| `C0 06 F4 00 C0` | unknown; logged and ignored |

These come from observing RadioMail, not from a published protocol. The
D750's own TNC ignores them, so with `CONFIG_KK_LINK_QSY_EXPERIMENT` enabled
(menuconfig → kk-link; off by default) the adapter translates them:
- **On a QSY:** it leaves KISS, switches the TNC band to VFO mode
  (`VM b,0`), sets the frequency (`FQ b,<Hz>`) and re-enters KISS (about 1 s;
  app frames wait meanwhile).
- **On restore or at the end of the session:** it puts back the band's
  original VFO frequency and VFO/memory mode.
- **If the adapter resets mid-session:** the TNC mode is restored, but the frequency is not.

Because RadioMail only sends these to an adapter named "B.B. Link", testing
needs a local build with that name. Keep it in a gitignored
`sdkconfig.defaults.local`:

```
CONFIG_KK_LINK_BLE_NAME="B.B. Link"
CONFIG_KK_LINK_QSY_EXPERIMENT=y
CONFIG_KK_LINK_LOG_APP_WRITES=y
```

and build it into its own directory:

```sh
idf.py -B build-bblink -D SDKCONFIG=build-bblink/sdkconfig \
  -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.defaults.local" \
  -p /dev/cu.usbmodemXXXX flash
```

## Hardware

- Espressif **ESP32-S3-DevKitC-1-N16R8** (16 MB flash, 8 MB octal PSRAM),
  available [on Amazon](https://link.amazon/B066pf0Qa).
- **USB-OTG solder jumper bridged** on the DevKitC, so the board's 5 V feeds
  VBUS on the native USB port (needed for USB host mode).
- DevKitC **"UART" USB-C port** (labeled **"COM"** on some board versions)
  **→ computer**: power, flashing and console logs.
- DevKitC **"USB" (native) USB-C port → radio's main-unit USB port**.
- **Kenwood TM-D750**: set menu **984 (KISS) = USB (Main Unit)** so the TNC's
  KISS port is the USB port (CAT works over USB with either setting). KISS runs
  on the band the radio's TNC is set to.

The radio enumerates as USB Full Speed VID `0x2166` / PID `0x9030` ("TM_D750"),
a composite device with a CDC-ACM serial port (interfaces 0 and 1) and USB
Audio (ignored).

### DTR handshake

The radio only reads and answers its USB serial port while the host holds
**DTR high**, and only after DTR **rises once the radio is ready**. Asserting
DTR right after enumeration is not enough: the radio silently ignores the port
(writes stall after two packets). kk-link therefore opens the port with DTR
low, waits briefly, then pulses DTR low → high and probes with `ID` until the
radio answers (normally on the first pulse). The radio confirms with a CDC
serial-state notification (DCD + DSR). RTS and line coding don't matter. This
works with menu 984 set to either Bluetooth or USB (Main Unit).

## Build and flash

Requires ESP-IDF 5.5 (5.3 or newer should work).

```sh
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor   # the DevKitC "UART"/"COM" port
```

`export.sh` must run with the same `python3` that `install.sh` used. On a Mac
whose default `python3` is newer than ESP-IDF supports, install with and export
under Homebrew's Python 3.12 by putting
`/opt/homebrew/opt/python@3.12/libexec/bin` first on `PATH`.

The `espressif/usb_host_cdc_acm` component is fetched by the IDF component
manager on the first build; NimBLE is part of ESP-IDF.

Expected console output for one app session:

```
I (...) ble_kiss: advertising as "K.K. Link"
I (...) bridge: radio answering (ID TM-D750) after 1 DTR pulse(s)
I (...) bridge: radio TNC mode: TN 0,1
I (...) ble_kiss: app subscribed: KISS session open
I (...) bridge: KISS on (band B); bridging app <-> radio
I (...) ble_kiss: app gone: KISS session closed
I (...) bridge: radio TNC mode restored: TN 0,1
I (...) bridge: KISS off
```

## References

- TH-D74/D75 CAT command set (works on the D750 for ID/FQ/MD/AS/TN/BT; `AI`
  is not supported; `BC` replies with two values):
  https://github.com/LA3QMA/TH-D74-Kenwood
- BLE-KISS API:
  https://github.com/hessu/aprs-specs/blob/master/BLE-KISS-API.md

## License

kk-link is licensed under the [Mozilla Public License 2.0](LICENSE)
(SPDX: `MPL-2.0`).
