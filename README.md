# kk-link

ESP32-S3 firmware that bridges iOS packet/KISS apps over Bluetooth Low Energy
to a Kenwood TM-D750 mobile radio over wired USB.

The TM-D750 only accepts incoming Bluetooth connections while it is in Pairing
Mode, so a Bluetooth SPP adapter can never reconnect to it. Its main-unit USB
port is a standard CDC-ACM serial port, so kk-link talks to the radio over USB
and keeps BLE only on the app side.

Status: **milestone 1**, a USB host spike. The firmware finds the radio on USB,
sends the read-only CAT queries `ID` and `FQ 0` and logs the replies. It
reconnects when the radio's USB cable is unplugged and plugged back in. The BLE
side is not implemented yet.

## Hardware

- Espressif **ESP32-S3-DevKitC-1-N16R8** (16 MB flash, 8 MB octal PSRAM).
- **USB-OTG solder jumper bridged** on the DevKitC, so the board's 5 V feeds
  VBUS on the native USB port (needed for USB host mode).
- DevKitC **"UART" USB-C port → computer**: power, flashing and console logs.
- DevKitC **"USB" (native) USB-C port → radio's main-unit USB port**.
- **Kenwood TM-D750**: set menu **984 (KISS) = USB (Main Unit)** to get KISS
  over USB (needed from milestone 2 on; CAT works over USB regardless).

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
idf.py -p /dev/cu.usbmodemXXXX flash monitor   # the DevKitC "UART" port
```

`export.sh` must run with the same `python3` that `install.sh` used. On a Mac
whose default `python3` is newer than ESP-IDF supports, install with and export
under Homebrew's Python 3.12 by putting
`/opt/homebrew/opt/python@3.12/libexec/bin` first on `PATH`.

The `espressif/usb_host_cdc_acm` component is fetched by the IDF component
manager on the first build.

Expected console output once the radio is plugged into the native USB port:

```
I (...) kk_link: radio answering after 1 DTR pulse(s)
I (...) kk_link: ID -> ID TM-D750
I (...) kk_link: FQ 0 -> FQ 0,0145470000
I (...) kk_link: band 0 frequency 145.470000 MHz
```

## References

- TH-D74/D75 CAT command set (works on the D750 for ID/FQ/MD/AS/TN/BT):
  https://github.com/LA3QMA/TH-D74-Kenwood
- BLE-KISS API (planned app-side interface):
  https://github.com/hessu/aprs-specs/blob/master/BLE-KISS-API.md
