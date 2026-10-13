# Airgoo LED Driver

## What this is

Python driver for the Airgoo AG-DRGB16 16-channel addressable RGB LED controller. Communicates via USB HID (`/dev/hidraw`) using the DRGB V4 protocol. No external dependencies.

## Architecture

Single file driver (`airgoo.py`) with:
- `AirgooController` class — main interface
- Direct `/dev/hidraw` I/O (no hidapi/libusb dependency)
- Optional background keepalive thread (not needed in practice)
- Multi-packet support for >316 LEDs total

## Key protocol details

- **VID/PID**: `0x2486:0x3216`
Source of truth: the vendor's OpenRGB driver files in `../Airgoo Source/` (from OpenRGB issue #4878). Both PIDs (0x3204 AG-DRGB04 = 4 channels, 0x3216 AG-DRGB16 = 16 channels) use protocol V4.

- **Keepalive is optional**: a 65-byte report with `0x65` in byte 1 every ~500ms. The vendor driver sends it ("exit hardware effects"), but it has not been needed in testing. Off by default; opt in with `start_keepalive()` / `run_keepalive()` or the CLI `--keepalive` flag.
- **Packet format**: 1025-byte HID reports. Byte 0 = report ID (0x00), byte 1 = `100 + packet_index`, byte 2 = `total_packets + 99`, bytes 3-4 = LED count carried in *this* packet (high, low), bytes 5-1024 = 1020 data bytes.
- **Data layout**: 72-byte zone header (36 zones max × 2 bytes, big-endian LED count) followed by concatenated RGB triplets in zone order. Colour order is R, G, B.
- **LED counts per packet**: first packet 316 (high=1, low=60; 72 header bytes + 948 LED bytes), later packets 340 (high=1, low=84; 1020 bytes). A final partial packet carries `remaining % 256` with high=1 if remaining >= 256. Vendor constants: `DRGB_V4_ONE_PACKAGE_SIZE=316`, `DRGB_V4_PACKAGE_SIZE=340`.
- **No zone padding needed.** Every update carries all 16 zone counts; unused zones are sent with count 0 (as the vendor driver does). Verified on hardware. An earlier version padded empty zones with a black LED based on a mistaken belief that the controller ignored 0-LED zones.
- Vendor stores per-zone count in an `unsigned char` (max 255); this driver allows 256 via the high byte, untested on hardware.
- Older Airgoo hardware uses V1-V3 (64-byte reports); not relevant to the 0x3204/0x3216 devices. See vendor `AirgooController_DRGB.cpp` if ever needed.

## Hardware setup

- Controller powered via SATA, data via USB
- USB cable MUST be data-capable (charge-only cables are invisible to the OS)
- WS2812B strips: standard 3-pin ARGB (5V, Data, GND). Check strip direction arrows.
- If `/dev/hidraw` disappears, USB rebind fixes it: `echo 'BUS-PORT' > /sys/bus/usb/drivers/usb/unbind && sleep 1 && echo 'BUS-PORT' > /sys/bus/usb/drivers/usb/bind`

## Device selection

- Auto-detect matches USB VID/PID from sysfs (`HID_ID=0003:00002486:00003216`), never guesses "first hidraw". No match → `FileNotFoundError`; several controllers → `MultipleDevicesError` listing them.
- Select explicitly with `--device /dev/hidrawN` or `--serial SERIAL` (library: `AirgooController(hidraw_path=..., serial=...)`). `python3 airgoo.py --list-devices` shows all hidraw devices, Airgoo ones marked `*`.
- `reconnect()` re-finds the same controller by serial, since the hidraw number can change after a USB rebind.

## Testing

Deploy to target host and run:
```bash
python3 airgoo.py --zone-name B8 --color FF0000 --leds 24
```

## Future plans

- Animation daemon for pixel art display (16 zones × N LEDs = grid display)
- Frame buffer abstraction for 2D pixel art on LED grid
