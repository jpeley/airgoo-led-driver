# Airgoo LED Driver

## What this is

Python driver for the Airgoo AG-DRGB16 16-channel addressable RGB LED controller. Communicates via USB HID (`/dev/hidraw`) using the DRGB V4 protocol. No external dependencies.

## Architecture

Single file driver (`airgoo.py`) with:
- `AirgooController` class — main interface
- Direct `/dev/hidraw` I/O (no hidapi/libusb dependency)
- Background keepalive thread to maintain direct mode
- Multi-packet support for >316 LEDs total

## Key protocol details

- **VID/PID**: `0x2486:0x3216`
- **All 16 zones must be present in every update packet** — the controller silently ignores partial updates. This is the main gotcha.
- **Keepalive required**: Send `0x65` byte every ~500ms or controller reverts to built-in pattern.
- **Packet format**: 1025-byte HID reports. Byte 0 = report ID (0x00), bytes 1-4 = header (packet num, total packets, LED count high/low), bytes 5-1024 = data.
- **Data layout**: 72-byte zone header (2 bytes per zone = LED count) followed by concatenated RGB triplets.

## Hardware setup

- Controller powered via SATA, data via USB
- USB cable MUST be data-capable (charge-only cables are invisible to the OS)
- WS2812B strips: standard 3-pin ARGB (5V, Data, GND). Check strip direction arrows.
- If `/dev/hidraw` disappears, USB rebind fixes it: `echo 'BUS-PORT' > /sys/bus/usb/drivers/usb/unbind && sleep 1 && echo 'BUS-PORT' > /sys/bus/usb/drivers/usb/bind`

## Testing

Deploy to target host and run:
```bash
python3 airgoo.py --zone-name B8 --color FF0000 --leds 24
```

## Future plans

- Animation daemon for pixel art display (16 zones × N LEDs = grid display)
- Frame buffer abstraction for 2D pixel art on LED grid
