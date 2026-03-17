# airgoo-led-driver

Python driver for the [Airgoo AG-DRGB16](https://www.amazon.com/Airgoo-16-Port-Signal-Controller-Configuration/dp/B0F3VV65WD) 16-channel addressable RGB LED controller. Talks directly to the hardware via USB HID — no OpenRGB, SignalRGB, or Windows required.

## Why this exists

The AG-DRGB16 is marketed as OpenRGB-compatible, but as of March 2026 the driver ([OpenRGB #4878](https://gitlab.com/CalcProgrammer1/OpenRGB/-/issues/4878)) has issues with per-zone updates and the CLI path. This driver talks directly to `/dev/hidraw`, implements the DRGB V4 protocol correctly, and is designed for headless Linux servers — perfect for homelab LED projects.

## Hardware

| Detail | Value |
|--------|-------|
| Controller | Airgoo AG-DRGB16 |
| Connection | USB 2.0 HID (VID `0x2486`, PID `0x3216`) |
| Power | SATA (LED power), USB (data only) |
| Channels | 16 (A1-A8, B1-B8) |
| LEDs per channel | Up to 256 addressable (WS2812B, SK6812, etc.) |
| Protocol | DRGB V4 |

## Quick start

```bash
# Set zone B8 to 24 red LEDs
python3 airgoo.py --zone-name B8 --color FF0000 --leds 24

# Set all 16 zones to 24 blue LEDs
python3 airgoo.py --color 0000FF --leds 24

# List zone names
python3 airgoo.py --list-zones
```

## Python API

```python
from airgoo import AirgooController

ctrl = AirgooController()  # auto-detects /dev/hidraw device

# Set colors per zone (list of (R, G, B) tuples)
ctrl.set_zone(15, [(255, 0, 0)] * 24)       # zone 15 = B8
ctrl.set_zone_by_name("B8", [(0, 255, 0)] * 24)  # same, by name

# Set a single pixel within an already-sized zone
ctrl.fill_zone(0, 0, 0, 0, 24)  # zone A1, 24 black LEDs
ctrl.set_pixel(0, 5, 255, 0, 0) # turn LED 5 red

# Send to hardware
ctrl.update()

# Start background keepalive (required — controller reverts after ~500ms)
ctrl.start_keepalive()

# ... do animation stuff, call ctrl.update() each frame ...

# Cleanup
ctrl.stop()
```

### Context manager

```python
with AirgooController() as ctrl:
    ctrl.fill_all(255, 0, 0, 24)
    ctrl.update()
    ctrl.run_keepalive()  # blocks until Ctrl+C
```

## Zone map

| Index | Name | Index | Name |
|-------|------|-------|------|
| 0 | A1 | 8 | B1 |
| 1 | A2 | 9 | B2 |
| 2 | A3 | 10 | B3 |
| 3 | A4 | 11 | B4 |
| 4 | A5 | 12 | B5 |
| 5 | A6 | 13 | B6 |
| 6 | A7 | 14 | B7 |
| 7 | A8 | 15 | B8 |

## Setup

### Dependencies

None beyond Python 3.10+ standard library. No pip packages required.

### USB permissions (udev rule)

Without this, you need root to access `/dev/hidraw*`:

```bash
sudo tee /etc/udev/rules.d/60-airgoo.rules << 'EOF'
# Airgoo DRGB controllers
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2486", ATTRS{idProduct}=="3216", MODE="0666"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2486", ATTRS{idProduct}=="3204", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Verify hardware detection

```bash
lsusb | grep 2486
# Bus 003 Device 002: ID 2486:3216 Airgoo AG-DRGB16

ls /dev/hidraw*
# /dev/hidraw0
```

If `/dev/hidraw*` doesn't appear after plugging in, check:
1. **Cable**: Must be a data-capable USB cable (not charge-only). This is the #1 issue.
2. **Port**: If the USB port is passed through to a VM/container, the host won't see it.
3. **Rebind**: `echo '3-1' > /sys/bus/usb/drivers/usb/unbind && sleep 1 && echo '3-1' > /sys/bus/usb/drivers/usb/bind` (adjust bus-port to match your setup).

## Protocol notes

The DRGB V4 protocol sends all zone data in a single bulk update:

1. **Header** (72 bytes): 2 bytes per zone (high/low byte of LED count), up to 36 zones
2. **RGB data**: 3 bytes per LED (R, G, B), concatenated across all zones in order
3. **Packetized**: 1025-byte HID reports (1 byte report ID + 1024 payload). First packet carries 1020 bytes of data, subsequent packets carry 1020 bytes each.
4. **Keepalive**: A 65-byte packet with byte 1 = `0x65` must be sent every ~500ms or the controller reverts to its built-in pattern.

**Critical quirk**: The controller ignores updates where only some zones have data. Every update must include all 16 zones in the header, even if most have 0 LEDs. This is why OpenRGB's per-zone CLI path doesn't work with this controller.

## Tested hardware

- Airgoo AG-DRGB16 (firmware as of March 2026)
- BTF-LIGHTING WS2812B strips
- Debian 12 (Bookworm), kernel 6.12

## License

MIT
