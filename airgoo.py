"""
Airgoo DRGB16 LED Controller - Python Driver

Direct HID control for Airgoo AG-DRGB16 (and related DRGB V4 controllers)
without requiring OpenRGB. Talks to the controller via /dev/hidraw.

Protocol: DRGB V4 over USB HID
  - VID: 0x2486, PID: 0x3216 (AG-DRGB16)
  - 16 channels (A1-A8, B1-B8), up to 256 addressable LEDs per channel
  - 1025-byte HID reports with multi-packet support for large payloads
  - Requires keepalive packets every ~500ms to stay in direct mode

Typical usage:
    from airgoo import AirgooController

    ctrl = AirgooController()              # auto-detects /dev/hidraw path
    ctrl.set_zone(15, [(255, 0, 0)] * 24)  # zone 15 = B8, 24 red LEDs
    ctrl.update()                          # sends to hardware
    ctrl.run_keepalive()                   # blocks, sends keepalive loop

For animation:
    ctrl = AirgooController()
    ctrl.set_zone(15, [(255, 0, 0)] * 24)
    ctrl.start_keepalive()                 # background thread
    ...
    ctrl.set_zone(15, new_colors)
    ctrl.update()
    ...
    ctrl.stop()                            # cleanup
"""

import os
import glob
import struct
import threading
import time
from typing import Optional


# USB identifiers
AIRGOO_VID = 0x2486
AIRGOO_AG_DRGB16_PID = 0x3216

# Protocol constants
REPORT_SIZE = 1025           # 1 byte report ID + 1024 bytes payload
REPORT_SIZE_SMALL = 65       # for keepalive packets
PAYLOAD_FIRST = 1020         # usable bytes in first packet (1024 - 4 header bytes)
PAYLOAD_NEXT = 1020          # usable bytes in subsequent packets
LEDS_FIRST = 316             # floor((1024 - 5 - 72) / 3) LEDs in first packet
LEDS_NEXT = 339              # floor((1024 - 5) / 3) LEDs in subsequent packets
HEADER_SIZE = 72             # 2 bytes per zone × 36 max zones
MAX_ZONES = 16               # AG-DRGB16 has 16 zones
MAX_LEDS_PER_ZONE = 256
KEEPALIVE_INTERVAL = 0.4     # seconds (spec says 500ms, we go slightly faster)
KEEPALIVE_BYTE = 0x65

# Zone names for AG-DRGB16
ZONE_NAMES = {
    0: "A1", 1: "A2", 2: "A3", 3: "A4",
    4: "A5", 5: "A6", 6: "A7", 7: "A8",
    8: "B1", 9: "B2", 10: "B3", 11: "B4",
    12: "B5", 13: "B6", 14: "B7", 15: "B8",
}


class AirgooController:
    """
    Direct HID driver for Airgoo AG-DRGB16 LED controllers.

    The controller uses the DRGB V4 protocol. Key behaviors:
      - All zones must be present in every update packet (even if empty).
        The controller ignores packets where some zones have data and others
        don't. Set unused zones to 0 LEDs.
      - The controller reverts to its built-in pattern if it doesn't receive
        a keepalive packet within ~500ms. Use start_keepalive() for a
        background thread, or call send_keepalive() manually.
      - Data is sent as 1025-byte HID reports. Payloads larger than 1020
        bytes are split across multiple packets automatically.

    Important: The /dev/hidraw node can disappear if the USB connection is
    interrupted. If writes fail, call reconnect() to re-detect the device.
    """

    def __init__(self, hidraw_path: Optional[str] = None):
        """
        Open the Airgoo controller.

        Args:
            hidraw_path: Path to hidraw device (e.g. "/dev/hidraw0").
                         Auto-detected if not specified.
        """
        self._fd: Optional[int] = None
        self._zones: list[list[tuple[int, int, int]]] = [[] for _ in range(MAX_ZONES)]
        self._keepalive_thread: Optional[threading.Thread] = None
        self._keepalive_running = False
        self._lock = threading.Lock()

        path = hidraw_path or self._find_device()
        if path is None:
            raise FileNotFoundError(
                "No Airgoo controller found. Check USB connection and "
                "/dev/hidraw permissions. See README for udev rules."
            )
        self._path = path
        self._fd = os.open(path, os.O_RDWR)

    def _find_device(self) -> Optional[str]:
        """Auto-detect the hidraw device for the Airgoo controller."""
        for path in sorted(glob.glob("/dev/hidraw*")):
            try:
                sysfs = f"/sys/class/hidraw/{os.path.basename(path)}/device/uevent"
                if os.path.exists(sysfs):
                    with open(sysfs) as f:
                        content = f.read()
                    vid_pid = f"{AIRGOO_VID:04X}:{AIRGOO_AG_DRGB16_PID:04X}"
                    if vid_pid in content:
                        return path
            except (OSError, PermissionError):
                continue

        # Fallback: try hidraw0 if sysfs isn't available
        for path in sorted(glob.glob("/dev/hidraw*")):
            return path
        return None

    def reconnect(self, hidraw_path: Optional[str] = None):
        """Re-open the device after USB reset or disconnection."""
        self.close()
        path = hidraw_path or self._find_device()
        if path is None:
            raise FileNotFoundError("No Airgoo controller found.")
        self._path = path
        self._fd = os.open(path, os.O_RDWR)

    # ── Zone management ──────────────────────────────────────────────

    def set_zone(self, zone: int, colors: list[tuple[int, int, int]]):
        """
        Set LED colors for a zone.

        Args:
            zone: Zone index 0-15 (see ZONE_NAMES for mapping).
            colors: List of (R, G, B) tuples, one per LED. Length determines
                    how many LEDs are addressed on that channel.

        Example:
            ctrl.set_zone(15, [(255, 0, 0)] * 24)  # B8, 24 red LEDs
        """
        if not 0 <= zone < MAX_ZONES:
            raise ValueError(f"Zone must be 0-{MAX_ZONES-1}, got {zone}")
        if len(colors) > MAX_LEDS_PER_ZONE:
            raise ValueError(f"Max {MAX_LEDS_PER_ZONE} LEDs per zone, got {len(colors)}")
        self._zones[zone] = list(colors)

    def set_zone_by_name(self, name: str, colors: list[tuple[int, int, int]]):
        """
        Set LED colors by zone name (e.g. "B8", "A1").

        Args:
            name: Zone name like "A1" through "A8" or "B1" through "B8".
            colors: List of (R, G, B) tuples.
        """
        name_upper = name.upper()
        for idx, zname in ZONE_NAMES.items():
            if zname == name_upper:
                self.set_zone(idx, colors)
                return
        raise ValueError(f"Unknown zone name: {name}. Valid: {list(ZONE_NAMES.values())}")

    def clear_zone(self, zone: int):
        """Clear all LEDs on a zone (set to 0 LEDs)."""
        self._zones[zone] = []

    def clear_all(self):
        """Clear all zones."""
        self._zones = [[] for _ in range(MAX_ZONES)]

    def fill_zone(self, zone: int, r: int, g: int, b: int, count: int):
        """Fill a zone with a single color."""
        self.set_zone(zone, [(r, g, b)] * count)

    def fill_all(self, r: int, g: int, b: int, count_per_zone: int):
        """Fill all zones with a single color."""
        for z in range(MAX_ZONES):
            self.fill_zone(z, r, g, b, count_per_zone)

    # ── Pixel buffer for display use ─────────────────────────────────

    def set_pixel(self, zone: int, led: int, r: int, g: int, b: int):
        """
        Set a single LED within a zone. The zone must already be sized
        (via set_zone or fill_zone).
        """
        if led < len(self._zones[zone]):
            self._zones[zone][led] = (r, g, b)

    # ── Protocol ─────────────────────────────────────────────────────

    # Minimum LEDs to report per zone. The controller silently ignores
    # updates where any zone has 0 LEDs — all zones must carry data.
    # Zones with no user-set LEDs get this many black (off) LEDs.
    MIN_LEDS_PER_ZONE = 1

    def update(self):
        """
        Send current zone state to the controller.

        Builds the V4 protocol packet(s) and writes to HID. All 16 zones
        are included in every update. Zones with no LEDs set are padded
        with black (off) pixels — the controller requires all zones to
        carry data or it silently ignores the entire update.
        """
        if self._fd is None:
            raise RuntimeError("Device not open. Call reconnect().")

        # Build effective zone data — pad empty zones with black pixels
        effective = []
        for z in range(MAX_ZONES):
            if self._zones[z]:
                effective.append(self._zones[z])
            else:
                effective.append([(0, 0, 0)] * self.MIN_LEDS_PER_ZONE)

        # Count total LEDs
        total_leds = sum(len(z) for z in effective)

        # Build data: 72-byte header + RGB payload
        data = bytearray(HEADER_SIZE + total_leds * 3)

        # Header: 2 bytes per zone (high byte, low byte of LED count)
        for z in range(MAX_ZONES):
            count = len(effective[z])
            data[z * 2] = (count >> 8) & 0xFF
            data[z * 2 + 1] = count & 0xFF

        # RGB data, concatenated across all zones in order
        offset = HEADER_SIZE
        for z in range(MAX_ZONES):
            for r, g, b in effective[z]:
                data[offset] = r & 0xFF
                data[offset + 1] = g & 0xFF
                data[offset + 2] = b & 0xFF
                offset += 3

        # Split into packets
        self._send_packets(data, total_leds)

    def _send_packets(self, data: bytes, total_leds: int):
        """Send data as one or more V4 HID packets."""
        if total_leds == 0:
            # Still send an empty frame to clear the display
            total_leds = 0

        # Calculate number of packets needed
        data_len = len(data)
        if data_len <= PAYLOAD_FIRST:
            num_packets = 1
        else:
            remaining = data_len - PAYLOAD_FIRST
            num_packets = 1 + (remaining + PAYLOAD_NEXT - 1) // PAYLOAD_NEXT

        remaining_leds = total_leds

        for i in range(num_packets):
            packet = bytearray(REPORT_SIZE)
            packet[0] = 0x00  # report ID

            # Packet sequence: i + 100
            packet[1] = i + 100
            # Total packets: num_packets + 99
            packet[2] = num_packets + 99

            # LED count encoding per the V4 protocol
            if i == 0:
                hig = 1 if remaining_leds >= 256 else 0
                low = 60 if remaining_leds > LEDS_FIRST else (remaining_leds % 256)
                chunk_leds = min(remaining_leds, LEDS_FIRST)
            else:
                hig = 1 if remaining_leds >= 256 else 0
                low = 84 if remaining_leds > LEDS_NEXT else (remaining_leds % 256)
                chunk_leds = min(remaining_leds, LEDS_NEXT)

            packet[3] = hig
            packet[4] = low

            # Copy data chunk
            src_start = i * PAYLOAD_FIRST if i == 0 else PAYLOAD_FIRST + (i - 1) * PAYLOAD_NEXT
            src_end = min(src_start + (PAYLOAD_FIRST if i == 0 else PAYLOAD_NEXT), data_len)
            packet[5:5 + (src_end - src_start)] = data[src_start:src_end]

            with self._lock:
                os.write(self._fd, bytes(packet))

            remaining_leds -= chunk_leds

    def send_keepalive(self):
        """Send a single keepalive packet to prevent revert to built-in pattern."""
        if self._fd is None:
            return
        packet = bytearray(REPORT_SIZE_SMALL)
        packet[0] = 0x00
        packet[1] = KEEPALIVE_BYTE
        with self._lock:
            try:
                os.write(self._fd, bytes(packet))
            except OSError:
                pass  # device may have disconnected

    # ── Keepalive management ─────────────────────────────────────────

    def start_keepalive(self):
        """Start background keepalive thread. Call stop() to terminate."""
        if self._keepalive_thread and self._keepalive_running:
            return
        self._keepalive_running = True
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, daemon=True
        )
        self._keepalive_thread.start()

    def _keepalive_loop(self):
        while self._keepalive_running:
            self.send_keepalive()
            time.sleep(KEEPALIVE_INTERVAL)

    def run_keepalive(self):
        """Block and send keepalive packets until KeyboardInterrupt."""
        try:
            while True:
                self.send_keepalive()
                time.sleep(KEEPALIVE_INTERVAL)
        except KeyboardInterrupt:
            pass

    # ── Lifecycle ────────────────────────────────────────────────────

    def close(self):
        """Stop keepalive and close the device."""
        self._keepalive_running = False
        if self._keepalive_thread:
            self._keepalive_thread.join(timeout=2)
            self._keepalive_thread = None
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    def stop(self):
        """Alias for close()."""
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        self.close()


# ── Convenience for CLI/testing ──────────────────────────────────────

def main():
    """Quick test: fill all zones with dim white."""
    import argparse

    parser = argparse.ArgumentParser(description="Airgoo AG-DRGB16 LED Controller")
    parser.add_argument("--device", "-d", help="hidraw device path (auto-detected if omitted)")
    parser.add_argument("--zone", "-z", type=int, help="Zone index 0-15 (default: all)")
    parser.add_argument("--zone-name", "-n", help="Zone name like A1, B8")
    parser.add_argument("--color", "-c", default="FF0000", help="Hex color (default: FF0000)")
    parser.add_argument("--leds", "-l", type=int, default=24, help="LEDs per zone (default: 24)")
    parser.add_argument("--list-zones", action="store_true", help="List zone names and exit")
    args = parser.parse_args()

    if args.list_zones:
        for idx, name in sorted(ZONE_NAMES.items()):
            print(f"  {idx:2d}: {name}")
        return

    r = int(args.color[0:2], 16)
    g = int(args.color[2:4], 16)
    b = int(args.color[4:6], 16)

    ctrl = AirgooController(hidraw_path=args.device)

    if args.zone_name:
        ctrl.set_zone_by_name(args.zone_name, [(r, g, b)] * args.leds)
    elif args.zone is not None:
        ctrl.set_zone(args.zone, [(r, g, b)] * args.leds)
    else:
        ctrl.fill_all(r, g, b, args.leds)

    ctrl.update()
    print(f"Color #{args.color} sent. Keepalive running (Ctrl+C to stop)...")
    ctrl.run_keepalive()


if __name__ == "__main__":
    main()
