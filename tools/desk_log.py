#!/usr/bin/env python3
"""Read/decode the persistent desk journal. Never erase or write ESP flash."""

import argparse
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import struct
import subprocess
import sys
import zlib

FLASH_OFFSET = 0x200000
FLASH_SIZE = 0x200000
SECTOR_SIZE = 4096
RECORD_SIZE = 32
MAGIC = 0x314A4453
HEADER = struct.Struct("<IHHQIIII")
RECORD = struct.Struct("<QII8sBBHI")
COMMANDS = {0: "IDLE", 1: "RECALL_STAND", 2: "RECALL_SIT", 3: "SAVE_STAND",
            4: "SAVE_SIT", 5: "DOWN_STEP", 6: "DOWN_HOLD", 7: "UP_STEP",
            8: "UP_HOLD", 14: "WAKE"}
CONTEXTS = {1: "dial", 2: "SIT", 3: "STAND", 4: "calibration"}
STOP_REASONS = {
    1: "STOP_REQUESTED", 2: "BUTTON_DURING_MOTION", 3: "DIAL_REVERSAL",
    4: "DIAL_OVERRIDE", 5: "DIAL_RELEASED", 6: "TELEMETRY_STALE",
    7: "UNEXPECTED_FRAME", 8: "TELEMETRY_REVERSED", 9: "NO_INITIAL_MOVEMENT",
    10: "HEIGHT_STALLED", 11: "STEP_WATCHDOG", 12: "TARGET_BRAKE",
    13: "MANUAL_WATCHDOG", 14: "PRESET_WATCHDOG", 15: "CALIBRATION_CANCELLED",
    16: "RST_RECEIVED", 17: "CALIBRATION_TIMEOUT",
}
RST = bytes.fromhex("55 AA 08 04 00 01 01 0C")
STARTUP = bytes.fromhex("55 AA 08 0F 00 0F 01 25")
# Exact frame observed during desk supply shutdown; diagnostic name only.
POWER_OFF = bytes.fromhex("55 AA 08 03 08 00 01 12")


@dataclass(frozen=True)
class Event:
    uptime_ms: int
    boot_id: int
    sequence: int
    data: bytes
    kind: int
    context: int


@dataclass
class Journal:
    events: list
    invalid_headers: int = 0
    invalid_records: int = 0


def crc_ok(raw):
    return zlib.crc32(raw[:28]) == struct.unpack_from("<I", raw, 28)[0]


def decode(blob):
    if len(blob) < 2 * SECTOR_SIZE or len(blob) % SECTOR_SIZE:
        raise ValueError("dump must contain whole 4096-byte sectors (at least two)")
    journal = Journal([])
    sectors = []
    for offset in range(0, len(blob), SECTOR_SIZE):
        raw = blob[offset:offset + RECORD_SIZE]
        if raw == b"\xff" * RECORD_SIZE:
            # An interrupted erase may have cleared the header but not its tail.
            if blob[offset:offset + SECTOR_SIZE] != b"\xff" * SECTOR_SIZE:
                journal.invalid_headers += 1
            continue
        magic, version, size, generation, _, _, reserved, _ = HEADER.unpack(raw)
        if (magic != MAGIC or version != 1 or size != RECORD_SIZE or
                generation == 0 or reserved != 0 or not crc_ok(raw)):
            journal.invalid_headers += 1
            continue
        sectors.append((generation, offset))
    for _, offset in sorted(sectors):
        for pos in range(offset + RECORD_SIZE, offset + SECTOR_SIZE, RECORD_SIZE):
            raw = blob[pos:pos + RECORD_SIZE]
            if raw == b"\xff" * RECORD_SIZE:
                continue
            uptime, boot, sequence, data, kind, context, reserved, _ = RECORD.unpack(raw)
            if reserved or kind not in range(1, 8) or not crc_ok(raw):
                journal.invalid_records += 1
                continue
            journal.events.append(Event(uptime, boot, sequence, data, kind, context))
    return journal


def tx_annotation(event):
    parts = []
    source = CONTEXTS.get(event.context & 7)
    if source:
        parts.append(source)
    reason = event.context >> 3 if event.data[5] == 0 else 0
    if reason:
        parts.append(STOP_REASONS.get(reason, f"STOP_REASON_{reason}"))
    return f" [{'; '.join(parts)}]" if parts else ""


def describe(event, raw_frames=False):
    data = event.data
    hex_data = data.hex(" ").upper()
    if event.kind == 1:
        return "BOOT", f"RESET={event.context} ELF_SHA256={data.hex()}"
    if event.kind == 6:
        return "LOGGER", f"GAP: {int.from_bytes(data, 'little')} queued records lost"
    if event.kind == 7:
        old_sit, old_stand, sit, stand = struct.unpack("<HHHH", data)
        def cm(value):
            return f"{value // 10}.{value % 10}"
        if event.context == 2:
            return "ESP", f"SAVE_SIT {cm(old_sit)} -> {cm(sit)} cm"
        if event.context == 3:
            return "ESP", f"SAVE_STAND {cm(old_stand)} -> {cm(stand)} cm"
        return "ESP", f"PRESETS SIT={cm(sit)} cm STAND={cm(stand)} cm"
    if event.kind == 5:
        command = COMMANDS.get(data[5], f"0x{data[5]:02X}")
        return "ESP", f"TX_ERROR attempted={command}{tx_annotation(event)}; delivery unknown; {hex_data}"
    if event.kind == 4:
        return "DESK -> ESP", f"INVALID {hex_data}"
    if event.kind == 3:
        text = COMMANDS.get(data[5], f"UNKNOWN 0x{data[5]:02X}")
        text += tx_annotation(event)
        return "ESP -> DESK", text + (f" | {hex_data}" if raw_frames else "")
    if data == RST:
        text = "RST"
    elif data == STARTUP:
        text = "STARTUP"
    elif data == POWER_OFF:
        text = "POWER_OFF"
    elif data[3] == 2:
        height = ((data[4] & 0x3f) << 8) | data[5]
        text = f"HEIGHT {height // 10}.{height % 10} cm"
        if data[4] & 0xc0 or data[6] != 1:
            text += f" [flags=0x{data[4] & 0xc0:02X} status=0x{data[6]:02X}]"
    else:
        return "DESK -> ESP", f"UNKNOWN {hex_data}"
    return "DESK -> ESP", text + (f" | {hex_data}" if raw_frames else "")


def format_event(event, raw_frames=False):
    seconds, millis = divmod(event.uptime_ms, 1000)
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    source, text = describe(event, raw_frames)
    return (f"{event.sequence:8d}  {event.boot_id:5d}  "
            f"{hours:02d}:{minutes:02d}:{seconds:02d}.{millis:03d}  "
            f"{source:11s}  {text}")


def render(journal, last=128, raw_frames=False):
    events = journal.events[-last:] if last else journal.events
    lines = [f"Stored: {len(journal.events)} records; showing: {len(events)}; "
             f"damaged/incomplete: {journal.invalid_headers} sectors, "
             f"{journal.invalid_records} records"]
    # Show firmware identification even when the boot event is outside --last.
    boots = {e.boot_id for e in events}
    for e in journal.events:
        if e.kind == 1 and e.boot_id in boots:
            lines.append(f"Boot {e.boot_id}: reset={e.context}, ELF_SHA256={e.data.hex()}")
    lines.append("       #   BOOT  UPTIME         SOURCE       EVENT")
    previous = None
    for event in events:
        if previous is not None and event.sequence != (previous + 1) & 0xffffffff:
            lines.append("          ... sequence gap: records missing or damaged ...")
        lines.append(format_event(event, raw_frames))
        previous = event.sequence
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="ESP serial port; enters bootloader and reads flash")
    source.add_argument("--input", type=Path, help="previously captured raw desk_log.bin")
    parser.add_argument("--output", type=Path, help="raw dump path when reading --port")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--last", type=int, default=128, help="show last N records (default 128)")
    selection.add_argument("--all", action="store_true", help="show all stored records")
    parser.add_argument("--raw", action="store_true", help="include full bytes for every UART frame")
    args = parser.parse_args()
    if args.last < 1:
        parser.error("--last must be positive; use --all for the whole journal")
    if args.input and args.output:
        parser.error("--output is only valid with --port")
    path = args.input
    if args.port:
        path = args.output or Path(f"desk-log-{datetime.now():%Y%m%d-%H%M%S}.bin")
        if path.exists():
            parser.error(f"refusing to overwrite existing capture: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        # Constants match partitions.journal.csv; read-flash cannot erase flash.
        command = [sys.executable, "-m", "esptool", "--chip", "esp32s3",
                   "--port", args.port, "--baud", "460800", "read-flash",
                   hex(FLASH_OFFSET), hex(FLASH_SIZE), str(path)]
        print("Reading journal; ESP is temporarily in the bootloader.", file=sys.stderr)
        subprocess.run(command, check=True, stdout=sys.stderr)
    journal = decode(path.read_bytes())
    output = render(journal, 0 if args.all else args.last, args.raw)
    if args.port:
        # Always save a full raw-text companion, even when stdout is --last 128.
        text_path = path.with_suffix(".txt")
        if text_path.exists():
            parser.error(f"refusing to overwrite existing text capture: {text_path}")
        text_path.write_text(render(journal, 0, True), encoding="utf-8")
        print(f"Saved {path} and {text_path}", file=sys.stderr)
    print(output, end="")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"desk_log: {error}", file=sys.stderr)
        sys.exit(1)
