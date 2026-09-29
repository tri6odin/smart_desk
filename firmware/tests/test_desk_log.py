"""Cross-language on-flash format, decoder, filtering and CLI checks."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import desk_log  # noqa: E402

WRITER = sys.argv.pop(1)


class DecoderTests(unittest.TestCase):
    def test_firmware_format_and_torn_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "journal.bin"
            subprocess.run([WRITER, "--fixture", str(path)], check=True)
            journal = desk_log.decode(path.read_bytes())
            self.assertEqual([e.sequence for e in journal.events], [1, 2, 3, 4])
            self.assertEqual(journal.invalid_records, 1)
            text = desk_log.render(journal, 0, True)
            self.assertIn("UP_HOLD [dial]", text)
            self.assertIn("UNKNOWN 55 AA 08 07 00 02 01 10", text)
            self.assertIn("1193:02:47.", text)  # uptime does not wrap at 2^32 ms
            self.assertNotIn("DIAL_UP_START", text)
            self.assertNotIn("SETTLED", text)
            self.assertEqual(desk_log.render(journal, 1).count("UP_HOLD"), 0)
            result = subprocess.run([sys.executable, str(ROOT / "tools/desk_log.py"),
                                     "--input", str(path), "--all"],
                                    capture_output=True, text=True, check=True)
            self.assertIn("Stored: 4 records", result.stdout)

    def test_empty_and_invalid_dump(self):
        self.assertEqual(desk_log.decode(b"\xff" * 8192).events, [])
        with self.assertRaises(ValueError):
            desk_log.decode(b"short")
        journal = desk_log.decode(b"\0" * 8192)
        self.assertEqual(journal.invalid_headers, 2)
        self.assertEqual(journal.events, [])

    def test_status_changes_and_failed_tx_are_visible(self):
        height = desk_log.Event(0, 1, 1, bytes.fromhex("55 AA 08 02 03 21 02 2D"), 2, 0)
        self.assertIn("status=0x02", desk_log.describe(height)[1])
        error = desk_log.Event(0, 1, 2, bytes.fromhex("AA 55 08 01 00 08 01 10"), 5, 0)
        source, text = desk_log.describe(error)
        self.assertEqual(source, "ESP")
        self.assertIn("delivery unknown", text)

    def test_partition_layout_matches_reader(self):
        lines = (ROOT / "firmware/partitions.journal.csv").read_text().splitlines()
        entries = [line.split(",") for line in lines if line and not line.startswith("#")]
        entries = {row[0].strip(): [part.strip() for part in row] for row in entries}
        self.assertEqual(int(entries["desk_log"][3], 0), desk_log.FLASH_OFFSET)
        self.assertEqual(int(entries["desk_log"][4], 0), desk_log.FLASH_SIZE)
        self.assertEqual(int(entries["nvs"][3], 0), 0x9000)
        self.assertEqual(int(entries["nvs"][4], 0), 0x6000)
        self.assertEqual(int(entries["factory"][3], 0), 0x10000)
        self.assertEqual(int(entries["factory"][4], 0), 0x177000)


if __name__ == "__main__":
    unittest.main()
