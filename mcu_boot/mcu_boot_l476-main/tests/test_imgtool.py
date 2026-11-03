"""Regression: ST imgtool must package an unsigned image with no protected TLVs."""
import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
from can_upload import check_image, prepare_image


class ImgtoolTest(unittest.TestCase):
    def test_hash_only_image(self):
        root = pathlib.Path(__file__).resolve().parents[1]
        tool = root / "Drivers/stm32_mcu_boot/scripts/imgtool.py"
        with tempfile.TemporaryDirectory() as directory:
            raw = pathlib.Path(directory) / "app.bin"
            packaged = pathlib.Path(directory) / "image.bin"
            raw.write_bytes(bytes(range(256)) + b"odd")
            subprocess.run([sys.executable, str(tool), "sign", "--align", "8",
                            "--header-size", "0x200", "--pad-header", "--slot-size", "0x70000",
                            "--overwrite-only", "--version", "1.0.0", str(raw), str(packaged)], check=True)
            check_image(packaged.read_bytes())
            subprocess.run([sys.executable, str(tool), "verify", str(packaged)], check=True)

    def test_automatic_package_verified_by_imgtool(self):
        root = pathlib.Path(__file__).resolve().parents[1]
        tool = root / "Drivers/stm32_mcu_boot/scripts/imgtool.py"
        raw = struct.pack("<II", 0x20018000, 0x08010209) + b"application payload"
        with tempfile.TemporaryDirectory() as directory:
            packaged = pathlib.Path(directory) / "converted.bin"
            packaged.write_bytes(prepare_image(raw))
            subprocess.run([sys.executable, str(tool), "verify", str(packaged)], check=True)


if __name__ == "__main__":
    unittest.main()
