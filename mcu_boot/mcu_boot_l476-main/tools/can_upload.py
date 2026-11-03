"""Package a raw CubeIDE application binary and upload it over classic CAN."""
import argparse
import hashlib
from pathlib import Path
import struct
import time
import zlib
import cantools

DATABASE = cantools.database.load_file(Path(__file__).resolve().parents[1] / "docs/bootloader.dbc", strict=True)
REQUEST = DATABASE.get_message_by_name("BootloaderRequest")
RESPONSE = DATABASE.get_message_by_name("BootloaderResponse")
ENTER_BOOT = DATABASE.get_message_by_name("ENTER_BOOT")
REQUEST_ID, RESPONSE_ID, ENTER_BOOT_ID = REQUEST.frame_id, RESPONSE.frame_id, ENTER_BOOT.frame_id
INFO, BEGIN, DATA, END, RESET = range(5)
ERRORS = ("OK", "bad frame", "bad state", "bad size", "bad offset", "flash error", "invalid image / SHA-256")


def check_image(image):
    """Check the file before allowing any erase on the device."""
    if len(image) < 32:
        raise ValueError("Expected an MCUboot image, not the raw application binary")
    magic, _, header, protected, size, flags = struct.unpack_from("<IIHHII", image)
    if magic != 0x96F3B83D or header != 512 or protected or flags or size < 8:
        raise ValueError("Unsupported image header: require 512-byte header, no protected TLVs/encryption")
    end = header + size
    if end + 4 > len(image):
        raise ValueError("Truncated image")
    magic, total = struct.unpack_from("<HH", image, end)
    if magic != 0x6907 or total < 4 or end + total != len(image):
        raise ValueError("Invalid TLV block; do not use imgtool --pad or --confirm")
    expected = hashlib.sha256(image[:end]).digest()
    found = 0
    pos = end + 4
    while pos < len(image):
        if pos + 4 > len(image):
            raise ValueError("Truncated TLV")
        kind, length = struct.unpack_from("<HH", image, pos)
        pos += 4
        if pos + length > len(image):
            raise ValueError("TLV exceeds image")
        if kind == 0x10:
            found += 1
            if length != 32 or image[pos:pos + length] != expected:
                raise ValueError("File SHA-256 does not match its MCUboot hash")
        pos += length
    if found != 1:
        raise ValueError("Expected exactly one SHA-256 TLV")
    return expected.hex()


def prepare_image(data):
    """Retain packaged images; wrap raw firmware linked at 0x08010200."""
    if len(data) >= 4 and struct.unpack_from("<I", data)[0] == 0x96F3B83D:
        check_image(data)
        payload = data[512:512 + struct.unpack_from("<I", data, 12)[0]]
        packaged = data
    else:
        payload = data
        header = struct.pack("<IIHHIIBBHII", 0x96F3B83D, 0, 512, 0, len(payload), 0,
                             1, 0, 0, 0, 0).ljust(512, b"\0")
        body = header + payload
        packaged = body + struct.pack("<HHHH", 0x6907, 40, 0x10, 32) + hashlib.sha256(body).digest()
    if len(payload) < 8:
        raise ValueError("Application binary must contain a vector table")
    sp, pc = struct.unpack_from("<II", payload)
    ram = 0x20000000 < sp <= 0x20018000 or 0x10000000 < sp <= 0x10008000
    if not ram or sp & 7 or not pc & 1 or not 0x08010200 <= (pc & ~1) < 0x08010200 + len(payload):
        raise ValueError("Invalid application vectors: build the CubeIDE application for 0x08010200; "
                         "a bootloader or a binary linked for 0x08000000 cannot be relocated by packaging")
    if len(packaged) > 0x6F800:
        raise ValueError("Packaged application exceeds the 456704-byte bank slot limit")
    check_image(packaged)
    return packaged


def print_progress(received, total):
    percent = 100 * received // total
    filled = 30 * received // total
    print(f"\rProgramming [{'#' * filled}{'-' * (30 - filled)}] {percent:3}% "
          f"({received}/{total} bytes)", end="", flush=True)


def request(bus, payload, expected_offset=None, timeout=1.0, retries=3):
    import can
    for _ in range(retries):
        bus.send(can.Message(arbitration_id=REQUEST_ID, is_extended_id=False, data=payload), timeout=timeout)
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            msg = bus.recv(remaining)
            if msg is None:
                break
            if (msg.arbitration_id != RESPONSE_ID or msg.is_extended_id or msg.is_remote_frame
                    or msg.is_error_frame or msg.is_fd or len(msg.data) != 8):
                continue
            response = RESPONSE.decode(msg.data, decode_choices=False)
            if response["ReplyCommand"] != payload[0] | 0x80:
                continue
            if response["ProtocolVersion"] != 1:
                raise RuntimeError("Unsupported bootloader protocol version")
            status = response["Status"]
            if status:
                error = ERRORS[status] if status < len(ERRORS) else f"error {status}"
                raise RuntimeError(f"Command {payload[0]} rejected: {error}")
            value = response["ValueBytes"]
            if expected_offset is None or value == expected_offset:
                return value
    raise TimeoutError(f"No matching ACK for command {payload[0]}; reset into bootloader and retry")


def command(cmd, value=0):
    field = {INFO: "InfoReserved", BEGIN: "ImageLength", END: "ImageCRC32", RESET: "ResetReserved"}[cmd]
    return REQUEST.encode({"Command": cmd, field: value})


def upload(bus, image):
    digest = check_image(image)
    maximum = request(bus, command(INFO))
    if len(image) > maximum:
        raise ValueError(f"Image is {len(image)} bytes; device accepts at most {maximum}")
    request(bus, command(BEGIN, len(image)), expected_offset=0, timeout=20)
    print_progress(0, len(image))
    last_percent = 0
    for offset in range(0, len(image), 4):
        chunk = image[offset:offset + 4].ljust(4, b"\xff")
        payload = REQUEST.encode({"Command": DATA, "ImageOffset": offset,
                                  **{f"PayloadByte{i}": byte for i, byte in enumerate(chunk)}})
        request(bus, payload, expected_offset=min(offset + 4, len(image)))
        received = min(offset + 4, len(image))
        percent = 100 * received // len(image)
        if percent != last_percent:
            print_progress(received, len(image))
            last_percent = percent
    print("\nTransfer complete; validating and committing image...")
    request(bus, command(END, zlib.crc32(image)), expected_offset=len(image), timeout=20)
    print(f"\rUploaded {len(image)} bytes. Device SHA-256 validation passed.\nImage SHA-256: {digest}")
    try:
        request(bus, command(RESET), timeout=1, retries=1)
    except TimeoutError:
        print("Image accepted; reset ACK not received. Reset the board to activate it.")
        return
    print("Reset acknowledged. Bootloader will validate both banks and boot the newest valid image.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Raw CubeIDE application .bin linked at 0x08010200, or an unpadded MCUboot image")
    parser.add_argument("--interface", default="slcan", help="python-can backend; stock WeAct uses slcan")
    parser.add_argument("--channel", required=True, help="e.g. COM5 or can0")
    args = parser.parse_args()
    raw = args.image.read_bytes()
    image = prepare_image(raw)
    if image != raw:
        packaged = args.image.with_suffix(".mcuboot.bin")
        packaged.write_bytes(image)
        print(f"Packaged {args.image} -> {packaged} ({len(image)} bytes)")
    else:
        print(f"Using packaged image {args.image} ({len(image)} bytes)")
    import can
    with can.Bus(interface=args.interface, channel=args.channel, bitrate=250000) as bus:
        bus.send(can.Message(arbitration_id=ENTER_BOOT_ID, is_extended_id=False,
                             data=ENTER_BOOT.encode({"BCI_BOOT": 1})), timeout=1.0)
        print("Sent ENTER_BOOT (0x601), BCI_BOOT=1. Waiting up to 30 seconds for the bootloader.")
        print("If the application does not handle ENTER_BOOT, reset into the bootloader manually.")
        deadline = time.monotonic() + 30
        while True:
            try:
                request(bus, command(INFO), timeout=0.3, retries=1)
                break
            except TimeoutError:
                if time.monotonic() >= deadline:
                    raise
        upload(bus, image)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc
