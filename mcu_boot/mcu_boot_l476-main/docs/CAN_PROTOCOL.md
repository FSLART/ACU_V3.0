# CAN upload protocol, version 1

This project-specific protocol transports an MCUboot image. It is not an upstream MCUboot serial recovery protocol. Classic CAN, 250 kbit/s, standard 11-bit data frames, DLC 8 only. Request ID: `0x600`; response ID: `0x580`. One uploader and one target share these fixed IDs. Integers are little-endian.

The [DBC file](bootloader.dbc) defines programming, entry and heartbeat frames, multiplexed request payloads, and command/status/state labels. Response `ValueBytes` means maximum upload size for INFO and accepted byte count for every other command.

## DBC and generated C codec

The saved DBC is the source of CAN IDs and bit layouts. The MCU uses cantools-generated [bootloader.h](../Core/Inc/bootloader.h) and [bootloader.c](../Core/Src/bootloader.c) to pack outgoing frames and unpack upload requests. The Python uploader loads the same DBC and uses `encode()`/`decode()` for entry, upload commands and replies. CAN HAL still handles filtering, mailboxes and transmission; the codec handles bytes only.

### After editing the DBC

1. Save `docs/bootloader.dbc` in your DBC editor.
2. From the repository root, run:

   ```powershell
   .venv\Scripts\python.exe -m pip install -r tools/requirements.txt
   .venv\Scripts\python.exe tools/generate_can.py
   .venv\Scripts\python.exe -m unittest discover -s tests -v
   ```

3. Rebuild and reflash the bootloader. In CubeIDE, refresh the project (F5) so new sources are included. CMake already includes `Core/Src/bootloader.c`.
4. If signal names change, update their callers in `boot_port.c`, `boot_update.c` and `can_upload.py`. Update any other ECU using these frames as well.

Do not edit generated files manually. `generate_can.py` calls cantools' C generator, checks the DBC strictly and writes both files directly to their build locations. The regression test detects a stale codec. The generated functions include both pack and unpack for every message. Unused functions are discarded by the firmware linker.

If you generate files yourself with the [cantools CLI](https://cantools.readthedocs.io/en/stable/#the-generate-c-source-subcommand), use database name `bootloader`, place `bootloader.h` in `Core/Inc` and `bootloader.c` in `Core/Src`. Avoid leaving a second C copy in a compiled source folder.

### C example: encode heartbeat and decode entry

```c
#include "bootloader.h"

uint8_t data[8];
struct bootloader_bci_boot_heartbeat_t heartbeat = {
    .boot = 0x55,
    .timestamp = HAL_GetTick(),
    .boot_version_major = BOOT_VERSION_MAJOR,
    .boot_version_minor = BOOT_VERSION_MINOR,
    .boot_version_patch = BOOT_VERSION_PATCH
};
bootloader_bci_boot_heartbeat_pack(data, &heartbeat, sizeof data);
/* Send data as a standard frame with BOOTLOADER_BCI_BOOT_HEARTBEAT_FRAME_ID. */

struct bootloader_enter_boot_t entry;
if (rx.IDE == CAN_ID_STD && rx.RTR == CAN_RTR_DATA &&
    rx.StdId == BOOTLOADER_ENTER_BOOT_FRAME_ID && rx.DLC == 8 &&
    bootloader_enter_boot_unpack(&entry, received, rx.DLC) == 0 && entry.bci_boot == 1) {
    RequestBootloader();
}
```

The receive example belongs in the application; see [porting step 9](PORTING.md#step-9--add-requestbootloader-to-the-application) for RAM reservation and `RequestBootloader()`. Always validate CAN frame type and DLC before calling unpack. Generated unpack checks buffer length but does not enforce every signal's allowed range; keep command, offset, size and state validation in the protocol handler.

### Python example: encode entry and decode heartbeat

```python
import cantools

db = cantools.database.load_file("docs/bootloader.dbc", strict=True)
entry = db.get_message_by_name("ENTER_BOOT")
payload = entry.encode({"BCI_BOOT": 1})  # 01 00 00 00 00 00 00 00
print(hex(entry.frame_id), payload.hex())

signals = db.decode_message(0x581, bytes.fromhex("55 78 56 34 12 01 00 00"),
                            decode_choices=False)
print(signals["Timestamp"])  # 305419896 milliseconds
print(signals["BootVersionMajor"], signals["BootVersionMinor"], signals["BootVersionPatch"])
```

Keep `docs/bootloader.dbc` available when distributing the uploader. It resolves the path relative to `can_upload.py`, so it works from another working directory when the repository layout is retained. Diagnostics remain separate from upload ACKs; the uploader ignores their IDs during programming.

## Enter bootloader from the running application

Before polling INFO, the uploader sends one standard CAN data frame with ID **`0x601`**, DLC **8**, and payload **`01 00 00 00 00 00 00 00`**. `ENTER_BOOT.BCI_BOOT` is an unsigned one-bit Intel signal at bit 0, with range 0-1. Unused bits are zero. It is a normal boolean, not a multiplexing selector.

The running application must receive this frame, check `BCI_BOOT=1`, set `0xB00710AD` at RAM address `0x20017FFC`, and reset. The bootloader consumes that RAM request and stays available for programming. There is no ENTER_BOOT acknowledgement; the uploader waits up to 30 seconds for the normal INFO response and does not resend the entry request during INFO retries. If the application lacks this handler, use manual reset/button entry. Reserve `0x601` for this application entry command on the CAN bus.

## Bootloader heartbeat

`BCI_BOOT_HEARTBEAT` is a standard data frame on **`0x581`**, DLC **8**, sent immediately after CAN starts, after successful INFO replies, and normally every **500 ms** while the bootloader loop runs. It stops when the application starts.

| Byte | Signal | Bootloader value |
| --- | --- | --- |
| 0 | `boot` | `0x55` (decimal 85), DBC label `BOOT` |
| 1-4 | `Timestamp` | Unsigned 32-bit `HAL_GetTick()` milliseconds, little-endian; resets on MCU startup and wraps modulo 2^32 |
| 5 | `BootVersionMajor` | Configured bootloader version major |
| 6 | `BootVersionMinor` | Configured bootloader version minor |
| 7 | `BootVersionPatch` | Configured bootloader version patch |

The DBC also labels `0xAA` (170) as `APP`; application-side transmission is separate and is not provided by this bootloader. Heartbeat is mode indication, not a programming ACK. The uploader ignores it because replies use `0x580`.

Transmission never waits and keeps one mailbox available for programming replies. Busy mailboxes or a failed enqueue skip that period; the next heartbeat samples the current tick. Blocking flash operations can delay the heartbeat; receiving boards must tolerate those gaps. Do not infer successful firmware installation from this frame. Reserve `0x581` alongside the other protocol IDs.

## CAN diagnostics

All diagnostic frames are standard CAN data frames, DLC 8, little-endian. They are observations; programming replies remain on `0x580`. The Python uploader ignores these IDs.

| ID | DBC message | Payload, in byte order |
| --- | --- | --- |
| `0x582` | `BCI_BOOT_INFO` | Protocol u8, board type u8, valid-bank mask u8, upload stage u8, maximum packaged bytes u32 |
| `0x583` | `BCI_BOOT_FIRMWARE_VERSION` | Selected application's major u8, minor u8, revision u16, build u32 |
| `0x584` | `BCI_BOOT_DECISION` | Selected bank u8, entry flags u8, rejected bank u8, reject reason u8, captured RCC_CSR u32 |
| `0x585` | `BCI_BOOT_UPLOAD_STATUS` | Target bank u8, stage u8, received bytes u24, total bytes u24 |
| `0x586` | `BCI_BOOT_ERROR` | Code u8, bank u8, command u8, source u8, detail u32 |
| `0x587` | `BCI_BOOT_ROLLBACK` | Rejected bank u8, selected bank u8, reason u8, rejected-version-known u8, rejected generation u32 |
| `0x588` | `BCI_BOOT_FLASH_ERROR` | Device-relative flash offset u32, raw HAL flash error u32 |
| `0x589` | `BCI_BOOT_REJECTED_VERSION` | Rejected header's major u8, minor u8, revision u16, build u32 |

`u8/u16/u24/u32` mean unsigned integers of that many bits. Bank 0 means none; 1/2 identify the physical bank. Board type 1 is this STM32L476RG port. Valid-bank mask bit 0 is bank 1, bit 1 is bank 2. Version on `0x583` is only sent for a selected valid image; it does not prove the application has run. Rejected version is unverified header metadata and only sent with `RejectedVersionKnown=1`.

Bootloader semantic version is configured by `BOOT_VERSION_MAJOR/MINOR/PATCH` in `mcuboot_config.h` and is independent of the application version. Current initial value is **1.0.0**. `BOOT_BOARD_TYPE` and `BOOT_PROTOCOL_VERSION` are defined there too; update the board-type DBC labels when porting.

Entry flags combine RAM request=1, button=2, no-valid-image recovery=4, rollback=8. Reset flags are captured before the bootloader clears them; consult the L476 reference manual for RCC_CSR bits. These indicate why the current boot happened, not a persistent reset history.

Upload stages: 0 IDLE, 1 ERASING, 2 RECEIVING, 3 VALIDATING, 4 COMMITTED, 5 FAILED. Counts include header/TLV bytes. Stage changes drain diagnostics for at most 20 ms before blocking erase/validation; progress publishes every 16 KiB and at the final byte. INFO requests report current accepted counts. A rejected command reports an error; a still-active receiving session or already-committed image retains that stage unless validation/flash fails.

Startup and INFO request a snapshot of currently applicable fields. Selection/rollback/version are startup state, not predictions of a later application boot. Last error persists in RAM until MCU reset. Under CAN congestion, pending fields coalesce to their latest values rather than forming an unbounded event queue. Diagnostic/heartbeat enqueue is non-blocking and keeps a mailbox free for programming replies. A bounded 20 ms drain is used before long flash phases, reset and handoff; some frames can be lost if the bus is unavailable. INFO can request another snapshot once the link recovers. Errors during CAN initialization cannot be delivered over a failed CAN link; UART remains the source for that failure.

### Error codes and Detail

| Code | Name | Detail meaning |
| --- | --- | --- |
| 0 | NONE | No error |
| 1 | HEADER | Image-header device-relative offset |
| 2 | TLV | TLV device-relative offset |
| 3 | HASH | Image-header device-relative offset |
| 4 | VECTORS | Rejected reset PC |
| 5 | CRC | Device-computed upload CRC |
| 6 | BAD_FRAME | Received DLC |
| 7 | BAD_STATE | Received byte count |
| 8 | BAD_SIZE | Requested length |
| 9 | BAD_OFFSET | Requested DATA offset |
| 10-14 | FLASH_READ/WRITE/ERASE/UNLOCK/LOCK | Affected device-relative flash offset |
| 15-16 | CAN_QUEUE/TIMEOUT | Raw HAL CAN error bits |
| 17 | HANDOFF | Rejected PC/offset, or 0 for missing header |
| 18 | FLASH_CONFIG | Raw FLASH_OPTR |
| 19 | INACTIVITY | Received byte count before timeout reset |
| 20 | RECORD | Device-relative commit-record offset |
| 21 | FLASH_OPERATION | Byte count when a flash-map operation failed without more specific backend details |

Source: 1 validation, 2 upload, 3 flash driver, 4 CAN, 5 boot. Command 255 means startup/no command; 0-4 are the existing commands. `0x588` accompanies a flash-driver error with its raw HAL detail. Add `FLASH_BASE_ADDRESS` to a device-relative offset to obtain the physical address before application bank remapping. Rejected generation/version are diagnostic metadata, not anti-rollback authorization.

Reserve IDs `0x580`–`0x589`, `0x600` and `0x601` for one target using this protocol. No node addressing is added by these diagnostics.

## Requests

| Command | Byte 0 | Bytes 1–3 | Byte 4 | Bytes 5–7 |
| --- | --- | --- | --- | --- |
| INFO | `00` | Zero | Zero | Zero |
| BEGIN | `01` | Low 3 bytes of image length | High length byte | Zero |
| DATA | `02` | 24-bit image offset | First payload byte | Other 3 payload bytes |
| END | `03` | Low 3 bytes of CRC32 | High CRC byte | Zero |
| RESET | `04` | Zero | Zero | Zero |

Reserved request bytes should be zero. DATA always sends four payload bytes; pad the final frame with `FF`. Only bytes within the declared image length count. Offsets start at zero (the MCUboot header), then advance by four. No windowing or pipelining: wait for each ACK.

## Replies

| Bytes | Meaning |
| --- | --- |
| 0 | Request opcode OR `0x80` |
| 1 | Status |
| 2–5 | INFO: maximum upload size; otherwise: accepted byte count |
| 6 | Protocol version, `1` |
| 7 | `0` idle, `1` receiving, `2` validated and committed for next boot |

| Status | Meaning |
| --- | --- |
| 0 | OK |
| 1 | Wrong DLC or unknown command |
| 2 | Invalid state, including premature END |
| 3 | Image length outside allowed bounds |
| 4 | Unexpected offset |
| 5 | Flash operation or commit-record failure |
| 6 | CRC or MCUboot image validation failure |

## Transaction and integrity

INFO returns `0x6F800` for this layout. BEGIN accepts at least 560 bytes and at most that maximum. It erases the inactive slot's final page first to invalidate the previous record, then its remaining pages. It preserves the selected valid bank. The slots start physically at `0x08010000` and `0x08090000`. BEGIN prepares length and generation in the first half of the final 16-byte bank record. A repeated BEGIN with the same length before any DATA is accepted does not erase again; a later BEGIN restarts the transfer. BEGIN after a successful END is rejected until reset.

DATA assembles 8-byte flash programming units from pairs of frames and pads the last unit with erased bytes. Only an exact retry of the most recently accepted DATA frame is acknowledged without programming again. Older or out-of-order frames fail. This avoids programming an L476 ECC doubleword twice after a lost ACK.

END carries the standard reflected CRC-32 used by Python `zlib.crc32(image)` (polynomial `0xEDB88320`, initial/final XOR `0xFFFFFFFF`) over the complete transferred file, including header and TLVs. The device also rereads flash and invokes MCUboot's SHA-256 validator, comparing the header+payload hash with the trailing hash TLV. Header size, image flags, TLV bounds, stack pointer, and reset vector must be valid. Protected TLVs, encryption, and compressed/other flagged images are unsupported.

Only after validation succeeds does the device program the complements of length and generation in the second half of the bank record and report state 2. END success confirms the uploaded image's integrity and activation record; it does not confirm that the application has run. An identical END CRC can be retried once complete. CRC checks transport; SHA checks flash contents. Neither authenticates firmware in this unsigned configuration.

RESET is accepted in any state. The device replies and resets only after reply transmission finishes. At each boot, `boot_select()` validates both committed images and selects the highest intact generation. A damaged latest image falls back to the retained valid bank; two invalid images result in CAN recovery. Bank 2 is remapped at handoff so the same binary linked at `0x08010200` runs in either physical bank. No image copying or MCUboot pending/swap trailer is used. Bank records occupy the final 16 bytes of each slot; the entire final 2 KiB page is excluded from uploads.

The host uses one-second command deadlines, 20 seconds for BEGIN/END, and up to three attempts; RESET gets one attempt. Accepted commands refresh the device's activity timer. Receiving/committed states reset after 30 seconds without a successful command. Upload progress lives in RAM: after reset restart from BEGIN. An incomplete upload has no valid commit record and is ignored. A completed committed image activates after reset.
