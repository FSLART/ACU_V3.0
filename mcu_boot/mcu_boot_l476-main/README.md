# STM32L476RG A/B CAN bootloader

The bootloader uses both physical flash banks for alternating application updates, with SHA-256 validation and fallback to the retained valid image. MCUboot supplies the image format and validator; `boot_select()` selects the bank instead of MCUboot's overwrite installer. Firmware is unsigned: SHA-256 detects corruption but does not authenticate its author.

## Guides

- **[Port to another STM32](docs/PORTING.md)** — target worksheet, strategy choices, exact files to edit, numbered steps and hardware checkpoints. Includes the application `RequestBootloader()` function and CAN entry example.
- **[Configure a new application's linker](docs/APPLICATION_LINKER.md)** — calculate flash origin/size, reserve the RAM request, configure vectors, and verify a normal CubeIDE `.bin`. Includes generic rules and the current L476 example.
- **[Use and understand the Python uploader](docs/UPLOADER.md)** — setup, direct CubeIDE `.bin` upload, automatic conversion, entry command, progress, retries and troubleshooting.
- **[CAN protocol](docs/CAN_PROTOCOL.md)** and **[DBC](docs/bootloader.dbc)** — message and signal definitions, [cantools regeneration and encoding/decoding examples](docs/CAN_PROTOCOL.md#dbc-and-generated-c-codec).

## Build and program the bootloader

```powershell
cmake -S . -B build-verified -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build-verified --target bootloader
```

CMake, Ninja and Arm GNU tools must be on PATH. CubeIDE can also build the existing Debug project. Program `build-verified/bootloader.elf` or `.hex` with ST-LINK; program a raw bootloader `.bin` at `0x08000000`. The optional CMake demo target is created only when `Application/main.c` exists.

The MCU must have 1 MiB flash, **DUALBANK=1, BFB2=0**, and boot from physical bank 1 with normal bank mapping. Firmware checks these requirements and logs an error rather than using incompatible erase geometry. It does not change option bytes.

## Pass the application binary directly to the uploader

Build your CubeIDE **application** with its vector table at **`0x08010200`**. Keep that same linker address for every update, regardless of which bank will run. Its startup must retain `VTOR=0x08010200`, for example with `USER_VECT_TAB_ADDRESS` and `VECT_TAB_OFFSET=0x10200U`. The application must not undo flash bank remapping.

A raw binary linked at `0x08000000` cannot be relocated just by adding a header. The uploader rejects invalid stack/reset vectors before contacting the device. Maximum unsigned application payload: **456152 bytes** (`0x6F5D8`), allowing for the header, SHA TLV and reserved metadata page. An existing compatible application linker script needs no change for bank alternation.

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r tools/requirements.txt
.venv\Scripts\python.exe tools/can_upload.py path/to/CubeIDE_application.bin --channel COM5
```

The script creates `CubeIDE_application.mcuboot.bin` beside the input, adding a 512-byte MCUboot header (version 1.0.0) and a SHA-256 TLV, then uploads that converted image. It also accepts already packaged, unpadded MCUboot images. The programming progress bar displays acknowledged bytes and reaches 100% at the end of transfer; validation/commit is a separate step reported afterward. Reset activates the new bank.

The default adapter interface is SLCAN, with classic CAN at **250 kbit/s**, request ID `0x602` and reply ID `0x580`. For SocketCAN use `--interface socketcan --channel can0` with a preconfigured 250 kbit/s interface. Only connect one target using these fixed IDs.

The uploader first sends `ENTER_BOOT` on CAN ID `0x601` with `BCI_BOOT=1`. The running application must handle that frame by setting the RAM bootloader request and resetting. If it lacks that handler, hold the PCB B1 button during manual reset to stay in the bootloader. The current PCB behavior is treated as **active-low**: pressed reads PC13 low. Release it after upload so the application can start. A RAM request of `0xB00710AD` at `0x20017FFC` also keeps the bootloader active until reset; the request is consumed once. If neither image is valid, it waits for recovery uploads without needing the button.

## Flash layout and bank selection

| Region | Physical start | Size |
| --- | --- | --- |
| Bootloader, bank 1 | `0x08000000` | 64 KiB |
| Application slot, bank 1 | `0x08010000` | 448 KiB |
| Reserved, bank 2 | `0x08080000` | 64 KiB |
| Application slot, bank 2 | `0x08090000` | 448 KiB |

Both slots have the same **offset within their bank**. The bank-2 vector table is physically at `0x08090200`, but bank remapping makes it execute at the existing logical address `0x08010200`. The handoff trampoline executes from the linker's existing `.RamFunc` section in SRAM; linker files are unchanged by this migration. The bootloader always starts in physical bank 1 after reset. There is no second bootloader copy and no firmware copy/swap between banks.

At reset, the bootloader checks both commit records, headers, vectors and SHA hashes and selects the highest valid upload generation. Version metadata does not control update order. If the latest committed image is invalid, the retained valid bank runs. If neither is valid, CAN recovery remains available. An existing bank-1 MCUboot image with an erased record is accepted as generation zero for migration. Old secondary-slot images at `0x08080000` are not selected by this new layout; upload them again through the script.

BEGIN erases only the inactive slot, invalidating its record first. It prepares image length and generation in the final 16 bytes of that slot. END checks transport CRC, rereads and validates the image, then programs the complementary length/generation in the second doubleword to commit it. Interrupted transfers and partial records are ignored at boot. Duplicate DATA/END retries never program an ECC doubleword again. Once committed, reset before starting another upload. A receiving or committed session resets after 30 seconds without a successful command.

Only two images are retained: starting the next upload replaces the previous fallback bank while preserving the selected running image. Rollback requires the retained bank to remain valid. This is corruption fallback at reset, not application health confirmation: a hash-valid image that hangs needs a watchdog/reset and a separate health-confirmation policy. Uncorrectable physical flash ECC faults and bank remapping still require board validation.

## Bootloader heartbeat

While in bootloader mode, the board sends **`BCI_BOOT_HEARTBEAT` on `0x581`** every 500 ms under normal conditions: `55 tick0 tick1 tick2 tick3 major minor patch`. Bytes 1-4 carry the unsigned 32-bit `HAL_GetTick()` millisecond value in little-endian order, which resets on MCU startup and wraps modulo 2^32. Bytes 5-7 always carry the bootloader version (currently 1.0.0, configured in `mcuboot_config.h`). It is a mode indicator, not an upload acknowledgement, and stops at application handoff. Flash operations can delay it. The DBC's `0xAA = APP` value requires an application-side sender.

## CAN diagnostics

The [DBC](docs/bootloader.dbc) now defines structured diagnostic frames on **`0x582`–`0x589`**: capabilities/board type, selected application version, bank decision/reset flags, upload stage/counts, last error, rollback reason/rejected generation, flash offset/HAL error and rejected image version. Details and numeric labels are in the [CAN protocol guide](docs/CAN_PROTOCOL.md#can-diagnostics).

Bootloader version is mandatory in the heartbeat, sent immediately after CAN starts, periodically, and after successful INFO replies. Diagnostic snapshots send at startup and after INFO; upload stages, 16 KiB progress and errors update on change. They use best-effort transmission with one mailbox reserved for replies and a bounded 20 ms drain before blocking phases/handoff. Congestion can coalesce changes; these frames do not replace programming ACKs.

## Logs and checks

Connect the ST-LINK virtual COM port at **115200 baud, 8N1**. INFO-level logs cover reset flags, flash configuration, validation of each bank, generation/version, selected bank, hold reasons, CAN startup, upload progress every 16 KiB, CRC/SHA failures, flash failures and resets. Immediately before UART shutdown the bootloader prints **`Boot bank 1`** or **`Boot bank 2`**, VTOR, stack pointer and reset handler. `MCUBOOT_LOG_LEVEL` in `Core/Inc/mcuboot_config/mcuboot_config.h` controls verbosity. Logging is blocking; successful DATA frames are not individually logged.

```powershell
.venv\Scripts\python.exe -m unittest discover -s tests -v
cmake --build build-verified --target boot_tests boot_port_tests
qemu-system-arm -M mps2-an386 -nographic -semihosting -kernel build-verified/boot_tests.elf
qemu-system-arm -M mps2-an386 -nographic -semihosting -kernel build-verified/boot_port_tests.elf
```

QEMU tests the real MCUboot validator against modeled flash: bank alternation, corruption fallback, incomplete transfer/commit, record corruption, legacy primary provisioning, bounds, duplicate frames, CRC failures and flash-operation errors. It does not emulate L476 flash hardware, CAN, PC13 polarity or bank remapping. Test both bank handoffs, physical power interruption, maximum image size and rollback on the board before deployment.

## CAN hardware

Use an external 3.3 V-compatible CAN transceiver: PA12 to TXD, PA11 to RXD, common ground, and CANH/CANL to the adapter. STM32 pins are logic signals, not CANH/CANL. Enable the transceiver and terminate the bus with 120 ohms at each physical end. The current 80 MHz PCLK1, prescaler 20, BS1 13 and BS2 2 produce 250 kbit/s; verify timing on hardware.

The board has no firmware authentication or node addressing. CAN `ENTER_BOOT` requires support in the running application; reset/B1 or the application RAM request provide bootloader entry. Details: [CAN protocol](docs/CAN_PROTOCOL.md), [porting](docs/PORTING.md). `STM32L476_MCUboot_Swap_Migration.md` describes an older alternative swap design, not the implemented A/B selector.
