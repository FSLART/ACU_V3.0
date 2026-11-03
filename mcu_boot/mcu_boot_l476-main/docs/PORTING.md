# Port this bootloader to another STM32

**Follow the steps in order. Test each “Done when” checkpoint before continuing.** The goal is one application binary, one fixed application linker address, CAN programming and corruption fallback where the target supports it.

You build two separate programs: the **bootloader** receives and validates updates; the **application** is your normal firmware. The PC runs [can_upload.py](../tools/can_upload.py); see the [Python uploader guide](UPLOADER.md) for its use.

This guide explains how to adapt the project. Its existing binaries/register code are for the STM32L476RG. Other STM32s may need different flash operations, CAN APIs, memory mapping or an entirely different update strategy. There is no universal set of register values for every STM32.

## Start here: choose your route

| What changes? | Your route |
| --- | --- |
| Same STM32L476RG, different board/pins | Keep the memory layout if flash options match. Concentrate on steps 3, 6 and 9, then test the complete flow. |
| Different STM32 with suitable bank remapping | Follow every step. Replace the flash driver and final remap code. |
| No suitable bank remapping | Step 2 explains alternatives; the current selector cannot be reused unchanged. |
| Only need application-to-bootloader entry | Go to [step 9](#step-9--add-requestbootloader-to-the-application). |

**Configure a new target's application linker once. You do not change it for every subsequent bank-alternating update.**

For the detailed application-project procedure, see [Configure a new application's linker](APPLICATION_LINKER.md): address/size calculations, CubeIDE settings, RAM/stack reservation, vectors and binary checks.

## Navigate the steps

| Phase | Steps |
| --- | --- |
| Plan | [1: Target worksheet](#step-1--fill-in-the-target-worksheet), [2: Update strategy](#step-2--choose-a-supported-update-strategy) |
| Create project | [3: CubeMX and UART](#step-3--generate-the-target-project-and-bring-up-uart) |
| Prepare storage | [4: Layout/linkers](#step-4--draw-the-layout-and-configure-linkers-once), [5: Flash/records](#step-5--implement-flash-operations-and-commit-records) |
| Connect and run | [6: CAN/UART/button](#step-6--adapt-can-uart-and-the-button), [7: Handoff](#step-7--adapt-jump-and-bank-remapping), [8: Python](#step-8--adapt-the-python-target-assumptions) |
| Application entry | [9: RequestBootloader and ENTER_BOOT](#step-9--add-requestbootloader-to-the-application) |
| Verify | [10: Build/tests](#step-10--build-and-check-the-port), [11: Board checks](#step-11--test-the-board-in-this-order) |

## Terms in plain language

| Term | Meaning |
| --- | --- |
| Flash bank | A physical part of the MCU's flash memory. |
| Image slot | Space reserved for one packaged application. |
| Raw `.bin` | Application code starting with its vector table. |
| Packaged image | Header + application + hash information; the script creates it. |
| Vector table | Startup and interrupt-handler addresses. |
| Logical address | Address the application was built to use. |
| Bank remapping | Hardware makes a physical bank appear at the same logical address. |
| Erase sector/page | Smallest flash region erased in one operation. |
| Programming unit | Size/alignment required for a legal flash write. |
| Commit record | Information written last to mark an upload ready to boot. |

## How the current project works

```text
Application running
  -> PC sends ENTER_BOOT (0x601), BCI_BOOT=1
  -> application writes a RAM request and resets
  -> bootloader sees request and waits for programming
  -> PC uploads into the inactive physical bank
  -> bootloader checks CRC, vectors and SHA, then commits
  -> PC requests reset
  -> bootloader selects newest valid committed image
  -> SRAM code remaps chosen bank and starts application
```

There is no image copy between banks. If the newest image is damaged, the older valid image runs **at the next reset**. Two invalid images lead to recovery mode. A hash-valid program that hangs needs a separate watchdog/health-confirmation policy.

MCUboot provides the image format and validator. The current flow uses its own `boot_select()` and records, not `boot_go()` or MCUboot pending/confirmation APIs. `MCUBOOT_OVERWRITE_ONLY` remains for ST validator/trailer geometry; it does not mean uploads overwrite the selected running bank.

## Step 1 — Fill in the target worksheet

Use the **datasheet, reference manual and errata for the exact chip and silicon revision**. A family name alone is insufficient.

| Find this | Your target | L476RG reference |
| --- | --- | --- |
| Exact chip/board | | STM32L476RG, current PCB |
| CPU/FPU/cache features | | Cortex-M4, hard-float build |
| Total flash | | 1 MiB |
| Physical bank boundaries | | Two 512 KiB banks; bank 2 at `0x08080000` |
| Erase geometry in selected option-byte mode | | Uniform 2 KiB pages |
| Programming unit/ECC rules | | 8-byte doublewords |
| Remap mechanism/reset mapping | | Runtime FB_MODE; normal mapping at bootloader entry |
| SRAM addresses/reset retention | | SRAM1 `0x20000000`, SRAM2 `0x10000000` |
| CAN peripheral/clock | | bxCAN, 80 MHz PCLK1 |
| CAN rate / logging | | 250 kbit/s / USART2, 115200 8N1 |
| Button pressed/released levels | | PC13, current PCB treated as active-low |
| Special requirements | | Protection/watchdog; other targets may have TrustZone or another core |

The reference L476 flash checks require **DUALBANK=1, BFB2=0, FB_MODE=0 on bootloader entry**. These option/register names are not universal.

Keep ST-LINK/SWD recovery available. Read existing option bytes before changes; follow the target manual for transitions that erase flash or change geometry.

**Done when:** you know the real flash/RAM map, legal erase/write operations, reset mapping and CAN hardware.

## Step 2 — Choose a supported update strategy

| Target capability | Route | One identical application binary? |
| --- | --- | --- |
| Two banks can appear at the same execution address; SRAM handoff supported | Port current A/B remapping | Yes, with symmetric slots/startup preserving mapping |
| Selection requires option-byte reload/reset | Design persistent selection/reset and bootloader placement | Potentially; current runtime trampoline is not that implementation |
| Two slots without suitable remapping | Install/swap into a fixed execution slot | Yes, using a verified installer replacing the selector |
| No remapping; separate images acceptable | Link an image for each execution slot | No; packaging cannot relocate fixed addresses |
| One bank with room for staging | Interruption-safe installation into a fixed slot | Yes, but install/metadata logic changes |
| Insufficient space for two images | External staging, smaller image or recovery redesign | Rollback requires space for the old image |
| No integrated CAN | External controller or different transport | Replace transport on MCU and PC |

Two banks alone do not make a binary executable from both. ST describes remapping/SRAM execution in [AN4767](https://www.st.com/resource/en/application_note/an4767-onthefly-firmware-update-for-dual-bank-stm32-microcontrollers-stmicroelectronics.pdf); use your target manual for exact behavior.

If changing strategy, later steps still identify what to port, but current records/selection are not a ready-made solution. The [historical swap notes](../STM32L476_MCUboot_Swap_Migration.md) are an alternative design, not a tested one-macro conversion.

**Done when:** you know where the bootloader starts after reset and how both images reach the addresses they were built to use.

## Step 3 — Generate the target project and bring up UART

1. Create a CubeMX/CubeIDE project for the exact target; keep the L476 project as reference.
2. Configure clocks, SWD, UART, CAN/transceiver pins, LED and optional button.
3. Generate correct HAL/device headers, startup, interrupts and linker script.
4. First run a simple UART/LED program to prove hardware setup.
5. Copy `boot_flash.c`, `boot_port.c`, `boot_update.c`, generated `bootloader.c` and their headers into user-owned folders.
6. Add this project's ST MCUboot and SHA-only mbed TLS sources/includes; preserve licenses/integration fixes. Keep the vendor folder structure: checked-in code uses relative include paths, so moving folders also requires fixing those includes.
7. Call the ported `boot_run()` after peripheral initialization in a CubeMX USER CODE section.

For bxCAN, the existing call is:

```c
/* USER CODE BEGIN 2 */
boot_run(&hcan1);
/* USER CODE END 2 */
```

For FDCAN, change `boot_run()`'s declaration/definition to the target handle type before using its handle. Renaming alone is insufficient.

### Exact files to review

| File/settings | Adapt |
| --- | --- |
| [boot_port.h](../Core/Inc/boot_port.h), generated `main.h`/interrupt headers, [cmsis.h](../Core/Inc/cmsis.h) | Target HAL/device includes, pins and handle types |
| Generated `main.c`, MSP/interrupt sources | Clocks, alternate functions, initialization and handlers |
| Target `system_*.c` and startup `.s` | Target reset/vector/clock implementation |
| [region_defs.h](../Core/Inc/mcuboot_config/region_defs.h), [mcuboot_config.h](../Core/Inc/mcuboot_config/mcuboot_config.h) | Layout, RAM ranges and library options |
| [boot_flash.c](../Core/Src/boot_flash.c) | Flash controller and writable bounds |
| [boot_update.c](../Core/Src/boot_update.c) | Buffer, record and validation assumptions |
| [boot_port.c](../Core/Src/boot_port.c) | CAN/UART/button and handoff |
| Bootloader/application linkers | Flash/RAM bounds, stack alignment, RAM code placement |
| [CMakeLists.txt](../CMakeLists.txt), CubeIDE settings | Sources/includes, device macro, CPU/FPU flags and linker paths |
| [can_upload.py](../tools/can_upload.py), [DBC](bootloader.dbc), tests | Matching addresses, limits, IDs, rate and fixtures |

Replace L4 HAL/CMSIS and `STM32L476xx` with the new device's equivalents. Use target-generated CPU/FPU flags consistently at compilation/linking; do not copy Cortex-M4 hard-float settings to an incompatible core. Update CMake and CubeIDE independently if using both.

**Done when:** the new project builds and logs are readable on the board.

## Step 4 — Draw the layout and configure linkers once

For current A/B remapping, slots must have equal sizes and the same offset within each bank:

```text
Physical bank 1: [bootloader/reserved B][application slot S]
Physical bank 2: [reserved area B     ][application slot S]
```

Bootloader starts in physical bank 1. This design does not put a second bootloader copy in bank 2.

### Worked L476 example

| Region | Physical start | Size |
| --- | --- | --- |
| Bootloader | `0x08000000` | `0x10000` = 64 KiB |
| Bank-1 slot | `0x08010000` | `0x70000` = 448 KiB |
| Bank-2 reserved | `0x08080000` | `0x10000` = 64 KiB |
| Bank-2 slot | `0x08090000` | `0x70000` = 448 KiB |

With the 512-byte header, the logical application vector address is `0x08010200`. Bank-2 vectors are physically `0x08090200`, mapped to that logical address when selected.

For uniform sectors and the current unsigned format:

```text
H = header size                  (current: 512)
T = complete hash TLV block       (current: 40)
E = final sector for metadata    (current: 2048)
M = maximum packaged upload      = slot size - E
Maximum raw application          = M - H - T

L476: M = 0x70000 - 0x800 = 456704
Raw maximum = 456704 - 512 - 40 = 456152 = 0x6F5D8
```

Use the actual final sector for variable geometry. Keep final programming-unit padding before metadata. Signatures or new metadata need recalculated limits.

Update matching macros in **both** configuration headers: some definitions are duplicated. Check flash base, both slots, both vector addresses, header size, erase geometry, upload maximum, record offset and valid RAM ranges. Keep the ST scratch entry non-overlapping; current A/B does not write it.

Bootloader FLASH must fit its built image, including RAM-function load data. Application FLASH starts at the **logical vector address**, with length no greater than the raw limit. Do not link it at the slot header or bootloader origin.

For reference L476 application startup, define `USER_VECT_TAB_ADDRESS` and `VECT_TAB_OFFSET=0x10200U`. Use the new target's equivalent and verify startup preserves VTOR/mapping. Respect its vector alignment and an eight-byte-aligned initial stack. Reserve the RAM request flag as in step 9.

**Done when:** maps show no overlap; raw application starts with valid SP/Thumb reset vectors inside the logical payload.

## Step 5 — Implement flash operations and commit records

Replace L476 assumptions in `boot_flash.c`: capacity, bank split, option checks, page numbering, 8-byte programming and bounds.

| Operation | Required behavior |
| --- | --- |
| Initialize | Reject incompatible capacity/geometry/mapping; log reason |
| Read | Validate range, read bytes, report faults |
| Erase | Erase the intended physical sector; report failure |
| Program | Check range/alignment, write legal units, verify contents |
| GetInfo | Actual geometry, program alignment and erased value |

**ST's adapter expects 0 on success, including reads/writes.** Do not return a CMSIS-style byte count. Driver offsets are relative to `FLASH_BASE_ADDRESS`, with area offsets already added. Add the flash base once.

Keep bootloader/reserved regions unwritable; preserve unlock/lock errors and readback checks. Follow target voltage/timing, execution-stall, cache, watchdog and ECC rules. On applicable ECC flash, programming all-ones padding can consume ECC state even though it appears erased.

### Different-sized sectors

Current `flash_area_erase()` in [flash_map.c](../Drivers/stm32_mcu_boot/bl2/src/flash_map.c) only implements uniform sectors; its non-uniform branch is unimplemented. Setting `sector_info` alone is **not enough**.

Implement sector iteration/alignment/error handling, update uniform enumeration in [flash_map_legacy.c](../Drivers/stm32_mcu_boot/bl2/ext/mcuboot/flash_map_legacy.c), and adapt BEGIN's final-sector erase. Check sector-array capacity and `MCUBOOT_MAX_IMG_SECTORS` against the actual number of sectors per slot. Metadata erases must not cross into another region.

### Different programming units

`boot_update.c` receives 4 bytes per DATA frame but buffers/programs 8 bytes. Review `word[8]`, `& 7U`, `~7U`, final padding and write calls. Keep four-byte CAN chunks for protocol 1 and assemble the new hardware's legal programming unit.

Current records occupy the final 16 bytes: BEGIN programs length/generation in 8 bytes; END programs their complements in another 8 bytes. Adapt buffering and records together. Prepare and commit must use **separately programmable units**; never program an ECC/flash unit twice just because different bytes change.

ST's secondary validator checks erased padding until the last 16 bytes. Larger/relocated records can conflict with it. Reconcile its padding boundary with the new layout while retaining SHA, TLV and bounds validation.

**Done when:** inactive-slot erase/write/readback preserves selected image/bootloader, and partial or failed commit writes cannot activate.

## Step 6 — Adapt CAN, UART and the button

Current MCU and script both use **250 kbit/s**. The bxCAN reference timing is:

```text
80 MHz / (prescaler 20 * (1 + BS1 13 + BS2 2)) = 250 kbit/s
```

Use actual peripheral clock, not assumed CPU frequency. For FDCAN, inspect kernel clock/divider. Verify oscillator tolerance, sample point and real bus timing.

| Existing bxCAN piece | FDCAN porting task |
| --- | --- |
| Handle/RX/TX headers | Target FDCAN types |
| Filter/start | Standard-ID filtering, unwanted/remote-frame rejection and controller start |
| FIFO polling/read | FDCAN APIs; normalize ID, frame type and actual length |
| `rx.DLC` | Convert HAL FDCAN DLC encoding to bytes before `boot_update_frame()` |
| Mailbox/completion/abort | Enqueue, completion, timeout/error handling with target APIs |
| Wire format | Classic CAN, standard ID, 8 bytes, no FD/BRS |

Use installed HAL examples for exact APIs; FDCAN is not a handle rename. See ST's [AN5348 overview](https://www.st.com/content/st_com/en/technical-documents/an5348-introduction-to-fdcan-peripherals-for-stm32-mcus-stmicroelectronics.html).

Keep stop-and-wait acknowledgements; acknowledge RESET before resetting. Measure flash/bus delays against MCU's 100 ms reply timeout, host deadlines and 30-second session timeout.

| ID | Direction | Meaning |
| --- | --- | --- |
| `0x602` | PC -> bootloader | Programming commands |
| `0x580` | Bootloader -> PC | ACK/status |
| `0x601` | PC -> application | `ENTER_BOOT.BCI_BOOT=1` |
| `0x581` | Bootloader -> other boards | `BCI_BOOT_HEARTBEAT`: `0x55`, millisecond tick and mandatory bootloader version |

Structured diagnostics additionally use `0x582`–`0x589`; see the [diagnostic layouts](CAN_PROTOCOL.md#can-diagnostics). Set `BOOT_VERSION_MAJOR/MINOR/PATCH`, `BOOT_BOARD_TYPE` and `BOOT_PROTOCOL_VERSION` in the configuration header. Version snapshots and HAL error details depend on this port; adapt reset flags, bank identification and the CAN drain hook (`boot_diag_flush`) along with flash/CAN code.

Reserve these IDs. Other traffic is allowed; collisions and multiple targets sharing these IDs are unsupported. Change IDs/signals in the DBC, regenerate `bootloader.h`/`bootloader.c`, then rebuild MCU and application. Python loads the saved DBC automatically. See [codec regeneration](CAN_PROTOCOL.md#dbc-and-generated-c-codec). Keep heartbeat/diagnostic IDs reserved too.

Use a suitable external transceiver, its correct enable state, common ground and proper bus termination. MCU TX/RX are not CANH/CANL. Start with one adapter/target before busy-bus testing.

The heartbeat is best-effort every 500 ms and reserves a TX mailbox for replies. Adapt its HAL calls when porting to FDCAN; blocking flash operations can delay it.

Replace `huart2`/generated setup for your target and retain log flushing before UART shutdown. Measure button pin pressed/released levels and update GPIO pulls plus every hold test consistently. Current PCB treats low as pressed; another board may differ. Provide another recovery-entry mechanism if there is no button.

**Done when:** INFO replies, logs work and releasing the button allows a valid application to start.

## Step 7 — Adapt jump and bank remapping

Edit `jump()`/`enter_app()` in `boot_port.c`. Current trampoline has literal **`0x40010000`** for L476 SYSCFG MEMRMP and uses `SYSCFG_MEMRMP_FB_MODE`. Reset flags, flash-cache bits and bank checks are also L476-specific. Replace them from the target manual.

- [ ] Validate SP/PC/image bounds before changing state.
- [ ] Save SP/PC from selected physical image before remapping.
- [ ] Print physical bank, then flush UART.
- [ ] Stop bootloader CAN/UART/DMA activity as required.
- [ ] Disable/clear interrupts and SysTick using actual target interrupt capacity; do not assume current eight NVIC register groups.
- [ ] Apply target-required cache/prefetch maintenance, barriers and clock rules.
- [ ] Install aligned logical application vector-table address.
- [ ] Remap entirely from SRAM, including literal pools and called functions.
- [ ] Set application MSP/CONTROL and branch to Thumb reset handler without returning.
- [ ] Ensure application startup does not undo mapping or restore bootloader vectors.

Linker must place `.RamFunc` in executable SRAM with a flash load address; startup must copy it. Section attribute alone is insufficient. Inspect ELF/disassembly: trampoline **and literals** must be in SRAM.

If no usable VTOR exists, implement the target vector-remapping mechanism. For TrustZone, implement secure/non-secure handoff and memory attribution. For multi-core chips, coordinate the other core/shared peripherals. The L476 trampoline does not supply those implementations.

Update MCU `boot_image_vectors_valid()` and Python checks for real RAM/execution ranges. Physical bank-2 firmware in this strategy still contains a logical bank-1 reset address.

**Done when:** small known-good apps start from each physical bank after separate resets, with interrupts working.

## Step 8 — Adapt the Python target assumptions

The script is not fully MCU-independent. Device INFO does not override earlier host-side L476 checks.

| Location in `can_upload.py` | Change/review |
| --- | --- |
| `prepare_image()` | RAM ranges, logical application address, header size, packaged maximum and vectors |
| `check_image()` | Accepted header/TLV format if changed; retain validation |
| DBC / `can.Bus()` | IDs/layout come from the DBC; current rate is hardcoded `250000` |
| `request()` / `upload()` | Deadlines if measured delays require changes |
| DBC/protocol/tests | Matching fields, limits, rate, addresses and fixtures |

Packaging cannot relocate a wrong-address binary. Keep one fixed application origin for compatible A/B remapping. Current generated header version is 1.0.0; selection uses upload generation, not version number.

**Done when:** valid target images pass host checks, wrong-address/corrupt/oversized images fail, and MCU/host size limits agree.

## Step 9 — Add RequestBootloader to the application

**Put this function in the application project.** It resets into this custom bootloader, not ST's factory ROM bootloader.

### 9.1 Reserve the RAM flag

Use the same address/magic in the application and bootloader's `BootloaderRequested()`. The word must survive software reset and be excluded from `.data`, `.bss`, heap, stack, startup clearing and RTOS allocation in **both linkers**. `volatile` alone does not reserve memory.

The L476 flag is `0x20017FFC`, in the final eight bytes of its 96 KiB SRAM1. One application-linker arrangement is:

Replace the SRAM1 entry **inside your existing `MEMORY { ... }` block**:

```ld
/* L476 SRAM1 example: reserve last 8 bytes, including request word. */
RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 0x17FF8
```

Set the stack symbol **outside that block**, keeping the rest of the linker script:

```ld
_estack = ORIGIN(RAM) + LENGTH(RAM);
```

This keeps initial stack eight-byte aligned. Existing bootloader linker reserves just the final word using length `0x17FFC`; check stack alignment separately when creating new linkers. Blindly using that length for an application's `_estack` produces a stack pointer rejected by the current validator.

Check target SRAM retention/reset clearing. With a write-back data cache, `volatile` plus `__DSB()` does not flush a dirty line to physical SRAM: use a non-cacheable reserved location or target-correct cache cleaning before reset, with appropriate cache-line reservation/alignment. See [ST AN4839 on cache maintenance](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf) for F7/H7 examples.

### 9.2 Copy this function

Place it in application `/* USER CODE BEGIN 0 */` or a separate application-owned `.c` file. This simple example matches current L476; change address/cache handling when porting.

```c
#include "main.h"  /* Application's target HAL/CMSIS declarations. */
#include <stdint.h>

#define BOOTLOADER_FLAG_ADDR  0x20017FFCUL
#define BOOTLOADER_MAGIC      0xB00710ADUL

void RequestBootloader(void)
{
    volatile uint32_t *flag =
        (volatile uint32_t *)BOOTLOADER_FLAG_ADDR;

    *flag = BOOTLOADER_MAGIC;

    __DSB();
    NVIC_SystemReset();

    while (1) {}
}
```

Call `RequestBootloader();` from a menu/button action or received CAN command. If another `.c` calls it, put `void RequestBootloader(void);` in an application header.

Sequence: **write flag -> reset -> bootloader consumes flag -> bootloader waits**. Releasing the button does not override the RAM request. After programming, script requests a second reset; without a new RAM request, normal application selection resumes.

### 9.3 Call it when ENTER_BOOT arrives

Script sends one standard frame on `0x601`, DLC 8:

```text
01 00 00 00 00 00 00 00
```

`BCI_BOOT` is unsigned Intel bit 0, length 1, factor 1, offset 0, range 0-1. **Application** must start CAN at matching rate and accept this ID. Bootloader's `0x602` filter does not provide an application handler.

Copy generated `Core/Inc/bootloader.h` and `Core/Src/bootloader.c` into the application and compile the source. For a bxCAN application, put this in the receive loop after CAN initialization/start:

```c
#include "bootloader.h"

if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0U)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    struct bootloader_enter_boot_t entry;

    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, data) == HAL_OK &&
        rx.IDE == CAN_ID_STD && rx.RTR == CAN_RTR_DATA &&
        rx.StdId == BOOTLOADER_ENTER_BOOT_FRAME_ID && rx.DLC == 8U &&
        bootloader_enter_boot_unpack(&entry, data, rx.DLC) == 0 && entry.bci_boot == 1U)
    {
        RequestBootloader();
    }
}
```

Integrate into an existing dispatcher if it already drains the FIFO; two consumers can steal each other's frames. Preserve other application filters/handlers. For FDCAN, use its headers/APIs and convert DLC before checking length.

No entry ACK is defined; normal bootloader INFO confirms entry. This repository does not automatically add the handler to an external application.

**Done when:** `0x601` causes reset and logs show `RAM magic=1` / `Application held: RAM bootloader request`.

## Step 10 — Build and check the port

After adapting target sources, flags and linker paths, run from project root:

```powershell
cmake -S . -B build-port -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build-port --target bootloader
.venv\Scripts\python.exe -m unittest discover -s tests -v
```

See [uploader setup](UPLOADER.md#1--one-time-setup) if `.venv` does not exist. Build intended CubeIDE configuration too; inspect maps for flash/RAM bounds, stack and RAM code. Adapt tests to new geometry, programming units, vectors and records. `test_boot.c` has hardcoded L476 values as well as configuration macros.

Reference QEMU test:

```powershell
cmake --build build-verified --target boot_tests
qemu-system-arm -M mps2-an386 -nographic -semihosting -kernel build-verified/boot_tests.elf
```

This models shared logic on an M4 machine, not real flash/CAN/remapping/options/retention. New CPU/TrustZone builds may need a separate compatible test configuration. Do not treat this M4 simulation as verification of another target's hardware.

**Done when:** intended builds succeed and software tests cover changed state/geometry logic. Compilation alone does not finish the port.

## Step 11 — Test the board in this order

Use small distinguishable apps A/B, such as different LED patterns. Record results.

| Test | Action | Expected result |
| --- | --- | --- |
| Logs | Program bootloader only | Banner/flash checks visible |
| Empty slots | Boot without valid image | Recovery; no invalid jump |
| CAN | Hold entry and send INFO | Correct reply ID/version/maximum |
| First upload | Upload A's raw `.bin` | Conversion/progress/validation/commit succeed |
| First boot | Release button/reset | Bank printed; A and interrupts work |
| Application entry | Run script while A runs | `0x601` -> flag/reset -> INFO |
| Second upload | Upload B | Other physical bank runs B |
| Alternation | Upload A again | Bank changes; linker stays fixed |
| Corruption fallback | With both valid, corrupt only newest test payload/reset | Retained image runs; rollback logged |
| Both invalid | Invalidate both | Recovery remains available |
| Bounds | Try wrong vectors/oversized image | Rejected; bootloader intact |
| Maximum size | Upload valid near-limit image | No payload/metadata overflow; both handoffs work |
| Busy CAN | Add representative unrelated traffic | Correct filtering/ACKs/retries |
| Cold start | Power-cycle with each committed bank | Correct selection/mapping |

Interrupt power at several points in inactive erase, record preparation, payload programming and commit. Selected valid bank should survive; partial records must not activate. Inspect flash after reconnecting. Next BEGIN replaces the previous fallback slot while preserving the selected image.

For corruption tests, ensure programmer only modifies the intended test sector; mass erase destroys the fallback. Physical ECC faults may cause exception/NMI rather than normal hash rejection. Current L476 NMI handler does not recover from them; implement/test a target-specific policy if required.

**Done when:** both bank boots, entry, bounds, interrupted updates and recovery pass on actual hardware.

## Troubleshooting

| Symptom | First checks |
| --- | --- |
| No logs | UART pins/baud, clock/init, `__io_putchar()` |
| Flash setup rejected | Exact part capacity, options, mapping/geometry |
| App runs only with button pressed | Inverted hold polarity |
| No entry after `0x601` | App handler/filter, bitrate, flag reservation/retention/startup clearing |
| INFO works, BEGIN fails | Protection, alignment, slot/metadata bounds |
| Only one bank boots | Symmetric offsets, remap, SRAM code, logical vectors/caches |
| First interrupt faults | VTOR/startup, pending interrupts, alignment |
| Host SHA passes, MCU fails | Readback/ECC, padding/TLV bounds, record overlap/caches |
| Interrupted upload activates | Commit order/torn-record rejection/erase overlap |
| CMake works, CubeIDE fails | Independent flags/sources/includes/linker settings |
| Hash-valid app hangs | Current selector has no runtime-health policy; design watchdog/test-boot confirmation |

## Completion checklist

- [ ] Exact chip/revision documented; manuals/errata checked.
- [ ] Strategy supported by target hardware.
- [ ] Bootloader/slots/records meet real erase/program boundaries.
- [ ] Application origin/vectors/stack correct.
- [ ] Both linkers reserve flag; reset/startup/cache behavior preserves it.
- [ ] Flash return conventions and two-phase commit correct.
- [ ] CAN/UART/button/application entry work.
- [ ] Script/DBC/tests match addresses, limits, IDs/rate.
- [ ] Both bank handoffs, interruption and corruption recovery pass on hardware.
- [ ] Recovery/SWD access retained.

## Security and library limits

Current images are unsigned. CRC checks transport; SHA checks against the supplied hash, which someone replacing an image can recompute. `ENTER_BOOT` has no authentication. Generation orders uploads; it is not an anti-rollback security counter.

For authenticated firmware, add the signature backend/public key, matching packaging, verification policy and tests. The [bundled SHA subset](../Drivers/mbedtls_sha256/README.md) is not a signature library. Recalculate TLV/payload limits and plan key/bootloader protection. One macro does not supply those features.

Preserve ST integration fixes: feature-gated includes, padding/trailer checks, sector-array capacity handling and unsigned packaging. [Upstream MCUboot porting](https://docs.mcuboot.com/PORTING.html) explains flash areas; this repository's ST interfaces/custom records are the implementation authority. Vendor updates require a separate migration.

## References

Use the target's own datasheet/reference/programming manual and errata for register-level decisions.

- [ST RM0351 — reference L476 flash/peripheral manual](https://www.st.com/resource/en/reference_manual/DM00083560.pdf).
- [ST AN4767 — dual-bank/SRAM execution concepts](https://www.st.com/resource/en/application_note/an4767-onthefly-firmware-update-for-dual-bank-stm32-microcontrollers-stmicroelectronics.pdf).
- [ST AN5348 — FDCAN overview](https://www.st.com/content/st_com/en/technical-documents/an5348-introduction-to-fdcan-peripherals-for-stm32-mcus-stmicroelectronics.html).
- [Upstream MCUboot porting](https://docs.mcuboot.com/PORTING.html) and [ST source](https://github.com/STMicroelectronics/stm32-mw-mcuboot): match interfaces to checked-in version.
