> Historical alternative: this document describes a swap-based design. The current implementation instead uses symmetric A/B slots, SRAM bank remapping and integrity fallback; see README.md.

# STM32L476RG CAN Bootloader: Migrate MCUboot Direct-XIP to Swap with Rollback

## Goal

**Build one application binary, with a fixed linker script, for every future firmware update.** The CAN uploader writes a new image into the secondary slot; MCUboot swaps it into the primary slot on reboot. The application always executes at the primary-slot address. If a test image is not confirmed, MCUboot can revert to the previous image on a subsequent reset.

This is a **migration plan**, not a drop-in set of defines. The MCUboot version, ST flash-map backend, CAN update code, flash erase/program implementation, and trailer layout must be checked before enabling swaps on hardware.

## 1. Existing project and observed failure

- MCU: STM32L476RG, 1 MiB internal flash, 2 KiB erase pages (verify option-byte bank configuration).
- Bootloader: `0x08000000` to `0x0800FFFF` (64 KiB).
- Primary slot (MCUboot `FLASH_AREA_0`, ID 1): `0x08010000` to `0x0807FFFF` (448 KiB).
- Secondary slot (MCUboot `FLASH_AREA_2`, ID 2): `0x08080000` to `0x080EFFFF` (448 KiB).
- Scratch region currently declared: `0x080F0000` to `0x080F1FFF` (8 KiB).
- Image header: `0x200` (512 bytes).
- Current build: `MCUBOOT_DIRECT_XIP` and `MCUBOOT_DIRECT_XIP_REVERT`.

In the debugger, MCUboot selected the secondary slot (`rsp.br_image_off = 0x80000`), so the vector table was at `0x08080200`. The initial stack pointer was `0x20018000`, but the reset vector was `0x08010ACD`: **the binary in slot 1 had been linked for slot 0**. That is why Direct-XIP could not launch it correctly. Swap mode avoids this address mismatch by executing all images from slot 0 after swapping.

## 2. Target behavior

1. Build application **once** for primary-slot execution (`0x08010200`).
2. Package/sign the application with the MCUboot image header and integrity/authentication metadata expected by the bootloader.
3. Upload the packaged image to the **secondary** slot via CAN; do not overwrite the primary image.
4. Mark the secondary image as pending for a **test** upgrade using MCUboot's supported trailer/update API.
5. Reset. MCUboot validates and swaps images, then boots the new image from primary slot.
6. Application performs self-tests and **confirms** itself through MCUboot's confirmation API.
7. If it fails or resets before confirmation, MCUboot reverts to the previous confirmed image on the next eligible boot.

A reset alone is not an instant rollback; rollback depends on MCUboot's state/trailer logic, a valid retained old image, and the application not confirming prematurely.

## 3. Flash layout (retain these addresses, subject to backend verification)

| Region | Start | End inclusive | Size |
|---|---|---|---|
| Bootloader | `0x08000000` | `0x0800FFFF` | 64 KiB |
| Primary slot | `0x08010000` | `0x0807FFFF` | 448 KiB |
| Secondary slot | `0x08080000` | `0x080EFFFF` | 448 KiB |
| Scratch (currently declared) | `0x080F0000` | `0x080F1FFF` | 8 KiB |
| Other reserved flash | `0x080F2000` | `0x080FFFFF` | 56 KiB |

**Do not assume that 8 KiB scratch is sufficient.** Confirm the specific MCUboot swap implementation, sector constraints, erase-page sizes, and scratch status/trailer overhead. Swap algorithms differ; if scratch swap is unsupported or unsuitable, consider an available move/offset swap strategy and redesign the layout accordingly.

## 4. Application linker script: configure once

In the **application** project's `STM32L476RGTx_FLASH.ld`, use a fixed start address:

```ld
FLASH (rx) : ORIGIN = 0x08010200, LENGTH = 0x6FE00
RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 0x17FFC
```

- `0x08010200` = primary slot start + 512-byte MCUboot header.
- The `0x6FE00` FLASH length is only the **physical space after the header**, **not the safe maximum firmware payload**. Reserve room for MCUboot TLVs, swap trailer, status, and any required alignment/padding. Set a smaller validated build/package limit after calculating trailer size for the chosen MCUboot version and sector count.
- `0x17FFC` RAM length reserves the final SRAM1 word at `0x20017FFC` for the application-to-bootloader request flag. Verify this is consistent with startup code and both projects' linker scripts.
- Do **not** switch this linker origin to `0x08080200` when uploading to slot 1: the swap operation will relocate the image into slot 0 before execution.

In the application startup, ensure `SCB->VTOR` points to `0x08010200` (e.g., via the correct `system_stm32l4xx.c` vector table configuration). Check that Cube-generated startup code does not later reset VTOR to `0x08000000`.

## 5. MCUboot configuration

In `mcuboot_config.h`, **remove**:

```c
#define MCUBOOT_DIRECT_XIP 1
#define MCUBOOT_DIRECT_XIP_REVERT 1
```

For a MCUboot version that supports **scratch-based swap**, the intended strategy is typically:

```c
#define MCUBOOT_IMAGE_NUMBER 1
#define MCUBOOT_SWAP_USING_SCRATCH 1
#define MCUBOOT_USE_FLASH_AREA_GET_SECTORS 1
```

Keep the existing cryptographic backend and fault-injection-hardening configuration consistent with the image packaging process. Do **not** simultaneously enable overwrite-only or Direct-XIP. Confirm the exact macro names and supported strategy in **your vendored MCUboot sources** before applying these changes; some MCUboot versions use different configuration paths and some swap strategies are deprecated.

Keep address and size definitions in `region_defs.h` as the single source of truth; avoid duplicate `BOOT_IMAGE_HEADER_SIZE` or flash-region macros in `mcuboot_config.h`.

### Bootloader handoff

After a successful swap, `boot_go()` should return the **primary** image offset (`0x10000`). The existing `jump()` may still accept either offset for compatibility, but the normal swap-mode execution path should be primary-only. Validate:

- `rsp.br_image_off == 0x10000` after swap.
- Vector table = `0x08010200`.
- Initial stack pointer is within valid SRAM.
- Reset vector is an odd Thumb address within the primary application's flash region.
- Button `stay` polarity is correct: on the standard NUCLEO-L476RG, B1 is active-high (`GPIO_PIN_SET` when pressed), subject to actual board wiring.

## 6. Flash backend and MCUboot port: required checks

Inspect `boot_flash.c`, `flash_map.c`, `flash_map_extended.c` / `flash_map_legacy.c`, and the `bootutil` sources actually compiled:

- [ ] Flash-area IDs map primary, secondary and scratch correctly (`FLASH_AREA_0_ID`, `FLASH_AREA_2_ID`, `FLASH_AREA_SCRATCH_ID`).
- [ ] Read, erase and write work in **both internal flash banks** and scratch, with correct absolute/relative address handling.
- [ ] Flash erase-page enumeration is correct (`MCUBOOT_USE_FLASH_AREA_GET_SECTORS`).
- [ ] Write alignment and erased value match the STM32L4 flash implementation (double-word programming where required).
- [ ] Scratch size, flash-sector geometry, status area, and MCUboot trailer fit; sector-count limits are adequate.
- [ ] Swap implementation is included in the build and all required symbols link.
- [ ] Bootloader region is protected from accidental erase/write by the CAN uploader and flash driver. Consider STM32 flash write protection after validation.
- [ ] Power-loss recovery is tested during each swap phase, not assumed.

**Do not enable an unverified swap implementation on a device you cannot recover via SWD.**

## 7. CAN uploader and update protocol: changes required

The previous uploader was designed around overwrite-only / a simple signed-image transfer. Swap mode needs additional steps:

1. **Erase only the secondary slot's intended image/trailer region**, using flash-page-aligned operations. Never erase the bootloader or primary slot during upload.
2. **Transfer the image** (header + firmware + TLVs) to secondary, with bounds checks that leave room for trailer/status metadata.
3. **Verify integrity** after transfer: at minimum image length, header, SHA-256 and whichever signature policy MCUboot is configured to require. MCUboot's own validation remains authoritative.
4. **Request a test upgrade** using the MCUboot bootutil API / trailer format appropriate for this port (typically an API such as `boot_set_pending(false)` where supported). Confirm whether this API targets the correct slot and how it accesses the flash backend.
5. **Reset** only after the image and pending state have been committed and verified.
6. Report CAN status and failures clearly; interrupted uploads must not leave the bootloader trying to boot a partial image.

**Important:** Do not hand-write trailer bytes based on assumptions from another MCUboot version. Inspect the exact installed bootutil headers/source. Your current `BOOT_UPLOAD_MAX_SIZE = FLASH_AREA_2_SIZE - FLASH_AREA_IMAGE_SECTOR_SIZE` is only a rough limit and must be recalculated from the actual trailer/layout requirements.

## 8. Image generation / signing

- Keep building the application at the fixed primary-slot linker address.
- Package with `--header-size 0x200`, the correct alignment, version and slot size.
- **Remove `--overwrite-only`** from any old imgtool command when switching to swap mode.
- Determine whether this MCUboot version and upload flow require `imgtool --pad` / trailer handling or whether the device sets pending status itself. Avoid creating conflicting trailer metadata.
- Ensure the resulting image fits **including header, TLVs, padding and reserved trailer**.
- Keep image validation/signature settings aligned between imgtool and MCUboot. A SHA-256 TLV alone is not equivalent to an authenticated signature.

Do not reuse a Bank-2-linked Direct-XIP binary: rebuild the application for the fixed primary-slot address.

## 9. Confirm the new firmware from the application

Once the new firmware has booted and passed a meaningful self-test, call the **version-appropriate MCUboot confirmation API**, often `boot_set_confirmed()` where the application has access to the bootutil API and flash backend.

Conceptual flow:

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    if (application_self_test_passed()) {
        /* Confirm the running test image using your MCUboot port's API.
           Check return status; do not confirm before essential checks pass. */
        // boot_set_confirmed();
    }

    for (;;) { /* normal application */ }
}
```

The example intentionally does **not** claim that `boot_set_confirmed()` is already linked into your application. The application needs the compatible bootutil API, flash backend, and flash access permissions. If you use a custom CAN command instead, design a secure, reliable confirmation path.

A watchdog can help recover from a hung trial image; test its interaction with swap/revert and the bootloader.

## 10. Migration / test checklist

- [ ] Back up a working bootloader and application and ensure SWD recovery works.
- [ ] Verify actual `region_defs.h`, flash-map backend, option bytes and erase geometry.
- [ ] Choose and validate a swap algorithm and scratch/layout sizing.
- [ ] Remove Direct-XIP defines; enable exactly one supported swap strategy.
- [ ] Keep application linker origin fixed at `0x08010200` and VTOR consistent.
- [ ] Rebuild the application from scratch; package with swap-compatible imgtool options.
- [ ] Adapt CAN updater for secondary-slot upload, validation and **test-pending** request.
- [ ] Boot old confirmed firmware from slot 0.
- [ ] Upload new firmware to slot 1 and reset; observe swap and new application launch.
- [ ] **Do not confirm** trial firmware; reset again and verify rollback to old firmware.
- [ ] Repeat and **confirm** trial firmware; reset and verify new firmware persists.
- [ ] Interrupt power during upload and at multiple points in swap; verify recoverability.
- [ ] Verify button-based bootloader entry and SRAM magic flag still work.
- [ ] Check flash wear, watchdog handling, application size limits, and bootloader write protection.

## 11. Files needed to finish the exact implementation

To turn this plan into a buildable patch, inspect:

1. `Core/Src/boot_update.c` and its header — CAN state machine, erase/write logic and reset command.
2. `Core/Src/boot_flash.c` — flash device, page enumeration and protection.
3. `Drivers/stm32_mcu_boot/bl2/src/flash_map.c` and relevant headers — area IDs and scratch mapping.
4. Vendored MCUboot version / `bootutil` configuration and trailer APIs.
5. Current application `.ld`, `system_stm32l4xx.c`, and `imgtool` command / Python CAN uploader.

**Bottom line:** With swap + revert, **the application linker script remains fixed at `0x08010200`**. All future application images are built for that address, uploaded to the secondary slot, and swapped into the primary slot by MCUboot. The remaining engineering work is enabling a supported swap strategy and implementing its image trailer, pending/confirmation and recovery handling correctly.
