cmake_minimum_required(VERSION 3.20)
project(MCU_BOOT_L476 C ASM)
set(CMAKE_C_STANDARD 11)
set(MCU Drivers/stm32_mcu_boot)
set(HAL Drivers/STM32F4xx_HAL_Driver)
add_compile_options(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -ffunction-sections -fdata-sections -Wall -Wextra)
add_compile_definitions(USE_HAL_DRIVER STM32F412xx)
include_directories(Core/Inc Core/Inc/mcuboot_config
    ${HAL}/Inc Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${MCU}/boot/bootutil/include ${MCU}/bl2/ext/mcuboot/include
    ${MCU}/platform/ext/driver Drivers/mbedtls_sha256/include)
set(HAL_SOURCES)
foreach(module hal hal_can hal_cortex hal_dma hal_dma_ex hal_exti hal_flash
        hal_flash_ex hal_gpio hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex hal_uart)
    list(APPEND HAL_SOURCES ${HAL}/Src/stm32f4xx_${module}.c)
endforeach()

set(PLATFORM_SOURCES ${HAL_SOURCES} Core/Src/system_stm32f4xx.c
    Core/Src/syscalls.c Core/Src/sysmem.c Core/Startup/startup_stm32f412retx.s)
set(BOOT_SOURCES)
foreach(module bootutil_misc caps fault_injection_hardening image_validate loader swap_scratch tlv)
    list(APPEND BOOT_SOURCES ${MCU}/boot/bootutil/src/${module}.c)
endforeach()
list(APPEND BOOT_SOURCES ${MCU}/bl2/src/flash_map.c
    ${MCU}/bl2/ext/mcuboot/flash_map_extended.c ${MCU}/bl2/ext/mcuboot/flash_map_legacy.c
    Drivers/mbedtls_sha256/library/sha256.c Drivers/mbedtls_sha256/library/platform_util.c)
add_executable(bootloader ${PLATFORM_SOURCES} ${BOOT_SOURCES}
    Core/Src/main.c Core/Src/stm32f4xx_it.c Core/Src/stm32f4xx_hal_msp.c
    Core/Src/boot_flash.c Core/Src/boot_port.c Core/Src/boot_update.c Core/Src/bootloader.c)
target_compile_options(bootloader PRIVATE -UNDEBUG)
set(FIRMWARE_TARGETS bootloader)
if(EXISTS "${CMAKE_SOURCE_DIR}/Application/main.c")
    add_executable(demo_app ${PLATFORM_SOURCES} Application/main.c)
    target_compile_definitions(demo_app PRIVATE USER_VECT_TAB_ADDRESS VECT_TAB_OFFSET=0x10200U)
    list(APPEND FIRMWARE_TARGETS demo_app)
endif()
foreach(target ${FIRMWARE_TARGETS})
    if(target STREQUAL "bootloader")
        set(script ${CMAKE_SOURCE_DIR}/STM32F412RETx_FLASH.ld)
    else()
        set(script ${CMAKE_SOURCE_DIR}/Application/STM32L476RG_APP.ld)
    endif()
    set_target_properties(${target} PROPERTIES SUFFIX .elf LINK_DEPENDS "${script}")
    target_link_options(${target} PRIVATE -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16
        -mfloat-abi=hard -T${script} --specs=nano.specs --specs=nosys.specs
        -Wl,--gc-sections -Wl,-Map=${target}.map -Wl,--print-memory-usage)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> ${target}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> ${target}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>)
endforeach()

add_executable(boot_tests EXCLUDE_FROM_ALL ${BOOT_SOURCES}
    Core/Src/boot_update.c Core/Src/bootloader.c tests/test_boot.c tests/startup.c)
add_executable(boot_port_tests EXCLUDE_FROM_ALL tests/test_heartbeat.c tests/startup.c Core/Src/boot_update.c Core/Src/bootloader.c)
foreach(test_target boot_tests boot_port_tests)
    target_compile_options(${test_target} PRIVATE -UNDEBUG)
    target_link_options(${test_target} PRIVATE -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16
        -mfloat-abi=hard -nostartfiles --specs=rdimon.specs
        -T${CMAKE_SOURCE_DIR}/tests/qemu.ld -Wl,--gc-sections)
    set_target_properties(${test_target} PROPERTIES SUFFIX .elf LINK_DEPENDS "${CMAKE_SOURCE_DIR}/tests/qemu.ld")
endforeach()
