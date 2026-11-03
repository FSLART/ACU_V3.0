/*
 ******************************************************************************
 * Linker script for STM32F412RETx (F4 series)
 * Flash: 512 KB (0x0800_0000 - 0x0807_FFFF)
 * RAM  : 256 KB (0x2000_0000 - 0x2003_FFFF)
 ******************************************************************************
 */

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM); /* end of RAM */

_Min_Heap_Size = 0x200;   /* required amount of heap */
_Min_Stack_Size = 0x400;  /* required amount of stack */

/* Memories definition */
MEMORY
{
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 256K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
}

/* Sections */
SECTIONS
{
  /* Vector table */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* Code and read-only data */
  .text :
  {
    . = ALIGN(4);
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) : { . = ALIGN(4); *(.ARM.extab* .gnu.linkonce.armextab.*) . = ALIGN(4); } >FLASH
  .ARM       (READONLY) : { . = ALIGN(4); __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; . = ALIGN(4); } >FLASH
  .preinit_array (READONLY) : { . = ALIGN(4); PROVIDE_HIDDEN (__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN (__preinit_array_end = .); . = ALIGN(4); } >FLASH
  .init_array (READONLY) : { . = ALIGN(4); PROVIDE_HIDDEN (__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN (__init_array_end = .); . = ALIGN(4); } >FLASH
  .fini_array (READONLY) : { . = ALIGN(4); PROVIDE_HIDDEN (__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN (__fini_array_end = .); . = ALIGN(4); } >FLASH

  /* Initialized data */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* Uninitialized data */
  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* Heap & stack */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ : { libc.a(*) libm.a(*) libgcc.a(*) }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
