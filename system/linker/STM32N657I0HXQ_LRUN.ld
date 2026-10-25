/*
******************************************************************************
**
** @file        : STM32N6XX_LRUN.ld
**
** @author      : GPM Application Team
**
** @brief       : Linker script for STM32N6XX Device from STM32N6 series
**                      2048 KBytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
******************************************************************************
** @attention
**
** Copyright (c) 2025 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM); /* end of "RAM" Ram type memory */
_sstack = _estack - _Min_Stack_Size;

_Min_Heap_Size = 0x200; /* required amount of heap */
_Min_Stack_Size = 0x4000; /* required amount of stack */

/* Memories definition. RAM spans FLEXRAM, AXISRAM1 and AXISRAM2, which are
 * behind RISAF7, RISAF2 and RISAF3. */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x34000400,   LENGTH = 2047K   /* 0x34000400 .. 0x34200000 */
  /* The first 32 MB of the 64 MB part. Its top 2 MB, 0x71E00000..0x71FFFFFF,
     is the OTA staging area and is outside this region. */
  EXTMEM_PSRAM (rw)     : ORIGIN = 0x70000000,   LENGTH = 0x01E00000
}

/* Sections */
SECTIONS
{
  /* The startup code into "RAM" Ram type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >RAM

  /* The program code and other data into "RAM" Ram type memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >RAM

  /* Constant data into "RAM" Ram type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >RAM

  .ARM.extab   (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
   {
     . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >RAM

  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
   {
     . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >RAM

  .preinit_array     (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >RAM

  .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >RAM

  .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >RAM

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM

  .noncacheable :
  {
    . = ALIGN(8);
    __snoncacheable = .;/* create symbol for start of section */
    KEEP(*(noncacheable_buffer))
    . = ALIGN(8);
    __enoncacheable = .;  /* create symbol for end of section */
  } > RAM

  .gnu.sgstubs :
  {
    . = ALIGN(4);
    *(.gnu.sgstubs*)   /* Secure Gateway stubs */
    . = ALIGN(4);
  } >RAM

  /* Epoch-controller blobs. VMA in PSRAM, LMA in RAM: Reset_Handler copies
   * them before zeroing .bss, which reuses the LMA range, so AT(addr) is used
   * and not "AT> RAM". The ALIGN(64), the blobs' own alignment, is outside
   * the section so __ext_psram_init_lma equals LOADADDR. */
  . = ALIGN(64);
  __ext_psram_init_lma = .;

  .ext_psram_init : AT(__ext_psram_init_lma)
  {
    _sext_psram_init = .;     /* VMA start */
    *(.ext_psram_init)
    *(.ext_psram_init*)
    . = ALIGN(16);
    _eext_psram_init = .;     /* VMA end */
  } > EXTMEM_PSRAM

  _sidata_ext_psram_init = LOADADDR(.ext_psram_init);

  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* DMA buffers. MPU_Config marks [__axisram_nc_start, __axisram_nc_end)
   * non-cacheable. Not loaded and not zeroed at startup. */
  .axisram_nc (NOLOAD) :
  {
    . = ALIGN(32);            /* cache line and MPU granule */
    __axisram_nc_start = .;
    KEEP(*(.axisram_nc))
    *(.axisram_nc*)
    . = ALIGN(32);
    __axisram_nc_end = .;
  } >RAM

    .ext_psram (NOLOAD) :
  {
    . = ALIGN(16);            /* DCMIPP requires 16-byte alignment */
    *(.ext_psram)
    *(.ext_psram*)
    . = ALIGN(32);
    __ext_psram_pool_start = .;  /* camera buffer pool, to the region end */
  } >EXTMEM_PSRAM

  __ext_psram_pool_end = ORIGIN(EXTMEM_PSRAM) + LENGTH(EXTMEM_PSRAM);

  /* User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left.
   * NOLOAD: it would otherwise take an LMA after .ext_psram_init's. */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* The loaded image ends at the LMA tail of .ext_psram_init and must stay
   * below the FSBL at 0x34180000. Runtime-only sections may extend above. */
  ASSERT(__ext_psram_init_lma + SIZEOF(.ext_psram_init) <= 0x34180000, "The loaded image would overlap the FSBL at 0x34180000. Reduce the code, data or model size.")

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
