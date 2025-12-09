ENTRY(Reset_Handler)

MEMORY
{
    ITCM     (rx)      : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM     (rwx)     : ORIGIN = 0x20000000, LENGTH = 128K
    RAM      (rwx)     : ORIGIN = 0x24000000, LENGTH = 512K
    FLASH    (rx)      : ORIGIN = 0x08000000, LENGTH = 1024K
}

SECTIONS
{
    /* Vector table */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    /* Code and constants */
    .text :
    {
        *(.text*)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))
    } > FLASH

    /* Exception tables */
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > FLASH

    /* Ensure .data LMA starts after FLASH code */
    . = ALIGN(8);

    /* Initialized data copied to RAM from FLASH */
    .data : AT(LOADADDR(.text) + SIZEOF(.text) + SIZEOF(.ARM.exidx) + SIZEOF(.ARM.extab))
    {
        _sdata = .;
        _edata = .;
    } > RAM

    /* Symbols for startup */
    _sidata = LOADADDR(.data);      /* source of .data in FLASH */
    _Min_Stack_Size = 0x1000;       /* 4 KB min stack */

    /* Zero-initialized BSS */
    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM

    /* Heap in RAM */
    _end = _ebss;
    _heap_start = _end;
    _heap_end = ORIGIN(RAM) + LENGTH(RAM);

    /* Stack top in DTCM */
    _estack = ORIGIN(DTCM) + LENGTH(DTCM);
}

/* Provide standard symbols for startup */
PROVIDE(_stack = _estack);