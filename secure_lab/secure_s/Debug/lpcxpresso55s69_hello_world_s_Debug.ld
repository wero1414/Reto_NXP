/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008 - 2013 Code Red Technologies Ltd, 
 * Copyright 2015, 2018 NXP
 * (c) NXP Semiconductors 2013-2019
 * Generated linker script file for LPC55S69
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v10.3.1 [Build 2233] [2019-02-20] on 22-abr-2019 2:09:16
 */

INCLUDE "lpcxpresso55s69_hello_world_s_Debug_library.ld"
INCLUDE "lpcxpresso55s69_hello_world_s_Debug_memory.ld"

ENTRY(ResetISR)

SECTIONS
{
    .text_Flash2 : ALIGN(4)
    {
       FILL(0xff)
        *(.text_Flash2*) /* for compatibility with previous releases */
        *(.text_veneer_table*) /* for compatibility with previous releases */
        *(.text.$Flash2*)
        *(.text.$veneer_table*)
        *(.rodata.$Flash2*)
        *(.rodata.$veneer_table*)
    } > veneer_table

    /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

    } > PROGRAM_FLASH

    .text : ALIGN(4)
    {
       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > PROGRAM_FLASH
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this. 
     */
    .ARM.extab : ALIGN(4) 
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > PROGRAM_FLASH

    __exidx_start = .;

    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > PROGRAM_FLASH
    __exidx_end = .;
 
    /* Section for TrustZone Secure Gateway veneers */
    .gnu.sgstubs : ALIGN (32)
    {
        . = ALIGN(32);
        _start_sg = .;
        *(.gnu.sgstubs*)
        . = ALIGN(32);
        _end_sg = .;
    } > PROGRAM_FLASH

    _etext = .;
        
    /* MAIN DATA SECTION */
    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > Ram0

    /* Main DATA section (Ram0) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       *(vtable)
       *(.ramfunc*)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
    } > Ram0 AT>PROGRAM_FLASH

    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > Ram0

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*) 
         . = ALIGN(4) ;
        _end_noinit = .;
    } > Ram0

    /* Reserve and place Heap within memory map */
    _HeapSize = 0x7f8;
    .heap :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > Ram0

     _StackSize = 0x7f8;
     /* Reserve space in memory for Stack */
    .heap2stackfill  :
    {
        . += _StackSize;
    } > Ram0
    /* Locate actual Stack in memory map */
    .stack ORIGIN(Ram0) + LENGTH(Ram0) - _StackSize - 0:  ALIGN(4)
    {
        _vStackBase = .;
        . = ALIGN(4);
        _vStackTop = . + _StackSize;
    } > Ram0

    /* ## Create checksum value (used in startup) ## */
    PROVIDE(__valid_user_code_checksum = 0 - 
                                         (_vStackTop 
                                         + (ResetISR + 1) 
                                         + (NMI_Handler + 1) 
                                         + (HardFault_Handler + 1) 
                                         + (( DEFINED(MemManage_Handler) ? MemManage_Handler : 0 ) + 1)   /* MemManage_Handler may not be defined */
                                         + (( DEFINED(BusFault_Handler) ? BusFault_Handler : 0 ) + 1)     /* BusFault_Handler may not be defined */
                                         + (( DEFINED(UsageFault_Handler) ? UsageFault_Handler : 0 ) + 1) /* UsageFault_Handler may not be defined */
                                         ) );

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}