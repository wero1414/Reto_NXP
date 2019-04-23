/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008 - 2013 Code Red Technologies Ltd, 
 * Copyright 2015, 2018 NXP
 * (c) NXP Semiconductors 2013-2019
 * Generated linker script file for LPC55S69
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v10.3.1 [Build 2233] [2019-02-20] on 23/04/2019 01:23:21 PM
 */

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 0xfe00 /* 65024 bytes (alias Flash) */  
  veneer_table (rx) : ORIGIN = 0x1000fe00, LENGTH = 0x200 /* 512 bytes (alias Flash2) */  
  Ram0 (rwx) : ORIGIN = 0x30000000, LENGTH = 0x7fff /* 32767 bytes (alias RAM) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x10000000  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x10000000 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x10000000 + 0xfe00 ; /* 65024 bytes */  
  __top_Flash = 0x10000000 + 0xfe00 ; /* 65024 bytes */  
  __base_veneer_table = 0x1000fe00  ; /* veneer_table */  
  __base_Flash2 = 0x1000fe00 ; /* Flash2 */  
  __top_veneer_table = 0x1000fe00 + 0x200 ; /* 512 bytes */  
  __top_Flash2 = 0x1000fe00 + 0x200 ; /* 512 bytes */  
  __base_Ram0 = 0x30000000  ; /* Ram0 */  
  __base_RAM = 0x30000000 ; /* RAM */  
  __top_Ram0 = 0x30000000 + 0x7fff ; /* 32767 bytes */  
  __top_RAM = 0x30000000 + 0x7fff ; /* 32767 bytes */  
