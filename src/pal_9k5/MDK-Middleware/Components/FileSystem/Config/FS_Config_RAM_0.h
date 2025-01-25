/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::File System:Drive
 * Copyright (c) 2004-2024 Arm Limited (or its affiliates). All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    FS_Config_RAM_%Instance%.h
 * Purpose: File System Configuration for RAM Drive
 * Rev.:    V6.3.0
 *----------------------------------------------------------------------------*/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
//------ With VS Code: Open Preview for Configuration Wizard -------------------

// Pretend NAND is a RAM drive
#include "mt29f4g.h"

// <h>RAM Drive %Instance%
// <i>Configuration for RAM assigned to drive letter "R%Instance%:"
#define RAM0_ENABLE 1

//   <o>Device Size <0x4C00-0xFFFFF000:0x400>
//   <i>Define the size of RAM device in bytes
//   <i>Default: 0x8000
#define RAM0_SIZE MT29F4G_BLOCK_COUNT* MT29F4G_PAGE_PER_BLOCK* MT29F4G_PAGE_SIZE

//   <e>Locate Drive Cache and Drive Buffer
//   <i>Locate RAM drive buffer at a specific address.
//   <i>If not enabled, the linker selects base address.
#define RAM0_RELOC 1

//     <s>Section Name
//     <i>Define the name of the section for the file system buffer.
//     <i>Linker script shall have this section defined.
#define RAM0_SECTION ".ram_d2_sec"

#define RAM0_FAT_JOURNAL 0

//   </e>

// </h>
