/*
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 * Copyright (C) 2015-2017 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @addtogroup      cpu_kinetis
 * @{
 *
 * @file
 * @brief           Sections definitions for the Freescale Kinetis MCUs
 *
 * @author          Johann Fischer <j.fischer@phytec.de>
 * @author          Joakim Nohlgård <joakim.nohlgard@eistec.se>
 *
 * @}
 */
OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)

_vectors_length = 0x400;
_flashsec_length = 0x10;
m_sector_size =  ( 2 * 1024 );
__vector_table_size__ = (48*4);
__region_ROM_start__   =  (0x00000000);
__region_ROM_end__     =  (0x0007FFFF);

m_fsl_prodInfo_start = (__region_ROM_end__ - m_sector_size + 1);
m_fsl_prodInfo_end   = (__region_ROM_end__);
FREESCALE_PROD_DATA_BASE_ADDR = m_fsl_prodInfo_start;

INCLUDE cortexm_rom_offset.ld

_flash_sec_offset = _vectors_length + _rom_offset;

__ram_vector_table__ = DEFINED(__ram_vector_table__) ? __ram_vector_table__ : 1;
__ram_vector_table_size__ = DEFINED(__ram_vector_table__) ? __vector_table_size__ : 0;

MEMORY
{
    vectors    : ORIGIN = _rom_start_addr + _rom_offset, LENGTH = _vectors_length
    flashsec   : ORIGIN = _rom_start_addr + _vectors_length + _rom_offset, LENGTH = _flashsec_length
    rom (rx)   : ORIGIN = _rom_start_addr + _vectors_length + _flashsec_length + _rom_offset, LENGTH = _fw_rom_length - (_vectors_length + _flashsec_length)
    ram (!rx)  : ORIGIN = _ram_start_addr, LENGTH = _ram_length
}

SECTIONS
{
    /* Interrupt vectors 0x00-0x3ff. */
    .vector :
    {
        __VECTOR_TABLE = .;
        _isr_vectors = .;
        KEEP(*(SORT(.vector*)))
    } > vectors
    ASSERT (SIZEOF(.vector) == _vectors_length, "Interrupt vector table of invalid size.")
    ASSERT (ADDR(.vector) == _rom_offset, "Interrupt vector table at invalid location (linker-script error?)")
    ASSERT (LOADADDR(.vector) == _rom_offset, "Interrupt vector table at invalid location (linker-script error?)")

    .interrupts_ram :
    {
        . = ALIGN(4);
        __VECTOR_RAM__ = .;
        __interrupts_ram_start__ = .; /* Create a global symbol at data start */
        *(.m_interrupts_ram)     /* This is a user defined section */
        . += __ram_vector_table_size__;
        . = ALIGN(4);
        __interrupts_ram_end__ = .; /* Define a global symbol at data end */
    } > ram
    __VECTOR_RAM = __VECTOR_RAM__;
    __RAM_VECTOR_TABLE_SIZE_BYTES = (__interrupts_ram_end__ - __interrupts_ram_start__);

    /* Flash configuration field, very important in order to not accidentally lock the device */
    /* Flash configuration field 0x400-0x40f. */
    .fcfield :
    {
        . = ALIGN(4);
        KEEP(*(.fcfield))
        . = ALIGN(4);
    } > flashsec
    ASSERT (SIZEOF(.fcfield) == _flashsec_length, "Flash configuration field of invalid size (linker-script error?)")
    ASSERT (ADDR(.fcfield) == _flash_sec_offset, "Flash configuration field at invalid position (linker-script error?)")
    ASSERT (LOADADDR(.fcfield) == _flash_sec_offset, "Flash configuration field at invalid position (linker-script error?)")
}

INCLUDE cortexm_base.ld
