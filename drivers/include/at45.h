//*****************************************************************************
//
//      COPYRIGHT (c) ATMEL Norway, 1996-2001
//
//      The copyright to the document(s) herein is the property of
//      ATMEL Norway, Norway.
//
//      The document(s) may be used  and/or copied only with the written
//      permission from ATMEL Norway or in accordance with the terms and
//      conditions stipulated in the agreement/contract under which the
//      document(s) have been supplied.
//
//*****************************************************************************
//
//  File........: DATAFLASH.H
//
//  Author(s)...: ATMEL Norway
//
//  Target(s)...: Independent
//
//  Description.: Defines and prototypes for AT45Dxxx
//
//  Revisions...:
//
//  YYYYMMDD - VER. - COMMENT                                       - SIGN.
//
//  20010117 - 0.10 - Generated file                                -  RM
//  20031009          port to avr-gcc/avr-libc                      - M.Thomas
//
//*****************************************************************************


/*****************************************************************************

Filename:    DataFlash.h
Description: DataFlash library file for the FlashShield 1.0

******************************************************************************

DataFlash library for the FlashShield 1.0

Copyright(c) 2011 Async Labs Inc. All rights reserved.

This program is free software; you can redistribute it and/or modify it
under the terms of version 2 of the GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59
Temple Place - Suite 330, Boston, MA 02111-1307, USA.

Contact Information:
<asynclabs@asynclabs.com>

-----------------------------------------------------------------------------
Author    Date       Comment
-----------------------------------------------------------------------------
AsyncLabs 03/17/2010 Initial version
AsyncLabs 01/10/2011 Clean up

*****************************************************************************/
#ifndef AT45L_H
#define AT45L_H

#include "periph/gpio.h"
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

//Dataflash commands
#define FlashPageRead           0xD2    // Main memory page read
#define FlashToBuf1Transfer     0x53    // Main memory page to buffer 1 transfer
#define Buf1Read                0xD4    // Buffer 1 read
#define FlashToBuf2Transfer     0x55    // Main memory page to buffer 2 transfer
#define Buf2Read                0xD6    // Buffer 2 read
#define StatusReg               0xD7    // Status register
#define Buf1ToFlashWE           0x83    // Buffer 1 to main memory page program with built-in erase
#define Buf1Write               0x84    // Buffer 1 write
#define Buf2ToFlashWE           0x86    // Buffer 2 to main memory page program with built-in erase
#define Buf2Write               0x87    // Buffer 2 write
#define ReadMfgID               0x9F    // Read Manufacturer and Device ID
#define READ_LOW_POWER          0x01    // Continuous array read (low power)

#define PAGE_SIZE_0             0x3d
#define PAGE_SIZE_1             0x2a
#define PAGE_SIZE_2             0x80
#define PAGE_256                0xa6
#define PAGE_264                0xa7


#define BOOT_FUNC __attribute__((used,section(".boot")))

/**
 * @brief device descriptor for S25FL
 */
typedef struct {
    spi_t          spi;
    gpio_t         cs;
    gpio_t         rst;
    gpio_t         wp;
} at45_t;

int at45_init(at45_t *dev, spi_t spi, gpio_t cs, gpio_t rst, gpio_t wp);
void at45_read(at45_t *dev, uint32_t address, uint8_t *buffer, uint32_t bytes);
void at45_write(at45_t *dev, uint32_t address, uint8_t *buffer, uint32_t bytes);


void at45_Read_DF_ID(at45_t *dev, uint8_t *id);

void at45_Page_To_Buffer(at45_t *dev, uint16_t PageAdr, uint8_t BufferNo);
void at45_Buffer_To_Page(at45_t *dev, uint8_t BufferNo, uint16_t PageAdr);

uint8_t at45_Buffer_Read_Byte(at45_t  *dev, uint8_t BufferNo, uint16_t IntPageAdr);
void at45_Buffer_Write_Byte(at45_t *dev, uint8_t BufferNo, uint16_t IntPageAdr, uint8_t Data);

#ifdef __cplusplus
}
#endif

#endif /* AT45L_H */
/** @} */
