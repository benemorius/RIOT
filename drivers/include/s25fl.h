/*
 * Arduino S25FLx Serial Flash library
 * By John-Mike Reed (Dr. Bleep) of Bleep labs
 * bleeplabs.com
 *
 * Usage guide at github.com/BleepLabs/S25FLx/
 * Datasheet for S25FL216K www.mouser.com/ds/2/380/S25FL216K_00-6756.pdf
 * This library can interface with most of the S25FL family with no modifications.
 *
 * This free library is realeased under Creative comoms license CC BY-SA 3.0
 * http://creativecommons.org/licenses/by-sa/3.0/deed.en_US
 */

/**
 * @defgroup    driver_pir S25FL flash memory
 * @ingroup     drivers
 * @brief       Device driver interface for S25FL flash memory
 * @{
 *
 * @file
 * @brief       Device driver interface for S25FL flash memory
 *
 * @author      John-Mike Reed <bleeplabs.com>
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef S25FL_H
#define S25FL_H

#include "kernel_types.h"
#include "periph/gpio.h"
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief S25FL control bytes
 * @{
 */
#define WREN        0x06    /** Write Enable */
#define WRDI        0x04    /** Write Disable */
#define RDSR        0x05    /** Read Status Register */
#define WRSR        0x01    /** Write Status Register */
#define READ        0x03    /** Read Data Bytes  */
#define FAST_READ   0x0b    /** Read Data Bytes at Higher Speed //Not used as as the 328 isn't fast enough  */
#define PP          0x02    /** Page Program  */
#define SE          0x20    /** Sector Erase (4k)  */
#define BE          0x20    /** Block Erase (64k)  */
#define CE          0xc7    /** Erase entire chip  */
#define DP          0xb9    /** Deep Power-down  */
#define RES         0xab    /** Release Power-down, return Device ID */
#define RDID        0x9F    /** Read Manufacture ID, memory type ID, capacity ID */
/** @} */

/**
 * @brief device descriptor for S25FL
 */
typedef struct {
	spi_t          spi;
    gpio_t         cs;
    gpio_t         rst;
    gpio_t         wp;
    gpio_t         pwr;
} s25fl_t;



// flash();
// byte stat();
// void waitforit();
// void write_enable();
// void erase_4k(unsigned long loc);
// void erase_64k(unsigned long loc);
// void erase_all();
// void read(unsigned long loc, uint8_t* array, unsigned long length);
// void write(unsigned long loc, uint8_t* array, unsigned long length);
// void write_reg(byte w);
// void read_info();





void erase_4k(s25fl_t *dev, uint32_t address);
void erase_64k(s25fl_t *dev, uint32_t address);
void erase_all(s25fl_t *dev, uint32_t address);
void s25fl_read(s25fl_t *dev, uint32_t address, uint8_t *buffer, uint32_t length);
void s25fl_write(s25fl_t *dev, uint32_t address, uint8_t *buffer, uint32_t length);
void s25fl_power(s25fl_t *dev, bool on);




/**
 * @brief               Initialize an S25FL flash memory
 *
 * The PIR motion sensor is interfaced by a single GPIO pin, specified by
 * `gpio`.
 *
 * @note
 * The sensor needs up to a minute to settle down before meaningful
 * measurements can be made.
 *
 * @param[out] dev      device descriptor of an PIR sensor
 * @param[in]  cs       the GPIO device the sensor is connected to
 *
 * @return              0 on success
 * @return              -1 on error
 */
int s25fl_init(s25fl_t* dev, spi_t spi, gpio_t cs, gpio_t rst, gpio_t wp, gpio_t pwr);

#ifdef __cplusplus
}
#endif

#endif /* S25FL_H */
/** @} */
