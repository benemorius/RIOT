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
 * @ingroup     driver_s25fl
 * @{
 *
 * @file
 * @brief       Device driver implementation for S25FL flash memory
 *
 * @author      John-Mike Reed <bleeplabs.com>
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "s25fl.h"
#include "thread.h"
#include "msg.h"
#include "xtimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**********************************************************************
 * internal API declaration
 **********************************************************************/
static uint8_t _spi_write(s25fl_t *dev, uint8_t byte);
static uint8_t _spi_read(s25fl_t *dev);
static uint8_t read_status(s25fl_t *dev);
static void ready_wait(s25fl_t *dev);
static void write_enable(s25fl_t *dev);
// static void write_reg(s25fl_t *dev, uint8_t byte);
static void read_info(s25fl_t *dev);

/**********************************************************************
 * public API implementation
 **********************************************************************/

int s25fl_init(s25fl_t *dev, spi_t spi, gpio_t cs, gpio_t rst, gpio_t wp, gpio_t pwr)
{
//     SPI.setBitOrder(MSBFIRST);

    dev->cs = cs;
    dev->rst = rst;
    dev->wp = wp;
    dev->pwr = pwr;
    dev->spi = spi;
    int ret;

    gpio_set(dev->cs);
    gpio_set(dev->wp);
    gpio_clear(dev->pwr); //on
    gpio_clear(dev->rst);

    gpio_init(dev->pwr, GPIO_OUT);
    gpio_init(dev->rst, GPIO_OUT);
    gpio_init(dev->wp, GPIO_OUT);
    gpio_init(dev->cs, GPIO_OUT);

    xtimer_usleep(100*1000);

    gpio_set(dev->rst);

    xtimer_usleep(100*1000);


    if ((ret = spi_init_master(dev->spi, SPI_CONF_FIRST_RISING, SPI_SPEED_100KHZ)))
        return ret;

    uint8_t status = read_status(dev);
    printf("flash status: 0x%02x\n", status);

    read_info(dev);

    return 0;
}

void s25fl_power(s25fl_t *dev, bool on)
{
    if (on)
        gpio_clear(dev->pwr);
    else
        gpio_set(dev->pwr);
}

/**********************************************************************
 * internal API implementation
 **********************************************************************/

static uint8_t _spi_write(s25fl_t *dev, uint8_t byte)
{
    char receive;
    spi_transfer_byte(dev->spi, byte, &receive);
    return (uint8_t)receive;
}

static uint8_t _spi_read(s25fl_t *dev)
{
    return _spi_write(dev, 0);
}





//read and return the status register.
static uint8_t read_status(s25fl_t *dev)
{
    gpio_clear(dev->cs);
    _spi_write(dev, RDSR);
    uint8_t s = _spi_read(dev);
    gpio_set(dev->cs);
    return s;
}

// use between each communication to make sure S25FLxx is ready to go.
static void ready_wait(s25fl_t *dev)
{
    uint8_t status;
    do {
        status = read_status(dev);
    } while (status & 1);
}


// Must be done to allow erasing or writing
static void write_enable(s25fl_t *dev)
{
    gpio_clear(dev->cs);
    _spi_write(dev, WREN);
    gpio_set(dev->cs);
    ready_wait(dev);
    // Serial.println("write enabled");
}


// Erase an entire 4k sector the location is in.
// For example "erase_4k(300);" will erase everything from 0-3999.
//
// All erase commands take time. No other actions can be preformed
// while the chip is errasing except for reading the register
void erase_4k(s25fl_t *dev, uint32_t address)
{
    ready_wait(dev);
    write_enable(dev);

    gpio_clear(dev->cs);
    _spi_write(dev, 0x20);
    _spi_write(dev, address>>16);
    _spi_write(dev, address>>8);
    _spi_write(dev, address & 0xFF);
    gpio_set(dev->cs);
    ready_wait(dev);
}

// Errase an entire 64_k sector the location is in.
// For example erase4k(530000) will erase everything from 524543 to 589823.

void erase_64k(s25fl_t *dev, uint32_t address)
{
    ready_wait(dev);
    write_enable(dev);

    gpio_clear(dev->cs);
    _spi_write(dev, 0x20);
    _spi_write(dev, address>>16);
    _spi_write(dev, address>>8);
    _spi_write(dev, address & 0xFF);
    gpio_set(dev->cs);
    ready_wait(dev);
}

//errases all the memory. Can take several seconds.
void erase_all(s25fl_t *dev, uint32_t address)
{
    ready_wait(dev);
    write_enable(dev);
    gpio_clear(dev->cs);
    _spi_write(dev, CE);
    gpio_set(dev->cs);
    ready_wait(dev);
}




// Read data from the flash chip. There is no limit "length". The entire memory can be read with one command.
//read_S25(starting location, array, number of bytes);
void s25fl_read(s25fl_t *dev, uint32_t address, uint8_t *buffer, uint32_t length)
{
    gpio_clear(dev->cs);
    _spi_write(dev, READ);           //control byte follow by location bytes
    _spi_write(dev, address>>16);   // convert the location integer to 3 bytes
    _spi_write(dev, address>>8);
    _spi_write(dev, address & 0xff);

    for (int i=0; i<length+1;i++){
        buffer[i] = _spi_read(dev);
    }
    gpio_set(dev->cs);
}

// Programs up to 256 bytes of data to flash chip. Data must be erased first. You cannot overwrite.
// Only one continuous page (256 Bytes) can be programmed at once so there's some
// sorcery going on here to make it not wrap around.
// It's most efficent to only program one page so if you're going for speed make sure your
// location %=0 (for example location=256, length=255.) or your length is less that the bytes remain
// in the page (location =120 , length= 135)


//write_S25(starting location, array, number of bytes);
void s25fl_write(s25fl_t *dev, uint32_t address, uint8_t *buffer, uint32_t length)
{

    if (length>255){
        unsigned long reps=length>>8;
        unsigned long length1;
        unsigned long array_count;
        unsigned long first_length;
        unsigned remainer0=length-(256*reps);
        unsigned long addressb=address;

        printf("reps %lu\n", reps);
        printf("remainer0 %u\n", remainer0);


        for (int i=0; i<(reps+2);i++){

            if (i==0){

                length1=256-(addressb & 0xff);
                first_length=length1;
                if (length1==0){i++;}
                array_count=0;
            }

            if (i>0 && i<(reps+1)){
                addressb= first_length+address+(256*(i-1));;

                array_count=first_length+(256*(i-1));
                length1=255;

            }
            if (i==(reps+1)){
                addressb+=(256);
                array_count+=256;
                length1=remainer0;
                if (remainer0==0){break;}

            }
            //Serial.print("i ");Serial.println(i);
            //Serial.print("addressb ");Serial.println(addressb);
            //Serial.print("length1 ");Serial.println(length1);
            //Serial.print("array_count ");Serial.println(array_count );


            write_enable(dev);
            ready_wait(dev);
            gpio_clear(dev->cs);
            _spi_write(dev, PP);
            _spi_write(dev, addressb>>16);
            _spi_write(dev, addressb>>8);
            _spi_write(dev, addressb & 0xff);

            for (unsigned long i=array_count; i<(length1+array_count+1) ; i++){
                _spi_write(dev, buffer[i]);
            }

            gpio_set(dev->cs);
            ready_wait(dev);
        }
    }

    if (length<=255){
        if (((address & 0xff)!=0) | ((address & 0xff)<length)){
            uint8_t remainer = address & 0xff;
            uint8_t length1 =256-remainer;
//             uint8_t length2 = length-length1;
            unsigned long page1_address = address;
            unsigned long page2_address = address+length1;

            write_enable(dev);
            ready_wait(dev);
            gpio_clear(dev->cs);
            _spi_write(dev, PP);
            _spi_write(dev, page1_address>>16);
            _spi_write(dev, page1_address>>8);
            _spi_write(dev, page1_address & 0xff);

            for (int i=0; i<length1;i++){
                _spi_write(dev, buffer[i]);
            }

            gpio_set(dev->cs);
            ready_wait(dev);
            write_enable(dev);

            ready_wait(dev);

            gpio_clear(dev->cs);
            _spi_write(dev, PP);
            _spi_write(dev, page2_address>>16);
            _spi_write(dev, page2_address>>8);
            _spi_write(dev, page2_address & 0xff);

            for (int i=length1; i<length+1;i++){
                _spi_write(dev, buffer[i]);
            }

            gpio_set(dev->cs);
            ready_wait(dev);
            //Serial.println("//////////");
            //Serial.print("remainer ");Serial.println(remainer);

            //Serial.print("length1 ");Serial.println(length1);
            //Serial.print("length2 ");Serial.println(length2);
            //Serial.print("page1_address ");Serial.println(page1_address);
            //Serial.print("page2_address ");Serial.println(page2_address);
            //Serial.println("//////////");
        }
        else{
            printf("address & 0xff = 0x%02lx\n", address);

            write_enable(dev); // Must be done before writing can commence. Erase clears it.
            ready_wait(dev);
            gpio_clear(dev->cs);
            _spi_write(dev, PP);
            _spi_write(dev, address>>16);
            _spi_write(dev, address>>8);
            _spi_write(dev, address & 0xff);

            for (int i=0; i<length+1;i++){
                _spi_write(dev, buffer[i]);
            }

            gpio_set(dev->cs);
            ready_wait(dev);
        }
    }
}

//Used in conjuture with the write protect pin to protect blocks.
//For example on the S25FL216K sending "write_reg(B00001000);" will protect 2 blocks, 30 and 31.
//See the datasheet for more. http://www.mouser.com/ds/2/380/S25FL216K_00-6756.pdf

// static void write_reg(s25fl_t *dev, uint8_t byte)
// {
//     gpio_clear(dev->cs);
//     _spi_write(dev, WRSR);
//     _spi_write(dev, byte);
//     gpio_set(dev->cs);
// }

static void read_info(s25fl_t *dev)
{
    gpio_clear(dev->cs);
    _spi_write(dev, 0x9F);
//     _spi_read(dev);
    uint8_t m = _spi_read(dev);
    uint8_t t = _spi_read(dev);
    uint8_t c = _spi_read(dev);
    gpio_set(dev->cs);


    if (c == 0){
        printf("Cannot read S25FL. Check wiring\n");
    }

    printf("Manufacturer ID: 0x%02x\n", m);
    printf("Memory type: 0x%02x", t);
    printf("Capacity: 0x%02x\n", c);
//     ready_wait(dev);
}


