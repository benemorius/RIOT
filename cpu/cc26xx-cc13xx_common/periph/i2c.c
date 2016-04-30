/*
    Copyright (c) 2016 <stilwellt@openlabs.co>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/

#include "mutex.h"
#include "periph_conf.h"
#include "periph/i2c.h"
#include "thread.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define I2C_READ (1)
#define I2C_WRITE (0)

int i2c_init_master(i2c_t dev, i2c_speed_t speed)
{
    return 0;
}

int i2c_init_slave(i2c_t dev, uint8_t address)
{
	/* TODO: implement slave mode */
	return -1;
}

int i2c_acquire(i2c_t dev)
{
	if(dev >= I2C_NUMOF){
		return -1;
	}
	return 0;
}

int i2c_release(i2c_t dev)
{
	if(dev >= I2C_NUMOF){
		return -1;
	}
	return 0;
}

static inline void _i2c_stop(i2c_t i2c)
{
}

static inline int _i2c_receive(i2c_t i2c, uint8_t* databuf, uint32_t length)
{
	return 0;
}

static inline int _i2c_transmit(i2c_t i2c, uint8_t* databuf, uint32_t length)
{
	return 0;
}

int i2c_read_byte(i2c_t dev, uint8_t address, char* data)
{
	return i2c_read_bytes(dev, address, data, 1);
}

int i2c_read_bytes(i2c_t dev, uint8_t address, char* data, int length)
{
	return 0;
}

int i2c_read_reg(i2c_t dev, uint8_t address, uint8_t reg, char* data)
{
	return i2c_read_regs(dev, address, reg, data, 1);
}

int i2c_read_regs(i2c_t dev, uint8_t address, uint8_t reg, char* data, int length)
{
	return 0;
}

int i2c_write_byte(i2c_t dev, uint8_t address, char data)
{
	return i2c_write_bytes(dev, address, &data, 1);
}

int i2c_write_bytes(i2c_t dev, uint8_t address, char *data, int length)
{
	return 0;
}

int i2c_write_reg(i2c_t dev, uint8_t address, uint8_t reg, char data)
{
    return i2c_write_regs(dev, address, reg, &data, 1);
}

int i2c_write_regs(i2c_t dev, uint8_t address, uint8_t reg, char* data, int length)
{
    return 0;
}

static inline void i2c_isr(i2c_t i2c, kernel_pid_t pid, volatile uint32_t* interrupt_flags, uint32_t* interrupt_flags_expected)
{

}

#if I2C_0_EN
void I2C0_IRQHandler(void)
{
}
#endif

#if I2C_1_EN
void I2C1_IRQHandler(void)
{
}
#endif
