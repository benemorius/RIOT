/*
 *    Copyright (c) 2016 Thomas Stilwell <stilwellt@openlabs.co>
 *
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 *
 *    The above copyright notice and this permission notice shall be
 *    included in all copies or substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @addtogroup  driver_periph
 * @{
 *
 * @file
 * @brief       Low-level I2C driver implementation
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "mutex.h"
#include "periph_conf.h"
#include "periph/i2c.h"
#include "thread.h"

#include "driverlib/i2c.h"
#include "driverlib/sys_ctrl.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/**
 * @brief   Get the base register for the given I2C device
 */
static inline i2c_t _dev(dev_t i2c)
{
    return i2c_config[i2c].dev;
}

int i2c_init_master(i2c_t dev, i2c_speed_t speed)
{
    /* First, make sure the SERIAL PD is on */
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
    while((PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON));

    /* Enable the clock to I2C */
    PRCMPeripheralRunEnable(i2c_config[dev].prcmp);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    /* Enable and initialize the I2C master module */
    I2CMasterInitExpClk(_dev(dev), SysCtrlClockGet(), true);

    I2CMasterDisable(_dev(dev));

    IOCIOPortPullSet(i2c_config[dev].ioid_sda, IOC_NO_IOPULL);
    IOCIOPortPullSet(i2c_config[dev].ioid_scl, IOC_NO_IOPULL);
    IOCPinTypeI2c(_dev(dev), i2c_config[dev].ioid_sda, i2c_config[dev].ioid_scl);

    /* Enable and initialize the I2C master module */
    I2CMasterInitExpClk(_dev(dev), SysCtrlClockGet(), true);

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

static bool i2c_status(i2c_t dev)
{
    uint32_t status;

    status = I2CMasterErr(_dev(dev));
    if(status & (I2C_MSTAT_DATACK_N_M | I2C_MSTAT_ADRACK_N_M)) {
        I2CMasterControl(_dev(dev), I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    }

    return status == I2C_MASTER_ERR_NONE;
}

int i2c_read_byte(i2c_t dev, uint8_t address, char* data)
{
	return i2c_read_bytes(dev, address, data, 1);
}
int i2c_read_bytes(i2c_t dev, uint8_t address, char *data, int length)
{
    uint8_t i;
    bool success;

    /* Set slave address */
    I2CMasterSlaveAddrSet(_dev(dev), address, true);

    /* Check if another master has access */
    while(I2CMasterBusBusy(_dev(dev)));

    /* Assert RUN + START + ACK */
    I2CMasterControl(_dev(dev), I2C_MASTER_CMD_BURST_RECEIVE_START);

    i = 0;
    success = true;
    while(i < (length - 1) && success) {
        while(I2CMasterBusy(_dev(dev)));
        success = i2c_status(dev);
        if(success) {
            data[i] = I2CMasterDataGet(_dev(dev));
            I2CMasterControl(_dev(dev), I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            i++;
        }
    }

    if(success) {
        while(I2CMasterBusy(_dev(dev)));
        success = i2c_status(dev);
        if(success) {
            data[length - 1] = I2CMasterDataGet(_dev(dev));
            I2CMasterControl(_dev(dev), I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            while(I2CMasterBusBusy(_dev(dev)));
        }
    }

    return success;
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
    uint32_t i;
    bool success;

    /* Write slave address */
    I2CMasterSlaveAddrSet(_dev(dev), address, false);

    /* Write first byte */
    I2CMasterDataPut(_dev(dev), data[0]);

    /* Check if another master has access */
    while(I2CMasterBusBusy(_dev(dev)));

    /* Assert RUN + START */
    I2CMasterControl(_dev(dev), I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(_dev(dev)));
    success = i2c_status(dev);

    for(i = 1; i < length && success; i++) {
        printf("another byte\n");
        /* Write next byte */
        I2CMasterDataPut(_dev(dev), data[i]);
        if(i < length - 1) {
            /* Clear START */
            I2CMasterControl(_dev(dev), I2C_MASTER_CMD_BURST_SEND_CONT);
            while(I2CMasterBusy(_dev(dev)));
            success = i2c_status(dev);
        }
    }

    /* Assert stop */
    if(success) {
        /* Assert STOP */
        I2CMasterControl(_dev(dev), I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(I2CMasterBusy(_dev(dev)));
        success = i2c_status(dev);
        while(I2CMasterBusBusy(_dev(dev)));
    }

    return success;


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
