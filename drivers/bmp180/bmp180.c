/*
 * Copyright (C) 2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_bmp180
 * @{
 *
 * @file
 * @brief       Device driver implementation for the BMP180/BMP085 temperature and pressure sensor.
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @}
 */

#include <math.h>

#include "log.h"
#include "bmp180.h"
#include "bmp180_internals.h"
#include "bmp180_params.h"
#include "periph/i2c.h"
#include "xtimer.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

/**
 * @brief   Allocation of memory for device descriptors
 */
bmp180_t bmp180_devs[BMP180_NUMOF];

/* Internal function prototypes */
static int32_t _calculate_180_pressure(bmp180_t *dev, int32_t up, int32_t b5);
static int32_t _calculate_180_temperature(bmp180_t *dev, int32_t ut, int32_t *b5);
static int32_t _calculate_280_pressure(bmp180_t *dev, int32_t up, int32_t t_fine);
static int32_t _calculate_280_temperature(bmp180_t *dev, int32_t ut, int32_t *t_fine);
static int _read_280_raw(bmp180_t *dev, int32_t *up, int32_t *ut);
static int _read_180_ut(bmp180_t *dev, int32_t *ut);
static int _read_180_up(bmp180_t *dev, int32_t *up);
static int _compute_180_b5(bmp180_t *dev, int32_t ut, int32_t *b5);

/*---------------------------------------------------------------------------*
 *                          BMP180 Core API                                 *
 *---------------------------------------------------------------------------*/

int bmp180_init(bmp180_t *dev, i2c_t i2c, uint8_t mode, uint8_t address)
{
    dev->i2c_dev = i2c;
    dev->address = address;
    uint8_t calibration_address;

    /* Clamp oversampling mode */
    if (mode > BMP180_ULTRAHIGHRES) {
        mode = BMP180_ULTRAHIGHRES;
    }

    /* Setting oversampling mode */
    dev->oversampling = mode;

    /* Acquire exclusive access */
    i2c_acquire(dev->i2c_dev);

    /* Initialize I2C interface */
    if (i2c_init_master(dev->i2c_dev, I2C_SPEED_FAST)) {
        DEBUG("[Error] I2C device not enabled\n");
        return -1;
    }

    /* Check sensor ID */
    char device_id;
    i2c_read_reg(dev->i2c_dev, dev->address, BMP180_REGISTER_ID, &device_id);
    if (device_id == BMP180_DEVICE_ID) {
        DEBUG("found BMP180\n");
        dev->type = BMP_180;
        calibration_address = BMP180_CALIBRATION_AC1;
    }
    else if (device_id == BMP280_DEVICE_ID) {
        DEBUG("found BMP280\n");
        dev->type = BMP_280;
        calibration_address = BMP280_CALIBRATION_T1;
        /* configure low pass filter */
        i2c_write_reg(dev->i2c_dev, dev->address, 0xf5, 0x7 << 2);
    }
    else {
        DEBUG("[Error] Wrong device ID: 0x%hx\n", device_id);
        i2c_release(dev->i2c_dev);
        return -1;
    }

    /* adding delay before reading calibration values to avoid timing issues */
    xtimer_usleep(BMP180_ULTRALOWPOWER_DELAY);

    /* Read calibration values, using contiguous register addresses */
    if (i2c_read_regs(dev->i2c_dev, dev->address, calibration_address, (char*)&dev->calibration, 24) < 0) {
        DEBUG("[Error] Cannot read calibration registers.\n");
        i2c_release(dev->i2c_dev);
        return -1;
    }

    /* Release I2C device */
    i2c_release(dev->i2c_dev);

    switch (dev->type) {
        case BMP_180:
            DEBUG("AC1: %i\n", (int)dev->calibration.bmp180.ac1);
            DEBUG("AC2: %i\n", (int)dev->calibration.bmp180.ac2);
            DEBUG("AC3: %i\n", (int)dev->calibration.bmp180.ac3);
            DEBUG("AC4: %i\n", (int)dev->calibration.bmp180.ac4);
            DEBUG("AC5: %i\n", (int)dev->calibration.bmp180.ac5);
            DEBUG("AC6: %i\n", (int)dev->calibration.bmp180.ac6);
            DEBUG("B1: %i\n",  (int)dev->calibration.bmp180.b1);
            DEBUG("B2: %i\n",  (int)dev->calibration.bmp180.b2);
            DEBUG("MB: %i\n",  (int)dev->calibration.bmp180.mb);
            DEBUG("MC: %i\n",  (int)dev->calibration.bmp180.mc);
            DEBUG("MD: %i\n",  (int)dev->calibration.bmp180.md);
            break;

        case BMP_280:
            DEBUG("dig_T1 %u\n", dev->calibration.bmp280.dig_T1);
            DEBUG("dig_T2 %i\n", dev->calibration.bmp280.dig_T2);
            DEBUG("dig_T3 %i\n", dev->calibration.bmp280.dig_T3);
            DEBUG("dig_P1 %u\n", dev->calibration.bmp280.dig_P1);
            DEBUG("dig_P2 %i\n", dev->calibration.bmp280.dig_P2);
            DEBUG("dig_P3 %i\n", dev->calibration.bmp280.dig_P3);
            DEBUG("dig_P4 %i\n", dev->calibration.bmp280.dig_P4);
            DEBUG("dig_P5 %i\n", dev->calibration.bmp280.dig_P5);
            DEBUG("dig_P6 %i\n", dev->calibration.bmp280.dig_P6);
            DEBUG("dig_P7 %i\n", dev->calibration.bmp280.dig_P7);
            DEBUG("dig_P8 %i\n", dev->calibration.bmp280.dig_P8);
            DEBUG("dig_P9 %i\n", dev->calibration.bmp280.dig_P9);
            break;
    }
    return 0;
}

void bmp180_auto_init(void)
{
    for (unsigned i = 0; i < BMP180_NUMOF; i++) {
        if (bmp180_init(&bmp180_devs[i], bmp180_params[i].i2c_dev, bmp180_params[i].mode, bmp180_params[i].address) < 0) {
            LOG_ERROR("Unable to initialize BMP180 sensor #%i\n", i);
        }
#ifdef MODULE_SAUL_REG
        for (unsigned j = 0; j < 2; j++) {
            bmp180_saul_reg[i][j].dev = &bmp180_devs[i];
            saul_reg_add(&bmp180_saul_reg[i][j]);
        }
#endif
    }
}

int bmp180_read_pressure(bmp180_t *dev, int32_t *pressure)
{
    int32_t temperature;
    bmp180_read_both(dev, pressure, &temperature);
    return 0;
}

int bmp180_read_temperature(bmp180_t *dev, int32_t *temperature)
{
    int32_t pressure;
    bmp180_read_both(dev, &pressure, temperature);
    return 0;
}

int bmp180_read_both(bmp180_t *dev, int32_t *pressure, int32_t *temperature)
{
    int32_t up = 0, ut = 0, t_fine = 0;

    switch (dev->type) {
        case BMP_180:
            _read_180_ut(dev, &ut);
            _read_180_up(dev, &up);
            *temperature = _calculate_180_temperature(dev, ut, &t_fine);
            *pressure = _calculate_180_pressure(dev, up, t_fine);
            break;

        case BMP_280:
            _read_280_raw(dev, &up, &ut);
            *temperature = _calculate_280_temperature(dev, ut, &t_fine);
            *pressure = _calculate_280_pressure(dev, up, t_fine);
            break;
    }
    return 0;
}

int bmp180_altitude(bmp180_t *dev, int32_t pressure_0, int32_t *altitude)
{
    int32_t p;
    bmp180_read_pressure(dev, &p);

    *altitude = (int32_t)(44330.0 * (1.0 - pow((double)p / pressure_0, 0.1903)));

    return 0;
}

int bmp180_sealevel_pressure(bmp180_t *dev, int32_t altitude, int32_t *pressure_0)
{
    int32_t p;
    bmp180_read_pressure(dev, &p);

    *pressure_0 = (int32_t)((double)p / pow(1.0 - (altitude / 44330.0), 5.255));

    return 0;
}

/*------------------------------------------------------------------------------------*/
/*                                Internal functions                                  */
/*------------------------------------------------------------------------------------*/

static int _read_180_ut(bmp180_t *dev, int32_t *output)
{
    /* Read UT (Uncompsensated Temperature value) */
    char ut[2] = {0};
    char control[2] = { BMP180_REGISTER_CONTROL, BMP180_TEMPERATURE_COMMAND };
    i2c_write_bytes(dev->i2c_dev, dev->address, control, 2);
    xtimer_usleep(BMP180_ULTRALOWPOWER_DELAY);
    if (i2c_read_regs(dev->i2c_dev, dev->address, BMP180_REGISTER_DATA, ut, 2) < 0) {
        DEBUG("[Error] Cannot read uncompensated temperature.\n");
        i2c_release(dev->i2c_dev);
        return -1;
    }
    *output = ( ut[0] << 8 ) | ut[1];

    DEBUG("UT: %i\n", (int)*output);

    return 0;
}

static int _read_180_up(bmp180_t *dev, int32_t *output)
{
    /* Read UP (Uncompsensated Pressure value) */
    char up[3] = {0};
    char control[2] = { BMP180_REGISTER_CONTROL, BMP180_PRESSURE_COMMAND | (dev->oversampling & 0x3) << 6 };
    i2c_write_bytes(dev->i2c_dev, dev->address, control, 2);
    switch (dev->oversampling) {
    case BMP180_ULTRALOWPOWER:
        xtimer_usleep(BMP180_ULTRALOWPOWER_DELAY);
        break;
    case BMP180_STANDARD:
        xtimer_usleep(BMP180_STANDARD_DELAY);
        break;
    case BMP180_HIGHRES:
        xtimer_usleep(BMP180_HIGHRES_DELAY);
        break;
    case BMP180_ULTRAHIGHRES:
        xtimer_usleep(BMP180_ULTRAHIGHRES_DELAY);
        break;
    default:
        xtimer_usleep(BMP180_ULTRALOWPOWER_DELAY);
        break;
    }
    if (i2c_read_regs(dev->i2c_dev, dev->address, BMP180_REGISTER_DATA, up, 3) < 0) {
        DEBUG("[Error] Cannot read uncompensated pressure.\n");
        i2c_release(dev->i2c_dev);
        return -1;
    }

    *output = ((up[0] << 16) | (up[1] << 8) | up[2]) >> (8 - dev->oversampling);

    DEBUG("UP: %i\n", (int)*output);

    return 0;
}

static int _read_280_raw(bmp180_t *dev, int32_t *up, int32_t *ut)
{
    /* acquire exclusive access */
    i2c_acquire(dev->i2c_dev);

    /* select oversampling configuration bits */
    uint8_t p_oversample, t_oversample;
    switch (dev->oversampling) {
        default:
        case BMP180_ULTRALOWPOWER:
            p_oversample = 1;
            t_oversample = 1;
            break;
        case BMP180_STANDARD:
            p_oversample = 3;
            t_oversample = 1;
            break;
        case BMP180_HIGHRES:
            p_oversample = 4;
            t_oversample = 1;
            break;
        case BMP180_ULTRAHIGHRES:
            p_oversample = 5;
            t_oversample = 2;
            break;
    }

    /* construct measurement command */
    uint8_t ctrl_meas = (p_oversample << 2) | (t_oversample << 5) | 1;
    char control[2] = { BMP180_REGISTER_CONTROL, ctrl_meas };

    /* begin sampling */
    i2c_write_bytes(dev->i2c_dev, dev->address, control, 2);

    /* sleep until sampling finishes */
    switch (dev->oversampling) {
        default:
        case BMP180_ULTRALOWPOWER:
            xtimer_usleep(BMP280_ULTRALOWPOWER_DELAY);
            break;
        case BMP180_STANDARD:
            xtimer_usleep(BMP280_STANDARD_DELAY);
            break;
        case BMP180_HIGHRES:
            xtimer_usleep(BMP280_HIGHRES_DELAY);
            break;
        case BMP180_ULTRAHIGHRES:
            xtimer_usleep(BMP280_ULTRAHIGHRES_DELAY);
            break;
    }

    /* check that sampling is complete */
//     char status;
//     i2c_read_reg(dev->i2c_dev, dev->address, BMP280_STATUS, &status);
//     printf("status 0x%x\n", status);

    /* read both pressure and temperature data */
    char buf[6];
    if (i2c_read_regs(dev->i2c_dev, dev->address, BMP280_PRESSURE_DATA, buf, 6) < 0) {
        DEBUG("[Error] Cannot read uncompensated values.\n");
        i2c_release(dev->i2c_dev);
        return -1;
    }

    /* release I2C device */
    i2c_release(dev->i2c_dev);

    /* decipher measurement data */
    *up = ((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
    *ut = ((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));

    DEBUG("UP: %li 0x%lx\n", *up, *up);
    DEBUG("UT: %li 0x%lx\n", *ut, *ut);

    return 0;
}

static int32_t _calculate_180_pressure(bmp180_t *dev, int32_t up, int32_t b5)
{
    int32_t x1, x2, x3, b3, b6, p;
    uint32_t b4, b7;

    b6 = b5 - 4000;
    x1 = ((int32_t)dev->calibration.bmp180.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = ((int32_t)dev->calibration.bmp180.ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = ((((int32_t)dev->calibration.bmp180.ac1*4 + x3) << dev->oversampling) + 2) >> 2;
    x1 = ((int32_t)dev->calibration.bmp180.ac3 * b6) >> 13;
    x2 = ((int32_t)dev->calibration.bmp180.b1 * (b6 * b6) >> 12) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (int32_t)dev->calibration.bmp180.ac4 * (uint32_t)(x3+32768) >> 15;
    b7 = ((uint32_t)up - b3) * (uint32_t)(50000UL >> dev->oversampling);
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    }
    else {
        p = (b7 / b4) * 2;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    return p + ((x1 + x2 + 3791) >> 4);
}

static int32_t _calculate_180_temperature(bmp180_t *dev, int32_t ut, int32_t *b5)
{
    _compute_180_b5(dev, ut, b5);
    return (*b5 + 8) >> 4;
}

static int32_t _calculate_280_pressure(bmp180_t *dev, int32_t up, int32_t t_fine)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->calibration.bmp280.dig_P6;
    var2 = var2 + ((var1*(int64_t)dev->calibration.bmp280.dig_P5)<<17);
    var2 = var2 + (((int64_t)dev->calibration.bmp280.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dev->calibration.bmp280.dig_P3)>>8)
         + ((var1 * (int64_t)dev->calibration.bmp280.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dev->calibration.bmp280.dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-up;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)dev->calibration.bmp280.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dev->calibration.bmp280.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calibration.bmp280.dig_P7)<<4);

    return p;
}

static int32_t _calculate_280_temperature(bmp180_t *dev, int32_t ut, int32_t *t_fine)
{
    int32_t var1 = ((((ut>>3) - ((int32_t)dev->calibration.bmp280.dig_T1<<1)))
                    * ((int32_t)dev->calibration.bmp280.dig_T2)) >> 11;
    int32_t var2 = (((((ut>>4) - ((int32_t)dev->calibration.bmp280.dig_T1))
                    * ((ut>>4) - ((int32_t)dev->calibration.bmp280.dig_T1))) >> 12)
                    * ((int32_t)dev->calibration.bmp280.dig_T3)) >> 14;
    *t_fine = (var1 + var2);
    return (*t_fine * 5 + 128) >> 8;
}

static int _compute_180_b5(bmp180_t *dev, int32_t ut, int32_t *output)
{
    int32_t x1, x2;
    x1 = (ut - dev->calibration.bmp180.ac6) * dev->calibration.bmp180.ac5 >> 15;
    x2 = (dev->calibration.bmp180.mc << 11) / (x1 + dev->calibration.bmp180.md);
    *output = x1 + x2;
    return 0;
}
