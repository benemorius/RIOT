/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32f0
 * @{
 *
 * @file
 * @brief       Low-level SPI driver implementation
 *
 * @author      Ryan Kurte <ryankurte@gmail.com>
 *
 * @}
 */

#include <math.h>

#include "cpu.h"
#include "board.h"
#include "periph_conf.h"
#include "periph/spi.h"
#include "mutex.h"

#if SPI_NUMOF

// static uint32_t speed_to_baud(spi_speed_t speed);
// static uint32_t conf_to_cpol(spi_conf_t conf);

mutex_t locks[2];

//SPI configuration storage
//static spi_state_t spi_config[SPI_NUMOF];
//TODO: add configuration for async mode

int spi_init_master(spi_t spi, spi_conf_t conf, spi_speed_t speed)
{
    return 0;
}

int spi_transfer_byte(spi_t spi, char out, char *in)
{
    return 0;
}

int spi_transfer_bytes(spi_t spi, char *out, char *in, unsigned int length)
{
    return 0;
}

int spi_acquire(spi_t dev)
{
	if (dev >= SPI_NUMOF) {
		return -1;
	}
    mutex_lock(&locks[dev]);
//     printf("acquire\n");
    return 0;
}

int spi_release(spi_t dev)
{
	if (dev >= SPI_NUMOF) {
		return -1;
	}
// 	printf("release\n");
    mutex_unlock(&locks[dev]);
	return 0;
}

int spi_transfer_reg(spi_t dev, uint8_t reg, char out, char *in)
{
	int trans_ret;

	trans_ret = spi_transfer_byte(dev, reg, in);
	if (trans_ret < 0) {
		return -1;
	}
	trans_ret = spi_transfer_byte(dev, out, in);
	if (trans_ret < 0) {
		return -1;
	}

	return 1;
}

int spi_transfer_regs(spi_t dev, uint8_t reg, char *out, char *in, unsigned int length)
{
	int trans_ret;

	trans_ret = spi_transfer_byte(dev, reg, in);
	if (trans_ret < 0) {
		return -1;
	}
	trans_ret = spi_transfer_bytes(dev, out, in, length);
	if (trans_ret < 0) {
		return -1;
	}

	return trans_ret;
}

// //Convert speeds to integers
// static uint32_t speed_to_baud(spi_speed_t speed)
// {
//     uint32_t baud;
//
//     switch (speed) {
//     case SPI_SPEED_100KHZ:
//         baud = 100000;
//         break;
//     case SPI_SPEED_400KHZ:
//         baud = 400000;
//         break;
//     case SPI_SPEED_1MHZ:
//         baud = 1000000;
//         break;
//     case SPI_SPEED_5MHZ:
//         baud = 4000000;
//         break;
//     case SPI_SPEED_10MHZ:
//         baud = 10000000;
//         break;
//     default:
//         baud = 1000000;
//         break;
//     }
//
//     return baud;
// }
//
// //Convert to normal modes
// static uint32_t conf_to_cpol(spi_conf_t conf)
// {
//     uint32_t cpol;
//
// 	switch (conf) {
// 		case SPI_CONF_FIRST_RISING:
// // 			cpol = usartClockMode0;
// 			break;
// 		case SPI_CONF_SECOND_RISING:
// // 			cpol = usartClockMode1;
// 			break;
// 		case SPI_CONF_FIRST_FALLING:
// // 			cpol = usartClockMode2;
// 			break;
// 		case SPI_CONF_SECOND_FALLING:
// // 			cpol = usartClockMode3;
// 			break;
// 		default:
// // 			cpol = usartClockMode0;
// 			break;
// 	}
//
//     return cpol;
// }

#endif
