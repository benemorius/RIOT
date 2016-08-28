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
 * @ingroup     cpu_cc26xx-cc13xx
 * @{
 *
 * @file
 * @brief       Low-level SPI driver implementation
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <math.h>

#include "board.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "mutex.h"

#if SPI_NUMOF

// static uint32_t speed_to_baud(spi_speed_t speed);
// static uint32_t conf_to_cpol(spi_conf_t conf);

/**
 * @brief Data-structure holding the state for a SPI device
 */
typedef struct {
    char(*cb)(char data);
} spi_state_t;

// static inline void irq_handler_transfer(spi_t *spi, spi_t dev);

/**
 * @brief Reserve memory for saving the SPI device's state
 */
// static spi_state_t spi_config[SPI_NUMOF];

/**
 * @brief Array holding one pre-initialized mutex for each SPI device
 */
static mutex_t locks[] =  {
    #if SPI_0_EN
    [0] = MUTEX_INIT,
    #endif
    #if SPI_1_EN
    [1] = MUTEX_INIT,
    #endif
    #if SPI_2_EN
    [2] = MUTEX_INIT
    #endif
};

/**
 * @brief   Get the base register for the given SPI device
 */
static inline spi_t _dev(spi_t spi)
{
//     return spi_config[spi].dev;
    return 0;
}

int spi_init_master(spi_t spi, spi_conf_t conf, spi_speed_t speed)
{
//     printf("spi %i init start\n", spi);

//     const ssi_conf_t *config = &spi_config[spi];
//     int proto = 0;
//
//     /* Enable peripheral power domain */
//     if(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON) {
//         PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
//         while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);
//     }
//
//     /* Enable serial power domain */
//     if(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON) {
//         PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
//         while(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON);
//     }
//
//     /* Enable SSI0 peripheral */
//     PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
//     PRCMPeripheralRunEnable(spi_config[spi].prcmp);
//
//     /* Apply settings and wait for them to take effect */
//     PRCMLoadSet();
//     while(!PRCMLoadGet());
//
//     SSIDisable(_dev(spi));
//
//     /*
//      * Map SSI signals to the correct GPIO pins and configure them as
//      * hardware controlled.
//      */
//     IOCPinTypeSsiMaster(_dev(spi),
//                         spi_config[spi].ioid_miso,
//                         spi_config[spi].ioid_mosi,
//                         IOID_UNUSED,
//                         spi_config[spi].ioid_clk
//                        );
//
//     SSIConfigSetExpClk(_dev(spi), SysCtrlClockGet(), proto, SSI_MODE_MASTER, 10000000, config->bits);
//
//     SSIEnable(_dev(spi));

    return 0;
}

BOOT_FUNC int spi_transfer_byte(spi_t spi, char out, char *in)
{
//     SSIDataPut(_dev(spi), out);
//     while((SSIStatus(_dev(spi)) & SSI_RX_NOT_EMPTY) == 0);
//     uint32_t receive;
//     SSIDataGet(_dev(spi), &receive);
//     *in = receive;
    return 0;
}

int spi_transfer_bytes(spi_t dev, char *out, char *in, unsigned int length)
{
    return 0;
}

int spi_transfer_reg(spi_t dev, uint8_t reg, char out, char *in)
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
