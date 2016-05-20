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
 * @ingroup         cpu_cc25xx-cc13xx
 * @{
 *
 * @file
 * @brief           Shared CPU specific definitions for TI CC25xx,CC13xx
 *
 * @author          Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef PERIPH_CPU_COMMON_H
#define PERIPH_CPU_COMMON_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
    * @brief   Length of the CPU_ID in octets
    */
#define CPUID_LEN           (8U)

/**
    * @brief   Use the shared SPI functions
    * @{
    */
#define PERIPH_SPI_NEEDS_TRANSFER_BYTES
#define PERIPH_SPI_NEEDS_TRANSFER_REG
#define PERIPH_SPI_NEEDS_TRANSFER_REGS
/** @} */

/**
    * @brief   Override the default gpio_t type definition
    * @{
    */
#define HAVE_GPIO_T
typedef uint32_t gpio_t;
/** @} */

/**
    * @brief   Definition of a fitting UNDEF value
    */
#define GPIO_UNDEF          GPIO_PIN_UNUSED

/**
    * @brief   Define a CPU specific GPIO pin generator macro
    */

#define GPIO_PIN(x)      (1 << x)

/**
    * @brief   Enable the given peripheral clock
    *
    * @param[in] bus       bus the peripheral is connected to
    * @param[in] mask      bit in the RCC enable register
    */
void periph_clk_en(uint8_t bus, uint32_t mask);

/**
    * @brief   Disable the given peripheral clock
    *
    * @param[in] bus       bus the peripheral is connected to
    * @param[in] mask      bit in the RCC enable register
    */
void periph_clk_dis(uint8_t bus, uint32_t mask);

/**
    * @brief   Configure the given pin to be used as ADC input
    *
    * @param[in] pin       pin to configure
    */
void gpio_init_analog(gpio_t pin);

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CPU_COMMON_H */
/** @} */
