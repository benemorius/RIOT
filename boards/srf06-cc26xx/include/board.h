#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#include "cpu.h"
#include "periph_conf.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**
 * @name The nominal CPU core clock in this board
 */
#define F_CPU               (48000000UL)

/**
 * @brief   Xtimer configuration
 * @{
 */
#define XTIMER_SHIFT            (4)
#define CC26X0_LOW_POWER_TIMER
/** @} */

/**
 * @name Assign the UART interface to be used for stdio
 */
#define STDIO               UART_0
#define STDIO_BAUDRATE      (115200U)
#define STDIO_RX_BUFSIZE    (64U)

/**
 * @name LED pin definitions
 * @{
 */

// #define LED_0_PORT				4
// #define LED_0_PIN				2
// #define LED_1_PORT				4
// #define LED_1_PIN				3

/** @} */

/**
 * @brief   Macros for controlling the on-board LEDs
 * @{
 */
#define LED0_PIN            GPIO_PIN(18)
#define LED1_PIN            GPIO_PIN(18)

#define LED0_ON             gpio_set(LED0_PIN)
#define LED0_OFF            gpio_clear(LED0_PIN)
#define LED0_TOGGLE         gpio_toggle(LED0_PIN)

#define LED1_ON             gpio_set(LED1_PIN)
#define LED1_OFF            gpio_clear(LED1_PIN)
#define LED1_TOGGLE         gpio_toggle(LED1_PIN)
/** @} */

/**
 * @name Define the interface to the AT86RF231 radio
 * @{
 */
// #define AT86RF231_SPI       SPI_0
// #define AT86RF231_CS        GPIO_10
// #define AT86RF231_INT       GPIO_5
// #define AT86RF231_RESET     GPIO_8
// #define AT86RF231_SLEEP     GPIO_6

#define GPIO_MEM_PWR GPIO_PIN(22)
#define GPIO_MEM_RST GPIO_PIN(21)
#define GPIO_MEM_CS GPIO_PIN(20)
#define GPIO_MEM_WP GPIO_PIN(19)

#define GPIO_BTN_B GPIO_PIN(24)

/**
 * Define the type for the radio packet length for the transceiver
 */
typedef uint8_t radio_packet_length_t;


#define GPIO_COUNT 1


#define MODULE_AT86RF231 1


/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif
