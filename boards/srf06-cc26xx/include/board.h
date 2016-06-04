#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**
 * @name The nominal CPU core clock in this board
 */
#define F_CPU               (48000000UL)

/**
 * @name Assign the peripheral timer to be used as hardware timer
 */
#define HW_TIMER            TIMER_0

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
 * @name Macros for controlling the on-board LEDs.
 * @{
 */
// #define LED_GREEN_ON        GPIO->P[LED_0_PORT].DOUTSET = 1 << LED_0_PIN;
// #define LED_GREEN_OFF       GPIO->P[LED_0_PORT].DOUTCLR = 1 << LED_0_PIN;
// #define LED_GREEN_TOGGLE    GPIO->P[LED_0_PORT].DOUTTGL = 1 << LED_0_PIN;
// #define LED_RED_ON          GPIO->P[LED_1_PORT].DOUTSET = 1 << LED_1_PIN;
// #define LED_RED_OFF         GPIO->P[LED_1_PORT].DOUTCLR = 1 << LED_1_PIN;
// #define LED_RED_TOGGLE      GPIO->P[LED_1_PORT].DOUTTGL = 1 << LED_1_PIN;
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
