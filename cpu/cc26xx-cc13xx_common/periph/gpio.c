/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_efm32
 * @{
 *
 * @file
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Hauke Petersen <mail@haukepetersen.de>
 * @author      Ryan Kurte <ryankurte@gmail.com>
 *
 * @}
 */

#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "periph/gpio.h"
#include "periph_conf.h"
#include "lpm.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define GPIO_CLKEN()     CMU_ClockEnable(cmuClock_GPIO, true)

#if GPIO_NUMOF

static inline void gpio_irq_handler(gpio_t dev);

/* guard file in case no GPIO devices are defined */

typedef struct {
    gpio_cb_t cb;
    void *arg;
} gpio_state_t;

static gpio_state_t gpio_config[GPIO_NUMOF];

int gpio_init_out(gpio_t dev, gpio_pp_t pullup)
{
    GPIO_Port_TypeDef port = 0;
    uint32_t pin = 0;

    GPIO_CLKEN();

    switch (dev) {
#if GPIO_0_EN
    case GPIO_0:
        port = GPIO_0_PORT;
        pin = GPIO_0_PIN;
        break;
#endif
#if GPIO_1_EN
    case GPIO_1:
        port = GPIO_1_PORT;
        pin = GPIO_1_PIN;
        break;
#endif
#if GPIO_2_EN
    case GPIO_2:
        port = GPIO_2_PORT;
        pin = GPIO_2_PIN;
        break;
#endif
#if GPIO_3_EN
    case GPIO_3:
        port = GPIO_3_PORT;
        pin = GPIO_3_PIN;
        break;
#endif
#if GPIO_4_EN
    case GPIO_4:
        port = GPIO_4_PORT;
        pin = GPIO_4_PIN;
        break;
#endif
#if GPIO_5_EN
    case GPIO_5:
        port = GPIO_5_PORT;
        pin = GPIO_5_PIN;
        break;
#endif
#if GPIO_6_EN
    case GPIO_6:
        port = GPIO_6_PORT;
        pin = GPIO_6_PIN;
        break;
#endif
#if GPIO_7_EN
    case GPIO_7:
        port = GPIO_7_PORT;
        pin = GPIO_7_PIN;
        break;
#endif
#if GPIO_8_EN
    case GPIO_8:
        port = GPIO_8_PORT;
        pin = GPIO_8_PIN;
        break;
#endif
#if GPIO_9_EN
    case GPIO_9:
        port = GPIO_9_PORT;
        pin = GPIO_9_PIN;
        break;
#endif
#if GPIO_10_EN
    case GPIO_10:
        port = GPIO_10_PORT;
        pin = GPIO_10_PIN;
        break;
#endif
#if GPIO_11_EN
    case GPIO_11:
        port = GPIO_11_PORT;
        pin = GPIO_11_PIN;
        break;
#endif
#if GPIO_12_EN
    case GPIO_12:
        port = GPIO_12_PORT;
        pin = GPIO_12_PIN;
        break;
#endif
#if GPIO_13_EN
    case GPIO_13:
        port = GPIO_13_PORT;
        pin = GPIO_13_PIN;
        break;
#endif
#if GPIO_14_EN
    case GPIO_14:
        port = GPIO_14_PORT;
        pin = GPIO_14_PIN;
        break;
#endif
#if GPIO_15_EN
    case GPIO_15:
        port = GPIO_15_PORT;
        pin = GPIO_15_PIN;
        break;
#endif
#if GPIO_16_EN
    case GPIO_16:
        port = GPIO_16_PORT;
        pin = GPIO_16_PIN;
        break;
#endif
#if GPIO_17_EN
    case GPIO_17:
        port = GPIO_17_PORT;
        pin = GPIO_17_PIN;
        break;
#endif
#if GPIO_18_EN
    case GPIO_18:
        port = GPIO_18_PORT;
        pin = GPIO_18_PIN;
        break;
#endif
#if GPIO_19_EN
    case GPIO_19:
        port = GPIO_19_PORT;
        pin = GPIO_19_PIN;
        break;
#endif
#if GPIO_20_EN
    case GPIO_20:
        port = GPIO_20_PORT;
        pin = GPIO_20_PIN;
        break;
#endif
#if GPIO_21_EN
    case GPIO_21:
        port = GPIO_21_PORT;
        pin = GPIO_21_PIN;
        break;
#endif
#if GPIO_22_EN
    case GPIO_22:
        port = GPIO_22_PORT;
        pin = GPIO_22_PIN;
        break;
#endif
#if GPIO_23_EN
    case GPIO_23:
        port = GPIO_23_PORT;
        pin = GPIO_23_PIN;
        break;
#endif
#if GPIO_24_EN
    case GPIO_24:
        port = GPIO_24_PORT;
        pin = GPIO_24_PIN;
        break;
#endif
#if GPIO_25_EN
    case GPIO_25:
        port = GPIO_25_PORT;
        pin = GPIO_25_PIN;
        break;
#endif
#if GPIO_26_EN
    case GPIO_26:
        port = GPIO_26_PORT;
        pin = GPIO_26_PIN;
        break;
#endif

    }

    //TODO: Pullup on output doesn't make sense?

    GPIO_PinModeSet(port, pin, gpioModePushPull, 1);

    return 0; /* all OK */
}

int gpio_init_in(gpio_t dev, gpio_pp_t pullup)
{
    GPIO_Port_TypeDef port = 0;
    uint32_t pin = 0;

    GPIO_CLKEN();

    switch (dev) {
#if GPIO_0_EN
    case GPIO_0:
        port = GPIO_0_PORT;
        pin = GPIO_0_PIN;
        break;
#endif
#if GPIO_1_EN
    case GPIO_1:
        port = GPIO_1_PORT;
        pin = GPIO_1_PIN;
        break;
#endif
#if GPIO_2_EN
    case GPIO_2:
        port = GPIO_2_PORT;
        pin = GPIO_2_PIN;
        break;
#endif
#if GPIO_3_EN
    case GPIO_3:
        port = GPIO_3_PORT;
        pin = GPIO_3_PIN;
        break;
#endif
#if GPIO_4_EN
    case GPIO_4:
        port = GPIO_4_PORT;
        pin = GPIO_4_PIN;
        break;
#endif
#if GPIO_5_EN
    case GPIO_5:
        port = GPIO_5_PORT;
        pin = GPIO_5_PIN;
        break;
#endif
#if GPIO_6_EN
    case GPIO_6:
        port = GPIO_6_PORT;
        pin = GPIO_6_PIN;
        break;
#endif
#if GPIO_7_EN
    case GPIO_7:
        port = GPIO_7_PORT;
        pin = GPIO_7_PIN;
        break;
#endif
#if GPIO_8_EN
    case GPIO_8:
        port = GPIO_8_PORT;
        pin = GPIO_8_PIN;
        break;
#endif
#if GPIO_9_EN
    case GPIO_9:
        port = GPIO_9_PORT;
        pin = GPIO_9_PIN;
        break;
#endif
#if GPIO_10_EN
    case GPIO_10:
        port = GPIO_10_PORT;
        pin = GPIO_10_PIN;
        break;
#endif
#if GPIO_11_EN
    case GPIO_11:
        port = GPIO_11_PORT;
        pin = GPIO_11_PIN;
        break;
#endif
#if GPIO_12_EN
    case GPIO_12:
        port = GPIO_12_PORT;
        pin = GPIO_12_PIN;
        break;
#endif
#if GPIO_13_EN
    case GPIO_13:
        port = GPIO_13_PORT;
        pin = GPIO_13_PIN;
        break;
#endif
#if GPIO_14_EN
    case GPIO_14:
        port = GPIO_14_PORT;
        pin = GPIO_14_PIN;
        break;
#endif
#if GPIO_15_EN
    case GPIO_15:
        port = GPIO_15_PORT;
        pin = GPIO_15_PIN;
        break;
#endif
#if GPIO_16_EN
    case GPIO_16:
        port = GPIO_16_PORT;
        pin = GPIO_16_PIN;
        break;
#endif
#if GPIO_17_EN
    case GPIO_17:
        port = GPIO_17_PORT;
        pin = GPIO_17_PIN;
        break;
#endif
#if GPIO_18_EN
    case GPIO_18:
        port = GPIO_18_PORT;
        pin = GPIO_18_PIN;
        break;
#endif
#if GPIO_19_EN
    case GPIO_19:
        port = GPIO_19_PORT;
        pin = GPIO_19_PIN;
        break;
#endif
#if GPIO_20_EN
    case GPIO_20:
        port = GPIO_20_PORT;
        pin = GPIO_20_PIN;
        break;
#endif
#if GPIO_21_EN
    case GPIO_21:
        port = GPIO_21_PORT;
        pin = GPIO_21_PIN;
        break;
#endif
#if GPIO_22_EN
    case GPIO_22:
        port = GPIO_22_PORT;
        pin = GPIO_22_PIN;
        break;
#endif
#if GPIO_23_EN
    case GPIO_23:
        port = GPIO_23_PORT;
        pin = GPIO_23_PIN;
        break;
#endif
#if GPIO_24_EN
    case GPIO_24:
        port = GPIO_24_PORT;
        pin = GPIO_24_PIN;
        break;
#endif
#if GPIO_25_EN
    case GPIO_25:
        port = GPIO_25_PORT;
        pin = GPIO_25_PIN;
        break;
#endif
#if GPIO_26_EN
    case GPIO_26:
        port = GPIO_26_PORT;
        pin = GPIO_26_PIN;
        break;
#endif

    }

    if (pullup == GPIO_NOPULL) {
        GPIO_PinModeSet(port, pin, gpioModeInput, 0);

    } else if (pullup == GPIO_PULLUP) {
        GPIO_PinModeSet(port, pin, gpioModeInputPull, 0);
        GPIO_PinOutSet(port, pin);

    } else if (pullup == GPIO_PULLDOWN) {
        GPIO_PinModeSet(port, pin, gpioModeInputPull, 0);
        GPIO_PinOutClear(port, pin);
    }

    return 0; /* everything alright here */
}

int gpio_init_int(gpio_t dev, gpio_pp_t pullup, gpio_flank_t flank, gpio_cb_t cb, void *arg)
{
    int res;
    uint32_t pin = 0;
    GPIO_Port_TypeDef port = 0;

    /* configure pin as input */
    res = gpio_init_in(dev, pullup);
    if (res < 0) {
        return res;
    }

    /* set interrupt priority (its the same for all EXTI interrupts) */
    NVIC_SetPriority(GPIO_ODD_IRQn, GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIO_EVEN_IRQn, GPIO_IRQ_PRIO);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    //NVIC_SetPriority(PCNT0_IRQn, GPIO_IRQ_PRIO);
    //NVIC_SetPriority(PCNT0_IRQn, GPIO_IRQ_PRIO);
    //NVIC_SetPriority(PCNT0_IRQn, GPIO_IRQ_PRIO);

    /* enable clock of the SYSCFG module for EXTI configuration */
    //RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    /* read pin number, set EXIT channel and enable global interrupt for EXTI channel */
    switch (dev) {
#if GPIO_0_EN
    case GPIO_0:
        pin = GPIO_0_PIN;
        port = GPIO_0_PORT;
        break;
#endif
#if GPIO_1_EN
    case GPIO_1:
        pin = GPIO_1_PIN;
        port = GPIO_1_PORT;
        break;
#endif
#if GPIO_2_EN
    case GPIO_2:
        pin = GPIO_2_PIN;
        port = GPIO_2_PORT;
        break;
#endif
#if GPIO_3_EN
    case GPIO_3:
        pin = GPIO_3_PIN;
        port = GPIO_3_PORT;
        break;
#endif
#if GPIO_4_EN
    case GPIO_4:
        pin = GPIO_4_PIN;
        port = GPIO_4_PORT;
        break;
#endif
#if GPIO_5_EN
    case GPIO_5:
        pin = GPIO_5_PIN;
        port = GPIO_5_PORT;
        break;
#endif
#if GPIO_6_EN
    case GPIO_6:
        pin = GPIO_6_PIN;
        port = GPIO_6_PORT;
        break;
#endif
#if GPIO_7_EN
    case GPIO_7:
        pin = GPIO_7_PIN;
        port = GPIO_7_PORT;
        break;
#endif
#if GPIO_8_EN
    case GPIO_8:
        pin = GPIO_8_PIN;
        port = GPIO_8_PORT;
        break;
#endif
#if GPIO_9_EN
    case GPIO_9:
        pin = GPIO_9_PIN;
        port = GPIO_9_PORT;
        break;
#endif
#if GPIO_10_EN
    case GPIO_10:
        pin = GPIO_10_PIN;
        port = GPIO_10_PORT;
        break;
#endif
#if GPIO_11_EN
    case GPIO_11:
        pin = GPIO_11_PIN;
        port = GPIO_11_PORT;
        break;
#endif
#if GPIO_12_EN
    case GPIO_12:
        pin = GPIO_12_PIN;
        port = GPIO_12_PORT;
        break;
#endif
#if GPIO_13_EN
    case GPIO_13:
        pin = GPIO_13_PIN;
        port = GPIO_13_PORT;
        break;
#endif
#if GPIO_14_EN
    case GPIO_14:
        pin = GPIO_14_PIN;
        port = GPIO_14_PORT;
        break;
#endif
#if GPIO_15_EN
    case GPIO_15:
        pin = GPIO_15_PIN;
        port = GPIO_15_PORT;
        break;
#endif
#if GPIO_16_EN
    case GPIO_16:
        pin = GPIO_16_PIN;
        port = GPIO_16_PORT;
        break;
#endif
#if GPIO_17_EN
    case GPIO_17:
        pin = GPIO_17_PIN;
        port = GPIO_17_PORT;
        break;
#endif
#if GPIO_18_EN
    case GPIO_18:
        pin = GPIO_18_PIN;
        port = GPIO_18_PORT;
        break;
#endif
#if GPIO_19_EN
    case GPIO_19:
        pin = GPIO_19_PIN;
        port = GPIO_19_PORT;
        break;
#endif
#if GPIO_20_EN
    case GPIO_20:
        pin = GPIO_20_PIN;
        port = GPIO_20_PORT;
        break;
#endif
#if GPIO_21_EN
    case GPIO_21:
        pin = GPIO_21_PIN;
        port = GPIO_21_PORT;
        break;
#endif
#if GPIO_22_EN
    case GPIO_22:
        pin = GPIO_22_PIN;
        port = GPIO_22_PORT;
        break;
#endif
#if GPIO_23_EN
    case GPIO_23:
        pin = GPIO_23_PIN;
        port = GPIO_23_PORT;
        break;
#endif
#if GPIO_24_EN
    case GPIO_24:
        pin = GPIO_24_PIN;
        port = GPIO_24_PORT;
        break;
#endif
#if GPIO_25_EN
    case GPIO_25:
        pin = GPIO_25_PIN;
        port = GPIO_25_PORT;
        break;
#endif

    }

    /* set callback */
    gpio_config[dev].cb = cb;
    gpio_config[dev].arg = arg;

    /* configure the event that triggers an interrupt */
    switch (flank) {
    case GPIO_RISING:
        GPIO_IntConfig(port, pin, true, false, true);
        break;
    case GPIO_FALLING:
        GPIO_IntConfig(port, pin, false, true, true);
        break;
    case GPIO_BOTH:
        GPIO_IntConfig(port, pin, true, true, true);
        break;
    }

    gpio_irq_enable(dev);

    return 0;
}

void gpio_irq_enable(gpio_t dev)
{
	DEBUG("gpio_irq_enable %i\n", (int)dev);
    switch (dev) {
#if GPIO_0_EN
    case GPIO_0:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_0_PIN, 1);
        break;
#endif
#if GPIO_1_EN
    case GPIO_1:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_1_PIN, 1);
        break;
#endif
#if GPIO_2_EN
    case GPIO_2:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_2_PIN, 1);
        break;
#endif
#if GPIO_3_EN
    case GPIO_3:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_3_PIN, 1);
        break;
#endif
#if GPIO_4_EN
    case GPIO_4:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_4_PIN, 1);
        break;
#endif
#if GPIO_5_EN
    case GPIO_5:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_5_PIN, 1);
        break;
#endif
#if GPIO_6_EN
    case GPIO_6:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_6_PIN, 1);
        break;
#endif
#if GPIO_7_EN
    case GPIO_7:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_7_PIN, 1);
        break;
#endif
#if GPIO_8_EN
    case GPIO_8:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_8_PIN, 1);
        break;
#endif
#if GPIO_9_EN
    case GPIO_9:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_9_PIN, 1);
        break;
#endif
#if GPIO_10_EN
    case GPIO_10:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_10_PIN , 1);
        break;
#endif
#if GPIO_11_EN
    case GPIO_11:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_11_PIN, 1);
        break;
#endif
#if GPIO_12_EN
    case GPIO_12:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_12_PIN, 1);
        break;
#endif
#if GPIO_13_EN
    case GPIO_13:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_13_PIN, 1);
        break;
#endif
#if GPIO_14_EN
    case GPIO_14:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_14_PIN, 1);
        break;
#endif
#if GPIO_15_EN
    case GPIO_15:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_15_PIN, 1);
        break;
#endif
#if GPIO_16_EN
    case GPIO_16:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_16_PIN, 1);
        break;
#endif
#if GPIO_17_EN
    case GPIO_17:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_17_PIN, 1);
        break;
#endif
#if GPIO_18_EN
    case GPIO_18:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_18_PIN, 1);
        break;
#endif
#if GPIO_19_EN
    case GPIO_19:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_19_PIN, 1);
        break;
#endif
#if GPIO_20_EN
    case GPIO_20:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_20_PIN, 1);
        break;
#endif
#if GPIO_21_EN
    case GPIO_21:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_21_PIN, 1);
        break;
#endif
#if GPIO_22_EN
    case GPIO_22:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_22_PIN, 1);
        break;
#endif
#if GPIO_23_EN
    case GPIO_23:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_23_PIN, 1);
        break;
#endif
#if GPIO_24_EN
    case GPIO_24:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_24_PIN, 1);
        break;
#endif
#if GPIO_25_EN
    case GPIO_25:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_25_PIN, 1);
        break;
#endif
#if GPIO_26_EN
    case GPIO_26:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_26_PIN, 1);
        break;
#endif

    }
}

void gpio_irq_disable(gpio_t dev)
{
	DEBUG("gpio_irq_disable %i\n", (int)dev);
	switch (dev) {
#if GPIO_0_EN
    case GPIO_0:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_0_PIN, 0);
        break;
#endif
#if GPIO_1_EN
    case GPIO_1:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_1_PIN, 0);
        break;
#endif
#if GPIO_2_EN
    case GPIO_2:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_2_PIN, 0);
        break;
#endif
#if GPIO_3_EN
    case GPIO_3:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_3_PIN, 0);
        break;
#endif
#if GPIO_4_EN
    case GPIO_4:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_4_PIN, 0);
        break;
#endif
#if GPIO_5_EN
    case GPIO_5:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_5_PIN, 0);
        break;
#endif
#if GPIO_6_EN
    case GPIO_6:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_6_PIN, 0);
        break;
#endif
#if GPIO_7_EN
    case GPIO_7:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_7_PIN, 0);
        break;
#endif
#if GPIO_8_EN
    case GPIO_8:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_8_PIN, 0);
        break;
#endif
#if GPIO_9_EN
    case GPIO_9:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_9_PIN, 0);
        break;
#endif
#if GPIO_10_EN
    case GPIO_10:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_10_PIN , 0);
        break;
#endif
#if GPIO_11_EN
    case GPIO_11:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_11_PIN, 0);
        break;
#endif
#if GPIO_12_EN
    case GPIO_12:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_12_PIN, 0);
        break;
#endif
#if GPIO_13_EN
    case GPIO_13:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_13_PIN, 0);
        break;
#endif
#if GPIO_14_EN
    case GPIO_14:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_14_PIN, 0);
        break;
#endif
#if GPIO_15_EN
    case GPIO_15:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_15_PIN, 0);
        break;
#endif
#if GPIO_16_EN
    case GPIO_16:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_16_PIN, 0);
        break;
#endif
#if GPIO_17_EN
    case GPIO_17:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_17_PIN, 0);
        break;
#endif
#if GPIO_18_EN
    case GPIO_18:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_18_PIN, 0);
        break;
#endif
#if GPIO_19_EN
    case GPIO_19:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_19_PIN, 0);
        break;
#endif
#if GPIO_20_EN
    case GPIO_20:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_20_PIN, 0);
        break;
#endif
#if GPIO_21_EN
    case GPIO_21:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_21_PIN, 0);
        break;
#endif
#if GPIO_22_EN
    case GPIO_22:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_22_PIN, 0);
        break;
#endif
#if GPIO_23_EN
    case GPIO_23:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_23_PIN, 0);
        break;
#endif
#if GPIO_24_EN
    case GPIO_24:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_24_PIN, 0);
        break;
#endif
#if GPIO_25_EN
    case GPIO_25:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_25_PIN, 0);
        break;
#endif
#if GPIO_26_EN
    case GPIO_26:
        BITBAND_Peripheral(&(GPIO->IEN), GPIO_26_PIN, 0);
        break;
#endif

    }
}

int gpio_read(gpio_t dev)
{
    GPIO_Port_TypeDef port = 0;
    uint32_t pin = 0;

    switch (dev) {
#if GPIO_0_EN
    case GPIO_0:
        port = GPIO_0_PORT;
        pin = GPIO_0_PIN;
        break;
#endif
#if GPIO_1_EN
    case GPIO_1:
        port = GPIO_1_PORT;
        pin = GPIO_1_PIN;
        break;
#endif
#if GPIO_2_EN
    case GPIO_2:
        port = GPIO_2_PORT;
        pin = GPIO_2_PIN;
        break;
#endif
#if GPIO_3_EN
    case GPIO_3:
        port = GPIO_3_PORT;
        pin = GPIO_3_PIN;
        break;
#endif
#if GPIO_4_EN
    case GPIO_4:
        port = GPIO_4_PORT;
        pin = GPIO_4_PIN;
        break;
#endif
#if GPIO_5_EN
    case GPIO_5:
        port = GPIO_5_PORT;
        pin = GPIO_5_PIN;
        break;
#endif
#if GPIO_6_EN
    case GPIO_6:
        port = GPIO_6_PORT;
        pin = GPIO_6_PIN;
        break;
#endif
#if GPIO_7_EN
    case GPIO_7:
        port = GPIO_7_PORT;
        pin = GPIO_7_PIN;
        break;
#endif
#if GPIO_8_EN
    case GPIO_8:
        port = GPIO_8_PORT;
        pin = GPIO_8_PIN;
        break;
#endif
#if GPIO_9_EN
    case GPIO_9:
        port = GPIO_9_PORT;
        pin = GPIO_9_PIN;
        break;
#endif
#if GPIO_10_EN
    case GPIO_10:
        port = GPIO_10_PORT;
        pin = GPIO_10_PIN;
        break;
#endif
#if GPIO_11_EN
    case GPIO_11:
        port = GPIO_11_PORT;
        pin = GPIO_11_PIN;
        break;
#endif
#if GPIO_12_EN
    case GPIO_12:
        port = GPIO_12_PORT;
        pin = GPIO_12_PIN;
        break;
#endif
#if GPIO_13_EN
    case GPIO_13:
        port = GPIO_13_PORT;
        pin = GPIO_13_PIN;
        break;
#endif
#if GPIO_14_EN
    case GPIO_14:
        port = GPIO_14_PORT;
        pin = GPIO_14_PIN;
        break;
#endif
#if GPIO_15_EN
    case GPIO_15:
        port = GPIO_15_PORT;
        pin = GPIO_15_PIN;
        break;
#endif
#if GPIO_16_EN
    case GPIO_16:
        port = GPIO_16_PORT;
        pin = GPIO_16_PIN;
        break;
#endif
#if GPIO_17_EN
    case GPIO_17:
        port = GPIO_17_PORT;
        pin = GPIO_17_PIN;
        break;
#endif
#if GPIO_18_EN
    case GPIO_18:
        port = GPIO_18_PORT;
        pin = GPIO_18_PIN;
        break;
#endif
#if GPIO_19_EN
    case GPIO_19:
        port = GPIO_19_PORT;
        pin = GPIO_19_PIN;
        break;
#endif
#if GPIO_20_EN
    case GPIO_20:
        port = GPIO_20_PORT;
        pin = GPIO_20_PIN;
        break;
#endif
#if GPIO_21_EN
    case GPIO_21:
        port = GPIO_21_PORT;
        pin = GPIO_21_PIN;
        break;
#endif
#if GPIO_22_EN
    case GPIO_22:
        port = GPIO_22_PORT;
        pin = GPIO_22_PIN;
        break;
#endif
#if GPIO_23_EN
    case GPIO_23:
        port = GPIO_23_PORT;
        pin = GPIO_23_PIN;
        break;
#endif
#if GPIO_24_EN
    case GPIO_24:
        port = GPIO_24_PORT;
        pin = GPIO_24_PIN;
        break;
#endif
#if GPIO_25_EN
    case GPIO_25:
        port = GPIO_25_PORT;
        pin = GPIO_25_PIN;
        break;
#endif
#if GPIO_26_EN
    case GPIO_26:
        port = GPIO_26_PORT;
        pin = GPIO_26_PIN;
        break;
#endif

    }

    if (GPIO->P[port].DOUT & (1 << pin)) {
        return GPIO_PinOutGet(port, pin);
    } else {
        return GPIO_PinInGet(port, pin);
    }
}

void gpio_set(gpio_t dev)
{
    GPIO_Port_TypeDef port = 0;
    uint32_t pin = 0;

    switch (dev) {
#if GPIO_0_EN
    case GPIO_0:
        port = GPIO_0_PORT;
        pin = GPIO_0_PIN;
        break;
#endif
#if GPIO_1_EN
    case GPIO_1:
        port = GPIO_1_PORT;
        pin = GPIO_1_PIN;
        break;
#endif
#if GPIO_2_EN
    case GPIO_2:
        port = GPIO_2_PORT;
        pin = GPIO_2_PIN;
        break;
#endif
#if GPIO_3_EN
    case GPIO_3:
        port = GPIO_3_PORT;
        pin = GPIO_3_PIN;
        break;
#endif
#if GPIO_4_EN
    case GPIO_4:
        port = GPIO_4_PORT;
        pin = GPIO_4_PIN;
        break;
#endif
#if GPIO_5_EN
    case GPIO_5:
        port = GPIO_5_PORT;
        pin = GPIO_5_PIN;
        break;
#endif
#if GPIO_6_EN
    case GPIO_6:
        port = GPIO_6_PORT;
        pin = GPIO_6_PIN;
        break;
#endif
#if GPIO_7_EN
    case GPIO_7:
        port = GPIO_7_PORT;
        pin = GPIO_7_PIN;
        break;
#endif
#if GPIO_8_EN
    case GPIO_8:
        port = GPIO_8_PORT;
        pin = GPIO_8_PIN;
        break;
#endif
#if GPIO_9_EN
    case GPIO_9:
        port = GPIO_9_PORT;
        pin = GPIO_9_PIN;
        break;
#endif
#if GPIO_10_EN
    case GPIO_10:
        port = GPIO_10_PORT;
        pin = GPIO_10_PIN;
        break;
#endif
#if GPIO_11_EN
    case GPIO_11:
        port = GPIO_11_PORT;
        pin = GPIO_11_PIN;
        break;
#endif
#if GPIO_12_EN
    case GPIO_12:
        port = GPIO_12_PORT;
        pin = GPIO_12_PIN;
        break;
#endif
#if GPIO_13_EN
    case GPIO_13:
        port = GPIO_13_PORT;
        pin = GPIO_13_PIN;
        break;
#endif
#if GPIO_14_EN
    case GPIO_14:
        port = GPIO_14_PORT;
        pin = GPIO_14_PIN;
        break;
#endif
#if GPIO_15_EN
    case GPIO_15:
        port = GPIO_15_PORT;
        pin = GPIO_15_PIN;
        break;
#endif
#if GPIO_16_EN
    case GPIO_16:
        port = GPIO_16_PORT;
        pin = GPIO_16_PIN;
        break;
#endif
#if GPIO_17_EN
    case GPIO_17:
        port = GPIO_17_PORT;
        pin = GPIO_17_PIN;
        break;
#endif
#if GPIO_18_EN
    case GPIO_18:
        port = GPIO_18_PORT;
        pin = GPIO_18_PIN;
        break;
#endif
#if GPIO_19_EN
    case GPIO_19:
        port = GPIO_19_PORT;
        pin = GPIO_19_PIN;
        break;
#endif
#if GPIO_20_EN
    case GPIO_20:
        port = GPIO_20_PORT;
        pin = GPIO_20_PIN;
        break;
#endif
#if GPIO_21_EN
    case GPIO_21:
        port = GPIO_21_PORT;
        pin = GPIO_21_PIN;
        break;
#endif
#if GPIO_22_EN
    case GPIO_22:
        port = GPIO_22_PORT;
        pin = GPIO_22_PIN;
        break;
#endif
#if GPIO_23_EN
    case GPIO_23:
        port = GPIO_23_PORT;
        pin = GPIO_23_PIN;
        break;
#endif
#if GPIO_24_EN
    case GPIO_24:
        port = GPIO_24_PORT;
        pin = GPIO_24_PIN;
        break;
#endif
#if GPIO_25_EN
    case GPIO_25:
        port = GPIO_25_PORT;
        pin = GPIO_25_PIN;
        break;
#endif
#if GPIO_26_EN
    case GPIO_26:
        port = GPIO_26_PORT;
        pin = GPIO_26_PIN;
        break;
#endif

    }

    GPIO_PinOutSet(port, pin);
}

void gpio_clear(gpio_t dev)
{
    GPIO_Port_TypeDef port = 0;
    uint32_t pin = 0;

    switch (dev) {
#if GPIO_0_EN
    case GPIO_0:
        port = GPIO_0_PORT;
        pin = GPIO_0_PIN;
        break;
#endif
#if GPIO_1_EN
    case GPIO_1:
        port = GPIO_1_PORT;
        pin = GPIO_1_PIN;
        break;
#endif
#if GPIO_2_EN
    case GPIO_2:
        port = GPIO_2_PORT;
        pin = GPIO_2_PIN;
        break;
#endif
#if GPIO_3_EN
    case GPIO_3:
        port = GPIO_3_PORT;
        pin = GPIO_3_PIN;
        break;
#endif
#if GPIO_4_EN
    case GPIO_4:
        port = GPIO_4_PORT;
        pin = GPIO_4_PIN;
        break;
#endif
#if GPIO_5_EN
    case GPIO_5:
        port = GPIO_5_PORT;
        pin = GPIO_5_PIN;
        break;
#endif
#if GPIO_6_EN
    case GPIO_6:
        port = GPIO_6_PORT;
        pin = GPIO_6_PIN;
        break;
#endif
#if GPIO_7_EN
    case GPIO_7:
        port = GPIO_7_PORT;
        pin = GPIO_7_PIN;
        break;
#endif
#if GPIO_8_EN
    case GPIO_8:
        port = GPIO_8_PORT;
        pin = GPIO_8_PIN;
        break;
#endif
#if GPIO_9_EN
    case GPIO_9:
        port = GPIO_9_PORT;
        pin = GPIO_9_PIN;
        break;
#endif
#if GPIO_10_EN
    case GPIO_10:
        port = GPIO_10_PORT;
        pin = GPIO_10_PIN;
        break;
#endif
#if GPIO_11_EN
    case GPIO_11:
        port = GPIO_11_PORT;
        pin = GPIO_11_PIN;
        break;
#endif
#if GPIO_12_EN
    case GPIO_12:
        port = GPIO_12_PORT;
        pin = GPIO_12_PIN;
        break;
#endif
#if GPIO_13_EN
    case GPIO_13:
        port = GPIO_13_PORT;
        pin = GPIO_13_PIN;
        break;
#endif
#if GPIO_14_EN
    case GPIO_14:
        port = GPIO_14_PORT;
        pin = GPIO_14_PIN;
        break;
#endif
#if GPIO_15_EN
    case GPIO_15:
        port = GPIO_15_PORT;
        pin = GPIO_15_PIN;
        break;
#endif
#if GPIO_16_EN
    case GPIO_16:
        port = GPIO_16_PORT;
        pin = GPIO_16_PIN;
        break;
#endif
#if GPIO_17_EN
    case GPIO_17:
        port = GPIO_17_PORT;
        pin = GPIO_17_PIN;
        break;
#endif
#if GPIO_18_EN
    case GPIO_18:
        port = GPIO_18_PORT;
        pin = GPIO_18_PIN;
        break;
#endif
#if GPIO_19_EN
    case GPIO_19:
        port = GPIO_19_PORT;
        pin = GPIO_19_PIN;
        break;
#endif
#if GPIO_20_EN
    case GPIO_20:
        port = GPIO_20_PORT;
        pin = GPIO_20_PIN;
        break;
#endif
#if GPIO_21_EN
    case GPIO_21:
        port = GPIO_21_PORT;
        pin = GPIO_21_PIN;
        break;
#endif
#if GPIO_22_EN
    case GPIO_22:
        port = GPIO_22_PORT;
        pin = GPIO_22_PIN;
        break;
#endif
#if GPIO_23_EN
    case GPIO_23:
        port = GPIO_23_PORT;
        pin = GPIO_23_PIN;
        break;
#endif
#if GPIO_24_EN
    case GPIO_24:
        port = GPIO_24_PORT;
        pin = GPIO_24_PIN;
        break;
#endif
#if GPIO_25_EN
    case GPIO_25:
        port = GPIO_25_PORT;
        pin = GPIO_25_PIN;
        break;
#endif
#if GPIO_26_EN
    case GPIO_26:
        port = GPIO_26_PORT;
        pin = GPIO_26_PIN;
        break;
#endif

    }

    GPIO_PinOutClear(port, pin);
}

void gpio_toggle(gpio_t dev)
{
    GPIO_Port_TypeDef port = 0;
    uint32_t pin = 0;

    switch (dev) {
#if GPIO_0_EN
    case GPIO_0:
        port = GPIO_0_PORT;
        pin = GPIO_0_PIN;
        break;
#endif
#if GPIO_1_EN
    case GPIO_1:
        port = GPIO_1_PORT;
        pin = GPIO_1_PIN;
        break;
#endif
#if GPIO_2_EN
    case GPIO_2:
        port = GPIO_2_PORT;
        pin = GPIO_2_PIN;
        break;
#endif
#if GPIO_3_EN
    case GPIO_3:
        port = GPIO_3_PORT;
        pin = GPIO_3_PIN;
        break;
#endif
#if GPIO_4_EN
    case GPIO_4:
        port = GPIO_4_PORT;
        pin = GPIO_4_PIN;
        break;
#endif
#if GPIO_5_EN
    case GPIO_5:
        port = GPIO_5_PORT;
        pin = GPIO_5_PIN;
        break;
#endif
#if GPIO_6_EN
    case GPIO_6:
        port = GPIO_6_PORT;
        pin = GPIO_6_PIN;
        break;
#endif
#if GPIO_7_EN
    case GPIO_7:
        port = GPIO_7_PORT;
        pin = GPIO_7_PIN;
        break;
#endif
#if GPIO_8_EN
    case GPIO_8:
        port = GPIO_8_PORT;
        pin = GPIO_8_PIN;
        break;
#endif
#if GPIO_9_EN
    case GPIO_9:
        port = GPIO_9_PORT;
        pin = GPIO_9_PIN;
        break;
#endif
#if GPIO_10_EN
    case GPIO_10:
        port = GPIO_10_PORT;
        pin = GPIO_10_PIN;
        break;
#endif
#if GPIO_11_EN
    case GPIO_11:
        port = GPIO_11_PORT;
        pin = GPIO_11_PIN;
        break;
#endif
#if GPIO_12_EN
    case GPIO_12:
        port = GPIO_13_PORT;
        pin = GPIO_12_PIN;
        break;
#endif
#if GPIO_13_EN
    case GPIO_13:
        port = GPIO_13_PORT;
        pin = GPIO_13_PIN;
        break;
#endif
#if GPIO_14_EN
    case GPIO_14:
        port = GPIO_14_PORT;
        pin = GPIO_14_PIN;
        break;
#endif
#if GPIO_15_EN
    case GPIO_15:
        port = GPIO_15_PORT;
        pin = GPIO_15_PIN;
        break;
#endif
#if GPIO_16_EN
    case GPIO_16:
        port = GPIO_16_PORT;
        pin = GPIO_16_PIN;
        break;
#endif
#if GPIO_17_EN
    case GPIO_17:
        port = GPIO_17_PORT;
        pin = GPIO_17_PIN;
        break;
#endif
#if GPIO_18_EN
    case GPIO_18:
        port = GPIO_18_PORT;
        pin = GPIO_18_PIN;
        break;
#endif
#if GPIO_19_EN
    case GPIO_19:
        port = GPIO_19_PORT;
        pin = GPIO_19_PIN;
        break;
#endif
#if GPIO_20_EN
    case GPIO_20:
        port = GPIO_20_PORT;
        pin = GPIO_20_PIN;
        break;
#endif
#if GPIO_21_EN
    case GPIO_21:
        port = GPIO_21_PORT;
        pin = GPIO_21_PIN;
        break;
#endif
#if GPIO_22_EN
    case GPIO_22:
        port = GPIO_22_PORT;
        pin = GPIO_22_PIN;
        break;
#endif
#if GPIO_23_EN
    case GPIO_23:
        port = GPIO_23_PORT;
        pin = GPIO_23_PIN;
        break;
#endif
#if GPIO_24_EN
    case GPIO_24:
        port = GPIO_24_PORT;
        pin = GPIO_24_PIN;
        break;
#endif
#if GPIO_25_EN
    case GPIO_25:
        port = GPIO_25_PORT;
        pin = GPIO_25_PIN;
        break;
#endif
#if GPIO_26_EN
    case GPIO_26:
        port = GPIO_26_PORT;
        pin = GPIO_26_PIN;
        break;
#endif

    }
    GPIO_PinOutToggle(port, pin);
}

void gpio_write(gpio_t dev, int value)
{
    if (value) {
        gpio_set(dev);
    } else {
        gpio_clear(dev);
    }
}

static inline void gpio_irq_handler(gpio_t dev)
{
    if(gpio_config[dev].cb != NULL) {
		DEBUG("gpio_irq_callback(): calling: %i\n", (int)dev);
        gpio_config[dev].cb(gpio_config[dev].arg);
    } else {
        DEBUG("gpio_irq_callback(): handler not bound: %i\n", (int)dev);
    }
}


void __attribute__((interrupt("IRQ"))) GPIO_EVEN_IRQHandler(void)
{
//     ISR_ENTER();

	if(lpm_get() >= LPM_SLEEP)
		lpm_awake();

    uint32_t iflags;

    /* Get all even interrupts. */
    iflags = GPIO_IntGetEnabled() & 0x00005555;
//     printf("e%x\n", iflags);

	/* Clean only even interrupts. */
	GPIO_IntClear(iflags);

    /* Call IRQ handlers */
#if GPIO_0_EN
    if (iflags & (1 << GPIO_IRQ_0)) {
        //GPIO->IFC = (1 << GPIO_IRQ_0);
        gpio_irq_handler(GPIO_0);
    }
#endif
#if GPIO_1_EN
    if (iflags & (1 << GPIO_IRQ_1)) {
        //GPIO->IFC = (1 << GPIO_IRQ_1);
        gpio_irq_handler(GPIO_1);
    }
#endif
#if GPIO_2_EN
    if (iflags & (1 << GPIO_IRQ_2)) {
        //GPIO->IFC = (1 << GPIO_IRQ_2);
        gpio_irq_handler(GPIO_2);
    }
#endif
#if GPIO_3_EN
if (iflags & (1 << GPIO_IRQ_3)) {
	//GPIO->IFC = (1 << GPIO_IRQ_3);
	gpio_irq_handler(GPIO_3);
}
#endif
#if GPIO_4_EN
if (iflags & (1 << GPIO_IRQ_4)) {
	//GPIO->IFC = (1 << GPIO_IRQ_4);
	gpio_irq_handler(GPIO_4);
}
#endif
#if GPIO_5_EN
if (iflags & (1 << GPIO_IRQ_5)) {
	//GPIO->IFC = (1 << GPIO_IRQ_5);
	gpio_irq_handler(GPIO_5);
}
#endif
#if GPIO_6_EN
if (iflags & (1 << GPIO_IRQ_6)) {
	gpio_irq_handler(GPIO_6);
}
#endif
#if GPIO_7_EN
if (iflags & (1 << GPIO_IRQ_7)) {
	gpio_irq_handler(GPIO_7);
}
#endif
#if GPIO_8_EN
if (iflags & (1 << GPIO_IRQ_8)) {
	gpio_irq_handler(GPIO_8);
}
#endif
#if GPIO_9_EN
if (iflags & (1 << GPIO_IRQ_9)) {
	gpio_irq_handler(GPIO_9);
}
#endif
#if GPIO_10_EN
if (iflags & (1 << GPIO_IRQ_10)) {
	gpio_irq_handler(GPIO_10);
}
#endif
#if GPIO_11_EN
if (iflags & (1 << GPIO_IRQ_11)) {
	gpio_irq_handler(GPIO_11);
}
#endif
#if GPIO_12_EN
if (iflags & (1 << GPIO_IRQ_12)) {
	gpio_irq_handler(GPIO_12);
}
#endif
#if GPIO_13_EN
if (iflags & (1 << GPIO_IRQ_13)) {
	gpio_irq_handler(GPIO_13);
}
#endif
#if GPIO_14_EN
if (iflags & (1 << GPIO_IRQ_14)) {
    gpio_irq_handler(GPIO_14);
}
#endif
#if GPIO_15_EN
if (iflags & (1 << GPIO_IRQ_15)) {
    gpio_irq_handler(GPIO_15);
}
#endif
#if GPIO_16_EN
if (iflags & (1 << GPIO_IRQ_16)) {
    gpio_irq_handler(GPIO_16);
}
#endif
#if GPIO_17_EN
if (iflags & (1 << GPIO_IRQ_17)) {
    gpio_irq_handler(GPIO_17);
}
#endif
#if GPIO_18_EN
if (iflags & (1 << GPIO_IRQ_18)) {
    gpio_irq_handler(GPIO_18);
}
#endif
#if GPIO_19_EN
if (iflags & (1 << GPIO_IRQ_19)) {
    gpio_irq_handler(GPIO_19);
}
#endif
#if GPIO_20_EN
if (iflags & (1 << GPIO_IRQ_20)) {
    gpio_irq_handler(GPIO_20);
}
#endif
#if GPIO_21_EN
if (iflags & (1 << GPIO_IRQ_21)) {
    gpio_irq_handler(GPIO_21);
}
#endif
#if GPIO_22_EN
if (iflags & (1 << GPIO_IRQ_22)) {
    gpio_irq_handler(GPIO_22);
}
#endif
#if GPIO_23_EN
if (iflags & (1 << GPIO_IRQ_23)) {
    gpio_irq_handler(GPIO_23);
}
#endif
#if GPIO_24_EN
if (iflags & (1 << GPIO_IRQ_24)) {
    gpio_irq_handler(GPIO_24);
}
#endif
#if GPIO_25_EN
if (iflags & (1 << GPIO_IRQ_25)) {
    gpio_irq_handler(GPIO_25);
}
#endif
#if GPIO_26_EN
if (iflags & (1 << GPIO_IRQ_26)) {
    gpio_irq_handler(GPIO_26);
}
#endif

    if (sched_context_switch_request) {
        thread_yield();
    }

//     ISR_EXIT();
}


void __attribute__((interrupt("IRQ"))) GPIO_ODD_IRQHandler(void)
{
//     ISR_ENTER();

	if(lpm_get() >= LPM_SLEEP)
		lpm_awake();

    uint32_t iflags;

    /* Get all odd interrupts. */
    iflags = GPIO_IntGetEnabled() & 0x0000AAAA;
//     printf("o%x\n", iflags);

	/* Clean only even interrupts. */
	GPIO_IntClear(iflags);

    /* Call IRQ handlers */
#if GPIO_0_EN
    if (iflags & (1 << GPIO_IRQ_0)) {
        //GPIO->IFC = (1 << GPIO_IRQ_0);
        gpio_irq_handler(GPIO_0);
    }
#endif
#if GPIO_1_EN
    if (iflags & (1 << GPIO_IRQ_1)) {
        //GPIO->IFC = (1 << GPIO_IRQ_1);
        gpio_irq_handler(GPIO_1);
    }
#endif
#if GPIO_2_EN
    if (iflags & (1 << GPIO_IRQ_2)) {
        //GPIO->IFC = (1 << GPIO_IRQ_2);
        gpio_irq_handler(GPIO_2);
    }
#endif
#if GPIO_3_EN
if (iflags & (1 << GPIO_IRQ_3)) {
	//GPIO->IFC = (1 << GPIO_IRQ_3);
	gpio_irq_handler(GPIO_3);
}
#endif
#if GPIO_4_EN
if (iflags & (1 << GPIO_IRQ_4)) {
	//GPIO->IFC = (1 << GPIO_IRQ_4);
	gpio_irq_handler(GPIO_4);
}
#endif
#if GPIO_5_EN
if (iflags & (1 << GPIO_IRQ_5)) {
	//GPIO->IFC = (1 << GPIO_IRQ_5);
	gpio_irq_handler(GPIO_5);
}
#endif
#if GPIO_6_EN
if (iflags & (1 << GPIO_IRQ_6)) {
	gpio_irq_handler(GPIO_6);
}
#endif
#if GPIO_7_EN
if (iflags & (1 << GPIO_IRQ_7)) {
	gpio_irq_handler(GPIO_7);
}
#endif
#if GPIO_8_EN
if (iflags & (1 << GPIO_IRQ_8)) {
	gpio_irq_handler(GPIO_8);
}
#endif
#if GPIO_9_EN
if (iflags & (1 << GPIO_IRQ_9)) {
	gpio_irq_handler(GPIO_9);
}
#endif
#if GPIO_10_EN
if (iflags & (1 << GPIO_IRQ_10)) {
	gpio_irq_handler(GPIO_10);
}
#endif
#if GPIO_11_EN
if (iflags & (1 << GPIO_IRQ_11)) {
	gpio_irq_handler(GPIO_11);
}
#endif
#if GPIO_12_EN
if (iflags & (1 << GPIO_IRQ_12)) {
	gpio_irq_handler(GPIO_12);
}
#endif
#if GPIO_13_EN
if (iflags & (1 << GPIO_IRQ_13)) {
	gpio_irq_handler(GPIO_13);
}
#endif
#if GPIO_14_EN
if (iflags & (1 << GPIO_IRQ_14)) {
    gpio_irq_handler(GPIO_14);
}
#endif
#if GPIO_15_EN
if (iflags & (1 << GPIO_IRQ_15)) {
    gpio_irq_handler(GPIO_15);
}
#endif
#if GPIO_16_EN
if (iflags & (1 << GPIO_IRQ_16)) {
    gpio_irq_handler(GPIO_16);
}
#endif
#if GPIO_17_EN
if (iflags & (1 << GPIO_IRQ_17)) {
    gpio_irq_handler(GPIO_17);
}
#endif
#if GPIO_18_EN
if (iflags & (1 << GPIO_IRQ_18)) {
    gpio_irq_handler(GPIO_18);
}
#endif
#if GPIO_19_EN
if (iflags & (1 << GPIO_IRQ_19)) {
    gpio_irq_handler(GPIO_19);
}
#endif
#if GPIO_20_EN
if (iflags & (1 << GPIO_IRQ_20)) {
    gpio_irq_handler(GPIO_20);
}
#endif
#if GPIO_21_EN
if (iflags & (1 << GPIO_IRQ_21)) {
    gpio_irq_handler(GPIO_21);
}
#endif
#if GPIO_22_EN
if (iflags & (1 << GPIO_IRQ_22)) {
    gpio_irq_handler(GPIO_22);
}
#endif
#if GPIO_23_EN
if (iflags & (1 << GPIO_IRQ_23)) {
    gpio_irq_handler(GPIO_23);
}
#endif
#if GPIO_24_EN
if (iflags & (1 << GPIO_IRQ_24)) {
    gpio_irq_handler(GPIO_24);
}
#endif
#if GPIO_25_EN
if (iflags & (1 << GPIO_IRQ_25)) {
    gpio_irq_handler(GPIO_25);
}
#endif
#if GPIO_26_EN
if (iflags & (1 << GPIO_IRQ_26)) {
    gpio_irq_handler(GPIO_26);
}
#endif

    if (sched_context_switch_request) {
        thread_yield();
    }

//     ISR_EXIT();
}


#endif /* GPIO_NUMOF */
