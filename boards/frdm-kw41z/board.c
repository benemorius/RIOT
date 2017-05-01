/*
 * Copyright (C) 2017 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_frdm-kw41z
 * @{
 *
 * @file
 * @brief       Board specific initialization for the FRDM-KW41Z
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"
#include "periph/rtt.h"
#include "mcg.h"
#if 0
/* Opcodes for AT45DB041E not yet implemented */
#include "vfs.h"
#include "fs/devfs.h"
#include "mtd_spi_nor.h"

static mtd_spi_nor_t frdm_nor_dev = {
    .base = {
        .driver = &mtd_spi_nor_driver,
        .page_size = 256,
        .pages_per_sector = 256,
        .sector_count = 8,
    },
    .opcode = &mtd_spi_nor_opcode_at45,
    .spi = FRDM_NOR_SPI_DEV,
    .cs = FRDM_NOR_SPI_CS,
    .addr_width = 4,
    .mode = SPI_MODE_3,
    .clk = SPI_CLK_10MHZ,
};

mtd_dev_t *mtd0 = (mtd_dev_t *)&frdm_nor_dev;

static devfs_t frdm_nor_devfs = {
    .path = "/mtd0",
    .f_op = &mtd_vfs_ops,
    .private_data = &frdm_nor_dev,
};

static inline int frdm_nor_init(void)
{
    int res = mtd_init(mtd0);

    if (res >= 0) {
        /* Register DevFS node */
        devfs_register(&mulle_nor_devfs);
    }

    return res;
}
#endif

static inline void set_lpuart_clock_source(void)
{
    /* Use OSCERCLK (external 32 MHz clock) */
    SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_LPUART0SRC_MASK) | SIM_SOPT2_LPUART0SRC(2);
}

void board_init(void)
{
    /* initialize the CPU core */
    cpu_init();

    set_lpuart_clock_source();
    /* Start the RTT, used as time base for xtimer */
    rtt_init();

    /* initialize and turn off LED3 (red onboard LED) */
    gpio_init(LED0_PIN, GPIO_OUT);
    gpio_set(LED0_PIN);
    /* frdm_nor_init(); */
}
