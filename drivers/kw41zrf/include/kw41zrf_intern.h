/*
 * Copyright (C) 2017 SKF AB
 * Copyright (C) 2016 Phytec Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_kw41zrf
 * @{
 *
 * @file
 * @brief       Internal function interfaces for kw41zrf driver
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 */

#ifndef KW2XRF_INTERN_H
#define KW2XRF_INTERN_H

#include <stdint.h>
#include "kw41zrf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   KW41Z transceiver power modes
 */
typedef enum {
    KW41ZRF_POWER_IDLE = 0, /**< All parts powered */
    KW41ZRF_POWER_DSM, /**< Deep sleep mode */
} kw41zrf_powermode_t;

/**
 * @brief   Timebase settings
 */
typedef enum kw41zrf_timer_timebase {
    KW41ZRF_TIMEBASE_500000HZ = 0b010,
    KW41ZRF_TIMEBASE_250000HZ = 0b011,
    KW41ZRF_TIMEBASE_125000HZ = 0b100,
    KW41ZRF_TIMEBASE_62500HZ  = 0b101,
    KW41ZRF_TIMEBASE_31250HZ  = 0b110,
    KW41ZRF_TIMEBASE_15625HZ  = 0b111,
} kw41zrf_timer_timebase_t;

/**
 * @brief   Mask all transceiver interrupts
 *
 * @param[in] dev       kw41zrf device descriptor
 */
static inline void kw41zrf_mask_irqs(void)
{
    bit_set32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TRCV_MSK_SHIFT);
}

/**
 * @brief   Allow transceiver interrupts
 *
 * @param[in] dev       kw41zrf device descriptor
 */
static inline void kw41zrf_unmask_irqs(void)
{
    bit_clear32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TRCV_MSK_SHIFT);
}

/**
 * @brief   Set the callback function for the radio ISR
 *
 * This callback will be called from ISR context when a radio_1 interrupt occurs
 *
 * @param[in]  cb   Pointer to callback function
 * @param[in]  arg  Argument that will be passed to the callback
 */
void kw41zrf_set_irq_callback(void (*cb)(void *arg), void *arg);

/**
 * @brief   Disable all interrupts on transceiver
 *
 * @param[in] dev       kw41zrf device descriptor
 */
void kw41zrf_disable_interrupts(kw41zrf_t *dev);

/**
 * @brief   Set power mode for device
 *
 * @param[in] dev       kw41zrf device descriptor
 * @param[in] pm        power mode value
 */
void kw41zrf_set_power_mode(kw41zrf_t *dev, kw41zrf_powermode_t pm);

/**
 * @brief
 *
 * @param[in] dev
 *
 * @return
 */
int kw41zrf_can_switch_to_idle(kw41zrf_t *dev);

/**
 * @brief   Initialize the Event Timer Block (up counter)
 *
 * The Event Timer Block provides:
 *   - Abort an RX and CCA sequence at pre-determined time
 *   - Latches "timestamp" value during packet reception
 *   - Initiates timer-triggered sequences
 *
 * @param[in] dev       kw41zrf device descriptor
 * @param[in] tb        timer base value
 */
void kw41zrf_timer_init(kw41zrf_t *dev, kw41zrf_timer_timebase_t tb);

/**
 * @brief   Enable start sequence time
 *
 * @param[in] dev       kw41zrf device descriptor
 */
void kw41zrf_timer2_seq_start_on(kw41zrf_t *dev);

/**
 * @brief   Disable start sequence timer
 *
 * @param[in] dev       kw41zrf device descriptor
 */
void kw41zrf_timer2_seq_start_off(kw41zrf_t *dev);

/**
 * @brief   Enable abort sequence timer
 *
 * @param[in] dev       kw41zrf device descriptor
 */
void kw41zrf_timer3_seq_abort_on(kw41zrf_t *dev);

/**
 * @brief   Disable abort sequence timer
 *
 * @param[in] dev       kw41zrf device descriptor
 */
void kw41zrf_timer3_seq_abort_off(kw41zrf_t *dev);

/**
 * @brief   Use T2CMP or T2PRIMECMP to Trigger Transceiver Operations
 *
 * @param[in] dev       kw41zrf device descriptor
 * @param[in] timeout   timeout value
 */
void kw41zrf_trigger_tx_ops_enable(kw41zrf_t *dev, uint32_t timeout);

/**
 * @brief   Disable Trigger for Transceiver Operations
 *
 * @param[in] dev       kw41zrf device descriptor
 */
void kw41zrf_trigger_tx_ops_disable(kw41zrf_t *dev);

/**
 * @brief   Use T3CMP to Abort an RX operation
 *
 * @param[in] dev       kw41zrf device descriptor
 * @param[in] timeout   timeout value
 */
void kw41zrf_abort_rx_ops_enable(kw41zrf_t *dev, uint32_t timeout);

/**
 * @brief   Disable Trigger to Abort an RX operation
 *
 * @param[in] dev       kw41zrf device descriptor
 */
void kw41zrf_abort_rx_ops_disable(kw41zrf_t *dev);

/**
 * @brief   Returns Timestamp of the actual received packet
 *
 * @param[in] dev       kw41zrf device descriptor
 *
 * @return              timestamp value
 */
uint32_t kw41zrf_get_timestamp(kw41zrf_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* KW2XRF_INTERN_H */
/** @} */
