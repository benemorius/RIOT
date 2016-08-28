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
 * @ingroup     cpu_cc26x0
 * @{
 *
 * @file
 * @brief       Internal interfaces for the cc26x0_rf driver
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef CC26X0_RF_INTERNAL_H_
#define CC26X0_RF_INTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CC2538_RX_FIFO_ADDR         0x40088000
#define CC2538_TX_FIFO_ADDR         0x40088200

/**
 * @brief   Read a single byte from the RX FIFO
 *
 * This function will return the first currently unread byte
 * from the RX FIFO, and advance the FIFO pointer by one. Hence,
 * subsequent calls to this function will return subsequent bytes
 * from the RX FIFO.
 *
 * @return  The first currently unread byte from the RX FIFO
 */
uint_fast8_t rfcore_read_byte(void);

/**
 * @brief   Peek a single byte from the RX FIFO
 *
 * Peeking, as opposed to reading, a byte from the RX FIFO
 * will not advance the FIFO pointer. Further, bytes may be
 * read from any position in the FIFO by providing an index.
 *
 * @param[in]  idx  The index of the byte to peek
 *
 * @return          The byte at index idx
 */
uint_fast8_t rfcore_peek_rx_fifo(int idx);

/**
 * @brief   Read the remaining unread data from the RX FIFO
 *
 * @param[out] buf          The buffer to read the data into
 * @param[in]  len          The maximum length of the buffer
 */
void rfcore_read_fifo(void *buf, uint_fast8_t len);

/**
 * @brief   Issue a command strobe from the CPU to the radio
 *
 * @param[in]  instr          The instruction to issue
 */
void rfcore_strobe(uint_fast8_t instr);

/**
 * @brief   Write a single byte to the next index of the TX FIFO
 *
 * @param[in]  byte          The byte to write
 */
void rfcore_write_byte(uint_fast8_t byte);

/**
 * @brief   Poke a single byte in the TX FIFO
 *
 * Poking, as opposed to writing, a byte to the TX FIFO
 * will not advance the FIFO pointer. Further, bytes may be
 * written to any position in the FIFO by providing an index.
 *
 * @param[in]  idx        The index of the byte to write to
 * @param[in]  byte       The byte to write
 */
void rfcore_poke_tx_fifo(int idx, uint_fast8_t byte);

/**
 * @brief   Write a string of bytes to the TX FIFO
 *
 * @param[in] buf          The buffer containing the data to write
 * @param[in] len          The length of the data to write
 */
void rfcore_write_fifo(const void *buf, uint_fast8_t len);

bool RFCORE_ASSERT_failure(const char *expr, const char *func, int line);

#ifdef __cplusplus
}
#endif

#endif /* CC26X0_RF_INTERNAL_H_ */
/** @} */
