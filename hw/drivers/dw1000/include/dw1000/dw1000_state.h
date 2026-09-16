/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Read the radio configuration back off the chip.
 *
 * Every field here is READ FROM THE DW1000, not copied from dw1000_config_t.
 * Its purpose is only for debugging, and cross-checking configuration.
 */

#ifndef __DW1000_STATE_H__
#define __DW1000_STATE_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <dw1000/dw1000.h>

/**
 * @brief What the chip is actually configured to, read back.
 *
 * Field names follow the DW1000 User Manual register fields they come
 * from, so a reader can check each against §7.2 rather than against this
 * comment.
 */
typedef struct dw1000_radio_state {
    /* CHAN_CTRL (UM §7.2.19) */
    uint8_t  tx_channel;        /**< TX_CHAN                            */
    uint8_t  rx_channel;        /**< RX_CHAN                            */
    uint8_t  tx_pcode;          /**< TX_PCODE, preamble code            */
    uint8_t  rx_pcode;          /**< RX_PCODE                           */
    uint8_t  rx_prf;            /**< RXPRF, as MHz: 16 or 64            */
    bool     dwsfd;             /**< DWSFD, proprietary SFD selected    */
    bool     tnssfd;            /**< TNSSFD, non-standard SFD on TX     */
    bool     rnssfd;            /**< RNSSFD, non-standard SFD on RX     */

    /* TX_FCTRL (UM §7.2.8) */
    uint16_t bitrate_kbps;      /**< TXBR decoded: 110, 850 or 6800     */
    uint16_t tx_plen;           /**< TXPSR/PE decoded: symbols          */
    /* PAC is not read back decoded: it is configured by writing a tuned
     * DRX_TUNE2 word chosen per PAC and PRF (dw1000.c:706), so the chip
     * holds the word and not the size. The word is reported raw below;
     * a differing drx_tune2 between two nodes means a differing PAC or
     * PRF, which is what a cross-platform diff needs to know. */

    /* Antenna delay, the two halves of it (UM §7.2.32, §7.2.47.2) */
    uint16_t tx_antenna_delay;  /**< TX_ANTD                            */
    uint16_t rx_antenna_delay;  /**< LDE_RXANTD                         */

    /* Raw, for anything this struct does not decode */
    uint32_t sys_cfg;           /**< SYS_CFG   (UM §7.2.6)              */
    uint32_t chan_ctrl;         /**< CHAN_CTRL, undecoded               */
    uint32_t tx_fctrl;          /**< TX_FCTRL, undecoded                */
    uint32_t drx_tune2;         /**< DRX_TUNE2 (UM §7.2.40.3), PAC/PRF  */
    uint32_t tx_power;          /**< TX_POWER  (UM §7.2.31)             */
} dw1000_radio_state_t;

/**
 * @brief Read the radio configuration off the chip.
 *
 * SPI traffic: several register reads. Call it at start-up or between
 * exchanges, never inside one -- it takes the same bus an exchange is
 * using, and on a host that serialises the bus it will simply wait.
 *
 * @param[in]  dw   driver context
 * @param[out] out  filled from the chip; untouched fields are zeroed
 */
void dw1000_get_radio_state(dw1000_t *dw, dw1000_radio_state_t *out);

/**
 * @brief Format a state as one line per topic, for diffing.
 *
 * The output is meant to be compared BETWEEN PLATFORMS with diff(1), so
 * the format is fixed and the same code produces it everywhere. Two
 * applications formatting "the same" fields their own way is how a diff
 * fills with noise and stops being read.
 *
 * Writes at most @p len bytes including the terminator, and returns what
 * snprintf() would: the length the full output needed, so a caller can
 * tell truncation from a fit.
 *
 * @param[out] buf  destination
 * @param[in]  len  size of @p buf; DW1000_RADIO_STATE_MAX is always enough
 * @param[in]  st   the state to render
 */
int dw1000_radio_state_format(char *buf, size_t len,
                              const dw1000_radio_state_t *st);

/**
 * @brief Buffer size that dw1000_radio_state_format() never exceeds.
 *
 * Measured, not guessed: every field set to the widest its type can
 * print gives 228 bytes plus the terminator. 256 leaves 27 spare, which
 * is room for one more short field before this has to be revisited.
 */
#define DW1000_RADIO_STATE_MAX 256

#endif
