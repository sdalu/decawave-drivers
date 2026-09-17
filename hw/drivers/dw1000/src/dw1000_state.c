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
 * Optional component (DW1000_SOURCES_STATE in dw1000.cmake). Every value
 * comes from a register read, not from dw1000_config_t.
 */

#include <stdio.h>
#include <string.h>

#include <dw1000/dw1000_state.h>
#include <dw1000/dw1000_reg.h>

/* TXPSR and PE are one 4-bit field to the chip, and the driver's own
 * DW1000_PLEN_* constants ARE the encoded values (dw1000.h:559-567), so
 * the decode is a lookup against those same constants rather than a
 * second table that could disagree with them. */
static uint16_t
plen_symbols(uint8_t encoded)
{
    switch (encoded) {
    case DW1000_PLEN_64:   return   64;
    case DW1000_PLEN_1024: return 1024;
    case DW1000_PLEN_4096: return 4096;
#if DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH
    /* The other five are proprietary and their constants only exist when
     * that option is on. The CHIP still reports whatever it holds, so a
     * build without the option decodes those to 0 and the raw tx_fctrl
     * beside it carries the truth, which is the right answer: a value
     * this build cannot name is not one it should name. */
    case DW1000_PLEN_128:  return  128;
    case DW1000_PLEN_256:  return  256;
    case DW1000_PLEN_512:  return  512;
    case DW1000_PLEN_1536: return 1536;
    case DW1000_PLEN_2048: return 2048;
#endif
    default:               return    0;   /* not a value this build names */
    }
}

static uint16_t
bitrate_kbps(uint8_t encoded)
{
    switch (encoded) {
    case DW1000_BITRATE_110KBPS:  return  110;
    case DW1000_BITRATE_850KBPS:  return  850;
    case DW1000_BITRATE_6800KBPS: return 6800;
    default:                      return    0;
    }
}

/* RXPRF is 01 for 16 MHz and 10 for 64 MHz (UM 7.2.19). 00 and 11 are
 * not defined, and are reported as 0 rather than guessed at. */
static uint8_t
prf_mhz(uint8_t encoded)
{
    switch (encoded) {
    case 1:  return 16;
    case 2:  return 64;
    default: return  0;
    }
}

void
dw1000_get_radio_state(dw1000_t *dw, dw1000_radio_state_t *out)
{
    memset(out, 0, sizeof(*out));

    out->chan_ctrl = _dw1000_reg_read32(dw, DW1000_REG_CHAN_CTRL,
                                        DW1000_OFF_NONE);
    out->tx_fctrl  = _dw1000_reg_read32(dw, DW1000_REG_TX_FCTRL,
                                        DW1000_OFF_NONE);
    out->sys_cfg   = _dw1000_reg_read32(dw, DW1000_REG_SYS_CFG,
                                        DW1000_OFF_NONE);
    out->drx_tune2 = _dw1000_reg_read32(dw, DW1000_REG_DRX_CONF,
                                        DW1000_OFF_DRX_TUNE2);
    out->tx_power  = _dw1000_reg_read32(dw, DW1000_REG_TX_POWER,
                                        DW1000_OFF_NONE);

    out->tx_antenna_delay = _dw1000_reg_read16(dw, DW1000_REG_TX_ANTD,
                                               DW1000_OFF_NONE);
    out->rx_antenna_delay = _dw1000_reg_read16(dw, DW1000_REG_LDE_IF,
                                               DW1000_OFF_LDE_RXANTD);

    /* CHAN_CTRL, UM 7.2.19 */
    out->tx_channel = (uint8_t)((out->chan_ctrl &
                                 DW1000_MSK_CHAN_CTRL_TX_CHAN) >> 0);
    out->rx_channel = (uint8_t)((out->chan_ctrl &
                                 DW1000_MSK_CHAN_CTRL_RX_CHAN) >> 4);
    out->tx_pcode   = (uint8_t)((out->chan_ctrl &
                                 DW1000_MSK_CHAN_CTRL_TX_PCODE) >> 22);
    out->rx_pcode   = (uint8_t)((out->chan_ctrl &
                                 DW1000_MSK_CHAN_CTRL_RX_PCODE) >> 27);
    out->rx_prf     = prf_mhz((uint8_t)((out->chan_ctrl &
                                 DW1000_MSK_CHAN_CTRL_RXPRF) >> 18));
    out->dwsfd      = (out->chan_ctrl & DW1000_FLG_CHAN_CTRL_DWSFD)  != 0;
    out->tnssfd     = (out->chan_ctrl & DW1000_FLG_CHAN_CTRL_TNSSFD) != 0;
    out->rnssfd     = (out->chan_ctrl & DW1000_FLG_CHAN_CTRL_RNSSFD) != 0;

    /* TX_FCTRL, UM 7.2.8 */
    out->bitrate_kbps = bitrate_kbps((uint8_t)((out->tx_fctrl &
                                 DW1000_MSK_TX_FCTRL_TXBR) >> 13));
    out->tx_plen      = plen_symbols((uint8_t)((out->tx_fctrl &
                                 DW1000_MSK_TX_FCTRL_PE_TXPSR) >> 18));
}

int
dw1000_radio_state_format(char *buf, size_t len,
                          const dw1000_radio_state_t *st)
{
    /* One topic per line, every line prefixed RADIO, so a caller can
     * emit them through whatever it already uses for output and a
     * reader can grep them out of a console log. Fields are named, not
     * positional: a diff of two nodes should say which value differs,
     * not which column. */
    return snprintf(buf, len,
        "RADIO chan=%u/%u pcode=%u/%u prf=%uM rate=%ukbps plen=%u\n"
        "RADIO sfd=dw:%u,tx:%u,rx:%u antd=tx:%u,rx:%u\n"
        "RADIO sys_cfg=0x%08lx chan_ctrl=0x%08lx tx_fctrl=0x%08lx\n"
        "RADIO drx_tune2=0x%08lx tx_power=0x%08lx",
        st->tx_channel, st->rx_channel, st->tx_pcode, st->rx_pcode,
        st->rx_prf, st->bitrate_kbps, st->tx_plen,
        (unsigned)st->dwsfd, (unsigned)st->tnssfd, (unsigned)st->rnssfd,
        st->tx_antenna_delay, st->rx_antenna_delay,
        (unsigned long)st->sys_cfg, (unsigned long)st->chan_ctrl,
        (unsigned long)st->tx_fctrl,
        (unsigned long)st->drx_tune2, (unsigned long)st->tx_power);
}
