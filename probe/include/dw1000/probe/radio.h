/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_PROBE_RADIO_H__
#define __DW1000_PROBE_RADIO_H__

/**
 * @file    radio.h
 * @brief   The radio a run uses: its settings, their spelling, their
 *          validation, and the line that says which one a run had.
 *
 * Every setting the chip's radio has, in the units an operator gives it
 * (channel number, kbps, MHz, symbols, device ticks), so that both
 * shells take them the same way and a capture can name them. Transmit
 * power is not here: it has its own spelling (`--power=`, the 0.5 dB
 * grid) and its own read-back, and is passed beside these.
 *
 * VALIDATION IS THE DRIVER'S. Each field goes through
 * <dw1000/dw1000_validate.h>, which says why a value is refused; the
 * combination goes through dw1000_radio_is_valid(), the check
 * dw1000_configure() itself makes. Nothing here restates a rule, so the
 * probe cannot accept a radio the driver refuses or the reverse. All of
 * it happens before the chip is touched.
 *
 * The defaults are this instrument's (probe/DESIGN.md, "Radio
 * configuration parity"): channel 5, 6.8 Mb/s, 64 MHz PRF, 128-symbol
 * preamble, PAC 8, code 10 both ways, the Decawave SFD. The antenna
 * delays have none: they are calibrated per platform, so the shell
 * supplies its own (probe/DESIGN.md, "Antenna delay: read it, do not
 * transcribe it").
 *
 * @addtogroup PROBE
 * @{
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <dw1000/dw1000.h>
#include <dw1000/probe/record.h>

/*===========================================================================*/
/* Settings                                                                  */
/*===========================================================================*/

/**
 * @brief A run's radio, in operator units.
 */
struct dw1000_probe_radio {
    uint8_t  channel;           /**< 1, 2, 3, 4, 5 or 7                  */
    uint16_t bitrate_kbps;      /**< 110, 850 or 6800                    */
    uint8_t  prf_mhz;           /**< 16 or 64                            */
    uint16_t preamble;          /**< preamble length, symbols            */
    uint8_t  pac;               /**< acquisition chunk, symbols          */
    uint8_t  tx_code;           /**< preamble codes, 1 .. 24             */
    uint8_t  rx_code;
    bool     sfd_decawave;      /**< the proprietary SFD, else the
                                     standard one                        */
    uint16_t tx_antenna_delay;  /**< device ticks, one way               */
    uint16_t rx_antenna_delay;  /**< device ticks, one way               */
};

/**
 * @brief This instrument's radio, with the platform's antenna delays.
 *
 * The Decawave SFD is the default where the driver is built with
 * DW1000_WITH_PROPRIETARY_SFD, the standard one where it is not.
 */
void dw1000_probe_radio_default(struct dw1000_probe_radio *radio,
                                uint16_t tx_antenna_delay,
                                uint16_t rx_antenna_delay);

/*===========================================================================*/
/* Spelling                                                                  */
/*===========================================================================*/

/**
 * @brief What dw1000_probe_radio_option() made of an argument.
 */
typedef enum {
    DW1000_PROBE_RADIO_OPTION_NONE = 0, /**< not a radio option           */
    DW1000_PROBE_RADIO_OPTION_SET,      /**< a radio option, applied      */
    DW1000_PROBE_RADIO_OPTION_BAD,      /**< a radio option, refused      */
} dw1000_probe_radio_option_t;

/**
 * @brief Apply one `--name=value` argument to @p radio.
 *
 * Spelt here, not in the shells, so that the two platforms take the
 * same options by construction:
 *
 *     --channel=N            --bitrate=KBPS       --prf=MHZ
 *     --preamble=SYMBOLS     --pac=SYMBOLS
 *     --code=N  (both)       --tx-code=N          --rx-code=N
 *     --sfd=decawave|standard
 *     --antenna-delay=TICKS  (both)
 *     --tx-antenna-delay=TICKS                    --rx-antenna-delay=TICKS
 *
 * Each value is checked on its own (dw1000_validate_*()); the
 * combination is dw1000_probe_radio_resolve()'s, once every option is in.
 *
 * @param[in,out] radio   changed only on SET
 * @param[in]     arg     one argument, as typed
 * @param[out]    errmsg  on BAD, why (a static string); may be NULL
 */
dw1000_probe_radio_option_t
dw1000_probe_radio_option(struct dw1000_probe_radio *radio, const char *arg,
                          const char **errmsg);

/** @brief The options above, for a shell's help text. */
#define DW1000_PROBE_RADIO_OPTIONS_HELP                                 \
    "--channel=N --bitrate=KBPS --prf=MHZ --preamble=SYMBOLS "          \
    "--pac=SYMBOLS --code=N --tx-code=N --rx-code=N "                   \
    "--sfd=decawave|standard --antenna-delay=TICKS "                    \
    "--tx-antenna-delay=TICKS --rx-antenna-delay=TICKS"

/*===========================================================================*/
/* Validation                                                                */
/*===========================================================================*/

/**
 * @brief Turn @p radio and a transmit power into what dw1000_configure()
 *        takes, or say why the driver would refuse it.
 *
 * Touches no chip. A shell calls this before anything irreversible:
 * before the chip is brought up on a host, before it is brought up again
 * to change an antenna delay on a board.
 *
 * @param[in]  radio     the settings
 * @param[in]  tx_power  encoded, as DW1000_TX_POWER_AUTO or
 *                       DW1000_TX_POWER_05DB()
 * @param[out] out       filled on success; the antenna delays are not
 *                       part of it (they are dw1000_config_t's)
 * @param[out] errmsg    on failure, why (a static string); may be NULL
 * @return     true if the driver accepts the combination
 */
bool dw1000_probe_radio_resolve(const struct dw1000_probe_radio *radio,
                                uint8_t tx_power, struct dw1000_radio *out,
                                const char **errmsg);

/*===========================================================================*/
/* The line                                                                  */
/*===========================================================================*/

/**
 * @brief Write the `SETUP channel=...` line: the radio this run has.
 *
 * Read back off the chip (dw1000_get_radio_state()), not echoed from the
 * settings, for the reason the configuration has always been read back:
 * a capture that records the request is recording its own intention. The
 * one field the chip does not hold in decoded form is the PAC (it holds a
 * tuned DRX_TUNE2 word), which comes from the driver's applied copy.
 *
 * Keys, in order: `channel=` `bitrate=` (kbps) `prf=` (MHz)
 * `preamble=` (symbols) `pac=` `tx_code=` `rx_code=` `sfd=`
 * (`decawave` or `standard`) `tx_antd=` `rx_antd=` (ticks)
 * `tx_power_db=`, then, when @p origin is not NULL, `node=` `role=`
 * `run=`. Every role emits one when it starts, before anything else, so
 * that two runs at different settings cannot be confused afterwards.
 *
 * SPI traffic: takes the bus lock itself; never inside an exchange.
 * Same return contract as dw1000_probe_record_format().
 */
size_t dw1000_probe_radio_format(char *buf, size_t len, dw1000_t *dw,
                                 const struct dw1000_probe_origin *origin);

/**
 * @brief Emit the SETUP line for a role about to run, directly.
 *
 * dw1000_probe_port_emit() is safe here: the role has not started, so
 * no exchange is in flight. Called by every role as its first act.
 */
void dw1000_probe_radio_emit(dw1000_t *dw, dw1000_probe_role_t role,
                             const char *node_name);

/** @} */

#endif
