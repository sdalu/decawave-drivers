/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Turn human radio values into the fields dw1000_radio_t wants
 *
 * A program that takes its radio settings from a command line, a
 * configuration file or an operator has the same job to do every time:
 * read a channel number, a bitrate in kbps, a PRF in MHz, a preamble
 * length in symbols, and turn each into the encoded value
 * <dw1000/dw1000.h> expects, or say why it cannot. This is that job,
 * once, with the message.
 *
 * Each function validates ONE field and nothing else. The combination is
 * not checked here and must not be: @p dw1000_configure() already refuses
 * an inconsistent radio (a preamble code that does not go with the PRF,
 * UM §10.5) and returns -1 without touching the chip. Re-stating those
 * rules in a caller is how two copies come to disagree, which is
 * exactly what happened to the sniffer this file was lifted out of, where
 * a hand-written check accepted preamble codes the driver rejects and
 * rejected ones it accepts. So: parse each field here, hand the result to
 * @p dw1000_configure(), and check ITS return value.
 *
 * On failure @p val is left alone and @p errmsg is set to a static
 * string; on success @p errmsg is set to NULL. Both may be NULL if you
 * only want the verdict. The strings are English, short, and say what
 * the accepted values are.
 *
 * This is host-facing convenience, not part of the driver's operation
 * (the only thing here is a value mapping and a set of message strings),
 * so it is a source list of its own, @p DW1000_SOURCES_VALIDATE, the way
 * <dw1000/dw1000_state.h> is. An image with no command line need not
 * carry the strings.
 */

#ifndef __DW1000_VALIDATE__H
#define __DW1000_VALIDATE__H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Validate a channel number
 *
 * The DW1000 has channels 1, 2, 3, 4, 5 and 7. There is no channel 6.
 *
 * @param[in]  channel  channel number
 * @param[out] val      @p dw1000_radio_t::channel (may be NULL)
 * @param[out] errmsg   why it was refused, or NULL (may be NULL)
 * @return true         @p channel is usable
 */
bool dw1000_validate_channel(int channel, uint8_t *val, const char **errmsg);

/**
 * @brief Validate a bitrate given in kbps
 *
 * One of 110, 850 or 6800.
 *
 * @param[in]  kbps     bitrate, in kbps
 * @param[out] val      @p dw1000_radio_t::bitrate (may be NULL)
 * @param[out] errmsg   why it was refused, or NULL (may be NULL)
 * @return true         @p kbps is usable
 */
bool dw1000_validate_bitrate(int kbps, uint8_t *val, const char **errmsg);

/**
 * @brief Validate a pulse repetition frequency given in MHz
 *
 * 16 or 64. The API has a 4 MHz value and the receiver does not support
 * it, so it is refused here with that as the reason.
 *
 * @param[in]  mhz      PRF, in MHz
 * @param[out] val      @p dw1000_radio_t::prf (may be NULL)
 * @param[out] errmsg   why it was refused, or NULL (may be NULL)
 * @return true         @p mhz is usable
 */
bool dw1000_validate_prf(int mhz, uint8_t *val, const char **errmsg);

/**
 * @brief Validate a preamble code
 *
 * 1 to 24, all of which this driver supports (they index
 * @p lde_repc_tunning). Which of them goes with your PRF is a rule about
 * the pair, so it is not checked here: 1..8 go with 16 MHz PRF and 9..24
 * with 64 MHz (UM §10.5), and @p dw1000_configure() enforces it.
 *
 * @param[in]  pcode    preamble code
 * @param[out] val      @p dw1000_radio_t::tx_pcode / ::rx_pcode (may be NULL)
 * @param[out] errmsg   why it was refused, or NULL (may be NULL)
 * @return true         @p pcode is usable
 */
bool dw1000_validate_pcode(int pcode, uint8_t *val, const char **errmsg);

/**
 * @brief Validate a preamble length given in symbols
 *
 * 64, 1024 and 4096 always; 128, 256, 512, 1536 and 2048 are proprietary
 * and need @p DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH, so what this
 * accepts depends on how the driver was compiled, and it must, since
 * @p dw1000_configure() refuses them in a build without the option. The
 * message says so when that is the reason.
 *
 * @param[in]  symbols  preamble length, in symbols
 * @param[out] val      @p dw1000_radio_t::tx_plen (may be NULL)
 * @param[out] errmsg   why it was refused, or NULL (may be NULL)
 * @return true         @p symbols is usable
 */
bool dw1000_validate_plen(int symbols, uint8_t *val, const char **errmsg);

/**
 * @brief Validate a preamble acquisition chunk size given in symbols
 *
 * 8, 16, 32 or 64. Which one suits your preamble length is a matter of
 * receiver sensitivity rather than validity (UM §4.1.1), and is not
 * checked here.
 *
 * @param[in]  symbols  PAC size, in symbols
 * @param[out] val      @p dw1000_radio_t::rx_pac (may be NULL)
 * @param[out] errmsg   why it was refused, or NULL (may be NULL)
 * @return true         @p symbols is usable
 */
bool dw1000_validate_pac(int symbols, uint8_t *val, const char **errmsg);

/**
 * @brief Validate an antenna delay given in metres, and convert it
 *
 * The one entry point here that is not a radio field: an antenna delay is
 * a distance, and what the chip wants is that distance as ticks of
 * @p DW1000_TIME_CLOCK_HZ in sixteen bits. So this converts as well as
 * checks (@p DW1000_METER_TO_CLOCK(), then the range) and is the
 * reason that macro says to come here for a value that came from a user.
 *
 * The result is NOT halved. A round-trip figure is halved by the caller,
 * as the @p DW1000_METER_TO_CLOCK() example shows, because whether the
 * number in hand is one way or two is the caller's to know.
 *
 * @param[in]  meters   antenna delay, in metres
 * @param[out] val      ticks, for dw1000_config_t::tx_antenna_delay or
 *                      ::rx_antenna_delay (may be NULL)
 * @param[out] errmsg   why it was refused, or NULL (may be NULL)
 * @return true         @p meters is usable
 */
bool dw1000_validate_antenna_delay(double meters, uint16_t *val,
				   const char **errmsg);

#ifdef __cplusplus
}
#endif

#endif
