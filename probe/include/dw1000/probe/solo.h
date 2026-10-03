/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_PROBE_SOLO_H__
#define __DW1000_PROBE_SOLO_H__

/**
 * @file    solo.h
 * @brief   What one node does on its own: the tx, rx and temperature
 *          roles, and what it reports about itself.
 *
 * The roles of probe/DESIGN.md's "What it does", items 4 and 5: transmit
 * only, receive only, and idle temperature sampling. They are what
 * separated transmit heating from receive heating, and the rx role is
 * also where the settle rule (<dw1000/probe/settle.h>) runs, since a die
 * settles with its receiver on and no peer at all.
 *
 * Chip mechanism like <dw1000/probe/exchange.h>, and under the same
 * contract: driven through <dw1000/dw1000.h> and <dw1000/probe/port.h>,
 * the radio callbacks feeding the capture hooks declared there
 * (dw1000_probe_rx_capture(), dw1000_probe_tx_capture(),
 * dw1000_probe_rx_error_capture()), and every line handed to the
 * caller's @p emit_line during the run, never to dw1000_probe_port_emit()
 * (see dw1000_probe_twr_resp_run() for why a run's lines are buffered).
 * The roles themselves print nothing; what they counted is returned, and
 * the application reports it.
 *
 * THE LINES. Every role first emits its `SETUP` line, directly
 * (dw1000_probe_radio_emit(): nothing is in flight yet), so that each
 * run says which radio produced it. Then every role samples the die: a
 * `TEMP` line
 * (dw1000_probe_temp_format()) when it starts, one every @p interval_ms
 * while it runs, and one when it ends. A sample that falls due while the
 * role is busy is taken late rather than twice, so the elapsed times say
 * when each reading was taken, never when it was owed. The rx role under
 * the settle rule emits a `SETTLE` line per window in place of the
 * per-interval `TEMP` lines, which bounds its line count by the give-up
 * time rather than by the readings (dw1000_probe_settle_lines()).
 *
 * @addtogroup PROBE
 * @{
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <dw1000/dw1000.h>
#include <dw1000/probe/settle.h>

/*===========================================================================*/
/* Defaults                                                                  */
/*===========================================================================*/

/** @brief Between two TEMP samples: 1 Hz, the rate at which the readings
 *         dither enough for a window mean to resolve (settle.h). */
#define DW1000_PROBE_SAMPLE_INTERVAL_MS 1000u

/** @brief Between two frames of the tx role. */
#define DW1000_PROBE_TX_GAP_US          10000u

/**
 * @brief The most lines a sampled run of @p seconds emits: the opening
 *        sample, one per interval, and the closing one.
 *
 * Exact for the rx and temperature roles, whose duration is given; the
 * tx role runs as long as its frames take, so for it this is a floor.
 */
size_t dw1000_probe_sampled_lines(uint32_t seconds, uint32_t interval_ms);

/*===========================================================================*/
/* tx                                                                        */
/*===========================================================================*/

/**
 * @brief What one tx run produced.
 */
struct dw1000_probe_tx_result {
    uint32_t attempted;     /**< frames asked for                          */
    uint32_t started;       /**< of those, transmits the driver accepted   */
    uint32_t completed;     /**< completions reported by the end of the run;
                                 short of @p started means a completion
                                 never came, not that a frame was lost     */
};

/**
 * @brief Transmit-only role: send @p count frames, @p gap_us apart.
 *
 * The frames carry no exchange and expect no answer; what this heats is
 * the far end, whose receiver is on. The receiver here stays off. Frame
 * i leaves at the start plus i times @p gap_us, an absolute schedule, so
 * that two transmitters do not drift into each other. A frame started
 * while the previous one is still on the air is refused by the driver,
 * and is counted as not started rather than retried.
 */
struct dw1000_probe_tx_result
dw1000_probe_tx_run(dw1000_t *dw, long count, uint32_t gap_us,
                    uint32_t interval_ms, const char *node_name,
                    void (*emit_line)(const char *line));

/*===========================================================================*/
/* rx                                                                        */
/*===========================================================================*/

/**
 * @brief What one rx run produced.
 */
struct dw1000_probe_rx_result {
    bool     failed;        /**< the receiver could not be started; the
                                 run ended at once, and nothing below
                                 means anything                            */
    uint32_t elapsed_ds;    /**< how long it listened, tenths of a second  */
    uint32_t received;      /**< frames the chip delivered                 */
    uint32_t rejected;      /**< frames the chip rejected: bad CRC, bad
                                 PHR, SFD timeout                          */
    dw1000_probe_settle_state_t settle; /**< the verdict; SAMPLING when no
                                 settle rule ran                           */
};

/**
 * @brief Receive-only role: listen for @p seconds, counting what arrives.
 *
 * Every frame counts, whoever sent it: the frames are not the
 * measurement, the receiver being on is.
 *
 * KEEPING THE RECEIVER ON is the driver's job double buffered (it
 * re-enables before the rx_ok callback runs) and nobody's single
 * buffered, where the probe applications' rx_ok does not re-arm (see the
 * applications' own notes). So single buffered this role re-arms from its
 * own loop when it sees a frame has arrived, which leaves the receiver off
 * for up to a loop turn per frame: the count is then a floor, and the
 * heating a little less than continuous. The bench runs it double
 * buffered.
 */
struct dw1000_probe_rx_result
dw1000_probe_rx_run(dw1000_t *dw, uint32_t seconds, uint32_t interval_ms,
                    const char *node_name,
                    void (*emit_line)(const char *line));

/**
 * @brief Receive-only role under the settle rule: listen until the die
 *        has settled, or the rule gives up.
 *
 * The same role as dw1000_probe_rx_run(), with its end decided by
 * @p params (which must be valid, dw1000_probe_settle_params_valid())
 * rather than by a duration, and its readings taken every
 * @p params->interval_ms. Emits `TEMP` at the start, `SETTLE` per window,
 * `TEMP` at the end.
 */
struct dw1000_probe_rx_result
dw1000_probe_rx_settle_run(dw1000_t *dw,
                           const struct dw1000_probe_settle_params *params,
                           const char *node_name,
                           void (*emit_line)(const char *line));

/*===========================================================================*/
/* temperature                                                               */
/*===========================================================================*/

/**
 * @brief What one temperature run produced.
 */
struct dw1000_probe_temperature_result {
    uint32_t samples;       /**< TEMP lines emitted                        */
};

/**
 * @brief Idle temperature role: radio stopped, sample the die for
 *        @p seconds.
 *
 * The baseline, and the cooling curve. @p seconds of 0 takes one reading
 * and emits one line, which is the "what is it now" a shell asks between
 * runs.
 */
struct dw1000_probe_temperature_result
dw1000_probe_temperature_run(dw1000_t *dw, uint32_t seconds,
                             uint32_t interval_ms, const char *node_name,
                             void (*emit_line)(const char *line));

/*===========================================================================*/
/* What the node is                                                          */
/*===========================================================================*/

/**
 * @brief Write who the chip is: its ids, its OTP calibration references,
 *        and the driver.
 *
 * Several lines, separated by '\n' with none after the last, the shape
 * dw1000_radio_state_format() has, so that a shell prints it one line at
 * a time and the two platforms' outputs diff. Reads only what
 * dw1000_initialise() already read; no bus traffic.
 *
 * @return as snprintf(): the length the text would have had
 */
size_t dw1000_probe_info_format(char *buf, size_t len, const dw1000_t *dw);

/** @brief Enough for dw1000_probe_info_format(). */
#define DW1000_PROBE_INFO_MAX 384

/**
 * @brief Write the transmit power the chip holds, read back off it.
 *
 * One line: `TX power: 7.5 dB (applied, read back from chip;
 * TX_POWER=0x...)`. One SPI read, under the bus lock this takes itself;
 * see probe/DESIGN.md, "What a transmit power read-back is worth".
 *
 * @return as snprintf()
 */
size_t dw1000_probe_power_format(char *buf, size_t len, dw1000_t *dw);

/** @} */

#endif
