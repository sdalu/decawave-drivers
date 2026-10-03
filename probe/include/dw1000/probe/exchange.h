/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_PROBE_EXCHANGE_H__
#define __DW1000_PROBE_EXCHANGE_H__

/**
 * @file    exchange.h
 * @brief   The two-node ranging exchange: twr_init / twr_resp.
 *
 * Chip mechanism, not host or bench: the wire frame, the four-frame
 * state machine and the two roles live here, driven through
 * <dw1000/dw1000.h> and the host primitives in <dw1000/probe/port.h>.
 * Nothing here prints to an operator: dw1000_probe_port_emit() is called
 * only for lines that are safe to emit (SETUP and READY, before anything
 * is in flight), everything else goes to the caller's @p emit_line, and
 * nothing here knows a shell, a console or an argv. An
 * application reports what these return however it reports anything
 * else.
 *
 * @addtogroup PROBE
 * @{
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <dw1000/dw1000.h>

/*===========================================================================*/
/* Results                                                                   */
/*===========================================================================*/

/**
 * @brief What one twr_init run produced.
 *
 * The roles print nothing to an operator; these counts are what an
 * application reports, in whatever form it reports anything.
 */
struct dw1000_probe_twr_init_result {
    uint16_t attempted;     /**< counted exchanges only; warm-up excluded */
    uint16_t resolved;      /**< of @p attempted, how many reached
                                 DW1000_PROBE_STATUS_OK               */
};

/**
 * @brief What one twr_resp run produced.
 */
struct dw1000_probe_twr_resp_result {
    uint32_t polled;        /**< POLLs answered with a RESPONSE attempt,
                                 the initiator's warm-up included        */
    uint32_t answered;      /**< of @p polled, how many REPORTs left     */
    uint32_t report_failed; /**< REPORT sends the driver did not confirm:
                                 counted so that "the initiator did not
                                 hear it" and "it never left" are told
                                 apart, the initiator's record saying
                                 `no-report` either way                  */
};

/*===========================================================================*/
/* Roles                                                                     */
/*===========================================================================*/

/*
 * THE EXCHANGE, and which end records it. Four frames (two-frame,
 * `ss`, skips FINAL):
 *
 *     POLL      initiator -> responder    t_sp / t_rp
 *     RESPONSE  responder -> initiator    t_sr / t_rr
 *     FINAL     initiator -> responder    t_sf / t_rf
 *     REPORT    responder -> initiator    t_rp, t_sr, t_rf, the
 *                                         responder's die and powers
 *
 * The INITIATOR records: it holds its own three instants and receives the
 * responder's three in REPORT, so the `TWR` line per attempt and the
 * closing `STATS` line are its. The responder answers, and emits the
 * `READY` line before listening and a `STATS` line of its own at the end.
 * Both lines go to @p emit_line, never to dw1000_probe_port_emit()
 * directly: dw1000_probe_port_emit() may be called only between runs (see
 * <dw1000/probe/port.h>), never between the frames of an exchange, where
 * a write that blocked would land in the measurement. Where that
 * buffering lives, and how big it is, is a host memory-budget decision
 * this library does not make, which is why @p emit_line is a callback
 * rather than a library-owned buffer. Both roles also emit their `SETUP`
 * line (dw1000_probe_radio_emit()) directly, first, before anything is in
 * flight.
 */

/**
 * @brief Responder role: answer up to @p count exchanges.
 *
 * Emits `SETUP` and `READY` directly, then answers each POLL with a
 * RESPONSE and, once FINAL is in (or at once, single-sided), a REPORT
 * after a fixed pause; and hands its closing `STATS` line to
 * @p emit_line, `of=` being the POLLs it answered and `completed=` the
 * REPORTs that left.
 *
 * IT DOES RETURN. The wait for a POLL is bounded, long until the first
 * POLL arrives (tens of seconds, a harness's head start) and short after
 * it, and a wait that ends with nothing ends the run: before any POLL it
 * means nobody came, after one that the initiator has finished. So
 * @p count is a ceiling, and need not be the initiator's count plus its
 * warm-up.
 *
 * @param dw         the driver instance
 * @param count      the most exchanges to answer
 * @param ss         true for the two-frame single-sided exchange; the
 *                   initiator must agree
 * @param own_addr   this node's address
 * @param peer_addr  unused: RESPONSE answers whoever sent the POLL; kept
 *                   for signature symmetry with dw1000_probe_twr_init_run()
 * @param node_name  the origin `node=` every emitted line carries
 * @param emit_line  called with the closing `STATS` line (never NULL)
 */
struct dw1000_probe_twr_resp_result
dw1000_probe_twr_resp_run(dw1000_t *dw, long count, bool ss,
                          uint16_t own_addr, uint16_t peer_addr,
                          const char *node_name,
                          void (*emit_line)(const char *line));

/**
 * @brief Initiator role: run @p warmup uncounted exchanges, then
 * @p count counted ones, and record each counted one.
 *
 * Emits `SETUP` directly, then hands @p emit_line one `TWR` line per
 * counted attempt and the closing `STATS` line, `of=` being the counted
 * attempts and `completed=` those that resolved. Warm-up attempts are
 * run and not recorded.
 *
 * @param warmup     exchanges run first and never counted, to get the
 *                   link (and the chip) into steady state
 * @param node_name  the origin `node=` every emitted line carries
 * @param emit_line  called once per finished line (never NULL)
 */
struct dw1000_probe_twr_init_result
dw1000_probe_twr_init_run(dw1000_t *dw, long count, bool ss, long warmup,
                          uint16_t own_addr, uint16_t peer_addr,
                          const char *node_name,
                          void (*emit_line)(const char *line));

/*===========================================================================*/
/* Capture: filled by the application's radio event callbacks              */
/*===========================================================================*/

/**
 * @brief Capture one received frame for the exchange roles.
 *
 * Called by the APPLICATION's rx_ok callback (the one registered as
 * dw1000_config_t.cb.rx_ok), before the receiver is re-armed, and while
 * the bus lock (dw1000_probe_port_bus_lock()) is already held:
 * dw1000_process_events() holds it across every callback it invokes, by
 * the contract in <dw1000/probe/port.h>. Reads the frame data, its
 * RMARKER time and its power estimate off the chip; a restart is free
 * to disturb all three (this driver runs without cfg->dblbuff, so
 * nothing latches them the way double buffered mode would), which is
 * why this must run before dw1000_rx_start().
 *
 * The application's callback owns everything else about handling an
 * rx_ok (counting it for its own purposes, re-arming the receiver),
 * this call is only the part the exchange roles need.
 *
 * @param dw      the driver instance the frame arrived on
 * @param length  the callback's own length parameter (already read by
 *                dw1000_process_events(); no separate
 *                dw1000_rx_get_frame_length() call needed)
 */
void dw1000_probe_rx_capture(dw1000_t *dw, size_t length);

/**
 * @brief Note one transmit completion, for the exchange roles' settle
 * wait on their own transmits, and for the tx role's completion count
 * (<dw1000/probe/solo.h>).
 *
 * Called by the application's tx_done callback (dw1000_config_t.cb.
 * tx_done), under the same bus-lock guarantee as
 * dw1000_probe_rx_capture() above.
 */
void dw1000_probe_tx_capture(void);

/**
 * @brief Note one frame the chip rejected (bad CRC, bad PHR, SFD
 * timeout), for the rx role's account (<dw1000/probe/solo.h>).
 *
 * Called by the application's rx_error callback (dw1000_config_t.cb.
 * rx_error), under the same bus-lock guarantee as the two above, and
 * before it re-arms the receiver. It is what lets a listener tell
 * "corrupt frames arriving" from "nothing arriving".
 */
void dw1000_probe_rx_error_capture(void);

/** @} */

#endif
