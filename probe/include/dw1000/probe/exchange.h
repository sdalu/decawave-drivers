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
 * Nothing here prints -- dw1000_probe_port_emit() is called only for
 * lines that are safe to emit (the READY line, and whatever @p emit_line
 * is handed after a run has finished; see dw1000_probe_twr_resp_run()
 * below) -- and nothing here knows a shell, a console or an argv. An
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
 * @brief What one twr_resp run produced.
 *
 * The role itself prints nothing; these counts are what an application
 * reports to its own operator, in whatever form that takes -- a shell
 * line, an argv-driven CLI's stdout, a log.
 */
struct dw1000_probe_twr_resp_result {
    uint16_t attempted;
    uint16_t resolved;      /**< of @p attempted, how many reached
                                 DW1000_PROBE_STATUS_OK               */
};

/**
 * @brief What one twr_init run produced.
 */
struct dw1000_probe_twr_init_result {
    uint32_t attempted;     /**< counted exchanges only; warm-up excluded */
    uint32_t reached;       /**< of @p attempted, how many reached FINAL
                                 (single-sided) or REPORT (four-frame)    */
};

/*===========================================================================*/
/* Roles                                                                     */
/*===========================================================================*/

/**
 * @brief Responder role: listen for @p count exchanges.
 *
 * Emits the `READY` line directly (dw1000_probe_port_emit(), before any
 * exchange is in flight, so that is safe on its own) and then, for each
 * attempt, formats one `TWR` line and hands it to @p emit_line -- never
 * to dw1000_probe_port_emit() directly. dw1000_probe_port_emit() may be
 * called only between runs (see <dw1000/probe/port.h>), never between
 * the frames of an exchange; a write that blocked there would land in
 * the measurement. Where that buffering lives, and how big it is, is a
 * host memory-budget decision this library does not make -- an
 * embedded application typically reuses one fixed static buffer across
 * every command that needs it (tx/rx/twr_resp alike), which is why
 * @p emit_line is a callback rather than a library-owned buffer: it
 * lets the caller supply that memory instead of this library
 * duplicating it. @p emit_line is also handed the closing `STATS` line.
 * The caller decides when it is safe to actually flush what it
 * buffered to dw1000_probe_port_emit() -- immediately after this
 * function returns is that time, since the run is then over.
 *
 * IT DOES RETURN, which was not always so and is worth stating because
 * it bounds how long a caller can be held. Every wait in the run has a
 * deadline, the first POLL's being much the longest (tens of seconds,
 * to cover a harness's head start); an attempt whose deadline passes
 * becomes a record with DW1000_PROBE_STATUS_NO_POLL rather than a wait
 * that cannot end. A run nobody answers therefore costs its first
 * attempt the long budget and each one after it the short one, and then
 * reports @p count no-poll records and a STATS line carrying the
 * reception account: what the receiver delivered, and why none of it
 * was accepted.
 *
 * @param dw         the driver instance
 * @param count      how many exchanges to answer
 * @param ss         true for the two-frame single-sided estimate only
 *                   (still sends FINAL, stops there -- see probe/src/
 *                   exchange.c's file comment on why)
 * @param own_addr   this node's address; RESPONSE answers whoever sent
 *                   the POLL, not a fixed peer, so @p peer_addr is
 *                   unused here and kept only for signature symmetry
 *                   with dw1000_probe_twr_init_run()
 * @param peer_addr  unused (see above)
 * @param node_name  the origin `node=` every emitted line carries
 * @param emit_line  called once per finished line (never NULL): the
 *                   `TWR` line per attempt, then the closing `STATS`
 *                   line
 */
struct dw1000_probe_twr_resp_result
dw1000_probe_twr_resp_run(dw1000_t *dw, long count, bool ss,
                          uint16_t own_addr, uint16_t peer_addr,
                          const char *node_name,
                          void (*emit_line)(const char *line));

/**
 * @brief Initiator role: run @p warmup uncounted exchanges, then
 * @p count counted ones.
 *
 * Emits nothing -- the initiator never holds both ends' numbers (see
 * <dw1000/probe/record.h>'s note on why the responder is the one that
 * emits a record) -- so there is no line to buffer or hand to a
 * callback here.
 *
 * @param warmup  exchanges run first and never counted, to get the
 *                link (and the chip) into steady state
 */
struct dw1000_probe_twr_init_result
dw1000_probe_twr_init_run(dw1000_t *dw, long count, bool ss, long warmup,
                          uint16_t own_addr, uint16_t peer_addr);

/*===========================================================================*/
/* Capture -- filled by the application's radio event callbacks             */
/*===========================================================================*/

/**
 * @brief Capture one received frame for the exchange roles.
 *
 * Called by the APPLICATION's rx_ok callback (the one registered as
 * dw1000_config_t.cb.rx_ok), before the receiver is re-armed, and while
 * the bus lock (dw1000_probe_port_bus_lock()) is already held --
 * dw1000_process_events() holds it across every callback it invokes, by
 * the contract in <dw1000/probe/port.h>. Reads the frame data, its
 * RMARKER time and its power estimate off the chip; a restart is free
 * to disturb all three (this driver runs without cfg->dblbuff, so
 * nothing latches them the way double buffered mode would), which is
 * why this must run before dw1000_rx_start().
 *
 * The application's callback owns everything else about handling an
 * rx_ok -- counting it for its own purposes, re-arming the receiver --
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
 * wait on their own transmits.
 *
 * Called by the application's tx_done callback (dw1000_config_t.cb.
 * tx_done), under the same bus-lock guarantee as
 * dw1000_probe_rx_capture() above. Self-contained: an application with
 * its own reasons to count tx completions (a `probe tx` command, say)
 * keeps its own counter and calls this in addition, from the same
 * callback -- the two are unrelated and neither reads the other's.
 */
void dw1000_probe_tx_capture(void);

/** @} */

#endif
