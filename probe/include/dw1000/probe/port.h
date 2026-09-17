/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_PROBE_PORT_H__
#define __DW1000_PROBE_PORT_H__

/**
 * @file    port.h
 * @brief   What a host must supply for the probe to run.
 *
 * The driver's own OSAL (<dw1000/osal.h>) offers delays, the io lines,
 * SPI and an assert, and deliberately nothing else: it is what a driver
 * needs, not what a measurement needs. A probe additionally needs to know
 * what time it is, to wait for a frame with a deadline, and to put a line
 * somewhere. Every existing consumer of this driver has hand-rolled those
 * three (tests/emulation/smoke.c and the Raspberry Pi application each
 * grew their own condition variable and clock loop), which is the
 * duplication this header exists to stop.
 *
 * Nothing here knows anything about a measurement. A port is chosen at
 * build time, the way an OSAL port is, and carries no policy: no
 * timeouts, no counts, no thresholds. Those are parameters, and their
 * values come from the application.
 *
 * @addtogroup PROBE
 * @{
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*===========================================================================*/
/* Time                                                                      */
/*===========================================================================*/

/**
 * @brief Monotonic host time, in microseconds.
 *
 * Monotonic is the requirement, not the epoch: only differences are ever
 * taken. A port that can only offer milliseconds multiplies by 1000 and
 * says so in its own header: the settle windows are tens of seconds, so
 * the resolution that matters is the frame deadline, which is
 * sub-millisecond.
 */
typedef uint64_t dw1000_probe_time_t;

/**
 * @brief Now, monotonic, in microseconds.
 */
dw1000_probe_time_t dw1000_probe_port_now(void);

/**
 * @brief Sleep for at least @p us microseconds.
 *
 * Distinct from the driver's _dw1000_delay_usec(), which takes a uint16_t
 * and busy-waits for register timing. This one may yield, and is used for
 * the gap between exchanges.
 */
void dw1000_probe_port_sleep(uint32_t us);

/*===========================================================================*/
/* Waiting for the radio                                                     */
/*===========================================================================*/

/**
 * @brief Wait until a radio event is pending, or @p timeout_us elapses.
 *
 * Binary-semaphore semantics, so that a wake-up cannot be lost:
 *
 *  - a dw1000_probe_port_wake() arriving BEFORE the wait is remembered, and the
 *    next wait returns true at once. This is not a detail: the event
 *    routinely lands between the transmit call returning and the wait
 *    being entered, and a port implementing this as a bare condition
 *    variable signal would drop exactly those;
 *  - a wait returning true consumes exactly one pending wake-up;
 *  - wake-ups arriving before a wait collapse into one. This is not
 *    merely tolerated, it is required: with the receiver set to
 *    re-enable itself the radio can take several frames back to back
 *    without the host in between, so wakes genuinely do arrive in
 *    bursts. One pending wake, with dw1000_process_events() then
 *    draining whatever it finds, is the shape that fits; counting them
 *    would only promise a correspondence the probe cannot use.
 *
 * A @p timeout_us of 0 polls. The probe loops against its own deadline
 * using dw1000_probe_port_now(), so a stale wake-up (a frame that turns
 * out to fail the peer filter) costs one iteration, and a port tracks
 * nothing.
 *
 * Neither this nor dw1000_probe_port_wake() touches the chip.
 * dw1000_process_events() is called by the PROBE, on the thread that
 * called this, after it returns true; never by the port, and never
 * from the context that calls dw1000_probe_port_wake().
 *
 * The permanent reason is the RTOS ports: there the wake context is an
 * interrupt handler, and a blocking SPI transaction is not something an
 * interrupt handler may start. That holds for every such port and will
 * not change. A host port has its own reasons, which are not the same
 * reasons and do not move together: under the emulation port the wake
 * context is the socket reader thread, and a driver call from it issues
 * a request and then blocks waiting for a reply that only that same
 * thread could have delivered: a hang, not an error.
 *
 * That port is itself the argument for stating the rule once rather than
 * per platform. It had TWO independent reasons for this prohibition, a
 * lock and the reader thread; one was removed and the other was not, and
 * for a while it was believed both had gone. A contract resting on any
 * particular port's internals would have been quietly wrong in between.
 * This one does not vary per port, and the probe is written once against
 * it.
 *
 * A wake-up may arrive from MORE THAN ONE context: a host port can have
 * several sources (a reader thread and a deadline thread, say) and a
 * bare-metal one several interrupt sources. The latch must tolerate
 * concurrent wakes from different threads, which is why it is a latch
 * and not a flag.
 *
 * @param[in] timeout_us  how long to wait at most, in microseconds
 * @return    true if a wake-up was pending or arrived in time;
 *            false on timeout
 */
bool dw1000_probe_port_wait(uint32_t timeout_us);

/**
 * @brief Note that a radio event is pending.
 *
 * The APPLICATION attaches the DW1000 interrupt line and calls this from
 * its handler. Attaching needs the pin, and a pin is a wiring fact that
 * belongs to the application (a devicetree node on one platform, a
 * GPIO number on another), so the port supplies the primitive and the
 * application wires it. A port that looked the pin up itself would be a
 * port that knows a board.
 *
 * Must be callable from interrupt context on every port; a port where it
 * is not defers, and says so in its own header.
 */
void dw1000_probe_port_wake(void);

/*===========================================================================*/
/* The bus                                                                   */
/*===========================================================================*/

/**
 * @brief Take and release exclusive use of the radio's bus.
 *
 * The driver is not thread safe and its OSAL ports add no serialisation
 * of their own: every consumer supplies it, and they each invented
 * their own before this existed. The exchange needs it because at least
 * two contexts reach the chip: whatever drives the exchange, and
 * whatever calls dw1000_process_events() after a wake-up.
 *
 * Recursive is NOT required and must not be relied upon: a caller
 * already holding the lock does not take it again. In particular the
 * driver's rx/tx callbacks run inside dw1000_process_events(), which is
 * called with the lock held, so they are already covered.
 *
 * A single-threaded host may implement both as no-ops, and should say so
 * in its own header rather than leaving a reader to wonder.
 */
void dw1000_probe_port_bus_lock(void);
void dw1000_probe_port_bus_unlock(void);

/*===========================================================================*/
/* Output                                                                    */
/*===========================================================================*/

/**
 * @brief Emit one finished line.
 *
 * The probe formats; the port transports. @p line is NUL-terminated and
 * carries no newline of its own; appending whatever the host's notion
 * of one is belongs here, since that is the part that differs between a
 * UART and a pipe.
 *
 * Called only between runs, never between the frames of an exchange: a
 * write that blocked there would land in the measurement. The probe
 * enforces that by buffering, and a port must not add buffering of its
 * own that outlives the call.
 */
void dw1000_probe_port_emit(const char *line);

/** @} */

#endif
