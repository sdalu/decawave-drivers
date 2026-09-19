/*
 * Copyright (c) 2024-2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The emulation, seen from outside: how to build one, and what it says
 * on the socket.
 *
 * The register model is driven by the port (the SPI and IO line calls of
 * dw1000/osal.h); what it cannot answer on its own, the air, it asks
 * a medium server over the rsvc unix socket, as `struct
 * dw1000_driver_iopkt` messages of type DW1000_RSVC_TX, RX, TX_DONE and
 * RX_CONFIG.
 *
 * The struct is the contract with that server, which is a separate
 * program and need not be written in C: the layout below is packed and
 * little-endian-on-the-wire by virtue of both ends being the same host,
 * and its fields and numeric constants must not move. Anything a server
 * implementer needs is in this one header.
 */

#ifndef __DW1000_EMULATION_H__
#define __DW1000_EMULATION_H__

#include <stdint.h>
#include <stddef.h>

#include "dw1000/dw1000.h"
#include "rsvc.h"


/*----------------------------------------------------------------------*/
/* Lifecycle                                                            */
/*----------------------------------------------------------------------*/

struct dw1000_emulation;

/**
 * Create a DW1000 register model attached to a remote service.
 *
 * @param rsvc		connection to the medium server
 * @param line_cb	called with DW1000_IOLINE_IRQ when the model
 *			raises its interrupt line
 * @param line_args	opaque argument for @p line_cb
 *
 * @return the emulation, to be stored in the `emulation` field of the
 *         ioline and SPI driver structures of dw1000/osal.h
 */
struct dw1000_emulation *
dw1000_emulation_create(rsvc_t *rsvc, void (*line_cb)(int line, void *args), void *line_args);

/**
 * Reset the register model to its power-on content.
 *
 * @param e		the emulation
 */
void dw1000_emulation_reset(struct dw1000_emulation *e);

/**
 * Drop the next transmit start, as the chip does under Errata 1.4 3.1
 * (TX-1) and when a start is written into an active receiver: the
 * TXSTRT is taken and nothing follows, no transmit flag, no completion,
 * the state unchanged. One start only; the one after goes out as usual.
 * For a test of what a host and the driver make of a send that never
 * began.
 */
void dw1000_emulation_drop_next_start(struct dw1000_emulation *e);

/**
 * Fail the next frame delivered to the model in its PHY header, as the
 * chip does when the header does not decode (UM 4.3.2, 4.3.3, 7.2.17):
 * RXPRD, RXSFDD, RXPHD and RXPHE are raised and none of RXDFR, RXFCG
 * and RXFCE, ICRBP stays where it is and the receive buffer keeps what
 * it held; with RXAUTR set the receiver goes back to hunting preamble,
 * with it clear it goes idle. One frame only; the one after it is
 * received as usual.
 *
 * None of those four bits is part of the double buffered swinging set,
 * so the error stands in the status word whichever buffer the host is
 * on. For a test of a receive error read in the same word as a good
 * frame's RXFCG, which the model's own bad-FCS error cannot produce,
 * RXFCE swinging with RXFCG.
 */
void dw1000_emulation_fail_next_frame(struct dw1000_emulation *e);

/**
 * Give the LDE a run of its own, @p ticks device clock ticks long.
 *
 * Zero, the default and the value a reset restores, keeps a reception
 * atomic: everything the chip writes for a frame is written at once,
 * which is what the model always did. Otherwise every reception
 * completes in two steps.
 *
 * At the first the frame is in: the payload reaches `RX_BUFFER`,
 * `RX_FINFO` is written, `RXPRD`, `RXSFDD`, `RXPHD` and `RXDFR` are
 * raised with `RXFCG` or `RXFCE`, and the chip's record of its last
 * reception is set. `LDEDONE` stays clear, `RX_TIME` keeps the stamp it
 * held for the previous frame into that buffer, `RX_TTCKI` and
 * `RX_TTCKO` keep theirs, and `ICRBP` does not move.
 *
 * At the second, @p ticks later, the run finishes: the three timestamp
 * registers are written, `LDEDONE` is set in the buffer's set and in
 * the record, and a frame with a good CRC moves `ICRBP` on (UM 4.3.2).
 *
 * A `TRXOFF` between the two cancels the second, as it cancels every
 * other deadline the model holds: `LDEDONE` never comes, `RX_TIME`
 * keeps the previous frame's stamp and `ICRBP` stays where it was. That
 * is the cut of `DW1000.md`, "A TRXOFF between RXFCG and LDEDONE leaves
 * the frame without its timestamp, and the IC pointer where it was",
 * and the reason the knob exists: a host cannot otherwise be shown a
 * good frame whose timestamp is not its own.
 *
 * What the bench measured is the cut and only the cut (5 of 4482 and
 * 11 of 4415 deliveries over two duplex soaks): a reception a
 * `TRXOFF` terminates after its payload and CRC are in posts `RXFCG`
 * and never posts `LDEDONE`, whatever the host does next. Whether an
 * untouched run ends before or after `RXFCG` was not measured, and the
 * two steps of the model are not a claim that it ends after: the knob
 * gives the run a length only so that a `TRXOFF` has somewhere to fall
 * inside it.
 *
 * Unlike the two knobs above it is not a one-shot: it stands until it
 * is set back to 0 or the model is reset.
 *
 * @param e		the emulation
 * @param ticks		length of the run, in units of
 *			@p DW1000_TIME_CLOCK_HZ; 0 for an atomic reception
 */
void dw1000_emulation_lde_delay(struct dw1000_emulation *e, uint32_t ticks);

/**
 * Stop the model's own thread.
 *
 * The model runs a thread to meet the times a host programs into DX_TIME
 * and to expire the receive timeouts, and that thread talks to the
 * medium server. Joining it is therefore the first of the three steps a
 * clean shutdown takes, and it has to come before the connection is
 * closed: a deadline that fired into a closed connection would wait for
 * a reply nobody is left to deliver.
 *
 * Idempotent. Must not be called from the line callback: that would be
 * the model waiting for a thread that is waiting for the callback to
 * return.
 *
 * @param e		the emulation, or NULL
 */
void dw1000_emulation_stop(struct dw1000_emulation *e);

/**
 * Release the model.
 *
 * Shutting down has an order, and it is not negotiable, because the
 * model and the rsvc connection each hold a thread that calls into the
 * other:
 *
 *   1. @p dw1000_emulation_stop(e)  : after this the model makes no
 *                                      further calls on the connection;
 *   2. @p rsvc_close(rsvc)          : after this the reader thread has
 *                                      been joined, so no frame can
 *                                      arrive in the model;
 *   3. @p dw1000_emulation_destroy(e).
 *
 * Step 1 is done for you if it has not been done already, which is safe
 * whenever nothing is arriving. Step 2 is not: destroying the model
 * while the connection is open leaves the reader thread able to deliver
 * a frame into freed memory.
 *
 * @param e		the emulation, or NULL
 */
void dw1000_emulation_destroy(struct dw1000_emulation *e);


/*----------------------------------------------------------------------*/
/* The clock                                                            */
/*----------------------------------------------------------------------*/

/**
 * The device time, now, as SYS_TIME would read it.
 *
 * The model has no oscillator to count, so it derives the device clock
 * from the host's CLOCK_REALTIME by the same formula the reference
 * medium server uses: seconds since the epoch times
 * @p DW1000_TIME_CLOCK_HZ, truncated, kept to 40 bits. That is what puts
 * a node's SYS_TIME on the same timeline as the RX and TX timestamps the
 * server hands it, which is the whole point: a delayed send is programmed
 * as "this many ticks after the timestamp of the frame I just received",
 * and the two have to be comparable.
 *
 * A medium server sharing the host should stamp with this same clock. It
 * is exported so that a medium living in the same process (as
 * tests/emulation does) can call it rather than reimplement it.
 *
 * @note Not the drifted clock. The reference server applies a per-node
 *       clock drift it is told about by its controller, which the node
 *       is never told; a node whose drift is non-zero therefore has a
 *       SYS_TIME that runs at a slightly different rate from the
 *       timestamps it is given. See port/emulation/README.md.
 *
 * @return device time in units of @p DW1000_TIME_CLOCK_HZ, 40 bits
 */
uint64_t dw1000_emulation_clock(void);


/*-- IO Packet definition and helpers ----------------------------------*/

/* Structure used for communication with the uwb process
 */
// 01005b4000
// 5b40 01 0004
struct  __attribute__((packed,aligned(1))) dw1000_driver_iopkt {
    uintptr_t drvid;
    uint8_t type;
    union {
	struct  {
	    uint8_t  flags;
	    uint16_t antenna_delay;
	    uint8_t  frame[DW1000_FRAME_MAXSIZE];
	} __attribute__((packed)) tx ;
	struct {
	    uint64_t timestamp;
	} __attribute__((packed)) tx_done;
	struct {
	    uint8_t  flags;
	    uint64_t timestamp;
	    uint8_t  frame[DW1000_FRAME_MAXSIZE];
	} __attribute__((packed)) rx;
	struct {
	    uint8_t  flags;
	} __attribute__((packed)) rx_config;
    } __attribute__((packed));
};




/* The length actually put on the wire, for each message the port sends.
 * A `frame` array is declared at its maximum and sent at its true
 * length, so a TX packet stops after the frame's last byte.
 */
#define DW1000_DRIVER_PKT_HDRLEN						\
    (sizeof(uintptr_t) + sizeof(uint8_t))
#define DW1000_DRIVER_PKT_DATALEN(field)					\
    sizeof(((struct dw1000_driver_iopkt*)NULL)->field)

#define DW1000_DRIVER_PKTMAXLEN_TX						\
    (DW1000_DRIVER_PKT_HDRLEN + DW1000_DRIVER_PKT_DATALEN(tx))

#define DW1000_DRIVER_PKTMAXLEN_RX_CONFIG					\
    (DW1000_DRIVER_PKT_HDRLEN + DW1000_DRIVER_PKT_DATALEN(rx_config))


#define DW1000_DRIVER_PKTLEN_TX(framelen)					\
    (DW1000_DRIVER_PKTMAXLEN_TX - DW1000_FRAME_MAXSIZE + (framelen))

#define DW1000_DRIVER_PKTLEN_RX_CONFIG()					\
    DW1000_DRIVER_PKTMAXLEN_RX_CONFIG




#define DW1000_RSVC_TX			0x03
#define DW1000_RSVC_RX			0x04
#define DW1000_RSVC_TX_DONE     	0x05
#define DW1000_RSVC_RX_CONFIG     	0x06

#define DW1000_RSVC_FLG_RANGING  	0x02

#endif
