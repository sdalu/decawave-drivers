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
 * dw1000/osal.h); what it cannot answer on its own -- the air -- it asks
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
 * Stop the model and release it.
 *
 * The model runs a thread of its own to meet the times a host programs
 * into DX_TIME and the receive timeouts, so it cannot simply be dropped
 * on the floor the way a bare register array could. Joins that thread and
 * frees @p e; the rsvc connection is the caller's and is left open.
 *
 * Must not be called from the line callback: that would be the model
 * waiting for a thread that is waiting for the callback to return.
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
 * medium server uses -- seconds since the epoch times
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
 *       timestamps it is given. See docs/emulation.md.
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
