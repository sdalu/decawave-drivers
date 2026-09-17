/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Frames are held in a ring, not in a single slot, and not sent from the
 * callback that receives them.
 *
 * The design is probe/src/exchange.c's, which arrived at it the hard way
 * (commit "probe/exchange: hold received frames in a ring, not one slot"):
 * a monotonic count of frames captured, the slot being that count modulo
 * the ring depth, and a producer that never refuses. What the sniffer
 * needs it for is not probe's reason -- probe had gaps between waits in
 * which a peer's answer was discarded -- but the same shape solves it.
 *
 * Here the reason is that forwarding costs far more than receiving. Under
 * double buffering the driver re-enables the receiver before calling
 * rx_ok and toggles the host side buffer pointer the moment rx_ok
 * returns, so everything the frame is wanted for has to be read out
 * inside that callback (dw1000.h's dblbuff note spells out the two
 * obligations). eth_send() is a blocking sendmsg(2) on a raw AF_PACKET
 * socket: doing it in there would hold the callback open across the
 * kernel's network path while the next frame is already landing in the
 * other buffer. So the callback does the one thing it must do now -- the
 * SPI read-out -- and the sendmsg happens afterwards, outside it.
 *
 * There is no lock here, and unlike probe there does not need to be one.
 * probe's producer runs on its port's event thread while a role thread
 * consumes, so it takes the bus lock around both ends. In this program
 * producer and consumer are the same thread: capture_put() is reached
 * from the rx_ok callback inside dw1000_process_events(), and
 * capture_get() is called from the loop in main.c that called
 * dw1000_process_events() and has returned from it. bitters does run a
 * GPIO IRQ thread, but it only dispatches registered callbacks and this
 * program registers none -- it polls the interrupt's file descriptor
 * itself. Nothing here is reachable from two threads at once. Were that
 * to change, this file is where the lock would go, and the comment in
 * probe/src/exchange.c explains why a mutex rather than atomics.
 */

#include <poll.h>

#include "capture.h"
#include "uwb.h"

/* Sixteen entries, at DW1000_FRAME_MAXSIZE + a length each: about 2 kB in
 * standard mode. Deeper than probe's four because the two are absorbing
 * different things -- probe covers the one or two frames a two-node
 * exchange makes in a gap, whereas this covers however many frames arrive
 * while the host is inside sendmsg(2), with no upper bound on the traffic
 * and every frame on the channel wanted. Not a tuned number either: it is
 * "enough that a scheduling hiccup on the forwarding side does not cost
 * frames", and capture_stats()'s overrun counter is what says whether it
 * was enough on a given run. */
#define RX_RING 16u

static struct capture_frame rx_ring[RX_RING];
static unsigned long        rx_frame_seq;	/* frames captured, monotonic */
static unsigned long        rx_read_upto;	/* frames consumed, monotonic */
static struct capture_stats stats;

void
capture_put(size_t length)
{
    struct capture_frame *slot = &rx_ring[rx_frame_seq % RX_RING];

    /* The reported length is not trusted into a fixed slot. In standard
     * mode the chip cannot report more than DW1000_FRAME_MAXSIZE, so this
     * is a guard rather than an expected path -- but it is the guard that
     * keeps a build with proprietary long frames turned on from writing
     * 1023 bytes into a 127 byte slot, and the counter is how such a
     * build would be noticed rather than silently mangled. */
    size_t copy_len = length;
    if (copy_len > sizeof(slot->data)) {
	copy_len = sizeof(slot->data);
	stats.truncated++;
    }

    /* The read-out itself: this is the part that cannot be deferred. */
    uwb_read_frame_data(slot->data, copy_len, 0);
    slot->length = copy_len;

    /* Published last, so the count that indexes the slot only advances
     * once the slot is whole. */
    rx_frame_seq++;
    stats.captured++;
}

bool
capture_get(struct capture_frame *out)
{
    if (rx_read_upto == rx_frame_seq)
	return false;

    /* Anything the producer has already written over is gone: the ring is
     * RX_RING deep, so a consumer further behind than that has lost the
     * oldest entries, and this is the only way a frame received by this
     * program is lost. Same accounting as probe's frame_wait(). */
    unsigned long oldest =
	rx_frame_seq > RX_RING ? rx_frame_seq - RX_RING : 0;
    if (rx_read_upto < oldest) {
	stats.overrun += oldest - rx_read_upto;
	rx_read_upto   = oldest;
    }

    /* Oldest first, one per call, so a burst leaves in arrival order
     * instead of collapsing to its last member. Copied out rather than
     * handed over by pointer: the caller is about to spend a syscall on
     * it, which is long enough for the slot to be wanted again. */
    *out = rx_ring[rx_read_upto % RX_RING];
    rx_read_upto++;
    stats.consumed++;

    return true;
}

const struct capture_stats *
capture_stats(void)
{
    return &stats;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
