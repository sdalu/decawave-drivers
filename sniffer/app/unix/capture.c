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
 * needs it for is not probe's reason (probe had gaps between waits in
 * which a peer's answer was discarded), but the same shape solves it.
 *
 * Here the reason is that forwarding costs far more than receiving. Under
 * double buffering the driver re-enables the receiver before calling
 * rx_ok and toggles the host side buffer pointer the moment rx_ok
 * returns, so everything the frame is wanted for has to be read out
 * inside that callback (dw1000.h's dblbuff note spells out the two
 * obligations). eth_send() is a blocking sendmsg(2) on a raw AF_PACKET
 * socket and pcapng_write() is a write(2) to a file or a pipe: doing
 * either in there would hold the callback open across the kernel while
 * the next frame is already landing in the other buffer. So the callback
 * does the things it must do now, the read-outs, and the output happens
 * afterwards, outside it.
 *
 * "The read-outs", plural, is the part that changed. The payload was
 * never the only thing in the swinging set: RX_TIME, RX_FQUAL, RX_TTCKI
 * and RX_TTCKO swing with the buffer pointer too, so the arrival
 * timestamp, the power estimates and the transmitter's clock drift are
 * readable here or nowhere. They used to be read nowhere, which meant a
 * sniffer that could say what a frame contained and nothing about how it
 * arrived. probe/src/exchange.c had already worked this out for the two
 * values it needs; capture_ops::read_meta is the same read-out, for all
 * of them, and it is a separate call so that a run that wants the highest
 * frame rate can leave it out and spend no SPI time on it.
 *
 * There is no lock here, and unlike probe there does not need to be one.
 * probe's producer runs on its port's event thread while a role thread
 * consumes, so it takes the bus lock around both ends. In this program
 * producer and consumer are the same thread: capture_put() is reached
 * from the rx_ok callback inside dw1000_process_events(), and
 * capture_get() is called from the loop in main.c that called
 * dw1000_process_events() and has returned from it. bitters does run a
 * GPIO IRQ thread, but it only dispatches registered callbacks and this
 * program registers none: it polls the interrupt's file descriptor
 * itself. Nothing here is reachable from two threads at once. Were that
 * to change, this file is where the lock would go, and the comment in
 * probe/src/exchange.c explains why a mutex rather than atomics.
 *
 * Nothing in this file touches the chip, or any header that does. That is
 * deliberate and it is new: the read-out goes through capture_ops, so the
 * ring, its overrun accounting and its truncation can be compiled and
 * run with no chip and no bitters, which is what tests/sniffer/capture.c
 * does and what there was previously no way to do.
 */

#include <string.h>
#include <time.h>

#include "capture.h"

/* Sixteen entries, at DW1000_FRAME_MAXSIZE + the per-frame record each:
 * about 2.5 kB in standard mode. Deeper than probe's four because the two
 * are absorbing different things: probe covers the one or two frames a
 * two-node exchange makes in a gap, whereas this covers however many
 * frames arrive while the host is inside sendmsg(2), with no upper bound
 * on the traffic and every frame on the channel wanted.
 *
 * Still not a tuned number, but no longer an unmeasurable one: it is
 * "enough that a scheduling hiccup on the forwarding side does not cost
 * frames", and capture_stats() now reports both whether it was exceeded
 * (overrun) and how close a run came to exceeding it (high_water), so a
 * bench run says which. */
#define RX_RING 16u

static const struct capture_ops *rx_ops;

static struct capture_frame rx_ring[RX_RING];
static unsigned long        rx_frame_seq;	/* frames captured, monotonic */
static unsigned long        rx_read_upto;	/* frames consumed, monotonic */
static struct capture_stats stats;

void
capture_init(const struct capture_ops *ops)
{
    rx_ops       = ops;
    rx_frame_seq = 0;
    rx_read_upto = 0;
    memset(&stats, 0, sizeof(stats));
}

void
capture_put(size_t length, bool ranging)
{
    struct capture_frame *slot = &rx_ring[rx_frame_seq % RX_RING];

    /* First, before any SPI traffic. This is the host's idea of when the
     * frame arrived, and it is what a pcapng timestamp is made of, so it
     * is taken at the top of the callback rather than after the read-out
     * has spent some tens of microseconds on the bus. The chip's own
     * instant, which is the one to use between two frames of the same
     * capture, comes from read_meta() below. */
    clock_gettime(CLOCK_REALTIME, &slot->wall);

    slot->flags    = ranging ? CAPTURE_F_RANGING : 0;
    slot->reported = length;
    slot->seq      = (uint32_t)rx_frame_seq;

    /* The reported length is not trusted into a fixed slot. In standard
     * mode the chip cannot report more than DW1000_FRAME_MAXSIZE, so this
     * is a guard rather than an expected path, but it is the guard that
     * keeps a build with proprietary long frames turned on from writing
     * 1023 bytes into a 127 byte slot. It is flagged as well as counted
     * now: the counter says a run truncated something, the flag says
     * which frame, and a receiver handed only the short length could
     * otherwise not tell a short frame from a mangled one. */
    size_t copy_len = length;
    if (copy_len > sizeof(slot->data)) {
	copy_len     = sizeof(slot->data);
	slot->flags |= CAPTURE_F_TRUNCATED;
	stats.truncated++;
    }

    /* The read-outs themselves: this is the part that cannot be deferred,
     * for the payload and for the metadata alike. read_meta is NULL when
     * the metadata read-out is turned off, and then the frame simply goes
     * out without CAPTURE_F_METADATA rather than with a zeroed block,
     * which a consumer could not tell from a real reading of zero. */
    rx_ops->read_frame_data(slot->data, copy_len, 0);
    slot->length = copy_len;

    if (rx_ops->read_meta != NULL) {
	rx_ops->read_meta(&slot->meta);
	slot->flags |= CAPTURE_F_METADATA;
    }

    /* Published last, so the count that indexes the slot only advances
     * once the slot is whole. */
    rx_frame_seq++;
    stats.captured++;

    /* How full the ring got, which is the measurement the ring depth
     * never had. Taken here rather than in capture_get() because this is
     * the only place the depth grows; the consumer only ever shrinks it. */
    unsigned long depth = rx_frame_seq - rx_read_upto;
    if (depth > stats.high_water)
	stats.high_water = depth;
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
     * it, which is long enough for the slot to be wanted again.
     *
     * Field by field, with only the bytes the frame actually has, rather
     * than one struct assignment. In standard mode that saves a hundred
     * bytes a frame and is not worth writing this way; in a build with
     * proprietary long frames the slot is 1 kB and almost all of it is
     * usually unused, and this loop runs once per frame forwarded. */
    const struct capture_frame *slot = &rx_ring[rx_read_upto % RX_RING];

    memcpy(out->data, slot->data, slot->length);
    out->length   = slot->length;
    out->reported = slot->reported;
    out->seq      = slot->seq;
    out->flags    = slot->flags;
    out->wall     = slot->wall;
    if (slot->flags & CAPTURE_F_METADATA)
	out->meta = slot->meta;

    rx_read_upto++;
    stats.consumed++;

    return true;
}

const struct capture_stats *
capture_stats(void)
{
    return &stats;
}

void
capture_to_dissect(struct dissect_frame *out, const struct capture_frame *in)
{
    out->struct_size = sizeof(*out);
    out->data        = in->data;
    out->length      = in->length;
    out->reported    = in->reported;
    out->seq         = in->seq;

    /* Translated a flag at a time rather than copied across as an
     * integer, for the reason wire.c gives for doing the same: the two
     * sets happen to agree today, and a copy would keep working until
     * one of them gained a flag the other does not have. CAPTURE_F_METADATA
     * deliberately has no DISSECT_F_ equivalent: a dissector does not
     * need to know how the capture was configured, only whether the
     * numbers in front of it are real, which the values below say. */
    out->flags = 0;
    if (in->flags & CAPTURE_F_TRUNCATED) out->flags |= DISSECT_F_TRUNCATED;
    if (in->flags & CAPTURE_F_RANGING  ) out->flags |= DISSECT_F_RANGING;

    if (in->flags & CAPTURE_F_METADATA) {
	out->rx_time         = in->meta.rx_time;
	out->clock_offset    = in->meta.clock_offset;
	out->clock_interval  = in->meta.clock_interval;
	out->power_signal    = in->meta.power_signal;
	out->power_firstpath = in->meta.power_firstpath;
	out->first_path      = in->meta.first_path;
	out->std_noise       = in->meta.std_noise;
	out->max_noise       = in->meta.max_noise;
    } else {
	out->rx_time         = 0;
	out->clock_offset    = 0;
	out->clock_interval  = 0;
	out->power_signal    = DISSECT_POWER_NONE;
	out->power_firstpath = DISSECT_POWER_NONE;
	out->first_path      = 0;
	out->std_noise       = 0;
	out->max_noise       = 0;
    }
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
