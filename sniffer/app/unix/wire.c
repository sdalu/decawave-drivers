/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The encoder for the header wire.h describes, and nothing else: no
 * socket, no chip, no Linux. Kept apart from eth.c for two reasons. The
 * format is the part another program has to agree with, so it is worth
 * being able to read on its own; and it is the part that can be proved
 * on any host, which tests/sniffer/capture.c does, while eth.c cannot be
 * compiled anywhere but Linux.
 *
 * Byte by byte, little endian, rather than a packed struct written
 * straight out. A struct would say the same thing on the two machines
 * this actually runs between and a different thing on the next one:
 * alignment, padding and byte order are all the compiler's to choose,
 * and a wire format is precisely what must not depend on them. It costs
 * a few stores per frame, against a sendmsg(2) in the same iteration.
 */

#include "wire.h"

/* Little endian stores. Named for their width so that a field's size and
 * the store used for it cannot drift apart when a field is added. */

static void
_put8(uint8_t *p, uint8_t v)
{
    p[0] = v;
}

static void
_put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)( v        & 0xff);
    p[1] = (uint8_t)((v >>  8) & 0xff);
}

static void
_put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)( v        & 0xff);
    p[1] = (uint8_t)((v >>  8) & 0xff);
    p[2] = (uint8_t)((v >> 16) & 0xff);
    p[3] = (uint8_t)((v >> 24) & 0xff);
}

static void
_put64(uint8_t *p, uint64_t v)
{
    _put32(p    , (uint32_t)( v        & 0xffffffffu));
    _put32(p + 4, (uint32_t)((v >> 32) & 0xffffffffu));
}

size_t
wire_encode(uint8_t *buf, size_t bufsz,
	    const struct capture_frame *frame,
	    unsigned long lost_ring, unsigned long lost_chip)
{
    const bool   meta = (frame->flags & CAPTURE_F_METADATA) != 0;
    const size_t len  = WIRE_HDR_SIZE + (meta ? WIRE_META_SIZE : 0);

    if (bufsz < len)
	return 0;

    /* The flags are re-derived rather than copied across as an integer.
     * WIRE_F_* and CAPTURE_F_* happen to have the same values today, and
     * a memcpy of one into the other would keep working right up until
     * one of the two gained a flag the other does not have, at which
     * point it would put a meaningless bit on the wire. */
    uint16_t flags = 0;
    if (frame->flags & CAPTURE_F_TRUNCATED) flags |= WIRE_F_TRUNCATED;
    if (frame->flags & CAPTURE_F_METADATA ) flags |= WIRE_F_METADATA;
    if (frame->flags & CAPTURE_F_RANGING  ) flags |= WIRE_F_RANGING;

    /* The two lengths are clamped into their 16 bits rather than
     * truncated silently. DW1000_FRAME_MAXSIZE is 1023 at its largest, so
     * neither can reach 65535 from this program; the clamp is here
     * because a length field that wraps is the one error a receiver
     * cannot detect, and it costs a comparison. */
    uint16_t reported = frame->reported > 0xffffu
	              ? 0xffffu : (uint16_t)frame->reported;
    uint16_t captured = frame->length   > 0xffffu
	              ? 0xffffu : (uint16_t)frame->length;

    /* Cumulative, and saturating at 32 bits rather than wrapping: a
     * receiver reading a loss count that went backwards would conclude
     * frames had been un-lost. A run that loses four billion frames has
     * other things to report. */
    uint32_t ring32 = (uint64_t)lost_ring > 0xffffffffu
	            ? 0xffffffffu : (uint32_t)lost_ring;
    uint32_t chip32 = (uint64_t)lost_chip > 0xffffffffu
	            ? 0xffffffffu : (uint32_t)lost_chip;

    _put8 (buf +  0, WIRE_MAGIC0);
    _put8 (buf +  1, WIRE_MAGIC1);
    _put8 (buf +  2, WIRE_MAGIC2);
    _put8 (buf +  3, WIRE_MAGIC3);
    _put8 (buf +  4, WIRE_VERSION);
    _put8 (buf +  5, (uint8_t)len);
    _put16(buf +  6, flags);
    _put32(buf +  8, frame->seq);
    _put16(buf + 12, reported);
    _put16(buf + 14, captured);
    _put32(buf + 16, ring32);
    _put32(buf + 20, chip32);
    _put64(buf + 24, (uint64_t)frame->wall.tv_sec * 1000000000ULL
	           + (uint64_t)frame->wall.tv_nsec);

    if (meta) {
	const struct capture_meta *m = &frame->meta;

	_put64(buf + 32, m->rx_time);
	_put32(buf + 40, (uint32_t)m->clock_offset);
	_put32(buf + 44, m->clock_interval);
	_put32(buf + 48, (uint32_t)m->power_signal);
	_put32(buf + 52, (uint32_t)m->power_firstpath);
	_put16(buf + 56, m->first_path);
	_put16(buf + 58, m->std_noise);
	_put16(buf + 60, m->max_noise);
	_put16(buf + 62, 0);
    }

    return len;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
