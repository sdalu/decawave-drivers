/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The sniffer has never had a test, because its logic used to call
 * straight into the chip. capture.c now reaches the chip only through
 * struct capture_ops (two function pointers), so this drives the ring
 * and the wire encoder with a fake pair instead: no chip, no radio,
 * the same trick tests/probe/format.c uses for the probe's line
 * format. Written against what capture.h and wire.h say, not against
 * capture.c and wire.c's bodies.
 *
 * Covers, in the ring (capture.c):
 *  - capture_init() resets the ring, the sequence number and the stats;
 *  - a put then a get round-trips the payload, the length, the reported
 *    length, the seq and the flags (CAPTURE_F_RANGING included);
 *  - seq counts from 0, never repeats, and gets come out oldest first;
 *  - a get on an empty ring returns false;
 *  - overrun: 17 puts lose exactly the oldest 1 (overrun == 1, the next
 *    get hands back seq 1), and 40 puts with no gets lose the oldest 24
 *    (overrun == 40 - 16, the next get hands back seq 24); in both cases
 *    consumed + overrun accounts for every frame capture_put() took;
 *  - truncation: a reported length past DW1000_FRAME_MAXSIZE is copied
 *    only up to DW1000_FRAME_MAXSIZE, flagged CAPTURE_F_TRUNCATED,
 *    counted in stats.truncated, and frame->reported keeps the full
 *    length while frame->length is the short one; the fake
 *    read_frame_data() fills a byte pattern keyed to the offset it is
 *    given, so what actually landed in the frame is checked, not just
 *    the lengths;
 *  - high_water: never above 1 when drained one at a time, 5 after 5
 *    puts with no gets in between;
 *  - metadata: absent (no CAPTURE_F_METADATA) when capture_ops.read_meta
 *    is NULL, present and byte-for-byte unchanged when it is supplied.
 *
 * Covers, in the wire encoder (wire.c):
 *  - 32 bytes written with no metadata, 64 with;
 *  - every header field at the offset wire.h's table gives, read back
 *    byte by byte (never by casting a struct over the buffer, which
 *    would test the host's layout rather than the wire format);
 *  - the magic, the version, and hdr_len equalling the return value;
 *  - a bufsz smaller than what is needed returns 0 and writes nothing;
 *  - the wall-clock timespec becomes one 64-bit little-endian count of
 *    nanoseconds since the epoch;
 *  - CAPTURE_POWER_NONE and a negative clock_offset survive the round
 *    trip through their unsigned encoding;
 *  - a truncated frame comes out with reported_len > captured_len in
 *    the header.
 *
 * One line per case; a failing case says why on its own line. Run by
 * tests/check-sniffer.sh.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "capture.h"
#include "wire.h"

static int failures;

static void
step(const char *what, const char *reason)
{
    if (reason == NULL) {
	printf("ok: %s\n", what);
    } else {
	printf("FAIL: %s (%s)\n", what, reason);
	failures++;
    }
}

/* Where a case formats the reason it failed for. One case runs at a
 * time, and the reason is printed before the next one starts.
 */
static char reason[256];

#define REASON(...) (snprintf(reason, sizeof(reason), __VA_ARGS__), reason)


/*----------------------------------------------------------------------*/
/* The fake chip: a capture_ops pair with no SPI and no radio		*/
/*----------------------------------------------------------------------*/

/* Fills the buffer with a pattern keyed to the absolute offset, so a
 * test can tell which bytes of a frame actually landed, rather than
 * trusting that they did because the lengths came out right. offset is
 * where in the (conceptual) over-the-air frame this call starts, which
 * is what makes the pattern independent of whether capture_put() reads
 * a frame in one call or several: byte i of the frame is always
 * (offset_of_byte_i) & 0xFF.
 */
static void
fake_read_frame_data(uint8_t *data, size_t length, size_t offset)
{
    size_t i;

    for (i = 0; i < length; i++)
	data[i] = (uint8_t)((offset + i) & 0xFF);
}

static bool fake_meta_called;
static struct capture_meta fake_meta_value;

static void
fake_read_meta(struct capture_meta *meta)
{
    fake_meta_called = true;
    *meta = fake_meta_value;
}

static const struct capture_ops ops_no_meta = {
    .read_frame_data = fake_read_frame_data,
    .read_meta	     = NULL,
};

static const struct capture_ops ops_with_meta = {
    .read_frame_data = fake_read_frame_data,
    .read_meta	     = fake_read_meta,
};

/* True iff data[0 .. length) matches fake_read_frame_data()'s pattern
 * for a frame that started at over-the-air offset 0.
 */
static bool
payload_matches(const uint8_t *data, size_t length)
{
    size_t i;

    for (i = 0; i < length; i++)
	if (data[i] != (uint8_t)(i & 0xFF))
	    return false;
    return true;
}


/*----------------------------------------------------------------------*/
/* 1. capture_init() resets everything					*/
/*----------------------------------------------------------------------*/

static const char *
case_init_resets(void)
{
    struct capture_frame f;
    const struct capture_stats *st;

    capture_init(&ops_no_meta);
    capture_put(20, false);
    capture_put(20, false);
    capture_get(&f);

    capture_init(&ops_no_meta);
    st = capture_stats();
    if (st->captured || st->consumed || st->overrun ||
	st->truncated || st->high_water)
	return REASON("stats not zeroed: captured=%lu consumed=%lu"
		      " overrun=%lu truncated=%lu high_water=%lu",
		      st->captured, st->consumed, st->overrun,
		      st->truncated, st->high_water);
    if (capture_get(&f))
	return "ring not empty after re-init";

    capture_put(30, false);
    if (!capture_get(&f))
	return "put/get failed right after re-init";
    if (f.seq != 0)
	return REASON("seq=%u after re-init, want 0", f.seq);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 2. Round trip: payload, length, reported, seq, flags			*/
/*----------------------------------------------------------------------*/

static const char *
case_roundtrip(void)
{
    struct capture_frame f;

    capture_init(&ops_no_meta);
    capture_put(40, true);

    if (!capture_get(&f))
	return "capture_get() returned false right after a put";
    if (f.length != 40)
	return REASON("length=%zu, want 40", f.length);
    if (f.reported != 40)
	return REASON("reported=%zu, want 40", f.reported);
    if (f.seq != 0)
	return REASON("seq=%u, want 0", f.seq);
    if (!(f.flags & CAPTURE_F_RANGING))
	return "CAPTURE_F_RANGING not set for a ranging frame";
    if (f.flags & CAPTURE_F_TRUNCATED)
	return "CAPTURE_F_TRUNCATED set for a frame well under the limit";
    if (f.flags & CAPTURE_F_METADATA)
	return "CAPTURE_F_METADATA set with read_meta == NULL";
    if (!payload_matches(f.data, f.length))
	return "payload bytes do not match what fake_read_frame_data() wrote";

    capture_init(&ops_no_meta);
    capture_put(15, false);
    if (!capture_get(&f))
	return "capture_get() returned false right after a put (2)";
    if (f.flags & CAPTURE_F_RANGING)
	return "CAPTURE_F_RANGING set for a non-ranging frame";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 3. seq from 0, never repeats, gets come out oldest first		*/
/*----------------------------------------------------------------------*/

static const char *
case_seq_order(void)
{
    struct capture_frame f;
    unsigned i;

    capture_init(&ops_no_meta);
    for (i = 0; i < 5; i++)
	capture_put(20 + i, false);

    for (i = 0; i < 5; i++) {
	if (!capture_get(&f))
	    return REASON("get %u: ring empty too soon", i);
	if (f.seq != i)
	    return REASON("get %u: seq=%u, want %u", i, f.seq, i);
	if (f.reported != 20 + i)
	    return REASON("get %u: reported=%zu, want %u",
			  i, f.reported, 20 + i);
    }
    if (capture_get(&f))
	return "a 6th get succeeded after only 5 puts";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 4. get on an empty ring						*/
/*----------------------------------------------------------------------*/

static const char *
case_get_empty(void)
{
    struct capture_frame f;

    capture_init(&ops_no_meta);
    if (capture_get(&f))
	return "capture_get() returned true on a freshly reset ring";
    return NULL;
}


/*----------------------------------------------------------------------*/
/* 5. Overrun: 17 puts, ring depth 16					*/
/*----------------------------------------------------------------------*/

static const char *
case_overrun_one(void)
{
    struct capture_frame f;
    const struct capture_stats *st;
    unsigned i, n;

    capture_init(&ops_no_meta);
    for (i = 0; i < 17; i++)
	capture_put(20, false);

    if (!capture_get(&f))
	return "ring reported empty after 17 puts";
    if (f.seq != 1)
	return REASON("first seq after 17 puts is %u, want 1", f.seq);

    st = capture_stats();
    if (st->overrun != 1)
	return REASON("overrun=%lu after 17 puts, want 1", st->overrun);

    n = 1; /* the get above */
    while (capture_get(&f))
	n++;
    if (n != 16)
	return REASON("drained %u frames after 17 puts, want 16", n);

    st = capture_stats();
    if (st->captured != 17)
	return REASON("captured=%lu, want 17", st->captured);
    if (st->consumed + st->overrun != st->captured)
	return REASON("consumed(%lu) + overrun(%lu) != captured(%lu)",
		      st->consumed, st->overrun, st->captured);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 6. Overrun: 40 puts, no gets in between				*/
/*----------------------------------------------------------------------*/

static const char *
case_overrun_deep(void)
{
    struct capture_frame f;
    const struct capture_stats *st;
    unsigned i, n;

    capture_init(&ops_no_meta);
    for (i = 0; i < 40; i++)
	capture_put(20, false);

    if (!capture_get(&f))
	return "ring reported empty after 40 puts";
    if (f.seq != 24)
	return REASON("first seq after 40 puts is %u, want 24", f.seq);

    st = capture_stats();
    if (st->overrun != 40 - 16)
	return REASON("overrun=%lu after 40 puts, want %d",
		      st->overrun, 40 - 16);

    n = 1;
    while (capture_get(&f))
	n++;
    if (n != 16)
	return REASON("drained %u frames after 40 puts, want 16", n);

    st = capture_stats();
    if (st->captured != 40)
	return REASON("captured=%lu, want 40", st->captured);
    if (st->consumed + st->overrun != st->captured)
	return REASON("consumed(%lu) + overrun(%lu) != captured(%lu)",
		      st->consumed, st->overrun, st->captured);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 7. Truncation							*/
/*----------------------------------------------------------------------*/

static const char *
case_truncation(void)
{
    struct capture_frame f;
    const struct capture_stats *st;
    size_t reported = DW1000_FRAME_MAXSIZE + 37;

    capture_init(&ops_no_meta);
    capture_put(reported, false);

    if (!capture_get(&f))
	return "capture_get() returned false after a truncating put";
    if (f.reported != reported)
	return REASON("reported=%zu, want %zu", f.reported, reported);
    if (f.length != DW1000_FRAME_MAXSIZE)
	return REASON("length=%zu, want DW1000_FRAME_MAXSIZE (%d)",
		      f.length, DW1000_FRAME_MAXSIZE);
    if (!(f.flags & CAPTURE_F_TRUNCATED))
	return "CAPTURE_F_TRUNCATED not set on a truncating put";
    if (!payload_matches(f.data, f.length))
	return "truncated payload bytes do not match the offset pattern";

    st = capture_stats();
    if (st->truncated != 1)
	return REASON("stats.truncated=%lu, want 1", st->truncated);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 8. high_water							*/
/*----------------------------------------------------------------------*/

static const char *
case_high_water_one_at_a_time(void)
{
    struct capture_frame f;
    const struct capture_stats *st;
    unsigned i;

    capture_init(&ops_no_meta);
    for (i = 0; i < 10; i++) {
	capture_put(20, false);
	if (!capture_get(&f))
	    return REASON("get %u failed right after its put", i);
    }

    st = capture_stats();
    if (st->high_water != 1)
	return REASON("high_water=%lu, want 1", st->high_water);

    return NULL;
}

static const char *
case_high_water_burst(void)
{
    const struct capture_stats *st;
    unsigned i;

    capture_init(&ops_no_meta);
    for (i = 0; i < 5; i++)
	capture_put(20, false);

    st = capture_stats();
    if (st->high_water != 5)
	return REASON("high_water=%lu after 5 puts with no gets, want 5",
		      st->high_water);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 9. Metadata								*/
/*----------------------------------------------------------------------*/

static const char *
case_metadata_absent(void)
{
    struct capture_frame f;

    capture_init(&ops_no_meta);
    fake_meta_called = false;
    capture_put(20, false);

    if (!capture_get(&f))
	return "capture_get() returned false";
    if (f.flags & CAPTURE_F_METADATA)
	return "CAPTURE_F_METADATA set with ops.read_meta == NULL";
    if (fake_meta_called)
	return "read_meta was called although ops.read_meta was NULL";

    return NULL;
}

static const char *
case_metadata_present(void)
{
    struct capture_frame f;

    memset(&fake_meta_value, 0, sizeof(fake_meta_value));
    fake_meta_value.rx_time	    = 0x000000A1B2C3D4E5ull;
    fake_meta_value.clock_offset    = -12345;
    fake_meta_value.clock_interval  = 0xCAFEBABEu;
    fake_meta_value.power_signal    = CAPTURE_POWER_NONE;
    fake_meta_value.power_firstpath = -8825;
    fake_meta_value.first_path	    = 4321;
    fake_meta_value.std_noise	    = 111;
    fake_meta_value.max_noise	    = 222;

    capture_init(&ops_with_meta);
    fake_meta_called = false;
    capture_put(20, false);

    if (!capture_get(&f))
	return "capture_get() returned false";
    if (!(f.flags & CAPTURE_F_METADATA))
	return "CAPTURE_F_METADATA not set with ops.read_meta supplied";
    if (!fake_meta_called)
	return "read_meta was never called although ops.read_meta was set";
    if (memcmp(&f.meta, &fake_meta_value, sizeof(f.meta)) != 0)
	return "frame->meta does not match what read_meta() wrote";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* Wire encoding: little-endian byte readers				*/
/*----------------------------------------------------------------------*/

static uint16_t
rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t
rd32(const uint8_t *p)
{
    return (uint32_t)p[0]
	 | ((uint32_t)p[1] << 8)
	 | ((uint32_t)p[2] << 16)
	 | ((uint32_t)p[3] << 24);
}

static uint64_t
rd64(const uint8_t *p)
{
    uint64_t v = 0;
    int i;

    for (i = 0; i < 8; i++)
	v |= (uint64_t)p[i] << (8 * i);
    return v;
}

/* A capture_frame with plausible, individually distinct field values;
 * every wire test case starts from a copy of this and changes only what
 * it is testing.
 */
static void
base_frame(struct capture_frame *f)
{
    memset(f, 0, sizeof(*f));
    f->length	   = 40;
    f->reported	   = 40;
    f->seq	   = 0x01020304u;
    f->flags	   = 0;
    f->wall.tv_sec  = 1700000000;
    f->wall.tv_nsec = 123456789;
}


/*----------------------------------------------------------------------*/
/* 10. wire_encode(): header, no metadata				*/
/*----------------------------------------------------------------------*/

static const char *
case_wire_header_no_metadata(void)
{
    struct capture_frame f;
    uint8_t buf[WIRE_MAX_SIZE];
    size_t n;
    uint64_t want_wall_ns;

    base_frame(&f);
    f.flags = CAPTURE_F_RANGING;

    memset(buf, 0xAA, sizeof(buf));
    n = wire_encode(buf, sizeof(buf), &f, 111, 222);

    if (n != WIRE_HDR_SIZE)
	return REASON("wire_encode() returned %zu, want %u (no metadata)",
		      n, WIRE_HDR_SIZE);

    if (buf[0] != WIRE_MAGIC0 || buf[1] != WIRE_MAGIC1 ||
	buf[2] != WIRE_MAGIC2 || buf[3] != WIRE_MAGIC3)
	return REASON("magic = %02x %02x %02x %02x",
		      buf[0], buf[1], buf[2], buf[3]);
    if (buf[4] != WIRE_VERSION)
	return REASON("version = %u, want %u", buf[4], WIRE_VERSION);
    if (buf[5] != WIRE_HDR_SIZE)
	return REASON("hdr_len = %u, want %u (no metadata)",
		      buf[5], WIRE_HDR_SIZE);
    if (n != buf[5])
	return REASON("hdr_len (%u) != return value (%zu)", buf[5], n);

    if (!(rd16(buf + 6) & WIRE_F_RANGING))
	return "WIRE_F_RANGING not set for a ranging frame";
    if (rd16(buf + 6) & WIRE_F_METADATA)
	return "WIRE_F_METADATA set although the frame carries no metadata";
    if (rd16(buf + 6) & WIRE_F_TRUNCATED)
	return "WIRE_F_TRUNCATED set for an untruncated frame";

    if (rd32(buf + 8) != f.seq)
	return REASON("seq = %u, want %u", rd32(buf + 8), f.seq);
    if (rd16(buf + 12) != f.reported)
	return REASON("reported_len = %u, want %zu",
		      rd16(buf + 12), f.reported);
    if (rd16(buf + 14) != f.length)
	return REASON("captured_len = %u, want %zu",
		      rd16(buf + 14), f.length);
    if (rd32(buf + 16) != 111u)
	return REASON("lost_ring = %u, want 111", rd32(buf + 16));
    if (rd32(buf + 20) != 222u)
	return REASON("lost_chip = %u, want 222", rd32(buf + 20));

    want_wall_ns = (uint64_t)f.wall.tv_sec * 1000000000ull
		 + (uint64_t)f.wall.tv_nsec;
    if (rd64(buf + 24) != want_wall_ns)
	return REASON("wall_ns = %llu, want %llu",
		      (unsigned long long)rd64(buf + 24),
		      (unsigned long long)want_wall_ns);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 11. wire_encode(): header and metadata block				*/
/*----------------------------------------------------------------------*/

static const char *
case_wire_header_metadata(void)
{
    struct capture_frame f;
    uint8_t buf[WIRE_MAX_SIZE];
    size_t n;

    base_frame(&f);
    f.flags = CAPTURE_F_METADATA;
    f.meta.rx_time	   = 0x000000A1B2C3D4E5ull;
    f.meta.clock_offset	   = -12345;
    f.meta.clock_interval  = 0xCAFEBABEu;
    f.meta.power_signal	   = CAPTURE_POWER_NONE;
    f.meta.power_firstpath = -8825;
    f.meta.first_path	   = 4321;
    f.meta.std_noise	   = 111;
    f.meta.max_noise	   = 222;

    memset(buf, 0xAA, sizeof(buf));
    n = wire_encode(buf, sizeof(buf), &f, 0, 0);

    if (n != WIRE_HDR_SIZE + WIRE_META_SIZE)
	return REASON("wire_encode() returned %zu, want %u (with metadata)",
		      n, WIRE_HDR_SIZE + WIRE_META_SIZE);
    if (buf[5] != WIRE_HDR_SIZE + WIRE_META_SIZE)
	return REASON("hdr_len = %u, want %u (with metadata)",
		      buf[5], WIRE_HDR_SIZE + WIRE_META_SIZE);
    if (!(rd16(buf + 6) & WIRE_F_METADATA))
	return "WIRE_F_METADATA not set although the frame carries metadata";

    if (rd64(buf + 32) != f.meta.rx_time)
	return REASON("rx_time = %llu, want %llu",
		      (unsigned long long)rd64(buf + 32),
		      (unsigned long long)f.meta.rx_time);
    if ((int32_t)rd32(buf + 40) != f.meta.clock_offset)
	return REASON("clock_offset = %d, want %d",
		      (int32_t)rd32(buf + 40), f.meta.clock_offset);
    if (rd32(buf + 44) != f.meta.clock_interval)
	return REASON("clock_interval = %u, want %u",
		      rd32(buf + 44), f.meta.clock_interval);
    if ((int32_t)rd32(buf + 48) != f.meta.power_signal)
	return REASON("power_signal = %d, want %d (CAPTURE_POWER_NONE)",
		      (int32_t)rd32(buf + 48), f.meta.power_signal);
    if ((int32_t)rd32(buf + 48) != WIRE_POWER_NONE)
	return REASON("power_signal on the wire = %d, WIRE_POWER_NONE = %d",
		      (int32_t)rd32(buf + 48), WIRE_POWER_NONE);
    if ((int32_t)rd32(buf + 52) != f.meta.power_firstpath)
	return REASON("power_firstpath = %d, want %d",
		      (int32_t)rd32(buf + 52), f.meta.power_firstpath);
    if (rd16(buf + 56) != f.meta.first_path)
	return REASON("first_path = %u, want %u",
		      rd16(buf + 56), f.meta.first_path);
    if (rd16(buf + 58) != f.meta.std_noise)
	return REASON("std_noise = %u, want %u",
		      rd16(buf + 58), f.meta.std_noise);
    if (rd16(buf + 60) != f.meta.max_noise)
	return REASON("max_noise = %u, want %u",
		      rd16(buf + 60), f.meta.max_noise);
    if (rd16(buf + 62) != 0)
	return REASON("reserved = %u, want 0", rd16(buf + 62));

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 12. wire_encode(): a bufsz too small writes nothing			*/
/*----------------------------------------------------------------------*/

static const char *
case_wire_bufsz_too_small(void)
{
    struct capture_frame f;
    uint8_t buf[WIRE_MAX_SIZE];
    uint8_t before[WIRE_MAX_SIZE];
    size_t n;

    base_frame(&f);
    memset(buf, 0x5A, sizeof(buf));
    memcpy(before, buf, sizeof(buf));
    n = wire_encode(buf, WIRE_HDR_SIZE - 1, &f, 0, 0);
    if (n != 0)
	return REASON("no-metadata, bufsz too small: returned %zu, want 0", n);
    if (memcmp(buf, before, sizeof(buf)) != 0)
	return "no-metadata, bufsz too small: buffer was written to";

    f.flags = CAPTURE_F_METADATA;
    memset(buf, 0x5A, sizeof(buf));
    memcpy(before, buf, sizeof(buf));
    n = wire_encode(buf, WIRE_HDR_SIZE + WIRE_META_SIZE - 1, &f, 0, 0);
    if (n != 0)
	return REASON("with-metadata, bufsz too small: returned %zu, want 0", n);
    if (memcmp(buf, before, sizeof(buf)) != 0)
	return "with-metadata, bufsz too small: buffer was written to";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 13. A truncated frame: reported_len > captured_len on the wire	*/
/*----------------------------------------------------------------------*/

static const char *
case_wire_truncated(void)
{
    struct capture_frame f;
    uint8_t buf[WIRE_MAX_SIZE];
    size_t n;

    base_frame(&f);
    f.length   = 50;
    f.reported = 200;
    f.flags    = CAPTURE_F_TRUNCATED;

    n = wire_encode(buf, sizeof(buf), &f, 0, 0);
    if (n != WIRE_HDR_SIZE)
	return REASON("wire_encode() returned %zu, want %u", n, WIRE_HDR_SIZE);
    if (!(rd16(buf + 6) & WIRE_F_TRUNCATED))
	return "WIRE_F_TRUNCATED not set for a truncated frame";
    if (rd16(buf + 12) != f.reported)
	return REASON("reported_len = %u, want %zu", rd16(buf + 12), f.reported);
    if (rd16(buf + 14) != f.length)
	return REASON("captured_len = %u, want %zu", rd16(buf + 14), f.length);
    if (!(rd16(buf + 12) > rd16(buf + 14)))
	return REASON("reported_len (%u) is not > captured_len (%u)",
		      rd16(buf + 12), rd16(buf + 14));

    return NULL;
}


/*----------------------------------------------------------------------*/
/* Plumbing								*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/* capture_to_dissect(): the flat form a dissector sees                  */
/*----------------------------------------------------------------------*/

/* dissect.h refuses to mention any driver type, because struct
 * capture_frame's size depends on DW1000_WITH_PROPRIETARY_LONG_FRAME and
 * must not cross a plugin boundary. capture_to_dissect() is the one place
 * the two representations meet, so it is the one place that mapping can
 * be got wrong, and the two halves worth checking are the metadata's
 * absence (a plugin must not read a stale slot as a reading) and the flag
 * translation (CAPTURE_F_METADATA deliberately has no DISSECT_F_
 * equivalent, so it must not leak through as some other bit).
 */
static const char *
case_to_dissect(void)
{
    struct capture_frame f;
    struct dissect_frame d;

    /* (a) With metadata: every field carried, flags translated. */
    capture_init(&ops_with_meta);
    fake_meta_value.rx_time         = UINT64_C(0x0011223344);
    fake_meta_value.clock_offset    = -4242;
    fake_meta_value.clock_interval  = 32768000u;
    fake_meta_value.power_signal    = -83250;
    fake_meta_value.power_firstpath = -91125;
    fake_meta_value.first_path      = 0xBEEF;
    fake_meta_value.std_noise       = 0x1234;
    fake_meta_value.max_noise       = 0x5678;
    capture_put(9, true);
    if (!capture_get(&f))
	return "capture_get returned nothing";

    capture_to_dissect(&d, &f);

    if (d.struct_size != sizeof(d))
	return REASON("struct_size %zu, want %zu", d.struct_size, sizeof(d));
    if (d.data != f.data)
	return "data does not point at the captured frame's bytes";
    if ((d.length != f.length) || (d.reported != f.reported))
	return REASON("length %zu/%zu, want %zu/%zu",
		      d.length, d.reported, f.length, f.reported);
    if (d.seq != f.seq)
	return REASON("seq %u, want %u", d.seq, f.seq);

    if (!(d.flags & DISSECT_F_RANGING))
	return "the ranging flag did not survive the translation";
    if (d.flags & DISSECT_F_TRUNCATED)
	return "an untruncated frame came out flagged truncated";
    /* CAPTURE_F_METADATA is 1u << 1 and DISSECT_F_TRUNCATED is 1u << 0,
     * DISSECT_F_RANGING 1u << 2; bit 1 must be clear, since the flat form
     * has no metadata flag and a copied integer would have set it. */
    if (d.flags & (1u << 1))
	return REASON("flags 0x%x has bit 1 set: CAPTURE_F_METADATA leaked"
		      " through as an integer copy", d.flags);

    if (d.rx_time != fake_meta_value.rx_time)
	return "rx_time did not carry";
    if (d.clock_offset != fake_meta_value.clock_offset)
	return "clock_offset did not carry";
    if (d.clock_interval != fake_meta_value.clock_interval)
	return "clock_interval did not carry";
    if (d.power_signal != fake_meta_value.power_signal)
	return "power_signal did not carry";
    if (d.power_firstpath != fake_meta_value.power_firstpath)
	return "power_firstpath did not carry";
    if ((d.first_path != fake_meta_value.first_path) ||
	(d.std_noise  != fake_meta_value.std_noise)  ||
	(d.max_noise  != fake_meta_value.max_noise))
	return "the quality figures did not carry";

    /* (b) Without metadata: the radio fields must read as "nothing was
     * read" rather than as whatever the slot held, so that a dissector
     * cannot mistake a stale value for a measurement. Written into d
     * first, so that a mapping which simply skipped them would be caught
     * here rather than pass on a zeroed struct. */
    capture_init(&ops_no_meta);
    capture_put(5, false);
    if (!capture_get(&f))
	return "capture_get returned nothing (no metadata)";

    memset(&d, 0xA5, sizeof(d));
    capture_to_dissect(&d, &f);

    if (d.rx_time != 0)
	return REASON("rx_time %llu with no metadata, want 0",
		      (unsigned long long)d.rx_time);
    if (d.clock_interval != 0)
	return "clock_interval is not 0 with no metadata";
    if (d.clock_offset != 0)
	return "clock_offset is not 0 with no metadata";
    if (d.power_signal != DISSECT_POWER_NONE)
	return "power_signal is not DISSECT_POWER_NONE with no metadata";
    if (d.power_firstpath != DISSECT_POWER_NONE)
	return "power_firstpath is not DISSECT_POWER_NONE with no metadata";
    if ((d.first_path != 0) || (d.std_noise != 0) || (d.max_noise != 0))
	return "the quality figures are not 0 with no metadata";
    if (d.flags & DISSECT_F_RANGING)
	return "the ranging flag appeared on a frame that had none";

    /* (c) A truncated frame: the flag translates, and both lengths come
     * across so that a dissector can see the one it must bound itself by
     * and the one it must not. */
    capture_init(&ops_no_meta);
    capture_put(DW1000_FRAME_MAXSIZE + 40u, false);
    if (!capture_get(&f))
	return "capture_get returned nothing (truncated)";

    capture_to_dissect(&d, &f);

    if (!(d.flags & DISSECT_F_TRUNCATED))
	return "a truncated frame came out unflagged";
    if (d.length != DW1000_FRAME_MAXSIZE)
	return REASON("length %zu, want %d", d.length, DW1000_FRAME_MAXSIZE);
    if (d.reported != DW1000_FRAME_MAXSIZE + 40u)
	return REASON("reported %zu, want %u", d.reported,
		      DW1000_FRAME_MAXSIZE + 40u);
    if (d.length >= d.reported)
	return "a truncated frame's length is not below its reported length";

    return NULL;
}


int
main(void)
{
    step("capture_init() resets everything",	 case_init_resets());
    step("round trip: payload/length/seq/flags",  case_roundtrip());
    step("seq from 0, oldest first",		  case_seq_order());
    step("get on an empty ring",		  case_get_empty());
    step("overrun: 17 puts, ring of 16",	   case_overrun_one());
    step("overrun: 40 puts, no gets",		   case_overrun_deep());
    step("truncation",				   case_truncation());
    step("high_water: drained one at a time",	   case_high_water_one_at_a_time());
    step("high_water: 5 puts before any get",	   case_high_water_burst());
    step("metadata absent (read_meta == NULL)",	   case_metadata_absent());
    step("metadata present, unchanged",		   case_metadata_present());
    step("wire: header, no metadata",		   case_wire_header_no_metadata());
    step("wire: header and metadata block",	   case_wire_header_metadata());
    step("wire: bufsz too small writes nothing",   case_wire_bufsz_too_small());
    step("wire: truncated reported_len/captured_len", case_wire_truncated());
    step("capture_to_dissect: the flat form for a dissector",
					     case_to_dissect());

    return failures == 0 ? 0 : 1;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
