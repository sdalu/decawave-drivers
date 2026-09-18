/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Emit one wire header, encoded by the real wire.c, together with the
 * values that went into it.
 *
 * This is what keeps tests/lua/uwbs_test.lua honest. The obvious way to
 * test a decoder is to type a header into it by hand and assert on the
 * fields, and the obvious way is wrong: the bytes would be a
 * transcription of wire.h just as the Lua's offsets are, so the test
 * would compare one reading of the documentation against another and
 * agree with itself. Here the bytes come out of wire_encode(), and the
 * expected values come out of the same run that produced them, so what
 * is being compared is the C encoder against the Lua decoder with
 * nothing typed twice.
 *
 * Printed as `key=value` lines, one per field, plus `hex=` for the
 * header itself. Read by tests/lua/uwbs_test.lua through
 * tests/check-lua.sh.
 *
 * usage: emit <case>
 *   meta     a frame with the metadata block
 *   metaonly the metadata block, with the ranging bit clear
 *   nometa   a frame without it
 *   trunc    a truncated frame, reported longer than captured
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "wire.h"

/* The frame's own bytes are not the header's business, but the Lua hands
 * them to the 802.15.4 dissector and the test checks how many, so the
 * payload is a real 802.15.4 frame: FCF 0x8841 (data, PAN id compressed,
 * short addresses), sequence 0x2a, PAN 0xcdab, 0x7856 -> 0x3412, two
 * bytes of payload, then a CRC. */
static const uint8_t frame[] = {
    0x41, 0x88, 0x2a, 0xab, 0xcd, 0x12, 0x34, 0x56, 0x78,
    0xde, 0xad, 0x00, 0x00
};

int
main(int argc, char **argv)
{
    struct capture_frame f;
    uint8_t              buf[WIRE_MAX_SIZE];
    size_t               n;
    const char          *which = (argc > 1) ? argv[1] : "meta";
    unsigned long        lost_ring = 7;
    unsigned long        lost_chip = 3;

    memset(&f, 0, sizeof(f));

    memcpy(f.data, frame, sizeof(frame));
    f.length        = sizeof(frame);
    f.reported      = sizeof(frame);
    f.seq           = 4242;
    f.flags         = CAPTURE_F_RANGING;
    f.wall.tv_sec   = 1700000000;
    f.wall.tv_nsec  = 123456789;

    if (strcmp(which, "nometa") == 0) {
	/* no CAPTURE_F_METADATA: a 32 byte header and nothing after it */
    } else if (strcmp(which, "metaonly") == 0) {
	/* Metadata set and ranging CLEAR, which is the one combination
	 * that tells the two flag bits apart. Every other case here has
	 * both set, so a decoder reading the ranging bit where it meant
	 * the metadata bit would agree with them and be wrong only on
	 * this one. */
	f.flags  = CAPTURE_F_METADATA;
    } else if (strcmp(which, "trunc") == 0) {
	f.flags   |= CAPTURE_F_METADATA | CAPTURE_F_TRUNCATED;
	f.reported = 300;
    } else {
	f.flags   |= CAPTURE_F_METADATA;
    }

    if (f.flags & CAPTURE_F_METADATA) {
	f.meta.rx_time         = UINT64_C(0x0011223344);
	f.meta.clock_offset    = -12345;
	f.meta.clock_interval  = 32768000u;
	f.meta.power_signal    = -83250;
	/* Deliberately the sentinel on one of the two, so that the Lua's
	 * handling of "no usable estimate" is exercised rather than only
	 * its arithmetic. */
	f.meta.power_firstpath = CAPTURE_POWER_NONE;
	f.meta.first_path      = 0xBEEF;
	f.meta.std_noise       = 0x1234;
	f.meta.max_noise       = 0x5678;
    }

    n = wire_encode(buf, sizeof(buf), &f, lost_ring, lost_chip);
    if (n == 0) {
	fprintf(stderr, "emit: wire_encode refused\n");
	return 1;
    }

    printf("hex=");
    for (size_t i = 0; i < n; i++)
	printf("%02x", buf[i]);
    printf("\n");

    printf("framehex=");
    for (size_t i = 0; i < f.length; i++)
	printf("%02x", f.data[i]);
    printf("\n");

    printf("hdr_len=%zu\n",      n);
    printf("version=%d\n",       WIRE_VERSION);
    printf("seq=%" PRIu32 "\n",  f.seq);
    printf("reported=%zu\n",     f.reported);
    printf("captured=%zu\n",     f.length);
    printf("lost_ring=%lu\n",    lost_ring);
    printf("lost_chip=%lu\n",    lost_chip);
    printf("wall_sec=%lld\n",    (long long)f.wall.tv_sec);
    printf("wall_nsec=%ld\n",    (long)f.wall.tv_nsec);
    printf("truncated=%d\n",     (f.flags & CAPTURE_F_TRUNCATED) ? 1 : 0);
    printf("ranging=%d\n",       (f.flags & CAPTURE_F_RANGING)   ? 1 : 0);
    printf("has_meta=%d\n",      (f.flags & CAPTURE_F_METADATA)  ? 1 : 0);

    if (f.flags & CAPTURE_F_METADATA) {
	printf("rx_time=%" PRIu64 "\n",  f.meta.rx_time);
	printf("clock_offset=%" PRId32 "\n",   f.meta.clock_offset);
	printf("clock_interval=%" PRIu32 "\n", f.meta.clock_interval);
	printf("power_signal=%" PRId32 "\n",   f.meta.power_signal);
	printf("power_firstpath=%" PRId32 "\n", f.meta.power_firstpath);
	printf("first_path=%u\n", (unsigned)f.meta.first_path);
	printf("std_noise=%u\n",  (unsigned)f.meta.std_noise);
	printf("max_noise=%u\n",  (unsigned)f.meta.max_noise);
    }

    return 0;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
