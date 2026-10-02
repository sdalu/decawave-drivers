/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * pcapng.c is provable with no chip and no radio, the same way
 * tests/probe/format.c proves the probe's line format: it never
 * includes <dw1000/dw1000.h> itself, only capture.h's struct
 * capture_frame, so this links pcapng.c alone and drives it with frames
 * built by hand. Checks:
 *
 *  - the Section Header Block and the Interface Description Block,
 *    field by field: both magics, the version, the unknown section
 *    length, linktype 195, the snaplen given to pcapng_open(), the
 *    if_tsresol option (value 9), and opt_endofopt closing each;
 *  - every block read anywhere in this file (through one shared helper)
 *    has its leading and trailing total-length words agree, and both
 *    are multiples of 4;
 *  - an Enhanced Packet Block whose captured length is not a multiple
 *    of 4, so the padding after packet_data is exercised;
 *  - a truncated frame, whose original_len comes out larger than its
 *    captured_len;
 *  - a frame with CAPTURE_F_METADATA carries exactly one opt_comment,
 *    code 1, with the length and text format_meta_comment() specifies,
 *    and a frame without the flag carries no option list at all;
 *  - the two comment sentinels: a power of CAPTURE_POWER_NONE prints
 *    "n/a" in place of a number (signal and firstpath alike), and a
 *    clock_interval of 0 prints "drift=n/a" rather than dividing by
 *    zero;
 *  - the EPB timestamp: bytes 4..7 and 8..11 of the body hold the high
 *    and low 32 bits of ONE 64-bit nanosecond count, not two 32-bit
 *    fields of their own, and not swapped; frame_init()'s fixture
 *    (tv_sec = 1700000000, tv_nsec = 123456789) gives a count whose high
 *    half is non-zero, so a swap, a half written alone, or seconds and
 *    nanoseconds kept as two separate fields all show up here;
 *  - a three-packet file, left on disk (its path is printed) for
 *    tcpdump -r to read outside this program, which is the one check
 *    this file cannot make of itself.
 *
 * One line per case; a failing case says why on its own line. Not run
 * by any tests/check-*.sh or tests/tests-*.sh yet: that wiring is
 * tests-sniffer.sh's, not this file's.
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "capture.h"
#include "pcapng.h"

/* build.sh (around line 127) passes this on the compiler command line,
 * the same way it passes it for cmdline.c's -V; a compile that has not,
 * such as this one, gets a placeholder here rather than one hidden
 * inside pcapng.c, which must never invent a version of its own. */
#ifndef VERSION
#define VERSION "test-0.0.0"
#endif

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

static char reason[256];

#define REASON(...) (snprintf(reason, sizeof(reason), __VA_ARGS__), reason)


/*----------------------------------------------------------------------*/
/* Reading a pcapng file back, byte-exact, with no library but this      */
/*----------------------------------------------------------------------*/

/* Every block this writer can produce fits comfortably under this: the
 * widest is a three-packet file's EPB carrying up to a few hundred
 * bytes of payload plus a comment, and no case here goes further. */
#define BLOCK_BODY_MAX 2048

struct block {
    uint32_t type;
    uint32_t total_len;
    uint8_t  body[BLOCK_BODY_MAX];	/* between the two length words */
    size_t   body_len;			/* total_len - 12 */
};

static uint32_t
get_u32(const uint8_t *p)
{
    uint32_t v;

    memcpy(&v, p, sizeof(v));
    return v;
}

static uint16_t
get_u16(const uint8_t *p)
{
    uint16_t v;

    memcpy(&v, p, sizeof(v));
    return v;
}

static uint64_t
get_u64(const uint8_t *p)
{
    uint64_t v;

    memcpy(&v, p, sizeof(v));
    return v;
}

/* Reads one block, checking the two structural properties pcapng-spec.md
 * requires of every one of them: the leading and trailing total-length
 * words agree, and both are multiples of 4. Every case below reads its
 * blocks through this, so those two properties are proved once for
 * every block in every file this test writes, not just for one of them.
 *
 * Returns 1 with *out filled in, 0 on a clean EOF between blocks, or -1
 * (with *why set) on anything short, mismatched, or misaligned.
 */
static int
read_block(FILE *f, struct block *out, const char **why)
{
    uint8_t  hdr[8];
    uint32_t trailer;
    size_t   n;

    n = fread(hdr, 1, sizeof(hdr), f);
    if (n == 0)
	return 0;
    if (n != sizeof(hdr)) {
	*why = "short block header";
	return -1;
    }

    out->type      = get_u32(hdr);
    out->total_len = get_u32(hdr + 4);

    if (out->total_len % 4 != 0) {
	*why = "block_total_length is not a multiple of 4";
	return -1;
    }
    if (out->total_len < 12) {
	*why = "block_total_length too small to hold its own length words";
	return -1;
    }
    out->body_len = out->total_len - 12;
    if (out->body_len > sizeof(out->body)) {
	*why = "block body larger than this test can hold";
	return -1;
    }

    if (out->body_len > 0 &&
	fread(out->body, 1, out->body_len, f) != out->body_len) {
	*why = "short block body";
	return -1;
    }

    if (fread(&trailer, sizeof(trailer), 1, f) != 1) {
	*why = "short trailing block_total_length";
	return -1;
    }
    if (trailer != out->total_len) {
	*why = "leading and trailing block_total_length disagree";
	return -1;
    }

    return 1;
}


/*----------------------------------------------------------------------*/
/* Building frames                                                       */
/*----------------------------------------------------------------------*/

static void
frame_init(struct capture_frame *f, size_t length, size_t reported)
{
    size_t i;

    memset(f, 0, sizeof(*f));
    for (i = 0; i < length; i++)
	f->data[i] = (uint8_t)(i * 7 + 3);	/* a pattern, not all zero */
    f->length   = length;
    f->reported = reported;
    f->wall.tv_sec  = 1700000000;
    f->wall.tv_nsec = 123456789;
}

/* A fresh path under TMPDIR (or /tmp), fixed per case rather than
 * mkstemp()'d: nothing here runs two cases at once, and a fixed name
 * means the file gate 2 hands to tcpdump -r is exactly the one this
 * prints, with no need to thread a name back out any other way.
 */
static void
test_path(char *buf, size_t bufsz, const char *name)
{
    const char *dir = getenv("TMPDIR");

    if (dir == NULL || dir[0] == '\0')
	dir = "/tmp";
    snprintf(buf, bufsz, "%s/pcapng-test-%s", dir, name);
}


/*----------------------------------------------------------------------*/
/* 1. Section Header Block and Interface Description Block, field by     */
/*    field                                                               */
/*----------------------------------------------------------------------*/

static const char *
case_shb_idb(void)
{
    char         path[256];
    char         userappl[64];
    FILE        *f;
    struct block b;
    const char  *why;
    int          rc;
    size_t       off;
    uint16_t     code, len;

    test_path(path, sizeof(path), "shb-idb.pcapng");

    rc = pcapng_open(path, 1500);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    f = fopen(path, "rb");
    if (f == NULL)
	return REASON("fopen(%s) failed", path);

    /* --- SHB --- */
    rc = read_block(f, &b, &why);
    if (rc != 1) {
	fclose(f);
	return rc == 0 ? "SHB missing" : REASON("SHB: %s", why);
    }
    if (b.type != 0x0A0D0D0Au) {
	fclose(f);
	return REASON("SHB block_type 0x%08x", b.type);
    }
    if (get_u32(&b.body[0]) != 0x1A2B3C4Du) {
	fclose(f);
	return "SHB byte_order_magic wrong";
    }
    if (get_u16(&b.body[4]) != 1 || get_u16(&b.body[6]) != 0) {
	fclose(f);
	return "SHB version is not 1.0";
    }
    if (get_u64(&b.body[8]) != UINT64_C(0xFFFFFFFFFFFFFFFF)) {
	fclose(f);
	return "SHB section_length is not -1 (unknown)";
    }

    snprintf(userappl, sizeof(userappl), "uwb-sniffer %s", VERSION);
    off  = 16;	/* byte_order_magic(4) + version(2+2) + section_length(8) */
    code = get_u16(&b.body[off]);
    len  = get_u16(&b.body[off + 2]);
    if (code != 4) {
	fclose(f);
	return REASON("SHB first option code %u, want 4 (shb_userappl)", code);
    }
    if (len != strlen(userappl) ||
	memcmp(&b.body[off + 4], userappl, len) != 0) {
	fclose(f);
	return REASON("SHB shb_userappl != \"%s\"", userappl);
    }
    off += 4 + ((len + 3u) & ~3u);
    if (get_u16(&b.body[off]) != 0 || get_u16(&b.body[off + 2]) != 0) {
	fclose(f);
	return "SHB missing opt_endofopt after shb_userappl";
    }
    off += 4;
    if (off != b.body_len) {
	fclose(f);
	return REASON("SHB has %zu trailing byte(s) past opt_endofopt",
		      b.body_len - off);
    }

    /* --- IDB --- */
    rc = read_block(f, &b, &why);
    if (rc != 1) {
	fclose(f);
	return rc == 0 ? "IDB missing" : REASON("IDB: %s", why);
    }
    if (b.type != 1) {
	fclose(f);
	return REASON("IDB block_type 0x%08x", b.type);
    }
    if (get_u16(&b.body[0]) != 195) {
	fclose(f);
	return REASON("IDB linktype %u, want 195", get_u16(&b.body[0]));
    }
    if (get_u16(&b.body[2]) != 0) {
	fclose(f);
	return "IDB reserved field is not 0";
    }
    if (get_u32(&b.body[4]) != 1500) {
	fclose(f);
	return REASON("IDB snaplen %u, want 1500", get_u32(&b.body[4]));
    }

    off  = 8;
    code = get_u16(&b.body[off]);
    len  = get_u16(&b.body[off + 2]);
    if (code != 9) {
	fclose(f);
	return REASON("IDB first option code %u, want 9 (if_tsresol)", code);
    }
    if (len != 1 || b.body[off + 4] != 9) {
	fclose(f);
	return "IDB if_tsresol value is not 9 (nanoseconds)";
    }
    off += 4 + 4;	/* 1-byte value padded to 4 */
    if (get_u16(&b.body[off]) != 0 || get_u16(&b.body[off + 2]) != 0) {
	fclose(f);
	return "IDB missing opt_endofopt after if_tsresol";
    }
    off += 4;
    if (off != b.body_len) {
	fclose(f);
	return REASON("IDB has %zu trailing byte(s) past opt_endofopt",
		      b.body_len - off);
    }

    fclose(f);
    unlink(path);
    return NULL;
}


/*----------------------------------------------------------------------*/
/* 2. EPB padding for a captured length that is not a multiple of 4      */
/*----------------------------------------------------------------------*/

static const char *
case_epb_padding(void)
{
    char                 path[256];
    struct capture_frame fr;
    FILE                *f;
    struct block         b;
    const char          *why;
    int                  rc;

    test_path(path, sizeof(path), "epb-padding.pcapng");
    frame_init(&fr, 7, 7);	/* 7 bytes: not a multiple of 4 */

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_write(&fr, NULL);
    if (rc != 0)
	return REASON("pcapng_write: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    f = fopen(path, "rb");
    if (f == NULL)
	return REASON("fopen(%s) failed", path);

    rc = read_block(f, &b, &why);	/* SHB */
    if (rc != 1) {
	fclose(f);
	return "SHB missing";
    }
    rc = read_block(f, &b, &why);	/* IDB */
    if (rc != 1) {
	fclose(f);
	return "IDB missing";
    }

    rc = read_block(f, &b, &why);
    fclose(f);
    unlink(path);
    if (rc != 1)
	return rc == 0 ? "EPB missing" : REASON("EPB: %s", why);

    if (b.type != 6)
	return REASON("EPB block_type 0x%08x", b.type);
    if (get_u32(&b.body[12]) != 7)
	return REASON("EPB captured_len %u, want 7", get_u32(&b.body[12]));
    if (get_u32(&b.body[16]) != 7)
	return REASON("EPB original_len %u, want 7", get_u32(&b.body[16]));

    /* body layout: interface_id(4) ts_high(4) ts_low(4) captured_len(4)
     * original_len(4) = 20, then 7 bytes of packet data padded to 8,
     * then nothing else (no metadata: no option list at all). */
    if (memcmp(&b.body[20], fr.data, 7) != 0)
	return "EPB packet data does not match the frame";
    if (b.body[20 + 7] != 0)
	return "EPB padding byte is not zero";
    if (b.body_len != 20 + 8)
	return REASON("EPB body_len %zu, want %d (no options)",
		      b.body_len, 20 + 8);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 3. A truncated frame: original_len > captured_len                     */
/*----------------------------------------------------------------------*/

static const char *
case_epb_truncated(void)
{
    char                 path[256];
    struct capture_frame fr;
    FILE                *f;
    struct block         b;
    const char          *why;
    int                  rc;

    test_path(path, sizeof(path), "epb-truncated.pcapng");
    frame_init(&fr, 20, 200);	/* driver reported 200, only 20 kept */
    fr.flags = CAPTURE_F_TRUNCATED;

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_write(&fr, NULL);
    if (rc != 0)
	return REASON("pcapng_write: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    f = fopen(path, "rb");
    if (f == NULL)
	return REASON("fopen(%s) failed", path);
    read_block(f, &b, &why);	/* SHB, checked field by field elsewhere */
    read_block(f, &b, &why);	/* IDB, likewise */
    rc = read_block(f, &b, &why);
    fclose(f);
    unlink(path);
    if (rc != 1)
	return rc == 0 ? "EPB missing" : REASON("EPB: %s", why);

    if (get_u32(&b.body[12]) != 20)
	return REASON("EPB captured_len %u, want 20", get_u32(&b.body[12]));
    if (get_u32(&b.body[16]) != 200)
	return REASON("EPB original_len %u, want 200", get_u32(&b.body[16]));
    if (get_u32(&b.body[16]) <= get_u32(&b.body[12]))
	return "EPB original_len is not larger than captured_len";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 4. Metadata: a comment option present, and absent                     */
/*----------------------------------------------------------------------*/

/* Formats the same line pcapng.c's format_meta_comment() specifies, so
 * the comparison below is against a string built independently rather
 * than against pcapng.c's own function.
 */
static size_t
expected_comment(char *buf, size_t bufsz, const struct capture_meta *m)
{
    char signal[24], firstpath[24], drift[32];

    if (m->power_signal == CAPTURE_POWER_NONE)
	snprintf(signal, sizeof(signal), "n/a");
    else
	snprintf(signal, sizeof(signal), "%.3fdBm", m->power_signal / 1000.0);

    if (m->power_firstpath == CAPTURE_POWER_NONE)
	snprintf(firstpath, sizeof(firstpath), "n/a");
    else
	snprintf(firstpath, sizeof(firstpath), "%.3fdBm",
		 m->power_firstpath / 1000.0);

    if (m->clock_interval == 0)
	snprintf(drift, sizeof(drift), "n/a");
    else
	snprintf(drift, sizeof(drift), "%.6f",
		 (double)m->clock_offset / (double)m->clock_interval);

    snprintf(buf, bufsz,
	     "rx_time=%" PRIu64 " signal=%s firstpath=%s drift=%s"
	     " fp=%u noise=%u/%u",
	     m->rx_time, signal, firstpath, drift,
	     (unsigned)m->first_path,
	     (unsigned)m->std_noise, (unsigned)m->max_noise);
    return strlen(buf);
}

static const char *
check_metadata_file(const char *path, const struct capture_frame *fr,
		     int want_comment)
{
    FILE        *f;
    struct block b;
    const char  *why;
    int          rc;
    size_t       data_off, opt_off;

    f = fopen(path, "rb");
    if (f == NULL)
	return REASON("fopen(%s) failed", path);
    read_block(f, &b, &why);	/* SHB, checked field by field elsewhere */
    read_block(f, &b, &why);	/* IDB, likewise */
    rc = read_block(f, &b, &why);
    fclose(f);
    unlink(path);
    if (rc != 1)
	return rc == 0 ? "EPB missing" : REASON("EPB: %s", why);

    data_off = 20;
    opt_off  = data_off + ((fr->length + 3u) & ~(size_t)3u);

    if (!want_comment) {
	if (opt_off != b.body_len)
	    return "EPB carries options although CAPTURE_F_METADATA is clear";
	return NULL;
    }

    if (opt_off == b.body_len)
	return "EPB has no option list although CAPTURE_F_METADATA is set";

    {
	char     expected[192];
	size_t   elen = expected_comment(expected, sizeof(expected), &fr->meta);
	uint16_t code  = get_u16(&b.body[opt_off]);
	uint16_t len   = get_u16(&b.body[opt_off + 2]);

	if (code != 1)
	    return REASON("EPB comment option code %u, want 1", code);
	if (len != elen)
	    return REASON("EPB comment length %u, want %zu", len, elen);
	if (memcmp(&b.body[opt_off + 4], expected, elen) != 0)
	    return REASON("EPB comment '%.*s' != '%s'", (int)len,
			  (const char *)&b.body[opt_off + 4], expected);

	opt_off += 4 + ((elen + 3u) & ~(size_t)3u);
	if (get_u16(&b.body[opt_off]) != 0 ||
	    get_u16(&b.body[opt_off + 2]) != 0)
	    return "EPB missing opt_endofopt after the comment";
	opt_off += 4;
	if (opt_off != b.body_len)
	    return "EPB has bytes past opt_endofopt";
    }

    return NULL;
}

static const char *
case_epb_with_metadata(void)
{
    char                 path[256];
    struct capture_frame fr;
    int                  rc;

    test_path(path, sizeof(path), "epb-meta.pcapng");
    frame_init(&fr, 10, 10);
    fr.flags = CAPTURE_F_METADATA;
    fr.meta.rx_time         = UINT64_C(123456789012);
    fr.meta.clock_offset    = -1500;
    fr.meta.clock_interval  = 3000000;
    fr.meta.power_signal    = -88250;
    fr.meta.power_firstpath = -90100;
    fr.meta.first_path      = 42;
    fr.meta.std_noise       = 10;
    fr.meta.max_noise       = 20;

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_write(&fr, NULL);
    if (rc != 0)
	return REASON("pcapng_write: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    return check_metadata_file(path, &fr, 1);
}

static const char *
case_epb_without_metadata(void)
{
    char                 path[256];
    struct capture_frame fr;
    int                  rc;

    test_path(path, sizeof(path), "epb-nometa.pcapng");
    frame_init(&fr, 10, 10);
    fr.flags = 0;	/* CAPTURE_F_METADATA clear: meta was never read */

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_write(&fr, NULL);
    if (rc != 0)
	return REASON("pcapng_write: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    return check_metadata_file(path, &fr, 0);
}


/*----------------------------------------------------------------------*/
/* 5. The two comment sentinels                                          */
/*----------------------------------------------------------------------*/

static const char *
case_comment_sentinels(void)
{
    char                 path[256];
    struct capture_frame fr;
    int                  rc;
    const char          *err;
    char                 expected[192];

    test_path(path, sizeof(path), "epb-sentinels.pcapng");
    frame_init(&fr, 4, 4);
    fr.flags = CAPTURE_F_METADATA;
    fr.meta.rx_time         = 1;
    fr.meta.clock_offset    = 5;
    fr.meta.clock_interval  = 0;			/* -> drift=n/a */
    fr.meta.power_signal    = CAPTURE_POWER_NONE;	/* -> signal=n/a */
    fr.meta.power_firstpath = CAPTURE_POWER_NONE;	/* -> firstpath=n/a */
    fr.meta.first_path      = 1;
    fr.meta.std_noise       = 2;
    fr.meta.max_noise       = 3;

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_write(&fr, NULL);
    if (rc != 0)
	return REASON("pcapng_write: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    err = check_metadata_file(path, &fr, 1);
    if (err != NULL)
	return err;

    expected_comment(expected, sizeof(expected), &fr.meta);
    if (strstr(expected, "signal=n/a") == NULL)
	return "sentinel fixture: signal=n/a not in the expected string";
    if (strstr(expected, "firstpath=n/a") == NULL)
	return "sentinel fixture: firstpath=n/a not in the expected string";
    if (strstr(expected, "drift=n/a") == NULL)
	return "sentinel fixture: drift=n/a not in the expected string";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 6. The EPB timestamp: one 64-bit nanosecond count, not two 32-bit     */
/*    halves of their own                                                */
/*----------------------------------------------------------------------*/

/* frame->wall is a struct timespec (seconds, nanoseconds); the EPB wants
 * one 64-bit count of nanoseconds since the epoch, split into its high
 * and low 32 bits at bytes 4..7 and 8..11 of the body (after
 * interface_id). This is the one field in the whole writer where "split
 * into two 32-bit fields" and "two fields that already happen to be
 * 32-bit and 32-bit-ish" (seconds, nanoseconds) are easy to conflate, so
 * it gets its own case rather than riding along inside another one:
 * every other case here only ever reads captured_len onward (byte 12+)
 * and would not notice ts_high and ts_low being wrong, missing, or
 * swapped.
 *
 * frame_init()'s fixture (tv_sec = 1700000000, tv_nsec = 123456789)
 * multiplies out to a count whose top 32 bits are non-zero, which is
 * what makes a swap, a half dropped, or the two source fields written
 * as-is instead of combined all visibly wrong here rather than
 * accidentally right.
 */
static const char *
case_epb_timestamp(void)
{
    char                 path[256];
    struct capture_frame fr;
    FILE                *f;
    struct block         b;
    const char          *why;
    int                  rc;
    uint64_t             want, got;

    test_path(path, sizeof(path), "epb-timestamp.pcapng");
    frame_init(&fr, 8, 8);

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_write(&fr, NULL);
    if (rc != 0)
	return REASON("pcapng_write: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    f = fopen(path, "rb");
    if (f == NULL)
	return REASON("fopen(%s) failed", path);
    read_block(f, &b, &why);	/* SHB, checked field by field elsewhere */
    read_block(f, &b, &why);	/* IDB, likewise */
    rc = read_block(f, &b, &why);
    fclose(f);
    unlink(path);
    if (rc != 1)
	return rc == 0 ? "EPB missing" : REASON("EPB: %s", why);

    want = (uint64_t)fr.wall.tv_sec * UINT64_C(1000000000)
	   + (uint64_t)fr.wall.tv_nsec;
    if ((want >> 32) == 0)
	return "test fixture bug: frame_init()'s timestamp has a zero"
	       " high half, which cannot exercise a swap";

    /* body layout: interface_id(4) ts_high(4) ts_low(4) ...; ts_high is
     * bytes 4..7, ts_low bytes 8..11. */
    got = ((uint64_t)get_u32(&b.body[4]) << 32) | get_u32(&b.body[8]);
    if (got != want)
	return REASON("EPB timestamp %" PRIu64 " ns, want %" PRIu64 " ns",
		      got, want);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 8. The note: a dissector's line, appended to the packet comment       */
/*----------------------------------------------------------------------*/

/* pcapng_write()'s second argument is where a dissector's one-line
 * reading of the frame goes. Three things have to hold and none of them
 * is obvious from the other cases: a note joins the radio metadata after
 * a bar rather than replacing it; a note on a frame with no metadata is a
 * comment all by itself, so the option list appears where it otherwise
 * would not; and a note too long for the buffer must leave the option's
 * length agreeing with what was actually written, because snprintf()
 * reports what it WOULD have written and counting that would run the
 * option past the end of the comment.
 */

/* Read the one comment option out of a file's single EPB. Returns the
 * option's length through @p len_out and points @p txt_out at its bytes
 * inside @p b, or a reason. A file whose EPB carries no option list at
 * all reports length 0. */
static const char *
read_epb_comment(const char *path, const struct capture_frame *fr,
		 struct block *b, const uint8_t **txt_out, size_t *len_out)
{
    FILE       *f;
    const char *why;
    int         rc;
    size_t      opt_off;

    f = fopen(path, "rb");
    if (f == NULL)
	return REASON("fopen(%s) failed", path);
    read_block(f, b, &why);	/* SHB */
    read_block(f, b, &why);	/* IDB */
    rc = read_block(f, b, &why);
    fclose(f);
    unlink(path);
    if (rc != 1)
	return rc == 0 ? "EPB missing" : REASON("EPB: %s", why);

    opt_off = 20 + ((fr->length + 3u) & ~(size_t)3u);
    if (opt_off == b->body_len) {
	*txt_out = NULL;
	*len_out = 0;
	return NULL;
    }

    if (get_u16(&b->body[opt_off]) != 1)
	return REASON("first option code %u, want 1 (opt_comment)",
		      get_u16(&b->body[opt_off]));

    *len_out = get_u16(&b->body[opt_off + 2]);
    *txt_out = &b->body[opt_off + 4];
    return NULL;
}

static const char *
case_epb_note(void)
{
    char                 path[256];
    struct capture_frame fr;
    struct block         b;
    const uint8_t       *txt;
    size_t               len;
    const char          *err;
    int                  rc;

    /* (a) metadata AND a note: the radio half, a bar, then the note. */
    test_path(path, sizeof(path), "epb-note-both.pcapng");
    frame_init(&fr, 4, 4);
    fr.flags = CAPTURE_F_METADATA;

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_write(&fr, "spank ftmbc round=3");
    if (rc != 0)
	return REASON("pcapng_write: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    err = read_epb_comment(path, &fr, &b, &txt, &len);
    if (err != NULL)
	return err;
    if (len == 0)
	return "metadata plus a note produced no comment at all";
    {
	char     expected[256];
	size_t   elen = expected_comment(expected, sizeof(expected), &fr.meta);
	int      n    = snprintf(expected + elen, sizeof(expected) - elen,
				 " | spank ftmbc round=3");
	size_t   want = elen + (size_t)n;

	if (len != want)
	    return REASON("comment length %zu, want %zu", len, want);
	if (memcmp(txt, expected, want) != 0)
	    return REASON("comment '%.*s' != '%s'", (int)len,
			  (const char *)txt, expected);
    }

    /* (b) a note on a frame with NO metadata: the note alone, and an
     * option list where case 5 proved there is none without one. */
    test_path(path, sizeof(path), "epb-note-only.pcapng");
    frame_init(&fr, 4, 4);
    fr.flags = 0;

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_write(&fr, "bare note");
    if (rc != 0)
	return REASON("pcapng_write: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    err = read_epb_comment(path, &fr, &b, &txt, &len);
    if (err != NULL)
	return err;
    if (len != strlen("bare note"))
	return REASON("note-only comment length %zu, want %zu",
		      len, strlen("bare note"));
    if (memcmp(txt, "bare note", len) != 0)
	return REASON("note-only comment '%.*s' != 'bare note'",
		      (int)len, (const char *)txt);

    /* (c) an empty note is not a note: no option list, exactly as NULL. */
    test_path(path, sizeof(path), "epb-note-empty.pcapng");
    frame_init(&fr, 4, 4);
    fr.flags = 0;

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);
    rc = pcapng_write(&fr, "");
    if (rc != 0)
	return REASON("pcapng_write: %d", rc);
    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    err = read_epb_comment(path, &fr, &b, &txt, &len);
    if (err != NULL)
	return err;
    /* txt, not len: a zero-length opt_comment and no option list at all
     * both give len 0, and they are not the same block. read_epb_comment()
     * leaves txt NULL only for the second. */
    if (txt != NULL)
	return REASON("an empty note produced an option list (%zu bytes)", len);

    /* (d) a note far longer than the comment buffer: whatever survives,
     * the option's length must describe bytes that are really there, and
     * the block must still walk (read_epb_comment() re-reads the block's
     * two total-length words, so a length past the end is caught there).
     */
    {
	char huge[1024];

	memset(huge, 'z', sizeof(huge) - 1);
	huge[sizeof(huge) - 1] = '\0';

	test_path(path, sizeof(path), "epb-note-huge.pcapng");
	frame_init(&fr, 4, 4);
	fr.flags = CAPTURE_F_METADATA;

	rc = pcapng_open(path, 2000);
	if (rc != 0)
	    return REASON("pcapng_open: %d", rc);
	rc = pcapng_write(&fr, huge);
	if (rc != 0)
	    return REASON("pcapng_write: %d", rc);
	rc = pcapng_close();
	if (rc != 0)
	    return REASON("pcapng_close: %d", rc);

	err = read_epb_comment(path, &fr, &b, &txt, &len);
	if (err != NULL)
	    return err;
	if (len == 0)
	    return "an overlong note produced no comment at all";
	if ((size_t)(txt - b.body) + len > b.body_len)
	    return REASON("comment option length %zu runs past the block"
			  " body (%zu bytes)", len, b.body_len);
	if (memchr(txt, '\0', len) != NULL)
	    return "comment option covers bytes past the string's end";
    }

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 7. A three-packet file, for tcpdump -r to read outside this program   */
/*----------------------------------------------------------------------*/

static const char *
case_three_packets(void)
{
    char                 path[256];
    struct capture_frame fr;
    FILE                *f;
    struct block         b;
    const char          *why;
    int                  rc, i;

    test_path(path, sizeof(path), "three.pcapng");

    rc = pcapng_open(path, 2000);
    if (rc != 0)
	return REASON("pcapng_open: %d", rc);

    frame_init(&fr, 30, 30);
    rc = pcapng_write(&fr, NULL);
    if (rc != 0)
	return REASON("pcapng_write #1: %d", rc);

    frame_init(&fr, 15, 15);
    fr.flags = CAPTURE_F_METADATA;
    fr.meta.rx_time         = 42;
    fr.meta.power_signal    = -75000;
    fr.meta.power_firstpath = -77500;
    fr.meta.clock_offset    = 100;
    fr.meta.clock_interval  = 1000000;
    rc = pcapng_write(&fr, NULL);
    if (rc != 0)
	return REASON("pcapng_write #2: %d", rc);

    frame_init(&fr, 127, 127);
    rc = pcapng_write(&fr, NULL);
    if (rc != 0)
	return REASON("pcapng_write #3: %d", rc);

    rc = pcapng_close();
    if (rc != 0)
	return REASON("pcapng_close: %d", rc);

    f = fopen(path, "rb");
    if (f == NULL)
	return REASON("fopen(%s) failed", path);
    rc = read_block(f, &b, &why);
    if (rc != 1) {
	fclose(f);
	return "SHB missing";
    }
    rc = read_block(f, &b, &why);
    if (rc != 1) {
	fclose(f);
	return "IDB missing";
    }
    for (i = 0; i < 3; i++) {
	rc = read_block(f, &b, &why);
	if (rc != 1) {
	    fclose(f);
	    return rc == 0 ? REASON("EPB #%d missing", i + 1)
			    : REASON("EPB #%d: %s", i + 1, why);
	}
	if (b.type != 6) {
	    fclose(f);
	    return REASON("EPB #%d block_type 0x%08x", i + 1, b.type);
	}
    }
    rc = read_block(f, &b, &why);
    fclose(f);
    if (rc != 0)
	return "more than three packets in the three-packet file";

    /* Left on disk on purpose: gate 2 (tcpdump -r) is not this program's
     * to run on itself, and the printed path is what the caller needs
     * to run it. */
    printf("pcapng: three-packet file left at %s\n", path);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* Plumbing                                                               */
/*----------------------------------------------------------------------*/

int
main(void)
{
    step("SHB and IDB, field by field",              case_shb_idb());
    step("EPB padding on a non-multiple-of-4 frame", case_epb_padding());
    step("EPB truncated: original_len > captured_len",
					     case_epb_truncated());
    step("EPB with metadata: one opt_comment",       case_epb_with_metadata());
    step("EPB without metadata: no option list",     case_epb_without_metadata());
    step("comment sentinels: n/a for power and drift",
					     case_comment_sentinels());
    step("EPB timestamp: one 64-bit ns count, not two 32-bit halves",
					     case_epb_timestamp());
    step("EPB note: a dissector line joins the comment",
					     case_epb_note());
    step("three-packet file for tcpdump -r",         case_three_packets());

    return failures == 0 ? 0 : 1;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
