/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * A pcapng writer for captured UWB frames: exactly the three block types
 * pcapng-spec.md describes (Section Header, Interface Description,
 * Enhanced Packet), in native byte order, and no other pcapng feature.
 * Wireshark and tcpdump both read this with no plugin, which classic
 * pcap cannot be made to do for a per-packet comment: that is the whole
 * reason this format was picked over the one the rest of the program
 * already knows (see sniffer/README.md; today the only output path
 * wraps a frame in an ethernet frame that no dissector understands).
 *
 * This file touches nothing Linux-specific and nothing chip-specific
 * beyond capture.h, whose struct capture_frame is the writer's one
 * input: no bitters, no <dw1000/dw1000.h> call, nothing that only
 * builds on the Raspberry Pi. That is deliberate, and it is what lets
 * tests/sniffer/pcapng.c link and run on a host that cannot build the
 * rest of the application at all: the block layout is provable with no
 * chip present, the same way tests/probe/format.c proves the probe's
 * line format with no radio.
 *
 * Byte order: pcapng-spec.md is explicit that every multi-byte field is
 * written in the host's own order and never swapped, because the
 * Section Header Block's byte_order_magic is what tells a reader which
 * order was used. Every block also repeats its own total length once
 * before its body and once after; writing the same computed value twice
 * is all that takes, since nothing here needs to seek back and patch it
 * in: every length is known before the first byte of its block goes out.
 */

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "capture.h"
#include "pcapng.h"

/* One open file at a time, exactly as eth.c keeps its socket in a
 * static struct: nothing here is reentrant, and nothing in this
 * program needs it to be. NULL means "not open". */
static FILE *pcapng_file;

/* Round n up to the next multiple of 4: pcapng pads every option value
 * and every packet's data to that boundary, so both writers below share
 * this rather than each spelling out the same mask. */
static size_t
round4(size_t n)
{
    return (n + 3u) & ~(size_t)3u;
}

/* The one place a short fwrite() is turned into a negative errno.
 * Nothing here retries: stdio's own buffering means a short fwrite()
 * only happens once the underlying write(2) has already failed, so
 * there is nothing left worth retrying. */
static int
wr_raw(const void *data, size_t len)
{
    if (len == 0)
	return 0;

    errno = 0;
    if (fwrite(data, 1, len, pcapng_file) != len)
	return errno ? -errno : -EIO;
    return 0;
}

static int
wr_u16(uint16_t v)
{
    return wr_raw(&v, sizeof(v));
}

static int
wr_u32(uint32_t v)
{
    return wr_raw(&v, sizeof(v));
}

static int
wr_u64(uint64_t v)
{
    return wr_raw(&v, sizeof(v));
}

/* 0..3 zero bytes: never more than an option's value or a packet's data
 * here need to round up by. */
static int
wr_pad(size_t len)
{
    static const uint8_t zero[4];

    return wr_raw(zero, len);
}

/* One option: its code, the value's true length (padding excluded, as
 * pcapng-spec.md requires), the value, and the padding that brings the
 * next field back onto a multiple of 4. opt_endofopt carries neither a
 * value nor padding, so it is written directly by the callers below
 * instead of through this. */
static int
wr_option(uint16_t code, const void *value, size_t len)
{
    int rc;

    rc = wr_u16(code);
    if (rc < 0)
	return rc;
    rc = wr_u16((uint16_t)len);
    if (rc < 0)
	return rc;
    rc = wr_raw(value, len);
    if (rc < 0)
	return rc;
    return wr_pad(round4(len) - len);
}

static int
wr_endofopt(void)
{
    int rc;

    rc = wr_u16(0);
    if (rc < 0)
	return rc;
    return wr_u16(0);
}

#define PCAPNG_BLOCKTYPE_SHB	0x0A0D0D0Au
#define PCAPNG_BYTEORDER_MAGIC	0x1A2B3C4Du
#define PCAPNG_BLOCKTYPE_IDB	0x00000001u
#define PCAPNG_BLOCKTYPE_EPB	0x00000006u

#define PCAPNG_OPT_SHB_USERAPPL	4u
#define PCAPNG_OPT_IF_TSRESOL	9u
#define PCAPNG_OPT_COMMENT	1u

/* LINKTYPE_IEEE802_15_4_WITH_FCS, not the _NOFCS one (230): capture.h's
 * note on the CRC says a captured frame is forwarded whole, CRC
 * included, so the with-FCS link type is the one that matches what is
 * actually on the wire here. */
#define PCAPNG_LINKTYPE_IEEE802_15_4_WITH_FCS	195u

static int
write_shb(void)
{
    char   userappl[64];
    int    n;
    size_t vlen, opt_total, total;
    int    rc;

    /* "uwb-sniffer <version>": VERSION is the same build-time macro
     * cmdline.c reports for -V, so a capture and the program that wrote
     * it always agree on what made it. */
    n = snprintf(userappl, sizeof(userappl), "uwb-sniffer %s", VERSION);
    if (n < 0)
	return -EIO;
    vlen = strlen(userappl);	/* what snprintf() actually left behind,
				   even if the call above truncated it */

    opt_total = 4u + round4(vlen);
    total     = 28u + opt_total + 4u;	/* fixed part, one option, opt_endofopt */

    rc = wr_u32(PCAPNG_BLOCKTYPE_SHB);
    if (rc < 0)
	return rc;
    rc = wr_u32((uint32_t)total);
    if (rc < 0)
	return rc;
    rc = wr_u32(PCAPNG_BYTEORDER_MAGIC);
    if (rc < 0)
	return rc;
    rc = wr_u16(1);	/* version_major */
    if (rc < 0)
	return rc;
    rc = wr_u16(0);	/* version_minor */
    if (rc < 0)
	return rc;
    rc = wr_u64(UINT64_C(0xFFFFFFFFFFFFFFFF));	/* section_length: unknown */
    if (rc < 0)
	return rc;

    rc = wr_option(PCAPNG_OPT_SHB_USERAPPL, userappl, vlen);
    if (rc < 0)
	return rc;
    rc = wr_endofopt();
    if (rc < 0)
	return rc;

    return wr_u32((uint32_t)total);
}

static int
write_idb(uint32_t snaplen)
{
    static const uint8_t tsresol = 9;	/* nanoseconds; see pcapng-spec.md */
    uint32_t opt_total, total;
    int      rc;

    opt_total = 4u + round4(1u);
    total     = 20u + opt_total + 4u;	/* fixed part, one option, opt_endofopt */

    rc = wr_u32(PCAPNG_BLOCKTYPE_IDB);
    if (rc < 0)
	return rc;
    rc = wr_u32(total);
    if (rc < 0)
	return rc;
    rc = wr_u16(PCAPNG_LINKTYPE_IEEE802_15_4_WITH_FCS);
    if (rc < 0)
	return rc;
    rc = wr_u16(0);	/* reserved */
    if (rc < 0)
	return rc;
    rc = wr_u32(snaplen);
    if (rc < 0)
	return rc;

    rc = wr_option(PCAPNG_OPT_IF_TSRESOL, &tsresol, 1u);
    if (rc < 0)
	return rc;
    rc = wr_endofopt();
    if (rc < 0)
	return rc;

    return wr_u32(total);
}

/* One human-readable line for the packet comment, filled in only when
 * CAPTURE_F_METADATA says frame->meta was read: rx_time and the two
 * powers are the chip's own account of the frame, drift is the
 * transmitter's clock offset against this receiver's own, and fp/noise
 * are the driver's quality figures (dw1000_rxinfo_t::first_path,
 * std_noise, max_noise; see capture.h).
 *
 * Two fields carry a sentinel rather than every value it can hold. A
 * power of CAPTURE_POWER_NONE (an estimate off a zero preamble
 * accumulation count) prints "n/a" instead of a number, for signal and
 * for firstpath alike, since capture.h documents the very same sentinel
 * for both fields; and a clock_interval of zero (RX_TTCKI not
 * available) prints "drift=n/a" rather than divide by it.
 *
 * Returns the length actually written, excluding the terminating NUL:
 * what wr_option() below needs, and never more than bufsz - 1, so a
 * wide value cannot make this module claim an option longer than what
 * it actually wrote.
 */
static size_t
format_meta_comment(char *buf, size_t bufsz, const struct capture_meta *meta)
{
    char signal[24];
    char firstpath[24];
    char drift[32];

    if (meta->power_signal == CAPTURE_POWER_NONE)
	snprintf(signal, sizeof(signal), "n/a");
    else
	snprintf(signal, sizeof(signal), "%.3fdBm",
		 meta->power_signal / 1000.0);

    if (meta->power_firstpath == CAPTURE_POWER_NONE)
	snprintf(firstpath, sizeof(firstpath), "n/a");
    else
	snprintf(firstpath, sizeof(firstpath), "%.3fdBm",
		 meta->power_firstpath / 1000.0);

    if (meta->clock_interval == 0)
	snprintf(drift, sizeof(drift), "n/a");
    else
	snprintf(drift, sizeof(drift), "%.6f",
		 (double)meta->clock_offset / (double)meta->clock_interval);

    snprintf(buf, bufsz,
	     "rx_time=%" PRIu64 " signal=%s firstpath=%s drift=%s"
	     " fp=%u noise=%u/%u",
	     meta->rx_time, signal, firstpath, drift,
	     (unsigned)meta->first_path,
	     (unsigned)meta->std_noise, (unsigned)meta->max_noise);

    return strlen(buf);
}

/* Large enough for format_meta_comment() at its widest (a 20-digit
 * rx_time, both powers and the drift at their most negative, and the
 * widest fp/noise pair), with room to spare. */
#define PCAPNG_COMMENT_MAX 192

static int
write_epb(const struct capture_frame *frame, const char *note)
{
    uint64_t ns;
    uint32_t captured_len, original_len, total;
    size_t   padded_data;
    char     comment[PCAPNG_COMMENT_MAX];
    size_t   comment_len = 0;
    int      have_comment;
    int      rc;

    ns = (uint64_t)frame->wall.tv_sec * UINT64_C(1000000000)
	 + (uint64_t)frame->wall.tv_nsec;

    captured_len = (uint32_t)frame->length;
    original_len = (uint32_t)frame->reported;
    padded_data  = round4(frame->length);

    /* The radio's account of the frame, then a dissector's, in that order
     * and separated by a bar: the radio part is always the same shape and
     * always true, while the dissector part is whatever a plugin made of
     * the bytes, so putting the dependable half first keeps a comment
     * readable even when the other half is missing or wrong. Either half
     * on its own is a comment; neither means no option list at all. */
    comment[0]   = '\0';
    have_comment = 0;

    if (frame->flags & CAPTURE_F_METADATA) {
	comment_len  = format_meta_comment(comment, sizeof(comment),
					   &frame->meta);
	have_comment = 1;
    }

    if ((note != NULL) && (note[0] != '\0')) {
	int n = snprintf(comment + comment_len, sizeof(comment) - comment_len,
			 "%s%s", have_comment ? " | " : "", note);
	/* snprintf() reports what it WOULD have written, so a note that
	 * did not fit must not be counted as though it had: the option
	 * length would then run past the buffer. */
	if (n > 0) {
	    size_t room = sizeof(comment) - comment_len - 1;
	    comment_len += ((size_t)n > room) ? room : (size_t)n;
	}
	have_comment = 1;
    }

    total = 32u + (uint32_t)padded_data;
    if (have_comment)
	total += (uint32_t)(4u + round4(comment_len)) + 4u;

    rc = wr_u32(PCAPNG_BLOCKTYPE_EPB);
    if (rc < 0)
	return rc;
    rc = wr_u32(total);
    if (rc < 0)
	return rc;
    rc = wr_u32(0);	/* interface_id: the one IDB this writer emits */
    if (rc < 0)
	return rc;
    rc = wr_u32((uint32_t)(ns >> 32));	/* timestamp_high */
    if (rc < 0)
	return rc;
    rc = wr_u32((uint32_t)ns);		/* timestamp_low */
    if (rc < 0)
	return rc;
    rc = wr_u32(captured_len);
    if (rc < 0)
	return rc;
    rc = wr_u32(original_len);
    if (rc < 0)
	return rc;

    rc = wr_raw(frame->data, frame->length);
    if (rc < 0)
	return rc;
    rc = wr_pad(padded_data - frame->length);
    if (rc < 0)
	return rc;

    if (have_comment) {
	rc = wr_option(PCAPNG_OPT_COMMENT, comment, comment_len);
	if (rc < 0)
	    return rc;
	rc = wr_endofopt();
	if (rc < 0)
	    return rc;
    }

    return wr_u32(total);
}

int
pcapng_open(const char *path, uint32_t snaplen)
{
    FILE *f;
    int   rc;

    if (pcapng_file != NULL)
	return -EBUSY;

    if (strcmp(path, "-") == 0) {
	f = stdout;
    } else {
	f = fopen(path, "wb");
	if (f == NULL)
	    return -errno;
    }

    pcapng_file = f;

    rc = write_shb();
    if (rc < 0)
	goto fail;
    rc = write_idb(snaplen);
    if (rc < 0)
	goto fail;

    return 0;

 fail:
    if (f != stdout)
	fclose(f);
    pcapng_file = NULL;
    return rc;
}

int
pcapng_write(const struct capture_frame *frame, const char *note)
{
    if (pcapng_file == NULL)
	return -EBADF;

    return write_epb(frame, note);
}

int
pcapng_close(void)
{
    FILE *f = pcapng_file;
    int   rc = 0;

    if (f == NULL)
	return -EBADF;

    /* stdout is not this module's to close: whoever set the process up
     * opened it, long before pcapng_open() ran, and may still want to
     * write to it afterwards. Flushing is what a writer owes it
     * instead, so nothing buffered here is still sitting in stdio when
     * this returns. */
    errno = 0;
    if (f == stdout) {
	if (fflush(f) != 0)
	    rc = errno ? -errno : -EIO;
    } else {
	if (fclose(f) != 0)
	    rc = errno ? -errno : -EIO;
    }

    pcapng_file = NULL;
    return rc;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
