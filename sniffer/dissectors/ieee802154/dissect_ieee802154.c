/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * A dissector for the IEEE 802.15.4 MAC header, and the reference for
 * writing one.
 *
 * It is here for two reasons. Every frame this sniffer sees is
 * 802.15.4-shaped, so reading the addresses off one is useful on its own
 * and needs no knowledge of what the payload means. And it is the worked
 * example the interface is documented by: it registers both ways (linked
 * in through DISSECTORS=, or loaded through --dissector=), it uses all
 * three hooks (open, accept, describe), and it is deliberately free of
 * anything outside <dissect.h> and the C library, which is what any
 * dissector for this program may assume.
 *
 * It parses the header and stops. What the payload means belongs to
 * whatever protocol is inside, and that protocol's definition lives in
 * its own repository: a dissector for it goes in a tree of its own, which
 * is exactly what DISSECTORS= is for. For spank, that tree would include
 * <spank/packet.h> and read the layout from the source of truth rather
 * than keeping a second copy of it here.
 *
 * Building this as a shared object:
 *
 *     make plugin                # -> ieee802154.so
 *     uwb-sniffer --dissector=./ieee802154.so ...
 *
 * Compiling it in instead:
 *
 *     DISSECTORS=$PWD/sniffer/dissectors/ieee802154 \
 *         sh sniffer/app/unix/build.sh
 *
 * Nothing in this file calls into the sniffer, which is what lets the
 * same source serve both routes. register.c holds the one call that
 * does, and explains why it has to be somewhere else.
 */

#include <stdio.h>
#include <string.h>

#include "ieee802154.h"

/* UM is not the reference here; IEEE 802.15.4-2011 §5.2.1.1 is. The frame
 * control field is two octets, little endian on the air:
 *
 *   bits 0..2    frame type
 *   bit  3       security enabled
 *   bit  4       frame pending
 *   bit  5       acknowledgement request
 *   bit  6       PAN id compression
 *   bits 7..9    reserved
 *   bits 10..11  destination addressing mode
 *   bits 12..13  frame version
 *   bits 14..15  source addressing mode
 */
#define FCF_TYPE(f)		( (f)        & 0x7u)
#define FCF_SECURITY(f)		(((f) >>  3) & 0x1u)
#define FCF_PENDING(f)		(((f) >>  4) & 0x1u)
#define FCF_ACKREQ(f)		(((f) >>  5) & 0x1u)
#define FCF_PANCOMP(f)		(((f) >>  6) & 0x1u)
#define FCF_DSTMODE(f)		(((f) >> 10) & 0x3u)
#define FCF_VERSION(f)		(((f) >> 12) & 0x3u)
#define FCF_SRCMODE(f)		(((f) >> 14) & 0x3u)

#define ADDR_NONE		0u
#define ADDR_RESERVED		1u
#define ADDR_SHORT		2u
#define ADDR_EXTENDED		3u

/* Frame types, and the mask --dissector=...:types= builds out of them. */
#define TYPE_BEACON		0u
#define TYPE_DATA		1u
#define TYPE_ACK		2u
#define TYPE_CMD		3u
#define TYPE_COUNT		8u

/* The CRC the radio leaves on the end of every frame. capture.h explains
 * why it is still there: a sniffer forwards the frame whole. It is not
 * part of the header, so the header parse must not walk into it. */
#define CRC_LENGTH		2u

static unsigned accept_mask = 0xffu;	/* every type, until open() says else */

static const char *
_type_name(unsigned type)
{
    switch (type) {
    case TYPE_BEACON:	return "beacon";
    case TYPE_DATA:	return "data";
    case TYPE_ACK:	return "ack";
    case TYPE_CMD:	return "cmd";
    default:		return "reserved";
    }
}

/* How many octets an addressing mode costs. ADDR_RESERVED is not a
 * length: a frame using it cannot be walked, which is why the parse below
 * refuses rather than guessing. */
static size_t
_addr_len(unsigned mode)
{
    switch (mode) {
    case ADDR_SHORT:	return 2;
    case ADDR_EXTENDED:	return 8;
    default:		return 0;
    }
}

struct mhr {
    unsigned fcf;
    unsigned type;
    unsigned seq;
    unsigned dst_mode, src_mode;
    bool     have_dst_pan, have_src_pan;
    uint16_t dst_pan, src_pan;
    uint8_t  dst[8], src[8];
    size_t   hdr_len;		/* octets the header took */
    size_t   payload_len;	/* what is left, CRC excluded */
};

/* Read a little-endian u16 at @p off, which the caller has already
 * checked is inside the frame. */
static uint16_t
_u16(const uint8_t *p, size_t off)
{
    return (uint16_t)((uint16_t)p[off] | ((uint16_t)p[off + 1] << 8));
}

/*
 * Parse the MAC header, or refuse.
 *
 * Every step is bounded by @p f->length and not by @p f->reported: a
 * truncated frame holds fewer bytes than the driver said it received,
 * and reading to the reported length would read past the buffer. That is
 * the one rule a dissector for this program must not get wrong, which is
 * why it is stated here as well as in dissect.h.
 *
 * Returns false on anything it cannot account for, which the hooks below
 * turn into "not my frame" rather than into a guess.
 */
static bool
_parse(const struct dissect_frame *f, struct mhr *m)
{
    size_t off;

    memset(m, 0, sizeof(*m));

    /* The shortest legal frame is an acknowledgement: FCF, sequence
     * number, CRC. */
    if (f->length < 2u + 1u + CRC_LENGTH)
	return false;

    m->fcf      = _u16(f->data, 0);
    m->seq      = f->data[2];
    m->type     = FCF_TYPE(m->fcf);
    m->dst_mode = FCF_DSTMODE(m->fcf);
    m->src_mode = FCF_SRCMODE(m->fcf);

    if ((m->dst_mode == ADDR_RESERVED) || (m->src_mode == ADDR_RESERVED))
	return false;

    off = 3;

    /* The addressing fields are present or absent according to the two
     * modes, and the source PAN id is additionally suppressed by the PAN
     * id compression bit (§5.2.1.1.5). Each read is checked against what
     * is actually in the buffer, leaving room for the CRC. */
#define NEED(n)								\
    do {								\
	if ((off + (n)) > (f->length - CRC_LENGTH))			\
	    return false;						\
    } while (0)

    if (m->dst_mode != ADDR_NONE) {
	NEED(2);
	m->dst_pan      = _u16(f->data, off);
	m->have_dst_pan = true;
	off += 2;

	NEED(_addr_len(m->dst_mode));
	memcpy(m->dst, &f->data[off], _addr_len(m->dst_mode));
	off += _addr_len(m->dst_mode);
    }

    if (m->src_mode != ADDR_NONE) {
	if (!FCF_PANCOMP(m->fcf) || (m->dst_mode == ADDR_NONE)) {
	    NEED(2);
	    m->src_pan      = _u16(f->data, off);
	    m->have_src_pan = true;
	    off += 2;
	} else {
	    /* Compressed: the source shares the destination's PAN id. */
	    m->src_pan      = m->dst_pan;
	    m->have_src_pan = m->have_dst_pan;
	}

	NEED(_addr_len(m->src_mode));
	memcpy(m->src, &f->data[off], _addr_len(m->src_mode));
	off += _addr_len(m->src_mode);
    }

#undef NEED

    m->hdr_len     = off;
    m->payload_len = (f->length - CRC_LENGTH) - off;

    return true;
}

/* An address as hex, most significant octet first, which is how both a
 * short and an extended address are written down. */
static void
_addr_str(char *out, size_t outsz, const uint8_t *addr, unsigned mode)
{
    size_t n = _addr_len(mode);
    size_t k = 0;

    if (n == 0) {
	snprintf(out, outsz, "-");
	return;
    }

    k += (size_t)snprintf(out + k, outsz - k, "0x");
    for (size_t i = n; (i > 0) && (k + 2 < outsz); i--)
	k += (size_t)snprintf(out + k, outsz - k, "%02x", addr[i - 1]);
}


/*======================================================================*/
/* The hooks                                                            */
/*======================================================================*/

/*
 * `types=` selects which frame types accept() will pass, by name, comma
 * separated: types=data,ack. Absent, or no args at all, means every
 * type, which is what a dissector linked in through DISSECTORS= always
 * gets, since there is no command line to carry args.
 */
static int
_open(const char *args)
{
    static const struct {
	const char *name;
	unsigned    type;
    } names[] = {
	{ "beacon", TYPE_BEACON },
	{ "data",   TYPE_DATA   },
	{ "ack",    TYPE_ACK    },
	{ "cmd",    TYPE_CMD    },
    };
    const char *p;

    if ((args == NULL) || (args[0] == '\0'))
	return 0;

    p = strstr(args, "types=");
    if (p == NULL) {
	fprintf(stderr, "ieee802154: unrecognised args '%s';"
			" the only one is types=<name>[,<name>]\n", args);
	return -1;
    }
    p += strlen("types=");

    accept_mask = 0;
    while (*p != '\0') {
	size_t len = strcspn(p, ",");
	bool   hit = false;

	for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
	    if ((strlen(names[i].name) == len) &&
		(strncmp(p, names[i].name, len) == 0)) {
		accept_mask |= 1u << names[i].type;
		hit = true;
		break;
	    }
	}

	if (!hit) {
	    fprintf(stderr, "ieee802154: unknown frame type '%.*s'\n",
		    (int)len, p);
	    return -1;
	}

	p += len;
	if (*p == ',')
	    p++;
    }

    if (accept_mask == 0) {
	fprintf(stderr, "ieee802154: types= selected nothing\n");
	return -1;
    }

    return 0;
}

/* A frame is accepted when it parses as 802.15.4 and its type was
 * selected. A frame that does not parse is refused, which is the useful
 * half of this hook: with --dissect-filter it keeps noise off the wire
 * before the sendmsg(2) that would have carried it. */
static bool
_accept(const struct dissect_frame *f)
{
    struct mhr m;

    if (!_parse(f, &m))
	return false;

    return (m.type < TYPE_COUNT) && ((accept_mask & (1u << m.type)) != 0);
}

static size_t
_describe(const struct dissect_frame *f, char *out, size_t outsz)
{
    struct mhr m;
    char       dst[24], src[24];
    int        n;

    /* 0 means "not mine", and the loop then offers the frame to the next
     * dissector. A header this cannot walk is exactly that, rather than
     * something to report a half-parse of. */
    if (!_parse(f, &m))
	return 0;

    _addr_str(dst, sizeof(dst), m.dst, m.dst_mode);
    _addr_str(src, sizeof(src), m.src, m.src_mode);

    n = snprintf(out, outsz,
		 "802.15.4 %s seq=%u %04x/%s -> %04x/%s payload=%zu%s%s%s%s",
		 _type_name(m.type), m.seq,
		 m.have_src_pan ? m.src_pan : 0, src,
		 m.have_dst_pan ? m.dst_pan : 0, dst,
		 m.payload_len,
		 FCF_ACKREQ(m.fcf)   ? " ack-req"  : "",
		 FCF_PENDING(m.fcf)  ? " pending"  : "",
		 FCF_SECURITY(m.fcf) ? " secured"  : "",
		 /* Said plainly: the payload length above is what arrived,
		  * not what was sent, and a consumer that does not know
		  * that will read the number as the frame's real size. */
		 (f->flags & DISSECT_F_TRUNCATED) ? " TRUNCATED" : "");

    if (n < 0)
	return 0;

    return ((size_t)n < outsz) ? (size_t)n : (outsz - 1);
}

/* Not static: register.c needs it, and register.c is a separate
 * translation unit for the reason its own comment gives. */
const struct dissector ieee802154_dissector = {
    .abi      = DISSECT_ABI,
    .name     = "ieee802154",
    .version  = "1",
    .open     = _open,
    .accept   = _accept,
    .describe = _describe,
    .close    = NULL,
};


/*======================================================================*/
/* Registration                                                         */
/*======================================================================*/

/* Only the runtime route is here. The compile-time one is in register.c,
 * because it calls into the sniffer and this file must not: a plugin that
 * references a sniffer symbol will not load. register.c says it at
 * length; DISSECT_SOURCES takes both files, `make plugin` takes only
 * this one.
 *
 * The runtime route: --dissector=./ieee802154.so looks for exactly this
 * symbol. Versioned in the name, so a plugin built against a later
 * interface fails to load rather than loading and being misread.
 * DISSECT_EXPORT is what keeps it visible under the -fvisibility=hidden
 * the Makefile builds with; without it the object loads and exports
 * nothing, which dissect.c reports but cannot explain. */
DISSECT_EXPORT const struct dissector *uwb_dissector_v1(void);

DISSECT_EXPORT const struct dissector *
uwb_dissector_v1(void)
{
    return &ieee802154_dissector;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
