/*
 * Copyright (c) 2019
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <signal.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>

#include <bitters.h>
#include <dw1000/dw1000_validate.h>

#include "config.h"
#include "cmdline.h"
#include "uwb.h"
#include "eth.h"
#include "capture.h"
#include "wire.h"
#include "pcapng.h"
#include "dissect.h"

#ifndef __arraycount
#define	__arraycount(__x)	(sizeof(__x) / sizeof(__x[0]))
#endif

/* A bound on how long a missed interrupt edge can go unnoticed, and
 * nothing else: every real wake-up arrives on the interrupt's file
 * descriptor. The same one second probe/app/unix/main.c uses, for the
 * same reason and with the same reasoning behind it. The loop used to
 * wait forever, which meant that if an edge were ever missed, the
 * program would sit in poll(2) with frames arriving and never look.
 */
#define POLL_TIMEOUT_MS		1000

/* How many events one wake-up may drain before going back to poll(2).
 *
 * dw1000_process_events() takes one snapshot of the status register and
 * handles at most one frame from it, and it returns whether it did: with
 * two frames in the two buffers, one call leaves one of them sitting
 * there. So it is called until it says there was nothing, which is what
 * its return value is for and what the loop used to discard.
 *
 * Bounded rather than a bare `while`, because the condition being waited
 * on is a bit in a register on the other end of an SPI bus: a status bit
 * that nothing here knows how to clear would turn an unbounded loop into
 * a program that never returns to poll(2) again. Sixteen is the ring's
 * depth, so a wake-up can never drain more than the ring can hold
 * anyway, and the poll timeout above is what picks up the remainder.
 */
#define DRAIN_MAX		16

/* How much room a dissector gets for its one line. It shares the pcapng
 * packet comment with the radio metadata, which takes most of
 * pcapng.c's 192 bytes, so this is deliberately the smaller half: a
 * dissector's job here is one line, not a decode tree. */
#define DISSECT_LINE_MAX	128

/* The compile-time dissector route.
 *
 * build.sh defines this to the registration symbol exported by the tree
 * named in DISSECTORS=, and defines it to nothing at all when there is no
 * such tree, which is why every use of it is inside #ifdef rather than
 * behind a runtime test. The macro is the symbol's NAME, so the extern
 * below declares whatever that tree calls its entry point and this file
 * needs to know nothing about it.
 */
#ifdef SNIFFER_DISSECT_INIT
extern void SNIFFER_DISSECT_INIT(void);
#endif



/* Producer half of the ring, and nothing else.
 *
 * Under double buffering this runs with the receiver already armed and the
 * next frame landing in the other buffer, so it does the one thing that
 * cannot wait (the read-outs, inside capture_put()) and returns. What
 * used to be here, a per-frame printf and a blocking sendmsg(2), now
 * happens in loop() below, on what the ring holds. It must not re-enable
 * the receiver: the driver already did. See capture.c.
 *
 * The ranging bit is passed on rather than dropped: it is one bit the
 * chip knows about the frame and a consumer cannot recover.
 */
static void
_rx_ok(uint32_t status, size_t length, bool ranging) {
    (void)status;

    capture_put(length, ranging);
}


/* Set by SIGINT and SIGTERM, read by loop().
 *
 * Ctrl-C used to kill the program outright, which threw away the whole
 * reception account: how many frames were captured, how many were lost
 * where, and how close the ring came to overflowing. Those numbers are
 * the answer to "is this capture complete?", so the one keystroke every
 * run ends with had better not be the one that discards them.
 */
static volatile sig_atomic_t stop_requested;

static void
_on_signal(int sig) {
    (void)sig;
    stop_requested = 1;
}




struct config config = {
    .ifname_default = "eth0",
    .ifname         = config.ifname_default,
    .verbose        = 0,
    .proto          = 9999,
    .tx_delay       = UINT16_MAX,
    .rx_delay       = UINT16_MAX,
    .channel        = 5,
    .bitrate        = 6800,
    .prf            = 64,
    .tx_pcode       = 10,
    .rx_pcode       = 10,
    .tx_plen        = 128,
    .rx_pac         = 8,
    .pcapng         = NULL,
    .count          = 0,
    .stats          = 0,
    .raw            = 0,
    .no_metadata    = 0,
    .no_dblbuff     = 0,
};




/* Radio configuration of the DW1000
 *
 * UM §9.3  : Data rate, preamble length, PRF
 * UM §4.1.1: Preamble detection
 * UM §10.5 : UWB channels and preamble codes
 *
 * Recommanded preamble length for the following bitrates:
 *  6800 kbps :   64 or  128 or  256
 *   850 kbps :  256 or  512 or 1024
 *   110 kbps : 2048 or 4096
 *
 * PLEN (Preamble length) / PAC (Preamble Acquisition Chunk)
 *   tx_plen: 64 | 128 | 256 | 512 | 1024 | 1536 | 2048 | 4096
 *   rx_pac : 8  | 8   | 16  | 16  | 32   | 64   | 64   | 64
 *
 * Recommanded preamble codes according to selected channel and PRF:
 *  Channel | Preamble codes | Preamble codes
 *          | for 16MHz PRF  | for 64 MHz PRF
 * ---------+----------------+----------------
 *     1    |     1, 2       |  9, 10, 11, 12
 *     2    |     3, 4       |  9, 10, 11, 12
 *     3    |     5, 6       |  9, 10, 11, 12
 *     4    |     7, 8       | 17, 18, 19, 20
 *     5    |     3, 4       |  9, 10, 11, 12
 *     7    |     7, 8       | 17, 18, 19, 20
 */
static struct dw1000_radio dw1000_radio = {
    .channel          = 5, // Possible to use 2 or 5 with the same parameters
    .bitrate          = DW1000_BITRATE_6800KBPS,
    .prf              = DW1000_PRF_64MHZ,
    .tx_plen          = DW1000_PLEN_128, // UM §9.3
    .rx_pac           = DW1000_PAC8,     // UM §4.1
    .tx_pcode         = 10,              // UM §10.5
    .rx_pcode         = 10,              // UM §10.5
#if DW1000_WITH_PROPRIETARY_SFD || DW1000_WITH_PROPRIETARY_LONG_FRAME
    .proprietary      = {
#if DW1000_WITH_PROPRIETARY_LONG_FRAME
       .long_frames  = 0,
#endif
#if DW1000_WITH_PROPRIETARY_SFD
       .sfd          = 1,
#endif
    },
#endif
#if DW1000_WITH_SFD_TIMEOUT
    .sfd_timeout      = DW1000_SFD_TIMEOUT_MAX
#endif
};


/* UWB hardware configuration
 * Antenna delay (UINT16_MAX -> using default value)
 */
static struct uwb_config uwb_config = {
    .antenna        = { .tx_delay = UINT16_MAX, .rx_delay = UINT16_MAX },
    .frame_delivery = _rx_ok,
    .dblbuff        = true,
};




/*======================================================================*/
/* Reporting                                                            */
/*======================================================================*/

/* One line per frame, only with -v. It stays out of the callback for the
 * reason the whole ring exists; here it costs the forwarding loop time
 * and nothing else. Without -v the program is quiet once running, which
 * is what you want at any real frame rate.
 */
static void
_report_frame(const struct capture_frame *frame, const char *note)
{
    char meta[160];

    meta[0] = '\0';
    if (frame->flags & CAPTURE_F_METADATA) {
	const struct capture_meta *m = &frame->meta;
	int  n = snprintf(meta, sizeof(meta), " rx_time=%" PRIu64, m->rx_time);

	if ((n > 0) && ((size_t)n < sizeof(meta))) {
	    if (m->power_signal == CAPTURE_POWER_NONE) {
		snprintf(meta + n, sizeof(meta) - n, " signal=n/a");
	    } else {
		snprintf(meta + n, sizeof(meta) - n, " signal=%.1fdBm",
			 m->power_signal / 1000.0);
	    }
	}
    }

    INFO("rx seq=%" PRIu32 " len=%zu/%zu%s%s%s%s%s",
	 frame->seq, frame->length, frame->reported,
	 (frame->flags & CAPTURE_F_TRUNCATED) ? " TRUNCATED" : "",
	 (frame->flags & CAPTURE_F_RANGING)   ? " ranging"   : "",
	 meta,
	 (note != NULL) ? " | " : "", (note != NULL) ? note : "");
}


/* The reception account, on the way out and on --stats.
 *
 * Both halves of it, which is the point: the ring's overrun is the host
 * failing to forward fast enough, the chip's is the host failing to read
 * out fast enough, and they are fixed by different things. high_water
 * against the ring's depth is what says whether the depth needs raising
 * at all, which the TODO that asked for this had no way to answer.
 */
static void
_report_stats(const char *what)
{
    const struct capture_stats  *cs = capture_stats();
    const struct uwb_rx_errors  *re = uwb_rx_errors();

    WARN("%s: captured %lu, forwarded %lu, ring high-water %lu",
	 what, cs->captured, cs->consumed, cs->high_water);

    if (cs->overrun || cs->truncated)
	WARN("%s: lost %lu to the ring (forwarding fell behind),"
	     " truncated %lu",
	     what, cs->overrun, cs->truncated);

    if (dissect_count() > 0) {
	const struct dissect_stats *ds = dissect_stats();

	/* described and unrecognised only move when a line was actually
	 * wanted (that is, under -v or -w): a run with neither never asks
	 * a dissector to describe anything, so they stay at zero while
	 * dropped still counts. dissect.h says the same from the other
	 * side. */
	WARN("%s: dissected %lu, unrecognised %lu, dropped by filter %lu",
	     what, ds->described, ds->unrecognised, ds->dropped);
    }

    if (re->total)
	WARN("%s: chip errors %lu"
	     " (overrun %lu, phy %lu, fcs %lu, sync %lu, lde %lu,"
	     " sfd-timeout %lu, rejected %lu, unexplained %lu)",
	     what, re->total,
	     re->overrun, re->phy, re->fcs, re->sync, re->lde,
	     re->sfd_timeout, re->rejected, re->unexplained);
}




/*======================================================================*/
/* The loop                                                             */
/*======================================================================*/

static int
_forward(const struct capture_frame *frame,
	 unsigned long lost_ring, unsigned long lost_chip)
{
    int         rc   = 0;
    const char *note = NULL;
    char        line[DISSECT_LINE_MAX];

    /* The dissectors, and the only place they run: on the forwarding
     * loop's thread, never in the receive callback. A dissector is
     * exactly the unbounded work the ring exists to keep out of rx_ok,
     * and a slow one shows up here as the ring's high-water mark
     * climbing toward its depth, which is the number that says whether
     * it is affordable.
     *
     * accept() first, so a frame nothing wants costs neither a describe()
     * nor a syscall. describe() only when a line has somewhere to go.
     */
    if (dissect_count() > 0) {
	struct dissect_frame df;

	capture_to_dissect(&df, frame);

	if (config.dissect_filter && !dissect_accept(&df))
	    return 0;

	if ((config.verbose || (config.pcapng != NULL)) &&
	    (dissect_describe(&df, line, sizeof(line)) > 0))
	    note = line;
    }

    if (config.verbose)
	_report_frame(frame, note);

    if (config.pcapng != NULL) {
	if (pcapng_write(frame, note) < 0) {
	    WARN_ERRNO("failed writing to %s", config.pcapng);
	    rc = -1;
	}
    }

    if (config.dst_valid) {
	uint8_t hdr[WIRE_MAX_SIZE];
	size_t  hdrlen = 0;

	/* --raw sends the frame alone, as this program always used to.
	 * It is kept because it is a wire format change: a receiver
	 * written against the old output goes on working with --raw,
	 * and the magic in the new header is what lets a receiver tell
	 * which it is being handed.
	 */
	if (!config.raw) {
	    hdrlen = wire_encode(hdr, sizeof(hdr), frame,
				 lost_ring, lost_chip);
	    if (hdrlen == 0) {
		/* Cannot happen: hdr is WIRE_MAX_SIZE, which is the
		 * largest header there is. Reported rather than ignored
		 * because the alternative is sending a frame with no
		 * header to a receiver that is expecting one. */
		WARN("internal error: the wire header did not fit");
		return -1;
	    }
	}

	if (eth_send(config.dst_addr, hdr, hdrlen,
		     frame->data, frame->length) < 0) {
	    WARN_ERRNO("failed sending ethernet frame");
	    rc = -1;
	}
    }

    return rc;
}


int loop(void) {
    struct pollfd pollfd[1];
    uwb_fill_pollfd(&pollfd[0]);

    /* Armed once, here, and not again per iteration. Under double
     * buffering the driver re-enables the receiver itself in the
     * good-frame path, and errors re-arm from _rx_error() in
     * uwb_dw1000.c, so there is nothing left for this loop to arm. The
     * re-arm that used to sit at the bottom of it is the mistake probe
     * removed in "probe: receive double-buffered, and stop arming the
     * receiver off": an unconditional re-arm around a wait turns the
     * receiver off for exactly the window a frame arrives in.
     */
    uwb_rx_start();

    unsigned long reported_lost_ring = 0;
    unsigned long reported_lost_chip = 0;
    struct timespec next_stats;

    clock_gettime(CLOCK_MONOTONIC, &next_stats);
    next_stats.tv_sec += config.stats;

    while (!stop_requested) {
	int n = poll(pollfd, __arraycount(pollfd), POLL_TIMEOUT_MS);
	if (n < 0) {
	    if (errno == EINTR)
		continue;		/* a signal; stop_requested says */
	    WARN_ERRNO("poll failed");
	    return -1;
	}

	if ((n > 0) && pollfd[0].revents) {
	    uwb_wait_events();

	    /* Until there is nothing left, not once. See DRAIN_MAX. */
	    for (int i = 0; i < DRAIN_MAX; i++) {
		if (!uwb_process_events())
		    break;
	    }
	}

	/* Drain what the callbacks captured. This is where the frame
	 * leaves the program, deliberately outside the callback that
	 * received it: eth_send() is a blocking sendmsg(2) and
	 * pcapng_write() a write(2).
	 *
	 * The loss counts go on each frame's header, and they are read
	 * per frame, after capture_get(): the ring's overrun is accounted
	 * by the read itself, so a count taken before the loop would tell
	 * a receiver about every gap except the one it is looking at.
	 */
	const struct capture_stats *cs = capture_stats();

	struct capture_frame frame;
	while (capture_get(&frame)) {
	    _forward(&frame, cs->overrun, uwb_rx_errors()->overrun);

	    /* Compared unsigned, with config.count known positive by the
	     * test before it: casting the count of frames forwarded down
	     * to a signed long would go negative on a 32-bit host after
	     * two billion frames, which is a long capture but not an
	     * impossible one. */
	    if ((config.count > 0) &&
		(capture_stats()->consumed >= (unsigned long)config.count)) {
		stop_requested = 1;
		break;
	    }
	}

	/* Frames lost, reported when a count moves rather than per frame,
	 * so a burst of losses does not itself cost time in this loop.
	 * Two counts, because they mean different things: the ring
	 * overran because forwarding fell behind the radio, the chip
	 * overran because the read-out fell behind it.
	 */
	if (cs->overrun != reported_lost_ring) {
	    WARN("lost %lu frame(s): forwarding fell behind the radio",
		 cs->overrun - reported_lost_ring);
	    reported_lost_ring = cs->overrun;
	}
	unsigned long chip = uwb_rx_errors()->overrun;
	if (chip != reported_lost_chip) {
	    WARN("lost %lu frame(s): the chip overran (both buffers held)",
		 chip - reported_lost_chip);
	    reported_lost_chip = chip;
	}

	if (config.stats > 0) {
	    struct timespec now;
	    clock_gettime(CLOCK_MONOTONIC, &now);
	    if ((now.tv_sec > next_stats.tv_sec) ||
		((now.tv_sec == next_stats.tv_sec) &&
		 (now.tv_nsec >= next_stats.tv_nsec))) {
		_report_stats("stats");
		next_stats = now;
		next_stats.tv_sec += config.stats;
	    }
	}
    }

    return 0;
}


int main(int argc, const char* argv[]) {
    uint8_t hwaddr[ETH_HWADDR_SIZE];

    /* Lookup for default ethernet interface
     */
    if (eth_get_first_interface(config.ifname_default) < 0) {
	DIE("No valid ethernet interface available");
    }

    /* Parse command line
     */
    if (cmdline_parse(&config, argc, argv) < 0) {
	DIE("Failed to parse command line");
    }

    /* Initialize bitters library
     */
    if (bitters_init() < 0) {
	DIE_ERRNO("bitters library initialisation failed");
    }

    /* Try to tune program for reduced latency
     */
    if (bitters_reduced_latency() < 0) {
	WARN_ERRNO("unable to tune program for reduced latency");
    }

    /* Dissectors, before anything touches the chip.
     *
     * Early for three reasons. --list-dissectors can then print and exit
     * without a radio. A dlopen() here is after bitters_reduced_latency()
     * (so mlockall(MCL_FUTURE) covers the new mappings) and well before
     * the receiver is armed, so no file is opened and nothing is mapped
     * on a SCHED_FIFO thread while frames are arriving. And a bad
     * --dissector is reported before a capture has been started and
     * abandoned.
     */
    if (!config.no_dissect) {
#ifdef SNIFFER_DISSECT_INIT
	SNIFFER_DISSECT_INIT();
#endif
	for (int i = 0; i < config.dissectors; i++) {
	    if (dissect_load(config.dissector[i]) < 0) {
		DIE("failed to load dissector: %s", config.dissector[i]);
	    }
	}
    }

    if (config.list_dissectors) {
	unsigned n = dissect_count();

	if (n == 0) {
	    INFO("no dissectors");
	} else {
	    for (unsigned i = 0; i < n; i++) {
		INFO("%-24s %s", dissect_name(i), dissect_version(i));
	    }
	}
	return EXIT_OK;
    }

    if (config.dissect_filter && (dissect_count() == 0)) {
	WARN("--dissect-filter with no dissector loaded: nothing will be"
	     " filtered");
    }

    /* Initialize network interface
     *
     * Only when there is somewhere to forward to. With -w and no
     * destination address the capture goes to the pcapng file and the
     * program opens no socket at all, which is what makes
     * `uwb-sniffer -w - | wireshark -k -i -` a one-host affair.
     */
    if (config.dst_valid) {
	if (eth_init(config.ifname, config.proto, hwaddr) < 0) {
	    DIE_ERRNO("failed to initialize ethernet interface");
	}
    }

    /* Initialize UWB
     */
    /* Metres on the command line, ticks in the chip, and the conversion
     * used to be computed and thrown away. cmdline_parse() validates with
     * a NULL out parameter, so what landed here was the raw metres: an
     * explicit --tx_delay 154.6 set 154 ticks, about 0.7 m, instead of
     * 32951. The defaults (UINT16_MAX) mean "leave the driver's own
     * alone" and must not be converted, which is what the guard is for.
     */
    if (config.tx_delay != UINT16_MAX)
	dw1000_validate_antenna_delay(config.tx_delay,
				      &uwb_config.antenna.tx_delay, NULL);
    if (config.rx_delay != UINT16_MAX)
	dw1000_validate_antenna_delay(config.rx_delay,
				      &uwb_config.antenna.rx_delay, NULL);

    uwb_config.dblbuff = config.no_dblbuff ? false : true;

    if (uwb_init(&uwb_config) < 0) {
	DIE("failed to initialise dw1000 driver");
    }

    /* Point the ring at the chip before the receiver is armed.
     *
     * The metadata read-out is part of this choice rather than a runtime
     * test per frame: with --no-metadata the ops carry no read_meta and
     * the callback never looks at RX_TIME, RX_FQUAL or the clock
     * tracking. That is about ten register reads a frame bought back
     * (one for the RMARKER, five behind the power estimate, two for the
     * clock tracking and two for the quality figures), for a capture
     * that then cannot say when anything arrived or how strongly.
     */
    capture_init(uwb_capture_ops(config.no_metadata ? false : true));

    /* Already checked, with the message, in cmdline_parse(); this is the
     * pass that writes the encoded values into the radio. The combination
     * they make is not checked here and must not be: that is
     * dw1000_configure()'s job, by way of uwb_config_dw1000_radio() below.
     */
    dw1000_validate_channel(config.channel,  &dw1000_radio.channel,  NULL);
    dw1000_validate_bitrate(config.bitrate,  &dw1000_radio.bitrate,  NULL);
    dw1000_validate_prf    (config.prf,      &dw1000_radio.prf,      NULL);
    dw1000_validate_pcode  (config.tx_pcode, &dw1000_radio.tx_pcode, NULL);
    dw1000_validate_pcode  (config.rx_pcode, &dw1000_radio.rx_pcode, NULL);
    dw1000_validate_plen   (config.tx_plen,  &dw1000_radio.tx_plen,  NULL);
    dw1000_validate_pac    (config.rx_pac,   &dw1000_radio.rx_pac,   NULL);

    if (uwb_config_dw1000_radio(&dw1000_radio) < 0) {
	DIE("selected UWB configuration is invalid");
    }

    /* Frame filtering off, said out loud.
     *
     * It was off before only because that is how the chip comes up after
     * a hard reset, and it is the one setting that, if it were ever on,
     * would make this program silently deaf to every frame not addressed
     * to it: a sniffer reporting nothing and no error, which is the worst
     * failure a capture tool has. Written after dw1000_configure(), which
     * writes SYS_CFG itself.
     *
     * There is deliberately no switch for the other way round. Hardware
     * filtering matches against the PAN id and the addresses in
     * PANADR/EUI (UM §5.2.1), and this program never writes them, so
     * turning it on would reject everything: a flag whose only effect is
     * to make the sniffer deaf is not an option worth offering.
     */
    uwb_rx_set_frame_filtering(0);

    /* Open the capture file before the receiver is armed, so that a path
     * that cannot be written is found out now rather than after the
     * first frame has been captured and dropped.
     */
    if (config.pcapng != NULL) {
	if (pcapng_open(config.pcapng,
			(uint32_t)DW1000_FRAME_MAXSIZE) < 0) {
	    DIE_ERRNO("failed to open %s for writing", config.pcapng);
	}
    }

    /* Last, after the radio is configured and the capture file is open,
     * so that a plugin's open() runs with everything else already in
     * place and is the only thing left that can refuse.
     */
    if (dissect_open_all() < 0) {
	DIE("a dissector refused to open");
    }

    /* Stop on a signal rather than dying on one, so that the reception
     * account survives the Ctrl-C every run ends with. Deliberately
     * without SA_RESTART: poll(2) returning EINTR is how the loop gets
     * to look at the flag promptly.
     */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = _on_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /*
     */
    if (config.dst_valid) {
	INFO("To capture packets on the remote side you can use:");
	INFO("# tcpdump ether proto 0x%04x"
	     " and ether src %02x:%02x:%02x:%02x:%02x:%02x",
	     config.proto,
	     hwaddr[0], hwaddr[1], hwaddr[2], hwaddr[3], hwaddr[4], hwaddr[5]);
    }

    /* Loop for incoming packets
     */
    int rc = loop();

    /* What the run saw, which is the thing that used to be thrown away
     * with the process. On stderr, so that `-w -` piping pcapng into
     * wireshark is unaffected by it.
     */
    _report_stats("total");

    if (config.pcapng != NULL) {
	if (pcapng_close() < 0)
	    WARN_ERRNO("failed closing %s", config.pcapng);
    }

    dissect_close_all();

    if (rc < 0)
	DIE("failed to run");

    return 0;
}




/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
