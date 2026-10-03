/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The responder's two promises about a link that is not working, driven
 * through port/emulation so that neither needs a chip:
 *
 *  - IT ENDS. A twr_resp run whose POLL never arrives returns, having
 *    emitted one no-poll record per attempt and a STATS line. The wait
 *    for a POLL used to be handed UINT64_MAX, on the stated grounds that
 *    the bench waits for a READY line before starting the initiator,
 *    which it does not and cannot do. What that premise bought was a
 *    board that sat in the wait until its console session was cut,
 *    printing nothing at all: the observation that started this was a
 *    log holding a command echo, READY, and no further line. Silence is
 *    the one report an instrument must never produce, because it is
 *    indistinguishable from a crash.
 *
 *  - IT SAYS WHAT IT HEARD. "The receiver heard nothing" and "the
 *    receiver heard frames and rejected every one of them" are different
 *    defects with different fixes, and the STATS line now separates
 *    them: `heard=` against the seven `drop_` counts.
 *
 * And the roles a node runs alone (<dw1000/probe/solo.h>), against the
 * same medium: that tx sends what it was asked and is told of every
 * completion; that rx counts what arrives and, separately, what the chip
 * rejected; that rx under the settle rule ends on the rule's verdict; and
 * that temperature takes one reading when given no duration and a
 * reading per interval when given one. The medium's die never moves, so
 * the settle step settles on the second window it can compare, which is
 * the rule's own minimum.
 *
 * Both budgets are overridden to milliseconds below. A gate cannot prove
 * that a wait ends by waiting out a thirty-second wait, and the defaults
 * are a property of the bench's harness rather than of this code; see
 * probe/src/exchange.c, which is why they are overridable at all.
 *
 * WHAT THIS DOES NOT COVER. `drop_seq` needs a frame of the right type
 * addressed to this node carrying the wrong exchange number, and the
 * only wait that asks for a sequence is the one for FINAL, reached only
 * after a RESPONSE has been sent. That turn-on happens on the chip
 * (WAIT4RESP) and issues no RX_CONFIG for a medium to answer, so a stub
 * has no hook to deliver into and the frame would have to arrive through
 * a re-arm the timeout drives, inside the same 20 ms the wait is bounded
 * by. It is left to the bench. `drop_unwatched` and `drop_overrun` are
 * likewise not forced: both are races by construction, and a test that
 * produced one on demand would be pinning its own scheduler.
 *
 * The medium is a thread and the two rules it must keep are smoke.c's,
 * for the same reasons given at the top of that file: the line callback
 * only signals, and every request is answered.
 *
 * The whole run is under alarm(20).
 */

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "dw1000/dw1000.h"
#include "dw1000/emulation.h"
#include "dw1000/osal.h"
#include "dw1000/probe/exchange.h"
#include "dw1000/probe/port.h"
#include "dw1000/probe/record.h"
#include "dw1000/probe/role.h"
#include "dw1000/probe/settle.h"
#include "dw1000/probe/solo.h"
#include "rsvc.h"

/*----------------------------------------------------------------------*/
/* The wire protocol (port/emulation/README.md)                                */
/*----------------------------------------------------------------------*/

/* The same copy tests/emulation/smoke.c makes, for the same reason:
 * <rsvc.h> exports the client API, while the framing a server has to
 * speak is described in port/emulation/README.md and written in rsvc.c, which
 * is not a header. A medium server is entitled to nothing but the
 * document, so a test standing in for one takes nothing else either.
 */
struct rsvc_inhdr {                     /* node -> server, 11 bytes */
    uint16_t type;
    uint64_t id;
    uint8_t  flags;
} __attribute__((packed));

struct rsvc_outhdr {                    /* server -> node, 12 bytes */
    uint16_t type;
    uint64_t id;
    int8_t   status;
    uint8_t  flags;
} __attribute__((packed));

#define RSVC_HDR_FLG_INTERRUPT          0x02


/*----------------------------------------------------------------------*/
/* Frame check sequence                                                 */
/*----------------------------------------------------------------------*/

/* smoke.c's copy of the routine the model uses (e_crc16_ccitt(), static
 * in osal.c), needed here for the opposite reason: a frame this test
 * injects was never transmitted by anything, so nothing has appended its
 * FCS. Delivered without one the model reports a receive error and the
 * frame never reaches a callback, which is how the first version of
 * this test managed to assert heard=0 against four frames it had
 * carefully queued.
 */
static uint16_t
crc16_ccitt(const uint8_t *src, size_t len)
{
    uint16_t seed = 0x0000;

    for (; len > 0; len--) {
	uint8_t e, f;

	e = seed ^ *src++;
	f = e ^ (e << 4);
	seed = (seed >> 8) ^ ((uint16_t)f << 8) ^
	       ((uint16_t)f << 3) ^ ((uint16_t)f >> 4);
    }
    return seed;
}

/*----------------------------------------------------------------------*/
/* Reporting                                                            */
/*----------------------------------------------------------------------*/

static int  failures;
static char reason[512];

#define REASON(...) (snprintf(reason, sizeof(reason), __VA_ARGS__), reason)

static void
step(const char *what, const char *why)
{
    if (why == NULL) {
	printf("ok: %s\n", what);
    } else {
	printf("FAIL: %s (%s)\n", what, why);
	failures++;
    }
}

/*----------------------------------------------------------------------*/
/* The stub medium                                                      */
/*----------------------------------------------------------------------*/

/* Same fake clock as smoke.c: nothing here measures a duration, the
 * stamps only have to move and stay ordered. */
#define STUB_CLOCK_START  0x0000010000000ull
#define STUB_CLOCK_STEP   63897600ull
#define STUB_CLOCK_MASK   ((1ull << DW1000_TIME_CLOCK_BITS) - 1)

/* Between one queued frame and the next. frame_wait() samples the
 * capture slot every 500 us and the slot is one deep, so frames arriving
 * faster than that would be counted as overruns rather than as the
 * reasons this test is asserting. Twenty times the poll interval is
 * margin enough to make the attribution deterministic without making the
 * test slow. */
#define STUB_DELIVER_GAP_US 10000

#define QUEUE_MAX  8
#define FRAME_MAX  40

struct queued {
    uint8_t data[FRAME_MAX];
    size_t  len;                /* without the CRC the model appends */
};

struct stub {
    int      fd;
    char     path[sizeof(((struct sockaddr_un *)0)->sun_path)];

    pthread_mutex_t lock;
    uint64_t clock;

    /* Frames to hand the node, one per RX_CONFIG, oldest first. Refilled
     * by the main thread between steps, while no role is running. */
    struct queued queue[QUEUE_MAX];
    unsigned      queued;
    unsigned      sent;
};

static uint64_t
stub_stamp(struct stub *s)
{
    uint64_t now = s->clock;

    s->clock = (s->clock + STUB_CLOCK_STEP) & STUB_CLOCK_MASK;
    return now;
}

static void
stub_send(struct stub *s, const struct sockaddr_un *peer, socklen_t peerlen,
	  uint16_t type, uint64_t id, int8_t status, uint8_t flags,
	  const void *data, size_t datalen)
{
    uint8_t buf[sizeof(struct rsvc_outhdr) + RSVC_DATA_MAXLEN];
    struct rsvc_outhdr hdr = {
	.type   = type,
	.id     = id,
	.status = status,
	.flags  = flags,
    };

    if (datalen > RSVC_DATA_MAXLEN) {
	fprintf(stderr, "stub: reply payload too big (%zu)\n", datalen);
	return;
    }

    memcpy(buf, &hdr, sizeof(hdr));
    if (datalen > 0)
	memcpy(buf + sizeof(hdr), data, datalen);

    if (sendto(s->fd, buf, sizeof(hdr) + datalen, 0,
	       (const struct sockaddr *)peer, peerlen) < 0)
	fprintf(stderr, "stub: sendto: %s\n", strerror(errno));
}

static void
stub_uwb_io(struct stub *s, const struct sockaddr_un *peer, socklen_t peerlen,
	    uint64_t id, const uint8_t *payload, size_t paylen)
{
    struct dw1000_driver_iopkt in, out;

    memset(&in, 0, sizeof(in));
    memcpy(&in, payload, paylen < sizeof(in) ? paylen : sizeof(in));

    switch (in.type) {
    case DW1000_RSVC_TX: {
	uint64_t stamp;

	pthread_mutex_lock(&s->lock);
	stamp = (DW1000_CLOCK_ROUNDUP(stub_stamp(s)) + in.tx.antenna_delay)
		& STUB_CLOCK_MASK;
	pthread_mutex_unlock(&s->lock);

	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, 0, 0, NULL, 0);

	/* Completed and dropped on the floor: this medium carries nothing
	 * anywhere, and a responder with no peer is the whole subject. */
	memset(&out, 0, sizeof(out));
	out.drvid             = in.drvid;
	out.type              = DW1000_RSVC_TX_DONE;
	out.tx_done.timestamp = stamp;
	stub_send(s, peer, peerlen, RSVC_UWB_IO, 0, 0,
		  RSVC_HDR_FLG_INTERRUPT, &out,
		  offsetof(struct dw1000_driver_iopkt, tx_done.timestamp) +
		  sizeof(out.tx_done.timestamp));
	break;
    }

    case DW1000_RSVC_RX_CONFIG: {
	struct queued q;
	bool     deliver;
	uint64_t stamp = 0;

	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, 0, 0, NULL, 0);

	pthread_mutex_lock(&s->lock);
	deliver = s->sent < s->queued;
	if (deliver) {
	    q     = s->queue[s->sent++];
	    stamp = stub_stamp(s);
	}
	pthread_mutex_unlock(&s->lock);

	if (!deliver)
	    break;

	/* Paced, for the reason STUB_DELIVER_GAP_US gives. This runs on
	 * the reader thread, which is allowed to sleep: what it may not
	 * do is call the driver. */
	dw1000_probe_port_sleep(STUB_DELIVER_GAP_US);

	memset(&out, 0, sizeof(out));
	out.drvid        = 0;
	out.type         = DW1000_RSVC_RX;
	out.rx.flags     = 0;
	out.rx.timestamp = stamp;
	memcpy(out.rx.frame, q.data, q.len);
	stub_send(s, peer, peerlen, RSVC_UWB_IO, 0, 0,
		  RSVC_HDR_FLG_INTERRUPT, &out,
		  offsetof(struct dw1000_driver_iopkt, rx.frame) + q.len);
	break;
    }

    default:
	fprintf(stderr, "stub: unknown UWB packet type 0x%02x\n", in.type);
	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, -1, 0, NULL, 0);
	break;
    }
}

static void *
stub_medium(void *args)
{
    struct stub *s = args;
    bool running = true;

    while (running) {
	uint8_t buf[sizeof(struct rsvc_inhdr) + RSVC_DATA_MAXLEN];
	struct sockaddr_un peer;
	socklen_t peerlen = sizeof(peer);
	struct rsvc_inhdr hdr;
	ssize_t  n;

	n = recvfrom(s->fd, buf, sizeof(buf), 0,
		     (struct sockaddr *)&peer, &peerlen);
	if (n < (ssize_t)sizeof(hdr))
	    break;

	memcpy(&hdr, buf, sizeof(hdr));

	switch (hdr.type) {
	case RSVC_UWB_IO:
	    stub_uwb_io(s, &peer, peerlen, hdr.id,
			buf + sizeof(hdr), (size_t)n - sizeof(hdr));
	    break;

	case RSVC_CLOSE:
	    stub_send(s, &peer, peerlen, hdr.type, hdr.id, 0, 0, NULL, 0);
	    running = false;
	    break;

	default:
	    /* Every request is answered, including one whose type this
	     * stub does not model: silence is a five-second stall. */
	    stub_send(s, &peer, peerlen, hdr.type, hdr.id, 0, 0, NULL, 0);
	    break;
	}
    }
    return NULL;
}

/*----------------------------------------------------------------------*/
/* The node                                                             */
/*----------------------------------------------------------------------*/

static dw1000_t dw;

static struct {
    pthread_mutex_t lock;
    pthread_cond_t  wake;
    bool            irq;
    bool            stop;
} evt = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .wake = PTHREAD_COND_INITIALIZER,
};

/* On the rsvc reader thread, inside the model's own mutex: it may only
 * say that something happened. */
static void
line_cb(int line, void *args)
{
    (void)args;

    if (line != DW1000_IOLINE_IRQ)
	return;

    pthread_mutex_lock(&evt.lock);
    evt.irq = true;
    pthread_cond_signal(&evt.wake);
    pthread_mutex_unlock(&evt.lock);
}

/* What every probe application's rx_ok does: capture first, because a
 * restart is free to disturb the timestamp and the power estimate, then
 * re-arm. frame_wait() never re-arms; this is the only thing that does. */
static void
cb_rx_ok(dw1000_t *d, uint32_t status, size_t length, bool ranging)
{
    (void)status; (void)ranging;

    dw1000_probe_rx_capture(d, length);
    dw1000_rx_start(d, DW1000_RX_IMMEDIATE);
}

static void
cb_rx_error(dw1000_t *d, uint32_t status)
{
    (void)status;
    dw1000_probe_rx_error_capture();
    dw1000_rx_start(d, DW1000_RX_IMMEDIATE);
}

static void
cb_rx_timeout(dw1000_t *d, uint32_t status)
{
    (void)status;
    dw1000_rx_start(d, DW1000_RX_IMMEDIATE);
}

static void
cb_tx_done(dw1000_t *d, uint32_t status)
{
    (void)d; (void)status;
    dw1000_probe_tx_capture();
}

/* The roles block on the calling thread, so events are pumped from
 * another one. The bus lock is held across dw1000_process_events()
 * because <dw1000/probe/port.h> requires the callbacks to run under it:
 * that is what lets the capture and its counter be plain variables. */
static void *
event_thread(void *args)
{
    (void)args;

    for (;;) {
	struct timespec deadline;
	bool fire;

	clock_gettime(CLOCK_REALTIME, &deadline);
	deadline.tv_nsec += 20 * 1000000L;
	if (deadline.tv_nsec >= 1000000000L) {
	    deadline.tv_sec  += 1;
	    deadline.tv_nsec -= 1000000000L;
	}

	pthread_mutex_lock(&evt.lock);
	while (!evt.irq && !evt.stop) {
	    if (pthread_cond_timedwait(&evt.wake, &evt.lock, &deadline) != 0)
		break;
	}
	if (evt.stop) {
	    pthread_mutex_unlock(&evt.lock);
	    return NULL;
	}
	fire = evt.irq;
	evt.irq = false;
	pthread_mutex_unlock(&evt.lock);

	if (fire) {
	    dw1000_probe_port_bus_lock();
	    dw1000_process_events(&dw);
	    dw1000_probe_port_bus_unlock();
	}
    }
}

/*----------------------------------------------------------------------*/
/* Capturing what the role emitted                                      */
/*----------------------------------------------------------------------*/

#define LINES_MAX 16

static char     lines[LINES_MAX][DW1000_PROBE_RECORD_MAX];
static unsigned nlines;

static void
collect(const char *line)
{
    if (nlines < LINES_MAX)
	snprintf(lines[nlines], sizeof(lines[0]), "%s", line);
    nlines++;
}

/* The value of `key=` in a STATS line, or -1 if it is not there. The
 * search includes the leading space so that `drop_seq=` cannot match
 * inside another key. */
static long
field(const char *line, const char *key)
{
    char pattern[64];
    const char *at;

    snprintf(pattern, sizeof(pattern), " %s=", key);
    if ((at = strstr(line, pattern)) == NULL)
	return -1;
    return strtol(at + strlen(pattern), NULL, 10);
}

/*----------------------------------------------------------------------*/
/* Frames to be rejected                                                */
/*----------------------------------------------------------------------*/

#define OWN_ADDR   0xc939
#define PEER_ADDR  0x000b

static void
put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
}

/* The probe's 12-byte header, built here rather than shared with
 * exchange.c on purpose: a test that built its frames with the code
 * under test would agree with it however wrong both were.
 *
 * @p len is the frame as the probe counts it, without the FCS; the two
 * bytes appended here are what the chip appends and what the length
 * reported to rx_ok includes.
 *
 * Under the stub's lock, and not because the queue is refilled while a
 * role runs: it is not. The receiver is still armed from the step
 * before, so the stub thread is still taking RX_CONFIG requests and
 * still reading `queued` to decide it has nothing to deliver. Without
 * the lock that read races this write, which is what ThreadSanitizer
 * said the first time it was asked. */
static void
enqueue(struct stub *s, const char *mark, char type, uint8_t seq,
	uint16_t dst, uint16_t src, size_t len)
{
    struct queued *q;
    uint16_t fcs;

    pthread_mutex_lock(&s->lock);

    if (s->queued >= QUEUE_MAX || len + DW1000_CRC_LENGTH > FRAME_MAX) {
	pthread_mutex_unlock(&s->lock);
	return;
    }

    q = &s->queue[s->queued++];
    memset(q, 0, sizeof(*q));
    memcpy(q->data, mark, 3);
    q->data[3] = (uint8_t)type;
    q->data[4] = seq;
    put_u16le(&q->data[5], dst);
    put_u16le(&q->data[7], src);

    fcs = crc16_ccitt(q->data, len);
    put_u16le(&q->data[len], fcs);
    q->len = len + DW1000_CRC_LENGTH;

    pthread_mutex_unlock(&s->lock);
}

/* Spoil the FCS of the frame queued last, so that the model reports it
 * as a receive error rather than delivering it. */
static void
corrupt_last(struct stub *s)
{
    pthread_mutex_lock(&s->lock);
    if (s->queued > 0) {
	struct queued *q = &s->queue[s->queued - 1];

	q->data[q->len - 1] ^= 0xff;
    }
    pthread_mutex_unlock(&s->lock);
}

/*----------------------------------------------------------------------*/
/* Steps                                                                */
/*----------------------------------------------------------------------*/

/* A run nobody answers ends, and says so once per attempt. */
static const char *
step_silence(struct stub *s)
{
    struct dw1000_probe_twr_resp_result r;
    unsigned i;

    pthread_mutex_lock(&s->lock);
    s->queued = s->sent = 0;
    pthread_mutex_unlock(&s->lock);

    nlines = 0;
    r = dw1000_probe_twr_resp_run(&dw, 3, false, OWN_ADDR, PEER_ADDR,
				  "T1", collect);

    if (r.attempted != 3 || r.resolved != 0)
	return REASON("attempted=%u resolved=%u, wanted 3 and 0",
		      r.attempted, r.resolved);
    if (nlines != 4)
	return REASON("%u lines, wanted 3 records and a STATS", nlines);

    for (i = 0; i < 3; i++) {
	if (strstr(lines[i], "status=no-poll") == NULL)
	    return REASON("record %u is not no-poll: %s", i, lines[i]);
	/* record.h promises the near end's die reading is in every
	 * record, a no-poll record included. */
	if (strstr(lines[i], "resp_temp=-") != NULL)
	    return REASON("record %u has no temperature: %s", i, lines[i]);
    }
    if (strncmp(lines[3], "STATS ", 6) != 0)
	return REASON("last line is not STATS: %s", lines[3]);
    if (field(lines[3], "heard") != 0)
	return REASON("heard=%ld with nothing sent: %s",
		      field(lines[3], "heard"), lines[3]);
    if (field(lines[3], "of") != 3 || field(lines[3], "completed") != 0)
	return REASON("STATS counts wrong: %s", lines[3]);

    return NULL;
}

/* ... and a run that hears only frames it cannot use says which kind. */
static const char *
step_account(struct stub *s)
{
    struct dw1000_probe_twr_resp_result r;
    static const struct {
	const char *key;
	long        want;
    } expect[] = {
	{ "heard",          4 },
	{ "drop_foreign",   1 },
	{ "drop_short",     1 },
	{ "drop_type",      1 },
	{ "drop_dst",       1 },
	{ "drop_overrun",   0 },
    };
    unsigned i;

    pthread_mutex_lock(&s->lock);
    s->queued = s->sent = 0;
    pthread_mutex_unlock(&s->lock);

    /* One per reason the classifier gives, in its own order. The first
     * carries somebody else's mark, the second ours but is eight bytes
     * where a header is twelve, the third is a RESPONSE where a POLL is
     * awaited, the fourth a POLL for another node. */
    enqueue(s, "xyz", 'P', 0, OWN_ADDR,  PEER_ADDR, 12);
    enqueue(s, "dwp", 'P', 0, OWN_ADDR,  PEER_ADDR,  8);
    enqueue(s, "dwp", 'R', 0, OWN_ADDR,  PEER_ADDR, 12);
    enqueue(s, "dwp", 'P', 0, 0x0042,    PEER_ADDR, 12);

    nlines = 0;
    r = dw1000_probe_twr_resp_run(&dw, 1, false, OWN_ADDR, PEER_ADDR,
				  "T1", collect);

    if (r.attempted != 1 || r.resolved != 0)
	return REASON("attempted=%u resolved=%u, wanted 1 and 0",
		      r.attempted, r.resolved);
    if (nlines != 2 || strncmp(lines[1], "STATS ", 6) != 0)
	return REASON("%u lines, wanted a record and a STATS", nlines);
    if (strstr(lines[0], "status=no-poll") == NULL)
	return REASON("the record is not no-poll: %s", lines[0]);

    for (i = 0; i < sizeof(expect) / sizeof(expect[0]); i++) {
	long got = field(lines[1], expect[i].key);

	if (got != expect[i].want)
	    return REASON("%s=%ld, wanted %ld: %s",
			  expect[i].key, got, expect[i].want, lines[1]);
    }

    return NULL;
}

/*----------------------------------------------------------------------*/
/* The roles a node runs alone                                          */
/*----------------------------------------------------------------------*/

/* The TEMP line's elapsed field, in tenths, or -1 if @p line is not
 * a TEMP line. */
static long
temp_elapsed_ds(const char *line)
{
    unsigned whole, tenth;

    if (sscanf(line, "TEMP %u.%u ", &whole, &tenth) != 2)
	return -1;
    return (long)whole * 10 + tenth;
}

static const char *
step_tx(struct stub *s)
{
    struct dw1000_probe_tx_result r;

    (void)s;
    nlines = 0;
    r = dw1000_probe_tx_run(&dw, 5, 2000, DW1000_PROBE_SAMPLE_INTERVAL_MS,
			    "T1", collect);

    if (r.attempted != 5 || r.started != 5 || r.completed != 5)
	return REASON("attempted=%u started=%u completed=%u, wanted 5 each",
		      (unsigned)r.attempted, (unsigned)r.started,
		      (unsigned)r.completed);
    if (nlines < 2 || nlines > LINES_MAX)
	return REASON("%u lines, wanted an opening and a closing TEMP",
		      nlines);
    if (temp_elapsed_ds(lines[0]) != 0)
	return REASON("first line is not TEMP at 0.0: %s", lines[0]);
    if (temp_elapsed_ds(lines[nlines - 1]) < 0)
	return REASON("last line is not TEMP: %s", lines[nlines - 1]);
    return NULL;
}

static const char *
step_rx(struct stub *s)
{
    struct dw1000_probe_rx_result r;
    unsigned i;

    pthread_mutex_lock(&s->lock);
    s->queued = s->sent = 0;
    pthread_mutex_unlock(&s->lock);

    /* Whoever's frames: the rx role counts them all. The middle one is
     * spoilt, and is the chip's to reject. */
    enqueue(s, "xyz", 'P', 0, OWN_ADDR, PEER_ADDR, 12);
    enqueue(s, "dwp", 'P', 0, OWN_ADDR, PEER_ADDR, 12);
    corrupt_last(s);
    enqueue(s, "dwp", 'R', 0, 0x0042,   PEER_ADDR, 12);

    nlines = 0;
    r = dw1000_probe_rx_run(&dw, 1, DW1000_PROBE_SAMPLE_INTERVAL_MS,
			    "T1", collect);

    if (r.failed)
	return "the receiver did not start";
    if (r.received != 2 || r.rejected != 1)
	return REASON("received=%u rejected=%u, wanted 2 and 1",
		      (unsigned)r.received, (unsigned)r.rejected);
    if (r.elapsed_ds < 10)
	return REASON("listened %u ds of 10", (unsigned)r.elapsed_ds);
    if (r.settle != DW1000_PROBE_SETTLE_SAMPLING)
	return "a timed run reports a settle verdict";
    if (nlines < 2 || nlines > dw1000_probe_sampled_lines(1, 1000))
	return REASON("%u lines for one second at 1 Hz", nlines);
    for (i = 0; i < nlines; i++)
	if (temp_elapsed_ds(lines[i]) < 0)
	    return REASON("line %u is not TEMP: %s", i, lines[i]);
    return NULL;
}

static const char *
step_rx_settle(struct stub *s)
{
    static const struct dw1000_probe_settle_params p = {
	.interval_ms = 100, .window_s = 1, .windows = 2,
	.threshold_cdeg = 20, .give_up_s = 4,
    };
    struct dw1000_probe_rx_result r;

    pthread_mutex_lock(&s->lock);
    s->queued = s->sent = 0;
    pthread_mutex_unlock(&s->lock);

    nlines = 0;
    r = dw1000_probe_rx_settle_run(&dw, &p, "T1", collect);

    if (r.failed)
	return "the receiver did not start";
    if (r.settle != DW1000_PROBE_SETTLE_SETTLED)
	return REASON("a still die ended %s",
		      dw1000_probe_settle_state_name(r.settle));
    if (nlines != 4)
	return REASON("%u lines, wanted TEMP, two SETTLE, TEMP", nlines);
    if (temp_elapsed_ds(lines[0]) != 0 || temp_elapsed_ds(lines[3]) < 0)
	return REASON("not bracketed by TEMP: %s / %s", lines[0], lines[3]);
    if (strstr(lines[1], "SETTLE window=1 ") != lines[1] ||
	strstr(lines[1], " spread=- ") == NULL ||
	strstr(lines[1], " state=unsettled ") == NULL)
	return REASON("first window: %s", lines[1]);
    if (strstr(lines[2], "SETTLE window=2 ") != lines[2] ||
	strstr(lines[2], " spread=0 ") == NULL ||
	strstr(lines[2], " state=settled ") == NULL ||
	strstr(lines[2], " role=rx ") == NULL)
	return REASON("second window: %s", lines[2]);
    return NULL;
}

static const char *
step_temperature(struct stub *s)
{
    struct dw1000_probe_temperature_result r;

    (void)s;
    nlines = 0;
    r = dw1000_probe_temperature_run(&dw, 0, DW1000_PROBE_SAMPLE_INTERVAL_MS,
				     "T1", collect);
    if (r.samples != 1 || nlines != 1 || temp_elapsed_ds(lines[0]) != 0)
	return REASON("no duration: %u samples, %u lines, first %s",
		      (unsigned)r.samples, nlines, lines[0]);

    nlines = 0;
    r = dw1000_probe_temperature_run(&dw, 1, 250, "T1", collect);
    if (r.samples != nlines)
	return REASON("%u samples reported, %u lines emitted",
		      (unsigned)r.samples, nlines);
    if (nlines < 3 || nlines > dw1000_probe_sampled_lines(1, 250))
	return REASON("%u lines for one second at 4 Hz", nlines);
    if (temp_elapsed_ds(lines[nlines - 1]) < 10)
	return REASON("ended before a second: %s", lines[nlines - 1]);
    return NULL;
}

/*----------------------------------------------------------------------*/
/* Bring-up                                                             */
/*----------------------------------------------------------------------*/

static void
on_alarm(int sig)
{
    static const char msg[] = "FAIL: timeout\n";

    (void)sig;
    if (write(STDOUT_FILENO, msg, sizeof(msg) - 1) < 0) { /* nothing to do */ }
    _exit(2);
}

static bool
make_socket_dir(char *dir, size_t dirlen, char *sock, size_t socklen)
{
    const char *tmp = getenv("TMPDIR");

    if (tmp == NULL || tmp[0] == '\0')
	tmp = "/tmp";

    if ((size_t)snprintf(dir, dirlen, "%s/dw1000-probe.XXXXXX", tmp) >= dirlen)
	return false;
    if (mkdtemp(dir) == NULL)
	return false;
    if ((size_t)snprintf(sock, socklen, "%s/medium", dir) >= socklen)
	return false;

    return true;
}

static bool
stub_start(struct stub *s, const char *path, pthread_t *thread)
{
    struct sockaddr_un addr;

    memset(s, 0, sizeof(*s));
    s->fd    = -1;
    s->clock = STUB_CLOCK_START;
    pthread_mutex_init(&s->lock, NULL);

    if ((size_t)snprintf(s->path, sizeof(s->path), "%s", path) >=
	sizeof(s->path))
	return false;

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, s->path, strlen(s->path));

    if ((s->fd = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0)
	return false;
    if (bind(s->fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	return false;

    return pthread_create(thread, NULL, stub_medium, s) == 0;
}

int
main(void)
{
    char dir[128];
    char sockpath[sizeof(((struct sockaddr_un *)0)->sun_path)];
    struct stub  stub;
    pthread_t    stub_thread, events;
    rsvc_t      *rsvc;
    struct dw1000_emulation *emulation;
    struct stat  st;

    static struct dw1000_ioline ioline_irq   = { .line = DW1000_IOLINE_IRQ };
    static struct dw1000_ioline ioline_reset = { .line = DW1000_IOLINE_RESET };
    static dw1000_spi_driver_t  spi;

    static const dw1000_config_t config = {
	.spi              = &spi,
	.irq              = &ioline_irq,
	.reset            = &ioline_reset,
	.wakeup           = NULL,
	.leds             = 0,
	.lde_loading      = 1,
	.dblbuff          = 0,
	.rxauto           = 0,   /* the callbacks above re-arm, as a probe
				  * application's do */
	.tx_antenna_delay = 16436,
	.rx_antenna_delay = 16436,
	.cb.tx_done       = cb_tx_done,
	.cb.rx_ok         = cb_rx_ok,
	.cb.rx_error      = cb_rx_error,
	.cb.rx_timeout    = cb_rx_timeout,
    };

    /* The probe's own radio, minus what the model does not need. */
    struct dw1000_radio radio = {
	.channel  = 5,
	.prf      = DW1000_PRF_64MHZ,
	.rx_pac   = DW1000_PAC8,
	.tx_plen  = DW1000_PLEN_128,
	.tx_pcode = 10,
	.rx_pcode = 10,
	.bitrate  = DW1000_BITRATE_6800KBPS,
	.tx_power = DW1000_TX_POWER_AUTO,
	.proprietary.sfd = 0,
    };

    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGALRM, on_alarm);
    alarm(20);

    if (!make_socket_dir(dir, sizeof(dir), sockpath, sizeof(sockpath))) {
	fprintf(stderr, "exchange: cannot make a socket directory\n");
	return 1;
    }
    if (!stub_start(&stub, sockpath, &stub_thread)) {
	fprintf(stderr, "exchange: cannot start the stub medium: %s\n",
		strerror(errno));
	return 1;
    }
    if ((rsvc = rsvc_open(sockpath, (char *)"probe", NULL)) == NULL) {
	fprintf(stderr, "exchange: rsvc_open failed\n");
	return 1;
    }

    emulation = dw1000_emulation_create(rsvc, line_cb, NULL);
    ioline_irq.emulation   = emulation;
    ioline_reset.emulation = emulation;
    spi.emulation          = emulation;

    dw1000_init(&dw, &config);
    dw1000_hardreset(&dw);
    if (dw1000_initialise(&dw) != 0) {
	fprintf(stderr, "exchange: dw1000_initialise failed\n");
	return 1;
    }
    if (dw1000_configure(&dw, &radio) != 0) {
	fprintf(stderr, "exchange: dw1000_configure failed\n");
	return 1;
    }
    if (pthread_create(&events, NULL, event_thread, NULL) != 0) {
	fprintf(stderr, "exchange: cannot start the event thread\n");
	return 1;
    }

    step("a run nobody answers ends", step_silence(&stub));
    step("and says what it heard",    step_account(&stub));
    step("tx sends and is told",      step_tx(&stub));
    step("rx counts, and rejections", step_rx(&stub));
    step("rx ends when settled",      step_rx_settle(&stub));
    step("temperature samples",       step_temperature(&stub));

    pthread_mutex_lock(&evt.lock);
    evt.stop = true;
    pthread_cond_signal(&evt.wake);
    pthread_mutex_unlock(&evt.lock);
    pthread_join(events, NULL);

    dw1000_txrx_off(&dw);
    dw1000_emulation_stop(emulation);
    rsvc_close(rsvc);
    pthread_join(stub_thread, NULL);
    dw1000_emulation_destroy(emulation);
    close(stub.fd);

    unlink(sockpath);
    step("cleanup", (rmdir(dir) == 0 && stat(dir, &st) < 0 && errno == ENOENT)
		    ? NULL
		    : REASON("the socket directory %s is still there", dir));

    return failures == 0 ? 0 : 1;
}

// Local Variables:
// mode: c
// c-basic-offset: 4
// End:
