/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The parts of port/emulation that need a clock: SYS_TIME, the delayed
 * send of UM 3.3, the delayed receive of UM 4.2, and the two receive
 * timeouts the model can raise (RXRFTO and RXPTO).
 *
 * tests/emulation/smoke.c already covers a frame going out and coming
 * back, and its medium runs a fake clock of its own stepping by a
 * millisecond per stamp: fine for checking that a timestamp arrives,
 * useless for checking when. The medium here stamps with
 * dw1000_emulation_clock() instead, the same clock the model's SYS_TIME
 * reads, which is the only way the two halves can be compared at all.
 *
 * It also holds its frames: nothing is delivered unless a step says so,
 * because a timeout is a thing that happens when no frame arrives.
 *
 * Both rules smoke.c states about any medium hold here too. The first
 * of them is worth restating, because this file once claimed it had
 * relaxed and it had not: the line callback still must not call the
 * driver. It no longer runs inside the model's mutex, which was one
 * reason; the other stands, and is the one smoke.c gives: the callback
 * runs on the rsvc reader thread, and a driver call from it waits for a
 * reply that only that thread could have delivered.
 *
 * The whole run is under alarm(60): the steps here deliberately wait for
 * timeouts, so it needs more room than the smoke test's alarm(20).
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
#include "dw1000/dw1000_send.h"
#include "dw1000/emulation.h"
#include "dw1000/osal.h"
#include "rsvc.h"

#if defined(__FreeBSD__)
#include <sys/sysctl.h>
#include <sys/user.h>
#endif


/*----------------------------------------------------------------------*/
/* How many threads this process has                                    */
/*----------------------------------------------------------------------*/

/* Used by one step, to tell a model that was joined from one that was
 * merely abandoned. Where the answer cannot be had, the step says so and
 * checks what it still can.
 */
static int
thread_count(void)
{
#if defined(__FreeBSD__)
    int mib[4] = { CTL_KERN, KERN_PROC,
		   KERN_PROC_PID | KERN_PROC_INC_THREAD, (int)getpid() };
    size_t len = 0;
    if (sysctl(mib, 4, NULL, &len, NULL, 0) != 0)
	return -1;
    return (int)(len / sizeof(struct kinfo_proc));
#elif defined(__linux__)
    FILE *f = fopen("/proc/self/status", "r");
    char  line[256];
    int   n = -1;
    if (f == NULL)
	return -1;
    while (fgets(line, sizeof(line), f))
	if (sscanf(line, "Threads: %d", &n) == 1)
	    break;
    fclose(f);
    return n;
#else
    return -1;
#endif
}


/*----------------------------------------------------------------------*/
/* The wire protocol (port/emulation/README.md)                                */
/*----------------------------------------------------------------------*/

/* Copied rather than included, for the reason smoke.c gives: a medium
 * server is entitled to the document and nothing else.
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

#define RSVC_HDR_FLG_INCLUDE_NICKNAME   0x01
#define RSVC_HDR_FLG_INTERRUPT          0x02

#define STUB_SEED                       0x715E00Du


/*----------------------------------------------------------------------*/
/* The stub medium                                                      */
/*----------------------------------------------------------------------*/

struct stub {
    int      fd;
    char     path[sizeof(((struct sockaddr_un *)0)->sun_path)];

    pthread_mutex_t lock;

    /* Set by a step before it runs, read by the stub thread. */
    bool     deliver;                   /* loop a transmitted frame back */

    /* A medium serving more than one node cannot take any one node's
     * RSVC_CLOSE as its cue to shut down, and the lifecycle step opens
     * and closes thirty connections of its own. So closing is answered
     * always and obeyed only once, when main() says the run is over.
     */
    bool     shutdown;

    /* When set, an RX_CONFIG is received and simply not answered:
     * the behaviour of a server that does not know a service type, or
     * has stopped between the request and the reply.
     */
    bool     swallow;

    /* When set, the completion of a send is held back this long
     * before it is sent, so that a step can act on the chip while
     * the frame is still on the air.
     */
    unsigned tx_done_delay_ms;

    /* Written by the stub thread, read by a step once it has finished. */
    uint8_t  frame[DW1000_FRAME_MAXSIZE];
    size_t   framelen;
    bool     ranging;
    bool     have_frame;
    unsigned tx_count;                  /* TX requests seen */
    unsigned rxcfg_count;               /* RX_CONFIG requests seen */
    uint64_t tx_seen_at;                /* when the TX request arrived */
    uint64_t tx_done_time;              /* what the last TX_DONE carried */
};

static void
stub_send(struct stub *s, const struct sockaddr_un *peer, socklen_t peerlen,
	  uint16_t type, uint64_t id, int8_t status, uint8_t flags,
	  const void *data, size_t datalen)
{
    uint8_t buf[sizeof(struct rsvc_outhdr) + RSVC_DATA_MAXLEN];
    struct rsvc_outhdr hdr = {
	.type = type, .id = id, .status = status, .flags = flags,
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
	size_t   framelen = paylen -
	    offsetof(struct dw1000_driver_iopkt, tx.frame);
	uint64_t now      = dw1000_emulation_clock();
	uint64_t stamp;
	unsigned delay_ms;

	pthread_mutex_lock(&s->lock);
	memcpy(s->frame, in.tx.frame, framelen);
	s->framelen   = framelen;
	s->ranging    = (in.tx.flags & DW1000_RSVC_FLG_RANGING) != 0;
	s->have_frame = true;
	s->tx_count++;
	s->tx_seen_at = now;

	/* The model asserts, for an immediate send, that TX_STAMP less
	 * its own TX_ANTD lands on a 512-tick boundary. So round first,
	 * then add the antenna delay the node supplied, which is the
	 * convention port/emulation/README.md states.
	 */
	stamp = (DW1000_CLOCK_ROUNDUP(now) + in.tx.antenna_delay)
	        & ((1ull << DW1000_TIME_CLOCK_BITS) - 1);
	s->tx_done_time = stamp;
	delay_ms        = s->tx_done_delay_ms;
	pthread_mutex_unlock(&s->lock);

	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, 0, 0, NULL, 0);

	if (delay_ms)
	    usleep(delay_ms * 1000);

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
	bool     deliver;
	size_t   framelen = 0;

	pthread_mutex_lock(&s->lock);
	s->rxcfg_count++;
	if (s->swallow) {
	    pthread_mutex_unlock(&s->lock);
	    return;                     /* no reply, deliberately */
	}
	pthread_mutex_unlock(&s->lock);

	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, 0, 0, NULL, 0);

	memset(&out, 0, sizeof(out));
	pthread_mutex_lock(&s->lock);
	deliver = s->deliver && s->have_frame;
	if (deliver) {
	    framelen         = s->framelen;
	    out.drvid        = 0;
	    out.type         = DW1000_RSVC_RX;
	    out.rx.flags     = s->ranging ? DW1000_RSVC_FLG_RANGING : 0;
	    out.rx.timestamp = dw1000_emulation_clock();
	    memcpy(out.rx.frame, s->frame, framelen);
	}
	pthread_mutex_unlock(&s->lock);

	if (deliver)
	    stub_send(s, peer, peerlen, RSVC_UWB_IO, 0, 0,
		      RSVC_HDR_FLG_INTERRUPT, &out,
		      offsetof(struct dw1000_driver_iopkt, rx.frame) +
		      framelen);
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
	uint8_t buf[sizeof(struct rsvc_inhdr) +
		    RSVC_NICKNAME_MAXLEN + 1 + RSVC_DATA_MAXLEN];
	struct sockaddr_un peer;
	socklen_t peerlen = sizeof(peer);
	struct rsvc_inhdr hdr;
	uint8_t *payload;
	size_t   paylen;
	ssize_t  n;

	n = recvfrom(s->fd, buf, sizeof(buf), 0,
		     (struct sockaddr *)&peer, &peerlen);
	if (n < 0) {
	    if (errno == EINTR)
		continue;
	    fprintf(stderr, "stub: recvfrom: %s\n", strerror(errno));
	    break;
	}
	/* Zero bytes means the socket has been shut down under us, which
	 * is how step_medium_vanishes() releases this thread once it has
	 * taken the socket's path away. Nothing in the protocol sends an
	 * empty datagram, so there is no legitimate reading to confuse it
	 * with, and continuing would spin, since a shut-down socket
	 * returns zero for ever.
	 */
	if (n == 0) {
	    running = false;
	    continue;
	}
	if ((size_t)n < sizeof(hdr)) {
	    fprintf(stderr, "stub: short datagram (%zd bytes)\n", n);
	    continue;
	}

	memcpy(&hdr, buf, sizeof(hdr));
	payload = buf + sizeof(hdr);
	paylen  = (size_t)n - sizeof(hdr);

	if (hdr.flags & RSVC_HDR_FLG_INCLUDE_NICKNAME) {
	    size_t nicklen = strnlen((char *)payload, paylen);
	    if (nicklen < paylen)
		nicklen++;
	    payload += nicklen;
	    paylen  -= nicklen;
	}

	switch (hdr.type) {
	case RSVC_OPEN:
	    stub_send(s, &peer, peerlen, hdr.type, hdr.id, 0, 0, NULL, 0);
	    break;

	case RSVC_SEED_GET: {
	    uint32_t seed = STUB_SEED;
	    stub_send(s, &peer, peerlen, hdr.type, hdr.id, 0, 0,
		      &seed, sizeof(seed));
	    break;
	}

	case RSVC_UWB_IO:
	    if (paylen < DW1000_DRIVER_PKT_HDRLEN) {
		fprintf(stderr, "stub: short UWB packet (%zu bytes)\n", paylen);
		stub_send(s, &peer, peerlen, hdr.type, hdr.id, -1, 0, NULL, 0);
		break;
	    }
	    stub_uwb_io(s, &peer, peerlen, hdr.id, payload, paylen);
	    break;

	case RSVC_CLOSE: {
	    bool last;
	    stub_send(s, &peer, peerlen, hdr.type, hdr.id, 0, 0, NULL, 0);
	    pthread_mutex_lock(&s->lock);
	    last = s->shutdown;
	    pthread_mutex_unlock(&s->lock);
	    if (last)
		running = false;
	    break;
	}

	default:
	    fprintf(stderr, "stub: unknown service type 0x%04x\n", hdr.type);
	    stub_send(s, &peer, peerlen, hdr.type, hdr.id, -1, 0, NULL, 0);
	    break;
	}
    }

    return NULL;
}


/*----------------------------------------------------------------------*/
/* The node                                                             */
/*----------------------------------------------------------------------*/

static struct {
    pthread_mutex_t lock;
    pthread_cond_t  wake;
    bool            irq;

    bool     tx_done;
    bool     rx_ok;
    bool     rx_error;
    bool     rx_timeout;
    uint32_t rx_timeout_status;
    int      rx_start_rc;               /* from a callback, when rearm */
} evt = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .wake = PTHREAD_COND_INITIALIZER,
};

static void cb_tx_done(dw1000_t *dw, uint32_t status)
{ (void)dw; (void)status; evt.tx_done = true; }

/* Set by a step: the rx_ok and rx_error callbacks re-arm the receiver,
 * the way a single buffered host's do, and record what the call said. */
static bool rearm;

static void cb_rx_ok(dw1000_t *dw, uint32_t status, size_t length, bool rng)
{
    (void)status; (void)length; (void)rng;
    evt.rx_ok = true;
    if (rearm)
	evt.rx_start_rc = dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
}

static void cb_rx_error(dw1000_t *dw, uint32_t status)
{
    (void)status;
    evt.rx_error = true;
    if (rearm)
	evt.rx_start_rc = dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
}

static void cb_rx_timeout(dw1000_t *dw, uint32_t status)
{ (void)dw; evt.rx_timeout = true; evt.rx_timeout_status = status; }

/* Called by the model, on the rsvc reader thread or on the model's own
 * deadline thread. Signals, and nothing else.
 */
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

static void
evt_reset(void)
{
    pthread_mutex_lock(&evt.lock);
    evt.irq = false;
    pthread_mutex_unlock(&evt.lock);

    evt.tx_done    = false;
    evt.rx_ok      = false;
    evt.rx_error   = false;
    evt.rx_timeout = false;
    evt.rx_timeout_status = 0;
    evt.rx_start_rc = 0;
}

/* Wait for the interrupt line, and leave whatever raised it unprocessed. */
static bool
wait_line(unsigned timeout_ms)
{
    struct timespec deadline;
    bool got;

    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
	deadline.tv_sec  += 1;
	deadline.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&evt.lock);
    while (!evt.irq) {
	if (pthread_cond_timedwait(&evt.wake, &evt.lock, &deadline) != 0)
	    break;
    }
    got = evt.irq;
    evt.irq = false;
    pthread_mutex_unlock(&evt.lock);

    return got;
}

static bool
wait_irq(dw1000_t *dw, unsigned timeout_ms)
{
    bool got = wait_line(timeout_ms);

    if (got)
	dw1000_process_events(dw);

    return got;
}


/*----------------------------------------------------------------------*/
/* Steps                                                                */
/*----------------------------------------------------------------------*/

#define PAYLOAD_LEN     20
#define FRAME_LEN       (PAYLOAD_LEN + DW1000_CRC_LENGTH)
#define IRQ_TIMEOUT_MS  2000

/* The radio this test configures is a 128-symbol preamble at 64 MHz PRF,
 * so one preamble symbol is 508 * 128 ticks and Ton is 136 of them:
 * 8 843 264 ticks, about 138.4 us. Every lead below is chosen against
 * that.
 */
#define USEC(x)         ((uint64_t)DW1000_USEC_TO_CLOCK(x))
#define MSEC(x)         USEC((x) * 1000)

static int     failures;
static char    reason[256];
static rsvc_t *g_rsvc;          /* the connection step_unanswered_call tunes */

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

static const uint8_t payload[PAYLOAD_LEN] = {
    0x41, 0x88, 0x00, 0xcf, 0xbc, 0x01, 0x00, 0x02, 0x00, 0x2a,
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99,
};

/* TX_TIME.TX_RAWST, which has no public accessor: the driver exposes the
 * RMARKER (TX_STAMP) and nothing else, and it is exactly the difference
 * between the two that a delayed send is being checked on here.
 */
static uint64_t
tx_get_rawst(dw1000_t *dw)
{
    uint64_t raw = 0;
    _dw1000_reg_read(dw, DW1000_REG_TX_TIME, DW1000_OFF_TX_TIME_TX_RAWST,
		     (uint8_t *)&raw, 5);
    return dw1000_le64_to_cpu(raw);
}

/* Signed distance on the 40-bit clock, as the model computes it. */
static int64_t
delta(uint64_t a, uint64_t b)
{
    uint64_t d = (b - a) & ((1ull << DW1000_TIME_CLOCK_BITS) - 1);
    return (d & (1ull << (DW1000_TIME_CLOCK_BITS - 1)))
	 ? (int64_t)d - (int64_t)(1ull << DW1000_TIME_CLOCK_BITS)
	 : (int64_t)d;
}

/* SYS_TIME used to read back zero for ever. It should now track the
 * clock the medium stamps with, and move on its own.
 */
static const char *
step_sys_time(dw1000_t *dw, struct stub *s)
{
    uint64_t before, reg, after, later;

    (void)s;

    before = dw1000_emulation_clock();
    reg    = dw1000_get_system_time(dw);
    after  = dw1000_emulation_clock();

    if (reg == 0)
	return "SYS_TIME still reads zero";
    if (delta(before, reg) < 0 || delta(reg, after) < 0)
	return REASON("SYS_TIME 0x%010" PRIx64 " is outside the bracket"
		      " [0x%010" PRIx64 ", 0x%010" PRIx64 "]",
		      reg, before, after);

    usleep(20000);
    later = dw1000_get_system_time(dw);

    /* 20 ms of sleep, so at least 10 ms of clock: a machine that slept
     * short by half is a machine with other problems.
     */
    if (delta(reg, later) < (int64_t)MSEC(10))
	return REASON("SYS_TIME advanced %" PRId64 " ticks over a 20 ms"
		      " sleep", delta(reg, later));

    return NULL;
}

/* A delayed send. The frame must leave near the programmed time, and
 * TX_RAWST must be that time exactly: UM 3.3 makes the RMARKER the
 * programmed value by construction, so there is nothing to round.
 */
static const char *
step_tx_delayed(dw1000_t *dw, struct stub *s)
{
    uint64_t dx, want_raw, raw, stamp, seen;
    unsigned before;
    uint16_t antd = 16436;

    evt_reset();
    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    before     = s->tx_count;
    pthread_mutex_unlock(&s->lock);

    /* 5 ms ahead: far enough that neither HPDWARN (which needs the start
     * time to be behind us) nor TXPUTE (which needs it within a few
     * microseconds) should be raised.
     */
    dx = (dw1000_get_system_time(dw) + MSEC(5)) & ((1ull << DW1000_TIME_CLOCK_BITS) - 1);
    want_raw = dx & ~0x1FFull;

    dw1000_tx_write_frame_data(dw, (uint8_t *)payload, PAYLOAD_LEN, 0);
    dw1000_tx_fctrl(dw, FRAME_LEN, 0, DW1000_TX_IMMEDIATE);
    dw1000_txrx_set_time(dw, dx);

    if (dw1000_tx_start(dw, DW1000_TX_DELAYED_START) != 0)
	return "dw1000_tx_start refused a send programmed 5 ms ahead";

    /* Nothing should have gone out yet. */
    pthread_mutex_lock(&s->lock);
    seen = s->tx_count;
    pthread_mutex_unlock(&s->lock);
    if (seen != before)
	return "the frame went out before its programmed time";

    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the delayed transmission";
    if (!evt.tx_done)
	return "the tx_done callback was not called";

    pthread_mutex_lock(&s->lock);
    seen = s->tx_seen_at;
    pthread_mutex_unlock(&s->lock);

    raw   = tx_get_rawst(dw);
    stamp = dw1000_tx_get_rmarker_time(dw);

    if (raw != want_raw)
	return REASON("TX_RAWST is 0x%010" PRIx64 ", programmed 0x%010"
		      PRIx64, raw, want_raw);
    if (stamp != ((want_raw + antd) & ((1ull << DW1000_TIME_CLOCK_BITS) - 1)))
	return REASON("TX_STAMP is 0x%010" PRIx64 ", want raw + antenna"
		      " delay 0x%010" PRIx64, stamp, want_raw + antd);

    /* And it really was held: the medium saw the request no earlier than
     * the programmed time.
     *
     * Only the lower bound is asserted, and it is the one that says
     * something about the model: a model that ignored DX_TIME and sent
     * at once would trip it by 5 ms. There is no upper bound, because
     * there is nothing honest to put in it: how long after the deadline
     * the frame actually reaches the medium is how long the host took to
     * schedule the model's deadline thread and push a datagram, which is
     * a property of the machine and its load, not of the register model.
     * An earlier version of this step allowed 2 ms and failed about one
     * run in twenty on a loaded box, measuring the scheduler and calling
     * it a defect.
     *
     * What would have been caught by an upper bound (a deadline armed
     * on the wrong lap of the 40-bit counter, which is 17.2 s out) is
     * already caught by wait_irq() above timing out, and the exact
     * TX_RAWST check is what proves the model computed the right moment
     * whatever the scheduler then did with it.
     *
     * The half-millisecond of slack is for the ordering of two clock
     * reads on different threads, not for lateness.
     */
    if (delta(want_raw, seen) < -(int64_t)USEC(500))
	return REASON("the medium saw the frame %" PRId64 " ticks BEFORE"
		      " the programmed time", delta(seen, want_raw));

    return NULL;
}

/* A delayed send programmed for a time already gone by. UM 3.3 raises
 * HPDWARN and leaves the decision to the host; the driver's decision is
 * to cancel and report -1.
 */
static const char *
step_tx_delayed_late(dw1000_t *dw, struct stub *s)
{
    unsigned before, after;
    uint64_t dx;

    evt_reset();
    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    before     = s->tx_count;
    pthread_mutex_unlock(&s->lock);

    dx = (dw1000_get_system_time(dw) - MSEC(5)) & ((1ull << DW1000_TIME_CLOCK_BITS) - 1);

    dw1000_tx_write_frame_data(dw, (uint8_t *)payload, PAYLOAD_LEN, 0);
    dw1000_tx_fctrl(dw, FRAME_LEN, 0, DW1000_TX_IMMEDIATE);
    dw1000_txrx_set_time(dw, dx);

    if (dw1000_tx_start(dw, DW1000_TX_DELAYED_START) == 0)
	return "dw1000_tx_start accepted a send programmed in the past";

    /* The driver answered TRXOFF, so the held frame must have been
     * dropped rather than going out a lap later.
     */
    usleep(50000);
    pthread_mutex_lock(&s->lock);
    after = s->tx_count;
    pthread_mutex_unlock(&s->lock);
    if (after != before)
	return "the cancelled frame was transmitted anyway";

    /* UM 7.2.17: HPDWARN "is READ ONLY. It will clear when the delayed
     * TX/RX is cancelled". The driver's answer to the warning was a
     * TRXOFF, which is that cancellation, so the bit must now be gone,
     * without the host writing anything, because writing it does
     * nothing. A model that latched it would leave it set here.
     */
    if ((_dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE) &
	 DW1000_FLG_SYS_STATUS_HPDWARN) != 0)
	return "HPDWARN survived the TRXOFF that cancelled the send";

    /* And the bit being read only is not a detail: try writing 1 to it,
     * the way a host would clear an ordinary status bit, and check that
     * nothing happens, here by confirming the next delayed send still
     * works. This is the consequence that matters. A model that latched
     * HPDWARN would refuse every delayed operation from here on, which
     * is a node that has silently lost delayed send for the rest of the
     * run after one late response.
     */
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
			DW1000_FLG_SYS_STATUS_HPDWARN);

    return NULL;
}

/* The step above left a node that has seen one late send. A good delayed
 * send must still work: this is what fails when HPDWARN is latched.
 */
static const char *
step_tx_delayed_after_late(dw1000_t *dw, struct stub *s)
{
    return step_tx_delayed(dw, s);
}

/* A send issued while the previous one is still in flight is refused.
 *
 * The send functions have always documented "the DW1000 is in IDLE
 * state" as a precondition and never checked it. Errata 1.4 3.3 (TX-2)
 * is what that costs: "the new data written is written erroneously at
 * offset 0, thus corrupting the data currently being transmitted". So a
 * caller sending faster than the air allows was accepted every time and
 * lost nearly every frame. Measured on the bench at 6.8 Mbps, a 27 byte
 * frame every 0.15 ms against 0.17 ms of airtime: 1 frame of 20000
 * arrived, every call having returned success.
 *
 * The refusal is on dw->tx_pending, the driver's own record, so it costs
 * no SPI. What this step pins down is the release as much as the
 * refusal: a flag that were never cleared would pass the middle check
 * here and fail the last one.
 *
 * Runs after the receive steps on purpose. With the guard removed the
 * sends here go out on top of one another and leave the transmitter
 * busy, which fails whatever follows; put earlier, a regression reports
 * as four receive failures with the real one buried first.
 */
static const char *
step_tx_while_busy(dw1000_t *dw, struct stub *s)
{
    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0)
	return "the first send was refused with an idle transmitter";

    /* Straight away, before anything has processed the completion. */
    int busy = dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
			      DW1000_TX_IMMEDIATE);
    if (busy != DW1000_TX_ERR_BUSY)
	return REASON("a second send while the first was in flight"
		      " answered %d, not DW1000_TX_ERR_BUSY (%d): its"
		      " buffer write would have corrupted the frame on air",
		      busy, DW1000_TX_ERR_BUSY);

    /* The completion releases the transmitter. */
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done)
	return "no completion for the first send";

    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0)
	return "the transmitter was still held after its completion was"
	       " reported: the flag is set somewhere and cleared nowhere";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done)
	return "no completion for the send after the release";

    /* And dw1000_txrx_idle() releases it too, for a caller that abandons
     * a transmission rather than waiting for it.
     */
    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0)
	return "the third send was refused";
    dw1000_txrx_idle(dw);
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0)
	return "dw1000_txrx_idle() did not release the transmitter";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done)
	return "no completion for the send after the idle";

    return NULL;
}

/* A receive error latched before a send does not abort the send.
 *
 * dw1000_txrx_idle() keeps the pending events, by design, so the event
 * pass that follows a send can find an error the receiver raised just
 * before it, while the frame is on the air. Answering that error with
 * TRXOFF, as the error path does with the transmitter idle, cuts the
 * frame: the chip raises no TXFRS for a transmission it never finished,
 * and the host waits for a completion that cannot come. ruby-dw1000 met
 * it on a two-node bench as one send in a thousand timing out while the
 * peer was transmitting too.
 *
 * The bad frame comes from the stub: a frame sent without the automatic
 * CRC carries payload where its FCS should be, and the model rejects it
 * on receive with RXFCE. The stub holds the completion of the send under
 * test back for a while, so that the event pass runs with the frame
 * still on the air, which is the ordering the race produces.
 */
static const char *
step_tx_over_stale_rx_error(dw1000_t *dw, struct stub *s)
{
    uint32_t status;

    /* Leave the bad frame in the stub. */
    pthread_mutex_lock(&s->lock);
    s->deliver          = false;
    s->tx_done_delay_ms = 0;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE | DW1000_TX_NO_AUTO_CRC) != 0)
	return "the uncrc'd frame was refused";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done)
	return "the uncrc'd frame did not complete";

    /* Get it back, and leave the error it raises unprocessed. */
    pthread_mutex_lock(&s->lock);
    s->deliver = true;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";
    if (!wait_line(IRQ_TIMEOUT_MS))
	return "no interrupt for the bad frame";
    status = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
    if (!(status & DW1000_FLG_SYS_STATUS_RXFCE))
	return REASON("RXFCE not in the status (0x%08" PRIx32 ") after"
		      " the bad frame", status);

    /* Send over it, the way a host does: IDLE, keeping the events. */
    pthread_mutex_lock(&s->lock);
    s->deliver          = false;
    s->tx_done_delay_ms = 100;
    pthread_mutex_unlock(&s->lock);

    dw1000_txrx_idle(dw);
    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0)
	return "the send over the stale error was refused";

    /* The event pass a host runs next, with the frame on the air. The
     * re-arm its rx_error does is refused, nothing written: the model
     * would otherwise abort on a receiver enabled during a send. */
    rearm = true;
    dw1000_process_events(dw);
    rearm = false;
    if (!evt.rx_error)
	return "the stale receive error was not reported";
    if (evt.rx_start_rc != DW1000_RX_ERR_BUSY)
	return REASON("dw1000_rx_start() from rx_error over the send answered"
		      " %d, not DW1000_RX_ERR_BUSY (%d)", evt.rx_start_rc,
		      DW1000_RX_ERR_BUSY);
    if (evt.tx_done)
	return "the completion was reported before the frame was out";

    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done)
	return "no completion for the send over the stale error: the"
	       " error path cut the frame on the air";

    pthread_mutex_lock(&s->lock);
    s->tx_done_delay_ms = 0;
    pthread_mutex_unlock(&s->lock);
    return NULL;
}

/* A receive start from a callback whose pass already carries the
 * completion is honoured. The send is over, TXFRS on the chip says so,
 * and a host that leaves the re-arm to rx_ok on seeing RXFCG beside its
 * completion (the SPANK firmwares do) would otherwise be left deaf, the
 * refusal above being keyed on tx_pending, which the TXFRS branch clears
 * only after the good-frame one has run.
 */
static const char *
step_rx_start_beside_completion(dw1000_t *dw, struct stub *s)
{
    uint32_t status;
    int      i;

    /* A good frame, latched and left unprocessed: the stub holds the
     * good frame the previous step sent last. */
    pthread_mutex_lock(&s->lock);
    s->deliver          = true;
    s->tx_done_delay_ms = 0;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";
    if (!wait_line(IRQ_TIMEOUT_MS))
	return "no interrupt for the good frame";
    status = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
    if (!(status & DW1000_FLG_SYS_STATUS_RXFCG))
	return REASON("RXFCG not in the status (0x%08" PRIx32 ") after"
		      " the good frame", status);

    /* Send over it, and let the send complete before the pass. */
    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    dw1000_txrx_idle(dw);
    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0)
	return "the send over the pending frame was refused";
    for (i = 0; i < 200 && !dw1000_tx_is_status_done(dw); i++)
	usleep(10000);
    if (!dw1000_tx_is_status_done(dw))
	return "the send over the pending frame did not complete";

    /* One pass carrying both. rx_ok re-arms, as a single buffered
     * host's does, and the stub answers that start with a frame, which
     * is what says the receiver really is on. */
    pthread_mutex_lock(&s->lock);
    s->deliver = true;
    pthread_mutex_unlock(&s->lock);

    rearm = true;
    dw1000_process_events(dw);
    rearm = false;
    if (!evt.rx_ok)
	return "the pending frame was not reported";
    if (!evt.tx_done)
	return "the completion was not reported in the same pass";
    if (evt.rx_start_rc != 0)
	return REASON("dw1000_rx_start() from rx_ok beside the completion"
		      " answered %d: refused, and a host trusting it is deaf",
		      evt.rx_start_rc);

    /* No edge to wait for: the line may not have dropped between the
     * completion and the new frame. Poll the flag instead. */
    evt.rx_ok = false;
    for (i = 0; i < 200; i++) {
	status = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS,
				    DW1000_OFF_NONE);
	if (status & DW1000_FLG_SYS_STATUS_RXFCG)
	    break;
	usleep(10000);
    }
    dw1000_process_events(dw);
    if (!evt.rx_ok)
	return "the receiver started from rx_ok beside the completion"
	       " received nothing";

    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);
    return NULL;
}

/* A frame the chip cannot carry is refused, not truncated.
 *
 * dw1000_tx_fctrl() clamps an over-long length, which is right for a
 * caller writing TX_FCTRL itself but wrong underneath the send
 * functions: there the length came from the caller's own payload, and
 * the automatic CRC is added to it before the ceiling applies. A payload
 * two bytes under the limit therefore produced a frame two bytes over,
 * which was clamped back to the limit and transmitted with its last
 * bytes missing, while the send reported success. The assert beside the
 * clamp catches nothing on four of the five ports.
 *
 * This port is the fifth, so what a regression looks like HERE is the
 * abort that assert raises, not the silent truncation the shipping ports
 * show: with the refusal removed the suite dies on
 * "Assertion failed: (length <= max_length)" instead of failing a step.
 * Either way the step is what notices. It is also the point of putting
 * the refusal in _dw1000_tx_prepare_fctrl() rather than in
 * dw1000_tx_fctrl(): the send returns -1 before the assert can fire, so
 * the behaviour is the same on all five ports.
 */
static const char *
step_tx_frame_too_long(dw1000_t *dw, struct stub *s)
{
    const size_t max = dw1000_tx_get_frame_maxsize(dw);
    uint8_t      big[DW1000_FRAME_MAXSIZE];
    unsigned     before, after;

    memset(big, 0x5A, sizeof(big));

    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    before     = s->tx_count;
    pthread_mutex_unlock(&s->lock);

    /* One byte of payload too many: with the CRC it is one over the
     * ceiling. This is the case that used to go out truncated.
     */
    int rc = dw1000_tx_send(dw, big, max - DW1000_CRC_LENGTH + 1,
			    DW1000_TX_IMMEDIATE);
    if (rc != DW1000_TX_ERR_FRAME_SIZE)
	return REASON("a %zu byte payload, one over the %zu byte ceiling"
		      " once the CRC is added, answered %d and not"
		      " DW1000_TX_ERR_FRAME_SIZE (%d)",
		      max - DW1000_CRC_LENGTH + 1, max, rc,
		      DW1000_TX_ERR_FRAME_SIZE);

    /* And refused means nothing was sent. */
    pthread_mutex_lock(&s->lock);
    after = s->tx_count;
    pthread_mutex_unlock(&s->lock);
    if (after != before)
	return "the refused frame went out anyway";

    /* Exactly the ceiling, CRC included, is still accepted: the check
     * must not cost a byte of the usable frame.
     */
    evt_reset();
    if (dw1000_tx_send(dw, big, max - DW1000_CRC_LENGTH,
		       DW1000_TX_IMMEDIATE) != 0)
	return REASON("a %zu byte payload, exactly the ceiling once the"
		      " CRC is added, was refused",
		      max - DW1000_CRC_LENGTH);
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done)
	return "the largest legal frame did not complete";

    /* Without the automatic CRC the whole frame is the caller's, so the
     * same payload that was refused above is legal here.
     */
    evt_reset();
    if (dw1000_tx_send(dw, big, max,
		       DW1000_TX_IMMEDIATE | DW1000_TX_NO_AUTO_CRC) != 0)
	return REASON("a %zu byte payload with DW1000_TX_NO_AUTO_CRC was"
		      " refused, though nothing is added to it", max);
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done)
	return "the largest legal uncrc'd frame did not complete";

    /* Leave a frame the medium can replay. The stub holds the last one
     * transmitted, and step_rx_frame_beats_timeout() turns delivery on
     * to get it back; the DW1000_TX_NO_AUTO_CRC frame just sent carries
     * filler where its FCS should be, so the model would reject it on
     * receive and that step would wait for a frame that never comes.
     * Send the ordinary payload last, as the other transmit steps
     * happen to leave behind.
     */
    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0)
	return "the closing frame was refused";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done)
	return "the closing frame did not complete";

    return NULL;
}

/* The embedded transmit timestamp, and the frame it has to fit in.
 *
 * DW1000_TX_DELAYED_EMBED_TIMESTAMP writes the programmed send time into
 * the frame at the caller's offset, into the chip's transmit buffer and
 * into the caller's own copy alike, so that the host knows when the
 * frame leaves without waiting for the completion. Two halves are
 * asserted here: the value really is the one TX_STAMP reports
 * afterwards, and an offset leaving no room for it is refused instead of
 * written past the end of the frame.
 *
 * The refusal is the half that was missing.
 * dw1000_tx_write_frame_data() clamps to the 1024 byte transmit buffer,
 * not to the frame length already in TX_FCTRL, so bytes past the end
 * were written where nothing transmits them; _dw1000_iovec_write()
 * dropped the very same bytes from the caller's copy, so a host checking
 * its copy against TX_STAMP agreed with the truncation; and the send
 * returned 0. A frame went out carrying a partial timestamp and nothing
 * anywhere said so.
 */
static const char *
step_tx_embed_timestamp(dw1000_t *dw, struct stub *s)
{
    const uint32_t lead = MSEC(5);
    const int      mode = DW1000_TX_DELAYED_EMBED_TIMESTAMP             |
			  DW1000_TX_DELAYED_EMBED_TIMESTAMP_40BIT       |
			  DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN |
			  DW1000_TX_DELAYED_DELAY;
    uint8_t   frame[PAYLOAD_LEN], untouched[PAYLOAD_LEN];
    unsigned  before, after;
    uint64_t  embedded = 0, stamp;
    int       rc;

    memcpy(frame,     payload, PAYLOAD_LEN);
    memcpy(untouched, payload, PAYLOAD_LEN);

    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    before     = s->tx_count;
    pthread_mutex_unlock(&s->lock);

    /* Five bytes asked for two bytes from the end: three of them have
     * nowhere to go.
     */
    rc = dw1000_tx_extended_send(dw, frame, PAYLOAD_LEN, mode,
				 (size_t)(PAYLOAD_LEN - 2), lead);
    if (rc != DW1000_TX_ERR_TIMESTAMP)
	return REASON("a 5 byte timestamp 2 bytes from the end of the"
		      " frame answered %d, not DW1000_TX_ERR_TIMESTAMP (%d)",
		      rc, DW1000_TX_ERR_TIMESTAMP);
    if (memcmp(frame, untouched, PAYLOAD_LEN) != 0)
	return "the refused send wrote into the caller's frame anyway";

    /* And one that starts past the end entirely. */
    rc = dw1000_tx_extended_send(dw, frame, PAYLOAD_LEN, mode,
				 (size_t)(PAYLOAD_LEN + 4), lead);
    if (rc == 0)
	return "a timestamp offset past the end of the frame was accepted";
    if (memcmp(frame, untouched, PAYLOAD_LEN) != 0)
	return "the refused send wrote into the caller's frame anyway";

    /* Refused means refused: nothing was handed to the chip to send. */
    pthread_mutex_lock(&s->lock);
    after = s->tx_count;
    pthread_mutex_unlock(&s->lock);
    if (after != before)
	return REASON("%u frame(s) went out for the two refused sends",
		      after - before);

    /* An offset that fits. DW1000_TX_DELAYED_START is deliberately not
     * passed: the extended send sets it itself for an embedded
     * timestamp, since the value embedded is a programmed time.
     */
    evt_reset();
    rc = dw1000_tx_extended_send(dw, frame, PAYLOAD_LEN, mode,
				 (size_t)4, lead);
    if (rc != 0)
	return REASON("a 5 byte timestamp at offset 4 of a %d byte frame"
		      " was refused (%d)", PAYLOAD_LEN, rc);

    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the embedded-timestamp send";
    if (!evt.tx_done)
	return "the tx_done callback was not called";

    /* It landed in its five bytes and nowhere else... */
    if ((memcmp(frame, untouched, 4) != 0) ||
	(memcmp(frame + 9, untouched + 9, PAYLOAD_LEN - 9) != 0))
	return "the embedded timestamp landed outside its five bytes";

    for (int i = 4 ; i >= 0 ; i--)
	embedded = (embedded << 8) | frame[4 + i];

    /* ...and it is the time the frame actually left, antenna delay
     * included, which is what the caller is promised it can rely on
     * without waiting for the completion.
     */
    stamp = dw1000_tx_get_rmarker_time(dw);
    if (embedded != stamp)
	return REASON("the frame carries 0x%010" PRIx64 " but TX_STAMP"
		      " reports 0x%010" PRIx64, embedded, stamp);

    return NULL;
}

/* The frame wait timeout: receiver on, nothing delivered, RXRFTO. */
static const char *
step_rx_frame_wait_timeout(dw1000_t *dw, struct stub *s)
{
    uint64_t started;
    int64_t  elapsed;

    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    dw1000_rx_set_timeout_preamble(dw, 0);      /* RXPTO out of the way */
    dw1000_rx_set_timeout(dw, 19500);           /* ~20 ms, 1.026 us units */

    /* Timed, not just awaited. Checking only that the flag turns up
     * inside two seconds would pass a model that raised the timeout the
     * instant the receiver was enabled, which is the failure most worth
     * catching here: the arithmetic of the period is the thing being
     * tested, and 19500 units of 65536 ticks is 20 ms.
     */
    started = dw1000_get_system_time(dw);

    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";

    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the frame wait timeout";

    elapsed = delta(started, dw1000_get_system_time(dw));
    if (elapsed < (int64_t)MSEC(18))
	return REASON("RXRFTO came after %" PRId64 " ticks, and 19500"
		      " units of 65536 ticks is %" PRIu64,
		      elapsed, (uint64_t)19500 * 65536);
    if (!evt.rx_timeout)
	return "the rx_timeout callback was not called";
    if (!(evt.rx_timeout_status & DW1000_FLG_SYS_STATUS_RXRFTO))
	return REASON("RXRFTO not in the status (0x%08" PRIx32 ")",
		      evt.rx_timeout_status);
    if (evt.rx_ok || evt.rx_error)
	return "a frame was reported as well as the timeout";

    dw1000_rx_set_timeout(dw, 0);
    return NULL;
}

/* The preamble detection timeout. DRX_PRETOC counts PACs: PAC8 at 64 MHz
 * PRF is 8 * 508 * 128 ticks, about 8.14 us, and the counter adds one to
 * what is programmed (UM 7.2.40.9).
 */
static const char *
step_rx_preamble_timeout(dw1000_t *dw, struct stub *s)
{
    uint64_t started;
    int64_t  elapsed;

    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    dw1000_rx_set_timeout(dw, 0);               /* RXRFTO out of the way */
    dw1000_rx_set_timeout_preamble(dw, 2456);   /* ~20 ms */

    /* (2456 + 1) PACs of 8 symbols at 508*128 ticks is 20.0 ms, and the
     * point of timing it is the +1 and the PAC size: a model that used
     * the programmed value raw, or took the PAC as 16 symbols, would
     * still raise RXPTO and would still pass an untimed check.
     */
    started = dw1000_get_system_time(dw);

    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";

    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the preamble detection timeout";

    elapsed = delta(started, dw1000_get_system_time(dw));
    if (elapsed < (int64_t)MSEC(18) || elapsed > (int64_t)MSEC(30))
	return REASON("RXPTO came after %" PRId64 " ticks; (2456+1) PACs"
		      " of 8 symbols is %" PRIu64,
		      elapsed, (uint64_t)2457 * 8 * 508 * 128);
    if (!evt.rx_timeout)
	return "the rx_timeout callback was not called";
    if (!(evt.rx_timeout_status & DW1000_FLG_SYS_STATUS_RXPTO))
	return REASON("RXPTO not in the status (0x%08" PRIx32 ")",
		      evt.rx_timeout_status);

    dw1000_rx_set_timeout_preamble(dw, 0);
    return NULL;
}

/* UM 7.2.14(b): a frame arriving stops the counter, so no RXRFTO. */
static const char *
step_rx_frame_beats_timeout(dw1000_t *dw, struct stub *s)
{
    pthread_mutex_lock(&s->lock);
    s->deliver = true;                  /* the frame held from step_tx */
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    dw1000_rx_set_timeout_preamble(dw, 0);
    dw1000_rx_set_timeout(dw, 19500);           /* ~20 ms */

    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";

    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the received frame";
    if (!evt.rx_ok)
	return "the rx_ok callback was not called";
    if (evt.rx_timeout)
	return "the timeout fired even though a frame arrived";

    /* And it must not fire late either, after the frame was taken. */
    evt_reset();
    if (wait_irq(dw, 60) && evt.rx_timeout)
	return "the timeout fired after the frame had been received";

    dw1000_rx_set_timeout(dw, 0);
    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);
    return NULL;
}

/* A delayed receive whose turn-on time has gone by: HPDWARN, and the
 * driver falls back to an immediate start (returning 1) unless told to
 * stay idle.
 */
static const char *
step_rx_delayed_late(dw1000_t *dw, struct stub *s)
{
    uint64_t dx;

    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    dw1000_rx_set_timeout(dw, 0);
    dw1000_rx_set_timeout_preamble(dw, 0);

    dx = (dw1000_get_system_time(dw) - MSEC(5)) & ((1ull << DW1000_TIME_CLOCK_BITS) - 1);
    dw1000_txrx_set_time(dw, dx);

    if (dw1000_rx_start(dw, DW1000_RX_DELAYED_START |
			    DW1000_RX_IDLE_ON_DELAY_ERROR) != -1)
	return "a delayed receive programmed in the past was accepted";

    dw1000_txrx_off(dw);
    return NULL;
}

/* A delayed receive that is in time: the receiver must not turn on
 * before the programmed moment.
 */
static const char *
step_rx_delayed(dw1000_t *dw, struct stub *s)
{
    uint64_t dx;
    unsigned before, during, after;
    int      rc;

    pthread_mutex_lock(&s->lock);
    s->deliver  = false;
    before      = s->rxcfg_count;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    dw1000_rx_set_timeout(dw, 0);
    dw1000_rx_set_timeout_preamble(dw, 0);

    /* 60 ms, not 20: the step checks the receiver is still off 5 ms in,
     * and a loaded machine can take several milliseconds to get back to
     * this thread after the usleep. The margin is for the test's own
     * scheduling, not the model's.
     */
    dx = (dw1000_get_system_time(dw) + MSEC(60)) & ((1ull << DW1000_TIME_CLOCK_BITS) - 1);
    dw1000_txrx_set_time(dw, dx);

    rc = dw1000_rx_start(dw, DW1000_RX_DELAYED_START |
			     DW1000_RX_IDLE_ON_DELAY_ERROR);
    if (rc != 0)
	return REASON("dw1000_rx_start returned %d for a receive programmed"
		      " 20 ms ahead", rc);

    usleep(5000);
    pthread_mutex_lock(&s->lock);
    during = s->rxcfg_count;
    pthread_mutex_unlock(&s->lock);
    if (during != before)
	return "the receiver turned on before its programmed time";

    usleep(120000);
    pthread_mutex_lock(&s->lock);
    after = s->rxcfg_count;
    pthread_mutex_unlock(&s->lock);
    if (after == before)
	return "the receiver never turned on";

    dw1000_txrx_off(dw);
    return NULL;
}


/* WAIT4RESP: the chip turns its own receiver on at the end of a
 * transmission, and the receive timeout counts from that moment.
 *
 * This is the path a two-way ranging exchange rests on, and the reason
 * it matters is latency: with WAIT4RESP the receiver is listening at the
 * end of the sender's own frame, with no software in between. A
 * responder that answers in under a millisecond will be missed by a host
 * that re-arms the receiver from its transmit-complete callback, and
 * heard by one that set WAIT4RESP, so a model that quietly required
 * the software path would let an exchange pass in emulation that fails
 * on a fast link, or fail one that works.
 *
 * Two claims, and the second is the one that was missing: the receiver
 * comes on without dw1000_rx_start() being called, and the frame wait
 * timeout starts at that turn-on (UM 7.2.14, "each time the receiver is
 * enabled").
 */
static const char *
step_wait4resp(dw1000_t *dw, struct stub *s)
{
    unsigned rxcfg_before;
    uint64_t started;
    int64_t  elapsed;

    pthread_mutex_lock(&s->lock);
    s->deliver = false;                 /* nobody answers */
    rxcfg_before = s->rxcfg_count;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    dw1000_rx_set_timeout_preamble(dw, 0);
    dw1000_rx_set_timeout(dw, 19500);           /* ~20 ms */

    started = dw1000_get_system_time(dw);

    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_RESPONSE_EXPECTED) != 0)
	return "dw1000_tx_send refused a send with a response expected";

    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the transmission";
    if (!evt.tx_done)
	return "the tx_done callback was not called";

    /* The receiver is on now, and nothing asked it to be: no RX_CONFIG
     * was sent, because this turn-on happens inside the model on the
     * reader thread and there is no request it could make there.
     */
    pthread_mutex_lock(&s->lock);
    if (s->rxcfg_count != rxcfg_before) {
	pthread_mutex_unlock(&s->lock);
	return "the model asked the medium to start receiving; a WAIT4RESP"
	       " turn-on must not send RX_CONFIG";
    }
    pthread_mutex_unlock(&s->lock);

    /* Nobody answers, so the frame wait timeout must end it, which it
     * can only do if the turn-on armed it. Before this was fixed the
     * host waited for ever here.
     */
    evt_reset();
    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the frame wait timeout after WAIT4RESP;"
	       " the turn-on did not start the timeout";
    if (!evt.rx_timeout)
	return "the rx_timeout callback was not called";
    if (!(evt.rx_timeout_status & DW1000_FLG_SYS_STATUS_RXRFTO))
	return REASON("RXRFTO not in the status (0x%08" PRIx32 ")",
		      evt.rx_timeout_status);

    elapsed = delta(started, dw1000_get_system_time(dw));
    if (elapsed < (int64_t)MSEC(18))
	return REASON("the timeout came %" PRId64 " ticks after the send,"
		      " and 19500 units of 65536 ticks is 20 ms", elapsed);

    dw1000_rx_set_timeout(dw, 0);
    return NULL;
}

/* Create and tear down a model over and over.
 *
 * Every other step here creates one model and destroys it at exit, so
 * the shutdown path runs once per process and never in the state a
 * caller that loops through configurations puts it in. That caller
 * exists (a transmit power sweep builds a model per setting), and a
 * thread left behind each time would surface as a failure to create the
 * nth one, a long way from the call that leaked the first.
 *
 * So: full cycles, each with a deadline armed and a frame on its way, so
 * that the join has something to race rather than a thread already
 * parked on its condition variable.
 *
 * The assertion is on growth, not on equality. A joined thread's entry
 * lingers briefly on FreeBSD, so the count after a cycle is not reliably
 * the count before it; what cannot happen is the count rising with the
 * iteration count, which is what a leak looks like.
 */
#define LIFECYCLE_ROUNDS        30

static const char *
step_lifecycle(dw1000_t *dw, struct stub *s)
{
    int before, after;

    (void)dw;

    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    before = thread_count();

    for (int i = 0 ; i < LIFECYCLE_ROUNDS ; i++) {
	struct dw1000_emulation *em;
	rsvc_t                  *rs;
	dw1000_t                 d;
	struct dw1000_ioline     irq   = { .line = DW1000_IOLINE_IRQ   };
	struct dw1000_ioline     reset = { .line = DW1000_IOLINE_RESET };
	dw1000_spi_driver_t      sp;
	dw1000_config_t          cfg;
	struct dw1000_radio      radio = {
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

	if ((rs = rsvc_open(s->path, (char *)"lifecycle", NULL)) == NULL)
	    return REASON("rsvc_open failed on round %d", i);

	/* No line callback: these models are not being watched, and the
	 * one the rest of the file uses belongs to the other model.
	 */
	em = dw1000_emulation_create(rs, NULL, NULL);

	memset(&sp, 0, sizeof(sp));
	memset(&cfg, 0, sizeof(cfg));
	cfg.spi              = &sp;
	cfg.irq              = &irq;
	cfg.reset            = &reset;
	cfg.lde_loading      = 1;
	cfg.tx_antenna_delay = 16436;
	cfg.rx_antenna_delay = 16436;
	irq.emulation   = em;
	reset.emulation = em;
	sp.emulation    = em;

	dw1000_init(&d, &cfg);
	dw1000_hardreset(&d);
	if (dw1000_initialise(&d) != 0)
	    return REASON("dw1000_initialise failed on round %d", i);
	if (dw1000_configure(&d, &radio) != 0)
	    return REASON("dw1000_configure failed on round %d", i);

	/* Something for the teardown to race: a receive timeout counting
	 * down, and a delayed send whose deadline is close enough that
	 * it may well fire while we are tearing down.
	 */
	dw1000_rx_set_timeout(&d, 19500);
	dw1000_rx_start(&d, DW1000_RX_IMMEDIATE);

	dw1000_tx_write_frame_data(&d, (uint8_t *)payload, PAYLOAD_LEN, 0);
	dw1000_tx_fctrl(&d, FRAME_LEN, 0, DW1000_TX_IMMEDIATE);
	dw1000_txrx_set_time(&d,
	    (dw1000_get_system_time(&d) + USEC(800)) &
	    ((1ull << DW1000_TIME_CLOCK_BITS) - 1));
	dw1000_txrx_off(&d);
	dw1000_tx_start(&d, DW1000_TX_DELAYED_START);

	dw1000_emulation_stop(em);
	rsvc_close(rs);
	dw1000_emulation_destroy(em);
	free(rs);
    }

    after = thread_count();

    if (before < 0 || after < 0)
	return NULL;            /* no counter on this platform */

    /* Slack for the entries of just-joined threads, which is a constant;
     * a leak would be LIFECYCLE_ROUNDS of them.
     */
    if (after > before + 4)
	return REASON("thread count went from %d to %d over %d create and"
		      " destroy cycles", before, after, LIFECYCLE_ROUNDS);

    return NULL;
}


/* A server that does not answer is an error, not a hang.
 *
 * Every call a node makes is a request with a reply, and a server that
 * has stopped, or that does not recognise a service type, which is
 * what an older server does with a newer node, leaves nothing to wake
 * the caller. That used to block the node for ever, and because a node's
 * work happens on these calls, for ever meant the run was over with no
 * indication why. The wait is bounded now.
 *
 * Two things are checked, and the second matters as much as the first: 
 * that the call comes back at all, and that it comes back no sooner than
 * the bound. A call that failed instantly would also "not hang", and
 * would be a far worse bug: it would abandon replies that were merely
 * in flight.
 */
static const char *
step_unanswered_call(dw1000_t *dw, struct stub *s)
{
    struct timespec t0, t1;
    double          waited_ms;

    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    s->swallow = true;
    pthread_mutex_unlock(&s->lock);

    /* 200 ms rather than the 5 s default: the bound is what is under
     * test, not its size, and a test should not take five seconds to
     * prove a timeout works.
     */
    rsvc_set_reply_timeout(g_rsvc, 200);

    clock_gettime(CLOCK_REALTIME, &t0);
    dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
    clock_gettime(CLOCK_REALTIME, &t1);

    waited_ms = (double)(t1.tv_sec - t0.tv_sec) * 1000.0
	      + (double)(t1.tv_nsec - t0.tv_nsec) / 1000000.0;

    pthread_mutex_lock(&s->lock);
    s->swallow = false;
    pthread_mutex_unlock(&s->lock);
    rsvc_set_reply_timeout(g_rsvc, 5000);

    if (waited_ms < 150.0)
	return REASON("the call came back after %.0f ms, before its 200 ms"
		      " bound: a reply still in flight would have been"
		      " abandoned", waited_ms);
    if (waited_ms > 3000.0)
	return REASON("the call took %.0f ms against a 200 ms bound",
		      waited_ms);

    /* And the model is usable again: the unanswered call left it idle,
     * and the unreachable latch cleared when the server answered the
     * next one. Without that, one stall would silence the node for the
     * rest of the run.
     */
    pthread_mutex_lock(&s->lock);
    s->deliver = true;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "the receiver could not be restarted after a timed-out call";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.rx_ok)
	return "no frame after the server started answering again";

    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);
    return NULL;
}


/* Defined with the rest of the harness below; this step needs to build a
 * medium of its own, which is the only reason they are used up here.
 */
static bool make_socket_dir(char *dir, size_t dirlen,
			    char *sock, size_t socklen);
static bool stub_start(struct stub *s, const char *path, pthread_t *thread);

/* The medium server disappears while the node is still running.
 *
 * This is not a corner case; it is how every simulation ends. The
 * simulator takes its socket away and its nodes carry on for a moment,
 * transmitting into a path that is no longer there. The model used to
 * abort on that (an assert on a failed writev inside rsvc, and an
 * EMU_FATAL on a failed RX_CONFIG), so an orderly shutdown produced
 * what looked like a crash, after the run's work was already done and
 * with nothing useful in it. Observed in spank's simulation, where it
 * fired for two nodes out of ten in one run and none in the next.
 *
 * What should happen instead is what a radio does when there is nothing
 * on the other end: nothing. The send fails, it is reported once, and
 * the model goes back to idle rather than sitting in TX waiting for a
 * completion that cannot arrive.
 *
 * The assertion this step makes is simply that it finishes. An abort
 * takes the whole process with it, so a model that still aborted would
 * not reach the end of this function, let alone the steps after it.
 */
static const char *
step_medium_vanishes(dw1000_t *dw, struct stub *s)
{
    char   dir2[128];
    char   path2[sizeof(((struct sockaddr_un *)0)->sun_path)];
    struct stub s2;
    pthread_t   th2;
    rsvc_t     *rs;
    struct dw1000_emulation *em;
    dw1000_t    d;
    struct dw1000_ioline irq   = { .line = DW1000_IOLINE_IRQ   };
    struct dw1000_ioline reset = { .line = DW1000_IOLINE_RESET };
    dw1000_spi_driver_t  sp;
    dw1000_config_t      cfg;
    struct dw1000_radio  radio = {
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

    (void)dw; (void)s;

    /* A medium of its own, so that taking it away does not take the
     * one every other step is using with it.
     */
    if (!make_socket_dir(dir2, sizeof(dir2), path2, sizeof(path2)))
	return "cannot make a second socket directory";
    if (!stub_start(&s2, path2, &th2))
	return REASON("cannot start the second medium: %s", strerror(errno));

    if ((rs = rsvc_open(path2, (char *)"vanish", NULL)) == NULL)
	return "rsvc_open failed against the second medium";

    em = dw1000_emulation_create(rs, NULL, NULL);
    memset(&sp, 0, sizeof(sp));
    memset(&cfg, 0, sizeof(cfg));
    cfg.spi = &sp; cfg.irq = &irq; cfg.reset = &reset;
    cfg.lde_loading = 1;
    cfg.tx_antenna_delay = 16436;
    cfg.rx_antenna_delay = 16436;
    irq.emulation = em; reset.emulation = em; sp.emulation = em;

    dw1000_init(&d, &cfg);
    dw1000_hardreset(&d);
    if (dw1000_initialise(&d) != 0)
	return "dw1000_initialise failed against the second medium";
    if (dw1000_configure(&d, &radio) != 0)
	return "dw1000_configure failed against the second medium";

    /* The server goes away, properly, which means the socket itself
     * and not merely its name. Unlinking the path is not enough: the
     * node's socket is *connected*, so the binding outlives the name and
     * sends keep succeeding. It is the endpoint's destruction, when the
     * server process exits, that makes a send fail, so that is what is
     * reproduced here.
     */
    shutdown(s2.fd, SHUT_RDWR);
    pthread_join(th2, NULL);
    close(s2.fd);
    unlink(path2);

    /* Each of these reached an abort before the fix: the transmit
     * through rsvc's assert on writev, the receive through the model's
     * EMU_FATAL on a failed RX_CONFIG.
     */
    dw1000_tx_send(&d, (uint8_t *)payload, PAYLOAD_LEN, DW1000_TX_IMMEDIATE);

    dw1000_txrx_off(&d);        /* a well-behaved host, as the driver is */
    dw1000_rx_start(&d, DW1000_RX_IMMEDIATE);

    /* And again, which is the part that says the model recovered rather
     * than merely survived: after a failed send or receive it must be
     * idle, not stuck in TX or RX refusing everything that follows. The
     * model asserts on a transmit started from anything but idle, so a
     * model that did not reset its state would abort right here.
     */
    dw1000_txrx_off(&d);
    dw1000_tx_send(&d, (uint8_t *)payload, PAYLOAD_LEN, DW1000_TX_IMMEDIATE);

    dw1000_emulation_stop(em);
    rsvc_close(rs);
    dw1000_emulation_destroy(em);
    free(rs);
    rmdir(dir2);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* Harness                                                              */
/*----------------------------------------------------------------------*/

static void
on_alarm(int sig)
{
    (void)sig;
    static const char msg[] = "timing: timed out\n";
    ssize_t n = write(STDERR_FILENO, msg, sizeof(msg) - 1);
    (void)n;
    _exit(1);
}

static bool
make_socket_dir(char *dir, size_t dirlen, char *sock, size_t socklen)
{
    const char *tmp = getenv("TMPDIR");

    if (tmp == NULL || tmp[0] == '\0')
	tmp = "/tmp";

    if ((size_t)snprintf(dir, dirlen, "%s/dw1000-timing.XXXXXX", tmp) >= dirlen)
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
    s->fd = -1;
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
    pthread_t    stub_thread;
    rsvc_t      *rsvc;
    struct dw1000_emulation *emulation;
    uint32_t     seed;
    size_t       seedlen = sizeof(seed);
    struct stat  st;
    dw1000_t     dw;

    static struct dw1000_ioline ioline_irq   = { .line = DW1000_IOLINE_IRQ   };
    static struct dw1000_ioline ioline_reset = { .line = DW1000_IOLINE_RESET };
    static dw1000_spi_driver_t spi;

    static const dw1000_config_t config = {
	.spi              = &spi,
	.irq              = &ioline_irq,
	.reset            = &ioline_reset,
	.wakeup           = NULL,
	.leds             = 0,
	.lde_loading      = 1,
	.dblbuff          = 0,
	.rxauto           = 0,
	.tx_antenna_delay = 16436,
	.rx_antenna_delay = 16436,
	.cb.tx_done       = cb_tx_done,
	.cb.rx_ok         = cb_rx_ok,
	.cb.rx_error      = cb_rx_error,
	.cb.rx_timeout    = cb_rx_timeout,
    };

    /* Channel 5, 64 MHz PRF, 6.8 Mbps, PAC 8, 128-symbol preamble: the
     * numbers the steps above are sized against.
     */
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
    alarm(60);

    if (!make_socket_dir(dir, sizeof(dir), sockpath, sizeof(sockpath))) {
	fprintf(stderr, "timing: cannot make a socket directory\n");
	return 1;
    }

    if (!stub_start(&stub, sockpath, &stub_thread)) {
	fprintf(stderr, "timing: cannot start the stub medium: %s\n",
		strerror(errno));
	return 1;
    }

    if ((rsvc = rsvc_open(sockpath, (char *)"timing", NULL)) == NULL) {
	fprintf(stderr, "timing: rsvc_open failed\n");
	return 1;
    }

    g_rsvc    = rsvc;
    emulation = dw1000_emulation_create(rsvc, line_cb, NULL);
    ioline_irq.emulation   = emulation;
    ioline_reset.emulation = emulation;
    spi.emulation          = emulation;

    if (rsvc_o(rsvc, RSVC_SEED_GET, &seed, &seedlen) < 0) {
	fprintf(stderr, "timing: RSVC_SEED_GET failed\n");
	return 1;
    }

    dw1000_init(&dw, &config);
    dw1000_hardreset(&dw);
    if (dw1000_initialise(&dw) != 0) {
	fprintf(stderr, "timing: dw1000_initialise failed\n");
	return 1;
    }
    if (dw1000_configure(&dw, &radio) != 0) {
	fprintf(stderr, "timing: dw1000_configure failed\n");
	return 1;
    }

    step("sys_time runs",        step_sys_time(&dw, &stub));
    step("tx delayed",           step_tx_delayed(&dw, &stub));
    step("tx delayed, late",     step_tx_delayed_late(&dw, &stub));
    step("tx delayed after late",step_tx_delayed_after_late(&dw, &stub));
    step("tx embed timestamp",   step_tx_embed_timestamp(&dw, &stub));
    step("tx frame too long",    step_tx_frame_too_long(&dw, &stub));
    step("rx frame wait timeout",step_rx_frame_wait_timeout(&dw, &stub));
    step("rx preamble timeout",  step_rx_preamble_timeout(&dw, &stub));
    step("rx frame beats timeout", step_rx_frame_beats_timeout(&dw, &stub));
    step("unanswered call",      step_unanswered_call(&dw, &stub));
    step("rx delayed",           step_rx_delayed(&dw, &stub));
    step("rx delayed, late",     step_rx_delayed_late(&dw, &stub));
    step("wait4resp auto-rx",    step_wait4resp(&dw, &stub));
    step("tx while busy",        step_tx_while_busy(&dw, &stub));
    step("tx over a stale rx error", step_tx_over_stale_rx_error(&dw, &stub));
    step("rx start beside the completion", step_rx_start_beside_completion(&dw, &stub));
    step("create/destroy cycles",step_lifecycle(&dw, &stub));
    step("medium vanishes",      step_medium_vanishes(&dw, &stub));

    dw1000_txrx_off(&dw);

    /* Shutdown order, see dw1000/emulation.h.
     * doing it before rsvc_close() is what keeps a deadline from firing
     * into a closed connection.
     */
    /* The three steps, in the order dw1000/emulation.h insists on:
     * stop the model's thread, close the connection (which joins the
     * reader), and only then free the model.
     */
    pthread_mutex_lock(&stub.lock);
    stub.shutdown = true;
    pthread_mutex_unlock(&stub.lock);

    dw1000_emulation_stop(emulation);
    rsvc_close(rsvc);
    dw1000_emulation_destroy(emulation);

    pthread_join(stub_thread, NULL);
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
