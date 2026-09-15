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
 * millisecond per stamp -- fine for checking that a timestamp arrives,
 * useless for checking when. The medium here stamps with
 * dw1000_emulation_clock() instead, the same clock the model's SYS_TIME
 * reads, which is the only way the two halves can be compared at all.
 *
 * It also holds its frames: nothing is delivered unless a step says so,
 * because a timeout is a thing that happens when no frame arrives.
 *
 * The two rules smoke.c states about any medium hold here too, and one
 * has changed in the model's favour: the line callback no longer runs
 * inside the model's mutex, so calling the driver from it would no
 * longer deadlock. It still must not -- dw1000_process_events() belongs
 * on the node's own thread -- and this test keeps to that.
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


/*----------------------------------------------------------------------*/
/* The wire protocol (docs/emulation.md)                                */
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

	pthread_mutex_lock(&s->lock);
	memcpy(s->frame, in.tx.frame, framelen);
	s->framelen   = framelen;
	s->ranging    = (in.tx.flags & DW1000_RSVC_FLG_RANGING) != 0;
	s->have_frame = true;
	s->tx_count++;
	s->tx_seen_at = now;

	/* The model asserts, for an immediate send, that TX_STAMP less
	 * its own TX_ANTD lands on a 512-tick boundary. So round first,
	 * then add the antenna delay the node supplied -- which is the
	 * convention docs/emulation.md states.
	 */
	stamp = (DW1000_CLOCK_ROUNDUP(now) + in.tx.antenna_delay)
	        & ((1ull << DW1000_TIME_CLOCK_BITS) - 1);
	s->tx_done_time = stamp;
	pthread_mutex_unlock(&s->lock);

	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, 0, 0, NULL, 0);

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

	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, 0, 0, NULL, 0);

	memset(&out, 0, sizeof(out));
	pthread_mutex_lock(&s->lock);
	s->rxcfg_count++;
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

	case RSVC_CLOSE:
	    stub_send(s, &peer, peerlen, hdr.type, hdr.id, 0, 0, NULL, 0);
	    running = false;
	    break;

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
} evt = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .wake = PTHREAD_COND_INITIALIZER,
};

static void cb_tx_done(dw1000_t *dw, uint32_t status)
{ (void)dw; (void)status; evt.tx_done = true; }

static void cb_rx_ok(dw1000_t *dw, uint32_t status, size_t length, bool rng)
{ (void)dw; (void)status; (void)length; (void)rng; evt.rx_ok = true; }

static void cb_rx_error(dw1000_t *dw, uint32_t status)
{ (void)dw; (void)status; evt.rx_error = true; }

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
}

static bool
wait_irq(dw1000_t *dw, unsigned timeout_ms)
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

static int  failures;
static char reason[256];

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
 * TX_RAWST must be that time exactly -- UM 3.3 makes the RMARKER the
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
     * something about the model -- a model that ignored DX_TIME and sent
     * at once would trip it by 5 ms. There is no upper bound, because
     * there is nothing honest to put in it: how long after the deadline
     * the frame actually reaches the medium is how long the host took to
     * schedule the model's deadline thread and push a datagram, which is
     * a property of the machine and its load, not of the register model.
     * An earlier version of this step allowed 2 ms and failed about one
     * run in twenty on a loaded box, measuring the scheduler and calling
     * it a defect.
     *
     * What would have been caught by an upper bound -- a deadline armed
     * on the wrong lap of the 40-bit counter, which is 17.2 s out -- is
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
     * TRXOFF, which is that cancellation, so the bit must now be gone --
     * without the host writing anything, because writing it does
     * nothing. A model that latched it would leave it set here.
     */
    if ((_dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE) &
	 DW1000_FLG_SYS_STATUS_HPDWARN) != 0)
	return "HPDWARN survived the TRXOFF that cancelled the send";

    /* And the bit being read only is not a detail: try writing 1 to it,
     * the way a host would clear an ordinary status bit, and check that
     * nothing happens -- here by confirming the next delayed send still
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

/* The frame wait timeout: receiver on, nothing delivered, RXRFTO. */
static const char *
step_rx_frame_wait_timeout(dw1000_t *dw, struct stub *s)
{
    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    dw1000_rx_set_timeout_preamble(dw, 0);      /* RXPTO out of the way */
    dw1000_rx_set_timeout(dw, 19500);           /* ~20 ms, 1.026 us units */

    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";

    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the frame wait timeout";
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
    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    dw1000_rx_set_timeout(dw, 0);               /* RXRFTO out of the way */
    dw1000_rx_set_timeout_preamble(dw, 2456);   /* ~20 ms */

    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";

    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the preamble detection timeout";
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
    step("rx frame wait timeout",step_rx_frame_wait_timeout(&dw, &stub));
    step("rx preamble timeout",  step_rx_preamble_timeout(&dw, &stub));
    step("rx frame beats timeout", step_rx_frame_beats_timeout(&dw, &stub));
    step("rx delayed",           step_rx_delayed(&dw, &stub));
    step("rx delayed, late",     step_rx_delayed_late(&dw, &stub));

    dw1000_txrx_off(&dw);

    /* The model owns a thread now, so it is stopped rather than dropped;
     * doing it before rsvc_close() is what keeps a deadline from firing
     * into a closed connection.
     */
    dw1000_emulation_destroy(emulation);

    rsvc_close(rsvc);
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
