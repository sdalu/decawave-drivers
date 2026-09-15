/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Smoke test for port/emulation: the driver, the register model, and a
 * medium -- all three in one process, with no chip and no server to
 * start.
 *
 * The medium is a thread rather than a program because that is the whole
 * point of the exercise: the port has no hardware behind it, so what is
 * left to check is that a node and a medium speaking the protocol of
 * docs/emulation.md carry a frame between them and agree on its
 * timestamps. The thread binds the server socket, answers every request,
 * and loops the node's own transmitted frame straight back at it, which
 * is enough to drive TX, RX, and a bad FCS through the driver's four
 * callbacks.
 *
 * Two rules the model imposes on any medium, both learned the hard way
 * (see osal.c):
 *
 *  - the IRQ line callback runs on the rsvc reader thread, inside the
 *    model's own non-recursive mutex, and every driver call takes that
 *    mutex to do its SPI transfers. So the callback here only signals a
 *    condition variable; dw1000_process_events() is called by the main
 *    thread, which holds nothing.
 *
 *  - the rsvc client blocks on a semaphore for a reply. The wait is
 *    bounded now (rsvc_set_reply_timeout(), five seconds by default), so
 *    a request left unanswered is an error rather than a hang -- but
 *    five seconds is a diagnosis, not a schedule, and a node that spends
 *    them has already lost whatever timing it had. So the stub still
 *    answers every type it is sent, with status -1 for one it does not
 *    know, and never by staying silent.
 *
 * The whole run is under alarm(20) for the hangs that remain possible.
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

/* Copied rather than included: <rsvc.h> exports the client API, while
 * the framing a server has to speak is described in docs/emulation.md
 * and written in rsvc.c, which is not a header. A medium server is
 * entitled to nothing but the document, so the test takes nothing else
 * either -- and a change to the framing that skips the document breaks
 * this test, which is the point.
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


/*----------------------------------------------------------------------*/
/* Frame check sequence                                                 */
/*----------------------------------------------------------------------*/

/* The same routine the model uses to append an FCS on transmit and to
 * check one on receive (e_crc16_ccitt(), static in osal.c). Recomputed
 * here so that the test checks the bytes on the wire against the 802.15.4
 * FCS, and not merely against the port agreeing with itself.
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
/* The stub medium                                                      */
/*----------------------------------------------------------------------*/

/* One millisecond of the device clock, which is what the fake clock
 * advances by every time it stamps something. Nothing in the test
 * measures a duration; the step only has to be visible and to keep the
 * timestamps ordered.
 */
#define STUB_CLOCK_START        0x0000010000000ull
#define STUB_CLOCK_STEP         63897600ull
#define STUB_CLOCK_MASK         ((1ull << DW1000_TIME_CLOCK_BITS) - 1)

#define STUB_SEED               0x5EEDF00Du

struct stub {
    int      fd;                        /* the bound server socket */
    char     path[sizeof(((struct sockaddr_un *)0)->sun_path)];

    /* Everything below is written by the stub thread and read by the
     * main thread once a step has completed, under this mutex.
     */
    pthread_mutex_t lock;

    uint64_t clock;                     /* 40-bit fake clock */

    uint8_t  frame[DW1000_FRAME_MAXSIZE]; /* the last frame transmitted */
    size_t   framelen;
    bool     ranging;
    bool     have_frame;
    unsigned delivered;                 /* RX deliveries so far */

    uint64_t tx_done_time;              /* what the last TX_DONE carried */
    uint64_t rx_time;                   /* ... and the last RX */
};

/* Take the clock's current value and move it on. */
static uint64_t
stub_stamp(struct stub *s)
{
    uint64_t now = s->clock;
    s->clock = (s->clock + STUB_CLOCK_STEP) & STUB_CLOCK_MASK;
    return now;
}

/* One datagram back to whoever sent the request. A reply and an
 * interrupt differ only by the id (0 for an interrupt) and the flag.
 */
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

/* An RSVC_UWB_IO request: TX, which the stub completes and remembers,
 * or RX_CONFIG, which hands the remembered frame back.
 */
static void
stub_uwb_io(struct stub *s, const struct sockaddr_un *peer, socklen_t peerlen,
	    uint64_t id, const uint8_t *payload, size_t paylen)
{
    struct dw1000_driver_iopkt in, out;

    /* Copied out of the datagram rather than cast onto it: the payload
     * starts 11 bytes into the buffer, so a pointer into it would be a
     * misaligned pointer to a packed struct.
     */
    memset(&in, 0, sizeof(in));
    memcpy(&in, payload, paylen < sizeof(in) ? paylen : sizeof(in));

    switch (in.type) {
    case DW1000_RSVC_TX: {
	size_t framelen = paylen -
	    offsetof(struct dw1000_driver_iopkt, tx.frame);
	uint64_t stamp;

	pthread_mutex_lock(&s->lock);
	memcpy(s->frame, in.tx.frame, framelen);
	s->framelen   = framelen;
	s->ranging    = (in.tx.flags & DW1000_RSVC_FLG_RANGING) != 0;
	s->have_frame = true;

	/* The port asserts that TX_STAMP minus its own TX_ANTD lands on a
	 * 512-tick boundary, so the rounding happens before the antenna
	 * delay the node supplied is added back on.
	 */
	stamp = (DW1000_CLOCK_ROUNDUP(stub_stamp(s)) + in.tx.antenna_delay)
	        & STUB_CLOCK_MASK;
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
	uint64_t stamp    = 0;

	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, 0, 0, NULL, 0);

	memset(&out, 0, sizeof(out));
	pthread_mutex_lock(&s->lock);
	deliver = s->have_frame;
	if (deliver) {
	    s->delivered++;
	    /* The second frame this medium ever delivers is delivered
	     * damaged, so that the driver's rx_error path is walked as
	     * well as its rx_ok one.
	     */
	    if (s->delivered == 2)
		s->frame[s->framelen - 1] ^= 0xFF;

	    framelen = s->framelen;
	    stamp    = stub_stamp(s);
	    s->rx_time = stamp;

	    /* The RX interrupt carries no node-local pointer, and the
	     * ranging flag is the sender's, echoed back.
	     */
	    out.drvid        = 0;
	    out.type         = DW1000_RSVC_RX;
	    out.rx.flags     = s->ranging ? DW1000_RSVC_FLG_RANGING : 0;
	    out.rx.timestamp = stamp;
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

	/* The nickname sits between the header and the payload, and only
	 * RSVC_OPEN carries one. Nothing here looks a node up, so it is
	 * stepped over rather than read.
	 */
	if (hdr.flags & RSVC_HDR_FLG_INCLUDE_NICKNAME) {
	    size_t nicklen = strnlen((char *)payload, paylen);
	    if (nicklen < paylen)
		nicklen++;              /* the NUL */
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

/* What the four callbacks saw. They are called by
 * dw1000_process_events(), on the main thread and nowhere else, so only
 * the interrupt flag crosses a thread and only it needs the mutex.
 */
static struct {
    pthread_mutex_t lock;
    pthread_cond_t  wake;
    bool            irq;

    bool     tx_done;
    uint32_t tx_done_status;

    bool     rx_ok;
    uint32_t rx_ok_status;
    size_t   rx_ok_length;
    bool     rx_ok_ranging;

    bool     rx_error;
    uint32_t rx_error_status;

    bool     rx_timeout;
    uint32_t rx_timeout_status;
} evt = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .wake = PTHREAD_COND_INITIALIZER,
};

static void
cb_tx_done(dw1000_t *dw, uint32_t status)
{
    (void)dw;
    evt.tx_done        = true;
    evt.tx_done_status = status;
}

static void
cb_rx_ok(dw1000_t *dw, uint32_t status, size_t length, bool ranging)
{
    (void)dw;
    evt.rx_ok         = true;
    evt.rx_ok_status  = status;
    evt.rx_ok_length  = length;
    evt.rx_ok_ranging = ranging;
}

static void
cb_rx_error(dw1000_t *dw, uint32_t status)
{
    (void)dw;
    evt.rx_error        = true;
    evt.rx_error_status = status;
}

static void
cb_rx_timeout(dw1000_t *dw, uint32_t status)
{
    (void)dw;
    evt.rx_timeout        = true;
    evt.rx_timeout_status = status;
}

/* The model calls this from its rsvc reader thread while holding its own
 * mutex, which every SPI transfer needs: anything here that reached for
 * the driver would deadlock against the call it is reporting. So it only
 * says that something happened.
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

/* Forget what the callbacks recorded, before the step that will fill it
 * in again. The interrupt flag goes too: a step waits for the interrupt
 * its own action raises.
 */
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
}

/* Wait for the line callback, then do what a real node would do from its
 * own thread: process the events, which is what calls the callbacks.
 */
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

static int failures;

/* One line per step. A step that fails says on that same line which of
 * its assertions went, so the output stays one line per step without
 * becoming useless when something breaks.
 */
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

/* Where a step formats the reason it failed for. One step runs at a
 * time, and the reason is printed before the next one starts.
 */
static char reason[256];

#define REASON(...) (snprintf(reason, sizeof(reason), __VA_ARGS__), reason)

static const uint8_t payload[PAYLOAD_LEN] = {
    0x41, 0x88, 0x00, 0xcf, 0xbc, 0x01, 0x00, 0x02, 0x00, 0x2a,
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99,
};

static const char *
step_tx(dw1000_t *dw, struct stub *s)
{
    uint8_t  frame[DW1000_FRAME_MAXSIZE];
    size_t   framelen;
    uint64_t sent, reported;
    uint16_t fcs, want;

    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0)
	return "dw1000_tx_send did not start the transmission";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the transmission";
    if (!evt.tx_done)
	return "the tx_done callback was not called";
    if (!(evt.tx_done_status & DW1000_FLG_SYS_STATUS_TXFRS))
	return REASON("TXFRS not in the tx_done status (0x%08" PRIx32 ")",
		      evt.tx_done_status);

    pthread_mutex_lock(&s->lock);
    framelen = s->framelen;
    memcpy(frame, s->frame, framelen);
    sent = s->tx_done_time;
    pthread_mutex_unlock(&s->lock);

    reported = dw1000_tx_get_rmarker_time(dw);
    if (reported != sent)
	return REASON("TX_STAMP is 0x%010" PRIx64 ", the medium sent"
		      " 0x%010" PRIx64, reported, sent);

    if (framelen != FRAME_LEN)
	return REASON("the medium saw %zu frame bytes, want %d",
		      framelen, FRAME_LEN);
    if (memcmp(frame, payload, PAYLOAD_LEN) != 0)
	return "the frame the medium saw is not the payload";

    /* The chip appends the FCS, the driver never sees it: so it is the
     * model's work that is being checked here.
     */
    fcs  = (uint16_t)frame[PAYLOAD_LEN] |
	   ((uint16_t)frame[PAYLOAD_LEN + 1] << 8);
    want = crc16_ccitt(payload, PAYLOAD_LEN);
    if (fcs != want)
	return REASON("FCS is 0x%04" PRIx16 ", want 0x%04" PRIx16, fcs, want);

    return NULL;
}

static const char *
step_rx_good(dw1000_t *dw, struct stub *s)
{
    uint8_t  data[PAYLOAD_LEN];
    uint64_t sent, reported;

    evt_reset();
    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the reception";
    if (!evt.rx_ok)
	return "the rx_ok callback was not called";
    if (evt.rx_ok_length != FRAME_LEN)
	return REASON("rx_ok reports %zu bytes, want %d",
		      evt.rx_ok_length, FRAME_LEN);
    if (evt.rx_ok_ranging)
	return "rx_ok reports a ranging frame";

    dw1000_rx_read_frame_data(dw, data, sizeof(data), 0);
    if (memcmp(data, payload, PAYLOAD_LEN) != 0)
	return "the frame read back is not the one that was sent";

    pthread_mutex_lock(&s->lock);
    sent = s->rx_time;
    pthread_mutex_unlock(&s->lock);

    reported = dw1000_rx_get_rmarker_time(dw);
    if (reported != sent)
	return REASON("RX_STAMP is 0x%010" PRIx64 ", the medium sent"
		      " 0x%010" PRIx64, reported, sent);

    return NULL;
}

static const char *
step_rx_bad_crc(dw1000_t *dw, struct stub *s)
{
    (void)s;

    evt_reset();
    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the damaged frame";
    if (evt.rx_ok)
	return "the rx_ok callback was called for a damaged frame";
    if (!evt.rx_error)
	return "the rx_error callback was not called";
    if (!(evt.rx_error_status & DW1000_FLG_SYS_STATUS_RXFCE))
	return REASON("RXFCE not in the rx_error status (0x%08" PRIx32 ")",
		      evt.rx_error_status);

    return NULL;
}

static const char *
step_tx_ranging(dw1000_t *dw, struct stub *s)
{
    bool ranging;

    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE | DW1000_TX_RANGING) != 0)
	return "dw1000_tx_send did not start the transmission";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the transmission";
    if (!evt.tx_done)
	return "the tx_done callback was not called";

    pthread_mutex_lock(&s->lock);
    ranging = s->ranging;
    pthread_mutex_unlock(&s->lock);
    if (!ranging)
	return "the medium was not told the frame was a ranging one";

    evt_reset();
    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0)
	return "dw1000_rx_start did not start the reception";
    if (!wait_irq(dw, IRQ_TIMEOUT_MS))
	return "no interrupt for the reception";
    if (!evt.rx_ok)
	return "the rx_ok callback was not called";
    if (!evt.rx_ok_ranging)
	return "rx_ok does not report a ranging frame";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* Plumbing                                                             */
/*----------------------------------------------------------------------*/

static void
on_alarm(int sig)
{
    static const char msg[] = "FAIL: timeout\n";

    (void)sig;
    /* Neither printf() nor exit() may be called from a signal handler. */
    if (write(STDOUT_FILENO, msg, sizeof(msg) - 1) < 0) { /* nothing to do */ }
    _exit(2);
}

/* The socket lives in a directory of its own, so that two runs at once
 * cannot collide and the cleanup is one rmdir that fails loudly if
 * anything was left behind.
 */
static bool
make_socket_dir(char *dir, size_t dirlen, char *sock, size_t socklen)
{
    const char *tmp = getenv("TMPDIR");

    if (tmp == NULL || tmp[0] == '\0')
	tmp = "/tmp";

    if ((size_t)snprintf(dir, dirlen, "%s/dw1000-smoke.XXXXXX", tmp) >= dirlen)
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
    pthread_t    stub_thread;
    rsvc_t      *rsvc;
    struct dw1000_emulation *emulation;
    uint32_t     seed;
    size_t       seedlen = sizeof(seed);
    struct stat  st;
    dw1000_t     dw;

    static struct dw1000_ioline ioline_irq = {
	.line = DW1000_IOLINE_IRQ,
    };
    static struct dw1000_ioline ioline_reset = {
	.line = DW1000_IOLINE_RESET,
    };
    static dw1000_spi_driver_t spi;

    /* The wakeup line is left unset: driving it aborts the process
     * (osal.c says so itself), and NULL is DW1000_IOLINE_NONE.
     */
    static const dw1000_config_t config = {
	.spi              = &spi,
	.irq              = &ioline_irq,
	.reset            = &ioline_reset,
	.wakeup           = NULL,
	.leds             = 0,
	.lde_loading      = 1,
	.dblbuff          = 0,   /* the model emulates neither the swing */
	.rxauto           = 0,   /* set nor an automatic re-enable */
	.tx_antenna_delay = 16436,
	.rx_antenna_delay = 16436,
	.cb.tx_done       = cb_tx_done,
	.cb.rx_ok         = cb_rx_ok,
	.cb.rx_error      = cb_rx_error,
	.cb.rx_timeout    = cb_rx_timeout,
    };

    /* No preamble length, PAC or preamble code that needs the
     * proprietary options: channel 5 at 64 MHz PRF, 6.8 Mbps.
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
    alarm(20);

    if (!make_socket_dir(dir, sizeof(dir), sockpath, sizeof(sockpath))) {
	fprintf(stderr, "smoke: cannot make a socket directory\n");
	return 1;
    }

    /* Before rsvc_open(), which sends RSVC_OPEN and waits for the
     * reply.
     */
    if (!stub_start(&stub, sockpath, &stub_thread)) {
	fprintf(stderr, "smoke: cannot start the stub medium: %s\n",
		strerror(errno));
	return 1;
    }

    if ((rsvc = rsvc_open(sockpath, (char *)"smoke", NULL)) == NULL) {
	fprintf(stderr, "smoke: rsvc_open failed\n");
	return 1;
    }

    emulation = dw1000_emulation_create(rsvc, line_cb, NULL);
    ioline_irq.emulation   = emulation;
    ioline_reset.emulation = emulation;
    spi.emulation          = emulation;

    if (rsvc_o(rsvc, RSVC_SEED_GET, &seed, &seedlen) < 0) {
	fprintf(stderr, "smoke: RSVC_SEED_GET failed\n");
	return 1;
    }

    dw1000_init(&dw, &config);
    dw1000_hardreset(&dw);
    if (dw1000_initialise(&dw) != 0) {
	fprintf(stderr, "smoke: dw1000_initialise failed\n");
	return 1;
    }
    if (dw1000_configure(&dw, &radio) != 0) {
	fprintf(stderr, "smoke: dw1000_configure failed\n");
	return 1;
    }

    step("tx",          step_tx(&dw, &stub));
    step("rx good",     step_rx_good(&dw, &stub));
    step("rx bad crc",  step_rx_bad_crc(&dw, &stub));
    step("tx ranging",  step_tx_ranging(&dw, &stub));

    dw1000_txrx_off(&dw);
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
