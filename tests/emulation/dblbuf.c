/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The double receive buffer of UM 4.3: the swinging set of table 7, the
 * HSRBP and ICRBP pointers, the HRBPT command, and the overrun.
 *
 * This one works at the register level rather than through the driver's
 * callbacks. What is being checked is which of two register sets an
 * access lands in, and that is a question about the model, not about
 * the driver's event loop -- so the steps read SYS_STATUS, RX_FINFO and
 * RX_BUFFER directly and say where each byte came from.
 *
 * Its medium delivers a burst: on one RX_CONFIG it sends as many frames
 * as the step asked for, back to back, each carrying its own number in
 * its first payload byte and a good FCS. That is what an overrun needs
 * and what a single-frame medium cannot produce -- and it only works
 * because the model now honours RXAUTR, without which the receiver
 * would be idle again before the second frame arrived.
 *
 * The rules smoke.c states about any medium hold here too.
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

#define STUB_SEED                       0x0DB1F00Du


/*----------------------------------------------------------------------*/
/* Frame check sequence                                                 */
/*----------------------------------------------------------------------*/

/* The model checks an incoming frame's FCS, so a burst frame has to
 * carry a real one or it is a bad-CRC frame and the IC pointer does not
 * move -- which is a different test.
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

#define BURST_PAYLOAD_LEN       20
#define BURST_FRAME_LEN         (BURST_PAYLOAD_LEN + DW1000_CRC_LENGTH)

struct stub {
    int      fd;
    char     path[sizeof(((struct sockaddr_un *)0)->sun_path)];

    pthread_mutex_t lock;

    unsigned burst;                     /* frames to send on RX_CONFIG */
    unsigned next_id;                   /* first payload byte of the next */
    unsigned sent;                      /* frames actually sent */
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

/* One burst frame: payload[0] is its number, the rest is filler, and the
 * last two bytes are the FCS the model will check.
 */
static size_t
burst_frame(uint8_t *frame, unsigned id)
{
    uint16_t fcs;

    memset(frame, 0xA5, BURST_PAYLOAD_LEN);
    frame[0] = (uint8_t)id;
    fcs = crc16_ccitt(frame, BURST_PAYLOAD_LEN);
    frame[BURST_PAYLOAD_LEN    ] = (uint8_t)(fcs & 0xFF);
    frame[BURST_PAYLOAD_LEN + 1] = (uint8_t)(fcs >> 8);

    return BURST_FRAME_LEN;
}

static void
stub_uwb_io(struct stub *s, const struct sockaddr_un *peer, socklen_t peerlen,
	    uint64_t id, const uint8_t *payload, size_t paylen)
{
    struct dw1000_driver_iopkt in, out;

    memset(&in, 0, sizeof(in));
    memcpy(&in, payload, paylen < sizeof(in) ? paylen : sizeof(in));

    switch (in.type) {
    case DW1000_RSVC_TX:
	/* Nothing here transmits; answer so nothing hangs. */
	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, 0, 0, NULL, 0);
	break;

    case DW1000_RSVC_RX_CONFIG: {
	unsigned n, first;

	stub_send(s, peer, peerlen, RSVC_UWB_IO, id, 0, 0, NULL, 0);

	pthread_mutex_lock(&s->lock);
	n     = s->burst;
	first = s->next_id;
	s->next_id += n;
	s->burst    = 0;
	s->sent    += n;
	pthread_mutex_unlock(&s->lock);

	for (unsigned i = 0 ; i < n ; i++) {
	    size_t framelen;

	    memset(&out, 0, sizeof(out));
	    out.drvid        = 0;
	    out.type         = DW1000_RSVC_RX;
	    out.rx.flags     = 0;
	    out.rx.timestamp = dw1000_emulation_clock();
	    framelen = burst_frame(out.rx.frame, first + i);

	    stub_send(s, peer, peerlen, RSVC_UWB_IO, 0, 0,
		      RSVC_HDR_FLG_INTERRUPT, &out,
		      offsetof(struct dw1000_driver_iopkt, rx.frame) +
		      framelen);
	}
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

/* The line callback only signals; nothing here waits on the driver's
 * event loop, since the steps read registers rather than watch
 * callbacks. It is still needed: the model calls it, and a node that
 * supplied none would simply never be told.
 */
static struct {
    pthread_mutex_t lock;
    pthread_cond_t  wake;
    bool            irq;
} evt = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .wake = PTHREAD_COND_INITIALIZER,
};

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


/*----------------------------------------------------------------------*/
/* Steps                                                                */
/*----------------------------------------------------------------------*/

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

/* The two pointers, as a host reads them out of SYS_STATUS. */
static void
buffer_pointers(dw1000_t *dw, int *host, int *ic)
{
    uint32_t st = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS,
				     DW1000_OFF_NONE);
    *host = DW1000_GET_FLG(st, SYS_STATUS_HSRBP);
    *ic   = DW1000_GET_FLG(st, SYS_STATUS_ICRBP);
}

static uint32_t
sys_status(dw1000_t *dw)
{
    return _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
}

/* The HRBPT command, as UM 7.2.15 has the host issue it: the last byte
 * of SYS_CTRL and nothing else.
 */
static void
hrbpt(dw1000_t *dw)
{
    _dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, 3,
		       1 << (DW1000_SFT_SYS_CTRL_HRBPT - 24));
}

/* Clear the four swinging status bits in the set the host is on, the way
 * UM 4.3.3 has it done between frames.
 */
static void
clear_rx_status(dw1000_t *dw)
{
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
			DW1000_FLG_SYS_STATUS_RXDFR   |
			DW1000_FLG_SYS_STATUS_RXFCG   |
			DW1000_FLG_SYS_STATUS_RXFCE   |
			DW1000_FLG_SYS_STATUS_LDEDONE);
}

/* The first payload byte of the frame in the set the host is on: which
 * burst frame this buffer holds.
 */
static int
buffered_frame_id(dw1000_t *dw)
{
    uint8_t byte = 0;
    _dw1000_reg_read(dw, DW1000_REG_RX_BUFFER, 0, &byte, 1);
    return byte;
}

/* Ask the medium for a burst and give the frames time to land. */
static void
deliver(dw1000_t *dw, struct stub *s, unsigned n)
{
    pthread_mutex_lock(&s->lock);
    s->burst = n;
    pthread_mutex_unlock(&s->lock);

    dw1000_rx_start(dw, DW1000_RX_IMMEDIATE | DW1000_RX_NO_DBLBUF_SYNC);
    usleep(150000);
}

/* Nothing received yet: the two pointers must agree, which is what UM
 * 4.3.1 requires before the receiver is enabled at all.
 */
static const char *
step_aligned_at_reset(dw1000_t *dw, struct stub *s)
{
    int host, ic;

    (void)s;
    buffer_pointers(dw, &host, &ic);
    if (host != ic)
	return REASON("HSRBP is %d and ICRBP is %d before any frame",
		      host, ic);
    return NULL;
}

/* One frame. UM 4.3.2: a good CRC moves ICRBP and nothing moves HSRBP,
 * so the two must now differ -- that difference is how a host knows
 * there is a frame waiting.
 */
static const char *
step_one_frame_moves_ic(dw1000_t *dw, struct stub *s)
{
    int host, ic, id;

    deliver(dw, s, 1);

    buffer_pointers(dw, &host, &ic);
    if (host == ic)
	return REASON("ICRBP did not move for a good frame (both %d)", host);
    if (!(sys_status(dw) & DW1000_FLG_SYS_STATUS_RXFCG))
	return "RXFCG is not set in the set the host is on";

    id = buffered_frame_id(dw);
    if (id != 0)
	return REASON("the host's buffer holds frame %d, want 0", id);

    /* And releasing it brings them back together. */
    clear_rx_status(dw);
    hrbpt(dw);
    buffer_pointers(dw, &host, &ic);
    if (host != ic)
	return REASON("HRBPT left HSRBP at %d and ICRBP at %d", host, ic);

    return NULL;
}

/* Two frames, neither released. They must be in different sets, and the
 * host must reach them one at a time, in order.
 */
static const char *
step_two_frames_two_sets(dw1000_t *dw, struct stub *s)
{
    int host, ic, first, second;

    deliver(dw, s, 2);

    /* Both buffers are full, so the IC has come all the way round and
     * the pointers agree again -- with two frames outstanding, not none.
     */
    buffer_pointers(dw, &host, &ic);
    if (host != ic)
	return REASON("after two frames HSRBP is %d and ICRBP is %d",
		      host, ic);

    first = buffered_frame_id(dw);
    if (first != 1)
	return REASON("the first buffer holds frame %d, want 1", first);

    clear_rx_status(dw);
    hrbpt(dw);

    second = buffered_frame_id(dw);
    if (second != 2)
	return REASON("the second buffer holds frame %d, want 2", second);
    if (!(sys_status(dw) & DW1000_FLG_SYS_STATUS_RXFCG))
	return "the second buffer carries no RXFCG of its own";

    clear_rx_status(dw);
    hrbpt(dw);
    return NULL;
}

/* Three frames, none released: the third has nowhere to go. UM 4.3.5
 * sets RXOVRR and abandons it.
 */
static const char *
step_overrun(dw1000_t *dw, struct stub *s)
{
    unsigned before, after;

    pthread_mutex_lock(&s->lock);
    before = s->next_id;
    pthread_mutex_unlock(&s->lock);

    deliver(dw, s, 3);

    if (!(sys_status(dw) & DW1000_FLG_SYS_STATUS_RXOVRR))
	return "a third frame with both buffers full did not set RXOVRR";

    /* The two buffers hold the first two frames of the burst; the third
     * was abandoned, so neither buffer carries it.
     */
    if (buffered_frame_id(dw) != (int)before)
	return REASON("the host's buffer holds frame %d, want %u",
		      buffered_frame_id(dw), before);
    clear_rx_status(dw);
    hrbpt(dw);
    if (buffered_frame_id(dw) != (int)(before + 1))
	return REASON("the other buffer holds frame %d, want %u",
		      buffered_frame_id(dw), before + 1);

    /* UM 4.3.5: RXOVRR "will be cleared as soon as the host issues the
     * HRBPT command" -- and the one just issued did release a buffer, so
     * it is already gone.
     */
    if (sys_status(dw) & DW1000_FLG_SYS_STATUS_RXOVRR)
	return "RXOVRR survived the HRBPT that freed a buffer";

    clear_rx_status(dw);
    hrbpt(dw);

    pthread_mutex_lock(&s->lock);
    after = s->sent;
    pthread_mutex_unlock(&s->lock);
    if (after != before + 3)
	return REASON("the medium sent %u frames in total, expected %u",
		      after, before + 3);

    return NULL;
}

/* With DIS_DRXB set there is one buffer. Neither pointer moves however
 * many frames arrive, every frame lands in the same place, and there is
 * nothing to overrun.
 *
 * The two bits are not checked against zero: the manual does not say
 * they are reset when double buffering is turned off, so they keep
 * whatever they last were. What matters is that they stop moving.
 */
static const char *
step_single_buffered(dw1000_t *dw, struct stub *s)
{
    int host0, ic0, host1, ic1, id;
    unsigned first;
    uint32_t cfg;

    cfg = _dw1000_reg_read32(dw, DW1000_REG_SYS_CFG, DW1000_OFF_NONE);
    DW1000_SET_FLG(cfg, SYS_CFG_DIS_DRXB);
    _dw1000_reg_write32(dw, DW1000_REG_SYS_CFG, DW1000_OFF_NONE, cfg);

    buffer_pointers(dw, &host0, &ic0);

    pthread_mutex_lock(&s->lock);
    first = s->next_id;
    pthread_mutex_unlock(&s->lock);

    deliver(dw, s, 2);

    buffer_pointers(dw, &host1, &ic1);
    if (host1 != host0 || ic1 != ic0)
	return REASON("single buffered, but the pointers moved from"
		      " (%d,%d) to (%d,%d)", host0, ic0, host1, ic1);
    if (sys_status(dw) & DW1000_FLG_SYS_STATUS_RXOVRR)
	return "single buffered, but an overrun was reported";

    /* The buffer holds the FIRST frame of the burst, not the last.
     *
     * UM 7.2.6 gives RXAUTR two different meanings: double buffered it
     * re-enables "after a frame reception event or failure", single
     * buffered only "after a frame reception failure". So a good frame
     * stops the receiver here, and the second frame of the burst arrives
     * with nothing listening and is dropped. A model that re-enabled on
     * a good frame in single-buffered mode would leave frame 2 in the
     * buffer, and that is what this used to assert.
     */
    id = buffered_frame_id(dw);
    if (id != (int)first)
	return REASON("the single buffer holds frame %d, want %u"
		      " (a good frame must not re-enable the receiver when"
		      " single buffered)", id, first);

    clear_rx_status(dw);
    return NULL;
}


/*----------------------------------------------------------------------*/
/* Harness                                                              */
/*----------------------------------------------------------------------*/

static void
on_alarm(int sig)
{
    (void)sig;
    static const char msg[] = "dblbuf: timed out\n";
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

    if ((size_t)snprintf(dir, dirlen, "%s/dw1000-dblbuf.XXXXXX", tmp) >= dirlen)
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

    /* Double buffered, and with the receiver re-enabling itself: UM
     * 4.3.1 recommends the pair, and without the second the burst the
     * medium sends would be one frame followed by silence.
     */
    static const dw1000_config_t config = {
	.spi              = &spi,
	.irq              = &ioline_irq,
	.reset            = &ioline_reset,
	.wakeup           = NULL,
	.leds             = 0,
	.lde_loading      = 1,
	.dblbuff          = 1,
	.rxauto           = 1,
	.tx_antenna_delay = 16436,
	.rx_antenna_delay = 16436,
	.cb.tx_done       = NULL,
	.cb.rx_ok         = NULL,
	.cb.rx_error      = NULL,
	.cb.rx_timeout    = NULL,
    };

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
	fprintf(stderr, "dblbuf: cannot make a socket directory\n");
	return 1;
    }
    if (!stub_start(&stub, sockpath, &stub_thread)) {
	fprintf(stderr, "dblbuf: cannot start the stub medium: %s\n",
		strerror(errno));
	return 1;
    }
    if ((rsvc = rsvc_open(sockpath, (char *)"dblbuf", NULL)) == NULL) {
	fprintf(stderr, "dblbuf: rsvc_open failed\n");
	return 1;
    }

    emulation = dw1000_emulation_create(rsvc, line_cb, NULL);
    ioline_irq.emulation   = emulation;
    ioline_reset.emulation = emulation;
    spi.emulation          = emulation;

    if (rsvc_o(rsvc, RSVC_SEED_GET, &seed, &seedlen) < 0) {
	fprintf(stderr, "dblbuf: RSVC_SEED_GET failed\n");
	return 1;
    }

    dw1000_init(&dw, &config);
    dw1000_hardreset(&dw);
    if (dw1000_initialise(&dw) != 0) {
	fprintf(stderr, "dblbuf: dw1000_initialise failed\n");
	return 1;
    }
    if (dw1000_configure(&dw, &radio) != 0) {
	fprintf(stderr, "dblbuf: dw1000_configure failed\n");
	return 1;
    }

    /* No receive timeout: a step that waits for a burst must not have
     * the receiver switch itself off underneath it.
     */
    dw1000_rx_set_timeout(&dw, 0);
    dw1000_rx_set_timeout_preamble(&dw, 0);

    step("aligned at reset",        step_aligned_at_reset(&dw, &stub));
    step("one frame moves ICRBP",   step_one_frame_moves_ic(&dw, &stub));
    step("two frames, two sets",    step_two_frames_two_sets(&dw, &stub));
    step("three frames overrun",    step_overrun(&dw, &stub));
    step("single buffered",         step_single_buffered(&dw, &stub));

    dw1000_txrx_off(&dw);
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
