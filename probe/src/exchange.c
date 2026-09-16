/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * The two-node ranging exchange: the wire frame (build/parse), the
 * bounded waits, and the `twr_resp` / `twr_init` role bodies.
 *
 * FRAME MATCHING, in one predicate (frame_classify() below): a captured
 * frame is ours only if the "dwp" mark is there, the type is the one
 * being waited for, dst equals our own address, and -- except for a POLL,
 * whose seq is not yet known -- the seq matches the exchange in
 * progress. Anything else is ignored and the wait continues.
 *
 * WHAT WAS IGNORED IS NOW COUNTED, which is the whole of the difference
 * between a predicate and a classifier here. "The receiver heard
 * nothing" and "the receiver heard frames and rejected every one of
 * them" are different defects with different fixes, and an instrument
 * that cannot tell them apart sends its operator to the wrong half of
 * the bench. The account is per run and is reported on the STATS line;
 * <dw1000/probe/record.h> states what it partitions. Two of its
 * categories are about this file rather than about the link: a frame
 * that arrived while no wait was running, and a frame overwritten in the
 * single capture slot before a wait could look at it.
 *
 * TWO PLACES WHERE THE BRIEF'S OWN WIRE FORMAT CANNOT BE FOLLOWED
 * LITERALLY, both flagged here and in the final report rather than
 * silently resolved:
 *
 *  1. FINAL's third word is specified as "an echo of t_sr as the
 *     initiator read it" -- but RESPONSE's payload is specified EMPTY,
 *     and t_sr is the responder's own local transmit time: nothing in
 *     this wire format ever carries it to the initiator for it to read
 *     and echo back. Sent as 0; nothing on the responder's side reads
 *     this word (see dw1000_probe_twr_resp_run() below).
 *  2. "Single-sided mode is the first two frames only" would leave the
 *     responder unable to learn t_sp or t_rr at all -- both are the
 *     INITIATOR's own local instants, and POLL/RESPONSE's payloads are
 *     both specified empty, so no frame in a literal two-frame exchange
 *     could carry them. This implementation reads "first two frames" as
 *     naming which INSTANTS the estimate uses (t_sp/t_rp/t_sr/t_rr, the
 *     pair POLL and RESPONSE each produce), not the wire frame count: a
 *     single-sided run still sends FINAL (the only frame that can carry
 *     t_sp/t_rr to the responder) and stops there, skipping REPORT.
 *
 * WHAT MOVED HERE FROM THE ZEPHYR APPLICATION (see the top-level report
 * for the fuller account): the Zephyr coupling was four things, all
 * replaced --
 *
 *  - k_mutex_lock()/unlock() on the driver bus -> dw1000_probe_port_
 *    bus_lock()/unlock(), already implemented by every port;
 *  - atomic_t/atomic_get() on the rx-frame-arrived counter (and,
 *    identically, on the tx-done counter frame_send() polls) -> plain
 *    counters, read only under the bus lock. Correct because the
 *    writer -- dw1000_probe_rx_capture()/dw1000_probe_tx_capture()
 *    below -- is called from the application's radio callbacks, which
 *    <dw1000/probe/port.h> guarantees run with the bus lock already
 *    held (inside dw1000_process_events()); a reader that also takes
 *    the lock therefore never races the writer, and atomics bought
 *    nothing an ordinary variable under that lock does not already
 *    give;
 *  - shell_print() progress lines -> deleted. The roles return counts
 *    (struct dw1000_probe_twr_resp_result / _twr_init_result); the
 *    application prints;
 *  - ARG_UNUSED() -> (void)x;.
 *
 * The rx capture buffer itself (struct probe_rx_frame and the counter,
 * both previously in the Zephyr application's src/main.c) moved here
 * too, as dw1000_probe_rx_capture() below: it is parsed by frame_wait(),
 * which belongs with the rest of the exchange's mechanism, not with
 * bring-up. What stayed in the application is the *callback* --
 * dw1000_cb_rx_ok(), registered in dw1000_config_t.cb.rx_ok -- because
 * attaching a callback into the driver's bring-up config is an
 * application concern; it now does nothing but call
 * dw1000_probe_rx_capture() and re-arm the receiver. The same split
 * applies to tx completion: dw1000_probe_tx_capture() is new here,
 * replacing the atomic counter frame_send() used to poll; the
 * application's own tx-done counter (used by a `probe tx` command that
 * has nothing to do with this exchange) is untouched and kept entirely
 * separate -- the two counters share a callback, not a variable.
 *
 * The per-run line buffering that <dw1000/probe/port.h> requires
 * (dw1000_probe_port_emit() called "only between runs") stays a host
 * decision for the same memory-budget reason the rx capture size does
 * not: dw1000_probe_twr_resp_run() takes an @p emit_line callback
 * rather than owning a buffer of its own, so a RAM-constrained
 * application can hand it the same static buffer it already reuses for
 * `probe tx` / `probe rx`, instead of this library doubling that
 * footprint.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <dw1000/dw1000.h>
#include <dw1000/probe/role.h>
#include <dw1000/probe/record.h>
#include <dw1000/probe/port.h>
#include <dw1000/probe/exchange.h>


/*======================================================================*/
/* Wire frame                                                           */
/*======================================================================*/

#define FRAME_TYPE_POLL     'P'
#define FRAME_TYPE_RESPONSE 'R'
#define FRAME_TYPE_FINAL    'F'
#define FRAME_TYPE_REPORT   'T'

#define FRAME_HDR_LEN  12
#define FRAME_WORD_LEN 5

/* 12-byte header + 5*5 REPORT payload, the widest frame this instrument
 * sends, plus a little slack; the 2-byte CRC the chip appends lands past
 * whatever a parser reads. */
#define FRAME_MAX 40

/* Wildcard for frame_wait()'s expected_seq: legal only while waiting for
 * a POLL, the first frame of a new exchange, whose seq the responder
 * cannot know ahead of time -- it is the initiator's to pick. */
#define FRAME_SEQ_ANY (-1)

static void
put_u40le(uint8_t *p, uint64_t v)
{
    int i;

    for (i = 0; i < FRAME_WORD_LEN; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}

static uint64_t
get_u40le(const uint8_t *p)
{
    uint64_t v = 0;
    int i;

    for (i = 0; i < FRAME_WORD_LEN; i++)
        v |= (uint64_t)p[i] << (8 * i);
    return v;
}

static void
put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
}

static uint16_t
get_u16le(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static size_t
frame_build_header(uint8_t *buf, char type, uint8_t seq, uint16_t dst, uint16_t src)
{
    buf[0] = 'd'; buf[1] = 'w'; buf[2] = 'p';
    buf[3] = (uint8_t)type;
    buf[4] = seq;
    put_u16le(&buf[5], dst);
    put_u16le(&buf[7], src);
    buf[9] = buf[10] = buf[11] = 0;
    return FRAME_HDR_LEN;
}

/* Why a captured frame was not the one being waited for. Every
 * condition has to hold for a frame to be ours, so which one gets the
 * blame cannot change whether a frame is accepted: the order below is a
 * reporting choice alone, each reason narrower than the one before it,
 * so that the first that fits is the informative one. */
enum frame_verdict {
    FRAME_MATCH = 0,
    FRAME_DROP_FOREIGN,     /* no "dwp" mark: somebody else's traffic    */
    FRAME_DROP_SHORT,       /* ours, too short for the awaited payload   */
    FRAME_DROP_TYPE,        /* ours, the wrong kind of frame             */
    FRAME_DROP_DST,         /* ours, addressed to another node           */
    FRAME_DROP_SEQ,         /* ours, a different exchange                */
    FRAME_VERDICT__COUNT
};

/* See the frame-matching predicate in the file comment above.
 * @p min_words bounds how much payload must actually be present for the
 * type being waited for (0 for POLL/RESPONSE, 3 for FINAL, 5 for
 * REPORT), so a truncated or corrupt frame that happens to pass the
 * header checks is not read past what it actually carries.
 */
static enum frame_verdict
frame_classify(const uint8_t *buf, size_t len, char want_type,
              int expected_seq, uint16_t own_addr, size_t min_words)
{
    /* The mark is asked for before the length, so that a runt of
     * somebody else's traffic is reported as foreign rather than as a
     * short frame of ours. Its own guard is the three bytes it reads. */
    if (len < 3 || buf[0] != 'd' || buf[1] != 'w' || buf[2] != 'p')
        return FRAME_DROP_FOREIGN;
    if (len < FRAME_HDR_LEN + min_words * FRAME_WORD_LEN)
        return FRAME_DROP_SHORT;
    if ((char)buf[3] != want_type)
        return FRAME_DROP_TYPE;
    if (get_u16le(&buf[5]) != own_addr)
        return FRAME_DROP_DST;
    if (expected_seq != FRAME_SEQ_ANY && buf[4] != (uint8_t)expected_seq)
        return FRAME_DROP_SEQ;
    return FRAME_MATCH;
}


/*======================================================================*/
/* Timing                                                                */
/*======================================================================*/

/* Default per-frame wait: ample at bench ranges (brief's own figure). A
 * constant, not a magic number, because it is a deliberate budget, not a
 * measured one. Passed straight to dw1000_rx_set_timeout(), whose unit
 * is "UWB microseconds" (~1.0256 standard us, per the DW1000 UM) --
 * close enough to a plain microsecond at this magnitude that no
 * conversion is applied. */
#define PROBE_EXCHANGE_FRAME_TIMEOUT_US 20000U

/* Gap between exchanges: a short pause so the initiator does not hammer
 * the channel back to back. */
#define PROBE_EXCHANGE_GAP_US 20000U

/* How long a responder waits for the FIRST POLL of a run. Generous,
 * because the gap it has to cover is the harness's and not the radio's:
 * the responder is started first, and the initiator follows after a
 * fixed head start the harness states rather than detects (12 s on this
 * bench). It cannot detect it, because the READY line a poller would
 * look for does not reach the log until the session ends; see
 * dw1000_probe_ready_format() in <dw1000/probe/record.h>. Thirty seconds
 * covers that head start with room to spare, and still ends a run that
 * nobody ever answered. */
#if !defined(PROBE_EXCHANGE_FIRST_POLL_TIMEOUT_US)
#define PROBE_EXCHANGE_FIRST_POLL_TIMEOUT_US 30000000U
#endif

/* And for every POLL after the first, by which time the initiator is
 * known to be running: its own gap between exchanges is 20 ms plus at
 * most four frame budgets of its own, so two seconds is two orders of
 * margin. What it buys is that an initiator which stops mid-run leaves a
 * short tail of no-poll records and a STATS line, instead of a responder
 * that never returns.
 *
 * Both are overridable at compile time, and both defaults are a harness
 * property rather than a radio one -- a bench with a different head
 * start wants a different first budget. tests/probe/exchange.c overrides
 * them to milliseconds, which is the only way a gate can prove a wait
 * ends without waiting out the wait. */
#if !defined(PROBE_EXCHANGE_POLL_TIMEOUT_US)
#define PROBE_EXCHANGE_POLL_TIMEOUT_US 2000000U
#endif

/* How long to wait for a transmit's completion to be reported before
 * giving up on reading its RMARKER time -- a frame this short is on air
 * well under a millisecond, so this is generous slack for the
 * application's event thread/loop to run, not a measured bound. */
#define PROBE_EXCHANGE_TX_SETTLE_TIMEOUT_US 50000U

/* How often frame_wait() re-checks the capture buffer and the clock.
 * Coarse enough not to spin the bus lock pointlessly, fine enough that
 * it does not eat a meaningful slice of a 20 ms frame budget. */
#define PROBE_EXCHANGE_POLL_INTERVAL_US 500U


/*======================================================================*/
/* Capture -- filled by the application's radio event callbacks         */
/*======================================================================*/

/* Both counters are plain, not atomic: the only writer is the
 * application's radio callback, called from inside
 * dw1000_process_events() -- which <dw1000/probe/port.h> guarantees
 * runs with the bus lock held -- and every reader below takes that same
 * lock before looking at either counter or the frame it guards. See the
 * file comment above for why this replaces what was an atomic_t under
 * Zephyr. */

struct rx_capture {
    uint8_t  data[FRAME_MAX];
    size_t   length;             /* as reported by the driver, CRC included */
    uint64_t rx_time;            /* dw1000_rx_get_rmarker_time(), at capture */
    double   power_signal_dbm;   /* dw1000_rx_get_power_estimate() signal    */
    double   power_firstpath_dbm;/* ... and first-path, both at capture      */
};

static struct rx_capture rx_frame;
static uint32_t          rx_frame_seq;
static uint32_t          tx_done_seq;

void
dw1000_probe_rx_capture(dw1000_t *dw, size_t length)
{
    size_t copy_len = length < sizeof(rx_frame.data) ? length : sizeof(rx_frame.data);

    dw1000_rx_read_frame_data(dw, rx_frame.data, copy_len, 0);
    rx_frame.length  = length;
    rx_frame.rx_time = dw1000_rx_get_rmarker_time(dw);
    dw1000_rx_get_power_estimate(dw, &rx_frame.power_signal_dbm,
                                 &rx_frame.power_firstpath_dbm);
    rx_frame_seq++;
}

void
dw1000_probe_tx_capture(void)
{
    tx_done_seq++;
}


/*======================================================================*/
/* The reception account                                                */
/*======================================================================*/

/* What became of every frame the callback captured during a run, kept so
 * that a run which resolved nothing can still say why. Reported on the
 * STATS line; <dw1000/probe/record.h> states what partitions what.
 *
 * Plain statics with no lock of their own, unlike the capture above:
 * every write is on the role's own thread, none is in a callback. They
 * follow rx_frame_seq, which IS written in a callback, and each read of
 * it below takes the bus lock as every other reader does. */
static struct {
    uint16_t unwatched;
    uint16_t overrun;
    uint16_t drop[FRAME_VERDICT__COUNT];  /* indexed by enum frame_verdict */
} heard;

static uint32_t heard_start;    /* rx_frame_seq when the run began       */
static uint32_t heard_upto;     /* how far the account has followed it   */

/* Saturating, because a count that has wrapped is worse than useless and
 * 65535 frames rejected already says everything a larger number would. */
static void
heard_bump(uint16_t *counter, uint32_t by)
{
    uint32_t v = (uint32_t)*counter + by;

    *counter = v > 65535u ? 65535u : (uint16_t)v;
}

static void
heard_reset(void)
{
    memset(&heard, 0, sizeof(heard));

    dw1000_probe_port_bus_lock();
    heard_start = rx_frame_seq;
    dw1000_probe_port_bus_unlock();
    heard_upto = heard_start;
}


/*======================================================================*/
/* Send / wait                                                          */
/*======================================================================*/

/* Turn the receiver on (or off, for timeout_us == 0: dw1000_rx_set_timeout()
 * treats that as "disable", so the receiver listens with no chip-side
 * bound at all -- used only for the responder's wait for the next POLL,
 * whose budget is seconds and does not fit the chip's 16-bit microsecond
 * field, which tops out at 65 ms. That wait is bounded, and bounded on
 * the host: frame_wait()'s deadline is what ends it. The comment that
 * used to stand here said the wait had no deadline because the bench
 * waits for READY before starting the initiator. The bench does not and
 * cannot -- see dw1000_probe_ready_format() in <dw1000/probe/record.h> --
 * and the consequence of believing it was a responder that hung with
 * nothing printed rather than reporting what it had failed to hear.
 * See dw1000_probe_twr_resp_run() below).
 * Always starts from IDLE via dw1000_txrx_off(), which
 * dw1000_rx_set_timeout() requires (see its own doc comment) -- so a
 * caller never has to reason about what a previous wait or transmit left
 * the chip in. */
static void
rx_arm(dw1000_t *dw, uint32_t timeout_us)
{
    uint32_t bounded = timeout_us > 65535U ? 65535U : timeout_us;

    dw1000_probe_port_bus_lock();
    dw1000_txrx_off(dw);
    dw1000_rx_set_timeout(dw, (uint16_t)bounded);
    dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
    dw1000_probe_port_bus_unlock();
}

/* Send one frame and, on success, return its RMARKER transmit time.
 * Turns the receiver off first (a caller may have left it armed from a
 * previous wait) so the transmitter is never racing an active receiver.
 *
 * @return false if the transmit itself could not be started, or its
 *         completion was not reported within the settle window -- the
 *         caller's NO_RESPONSE-shaped statuses all come from here.
 */
static bool
frame_send(dw1000_t *dw, const uint8_t *buf, size_t len, uint64_t *tx_time,
           bool expect_reply)
{
    uint8_t  txbuf[FRAME_MAX];
    uint32_t tx_done_before;
    bool     started;
    dw1000_probe_time_t deadline;

    memcpy(txbuf, buf, len);

    dw1000_probe_port_bus_lock();
    tx_done_before = tx_done_seq;
    dw1000_txrx_off(dw);
    dw1000_tx_write_frame_data(dw, txbuf, len, 0);
    /* When a reply is expected the CHIP arms the receiver at the end of
     * this transmission (WAIT4RESP), with the frame timeout set here,
     * before TXSTRT -- nothing in software is in that path. It used to
     * be: wait for tx_done through the event thread, then rx_arm(); and a
     * peer answering inside that turnaround was never heard. A Raspberry
     * Pi answers a POLL in 0.92 ms, the nRF52 boards in 1.49 ms, which is
     * why board-to-board worked and board-to-Pi did not. The driver, the
     * port, the wiring and the RF were each suspected before the harness
     * measuring them was. */
    int mode = DW1000_TX_IMMEDIATE | (expect_reply ? DW1000_TX_RESPONSE_EXPECTED : 0);
    if (expect_reply) {
        uint32_t t = PROBE_EXCHANGE_FRAME_TIMEOUT_US;
        dw1000_rx_set_timeout(dw, (uint16_t)(t > 65535U ? 65535U : t));
    }
    dw1000_tx_fctrl(dw, len + 2, 0, mode);
    started = dw1000_tx_start(dw, mode) == 0;
    dw1000_probe_port_bus_unlock();

    if (!started)
        return false;

    deadline = dw1000_probe_port_now() + PROBE_EXCHANGE_TX_SETTLE_TIMEOUT_US;
    for (;;) {
        uint32_t cur;

        dw1000_probe_port_bus_lock();
        cur = tx_done_seq;
        dw1000_probe_port_bus_unlock();

        if (cur - tx_done_before >= 1)
            break;
        if (dw1000_probe_port_now() >= deadline)
            return false;
        dw1000_probe_port_sleep(PROBE_EXCHANGE_POLL_INTERVAL_US);
    }

    dw1000_probe_port_bus_lock();
    *tx_time = dw1000_tx_get_rmarker_time(dw);
    dw1000_probe_port_bus_unlock();
    return true;
}

/* Wait for one frame of type @p want_type, matching @p expected_seq (or
 * FRAME_SEQ_ANY) and addressed to @p own_addr, until @p deadline
 * (dw1000_probe_port_now()-based -- the host-side backstop). Anything
 * else captured meanwhile is not ours and the wait continues: the
 * application's rx_ok/rx_timeout/rx_error callbacks already re-arm the
 * receiver on every event, including a filtered-out frame or an
 * on-chip timeout, so this loop only watches the clock and the capture
 * buffer -- it never re-enables the receiver itself.
 */
static bool
frame_wait(char want_type, int expected_seq, uint16_t own_addr, size_t min_words,
          dw1000_probe_time_t deadline, struct rx_capture *out)
{
    uint32_t seen;
    bool     matched = false;

    dw1000_probe_port_bus_lock();
    seen = rx_frame_seq;
    dw1000_probe_port_bus_unlock();

    /* Frames captured since the previous wait ended. The exchange is a
     * sequence of waits with gaps between them and nothing looks at the
     * capture slot in a gap, so these are gone: the slot holds only the
     * newest. They are counted rather than examined, because the count
     * is the one thing that can be said about them, and because a peer
     * answering faster than this role gets back to waiting lands here.
     * That would be this instrument losing a frame, not the link. */
    heard_bump(&heard.unwatched, seen - heard_upto);

    while (dw1000_probe_port_now() < deadline) {
        struct rx_capture snap;
        bool got = false;

        dw1000_probe_port_bus_lock();
        if (rx_frame_seq != seen) {
            /* Everything but the newest is gone for the same reason: one
             * slot, so a burst arriving between two polls of this loop
             * leaves only its last frame behind. */
            heard_bump(&heard.overrun, rx_frame_seq - seen - 1);
            seen = rx_frame_seq;
            /* Single-producer/single-consumer snapshot, taken under the
             * same lock the producer (dw1000_probe_rx_capture(), called
             * from the application's rx_ok callback) writes under -- so
             * this copy can never observe a torn write. */
            snap = rx_frame;
            got  = true;
        }
        dw1000_probe_port_bus_unlock();

        if (got) {
            enum frame_verdict verdict =
                frame_classify(snap.data, snap.length, want_type,
                               expected_seq, own_addr, min_words);

            if (verdict == FRAME_MATCH) {
                *out    = snap;
                matched = true;
                break;
            }
            heard_bump(&heard.drop[verdict], 1);
        }
        dw1000_probe_port_sleep(PROBE_EXCHANGE_POLL_INTERVAL_US);
    }

    /* Wherever it ended, the account has now followed the capture this
     * far, so the next wait counts its gap from here and not from the
     * start of the run. */
    heard_upto = seen;
    return matched;
}


/*======================================================================*/
/* twr_resp                                                             */
/*======================================================================*/

struct dw1000_probe_twr_resp_result
dw1000_probe_twr_resp_run(dw1000_t *dw, long count, bool ss,
                          uint16_t own_addr, uint16_t peer_addr,
                          const char *node_name,
                          void (*emit_line)(const char *line))
{
    (void)peer_addr; /* RESPONSE answers whoever sent the POLL, not a
                       * fixed configured peer -- see the report. */

    const struct dw1000_probe_origin origin = {
        .node = node_name,
        .role = DW1000_PROBE_ROLE_TWR_RESP,
        .run  = NULL,
    };

    heard_reset();

    char line[DW1000_PROBE_RECORD_MAX];
    dw1000_probe_ready_format(line, sizeof(line), &origin);
    dw1000_probe_port_emit(line); /* safe here: no exchange is in flight yet */

    struct dw1000_probe_twr_resp_result result = { 0, 0 };
    long i;

    for (i = 0; i < count; i++) {
        struct dw1000_probe_record rec;
        memset(&rec, 0, sizeof(rec));
        rec.seq      = (uint16_t)i;
        rec.exchange = ss ? DW1000_PROBE_EXCHANGE_SS : DW1000_PROBE_EXCHANGE_DS;

        /* Wait for the next POLL, bounded, and the bound is the whole
         * point of it. This wait used to be given UINT64_MAX on the
         * stated grounds that the bench waits for READY before starting
         * the initiator; the bench does not and cannot, and what the
         * false premise bought was a responder that could not end. A
         * board that heard no POLL sat here until its console session
         * was cut, having emitted no record and no STATS -- silence,
         * where a run of no-poll records would have said which end had
         * gone deaf and what, if anything, it was hearing instead.
         *
         * The first attempt gets the long budget because the gap it
         * covers is the harness's head start; every attempt after it
         * gets the short one, the initiator being known to be running by
         * then. Both are stated at the top of this file. */
        rx_arm(dw, 0);
        struct rx_capture poll;
        dw1000_probe_time_t poll_deadline = dw1000_probe_port_now() +
            (i == 0 ? PROBE_EXCHANGE_FIRST_POLL_TIMEOUT_US
                    : PROBE_EXCHANGE_POLL_TIMEOUT_US);
        bool got_poll = frame_wait(FRAME_TYPE_POLL, FRAME_SEQ_ANY, own_addr, 0,
                                   poll_deadline, &poll);

        result.attempted++;

        /* Read whatever happened, so that <dw1000/probe/record.h>'s
         * promise -- the near end's temperature and voltage are in every
         * record -- stays true of a no-poll record too. Read here rather
         * than before the wait, so that it is the die at the attempt and
         * not the die up to thirty seconds earlier. */
        int16_t  temp;
        uint16_t vbat;
        dw1000_probe_port_bus_lock();
        dw1000_read_temp_vbat(dw, &temp, &vbat);
        dw1000_probe_port_bus_unlock();
        rec.resp_temp = temp;
        rec.resp_vbat = vbat;
        rec.present  |= DW1000_PROBE_F_RESP_TEMP | DW1000_PROBE_F_RESP_VBAT;

        if (!got_poll) {
            /* Nothing arrived. The STATS line at the end of the run is
             * where a reader learns whether that means nothing was
             * heard at all, or frames were heard and rejected. */
            rec.status = DW1000_PROBE_STATUS_NO_POLL;
        } else {
            rec.status = DW1000_PROBE_STATUS_NO_RESPONSE;

            uint8_t  wire_seq  = poll.data[4];
            uint16_t poll_src  = get_u16le(&poll.data[7]);

            rec.t_rp     = poll.rx_time;
            rec.present |= DW1000_PROBE_F_T_RP;

            uint16_t packed;
            if (dw1000_probe_pack_power(poll.power_signal_dbm, &packed)) {
                rec.resp_rx  = packed;
                rec.present |= DW1000_PROBE_F_RESP_RX;
            }
            if (dw1000_probe_pack_power(poll.power_firstpath_dbm, &packed)) {
                rec.resp_fp  = packed;
                rec.present |= DW1000_PROBE_F_RESP_FP;
            }

            /* RESPONSE, addressed back to whoever sent the POLL. */
            uint8_t resp_buf[FRAME_HDR_LEN];
            frame_build_header(resp_buf, FRAME_TYPE_RESPONSE, wire_seq,
                               poll_src, own_addr);

            uint64_t t_sr;
            if (frame_send(dw, resp_buf, sizeof(resp_buf), &t_sr, true)) {
                rec.t_sr     = t_sr;
                rec.present |= DW1000_PROBE_F_T_SR;
                rec.status   = DW1000_PROBE_STATUS_NO_FINAL;

                struct rx_capture final;
                dw1000_probe_time_t final_deadline =
                    dw1000_probe_port_now() + PROBE_EXCHANGE_FRAME_TIMEOUT_US;

                if (frame_wait(FRAME_TYPE_FINAL, wire_seq, own_addr, 3,
                              final_deadline, &final)) {
                    rec.t_sp = get_u40le(&final.data[FRAME_HDR_LEN + 0 * FRAME_WORD_LEN]);
                    rec.t_rr = get_u40le(&final.data[FRAME_HDR_LEN + 1 * FRAME_WORD_LEN]);
                    /* word 2, the t_sr echo, is not read here: see the file
                     * comment above -- nothing on this driver's side ever
                     * has a genuine value to check it against. */
                    rec.present |= DW1000_PROBE_F_T_SP | DW1000_PROBE_F_T_RR;

                    if (ss) {
                        dw1000_probe_distances(&rec);
                        rec.status = DW1000_PROBE_STATUS_OK;
                    } else {
                        rec.t_rf     = final.rx_time;
                        rec.present |= DW1000_PROBE_F_T_RF;
                        rec.status   = DW1000_PROBE_STATUS_NO_REPORT;

                        struct rx_capture report;
                        rx_arm(dw, PROBE_EXCHANGE_FRAME_TIMEOUT_US);
                        dw1000_probe_time_t report_deadline =
                            dw1000_probe_port_now() + PROBE_EXCHANGE_FRAME_TIMEOUT_US;

                        if (frame_wait(FRAME_TYPE_REPORT, wire_seq, own_addr, 5,
                                      report_deadline, &report)) {
                            rec.t_sf = get_u40le(&report.data[FRAME_HDR_LEN + 0 * FRAME_WORD_LEN]);
                            rec.init_temp = (int16_t)get_u40le(
                                &report.data[FRAME_HDR_LEN + 1 * FRAME_WORD_LEN]);
                            rec.init_vbat = (uint16_t)get_u40le(
                                &report.data[FRAME_HDR_LEN + 2 * FRAME_WORD_LEN]);
                            rec.init_rx = (uint16_t)get_u40le(
                                &report.data[FRAME_HDR_LEN + 3 * FRAME_WORD_LEN]);
                            rec.init_fp = (uint16_t)get_u40le(
                                &report.data[FRAME_HDR_LEN + 4 * FRAME_WORD_LEN]);
                            rec.present |= DW1000_PROBE_F_T_SF | DW1000_PROBE_F_INIT_TEMP |
                                           DW1000_PROBE_F_INIT_VBAT | DW1000_PROBE_F_INIT_RX |
                                           DW1000_PROBE_F_INIT_FP;

                            if (dw1000_probe_distances(&rec)) {
                                bool have_sym = (rec.present & DW1000_PROBE_F_SYM) != 0;
                                rec.status = have_sym ? DW1000_PROBE_STATUS_OK
                                                       : DW1000_PROBE_STATUS_BAD_DISTANCE;
                            }
                        } else {
                            /* NO_REPORT: the single-sided estimate is still
                             * recoverable from the four instants that did
                             * arrive. */
                            dw1000_probe_distances(&rec);
                        }
                    }
                }
            }
        }

        if (rec.status == DW1000_PROBE_STATUS_OK)
            result.resolved++;

        dw1000_probe_record_format(line, sizeof(line), &rec, &origin);
        emit_line(line);
    }

    /* Stop listening. */
    uint32_t txpower_reg;
    uint32_t captured;

    dw1000_probe_port_bus_lock();
    dw1000_txrx_off(dw);
    /* The tail of the run: anything captured after the last wait ended
     * is unwatched exactly as any other gap is, and counting it here is
     * what makes the account add up to the frames delivered. */
    heard_bump(&heard.unwatched, rx_frame_seq - heard_upto);
    heard_upto  = rx_frame_seq;
    captured    = rx_frame_seq - heard_start;
    txpower_reg = dw1000_tx_get_power(dw);
    dw1000_probe_port_bus_unlock();

    struct dw1000_probe_stats stats;
    memset(&stats, 0, sizeof(stats));
    stats.tx_power_db_x10     = (int16_t)(dw1000_tx_power_to_05db(txpower_reg) * 5u);
    stats.tx_power_req_db_x10 = -1; /* DW1000_TX_POWER_AUTO -- see the
                                      * application's bring-up          */
    stats.driver_version      = DW1000_VERSION_FULL;
    stats.attempted            = result.attempted;
    stats.resolved             = result.resolved;

    stats.heard          = captured > 65535u ? 65535u : (uint16_t)captured;
    stats.drop_unwatched = heard.unwatched;
    stats.drop_overrun   = heard.overrun;
    stats.drop_foreign   = heard.drop[FRAME_DROP_FOREIGN];
    stats.drop_short     = heard.drop[FRAME_DROP_SHORT];
    stats.drop_type      = heard.drop[FRAME_DROP_TYPE];
    stats.drop_dst       = heard.drop[FRAME_DROP_DST];
    stats.drop_seq       = heard.drop[FRAME_DROP_SEQ];

    dw1000_probe_stats_format(line, sizeof(line), &stats, &origin);
    emit_line(line);

    return result;
}


/*======================================================================*/
/* twr_init                                                             */
/*======================================================================*/

struct dw1000_probe_twr_init_result
dw1000_probe_twr_init_run(dw1000_t *dw, long count, bool ss, long warmup,
                          uint16_t own_addr, uint16_t peer_addr)
{
    uint8_t  wire_seq  = 0;
    long     total     = warmup + count;
    struct dw1000_probe_twr_init_result result = { 0, 0, 0, 0, 0 };
    long i;

    /* The initiator has no STATS line to report an account on, so these
     * counts go nowhere. Reset anyway: leaving them would have a later
     * responder run on the same node report this run's frames as its
     * own. */
    heard_reset();

    for (i = 0; i < total; i++) {
        bool counted = (i >= warmup);
        if (counted)
            result.attempted++;

        uint8_t poll_buf[FRAME_HDR_LEN];
        frame_build_header(poll_buf, FRAME_TYPE_POLL, wire_seq, peer_addr, own_addr);

        uint64_t t_sp;
        bool got_final = false;

        if (frame_send(dw, poll_buf, sizeof(poll_buf), &t_sp, true)) {
            dw1000_probe_time_t deadline =
                dw1000_probe_port_now() + PROBE_EXCHANGE_FRAME_TIMEOUT_US;

            struct rx_capture response;
            if (frame_wait(FRAME_TYPE_RESPONSE, wire_seq, own_addr, 0,
                          deadline, &response)) {
                dw1000_probe_time_t t_after_final = 0;
                uint64_t t_rr = response.rx_time;

                uint8_t final_buf[FRAME_HDR_LEN + 3 * FRAME_WORD_LEN];
                frame_build_header(final_buf, FRAME_TYPE_FINAL, wire_seq,
                                   peer_addr, own_addr);
                put_u40le(&final_buf[FRAME_HDR_LEN + 0 * FRAME_WORD_LEN], t_sp);
                put_u40le(&final_buf[FRAME_HDR_LEN + 1 * FRAME_WORD_LEN], t_rr);
                /* word 2: nothing genuine to echo -- see the file comment
                 * above. Sent as 0. */
                put_u40le(&final_buf[FRAME_HDR_LEN + 2 * FRAME_WORD_LEN], 0);

                uint64_t t_sf;
                if (frame_send(dw, final_buf, sizeof(final_buf), &t_sf, false)) {
                    got_final = true;
                    t_after_final = dw1000_probe_port_now();

                    if (!ss) {
                        int16_t  temp;
                        uint16_t vbat;
                        dw1000_probe_port_bus_lock();
                        dw1000_read_temp_vbat(dw, &temp, &vbat);
                        dw1000_probe_port_bus_unlock();

                        uint16_t init_rx = 0, init_fp = 0;
                        (void)dw1000_probe_pack_power(response.power_signal_dbm, &init_rx);
                        (void)dw1000_probe_pack_power(response.power_firstpath_dbm, &init_fp);

                        uint8_t report_buf[FRAME_HDR_LEN + 5 * FRAME_WORD_LEN];
                        frame_build_header(report_buf, FRAME_TYPE_REPORT, wire_seq,
                                          peer_addr, own_addr);
                        put_u40le(&report_buf[FRAME_HDR_LEN + 0 * FRAME_WORD_LEN], t_sf);
                        put_u40le(&report_buf[FRAME_HDR_LEN + 1 * FRAME_WORD_LEN],
                                 (uint64_t)(uint16_t)temp);
                        put_u40le(&report_buf[FRAME_HDR_LEN + 2 * FRAME_WORD_LEN], vbat);
                        put_u40le(&report_buf[FRAME_HDR_LEN + 3 * FRAME_WORD_LEN], init_rx);
                        put_u40le(&report_buf[FRAME_HDR_LEN + 4 * FRAME_WORD_LEN], init_fp);

                        /* Tested, where it once was not. Every other
                         * send in this exchange is checked; this one was
                         * cast to void, so a REPORT that never left the
                         * chip looked exactly like one the peer failed to
                         * hear -- the responder said `no-report` and the
                         * initiator still counted the exchange as having
                         * reached REPORT. Measured 2026-09-16 on a Unix
                         * initiator against a board responder: 10 of 20
                         * exchanges "reached REPORT" while the responder
                         * heard 38 frames and matched no REPORT at all. */
                        /* The interval the exchange has a minimum on.
                         * See turnaround_min_us in <probe/exchange.h>:
                         * an initiator quicker than the responder's own
                         * turnaround loses every REPORT, and nothing
                         * else on either side reports why. */
                        uint32_t turn = (uint32_t)(dw1000_probe_port_now()
                                                   - t_after_final);
                        if (result.turnaround_max_us == 0 ||
                            turn > result.turnaround_max_us)
                            result.turnaround_max_us = turn;
                        if (result.turnaround_min_us == 0 ||
                            turn < result.turnaround_min_us)
                            result.turnaround_min_us = turn;

                        uint64_t unused_tx_time;
                        if (!frame_send(dw, report_buf, sizeof(report_buf),
                                        &unused_tx_time, false))
                            result.report_failed++;
                    }
                }
            }
        }

        if (counted && got_final)
            result.reached++;

        wire_seq++;

        if (i + 1 < total)
            dw1000_probe_port_sleep(PROBE_EXCHANGE_GAP_US);
    }

    dw1000_probe_port_bus_lock();
    dw1000_txrx_off(dw);
    dw1000_probe_port_bus_unlock();

    return result;
}
