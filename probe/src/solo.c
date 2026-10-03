/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * The roles a node runs on its own (tx, rx, temperature), and what it
 * reports about itself. See <dw1000/probe/solo.h> for the contract.
 *
 * WHERE THESE CAME FROM. All three were written as shell commands in the
 * Zephyr application (zephyr-redskin/probe/src/shell.c, `probe tx`,
 * `probe rx`, `probe temp`), so the Raspberry Pi application did not
 * have them, and "the same instrument on both platforms" was true of the
 * exchange alone. They are mechanism by probe/DESIGN.md's split line, so
 * they moved here, and the two shells are left choosing values. Three
 * things changed on the way, each stated where it lives below: tx paces
 * its frames on an absolute schedule rather than from the end of each
 * send; a sample that falls due late is taken once rather than once per
 * interval it missed (the shell version emitted several identical-time
 * lines); and a gap between frames longer than the sampling interval no
 * longer suspends the sampling.
 *
 * Counting is the capture's (exchange.c, read through capture.h): a
 * role snapshots the monotonic counts when it starts and takes the
 * difference when it ends, so nothing is reset and no role can disturb
 * another's account.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <dw1000/dw1000.h>
#include <dw1000/probe/role.h>
#include <dw1000/probe/record.h>
#include <dw1000/probe/port.h>
#include <dw1000/probe/settle.h>
#include <dw1000/probe/solo.h>

#include "capture.h"


/*======================================================================*/
/* Timing                                                               */
/*======================================================================*/

/* How often a role's loop wakes to look at the clock, double buffered:
 * sampling is all it watches, and a sample owed is taken at most this
 * late. */
#define SOLO_TICK_US        50000U

/* Single buffered, the rx role re-arms the receiver itself after every
 * frame (see dw1000_probe_rx_run() in solo.h), so its loop turns at the
 * exchange's own frame-watching rate: the receiver is off for at most
 * this long per frame. */
#define SOLO_TICK_REARM_US    500U

/* After the tx role's last frame, how long its completions get to be
 * reported. A frame this short is on the air well under a millisecond,
 * so this is slack for the event thread, not a measured bound. */
#define SOLO_TX_SETTLE_US  100000U

/* Content does not matter, only that a frame goes out: eight bytes and
 * the two of CRC the chip appends. Not const, because
 * dw1000_tx_write_frame_data() takes a non-const buffer. */
static uint8_t tx_payload[8] = { 'P', 'R', 'O', 'B', 'E', 'T', 'X', '!' };


/*======================================================================*/
/* Sampling                                                             */
/*======================================================================*/

struct sampler {
    dw1000_t                   *dw;
    dw1000_probe_time_t         start;
    dw1000_probe_time_t         next;
    uint32_t                    interval_us;
    struct dw1000_probe_origin  origin;
    void                      (*emit_line)(const char *line);
    uint32_t                    lines;
};

size_t
dw1000_probe_sampled_lines(uint32_t seconds, uint32_t interval_ms)
{
    return (size_t)((uint64_t)seconds * 1000u / interval_ms) + 2;
}

static uint32_t
elapsed_ds(const struct sampler *s, dw1000_probe_time_t now)
{
    return (uint32_t)((now - s->start) / 100000u);
}

static void
read_die(dw1000_t *dw, int16_t *temp, uint16_t *vbat)
{
    dw1000_probe_port_bus_lock();
    dw1000_read_temp_vbat(dw, temp, vbat);
    dw1000_probe_port_bus_unlock();
}

static void
sample_emit(struct sampler *s, dw1000_probe_time_t now,
            int16_t temp, uint16_t vbat)
{
    char line[DW1000_PROBE_RECORD_MAX];

    dw1000_probe_temp_format(line, sizeof(line), elapsed_ds(s, now),
                             temp, vbat, &s->origin);
    s->emit_line(line);
    s->lines++;
}

/* Takes and emits the opening sample, at elapsed 0. */
static void
sampler_start(struct sampler *s, dw1000_t *dw, uint32_t interval_ms,
              dw1000_probe_role_t role, const char *node_name,
              void (*emit_line)(const char *line))
{
    int16_t  temp;
    uint16_t vbat;

    s->dw          = dw;
    s->interval_us = interval_ms * 1000u;
    s->origin.node = node_name;
    s->origin.role = role;
    s->origin.run  = NULL;
    s->emit_line   = emit_line;
    s->lines       = 0;

    read_die(dw, &temp, &vbat);
    s->start = dw1000_probe_port_now();
    s->next  = s->start + s->interval_us;
    sample_emit(s, s->start, temp, vbat);
}

/* A reading if one is due, not emitted: the caller decides whether it
 * becomes a TEMP line or feeds the settle rule. One reading however many
 * intervals have passed, so that every line's elapsed time is when it was
 * read; the intervals it missed are skipped, not made up. */
static bool
sampler_due(struct sampler *s, dw1000_probe_time_t *at,
            int16_t *temp, uint16_t *vbat)
{
    dw1000_probe_time_t now = dw1000_probe_port_now();

    if (now < s->next)
        return false;

    read_die(s->dw, temp, vbat);
    *at = now;
    do {
        s->next += s->interval_us;
    } while (s->next <= now);
    return true;
}

static void
sampler_end(struct sampler *s)
{
    int16_t  temp;
    uint16_t vbat;

    read_die(s->dw, &temp, &vbat);
    sample_emit(s, dw1000_probe_port_now(), temp, vbat);
}

/* Sleep until @p deadline, emitting TEMP lines as they fall due. */
static void
sampler_wait_until(struct sampler *s, dw1000_probe_time_t deadline)
{
    for (;;) {
        dw1000_probe_time_t now = dw1000_probe_port_now();
        dw1000_probe_time_t at;
        int16_t  temp;
        uint16_t vbat;

        if (sampler_due(s, &at, &temp, &vbat)) {
            sample_emit(s, at, temp, vbat);
            now = dw1000_probe_port_now();
        }
        if (now >= deadline)
            return;

        uint64_t left = deadline - now;
        dw1000_probe_port_sleep(left < SOLO_TICK_US ? (uint32_t)left
                                                    : SOLO_TICK_US);
    }
}


/*======================================================================*/
/* tx                                                                   */
/*======================================================================*/

struct dw1000_probe_tx_result
dw1000_probe_tx_run(dw1000_t *dw, long count, uint32_t gap_us,
                    uint32_t interval_ms, const char *node_name,
                    void (*emit_line)(const char *line))
{
    struct dw1000_probe_tx_result result = { 0, 0, 0 };
    struct sampler s;
    uint32_t tx_before;
    long     i;

    /* The receiver off, and kept off: this role heats the far end. */
    dw1000_probe_port_bus_lock();
    dw1000_txrx_off(dw);
    tx_before = _dw1000_probe_tx_count();
    dw1000_probe_port_bus_unlock();

    sampler_start(&s, dw, interval_ms, DW1000_PROBE_ROLE_TX,
                  node_name, emit_line);

    /* Paced on an ABSOLUTE schedule, frame i at the start plus i gaps,
     * never "send, then sleep the gap". A send takes a moment of its own,
     * different on every host, so relative pacing gives two transmitters
     * periods a fraction of a millisecond apart, and they drift into each
     * other within a few dozen frames: DW1000.md, "Two bursts paced from
     * the end of each send drift into each other". The shell version of
     * this role paced relatively, and PROPAGATE.md carried it as owed. A
     * frame whose slot has already passed goes at once, and the next
     * keeps its own slot: lateness is not carried forward. */
    dw1000_probe_time_t t0 = dw1000_probe_port_now();

    for (i = 0; i < count; i++) {
        bool ok;

        /* Slept through the sampler, so a gap longer than the sampling
         * interval does not suspend the sampling. */
        sampler_wait_until(&s, t0 + (dw1000_probe_time_t)i * gap_us);

        result.attempted++;

        /* One frame, one hold of the bus, so the event thread reports
         * completions as they happen rather than at the end. */
        dw1000_probe_port_bus_lock();
        dw1000_tx_write_frame_data(dw, tx_payload, sizeof(tx_payload), 0);
        dw1000_tx_fctrl(dw, sizeof(tx_payload) + DW1000_CRC_LENGTH, 0,
                        DW1000_TX_IMMEDIATE);
        ok = dw1000_tx_start(dw, DW1000_TX_IMMEDIATE) == 0;
        dw1000_probe_port_bus_unlock();

        if (ok)
            result.started++;
    }

    /* The last completions, before the closing sample and the count. */
    dw1000_probe_time_t deadline = dw1000_probe_port_now() + SOLO_TX_SETTLE_US;
    for (;;) {
        uint32_t done;

        dw1000_probe_port_bus_lock();
        done = _dw1000_probe_tx_count() - tx_before;
        dw1000_probe_port_bus_unlock();

        result.completed = done;
        if (done >= result.started || dw1000_probe_port_now() >= deadline)
            break;
        dw1000_probe_port_sleep(1000);
    }

    sampler_end(&s);
    return result;
}


/*======================================================================*/
/* rx                                                                   */
/*======================================================================*/

/* One body for both rx entry points: @p settle NULL is a timed run of
 * @p seconds, sampled every @p interval_ms; otherwise the rule decides
 * the end and the sampling interval. */
static struct dw1000_probe_rx_result
rx_run(dw1000_t *dw, uint32_t seconds, uint32_t interval_ms,
       const struct dw1000_probe_settle_params *settle,
       const char *node_name, void (*emit_line)(const char *line))
{
    struct dw1000_probe_rx_result result;
    struct dw1000_probe_settle rule;
    struct sampler s;
    uint32_t rx_before, err_before, seen;
    bool     rearm = !dw->config->dblbuff;
    uint32_t tick  = rearm ? SOLO_TICK_REARM_US : SOLO_TICK_US;
    int      rc;

    memset(&result, 0, sizeof(result));
    result.settle = DW1000_PROBE_SETTLE_SAMPLING;
    if (settle != NULL) {
        dw1000_probe_settle_start(&rule, settle);
        interval_ms = settle->interval_ms;
    }

    sampler_start(&s, dw, interval_ms, DW1000_PROBE_ROLE_RX,
                  node_name, emit_line);

    /* From IDLE, with no chip-side timeout: the listening is bounded by
     * this loop's clock, not by the chip's 16-bit microsecond field. */
    dw1000_probe_port_bus_lock();
    rx_before  = _dw1000_probe_rx_count();
    err_before = _dw1000_probe_rx_error_count();
    dw1000_txrx_off(dw);
    dw1000_rx_set_timeout(dw, 0);
    rc = dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
    dw1000_probe_port_bus_unlock();

    if (rc < 0) {
        result.failed = true;
        sampler_end(&s);
        return result;
    }

    seen = rx_before;
    dw1000_probe_time_t end = s.start + (dw1000_probe_time_t)seconds * 1000000u;

    for (;;) {
        dw1000_probe_time_t at;
        int16_t  temp;
        uint16_t vbat;

        if (rearm) {
            dw1000_probe_port_bus_lock();
            uint32_t now_seen = _dw1000_probe_rx_count();
            if (now_seen != seen) {
                seen = now_seen;
                dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
            }
            dw1000_probe_port_bus_unlock();
        }

        if (sampler_due(&s, &at, &temp, &vbat)) {
            if (settle == NULL) {
                sample_emit(&s, at, temp, vbat);
            } else if (dw1000_probe_settle_feed(&rule, temp)
                       != DW1000_PROBE_SETTLE_SAMPLING) {
                char line[DW1000_PROBE_RECORD_MAX];

                dw1000_probe_settle_format(line, sizeof(line), &rule,
                                           elapsed_ds(&s, at), &s.origin);
                emit_line(line);
                s.lines++;
                result.settle = rule.state;
                if (rule.state == DW1000_PROBE_SETTLE_SETTLED ||
                    rule.state == DW1000_PROBE_SETTLE_GAVE_UP)
                    break;
            }
        }
        if (settle == NULL && dw1000_probe_port_now() >= end)
            break;

        dw1000_probe_port_sleep(tick);
    }

    /* Stop listening, then count: a frame delivered after this is not
     * this run's. */
    dw1000_probe_port_bus_lock();
    dw1000_txrx_off(dw);
    result.received = _dw1000_probe_rx_count()       - rx_before;
    result.rejected = _dw1000_probe_rx_error_count() - err_before;
    dw1000_probe_port_bus_unlock();

    result.elapsed_ds = elapsed_ds(&s, dw1000_probe_port_now());
    sampler_end(&s);
    return result;
}

struct dw1000_probe_rx_result
dw1000_probe_rx_run(dw1000_t *dw, uint32_t seconds, uint32_t interval_ms,
                    const char *node_name,
                    void (*emit_line)(const char *line))
{
    return rx_run(dw, seconds, interval_ms, NULL, node_name, emit_line);
}

struct dw1000_probe_rx_result
dw1000_probe_rx_settle_run(dw1000_t *dw,
                           const struct dw1000_probe_settle_params *params,
                           const char *node_name,
                           void (*emit_line)(const char *line))
{
    return rx_run(dw, 0, params->interval_ms, params, node_name, emit_line);
}


/*======================================================================*/
/* temperature                                                          */
/*======================================================================*/

struct dw1000_probe_temperature_result
dw1000_probe_temperature_run(dw1000_t *dw, uint32_t seconds,
                             uint32_t interval_ms, const char *node_name,
                             void (*emit_line)(const char *line))
{
    struct dw1000_probe_temperature_result result;
    struct sampler s;

    /* Radio stopped: this is the baseline, and the cooling curve. */
    dw1000_probe_port_bus_lock();
    dw1000_txrx_off(dw);
    dw1000_probe_port_bus_unlock();

    sampler_start(&s, dw, interval_ms, DW1000_PROBE_ROLE_TEMPERATURE,
                  node_name, emit_line);

    if (seconds > 0) {
        sampler_wait_until(&s, s.start +
                           (dw1000_probe_time_t)seconds * 1000000u);
        sampler_end(&s);
    }

    result.samples = s.lines;
    return result;
}


/*======================================================================*/
/* What the node is                                                     */
/*======================================================================*/

size_t
dw1000_probe_info_format(char *buf, size_t len, const dw1000_t *dw)
{
    return (size_t)snprintf(buf, len,
        "--< Chip >--------------\n"
        "Device id       : 0x%08lx\n"
        "Chip id (OTP)   : 0x%08lx\n"
        "Lot id  (OTP)   : 0x%08lx\n"
        "OTP revision    : %u\n"
        "XTAL trim       : 0x%02x\n"
        "--< OTP calibration references >--\n"
        "Vbat @ 3.3V     : %u\n"
        "Vbat @ 3.7V     : %u\n"
        "Temp @ 23C      : %u\n"
        "Temp @ ant. cal.: %u\n"
        "--< Driver >------------\n"
        "Version         : %s",
        (unsigned long)dw->id.device,
        (unsigned long)dw->id.chip,
        (unsigned long)dw->id.lot,
        (unsigned)dw->otp_rev,
        (unsigned)dw->xtrim,
        (unsigned)dw->ref_vbat_33,
        (unsigned)dw->ref_vbat_37,
        (unsigned)dw->ref_temp_23,
        (unsigned)dw->ref_temp_ant,
        DW1000_VERSION_FULL);
}

size_t
dw1000_probe_power_format(char *buf, size_t len, dw1000_t *dw)
{
    uint32_t reg;
    unsigned tenths;

    dw1000_probe_port_bus_lock();
    reg = dw1000_tx_get_power(dw);
    dw1000_probe_port_bus_unlock();

    tenths = dw1000_tx_power_to_05db(reg) * 5u;
    return (size_t)snprintf(buf, len,
        "TX power: %u.%u dB (applied, read back from chip; TX_POWER=0x%08lx)",
        tenths / 10, tenths % 10, (unsigned long)reg);
}
