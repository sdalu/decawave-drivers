/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * DW1000 probe -- Linux/Raspberry Pi application.
 *
 * Where zephyr-redskin/probe's src/shell.c offers `probe twr_resp` and
 * `probe twr_init` as shell commands, this offers the same two roles as
 * one argv-driven run: bring the radio up, run the selected role once,
 * report a summary, exit. It is deliberately not a daemon -- unlike
 * rpi-redskin/main.c, which this is modelled on for the bring-up shape
 * (bitters GPIO/SPI wiring, the DW1000 pin map, the processing thread),
 * there is no protocol running underneath needing a long-lived process,
 * no reset signal, no debug channel: one role, one run, one exit code.
 *
 * NOT BUILT, NOT RUN: this needs a Raspberry Pi with a DW1000 attached
 * and the bitters GPIO/SPI library, neither of which is available in
 * the environment this was written in. See probe/app/unix/build.sh.
 */

/* pthread_setname_np() is a glibc extension. */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <pthread.h>
#if defined(__FreeBSD__)
#include <pthread_np.h>
#endif
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <bitters.h>
#include <bitters/gpio.h>
#include <bitters/spi.h>
#include <bitters/rpi.h>
#include <bitters/version.h>

#include <dw1000/dw1000.h>
#include <dw1000/dw1000_version.h>
#include <dw1000/probe/port.h>
#include <dw1000/probe/record.h>
#include <dw1000/probe/role.h>
#include <dw1000/probe/exchange.h>

#include "config.h"


/*======================================================================*/
/* GPIO / SPI wiring                                                    */
/*======================================================================*/

static bitters_gpio_pin_t
dw1000_reset = RPI_GPIO_PIN_INITIALIZER(DW1000_RESET);
static bitters_gpio_pin_t
dw1000_irq   = RPI_GPIO_PIN_INITIALIZER(DW1000_IRQ);
static bitters_gpio_pin_t
dw1000_wakeup = RPI_GPIO_PIN_INITIALIZER(DW1000_WAKEUP);
static bitters_spi_t
dw1000_spi   = RPI_SPI_INITIALIZER(DW1000_SPI, 0);

/* dw1000_wakeup IS driven here, high, as rpi-redskin does. The driver
 * only asserts it at init and never on the transmit path, and the Zephyr
 * application has no such line -- but on this HAT the pin exists and is
 * an input to the chip, and left unconfigured it floats. The working
 * stack drives it and prints a boot-time hint to pull it up; an earlier
 * version of this file reasoned from the Zephyr side and left it
 * floating. */

/* SPI: initialised at the low speed (required during dw1000_initialise()
 * and dw1000_hardreset()); the driver's own _dw1000_spi_high_speed()/
 * _dw1000_spi_low_speed() (port/unix/dw/osal) switch cfg.speed for the
 * rest of a transaction, exactly as they do for rpi-redskin. */
static struct bitters_spi_cfg dw1000_spi_cfg = {
    .mode     = BITTERS_SPI_MODE_0,
    .transfer = BITTERS_SPI_TRANSFER_MSB,
    .word     = BITTERS_SPI_WORDSIZE(8),
    .speed    = 3000000,
};
static dw1000_spi_driver_t dw1000_spi_drv = {
    .dev        = &dw1000_spi,
    .config     = &dw1000_spi_cfg,
    .low_speed  = 3000000,
    .high_speed = 20000000,
};


/*======================================================================*/
/* Driver bring-up                                                      */
/*======================================================================*/

/* Not calibrated for this instrument -- see zephyr-redskin/probe/src/
 * main.c's own note by the same name. rpi-redskin's HAT uses 154.6 m;
 * kept the same here since this app targets the same hardware, so the
 * two remain comparable the way the Zephyr probe and redskin are on
 * their bench. */
#define PROBE_SPEED_OF_LIGHT_MPS 299792458LL
#define PROBE_METER_TO_CLOCK(dm)					\
    (((int64_t)(dm) * DW1000_TIME_CLOCK_HZ) / (10 * PROBE_SPEED_OF_LIGHT_MPS))
#define PROBE_ANTENNA_DELAY_ROUNDTRIP_M 1546

static void dw1000_cb_tx_done(dw1000_t *dw, uint32_t status);
static void dw1000_cb_rx_timeout(dw1000_t *dw, uint32_t status);
static void dw1000_cb_rx_error(dw1000_t *dw, uint32_t status);
static void dw1000_cb_rx_ok(dw1000_t *dw, uint32_t status,
                            size_t length, bool ranging);

static const dw1000_config_t dw1000_config = {
    .spi              = &dw1000_spi_drv,
    .irq              = &dw1000_irq,
    .reset            = &dw1000_reset,
    .wakeup           = &dw1000_wakeup,
    /* As rpi-redskin has them. Not for the LEDs: the driver's LED block
     * is also where PMSC_CTRL0's GPDCE and KHZCLKEN get enabled, beside
     * the driver author's own note "XXX: seems to be mandatory?!" -- and
     * with leds == 0 that block is skipped entirely. Under test. */
    .leds             = DW1000_LED_ALL,
    .leds_blink_time  = 3,
    .lde_loading      = 1,
    .rxauto           = 1,
    .tx_antenna_delay = PROBE_METER_TO_CLOCK(PROBE_ANTENNA_DELAY_ROUNDTRIP_M) / 2,
    .rx_antenna_delay = PROBE_METER_TO_CLOCK(PROBE_ANTENNA_DELAY_ROUNDTRIP_M) / 2,
    .cb.tx_done       = dw1000_cb_tx_done,
    .cb.rx_timeout    = dw1000_cb_rx_timeout,
    .cb.rx_error      = dw1000_cb_rx_error,
    .cb.rx_ok         = dw1000_cb_rx_ok,
};

static dw1000_t dw0;

/* Same channel/PRF/preamble choice as zephyr-redskin/probe/src/main.c's
 * probe_radio, so a run from this host and a run from either Zephyr
 * board are comparable. tx_power is filled in from argv, in main(). */
static struct dw1000_radio probe_radio = {
    .channel  = 5,
    .bitrate  = DW1000_BITRATE_6800KBPS,
    .prf      = DW1000_PRF_64MHZ,
    .tx_plen  = DW1000_PLEN_128,
    .rx_pac   = DW1000_PAC8,
    .tx_pcode = 10,
    .rx_pcode = 10,
#if DW1000_WITH_PROPRIETARY_SFD
    .proprietary = {
        .sfd = 1,
    },
#endif
};


/*======================================================================*/
/* Radio event callbacks                                                */
/*======================================================================*/

/* Same shape as zephyr-redskin/probe/src/main.c's: capture (for the
 * exchange roles) then re-arm, never the other way round -- a restart is
 * free to disturb the very registers the capture reads. */
static void
dw1000_cb_tx_done(dw1000_t *dw, uint32_t status)
{
    (void)dw;
    (void)status;

    dw1000_probe_tx_capture();
}

static void
dw1000_cb_rx_ok(dw1000_t *dw, uint32_t status, size_t length, bool ranging)
{
    (void)status;
    (void)ranging;

    dw1000_probe_rx_capture(dw, length);
    dw1000_rx_start(&dw0, DW1000_RX_IMMEDIATE);
}

static void
dw1000_cb_rx_timeout(dw1000_t *dw, uint32_t status)
{
    (void)status;

    dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
}

static void
dw1000_cb_rx_error(dw1000_t *dw, uint32_t status)
{
    (void)status;

    dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
}


/*======================================================================*/
/* Interrupt line and event thread                                     */
/*======================================================================*/

/* THE ONE INVIOLABLE RULE, same as zephyr-redskin/probe/src/main.c's:
 * this callback does exactly one thing -- latch the wake-up -- and
 * nothing else. It runs on one of bitters' own GPIO IRQ threads (see
 * bitters_gpio_irq_callback()'s own doc comment), not inside a real
 * signal handler, so calling into dw1000_probe_port_wake() (a pthread
 * mutex/cond under port/unix/src/port.c) is safe here the same way it
 * is safe from any ordinary thread.
 */
static void
dw1000_irq_cb(bitters_gpio_pin_t *pin, void *args)
{
    (void)pin;
    (void)args;

    dw1000_probe_port_wake();
}

/* Not depended upon for correctness -- every real wake-up is latched by
 * dw1000_irq_cb() above -- only a bound on how long a missed edge could
 * go unnoticed, same as the Zephyr application's event thread. */
#define PROBE_EVENT_WAIT_TIMEOUT_US 1000000U

static volatile sig_atomic_t event_thread_stop;

static void *
dw1000_event_thread(void *arg)
{
    (void)arg;
    pthread_setname_np(pthread_self(), "dw1000-event");

    while (!event_thread_stop) {
        if (dw1000_probe_port_wait(PROBE_EVENT_WAIT_TIMEOUT_US)) {
            dw1000_probe_port_bus_lock();
            dw1000_process_events(&dw0);
            dw1000_probe_port_bus_unlock();
        }
    }
    return NULL;
}


/*======================================================================*/
/* Line buffering for twr_resp -- see <dw1000/probe/exchange.h>'s note   */
/* on why dw1000_probe_twr_resp_run() takes a callback rather than       */
/* owning a buffer of its own: a Linux process has no RAM budget to      */
/* protect the way the DWM1001 does, so this simply heap-allocates       */
/* enough for the run it is about to make, rather than reusing a fixed   */
/* static buffer the way the embedded application does.                 */
/*======================================================================*/

static char   *line_buf;
static size_t  line_buf_cap;
static size_t  line_buf_used;

static void
line_buf_start(size_t capacity)
{
    free(line_buf);
    line_buf      = malloc(capacity * DW1000_PROBE_RECORD_MAX);
    line_buf_cap  = (line_buf != NULL) ? capacity : 0;
    line_buf_used = 0;
}

static void
line_buf_append(const char *line)
{
    if (line_buf_used >= line_buf_cap)
        return; /* dropped, same policy as the embedded application's */

    char *slot = line_buf + line_buf_used * DW1000_PROBE_RECORD_MAX;
    strncpy(slot, line, DW1000_PROBE_RECORD_MAX - 1);
    slot[DW1000_PROBE_RECORD_MAX - 1] = '\0';
    line_buf_used++;
}

static void
line_buf_dump(void)
{
    size_t i;

    for (i = 0; i < line_buf_used; i++)
        dw1000_probe_port_emit(line_buf + i * DW1000_PROBE_RECORD_MAX);
    line_buf_used = 0;
}


/*======================================================================*/
/* argv                                                                  */
/*======================================================================*/

static void
usage(const char *prog)
{
    fprintf(stderr,
        "usage: %s [options] <role> <own_addr> <peer_addr> <count>\n"
        "\n"
        "  role        twr_init | twr_resp\n"
        "  own_addr    this node's address (hex accepted, e.g. 0xc939)\n"
        "  peer_addr   the peer's address (twr_resp: RESPONSE answers\n"
        "              whoever sent the POLL, so this is unused but\n"
        "              still required, for symmetry with twr_init)\n"
        "  count       exchanges to run\n"
        "\n"
        "options:\n"
        "  --ss              two-frame single-sided estimate only\n"
        "  --warmup=N        twr_init: uncounted exchanges first (default 5)\n"
        "  --power=<dB>|auto transmit power, 0..30.5 on the 0.5 dB grid\n"
        "                    (default: auto)\n"
        "  --node=NAME       origin node name emitted lines carry\n"
        "                    (default: this host's hostname)\n",
        prog);
}

static bool
parse_addr(const char *s, uint16_t *out)
{
    char *end;
    long  v = strtol(s, &end, 0); /* base 0: "0xc939" accepted */

    if (*end != '\0' || v <= 0 || v > 0xffff)
        return false;
    *out = (uint16_t)v;
    return true;
}

/* Same grammar and grid as zephyr-redskin/probe/src/shell.c's `probe
 * power`: tenths of a dB, hand-parsed because the chip's grid is half-dB
 * steps and a float would only invite a value between two settings.
 */
static bool
parse_power(const char *s, uint8_t *encoded)
{
    if (strcmp(s, "auto") == 0) {
        *encoded = DW1000_TX_POWER_AUTO;
        return true;
    }

    const char *p = s;
    int tenths = 0, digits = 0;

    for (; *p >= '0' && *p <= '9'; p++) {
        tenths = tenths * 10 + (*p - '0');
        digits++;
    }
    tenths *= 10;
    if (*p == '.') {
        p++;
        if (*p < '0' || *p > '9')
            return false;
        tenths += (*p++ - '0');
    }
    if (digits == 0 || *p != '\0')
        return false;
    if (tenths % 5 != 0 || tenths > 305)
        return false;

    *encoded = DW1000_TX_POWER_05DB((uint8_t)(tenths / 5));
    return true;
}


/*======================================================================*/
/* main                                                                  */
/*======================================================================*/

int
main(int argc, char *argv[])
{
    const char *prog = argv[0];
    bool     ss          = false;
    long     warmup      = 5;
    uint8_t  power        = DW1000_TX_POWER_AUTO;
    char     node_name[64] = {0};

    while (argc > 1 && argv[1][0] == '-') {
        if (strcmp(argv[1], "--ss") == 0) {
            ss = true;
        } else if (strncmp(argv[1], "--warmup=", 9) == 0) {
            warmup = strtol(argv[1] + 9, NULL, 10);
        } else if (strncmp(argv[1], "--power=", 8) == 0) {
            if (!parse_power(argv[1] + 8, &power))
                DIE("invalid --power value '%s'", argv[1] + 8);
        } else if (strncmp(argv[1], "--node=", 7) == 0) {
            strncpy(node_name, argv[1] + 7, sizeof(node_name) - 1);
        } else if (strcmp(argv[1], "--") == 0) {
            argc--; argv++;
            break;
        } else {
            fprintf(stderr, "%s: unknown option %s\n", prog, argv[1]);
            usage(prog);
            return EXIT_USAGE;
        }
        argc--; argv++;
    }

    if (argc != 5) {
        usage(prog);
        return EXIT_USAGE;
    }

    const char *role_name = argv[1];
    bool is_resp;
    if (strcmp(role_name, "twr_resp") == 0) {
        is_resp = true;
    } else if (strcmp(role_name, "twr_init") == 0) {
        is_resp = false;
    } else {
        fprintf(stderr, "%s: unknown role '%s' (twr_init or twr_resp)\n",
               prog, role_name);
        return EXIT_USAGE;
    }

    uint16_t own_addr, peer_addr;
    if (!parse_addr(argv[2], &own_addr) || !parse_addr(argv[3], &peer_addr)) {
        fprintf(stderr, "%s: invalid address\n", prog);
        return EXIT_USAGE;
    }

    char *end;
    long count = strtol(argv[4], &end, 10);
    if (*end != '\0' || count <= 0) {
        fprintf(stderr, "%s: invalid count '%s'\n", prog, argv[4]);
        return EXIT_USAGE;
    }

    if (node_name[0] == '\0') {
        if (gethostname(node_name, sizeof(node_name) - 1) != 0)
            strcpy(node_name, "-");
    }

    INFO("probe (unix) -- dw1000 %s / bitters %s",
        DW1000_VERSION_FULL, bitters_version());
    INFO("PID = %d", getpid());
    INFO("Antenna delay tx=%u / rx=%u",
        dw1000_config.tx_antenna_delay, dw1000_config.rx_antenna_delay);

    if (bitters_init() < 0)
        DIE_ERRNO("bitters library initialisation failed");

    struct bitters_gpio_cfg reset_cfg = {
        .dir    = BITTERS_GPIO_DIR_OUTPUT,
        .defval = 1, /* active-low: inactive is high, same polarity as
                      * zephyr-redskin's GPIO_OUTPUT_INACTIVE */
        .label  = "dw1000-reset",
    };
    struct bitters_gpio_cfg irq_cfg = {
        .dir       = BITTERS_GPIO_DIR_INPUT,
        .interrupt = BITTERS_GPIO_INTERRUPT_RISING_EDGE,
        .label     = "dw1000-int",
    };
    struct bitters_gpio_cfg wakeup_cfg = {
        .dir    = BITTERS_GPIO_DIR_OUTPUT,
        .defval = 1,
        .label  = "dw1000-wakeup",
    };
    if (bitters_gpio_pin_enable(&dw1000_reset, &reset_cfg) < 0 ||
        bitters_gpio_pin_enable(&dw1000_wakeup, &wakeup_cfg) < 0 ||
        bitters_gpio_pin_enable(&dw1000_irq, &irq_cfg) < 0)
        DIE_ERRNO("unable to configure gpio for dw1000");

    if (bitters_spi_enable(&dw1000_spi, &dw1000_spi_cfg) < 0)
        DIE_ERRNO("unable to configure spi for dw1000");

    dw1000_init(&dw0, &dw1000_config);
    /* No dw1000_hardreset() here: the working stack on this hardware
     * (spank/port/hal/io/dw1000/src/driver.c) goes init -> initialise ->
     * configure without one, and initialise() performs its own reset. A
     * hard reset released early leaves the chip's clocks half configured
     * -- receive works on defaults, transmit does not. Under test. */
    if (dw1000_initialise(&dw0) < 0)
        DIE("DW1000 not identified");

    probe_radio.tx_power = power;
    if (dw1000_configure(&dw0, &probe_radio) < 0)
        DIE("DW1000 radio configuration rejected");

    if (bitters_gpio_irq_callback(&dw1000_irq, dw1000_irq_cb, NULL) < 0)
        DIE_ERRNO("failed to attach DW1000 IRQ callback");

    /* PROBE_TXTEST=1: the rawest possible transmit, bypassing the exchange,
     * the receiver and the event thread entirely -- write, start, poll
     * TXFRS. A listener that hears this and not the exchange indicts the
     * exchange's interaction; one that hears neither indicts bring-up. */
    if (getenv("PROBE_TXTEST") != NULL) {
        uint8_t f[20];
        int     done = 0, started = 0;
        int     count = atoi(getenv("PROBE_TXTEST"));
        if (count < 1) count = 100;
        for (int i = 0; i < (int)sizeof f; i++) f[i] = (uint8_t)i;
        for (int n = 0; n < count; n++) {
            dw1000_probe_port_bus_lock();
            dw1000_txrx_off(&dw0);
            dw1000_tx_write_frame_data(&dw0, f, sizeof f, 0);
            dw1000_tx_fctrl(&dw0, sizeof f + 2, 0, DW1000_TX_IMMEDIATE);
            int rc = dw1000_tx_start(&dw0, DW1000_TX_IMMEDIATE);
            dw1000_probe_port_bus_unlock();
            if (rc == 0) started++;
            dw1000_probe_port_sleep(5000);
            dw1000_probe_port_bus_lock();
            if (dw1000_tx_is_status_done(&dw0)) done++;
            dw1000_tx_clear_status_done(&dw0);
            dw1000_probe_port_bus_unlock();
            /* 200 ms pitch, so 100 frames span 20 s: a listener attached
             * through tribe-control starts ~5 s late, and a 2 s burst at
             * 20 ms was caught only at its tail (11-30 of 100). At this
             * pitch: 99 of 100. Change it and the count means nothing. */
            dw1000_probe_port_sleep(195000);
        }
        printf("TXTEST: %d/%d started, %d saw TXFRS\n", started, count, done);
        return 0;
    }

    pthread_t event_tid;
    if (pthread_create(&event_tid, NULL, dw1000_event_thread, NULL) != 0)
        DIE_ERRNO("failed to start the event thread");

    int rc = EXIT_OK;
    if (is_resp) {
        line_buf_start((size_t)count + 1);
        struct dw1000_probe_twr_resp_result result =
            dw1000_probe_twr_resp_run(&dw0, count, ss, own_addr, peer_addr,
                                      node_name, line_buf_append);
        line_buf_dump();
        INFO("twr_resp: %u/%u resolved", result.resolved, result.attempted);
    } else {
        struct dw1000_probe_twr_init_result result =
            dw1000_probe_twr_init_run(&dw0, count, ss, warmup, own_addr, peer_addr);
        INFO("twr_init: %" PRIu32 "/%" PRIu32
            " exchanges reached %s (warmup=%ld, not counted)",
            result.reached, result.attempted,
            ss ? "FINAL" : "REPORT", warmup);
        /* Only worth a line when it happened: an unconfirmed REPORT send
         * is the difference between "the peer did not hear us" and "we
         * never got it out", and the peer cannot tell them apart. */
        INFO("twr_init: FINAL->REPORT turnaround %" PRIu32 "-%" PRIu32 " us",
             result.turnaround_min_us, result.turnaround_max_us);
        if (result.report_failed > 0)
            INFO("twr_init: %" PRIu32 " REPORT send(s) unconfirmed",
                 result.report_failed);
    }

    /* Shut the radio down before tearing down the event thread and the
     * pins/bus it depends on. */
    dw1000_probe_port_bus_lock();
    dw1000_txrx_off(&dw0);
    dw1000_probe_port_bus_unlock();

    /* If a wake-up is already latched when the nudge below arrives, the
     * event thread's current dw1000_probe_port_wait() returns true and
     * it runs one more dw1000_process_events() before it next tests
     * event_thread_stop -- and a stray rx_timeout/rx_error there would
     * re-arm the receiver right after the txrx_off() above. Harmless
     * for a process about to tear down its pins and exit, so left as
     * best-effort rather than adding a second flag to close that
     * window; a long-lived host that reused this shutdown sequence
     * would want to. */
    event_thread_stop = 1;
    dw1000_probe_port_wake(); /* nudge the event thread out of its wait */
    pthread_join(event_tid, NULL);

    if (bitters_spi_disable(&dw1000_spi) < 0)
        WARN_ERRNO("unable to disable spi for dw1000");
    if (bitters_gpio_pin_disable(&dw1000_irq) < 0 ||
        bitters_gpio_pin_disable(&dw1000_wakeup) < 0 ||
        bitters_gpio_pin_disable(&dw1000_reset) < 0)
        WARN_ERRNO("unable to disable gpio for dw1000");

    free(line_buf);

    return rc;
}
