/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * DW1000 probe: Linux/Raspberry Pi application.
 *
 * Where zephyr-redskin/probe's src/shell.c offers the roles as `probe`
 * shell commands, this offers the same five (twr_init, twr_resp, tx, rx,
 * temperature) and the same three read-backs (info, config, power) as
 * one argv-driven run: bring the radio up, run the selected role once,
 * report a summary, exit. Both drive the roles in probe/src, so what is
 * left here is choosing values: argv, defaults and the pin wiring. It is deliberately not a daemon, unlike
 * rpi-redskin/main.c, which this is modelled on for the bring-up shape
 * (bitters GPIO/SPI wiring, the DW1000 pin map, the processing thread),
 * there is no protocol running underneath needing a long-lived process,
 * no reset signal, no debug channel: one role, one run, one exit code.
 *
 * Needs a Raspberry Pi with a DW1000 attached and the bitters GPIO/SPI
 * library, so it builds nowhere else; see probe/app/unix/build.sh,
 * which takes BITTERS= and reads both trees' manifests. Built and run on
 * rpi-a; the bench pair for measurements is rpi-c to rpi-d.
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
#include <limits.h>

#include <bitters.h>
#include <bitters/gpio.h>
#include <bitters/spi.h>
#include <bitters/rpi.h>
#include <bitters/version.h>

#include <dw1000/dw1000.h>
#include <dw1000/dw1000_state.h>
#include <dw1000/dw1000_version.h>
#include <dw1000/probe/port.h>
#include <dw1000/probe/record.h>
#include <dw1000/probe/role.h>
#include <dw1000/probe/exchange.h>
#include <dw1000/probe/radio.h>
#include <dw1000/probe/settle.h>
#include <dw1000/probe/solo.h>

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
 * application has no such line. But on this HAT the pin exists and is
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

/* Not calibrated for this instrument; see zephyr-redskin/probe/src/
 * main.c's own note by the same name. rpi-redskin's HAT uses 154.6 m;
 * kept the same here since this app targets the same hardware, so the
 * two remain comparable the way the Zephyr probe and redskin are on
 * their bench. */
/* The conversion is DW1000_METER_TO_CLOCK() in <dw1000/dw1000.h> now,
 * where the sniffer and rpi-redskin reach it too. This file used to
 * spell its own, in integer decimetres, because metres as an integer are
 * too coarse for an antenna delay (one metre is 213 ticks). The driver's
 * is floating point for that reason, so the figure is written in metres
 * as it is everywhere it is measured and written down. Same answer:
 * 154.6 m is 32951 ticks either way, 16475 after halving.
 *
 * A default and no more: --antenna-delay= (and its tx/rx halves) sets
 * another for one run, in the ticks the SETUP line prints.
 */
#define PROBE_ANTENNA_DELAY_ROUNDTRIP_M 154.6

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
     * the driver author's own note "XXX: seems to be mandatory?!", and
     * with leds == 0 that block is skipped entirely. Under test. */
    .leds             = DW1000_LED_ALL,
    .leds_blink_time  = 3,
    .lde_loading      = 1,
    .rxauto           = 0,     // A sender leaves the chip's re-enable off: see hw/drivers/dw1000/README.md
    /* Not an experiment any more, and for the responder not a choice:
     * benched against rpi-d, 4 interleaved rounds of 30
     * exchanges per combination, a single-buffered responder resolved
     * 0 of 120: it hears the POLL and the FINAL and never the REPORT.
     * Double buffered it resolved 120 of 120. AUDIT.md carries the
     * table.
     *
     * The initiator does have a choice, which --no-dblbuff exposes, and
     * it costs about 2.3 cm on asym_mm (and slightly more spread). The
     * guide's older 6 to 10 cm figure is not reproduced here, and the
     * experiment that would attribute it (both ends single buffered)
     * cannot be run, because that configuration does not resolve. */
    .dblbuff          = 1,
    .tx_antenna_delay = DW1000_METER_TO_CLOCK(PROBE_ANTENNA_DELAY_ROUNDTRIP_M) / 2,
    .rx_antenna_delay = DW1000_METER_TO_CLOCK(PROBE_ANTENNA_DELAY_ROUNDTRIP_M) / 2,
    .cb.tx_done       = dw1000_cb_tx_done,
    .cb.rx_timeout    = dw1000_cb_rx_timeout,
    .cb.rx_error      = dw1000_cb_rx_error,
    .cb.rx_ok         = dw1000_cb_rx_ok,
};

static dw1000_t dw0;

/* The radio itself is not written here: its defaults are the library's
 * (dw1000_probe_radio_default(), the same on a board), every field can
 * be changed from argv, and the antenna delays above are this platform's
 * defaults for the two it leaves to the shell. main() resolves the lot
 * through the driver's own validation before the chip is touched. */


/*======================================================================*/
/* Radio event callbacks                                                */
/*======================================================================*/

/* Same shape as zephyr-redskin/probe/src/main.c's: capture (for the
 * exchange roles) then re-arm, never the other way round: a restart is
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

    /* Counted for the rx role's account of what the chip rejected. */
    dw1000_probe_rx_error_capture();
    dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
}


/*======================================================================*/
/* Interrupt line and event thread                                     */
/*======================================================================*/

/* THE ONE INVIOLABLE RULE, same as zephyr-redskin/probe/src/main.c's:
 * this callback does exactly one thing, latch the wake-up, and
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

/* Not depended upon for correctness (every real wake-up is latched by
 * dw1000_irq_cb() above), only a bound on how long a missed edge could
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
/* Line buffering: see <dw1000/probe/exchange.h>'s note on why the      */
/* roles take a callback rather than owning a buffer of their own. A    */
/* Linux process has no RAM budget to protect the way the DWM1001 does, */
/* so this one grows as the run needs it and has no cap: a board refuses */
/* a run its fixed buffer cannot hold, this does not have to.           */
/*======================================================================*/

static char   *line_buf;
static size_t  line_buf_cap;    /* in lines */
static size_t  line_buf_used;
static size_t  line_buf_dropped;

static void
line_buf_append(const char *line)
{
    if (line_buf_used >= line_buf_cap) {
        /* Grown between frames only in the sense that a role calls this
         * at the end of a step, never inside a wait; a doubling makes it
         * rare, and a run of 64 lines never grows at all. */
        size_t cap = line_buf_cap ? 2 * line_buf_cap : 64;
        char  *buf = realloc(line_buf, cap * DW1000_PROBE_RECORD_MAX);

        if (buf == NULL) {
            line_buf_dropped++;
            return;
        }
        line_buf     = buf;
        line_buf_cap = cap;
    }

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
    if (line_buf_dropped > 0)
        WARN("%zu line(s) lost: out of memory", line_buf_dropped);
    line_buf_used    = 0;
    line_buf_dropped = 0;
}

/* A multi-line block (the driver's radio state, the probe's chip info)
 * one INFO line at a time. */
static void
print_lines(char *text)
{
    for (char *p = text, *nl; p; p = nl) {
        nl = strchr(p, '\n');
        if (nl) *nl++ = '\0';
        INFO("%s", p);
    }
}


/*======================================================================*/
/* argv                                                                  */
/*======================================================================*/

static void
usage(FILE *out, const char *prog)
{
    fprintf(out,
        "usage: %s [options] <role> <arguments>\n"
        "\n"
        "roles:\n"
        "  twr_init <own_addr> <peer_addr> <count>\n"
        "  twr_resp <own_addr> <peer_addr> <count>\n"
        "              the two-node exchange; addresses in hex accepted\n"
        "              (0xc939); twr_resp answers whoever polls it, so\n"
        "              its peer_addr is unused, but required for symmetry\n"
        "  tx <count>  transmit count frames, --gap apart\n"
        "  rx <seconds>\n"
        "              listen, counting received and rejected frames\n"
        "  rx          with --settle: listen until the die settles\n"
        "  temperature [<seconds>]\n"
        "              radio stopped, sample the die; no seconds: one\n"
        "              reading\n"
        "\n"
        "read-backs (bring the radio up, print, exit):\n"
        "  info        chip and lot id, OTP references, radio, power\n"
        "  config      the radio configuration read back off the chip\n"
        "  power       the applied transmit power read back off the chip\n"
        "\n"
        "options, every role:\n"
        "  --power=<dB>|auto transmit power, 0..30.5 on the 0.5 dB grid\n"
        "                    (default: auto)\n"
        "  --channel=N       1 2 3 4 5 7 (default 5)\n"
        "  --bitrate=KBPS    110 850 6800 (default 6800)\n"
        "  --prf=MHZ         16 64 (default 64)\n"
        "  --preamble=SYMBOLS  preamble length (default 128)\n"
        "  --pac=SYMBOLS     8 16 32 64 (default 8)\n"
        "  --code=N          preamble code, both ways (default 10);\n"
        "                    --tx-code=N, --rx-code=N for one of them\n"
        "  --sfd=decawave|standard  (default decawave)\n"
        "  --antenna-delay=TICKS  both ways (default 16475);\n"
        "                    --tx-antenna-delay=, --rx-antenna-delay=\n"
        "  --node=NAME       origin node name emitted lines carry\n"
        "                    (default: this host's hostname)\n"
        "  --dblbuff         double-buffered receive (default)\n"
        "  --no-dblbuff      single-buffered, to compare against\n"
        "options, one role:\n"
        "  --ss              twr_*: two-frame single-sided estimate only\n"
        "  --warmup=N        twr_init: uncounted exchanges first (default 5)\n"
        "  --gap=MS          tx: between frames (default 10)\n"
        "  --settle          rx: until settled, rather than for <seconds>\n"
        "  --settle-window=S      rx: window length (default 30)\n"
        "  --settle-windows=N     rx: windows that must agree (default 4)\n"
        "  --settle-threshold=DEG rx: their means' spread below (0.2)\n"
        "  --settle-give-up=S     rx: give up after (default 900)\n"
        "                    each --settle-* implies --settle\n"
        "\n"
        "  --help            this, and exit\n"
        "  --version         the probe's driver and bitters, and exit\n",
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

/* A whole number in [min, max]. */
static bool
parse_ulong(const char *s, unsigned long min, unsigned long max,
            unsigned long *out)
{
    char *end;
    unsigned long v;

    if (*s < '0' || *s > '9')
        return false;
    v = strtoul(s, &end, 10);
    if (*end != '\0' || v < min || v > max)
        return false;
    *out = v;
    return true;
}

/* Degrees with at most two decimals, to hundredths: "0.2" -> 20. By hand
 * for the reason parse_power() gives; the same grammar as the Zephyr
 * shell's --settle-threshold. */
static bool
parse_cdeg(const char *s, uint16_t *out)
{
    const char *p = s;
    unsigned long v = 0;
    int digits = 0, decimals = 0;

    for (; *p >= '0' && *p <= '9' && v < 1000; p++, digits++)
        v = v * 10 + (unsigned long)(*p - '0');
    if (*p == '.')
        for (p++; *p >= '0' && *p <= '9' && decimals < 2; p++, decimals++)
            v = v * 10 + (unsigned long)(*p - '0');
    if (digits == 0 || *p != '\0')
        return false;
    for (; decimals < 2; decimals++)
        v *= 10;
    if (v == 0 || v > 65535)
        return false;
    *out = (uint16_t)v;
    return true;
}


/*======================================================================*/
/* main                                                                  */
/*======================================================================*/

/* What argv asked for: a role, or one of the read-backs. */
enum action { ACTION_ROLE, ACTION_INFO, ACTION_CONFIG, ACTION_POWER };

int
main(int argc, char *argv[])
{
    const char *prog = argv[0];
    bool     ss          = false;
    long     warmup      = 5;
    bool     warmup_set  = false;
    uint32_t gap_us      = DW1000_PROBE_TX_GAP_US;
    bool     gap_set     = false;
    bool     settle      = false;
    struct dw1000_probe_settle_params settle_params =
        DW1000_PROBE_SETTLE_PARAMS_DEFAULT;
    uint8_t  power        = DW1000_TX_POWER_AUTO;
    char     node_name[64] = {0};
    /* Default on, as the config template has it. The switch exists so a
     * bench run can interleave the two modes without rebuilding between
     * them, which is the only way to compare them against the same air:
     * rpi-redskin has the same pair of flags, for the same reason. */
    bool     dblbuff      = dw1000_config.dblbuff;
    struct dw1000_probe_radio radio;
    unsigned long v;
    int      rc           = EXIT_OK;

    dw1000_probe_radio_default(&radio, dw1000_config.tx_antenna_delay,
                               dw1000_config.rx_antenna_delay);

    while (argc > 1 && argv[1][0] == '-') {
        const char *a = argv[1];
        const char *why;

        /* The radio's own options, spelt by the library so that a board
         * takes exactly the same ones. */
        switch (dw1000_probe_radio_option(&radio, a, &why)) {
        case DW1000_PROBE_RADIO_OPTION_SET:
            argc--; argv++;
            continue;
        case DW1000_PROBE_RADIO_OPTION_BAD:
            fprintf(stderr, "%s: %s: %s\n", prog, a, why);
            return EXIT_USAGE;
        case DW1000_PROBE_RADIO_OPTION_NONE:
            break;
        }

        if (strcmp(a, "--ss") == 0) {
            ss = true;
        } else if (strncmp(a, "--warmup=", 9) == 0) {
            if (!parse_ulong(a + 9, 0, 100000, &v))
                DIE("invalid --warmup value '%s'", a + 9);
            warmup     = (long)v;
            warmup_set = true;
        } else if (strncmp(a, "--gap=", 6) == 0) {
            if (!parse_ulong(a + 6, 0, UINT32_MAX / 1000, &v))
                DIE("invalid --gap value '%s'", a + 6);
            gap_us  = (uint32_t)v * 1000u;
            gap_set = true;
        } else if (strcmp(a, "--settle") == 0) {
            settle = true;
        } else if (strncmp(a, "--settle-window=", 16) == 0) {
            if (!parse_ulong(a + 16, 1, UINT32_MAX, &v))
                DIE("invalid --settle-window value '%s'", a + 16);
            settle_params.window_s = (uint32_t)v;
            settle = true;
        } else if (strncmp(a, "--settle-windows=", 17) == 0) {
            if (!parse_ulong(a + 17, 2, DW1000_PROBE_SETTLE_WINDOWS_MAX, &v))
                DIE("invalid --settle-windows value '%s' (2..%d)", a + 17,
                    DW1000_PROBE_SETTLE_WINDOWS_MAX);
            settle_params.windows = (uint8_t)v;
            settle = true;
        } else if (strncmp(a, "--settle-threshold=", 19) == 0) {
            if (!parse_cdeg(a + 19, &settle_params.threshold_cdeg))
                DIE("invalid --settle-threshold value '%s' (degrees, "
                    "two decimals at most)", a + 19);
            settle = true;
        } else if (strncmp(a, "--settle-give-up=", 17) == 0) {
            if (!parse_ulong(a + 17, 1, UINT32_MAX, &v))
                DIE("invalid --settle-give-up value '%s'", a + 17);
            settle_params.give_up_s = (uint32_t)v;
            settle = true;
        } else if (strncmp(a, "--power=", 8) == 0) {
            if (!parse_power(a + 8, &power))
                DIE("invalid --power value '%s'", a + 8);
        } else if (strncmp(a, "--node=", 7) == 0) {
            strncpy(node_name, a + 7, sizeof(node_name) - 1);
        } else if (strcmp(a, "--dblbuff") == 0) {
            dblbuff = true;
        } else if (strcmp(a, "--no-dblbuff") == 0) {
            dblbuff = false;
        } else if (strcmp(a, "--help") == 0) {
            usage(stdout, prog);
            return EXIT_OK;
        } else if (strcmp(a, "--version") == 0) {
            printf("probe (unix): dw1000 %s / bitters %s\n",
                   DW1000_VERSION_FULL, bitters_version());
            return EXIT_OK;
        } else if (strcmp(a, "--") == 0) {
            argc--; argv++;
            break;
        } else {
            fprintf(stderr, "%s: unknown option %s\n", prog, a);
            usage(stderr, prog);
            return EXIT_USAGE;
        }
        argc--; argv++;
    }

    if (argc < 2) {
        usage(stderr, prog);
        return EXIT_USAGE;
    }

    /* The verb: a canonical role name, read through the one table that
     * spells them (<dw1000/probe/role.h>), or a read-back. */
    const char          *verb   = argv[1];
    char               **args   = argv + 2;
    int                  nargs  = argc - 2;
    enum action          action = ACTION_ROLE;
    dw1000_probe_role_t  role   = DW1000_PROBE_ROLE__COUNT;

    if      (strcmp(verb, "info")   == 0) action = ACTION_INFO;
    else if (strcmp(verb, "config") == 0) action = ACTION_CONFIG;
    else if (strcmp(verb, "power")  == 0) action = ACTION_POWER;
    else if (!dw1000_probe_role_lookup(verb, &role)) {
        fprintf(stderr, "%s: unknown role '%s'\n", prog, verb);
        usage(stderr, prog);
        return EXIT_USAGE;
    }

    bool is_twr = action == ACTION_ROLE &&
                  (role == DW1000_PROBE_ROLE_TWR_INIT ||
                   role == DW1000_PROBE_ROLE_TWR_RESP);

    /* An option meant for one role and given to another is refused rather
     * than ignored: a run that silently dropped --settle would look like
     * one that settled. */
    if ((ss && !is_twr) ||
        (warmup_set && !(action == ACTION_ROLE &&
                         role == DW1000_PROBE_ROLE_TWR_INIT)) ||
        (gap_set && !(action == ACTION_ROLE && role == DW1000_PROBE_ROLE_TX)) ||
        (settle && !(action == ACTION_ROLE && role == DW1000_PROBE_ROLE_RX))) {
        fprintf(stderr, "%s: an option given does not apply to '%s'\n",
                prog, verb);
        return EXIT_USAGE;
    }
    if (settle && !dw1000_probe_settle_params_valid(&settle_params)) {
        fprintf(stderr, "%s: these settle values cannot settle: the give-up "
                "must cover the windows, a window one reading\n", prog);
        return EXIT_USAGE;
    }

    uint16_t      own_addr = 0, peer_addr = 0;
    long          count    = 0;
    unsigned long seconds  = 0;
    int           want     = 0;     /* positional arguments the verb takes */

    if (action == ACTION_ROLE) {
        switch (role) {
        case DW1000_PROBE_ROLE_TWR_INIT:
        case DW1000_PROBE_ROLE_TWR_RESP:  want = 3;               break;
        case DW1000_PROBE_ROLE_TX:        want = 1;               break;
        case DW1000_PROBE_ROLE_RX:        want = settle ? 0 : 1;  break;
        case DW1000_PROBE_ROLE_TEMPERATURE:
                                          want = nargs > 0 ? 1 : 0; break;
        default:                                                  break;
        }
    }
    if (nargs != want) {
        usage(stderr, prog);
        return EXIT_USAGE;
    }

    if (is_twr) {
        if (!parse_addr(args[0], &own_addr) || !parse_addr(args[1], &peer_addr)) {
            fprintf(stderr, "%s: invalid address\n", prog);
            return EXIT_USAGE;
        }
        args += 2;
    }
    if (is_twr || (action == ACTION_ROLE && role == DW1000_PROBE_ROLE_TX)) {
        if (!parse_ulong(args[0], 1, LONG_MAX, &v)) {
            fprintf(stderr, "%s: invalid count '%s'\n", prog, args[0]);
            return EXIT_USAGE;
        }
        count = (long)v;
    } else if (want == 1) {
        /* rx and temperature: seconds, bounded so that the sampling
         * arithmetic stays in range. */
        if (!parse_ulong(args[0], role == DW1000_PROBE_ROLE_RX ? 1 : 0,
                         UINT32_MAX / 1000, &seconds)) {
            fprintf(stderr, "%s: invalid seconds '%s'\n", prog, args[0]);
            return EXIT_USAGE;
        }
    }

    if (node_name[0] == '\0') {
        if (gethostname(node_name, sizeof(node_name) - 1) != 0)
            strcpy(node_name, "-");
    }

    /* The combination, by the driver's rule, before the chip is touched:
     * a refused radio is a usage error, not a run that fails half-way. */
    struct dw1000_radio probe_radio;
    {
        const char *why;

        if (!dw1000_probe_radio_resolve(&radio, power, &probe_radio, &why)) {
            fprintf(stderr, "%s: %s\n", prog, why);
            return EXIT_USAGE;
        }
    }

    INFO("probe (unix): dw1000 %s / bitters %s",
        DW1000_VERSION_FULL, bitters_version());
    INFO("PID = %d", getpid());
    INFO("Antenna delay tx=%u / rx=%u",
        radio.tx_antenna_delay, radio.rx_antenna_delay);

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

    /* dw1000_init() keeps the pointer, so the copy the switch writes to
     * is static rather than automatic; the template above stays const. */
    static dw1000_config_t dw1000_config_run;
    dw1000_config_run                  = dw1000_config;
    dw1000_config_run.dblbuff          = dblbuff;
    dw1000_config_run.tx_antenna_delay = radio.tx_antenna_delay;
    dw1000_config_run.rx_antenna_delay = radio.rx_antenna_delay;

    dw1000_init(&dw0, &dw1000_config_run);
    /* init -> hardreset -> initialise, which is what
     * spank/port/hal/io/dw1000/src/driver.c does and what
     * sniffer/app/unix/uwb_dw1000.c does after it. This application used
     * to skip the reset and was the one thing on the bench bringing the
     * DW1000 up differently from everything else on it, on the strength
     * of a comment that misread the very file it cited as precedent.
     */
    dw1000_hardreset(&dw0);
    if (dw1000_initialise(&dw0) < 0)
        DIE("DW1000 not identified");

    if (dw1000_configure(&dw0, &probe_radio) < 0)
        DIE("DW1000 radio configuration rejected");

    /* Read back off the chip, not echoed from the config: the two are
     * the same only when the chip honoured what it was asked. Printed
     * once at start-up, never inside an exchange: it is SPI traffic.
     * The identical lines come out of the Zephyr shell's `probe info`,
     * `probe config` and `probe power`, so the two platforms diff
     * directly. A role run prints the configuration and the power, so
     * that every run's log says what the radio was; a read-back prints
     * what it was asked for and nothing else. */
    {
        dw1000_radio_state_t st;
        char                 text[DW1000_PROBE_INFO_MAX > DW1000_RADIO_STATE_MAX
                                  ? DW1000_PROBE_INFO_MAX
                                  : DW1000_RADIO_STATE_MAX];

        if (action == ACTION_INFO) {
            dw1000_probe_info_format(text, sizeof(text), &dw0);
            print_lines(text);
        }
        if (action != ACTION_POWER) {
            dw1000_get_radio_state(&dw0, &st);
            dw1000_radio_state_format(text, sizeof(text), &st);
            print_lines(text);
        }
        if (action != ACTION_CONFIG) {
            dw1000_probe_power_format(text, sizeof(text), &dw0);
            INFO("%s", text);
        }
    }

    if (action != ACTION_ROLE)
        goto pins;


    if (bitters_gpio_irq_callback(&dw1000_irq, dw1000_irq_cb, NULL) < 0)
        DIE_ERRNO("failed to attach DW1000 IRQ callback");

    /* PROBE_TXTEST=1: the rawest possible transmit, bypassing the exchange,
     * the receiver and the event thread entirely: write, start, poll
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

    switch (role) {
    case DW1000_PROBE_ROLE_TWR_RESP: {
        struct dw1000_probe_twr_resp_result result =
            dw1000_probe_twr_resp_run(&dw0, count, ss, own_addr, peer_addr,
                                      node_name, line_buf_append);
        line_buf_dump();
        INFO("twr_resp: %u/%u resolved", result.resolved, result.attempted);
        break;
    }

    case DW1000_PROBE_ROLE_TWR_INIT: {
        struct dw1000_probe_twr_init_result result =
            dw1000_probe_twr_init_run(&dw0, count, ss, warmup, own_addr,
                                      peer_addr, node_name);
        INFO("twr_init: %" PRIu32 "/%" PRIu32
            " exchanges reached %s (warmup=%ld, not counted)",
            result.reached, result.attempted,
            ss ? "FINAL" : "REPORT", warmup);
        /* Only worth a line when it happened: an unconfirmed REPORT send
         * is the difference between "the peer did not hear us" and "we
         * never got it out", and the peer cannot tell them apart. */
        if (result.report_failed > 0)
            INFO("twr_init: %" PRIu32 " REPORT send(s) unconfirmed",
                 result.report_failed);
        break;
    }

    case DW1000_PROBE_ROLE_TX: {
        struct dw1000_probe_tx_result result =
            dw1000_probe_tx_run(&dw0, count, gap_us,
                                DW1000_PROBE_SAMPLE_INTERVAL_MS,
                                node_name, line_buf_append);
        line_buf_dump();
        INFO("tx: %" PRIu32 "/%ld started, %" PRIu32 " completed",
             result.started, count, result.completed);
        break;
    }

    case DW1000_PROBE_ROLE_RX: {
        struct dw1000_probe_rx_result result = settle
            ? dw1000_probe_rx_settle_run(&dw0, &settle_params,
                                         node_name, line_buf_append)
            : dw1000_probe_rx_run(&dw0, (uint32_t)seconds,
                                  DW1000_PROBE_SAMPLE_INTERVAL_MS,
                                  node_name, line_buf_append);
        line_buf_dump();
        if (result.failed) {
            WARN("rx: failed to start the receiver");
            rc = EXIT_ERROR;
            break;
        }
        INFO("rx: %" PRIu32 " frame%s received over %" PRIu32 ".%" PRIu32 " s",
             result.received, result.received == 1 ? "" : "s",
             result.elapsed_ds / 10, result.elapsed_ds % 10);
        INFO("rx: %" PRIu32 " frames REJECTED by the chip "
             "(rx_error: bad CRC, PHR, SFD timeout)", result.rejected);
        if (settle)
            INFO("rx: %s", dw1000_probe_settle_state_name(result.settle));
        break;
    }

    case DW1000_PROBE_ROLE_TEMPERATURE: {
        struct dw1000_probe_temperature_result result =
            dw1000_probe_temperature_run(&dw0, (uint32_t)seconds,
                                         DW1000_PROBE_SAMPLE_INTERVAL_MS,
                                         node_name, line_buf_append);
        line_buf_dump();
        INFO("temperature: %" PRIu32 " sample%s", result.samples,
             result.samples == 1 ? "" : "s");
        break;
    }

    default:
        break;
    }

    /* Shut the radio down before tearing down the event thread and the
     * pins/bus it depends on. */
    dw1000_probe_port_bus_lock();
    dw1000_txrx_off(&dw0);
    dw1000_probe_port_bus_unlock();

    /* If a wake-up is already latched when the nudge below arrives, the
     * event thread's current dw1000_probe_port_wait() returns true and
     * it runs one more dw1000_process_events() before it next tests
     * event_thread_stop, and a stray rx_timeout/rx_error there would
     * re-arm the receiver right after the txrx_off() above. Harmless
     * for a process about to tear down its pins and exit, so left as
     * best-effort rather than adding a second flag to close that
     * window; a long-lived host that reused this shutdown sequence
     * would want to. */
    event_thread_stop = 1;
    dw1000_probe_port_wake(); /* nudge the event thread out of its wait */
    pthread_join(event_tid, NULL);

 pins:
    if (bitters_spi_disable(&dw1000_spi) < 0)
        WARN_ERRNO("unable to disable spi for dw1000");
    if (bitters_gpio_pin_disable(&dw1000_irq) < 0 ||
        bitters_gpio_pin_disable(&dw1000_wakeup) < 0 ||
        bitters_gpio_pin_disable(&dw1000_reset) < 0)
        WARN_ERRNO("unable to disable gpio for dw1000");

    free(line_buf);

    return rc;
}
