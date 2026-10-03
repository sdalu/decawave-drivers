/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * The radio a run uses: see <dw1000/probe/radio.h>. Every rule about what
 * the chip accepts is the driver's (dw1000_validate_*(),
 * dw1000_radio_is_valid()); what is here is spelling, units and the
 * line.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dw1000/dw1000.h>
#include <dw1000/dw1000_state.h>
#include <dw1000/dw1000_validate.h>
#include <dw1000/probe/port.h>
#include <dw1000/probe/record.h>
#include <dw1000/probe/role.h>
#include <dw1000/probe/radio.h>


static void
say(const char **errmsg, const char *why)
{
    if (errmsg != NULL)
        *errmsg = why;
}


/*======================================================================*/
/* Settings                                                             */
/*======================================================================*/

void
dw1000_probe_radio_default(struct dw1000_probe_radio *radio,
                           uint16_t tx_antenna_delay,
                           uint16_t rx_antenna_delay)
{
    radio->channel          = 5;
    radio->bitrate_kbps     = 6800;
    radio->prf_mhz          = 64;
    radio->preamble         = 128;
    radio->pac              = 8;
    radio->tx_code          = 10;
    radio->rx_code          = 10;
    radio->sfd_decawave     = DW1000_WITH_PROPRIETARY_SFD ? true : false;
    radio->tx_antenna_delay = tx_antenna_delay;
    radio->rx_antenna_delay = rx_antenna_delay;
}


/*======================================================================*/
/* Spelling                                                             */
/*======================================================================*/

/* A whole number, as typed: digits only, so that "5x" and "-1" are not
 * read as 5 and as a large unsigned. */
static bool
whole(const char *s, unsigned long max, unsigned long *out)
{
    char *end;
    unsigned long v;

    if (*s < '0' || *s > '9')
        return false;
    v = strtoul(s, &end, 10);
    if (*end != '\0' || v > max)
        return false;
    *out = v;
    return true;
}

/* The option's value when @p arg is `--name=value`, else NULL. */
static const char *
value_of(const char *arg, const char *name)
{
    size_t n = strlen(name);

    if (strncmp(arg, name, n) != 0 || arg[n] != '=')
        return NULL;
    return arg + n + 1;
}

dw1000_probe_radio_option_t
dw1000_probe_radio_option(struct dw1000_probe_radio *radio, const char *arg,
                          const char **errmsg)
{
    const char *v;
    unsigned long n;

    say(errmsg, NULL);

    /* Each numeric field: parsed, then checked on its own by the driver's
     * validator, whose message says what the accepted values are. */
#define FIELD(name, validate, field, max)                               \
    if ((v = value_of(arg, name)) != NULL) {                            \
        if (!whole(v, (max), &n)) {                                     \
            say(errmsg, name " takes a whole number");                  \
            return DW1000_PROBE_RADIO_OPTION_BAD;                       \
        }                                                               \
        if (!validate((int)n, NULL, errmsg))                            \
            return DW1000_PROBE_RADIO_OPTION_BAD;                       \
        field;                                                          \
        return DW1000_PROBE_RADIO_OPTION_SET;                           \
    }

    FIELD("--channel",  dw1000_validate_channel, radio->channel      = (uint8_t)n,  255)
    FIELD("--bitrate",  dw1000_validate_bitrate, radio->bitrate_kbps = (uint16_t)n, 65535)
    FIELD("--prf",      dw1000_validate_prf,     radio->prf_mhz      = (uint8_t)n,  255)
    FIELD("--preamble", dw1000_validate_plen,    radio->preamble     = (uint16_t)n, 65535)
    FIELD("--pac",      dw1000_validate_pac,     radio->pac          = (uint8_t)n,  255)
    FIELD("--code",     dw1000_validate_pcode,
          radio->tx_code = radio->rx_code = (uint8_t)n, 255)
    FIELD("--tx-code",  dw1000_validate_pcode,   radio->tx_code      = (uint8_t)n,  255)
    FIELD("--rx-code",  dw1000_validate_pcode,   radio->rx_code      = (uint8_t)n,  255)
#undef FIELD

    if ((v = value_of(arg, "--sfd")) != NULL) {
        if (strcmp(v, "decawave") == 0) {
#if DW1000_WITH_PROPRIETARY_SFD
            radio->sfd_decawave = true;
#else
            say(errmsg, "the Decawave SFD needs a driver built with "
                        "DW1000_WITH_PROPRIETARY_SFD");
            return DW1000_PROBE_RADIO_OPTION_BAD;
#endif
        } else if (strcmp(v, "standard") == 0) {
            radio->sfd_decawave = false;
        } else {
            say(errmsg, "--sfd is decawave or standard");
            return DW1000_PROBE_RADIO_OPTION_BAD;
        }
        return DW1000_PROBE_RADIO_OPTION_SET;
    }

    /* Antenna delays, in device ticks, one way: the unit the chip holds
     * and the SETUP line prints, so a value read off one capture can be
     * typed into the next run unchanged. Every 16-bit value is a delay
     * the chip takes. */
    {
        static const char *const names[] = {
            "--antenna-delay", "--tx-antenna-delay", "--rx-antenna-delay",
        };
        size_t i;

        for (i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
            if ((v = value_of(arg, names[i])) == NULL)
                continue;
            if (!whole(v, 65535, &n)) {
                say(errmsg, "an antenna delay is device ticks, 0 .. 65535");
                return DW1000_PROBE_RADIO_OPTION_BAD;
            }
            if (i != 2) radio->tx_antenna_delay = (uint16_t)n;
            if (i != 1) radio->rx_antenna_delay = (uint16_t)n;
            return DW1000_PROBE_RADIO_OPTION_SET;
        }
    }

    return DW1000_PROBE_RADIO_OPTION_NONE;
}


/*======================================================================*/
/* Validation                                                           */
/*======================================================================*/

bool
dw1000_probe_radio_resolve(const struct dw1000_probe_radio *radio,
                           uint8_t tx_power, struct dw1000_radio *out,
                           const char **errmsg)
{
    memset(out, 0, sizeof(*out));
    say(errmsg, NULL);

    if (!dw1000_validate_channel(radio->channel,      &out->channel,  errmsg) ||
        !dw1000_validate_bitrate(radio->bitrate_kbps, &out->bitrate,  errmsg) ||
        !dw1000_validate_prf    (radio->prf_mhz,      &out->prf,      errmsg) ||
        !dw1000_validate_plen   (radio->preamble,     &out->tx_plen,  errmsg) ||
        !dw1000_validate_pac    (radio->pac,          &out->rx_pac,   errmsg) ||
        !dw1000_validate_pcode  (radio->tx_code,      &out->tx_pcode, errmsg) ||
        !dw1000_validate_pcode  (radio->rx_code,      &out->rx_pcode, errmsg))
        return false;

#if DW1000_WITH_PROPRIETARY_SFD
    out->proprietary.sfd = radio->sfd_decawave ? 1 : 0;
#else
    if (radio->sfd_decawave) {
        say(errmsg, "the Decawave SFD needs a driver built with "
                    "DW1000_WITH_PROPRIETARY_SFD");
        return false;
    }
#endif
    out->tx_power = tx_power;

    /* Every field passed on its own, so a refusal here is the
     * combination's. The message names the one rule across fields the
     * driver has; the verdict is the driver's, whatever the message. */
    if (!dw1000_radio_is_valid(out)) {
        say(errmsg, "the driver refuses this combination: a preamble code "
                    "must suit the PRF (1..8 at 16 MHz, 9..24 at 64 MHz, "
                    "UM 10.5)");
        return false;
    }
    return true;
}


/*======================================================================*/
/* The line                                                             */
/*======================================================================*/

static unsigned
pac_symbols(uint8_t pac)
{
    switch (pac) {
    case DW1000_PAC8:  return 8;
    case DW1000_PAC16: return 16;
    case DW1000_PAC32: return 32;
    case DW1000_PAC64: return 64;
    }
    return 0;
}

static const char *
fmt_str(const char *s)
{
    return (s != NULL && s[0] != '\0') ? s : "-";
}

size_t
dw1000_probe_radio_format(char *buf, size_t len, dw1000_t *dw,
                          const struct dw1000_probe_origin *origin)
{
    dw1000_radio_state_t st;
    uint8_t  pac;
    unsigned power;
    int      n;

    dw1000_probe_port_bus_lock();
    dw1000_get_radio_state(dw, &st);
    pac = dw->radio.rx_pac;
    dw1000_probe_port_bus_unlock();

    power = dw1000_tx_power_to_05db(st.tx_power) * 5u;

    n = snprintf(buf, len,
        "SETUP channel=%u bitrate=%u prf=%u preamble=%u pac=%u"
        " tx_code=%u rx_code=%u sfd=%s tx_antd=%u rx_antd=%u"
        " tx_power_db=%u.%u",
        (unsigned)st.tx_channel, (unsigned)st.bitrate_kbps,
        (unsigned)st.rx_prf, (unsigned)st.tx_plen, pac_symbols(pac),
        (unsigned)st.tx_pcode, (unsigned)st.rx_pcode,
        st.dwsfd ? "decawave" : "standard",
        (unsigned)st.tx_antenna_delay, (unsigned)st.rx_antenna_delay,
        power / 10, power % 10);
    if (n < 0)
        return 0;
    if (origin == NULL)
        return (size_t)n;

    size_t used = (size_t)n < len ? (size_t)n : (len > 0 ? len - 1 : 0);
    int m = snprintf(buf + used, len - used, " node=%s role=%s run=%s",
                     fmt_str(origin->node),
                     dw1000_probe_role_name(origin->role),
                     fmt_str(origin->run));
    return (size_t)n + (m < 0 ? 0 : (size_t)m);
}

void
dw1000_probe_radio_emit(dw1000_t *dw, dw1000_probe_role_t role,
                        const char *node_name)
{
    const struct dw1000_probe_origin origin = {
        .node = node_name,
        .role = role,
        .run  = NULL,
    };
    char line[DW1000_PROBE_RECORD_MAX];

    dw1000_probe_radio_format(line, sizeof(line), dw, &origin);
    dw1000_probe_port_emit(line);
}
