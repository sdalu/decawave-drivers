/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    record.c
 * @brief   The four lines, and the one piece of arithmetic behind them.
 *
 * @addtogroup PROBE
 * @{
 */

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include <dw1000/probe/record.h>
#include <dw1000/probe/role.h>

/*===========================================================================*/
/* Status                                                                    */
/*===========================================================================*/

const char *
dw1000_probe_status_name(dw1000_probe_status_t status)
{
    switch (status) {
    case DW1000_PROBE_STATUS_OK:           return "ok";
    case DW1000_PROBE_STATUS_NO_POLL:      return "no-poll";
    case DW1000_PROBE_STATUS_NO_RESPONSE:  return "no-response";
    case DW1000_PROBE_STATUS_NO_FINAL:     return "no-final";
    case DW1000_PROBE_STATUS_NO_REPORT:    return "no-report";
    case DW1000_PROBE_STATUS_BAD_DISTANCE: return "bad-distance";
    case DW1000_PROBE_STATUS__COUNT:       break;
    }
    return "invalid";
}

/*===========================================================================*/
/* Which exchange                                                           */
/*===========================================================================*/

const char *
dw1000_probe_exchange_name(dw1000_probe_exchange_t exchange)
{
    switch (exchange) {
    case DW1000_PROBE_EXCHANGE_SS:      return "ss";
    case DW1000_PROBE_EXCHANGE_DS:      return "ds";
    case DW1000_PROBE_EXCHANGE__COUNT:  break;
    }
    return "invalid";
}

/*===========================================================================*/
/* Distances                                                                */
/*===========================================================================*/

/* Stated here rather than taken from <dw1000/dw1000.h>: this component is
 * deliberately free of that header, and that freedom is what lets the
 * format be proved with no chip.
 */
#define SPEED_OF_LIGHT   299792458.0             /* m/s, exact            */
#define TIME_CLOCK_HZ    (499200000.0 * 128.0)   /* device ticks/s        */

static const double tenths_per_tick =
    10.0 * 1000.0 * SPEED_OF_LIGHT / TIME_CLOCK_HZ;

/* Signed wrapping delta between two 40-bit device timestamps. The
 * counter wraps, and the symmetric/Neirynck formulas subtract intervals
 * that can legitimately come out the other way round, so an unsigned
 * subtraction here is exactly the bug this helper exists to prevent.
 */
static int64_t
delta40(uint64_t later, uint64_t earlier)
{
    uint64_t d  = (later - earlier) & 0xFFFFFFFFFFull; /* 40-bit mask */
    int64_t  sd = (int64_t)d;

    if (d > 0x7FFFFFFFFFull)
	sd -= (int64_t)0x10000000000ull; /* take the short way round */
    return sd;
}

/* num/den, an exact rational up to here, converted to tenths of a
 * millimetre only at the very end.
 */
static int32_t
ticks_to_dmm(int64_t num, int64_t den)
{
    return (int32_t)llround((double)num / (double)den * tenths_per_tick);
}

bool
dw1000_probe_distances(struct dw1000_probe_record *record)
{
    int64_t round1, reply1, round2, reply2;
    int64_t sum;

    if ((record->present & DW1000_PROBE_F_INSTANTS_SS)
	    != DW1000_PROBE_F_INSTANTS_SS)
	return false;

    round1 = delta40(record->t_rr, record->t_sp);
    reply1 = delta40(record->t_sr, record->t_rp);

    record->ss_dmm = ticks_to_dmm(round1 - reply1, 2);
    record->present |= DW1000_PROBE_F_SS;

    if ((record->present & DW1000_PROBE_F_INSTANTS) == DW1000_PROBE_F_INSTANTS) {
	round2 = delta40(record->t_rf, record->t_sr);
	reply2 = delta40(record->t_sf, record->t_rr);

	sum = round1 + reply1 + round2 + reply2;
	if (sum != 0) {
	    /* round1*round2 and reply1*reply2 each reach ~1.6e16 for a
	     * 2 ms reply: past a double's 53-bit mantissa but well
	     * inside int64_t, so both products and their difference are
	     * taken here, in int64_t, before any float is involved.
	     */
	    int64_t asym_num = round1 * round2 - reply1 * reply2;

	    record->sym_dmm  = ticks_to_dmm((round1 - reply1) + (round2 - reply2), 4);
	    record->asym_dmm = ticks_to_dmm(asym_num, sum);
	    record->present |= DW1000_PROBE_F_SYM | DW1000_PROBE_F_ASYM;
	}
    }

    return true;
}

/*===========================================================================*/
/* Power packing                                                            */
/*===========================================================================*/

bool
dw1000_probe_pack_power(double dbm, uint16_t *out)
{
    double scaled;
    long   v;

    if (!isfinite(dbm))
	return false;

    scaled = -dbm * 100.0;
    v = lround(scaled);
    if (v < 0)
	v = 0;
    else if (v > 65535)
	v = 65535;

    *out = (uint16_t)v;
    return true;
}

/*===========================================================================*/
/* Field formatting                                                         */
/*===========================================================================*/

/* Each helper renders one optional or origin field into the caller's
 * scratch buffer and returns the pointer to use in the final snprintf():
 * either that buffer, or a string literal for the absent/default case.
 * None of them can overflow their own scratch buffer: every field this
 * record carries is a fixed-width integer, so 24 bytes is slack for the
 * widest of them (a signed 64-bit tick count) with room to spare.
 */
#define DW1000_PROBE_FIELD_BUFSZ 24

static const char *
fmt_absent(uint32_t present, uint32_t bit)
{
    return (present & bit) ? NULL : "-";
}

static const char *
fmt_u64(char *buf, size_t len, uint32_t present, uint32_t bit, uint64_t v)
{
    const char *absent = fmt_absent(present, bit);
    if (absent != NULL)
	return absent;
    snprintf(buf, len, "%" PRIu64, v);
    return buf;
}

static const char *
fmt_u16(char *buf, size_t len, uint32_t present, uint32_t bit, uint16_t v)
{
    const char *absent = fmt_absent(present, bit);
    if (absent != NULL)
	return absent;
    snprintf(buf, len, "%u", (unsigned)v);
    return buf;
}

static const char *
fmt_i16(char *buf, size_t len, uint32_t present, uint32_t bit, int16_t v)
{
    const char *absent = fmt_absent(present, bit);
    if (absent != NULL)
	return absent;
    snprintf(buf, len, "%d", (int)v);
    return buf;
}

/* One decimal, tenths taken as a signed magnitude: the sign is taken
 * once, off the whole value, and the two digits either side of the point
 * are both magnitude: -125 is "-12.5", not "-12.-5", and -5 is "-0.5"
 * rather than rounding to "-1.5" or dropping the sign on the fraction.
 */
static void
fmt_tenths_core(char *buf, size_t len, int64_t v_x10)
{
    int64_t mag = v_x10 < 0 ? -v_x10 : v_x10;
    snprintf(buf, len, "%s%" PRId64 ".%" PRId64,
	     v_x10 < 0 ? "-" : "", mag / 10, mag % 10);
}

static const char *
fmt_tenths(char *buf, size_t len, uint32_t present, uint32_t bit, int32_t v)
{
    const char *absent = fmt_absent(present, bit);
    if (absent != NULL)
	return absent;
    fmt_tenths_core(buf, len, v);
    return buf;
}

/* tx_power_db_x10: a read-back applied value, always present. */
static const char *
fmt_tenths_always(char *buf, size_t len, int16_t v)
{
    fmt_tenths_core(buf, len, v);
    return buf;
}

/* tx_power_req_db_x10: -1 is the automatic setting, printed "auto"
 * rather than the "-0.1" the bit pattern would otherwise give.
 */
static const char *
fmt_tx_power_req(char *buf, size_t len, int16_t v)
{
    if (v == -1)
	return "auto";
    fmt_tenths_core(buf, len, v);
    return buf;
}

static const char *
fmt_str(const char *s)
{
    return (s != NULL && s[0] != '\0') ? s : "-";
}

/*===========================================================================*/
/* The lines                                                                 */
/*===========================================================================*/

size_t
dw1000_probe_record_format(char *buf, size_t len,
		    const struct dw1000_probe_record *record,
		    const struct dw1000_probe_origin *origin)
{
    uint32_t p = record->present;

    char b_t_sp[DW1000_PROBE_FIELD_BUFSZ], b_t_rp[DW1000_PROBE_FIELD_BUFSZ];
    char b_t_sr[DW1000_PROBE_FIELD_BUFSZ], b_t_rr[DW1000_PROBE_FIELD_BUFSZ];
    char b_t_sf[DW1000_PROBE_FIELD_BUFSZ], b_t_rf[DW1000_PROBE_FIELD_BUFSZ];
    char b_sym[DW1000_PROBE_FIELD_BUFSZ], b_asym[DW1000_PROBE_FIELD_BUFSZ];
    char b_resp_temp[DW1000_PROBE_FIELD_BUFSZ], b_resp_vbat[DW1000_PROBE_FIELD_BUFSZ];
    char b_init_temp[DW1000_PROBE_FIELD_BUFSZ], b_init_vbat[DW1000_PROBE_FIELD_BUFSZ];
    char b_resp_rx[DW1000_PROBE_FIELD_BUFSZ], b_resp_fp[DW1000_PROBE_FIELD_BUFSZ];
    char b_init_rx[DW1000_PROBE_FIELD_BUFSZ], b_init_fp[DW1000_PROBE_FIELD_BUFSZ];
    char b_ss[DW1000_PROBE_FIELD_BUFSZ];

    return (size_t)snprintf(buf, len,
	"TWR seq=%u"
	" t_sp=%s t_rp=%s t_sr=%s t_rr=%s t_sf=%s t_rf=%s"
	" sym_mm=%s asym_mm=%s"
	" resp_temp=%s resp_vbat=%s init_temp=%s init_vbat=%s"
	" resp_rx=%s resp_fp=%s init_rx=%s init_fp=%s"
	" exchange=%s ss_mm=%s"
	" status=%s node=%s role=%s run=%s",
	(unsigned)record->seq,
	fmt_u64(b_t_sp, sizeof(b_t_sp), p, DW1000_PROBE_F_T_SP, record->t_sp),
	fmt_u64(b_t_rp, sizeof(b_t_rp), p, DW1000_PROBE_F_T_RP, record->t_rp),
	fmt_u64(b_t_sr, sizeof(b_t_sr), p, DW1000_PROBE_F_T_SR, record->t_sr),
	fmt_u64(b_t_rr, sizeof(b_t_rr), p, DW1000_PROBE_F_T_RR, record->t_rr),
	fmt_u64(b_t_sf, sizeof(b_t_sf), p, DW1000_PROBE_F_T_SF, record->t_sf),
	fmt_u64(b_t_rf, sizeof(b_t_rf), p, DW1000_PROBE_F_T_RF, record->t_rf),
	fmt_tenths(b_sym,  sizeof(b_sym),  p, DW1000_PROBE_F_SYM,  record->sym_dmm),
	fmt_tenths(b_asym, sizeof(b_asym), p, DW1000_PROBE_F_ASYM, record->asym_dmm),
	fmt_i16(b_resp_temp, sizeof(b_resp_temp),
		p, DW1000_PROBE_F_RESP_TEMP, record->resp_temp),
	fmt_u16(b_resp_vbat, sizeof(b_resp_vbat),
		p, DW1000_PROBE_F_RESP_VBAT, record->resp_vbat),
	fmt_i16(b_init_temp, sizeof(b_init_temp),
		p, DW1000_PROBE_F_INIT_TEMP, record->init_temp),
	fmt_u16(b_init_vbat, sizeof(b_init_vbat),
		p, DW1000_PROBE_F_INIT_VBAT, record->init_vbat),
	fmt_u16(b_resp_rx, sizeof(b_resp_rx), p, DW1000_PROBE_F_RESP_RX, record->resp_rx),
	fmt_u16(b_resp_fp, sizeof(b_resp_fp), p, DW1000_PROBE_F_RESP_FP, record->resp_fp),
	fmt_u16(b_init_rx, sizeof(b_init_rx), p, DW1000_PROBE_F_INIT_RX, record->init_rx),
	fmt_u16(b_init_fp, sizeof(b_init_fp), p, DW1000_PROBE_F_INIT_FP, record->init_fp),
	dw1000_probe_exchange_name(record->exchange),
	fmt_tenths(b_ss, sizeof(b_ss), p, DW1000_PROBE_F_SS, record->ss_dmm),
	dw1000_probe_status_name(record->status),
	fmt_str(origin->node),
	dw1000_probe_role_name(origin->role),
	fmt_str(origin->run));
}

size_t
dw1000_probe_stats_format(char *buf, size_t len,
		   const struct dw1000_probe_stats *stats,
		   const struct dw1000_probe_origin *origin)
{
    char b_applied[DW1000_PROBE_FIELD_BUFSZ], b_req[DW1000_PROBE_FIELD_BUFSZ];

    /* The reception account goes between `of=` and the origin, so that
       the origin keys stay last as they are on every other line. A
       positional reader of the STATS line would break on any insertion
       at all; this line has never had one, and record.h states the key
       set as the contract. */
    return (size_t)snprintf(buf, len,
	"STATS role=%s tx_power_db=%s tx_power_req_db=%s driver=%s"
	" completed=%u of=%u"
	" heard=%u drop_unwatched=%u drop_overrun=%u drop_foreign=%u"
	" drop_short=%u drop_type=%u drop_dst=%u drop_seq=%u"
	" node=%s run=%s",
	dw1000_probe_role_name(origin->role),
	fmt_tenths_always(b_applied, sizeof(b_applied), stats->tx_power_db_x10),
	fmt_tx_power_req(b_req, sizeof(b_req), stats->tx_power_req_db_x10),
	fmt_str(stats->driver_version),
	(unsigned)stats->resolved,
	(unsigned)stats->attempted,
	(unsigned)stats->heard,
	(unsigned)stats->drop_unwatched,
	(unsigned)stats->drop_overrun,
	(unsigned)stats->drop_foreign,
	(unsigned)stats->drop_short,
	(unsigned)stats->drop_type,
	(unsigned)stats->drop_dst,
	(unsigned)stats->drop_seq,
	fmt_str(origin->node),
	fmt_str(origin->run));
}

size_t
dw1000_probe_ready_format(char *buf, size_t len,
		   const struct dw1000_probe_origin *origin)
{
    return (size_t)snprintf(buf, len,
	"READY role=%s node=%s run=%s",
	dw1000_probe_role_name(origin->role),
	fmt_str(origin->node),
	fmt_str(origin->run));
}

size_t
dw1000_probe_temp_format(char *buf, size_t len,
		  uint32_t elapsed_ds, int16_t temp, uint16_t vbat,
		  const struct dw1000_probe_origin *origin)
{
    return (size_t)snprintf(buf, len,
	"TEMP %u.%u %d %u node=%s run=%s",
	(unsigned)(elapsed_ds / 10), (unsigned)(elapsed_ds % 10),
	(int)temp, (unsigned)vbat,
	fmt_str(origin->node),
	fmt_str(origin->run));
}

/** @} */
