/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Stage 0 of the probe: the format is provable with no chip, no driver
 * and no radio, which is the whole point of keeping probe/include/probe/
 * free of <dw1000/dw1000.h>. This links only probe/src (role and record)
 * and port/emulation, and checks:
 *
 *  - the complete TWR line, byte for byte, against the jig in the brief;
 *  - that a cleared DW1000_PROBE_F_* bit prints '-' and nothing else moves;
 *  - the sign/magnitude split for negative tenths-of-a-mm distances;
 *  - NULL and empty origin strings, both printing '-';
 *  - every status and role name, canonical and out of range;
 *  - dw1000_probe_role_lookup()'s round trip and its rejections;
 *  - dw1000_probe_pack_power()'s rounding, clamp and non-finite rejection;
 *  - that a too-small buffer reports the untruncated length and writes
 *    nothing past it;
 *  - the STATS, READY and TEMP lines;
 *  - and, since it is linked in regardless, that the emulation port's
 *    clock actually moves;
 *  - dw1000_probe_distances(): no-drift, drift (single-sided, symmetric
 *    and Neirynck disagreeing by the stated amounts), the 40-bit counter
 *    wrap, single-sided only, a missing instant, and the four intervals
 *    summing to zero;
 *  - dw1000_probe_exchange_name(), canonical and out of range.
 *
 * One line per case; a failing case says why on its own line. Run by
 * tests/check-probe.sh.
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <dw1000/probe/port.h>
#include <dw1000/probe/record.h>
#include <dw1000/probe/role.h>

static int failures;

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

/* Where a case formats the reason it failed for. One case runs at a
 * time, and the reason is printed before the next one starts.
 */
static char reason[256];

#define REASON(...) (snprintf(reason, sizeof(reason), __VA_ARGS__), reason)


/*----------------------------------------------------------------------*/
/* 1. The complete record, byte for byte                                */
/*----------------------------------------------------------------------*/

static const char *
case_complete_record(void)
{
    static const char expected[] =
	"TWR seq=7 t_sp=1000000 t_rp=2000000 t_sr=3000000 t_rr=4000000"
	" t_sf=5000000 t_rf=6000000 sym_mm=362.5 asym_mm=-12.5"
	" resp_temp=2537 resp_vbat=3300 init_temp=-150 init_vbat=3280"
	" resp_rx=8825 resp_fp=9012 init_rx=8790 init_fp=8955"
	" exchange=ds ss_mm=1032.2"
	" status=ok node=D4 role=twr_resp run=r1";
    struct dw1000_probe_record r = {
	.seq = 7, .status = DW1000_PROBE_STATUS_OK, .present = DW1000_PROBE_F_ALL,
	.t_sp = 1000000, .t_rp = 2000000, .t_sr = 3000000,
	.t_rr = 4000000, .t_sf = 5000000, .t_rf = 6000000,
	.exchange = DW1000_PROBE_EXCHANGE_DS,
	.ss_dmm = 10322, .sym_dmm = 3625, .asym_dmm = -125,
	.resp_temp = 2537, .resp_vbat = 3300,
	.init_temp = -150, .init_vbat = 3280,
	.resp_rx = 8825, .resp_fp = 9012,
	.init_rx = 8790, .init_fp = 8955,
    };
    struct dw1000_probe_origin o = {
	.node = "D4", .role = DW1000_PROBE_ROLE_TWR_RESP, .run = "r1",
    };
    char buf[DW1000_PROBE_RECORD_MAX];
    size_t n;

    n = dw1000_probe_record_format(buf, sizeof(buf), &r, &o);
    if (n != strlen(expected))
	return REASON("length %zu, want %zu", n, strlen(expected));
    if (strcmp(buf, expected) != 0)
	return REASON("got '%s'", buf);
    return NULL;
}


/*----------------------------------------------------------------------*/
/* 2. A cleared bit prints '-'                                          */
/*----------------------------------------------------------------------*/

static const char *
case_cleared_bits(void)
{
    struct dw1000_probe_record base = {
	.seq = 7, .status = DW1000_PROBE_STATUS_OK, .present = DW1000_PROBE_F_ALL,
	.t_sp = 1000000, .t_rp = 2000000, .t_sr = 3000000,
	.t_rr = 4000000, .t_sf = 5000000, .t_rf = 6000000,
	.sym_dmm = 3625, .asym_dmm = -125,
	.resp_temp = 2537, .resp_vbat = 3300,
	.init_temp = -150, .init_vbat = 3280,
	.resp_rx = 8825, .resp_fp = 9012,
	.init_rx = 8790, .init_fp = 8955,
    };
    struct dw1000_probe_origin o = {
	.node = "D4", .role = DW1000_PROBE_ROLE_TWR_RESP, .run = "r1",
    };
    /* Leading space in the needle so that "sym_mm=-" cannot match inside
     * "asym_mm=..." -- "asym_mm" contains "sym_mm" as a substring.
     */
    static const struct { uint32_t bit; const char *needle; } t[] = {
	{ DW1000_PROBE_F_INIT_RX,   " init_rx=-"   },
	{ DW1000_PROBE_F_INIT_TEMP, " init_temp=-" },
	{ DW1000_PROBE_F_SYM,       " sym_mm=-"    },
	{ DW1000_PROBE_F_ASYM,      " asym_mm=-"   },
    };
    char buf[DW1000_PROBE_RECORD_MAX];
    size_t i;

    for (i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
	struct dw1000_probe_record r = base;
	r.present = DW1000_PROBE_F_ALL & ~t[i].bit;
	dw1000_probe_record_format(buf, sizeof(buf), &r, &o);
	if (strstr(buf, t[i].needle) == NULL)
	    return REASON("clearing 0x%x: '%s' not in '%s'",
			  t[i].bit, t[i].needle, buf);
    }
    return NULL;
}


/*----------------------------------------------------------------------*/
/* 3. Negative distances                                                */
/*----------------------------------------------------------------------*/

static const char *
case_negative_distances(void)
{
    static const struct { int32_t v; const char *want; } t[] = {
	{ -125, "-12.5" },
	{ -5,   "-0.5"  },
	{ 0,    "0.0"   },
    };
    struct dw1000_probe_record r = { .status = DW1000_PROBE_STATUS_OK, .present = DW1000_PROBE_F_SYM };
    struct dw1000_probe_origin o = { .node = "N", .role = DW1000_PROBE_ROLE_TWR_RESP, .run = "r" };
    char buf[DW1000_PROBE_RECORD_MAX];
    char needle[64];
    size_t i;

    for (i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
	r.sym_dmm = t[i].v;
	dw1000_probe_record_format(buf, sizeof(buf), &r, &o);
	/* asym is absent throughout, so the value ends unambiguously at
	 * " asym_mm=-".
	 */
	snprintf(needle, sizeof(needle), " sym_mm=%s asym_mm=-", t[i].want);
	if (strstr(buf, needle) == NULL)
	    return REASON("sym_dmm=%d: '%s' not in '%s'",
			  t[i].v, needle, buf);
    }
    return NULL;
}


/*----------------------------------------------------------------------*/
/* 4. NULL and empty origin strings                                     */
/*----------------------------------------------------------------------*/

static const char *
case_null_and_empty_origin(void)
{
    struct dw1000_probe_origin null_o  = { .node = NULL, .role = DW1000_PROBE_ROLE_RX, .run = NULL };
    struct dw1000_probe_origin empty_o = { .node = "",   .role = DW1000_PROBE_ROLE_RX, .run = "" };
    char buf[128];

    dw1000_probe_ready_format(buf, sizeof(buf), &null_o);
    if (strcmp(buf, "READY role=rx node=- run=-") != 0)
	return REASON("NULL node/run: got '%s'", buf);

    dw1000_probe_ready_format(buf, sizeof(buf), &empty_o);
    if (strcmp(buf, "READY role=rx node=- run=-") != 0)
	return REASON("empty node/run: got '%s'", buf);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 5. Every status and role name, including out of range                */
/*----------------------------------------------------------------------*/

static const char *
case_status_names(void)
{
    static const char *want[] = {
	"ok", "no-response", "no-final", "no-report", "bad-distance",
    };
    size_t i;
    const char *got;

    for (i = 0; i < sizeof(want) / sizeof(want[0]); i++) {
	got = dw1000_probe_status_name((dw1000_probe_status_t)i);
	if (strcmp(got, want[i]) != 0)
	    return REASON("status %zu: got '%s', want '%s'", i, got, want[i]);
    }
    if (strcmp(dw1000_probe_status_name(DW1000_PROBE_STATUS__COUNT), "invalid") != 0)
	return "status __COUNT should be 'invalid'";
    if (strcmp(dw1000_probe_status_name((dw1000_probe_status_t)999), "invalid") != 0)
	return "status 999 should be 'invalid'";
    return NULL;
}

static const char *
case_role_names(void)
{
    static const char *want[] = {
	"twr_init", "twr_resp", "tx", "rx", "temperature",
    };
    size_t i;
    const char *got;

    for (i = 0; i < sizeof(want) / sizeof(want[0]); i++) {
	got = dw1000_probe_role_name((dw1000_probe_role_t)i);
	if (strcmp(got, want[i]) != 0)
	    return REASON("role %zu: got '%s', want '%s'", i, got, want[i]);
    }
    if (strcmp(dw1000_probe_role_name(DW1000_PROBE_ROLE__COUNT), "invalid") != 0)
	return "role __COUNT should be 'invalid'";
    if (strcmp(dw1000_probe_role_name((dw1000_probe_role_t)999), "invalid") != 0)
	return "role 999 should be 'invalid'";
    return NULL;
}


/*----------------------------------------------------------------------*/
/* 6. dw1000_probe_role_lookup()                                               */
/*----------------------------------------------------------------------*/

static const char *
case_role_lookup(void)
{
    size_t i;
    dw1000_probe_role_t r;

    for (i = 0; i < DW1000_PROBE_ROLE__COUNT; i++) {
	const char *name = dw1000_probe_role_name((dw1000_probe_role_t)i);
	if (!dw1000_probe_role_lookup(name, &r))
	    return REASON("lookup('%s') failed", name);
	if (r != (dw1000_probe_role_t)i)
	    return REASON("lookup('%s') gave %d, want %zu",
			  name, (int)r, i);
    }
    if (dw1000_probe_role_lookup("resp", &r))
	return "lookup('resp') should have failed";
    if (dw1000_probe_role_lookup("", &r))
	return "lookup('') should have failed";
    return NULL;
}


/*----------------------------------------------------------------------*/
/* 7. dw1000_probe_pack_power()                                                */
/*----------------------------------------------------------------------*/

static const char *
case_pack_power(void)
{
    uint16_t out;

    if (!dw1000_probe_pack_power(-88.25, &out) || out != 8825)
	return REASON("-88.25 -> %u, want 8825", out);

    if (!dw1000_probe_pack_power(-1000.0, &out) || out != 65535)
	return REASON("clamp: -1000.0 -> %u, want 65535", out);

    if (!dw1000_probe_pack_power(0.0, &out) || out != 0)
	return REASON("0.0 -> %u, want 0", out);

    if (dw1000_probe_pack_power(INFINITY, &out))
	return "INFINITY should return false";
    if (dw1000_probe_pack_power(-INFINITY, &out))
	return "-INFINITY should return false";
    if (dw1000_probe_pack_power(NAN, &out))
	return "NAN should return false";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 8. Truncation                                                        */
/*----------------------------------------------------------------------*/

static const char *
case_truncation(void)
{
    struct dw1000_probe_record r = {
	.seq = 7, .status = DW1000_PROBE_STATUS_OK, .present = DW1000_PROBE_F_ALL,
	.t_sp = 1000000, .t_rp = 2000000, .t_sr = 3000000,
	.t_rr = 4000000, .t_sf = 5000000, .t_rf = 6000000,
	.sym_dmm = 3625, .asym_dmm = -125,
	.resp_temp = 2537, .resp_vbat = 3300,
	.init_temp = -150, .init_vbat = 3280,
	.resp_rx = 8825, .resp_fp = 9012,
	.init_rx = 8790, .init_fp = 8955,
    };
    struct dw1000_probe_origin o = {
	.node = "D4", .role = DW1000_PROBE_ROLE_TWR_RESP, .run = "r1",
    };
    char   full[DW1000_PROBE_RECORD_MAX];
    size_t want;

    /* A small buffer with one guard byte right after it, in the same
     * array -- so nothing but the compiler's own padding rules can
     * separate them, unlike two struct members.
     */
    char   arena[16];
    size_t small_len = 8;
    size_t got;

    want = dw1000_probe_record_format(full, sizeof(full), &r, &o);

    arena[small_len] = (char)0xAA;
    got = dw1000_probe_record_format(arena, small_len, &r, &o);

    if (got != want)
	return REASON("truncated return %zu, want %zu", got, want);
    if (got < small_len)
	return "return value does not indicate truncation";
    if (arena[small_len] != (char)0xAA)
	return "the guard byte after the buffer was overwritten";
    if (memcmp(arena, full, small_len - 1) != 0)
	return "truncated bytes do not match the full line's prefix";
    if (arena[small_len - 1] != '\0')
	return "truncated buffer is not NUL-terminated";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 9. STATS, READY, TEMP                                                */
/*----------------------------------------------------------------------*/

static const char *
case_stats_ready_temp(void)
{
    struct dw1000_probe_stats st = {
	.tx_power_db_x10     = 75,
	.tx_power_req_db_x10 = -1,
	.driver_version      = "1.1.0",
	.attempted           = 40,
	.resolved            = 38,
    };
    struct dw1000_probe_origin o = {
	.node = "D4", .role = DW1000_PROBE_ROLE_TWR_RESP, .run = "r1",
    };
    char buf[DW1000_PROBE_RECORD_MAX];

    dw1000_probe_stats_format(buf, sizeof(buf), &st, &o);
    if (strcmp(buf,
	    "STATS role=twr_resp tx_power_db=7.5 tx_power_req_db=auto"
	    " driver=1.1.0 completed=38 of=40 node=D4 run=r1") != 0)
	return REASON("STATS: got '%s'", buf);

    dw1000_probe_ready_format(buf, sizeof(buf), &o);
    if (strcmp(buf, "READY role=twr_resp node=D4 run=r1") != 0)
	return REASON("READY: got '%s'", buf);

    dw1000_probe_temp_format(buf, sizeof(buf), 125, 2537, 3300, &o);
    if (strcmp(buf, "TEMP 12.5 2537 3300 node=D4 run=r1") != 0)
	return REASON("TEMP: got '%s'", buf);

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 10. The emulation port's clock, since it is linked in regardless     */
/*----------------------------------------------------------------------*/

static const char *
case_port_time(void)
{
    dw1000_probe_time_t a, b;

    a = dw1000_probe_port_now();
    dw1000_probe_port_sleep(2000); /* 2 ms */
    b = dw1000_probe_port_now();

    if (b < a)
	return "dw1000_probe_port_now() went backwards";
    if (b - a == 0)
	return "dw1000_probe_port_now() did not move across dw1000_probe_port_sleep()";
    return NULL;
}


/*----------------------------------------------------------------------*/
/* 11. dw1000_probe_distances()                                         */
/*----------------------------------------------------------------------*/

/* Runs the four instants through dw1000_probe_distances() and checks all
 * three estimates against the ground truth, computed from the reference
 * implementation.
 */
static const char *
check_vector(const char *label,
	     uint64_t t_sp, uint64_t t_rp, uint64_t t_sr,
	     uint64_t t_rr, uint64_t t_sf, uint64_t t_rf,
	     int32_t want_ss, int32_t want_sym, int32_t want_asym)
{
    struct dw1000_probe_record r = {
	.present = DW1000_PROBE_F_INSTANTS,
	.t_sp = t_sp, .t_rp = t_rp, .t_sr = t_sr,
	.t_rr = t_rr, .t_sf = t_sf, .t_rf = t_rf,
    };

    if (!dw1000_probe_distances(&r))
	return REASON("%s: dw1000_probe_distances() returned false", label);
    if ((r.present & DW1000_PROBE_F_SS) == 0)
	return REASON("%s: F_SS not set", label);
    if ((r.present & (DW1000_PROBE_F_SYM | DW1000_PROBE_F_ASYM))
	    != (DW1000_PROBE_F_SYM | DW1000_PROBE_F_ASYM))
	return REASON("%s: F_SYM/F_ASYM not set", label);
    if (r.ss_dmm != want_ss)
	return REASON("%s: ss_dmm=%d, want %d", label, r.ss_dmm, want_ss);
    if (r.sym_dmm != want_sym)
	return REASON("%s: sym_dmm=%d, want %d", label, r.sym_dmm, want_sym);
    if (r.asym_dmm != want_asym)
	return REASON("%s: asym_dmm=%d, want %d", label, r.asym_dmm, want_asym);
    return NULL;
}

/* A. No clock drift: all three estimates agree. */
static const char *
case_distances_no_drift(void)
{
    return check_vector("A",
	1000000, 1000220, 26000220, 26000440, 57000440, 57000660,
	10322, 10322, 10322);
}

/* B. Responder's crystal 20 ppm fast: the three MUST disagree, and by
 * these amounts -- ss_dmm is negative, which is why the field is signed.
 */
static const char *
case_distances_drift(void)
{
    return check_vector("B",
	1000000, 1000220, 26000720, 26000440, 57000440, 57001780,
	-1408, 11729, 10322);
}

/* C. The 40-bit counter wraps mid-exchange: same intervals, same result,
 * as A.
 */
static const char *
case_distances_wrap(void)
{
    return check_vector("C",
	1099511627276ull, 1099511627496ull, 1099536627496ull,
	1099536627716ull, 1099567627716ull, 1099567627936ull,
	10322, 10322, 10322);
}

/* Only the four SS instants present: true, ss bit set, sym/asym clear. */
static const char *
case_distances_ss_only(void)
{
    struct dw1000_probe_record r = {
	.present = DW1000_PROBE_F_INSTANTS_SS,
	.t_sp = 1000000, .t_rp = 1000220,
	.t_sr = 26000220, .t_rr = 26000440,
    };

    if (!dw1000_probe_distances(&r))
	return "ss-only: dw1000_probe_distances() returned false";
    if ((r.present & DW1000_PROBE_F_SS) == 0)
	return "ss-only: F_SS not set";
    if (r.present & (DW1000_PROBE_F_SYM | DW1000_PROBE_F_ASYM))
	return "ss-only: F_SYM/F_ASYM set with no t_sf/t_rf";
    if (r.ss_dmm != 10322)
	return REASON("ss-only: ss_dmm=%d, want 10322", r.ss_dmm);
    return NULL;
}

/* One SS instant missing: false, and the record is untouched. */
static const char *
case_distances_missing_instant(void)
{
    struct dw1000_probe_record r = {
	.present = DW1000_PROBE_F_INSTANTS_SS & ~DW1000_PROBE_F_T_RR,
	.t_sp = 1000000, .t_rp = 1000220,
	.t_sr = 26000220, .t_rr = 26000440,
    };
    struct dw1000_probe_record before = r;

    if (dw1000_probe_distances(&r))
	return "missing instant: dw1000_probe_distances() returned true";
    if (memcmp(&r, &before, sizeof(r)) != 0)
	return "missing instant: the record was modified";
    return NULL;
}

/* All six present, but the four intervals sum to zero: still true, ss
 * still valid, sym/asym left clear (DW1000_PROBE_STATUS_BAD_DISTANCE).
 * round1=1000 reply1=1000 round2=1000 reply2=-3000, chosen so the sum is
 * exactly zero; reply2 is realized as a 40-bit wraparound (t_sf just
 * behind t_rr) rather than a negative timestamp.
 */
static const char *
case_distances_bad_distance(void)
{
    struct dw1000_probe_record r = {
	.present = DW1000_PROBE_F_INSTANTS,
	.t_sp = 0, .t_rp = 0, .t_sr = 1000, .t_rr = 1000,
	.t_sf = 1099511625776ull, .t_rf = 2000,
    };

    if (!dw1000_probe_distances(&r))
	return "bad-distance: dw1000_probe_distances() returned false";
    if ((r.present & DW1000_PROBE_F_SS) == 0)
	return "bad-distance: F_SS not set";
    if (r.present & (DW1000_PROBE_F_SYM | DW1000_PROBE_F_ASYM))
	return "bad-distance: F_SYM/F_ASYM set despite a zero-sum interval";
    if (r.ss_dmm != 0)
	return REASON("bad-distance: ss_dmm=%d, want 0", r.ss_dmm);
    return NULL;
}


/*----------------------------------------------------------------------*/
/* 12. dw1000_probe_exchange_name()                                     */
/*----------------------------------------------------------------------*/

static const char *
case_exchange_names(void)
{
    if (strcmp(dw1000_probe_exchange_name(DW1000_PROBE_EXCHANGE_SS), "ss") != 0)
	return "exchange SS: wrong name";
    if (strcmp(dw1000_probe_exchange_name(DW1000_PROBE_EXCHANGE_DS), "ds") != 0)
	return "exchange DS: wrong name";
    if (strcmp(dw1000_probe_exchange_name(DW1000_PROBE_EXCHANGE__COUNT), "invalid") != 0)
	return "exchange __COUNT should be 'invalid'";
    if (strcmp(dw1000_probe_exchange_name((dw1000_probe_exchange_t)999), "invalid") != 0)
	return "exchange 999 should be 'invalid'";
    return NULL;
}


/*----------------------------------------------------------------------*/
/* Plumbing                                                             */
/*----------------------------------------------------------------------*/

int
main(void)
{
    step("complete record",            case_complete_record());
    step("cleared bit prints -",       case_cleared_bits());
    step("negative distances",         case_negative_distances());
    step("NULL/empty node and run",    case_null_and_empty_origin());
    step("status names",               case_status_names());
    step("role names",                 case_role_names());
    step("role lookup round-trip",     case_role_lookup());
    step("pack power",                 case_pack_power());
    step("truncation",                 case_truncation());
    step("STATS/READY/TEMP lines",     case_stats_ready_temp());
    step("port now/sleep move time",   case_port_time());
    step("distances: no drift (A)",    case_distances_no_drift());
    step("distances: drift (B)",       case_distances_drift());
    step("distances: 40-bit wrap (C)", case_distances_wrap());
    step("distances: ss only",         case_distances_ss_only());
    step("distances: missing instant", case_distances_missing_instant());
    step("distances: zero-sum",        case_distances_bad_distance());
    step("exchange names",             case_exchange_names());

    return failures == 0 ? 0 : 1;
}
