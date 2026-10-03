/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    settle.c
 * @brief   The settle rule, and its line.
 *
 * @addtogroup PROBE
 * @{
 */

#include <stdio.h>
#include <string.h>

#include <dw1000/probe/role.h>
#include <dw1000/probe/settle.h>

/*===========================================================================*/
/* Parameters                                                                */
/*===========================================================================*/

bool
dw1000_probe_settle_params_valid(const struct dw1000_probe_settle_params *p)
{
    if (p == NULL)
	return false;
    if (p->interval_ms == 0 || p->window_s == 0 || p->threshold_cdeg == 0)
	return false;
    if (p->windows < 2 || p->windows > DW1000_PROBE_SETTLE_WINDOWS_MAX)
	return false;
    /* A window holds at least one reading. */
    if ((uint64_t)p->window_s * 1000u < p->interval_ms)
	return false;
    /* And the give-up leaves room for the windows to be compared. */
    if ((uint64_t)p->window_s * p->windows > p->give_up_s)
	return false;
    return true;
}

size_t
dw1000_probe_settle_lines(const struct dw1000_probe_settle_params *p)
{
    return (size_t)(p->give_up_s / p->window_s) + 2;
}

/*===========================================================================*/
/* The rule                                                                  */
/*===========================================================================*/

const char *
dw1000_probe_settle_state_name(dw1000_probe_settle_state_t state)
{
    switch (state) {
    case DW1000_PROBE_SETTLE_SAMPLING:  return "sampling";
    case DW1000_PROBE_SETTLE_UNSETTLED: return "unsettled";
    case DW1000_PROBE_SETTLE_SETTLED:   return "settled";
    case DW1000_PROBE_SETTLE_GAVE_UP:   return "gave-up";
    case DW1000_PROBE_SETTLE__COUNT:    break;
    }
    return "invalid";
}

void
dw1000_probe_settle_start(struct dw1000_probe_settle *s,
			  const struct dw1000_probe_settle_params *p)
{
    memset(s, 0, sizeof(*s));
    s->params             = *p;
    s->samples_per_window = (uint32_t)((uint64_t)p->window_s * 1000u /
					p->interval_ms);
    s->windows_max        = p->give_up_s / p->window_s;
    s->state              = DW1000_PROBE_SETTLE_SAMPLING;
    s->spread             = -1;
}

/* Rounded to the nearest hundredth, halves away from zero, so that a
 * die sitting between two codes does not have its mean pulled towards
 * zero by truncation. */
static int16_t
mean_of(int32_t sum, uint32_t n)
{
    int32_t half = (int32_t)(n / 2);

    return (int16_t)(sum >= 0 ? (sum + half) / (int32_t)n
			      : (sum - half) / (int32_t)n);
}

dw1000_probe_settle_state_t
dw1000_probe_settle_feed(struct dw1000_probe_settle *s, int16_t temp)
{
    if (s->state == DW1000_PROBE_SETTLE_SETTLED ||
	s->state == DW1000_PROBE_SETTLE_GAVE_UP)
	return s->state;

    s->sum += temp;
    s->n++;
    if (s->n < s->samples_per_window)
	return DW1000_PROBE_SETTLE_SAMPLING;

    /* The window is full: close it. */
    s->mean = mean_of(s->sum, s->n);
    s->means[s->closed % DW1000_PROBE_SETTLE_WINDOWS_MAX] = s->mean;
    s->closed++;
    s->sum = 0;
    s->n   = 0;

    if (s->closed >= s->params.windows) {
	int16_t  lo = s->mean, hi = s->mean;
	uint32_t i;

	for (i = 1; i <= s->params.windows; i++) {
	    int16_t m = s->means[(s->closed - i) % DW1000_PROBE_SETTLE_WINDOWS_MAX];

	    if (m < lo) lo = m;
	    if (m > hi) hi = m;
	}
	s->spread = (int32_t)hi - lo;
    }

    if (s->spread >= 0 && s->spread < (int32_t)s->params.threshold_cdeg)
	s->state = DW1000_PROBE_SETTLE_SETTLED;
    else if (s->closed >= s->windows_max)
	s->state = DW1000_PROBE_SETTLE_GAVE_UP;
    else
	s->state = DW1000_PROBE_SETTLE_UNSETTLED;

    return s->state;
}

/*===========================================================================*/
/* The line                                                                  */
/*===========================================================================*/

static const char *
fmt_str(const char *s)
{
    return (s != NULL && s[0] != '\0') ? s : "-";
}

size_t
dw1000_probe_settle_format(char *buf, size_t len,
			   const struct dw1000_probe_settle *s,
			   uint32_t elapsed_ds,
			   const struct dw1000_probe_origin *origin)
{
    char spread[16];

    if (s->spread < 0)
	snprintf(spread, sizeof(spread), "-");
    else
	snprintf(spread, sizeof(spread), "%ld", (long)s->spread);

    return (size_t)snprintf(buf, len,
	"SETTLE window=%lu elapsed=%u.%u mean=%d spread=%s threshold=%u"
	" state=%s node=%s role=%s run=%s",
	(unsigned long)s->closed,
	(unsigned)(elapsed_ds / 10), (unsigned)(elapsed_ds % 10),
	(int)s->mean, spread, (unsigned)s->params.threshold_cdeg,
	dw1000_probe_settle_state_name(s->state),
	fmt_str(origin->node),
	dw1000_probe_role_name(origin->role),
	fmt_str(origin->run));
}

/** @} */
