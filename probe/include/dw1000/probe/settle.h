/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_PROBE_SETTLE_H__
#define __DW1000_PROBE_SETTLE_H__

/**
 * @file    settle.h
 * @brief   When a die has stopped warming: the settle rule.
 *
 * Free of <dw1000/dw1000.h>, like the record: the rule is arithmetic over
 * temperature readings, so it is proved with no chip (tests/probe/
 * format.c) and fed by whichever role takes the readings
 * (dw1000_probe_rx_run() in <dw1000/probe/solo.h>).
 *
 * THE RULE, as probe/DESIGN.md states it: readings are grouped into
 * consecutive windows of @p window_s seconds; the die is settled once the
 * means of the last @p windows windows lie within @p threshold_cdeg of one
 * another; the attempt gives up after @p give_up_s seconds. The defaults
 * are DESIGN.md's: four 30 s windows within 0.2 °C, giving up at 15
 * minutes.
 *
 * "Within the threshold of one another" is read as the SPREAD of those
 * means (largest minus smallest), not as each step between neighbours. A
 * die climbing 0.15 °C a window passes a step test for ever while moving
 * 0.45 °C across four windows, which is exactly the "climbing steadily"
 * case the rule exists to refuse.
 *
 * Two things the rule rests on, and a caller must not undo:
 *
 *  - WINDOW MEANS, NEVER RAW READINGS. One reading resolves about 1.1 °C
 *    (one LSB of the DW1000's 8-bit SAR) and the threshold is a fifth of
 *    that. Only a mean over readings that dither between two adjacent
 *    codes resolves it, so this compares means and nothing else;
 *  - WINDOWS ARE COUNTED IN SAMPLES, at the role's sampling interval: a
 *    30 s window at 1 Hz is thirty readings. Closing windows by sample
 *    count rather than by clock makes the rule a pure function of the
 *    readings it was fed, which is what lets it be tested without one.
 *
 * Values are the shell's (probe/DESIGN.md, "The split line"); their units,
 * defaults and validation are here.
 *
 * @addtogroup PROBE
 * @{
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <dw1000/probe/record.h>

/*===========================================================================*/
/* Parameters                                                                */
/*===========================================================================*/

/** @brief The most windows the rule can be asked to compare. */
#define DW1000_PROBE_SETTLE_WINDOWS_MAX 16

/**
 * @brief The settle rule's values.
 */
struct dw1000_probe_settle_params {
    uint32_t interval_ms;     /**< between two readings                 */
    uint32_t window_s;        /**< one window's length                  */
    uint8_t  windows;         /**< consecutive windows that must agree,
                                   2 .. DW1000_PROBE_SETTLE_WINDOWS_MAX  */
    uint16_t threshold_cdeg;  /**< their means' spread must be BELOW this,
                                   hundredths of a degree Celsius       */
    uint32_t give_up_s;       /**< stop trying after this long          */
};

/** @brief probe/DESIGN.md's values: 1 Hz, four 30 s windows within
 *         0.2 °C, giving up at 15 minutes. */
#define DW1000_PROBE_SETTLE_PARAMS_DEFAULT                              \
    { .interval_ms = 1000, .window_s = 30, .windows = 4,                \
      .threshold_cdeg = 20, .give_up_s = 900 }

/**
 * @brief Whether @p params describe a rule that can be run.
 *
 * Refused: a zero interval, window or threshold; fewer than two windows
 * (one window has no spread) or more than DW1000_PROBE_SETTLE_WINDOWS_MAX;
 * a window shorter than one interval (it would hold no reading); and a
 * give-up shorter than the windows it has to compare, which could never
 * settle.
 */
bool dw1000_probe_settle_params_valid(
    const struct dw1000_probe_settle_params *params);

/**
 * @brief The most lines a settle run emits: a SETTLE line per window
 *        until give-up, and the TEMP lines that open and close it.
 *
 * For a host that buffers a run's lines in fixed memory, which is how a
 * board does it, to refuse a run that would not fit rather than lose its
 * end. Meaningful only for valid @p params.
 */
size_t dw1000_probe_settle_lines(
    const struct dw1000_probe_settle_params *params);

/*===========================================================================*/
/* The rule                                                                  */
/*===========================================================================*/

/**
 * @brief What feeding one reading did.
 */
typedef enum {
    DW1000_PROBE_SETTLE_SAMPLING = 0,  /**< the window is still open        */
    DW1000_PROBE_SETTLE_UNSETTLED,     /**< a window closed; not settled   */
    DW1000_PROBE_SETTLE_SETTLED,       /**< a window closed; settled       */
    DW1000_PROBE_SETTLE_GAVE_UP,       /**< a window closed at the give-up
                                            time, not settled              */
    DW1000_PROBE_SETTLE__COUNT
} dw1000_probe_settle_state_t;

/**
 * @brief "sampling", "unsettled", "settled" or "gave-up", as printed in
 *        `state=`. Never NULL; out of range gives "invalid".
 */
const char *dw1000_probe_settle_state_name(dw1000_probe_settle_state_t state);

/**
 * @brief One settle attempt. Opaque in use; public so it can live on a
 *        stack, which is where a role keeps it.
 */
struct dw1000_probe_settle {
    struct dw1000_probe_settle_params params;
    uint32_t samples_per_window;
    uint32_t windows_max;            /**< windows that fit the give-up    */

    int32_t  sum;                    /**< of the open window's readings   */
    uint32_t n;                      /**< readings in the open window     */

    uint32_t closed;                 /**< windows closed so far           */
    int16_t  means[DW1000_PROBE_SETTLE_WINDOWS_MAX]; /**< ring, last ones */

    dw1000_probe_settle_state_t state;  /**< after the last window closed */
    int16_t  mean;                   /**< of the last window closed       */
    int32_t  spread;                 /**< of the last @p windows means; -1
                                          while fewer have closed         */
};

/**
 * @brief Start an attempt. @p params must be valid.
 */
void dw1000_probe_settle_start(struct dw1000_probe_settle *settle,
                               const struct dw1000_probe_settle_params *params);

/**
 * @brief Feed one reading, in hundredths of a degree Celsius.
 *
 * Returns SAMPLING until the reading that fills a window, then the
 * verdict on that window. SETTLED and GAVE_UP are final: feeding again
 * after either starts nothing new and returns the same verdict.
 */
dw1000_probe_settle_state_t
dw1000_probe_settle_feed(struct dw1000_probe_settle *settle, int16_t temp);

/*===========================================================================*/
/* The line                                                                  */
/*===========================================================================*/

/**
 * @brief Write the `SETTLE window=...` line for the window just closed.
 *
 * Keyed, not positional: there is no reference instrument's line to
 * mirror, and a keyed line can grow. The keys, in order: `window=` (1 for
 * the first), `elapsed=` (seconds, one decimal, as TEMP has it),
 * `mean=` (hundredths of a degree, as TEMP's reading), `spread=`
 * (hundredths, `-` while fewer than @p windows windows have closed),
 * `threshold=`, `state=`, then the origin's `node=`, `role=` and
 * `run=`. Same contract as dw1000_probe_record_format().
 */
size_t dw1000_probe_settle_format(char *buf, size_t len,
                                  const struct dw1000_probe_settle *settle,
                                  uint32_t elapsed_ds,
                                  const struct dw1000_probe_origin *origin);

/** @} */

#endif
