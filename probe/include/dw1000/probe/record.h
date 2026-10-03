/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_PROBE_RECORD_H__
#define __DW1000_PROBE_RECORD_H__

/**
 * @file    record.h
 * @brief   What one exchange produced, and how it is written down.
 *
 * Deliberately free of <dw1000/dw1000.h>: a record is numbers and a
 * format, and keeping the driver out of this header is what lets the
 * format be proved with no chip, no port and no radio. Every driver type
 * mirrored here is a fixed-width integer, so that stays true.
 *
 * The field set, order and units are the reference instrument's, so that
 * one parser serves both. The order is kept even though the format is
 * key=value, so that a positional reader still works; fields this
 * instrument adds are appended, never interleaved.
 *
 * @addtogroup PROBE
 * @{
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <dw1000/probe/role.h>

/*===========================================================================*/
/* Status                                                                    */
/*===========================================================================*/

/**
 * @brief How an exchange attempt ended, FROM THE INITIATOR'S SIDE.
 *
 * The initiator is the only end that emits a record: it is the one
 * holding both ends' numbers, because the responder's arrived in REPORT,
 * so these name the step at which the initiator's attempt stopped. Each
 * names the frame that did not happen, in the order the frames go.
 *
 * The reference instrument emits nothing at all for an attempt that did
 * not resolve. This one emits a line regardless, because the count of
 * attempts that resolved is the one statistic a board could already
 * produce, and losing it would be a regression.
 *
 * NO_RESPONSE does not say which way the loss went: a POLL the responder
 * never heard and a RESPONSE the initiator never heard look the same from
 * here. The responder's own STATS line (`of=`, the POLLs it answered)
 * tells them apart across a run.
 */
typedef enum {
    DW1000_PROBE_STATUS_OK = 0,        /**< every frame, distances computed       */
    DW1000_PROBE_STATUS_NO_POLL,       /**< the POLL could not be sent: no
                                     transmit completion, t_sp absent      */
    DW1000_PROBE_STATUS_NO_RESPONSE,   /**< POLL sent, no RESPONSE by deadline    */
    DW1000_PROBE_STATUS_NO_FINAL,      /**< RESPONSE received, FINAL could not be
                                     sent: t_sf absent                     */
    DW1000_PROBE_STATUS_NO_REPORT,     /**< FINAL sent (or, single-sided,
                                     RESPONSE received), no REPORT: no
                                     distance, every estimate needing t_sr */
    DW1000_PROBE_STATUS_BAD_DISTANCE,  /**< six instants, intervals sum to zero   */
    DW1000_PROBE_STATUS__COUNT
} dw1000_probe_status_t;

/**
 * @brief The canonical spelling of a status, as printed in `status=`.
 *
 * Canonical for the same reason role names are: printed by every host,
 * read by one parser. Never returns NULL; out of range gives "invalid".
 */
const char *dw1000_probe_status_name(dw1000_probe_status_t status);

/*===========================================================================*/
/* Which exchange                                                            */
/*===========================================================================*/

/**
 * @brief Which exchange produced a record.
 *
 * Two frames or four. They are not interchangeable and the record says
 * which, because the estimates it carries differ: a two-frame exchange
 * can only be estimated one way.
 */
typedef enum {
    DW1000_PROBE_EXCHANGE_SS = 0,   /**< POLL, RESPONSE                    */
    DW1000_PROBE_EXCHANGE_DS,       /**< POLL, RESPONSE, FINAL, REPORT     */
    DW1000_PROBE_EXCHANGE__COUNT
} dw1000_probe_exchange_t;

/**
 * @brief "ss" or "ds", as printed in `exchange=`. Never NULL.
 */
const char *dw1000_probe_exchange_name(dw1000_probe_exchange_t exchange);

/*===========================================================================*/
/* Presence                                                                  */
/*===========================================================================*/

/**
 * @brief Which of the optional fields this record actually carries.
 *
 * A missing value is printed `-`. A sentinel inside the value would not
 * do: 0 is a legal device timestamp and 0 mV is a legal (if alarming)
 * battery reading, so absence is tracked beside the numbers rather than
 * inside them.
 *
 * A field is absent for one of three reasons, and a reader need not tell
 * them apart: `-` is `-`:
 *
 *  - it never travelled. The responder's readings and instants ride in
 *    REPORT, so an attempt that ended earlier has the initiator's and
 *    not the responder's;
 *  - it could not be computed: both distances, when the four intervals
 *    sum to zero (DW1000_PROBE_STATUS_BAD_DISTANCE);
 *  - there was no estimate: dw1000_rx_get_power_estimate() yields
 *    -INFINITY when the preamble accumulation count came out 0: a frame
 *    shorter than the SFD adjustment, or a failed read of RX_FINFO. The
 *    reference instrument carries that case as 0, an in-band sentinel of
 *    exactly the kind this header refuses; the probe clears the bit and
 *    prints `-`. That is a deliberate departure, and the only one a
 *    reader could notice without being told.
 *
 * Temperature and voltage have no failure path (the driver's read is a
 * void function), so the initiator's pair, the recording end's, is
 * present in every record.
 */
#define DW1000_PROBE_F_T_SP        (1u <<  0)
#define DW1000_PROBE_F_T_RP        (1u <<  1)
#define DW1000_PROBE_F_T_SR        (1u <<  2)
#define DW1000_PROBE_F_T_RR        (1u <<  3)
#define DW1000_PROBE_F_T_SF        (1u <<  4)
#define DW1000_PROBE_F_T_RF        (1u <<  5)
#define DW1000_PROBE_F_SYM         (1u <<  6)
#define DW1000_PROBE_F_ASYM        (1u <<  7)
#define DW1000_PROBE_F_RESP_TEMP   (1u <<  8)
#define DW1000_PROBE_F_RESP_VBAT   (1u <<  9)
#define DW1000_PROBE_F_INIT_TEMP   (1u << 10)
#define DW1000_PROBE_F_INIT_VBAT   (1u << 11)
#define DW1000_PROBE_F_RESP_RX     (1u << 12)
#define DW1000_PROBE_F_RESP_FP     (1u << 13)
#define DW1000_PROBE_F_INIT_RX     (1u << 14)
#define DW1000_PROBE_F_INIT_FP     (1u << 15)
#define DW1000_PROBE_F_SS          (1u << 16)

/** @brief The four instants a two-frame exchange yields. */
#define DW1000_PROBE_F_INSTANTS_SS 0x000fu
/** @brief All six a four-frame exchange yields. */
#define DW1000_PROBE_F_INSTANTS    0x003fu
/** @brief Every estimate. */
#define DW1000_PROBE_F_DISTANCES   0x100c0u
/** @brief Everything a complete four-frame exchange yields. */
#define DW1000_PROBE_F_ALL         0x1ffffu

/*===========================================================================*/
/* The record                                                                */
/*===========================================================================*/

/**
 * @brief One exchange attempt.
 *
 * Units are the reference instrument's, unconverted, so that the two
 * sets of numbers are comparable without anybody remembering a scale
 * factor:
 *
 *  - instants        device clock ticks (40 bits used of 64)
 *  - distances       TENTHS of a millimetre, signed, printed with one
 *                    decimal as `sym_mm=362.5`. Tenths because the
 *                    reference prints one decimal and the format is the
 *                    contract; signed because a negative distance is a
 *                    real result and means the antenna delay is too large
 *  - temperature     hundredths of a degree Celsius, signed
 *  - voltage         millivolts
 *  - receive power   hundredths of a dBm, NEGATED: 8825 is -88.25 dBm
 *
 * `init_` is the initiator, the node that emits the record; `resp_` is
 * the responder, whose readings arrived in REPORT. `resp_rx` and
 * `resp_fp` are what the responder made of the POLL, `init_rx` and
 * `init_fp` what the initiator made of the RESPONSE.
 *
 * Two places where the wire is narrower than the field, both of them the
 * reference instrument's doing and neither worth copying:
 *
 *  - @p seq is a single byte on the wire and wraps at 256; it is widened
 *    here so a run longer than that still counts up;
 *  - the far end's temperature travels masked to 16 bits unsigned and
 *    is read back unsigned, so the reference prints a sub-zero far-end
 *    temperature as 65xxx. It is signed here, and the probe
 *    sign-extends on receipt.
 */
struct dw1000_probe_record {
    uint16_t       seq;         /**< exchange number within the run      */
    dw1000_probe_status_t status;      /**< how it ended                        */
    uint32_t       present;     /**< DW1000_PROBE_F_* for the fields below      */

    uint64_t t_sp;              /**< initiator sent POLL       (local)     */
    uint64_t t_rp;              /**< responder received POLL   (in REPORT) */
    uint64_t t_sr;              /**< responder sent RESPONSE   (in REPORT) */
    uint64_t t_rr;              /**< initiator recvd RESPONSE  (local)     */
    uint64_t t_sf;              /**< initiator sent FINAL      (local)     */
    uint64_t t_rf;              /**< responder received FINAL  (in REPORT) */

    dw1000_probe_exchange_t exchange; /**< two frames or four            */

    int32_t  ss_dmm;            /**< single-sided, tenths of a mm        */
    int32_t  sym_dmm;           /**< symmetric / classic, tenths of a mm */
    int32_t  asym_dmm;          /**< asymmetric / Neirynck, tenths of mm */

    int16_t  resp_temp;
    uint16_t resp_vbat;
    int16_t  init_temp;
    uint16_t init_vbat;
    uint16_t resp_rx;
    uint16_t resp_fp;
    uint16_t init_rx;
    uint16_t init_fp;
};

/**
 * @brief Compute every estimate the record's instants allow.
 *
 * Fills `ss_dmm`, and for a four-frame exchange `sym_dmm` and
 * `asym_dmm` too, setting the matching presence bits. The three are kept
 * side by side rather than one being chosen, because they disagree in a
 * way that is itself the measurement:
 *
 *   round1 = t_rr - t_sp   reply1 = t_sr - t_rp
 *   round2 = t_rf - t_sr   reply2 = t_sf - t_rr
 *
 *   single-sided  (round1 - reply1) / 2
 *   symmetric     ((round1 - reply1) + (round2 - reply2)) / 4
 *   Neirynck      (round1*round2 - reply1*reply2) / (sum of all four)
 *
 * round1 and reply2 are measured on the initiator's clock, reply1 and
 * round2 on the responder's, so a crystal offset between the two nodes
 * biases them differently. On a true 1032.0 mm link with the responder
 * 20 ppm fast (an ordinary part-to-part offset), single-sided reports
 * -140.8 mm, symmetric 1172.9 mm, and Neirynck 1032.2 mm. That is why
 * the distances are signed, and why an instrument reports all three
 * rather than picking one.
 *
 * Every interval is a wrapping 40-bit subtraction taken signed: the
 * counter wraps, and the symmetric formula subtracts intervals that can
 * legitimately come out the other way round.
 *
 * @return false only when not even the single-sided estimate could be
 *         computed, which means one of the four two-frame instants was
 *         missing. A four-interval sum of zero is NOT that case: it costs
 *         `sym` and `asym` alone (their bits stay clear and the record
 *         is DW1000_PROBE_STATUS_BAD_DISTANCE), while the single-sided
 *         estimate still stands, so the answer is still true. (The
 *         earlier wording of this line said otherwise and contradicted
 *         both the presence-bit note above and the status enum; it was
 *         wrong, and an implementer caught it.)
 */
bool dw1000_probe_distances(struct dw1000_probe_record *record);

/**
 * @brief The reference instrument's power packing, for one value.
 *
 * dBm to negated hundredths, saturating. The one piece of the record's
 * arithmetic that is not a plain copy, and the place the -INFINITY case
 * is decided, so it is here where the format is proved rather than in an
 * exchange nobody can run without a radio.
 *
 * @param[in]  dbm  an estimate from dw1000_rx_get_power_estimate()
 * @param[out] out  set only when true is returned
 * @return     false if @p dbm is not finite, in which case the field is
 *             absent and its DW1000_PROBE_F_* bit must be left clear
 */
bool dw1000_probe_pack_power(double dbm, uint16_t *out);

/**
 * @brief Where a record came from.
 *
 * The reference instrument's line carries none of this: one process on
 * one host measured one pair, and the harness knew which. An instrument
 * that runs on every node cannot assume that. The slots are here because
 * the record is here; the values mean nothing to this library.
 *
 * The role is carried as the enum, not as a string, so that it can only
 * ever be spelled by dw1000_probe_role_name(); an application filling in a
 * string by hand is precisely how `resp` and `twr_resp` come to coexist.
 *
 * A NULL or empty @p node or @p run is printed `-`, like any other absent
 * value. Both should be short: see DW1000_PROBE_RECORD_MAX.
 */
struct dw1000_probe_origin {
    const char  *node;          /**< which node emitted this             */
    dw1000_probe_role_t role;          /**< spelled by dw1000_probe_role_name()        */
    const char  *run;           /**< which run it belongs to             */
};

/*===========================================================================*/
/* The lines                                                                 */
/*===========================================================================*/

/**
 * @brief Write the `TWR seq=...` line for one record.
 *
 * Formatting lives here, not in an application, for two reasons: the line
 * must be byte-identical on every platform or the single parser stops
 * being single, and it must be provable with no chip.
 *
 * Writes at most @p len bytes including the NUL, and never a newline:
 * that is dw1000_probe_port_emit()'s.
 *
 * @return the length the line would have had, excluding the NUL, the way
 *         snprintf() reports it: a value >= @p len means it was truncated.
 */
size_t dw1000_probe_record_format(char *buf, size_t len,
                           const struct dw1000_probe_record *record,
                           const struct dw1000_probe_origin *origin);

/**
 * @brief Enough for any line these formatters produce, PROVIDED the
 *        origin's `node` and `run` are each at most 64 bytes.
 *
 * Every numeric field at its widest plus slack. The origin strings are
 * the application's and cannot be bounded from here, which is why the
 * bound is stated rather than assumed: a longer one truncates, and
 * truncation is visible in the return value.
 */
#define DW1000_PROBE_RECORD_MAX 512

/**
 * @brief What a whole run amounted to.
 *
 * Transmit power lives here rather than on the record line, as the
 * reference instrument has it, because it does not change within a run.
 * One STATS line per run: a harness reading these takes the first one it
 * sees, so a sweep emits one run per setting rather than several lines
 * from one process.
 *
 * Printed keys, fixed because a parser reads them: `role=` `tx_power_db=`
 * `tx_power_req_db=` `driver=` `completed=` `of=`, then the reception
 * account below. Both ends emit one. From the initiator, `of=` is the
 * counted attempts and `completed=` those that resolved; from the
 * responder, `of=` is the POLLs it answered (warm-up included) and
 * `completed=` the REPORTs that left. The first, fourth and fifth-through-sixth match the
 * reference; everything after them is an addition, appended rather than
 * interleaved for the reason the record line gives.
 *
 * THE RECEPTION ACCOUNT answers a question `completed=0 of=40` cannot:
 * whether the receiver heard nothing at all, or heard frames and threw
 * every one of them away. Those are different defects with different
 * fixes, and telling them apart used to need a second run with a second
 * instrument.
 *
 * `heard=` is every frame the chip delivered to the application's rx_ok
 * callback during the run. The seven `drop_` counts and the frames that
 * matched partition it exactly, so `heard` minus their sum is the number
 * of frames that were the one being waited for. Two of the seven are
 * about this instrument rather than the link, and are the more
 * interesting for that:
 *
 *  - `drop_unwatched=` arrived while no wait was running. The exchange
 *    is a sequence of waits with gaps between them, and this counts the
 *    frames that landed in a gap: a peer answering faster than this
 *    role gets back into its wait. It is NOT a loss: the ring holds
 *    them and the next wait examines them. It was a loss once, and the
 *    day it cost is in exchange.c's comment on the ring;
 *  - `drop_overrun=` arrived and was overwritten before being examined,
 *    the consumer having fallen more than RX_RING frames behind. This
 *    is now the only way a received frame is lost.
 *
 * The other five are the frame-matching predicate's own reasons, in the
 * order it applies them, each one narrower than the last: `drop_foreign=`
 * carried no `dwp` mark and belongs to somebody else; `drop_short=` was
 * too short to hold the payload the awaited type needs; `drop_type=` was
 * one of ours of the wrong kind; `drop_dst=` was addressed elsewhere;
 * `drop_seq=` belonged to a different exchange.
 */
struct dw1000_probe_stats {
    int16_t     tx_power_db_x10;     /**< APPLIED power, read back from the
                                          chip, tenths of a dB: the grid is
                                          half-dB steps, so 75 is 7.5 dB    */
    int16_t     tx_power_req_db_x10; /**< REQUESTED power, tenths of a dB;
                                          -1 for the automatic setting.
                                          Kept beside the applied value
                                          because a request-only line
                                          records an intention, and a
                                          result-only line cannot show a
                                          request that was not honoured,
                                          a clamp at the top of the range,
                                          or AUTO resolving to 7.5 dB       */
    const char *driver_version;      /**< which driver produced these
                                          numbers; the instrument and its
                                          driver must be one commit         */
    uint16_t    attempted;           /**< printed `of=`; per role, see
                                          above                             */
    uint16_t    resolved;            /**< of those, how many went through;
                                          printed `completed=`              */

    /* The reception account. See the note above for what partitions
       what; all eight saturate rather than wrap, because a count that
       has gone round is worse than useless and 65535 frames rejected
       says everything a larger number would. */
    uint16_t    heard;               /**< frames delivered during the run   */
    uint16_t    drop_unwatched;      /**< no wait was running               */
    uint16_t    drop_overrun;        /**< overwritten before it was read    */
    uint16_t    drop_foreign;        /**< no `dwp` mark                     */
    uint16_t    drop_short;          /**< too short for the awaited type    */
    uint16_t    drop_type;           /**< ours, wrong kind                  */
    uint16_t    drop_dst;            /**< addressed elsewhere               */
    uint16_t    drop_seq;            /**< a different exchange              */
};

/**
 * @brief Write the `STATS role=...` line. Same contract as above.
 */
size_t dw1000_probe_stats_format(char *buf, size_t len,
                          const struct dw1000_probe_stats *stats,
                          const struct dw1000_probe_origin *origin);

/**
 * @brief Write the `READY ...` line a responder prints before listening.
 *
 * A marker in the capture, not a signal anything can wait for, and the
 * difference has cost a bench campaign already. A harness that buffers
 * its console (a Ruby one does, whenever stdout is not a terminal) holds
 * the whole log until the session ends, so a reader polling for this
 * line cannot ever see it in time: the line was printed, it was simply
 * not written yet. What a harness does instead is give the initiator a
 * fixed head start and say so, and what the responder does instead is
 * bound its own wait for the first POLL, so that a start-up that went
 * wrong shows up as no-poll records rather than as silence.
 *
 * It stays worth printing. It is how a capture says which run its lines
 * belong to, and it is how the responder reports that bring-up got as
 * far as listening.
 */
size_t dw1000_probe_ready_format(char *buf, size_t len,
                          const struct dw1000_probe_origin *origin);

/**
 * @brief Write the `TEMP <elapsed> <temp> <vbat>` line.
 *
 * Positional, as the reference instrument prints it, and the output of
 * the temperature role: the die baseline, and the settle phase's raw
 * material. @p elapsed_ds is tenths of a second, printed with one decimal.
 */
size_t dw1000_probe_temp_format(char *buf, size_t len,
                         uint32_t elapsed_ds, int16_t temp, uint16_t vbat,
                         const struct dw1000_probe_origin *origin);

/** @} */

#endif
