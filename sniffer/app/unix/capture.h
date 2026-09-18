/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CAPTURE__H
#define __CAPTURE__H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <time.h>
#include <dw1000/dw1000.h>

#include "dissect.h"

/**
 * @name Per-frame flags
 *
 * @p CAPTURE_F_TRUNCATED says the driver reported more than a slot
 * holds, so @p length is short of @p reported; @p CAPTURE_F_METADATA
 * says @p meta was filled in (it is not when the metadata read-out is
 * turned off); @p CAPTURE_F_RANGING is the chip's ranging bit for the
 * frame, which the program used to discard.
 * @{
 */
#define CAPTURE_F_TRUNCATED	(1u << 0)
#define CAPTURE_F_METADATA	(1u << 1)
#define CAPTURE_F_RANGING	(1u << 2)
/** @} */

/**
 * What the chip knew about a frame besides its bytes.
 *
 * Every field here comes out of a register that swings with the double
 * buffer pointer (@p RX_TIME, @p RX_FQUAL, @p RX_TTCKI, @p RX_TTCKO; see
 * hw/drivers/dw1000/README.md, "Double buffered receive"), so it is read
 * in the rx_ok callback beside the payload or not at all. This is the
 * half of a capture the program used to throw away: a sniffer that
 * reports only the bytes cannot say when a frame arrived, how strong it
 * was, or how far the transmitter's clock had drifted.
 *
 * The two powers are kept in milli-dBm rather than as a double so that
 * @p wire.c has nothing to convert and nothing to decide: the estimates
 * carry about a tenth of a dB of meaning, and @p CAPTURE_POWER_NONE
 * distinguishes "no usable estimate" (which the driver reports as
 * -INFINITY, for a preamble accumulation count of 0) from a genuine
 * reading, which a float cannot do on the wire without the receiver
 * knowing our NaN conventions.
 */
#define CAPTURE_POWER_NONE	INT32_MIN

struct capture_meta {
    uint64_t rx_time;		/**< RMARKER, 40-bit chip clock          */
    int32_t  clock_offset;	/**< RX_TTCKO, transmitter clock offset  */
    uint32_t clock_interval;	/**< RX_TTCKI, 0 when unavailable        */
    int32_t  power_signal;	/**< milli-dBm, or CAPTURE_POWER_NONE    */
    int32_t  power_firstpath;	/**< milli-dBm, or CAPTURE_POWER_NONE    */
    uint16_t first_path;	/**< dw1000_rxinfo_t::first_path         */
    uint16_t std_noise;		/**< dw1000_rxinfo_t::std_noise          */
    uint16_t max_noise;		/**< dw1000_rxinfo_t::max_noise          */
};

/**
 * One frame, as it was taken off the chip.
 *
 * @p length is what is held in @p data, @p reported is what the driver
 * said: a frame longer than a slot is truncated to fit, flagged, and
 * counted, and sending @p reported out of a buffer that does not hold it
 * would read past the end. Both are on the wire, because a receiver that
 * is handed only the short one cannot tell a short frame from a mangled
 * one.
 *
 * The CRC is included, as it is in the driver's own report (see
 * @p dw1000_rx_get_frame_info(); @p DW1000_CRC_LENGTH is what to
 * subtract for the payload alone): a sniffer forwards the frame whole,
 * so nothing is subtracted here, and that is also why the pcapng link
 * type is the with-FCS one.
 *
 * @p wall is the host clock at read-out, not the chip's: it is what a
 * pcapng timestamp and a far-side reassembly need, and @p meta.rx_time
 * is the precise one to use for anything between two frames of the same
 * capture.
 */
struct capture_frame {
    uint8_t  data[DW1000_FRAME_MAXSIZE];
    size_t   length;		/**< bytes held in @p data               */
    size_t   reported;		/**< bytes the driver reported           */
    uint32_t seq;		/**< 0-based, monotonic, never reused    */
    uint32_t flags;		/**< CAPTURE_F_*                         */
    struct timespec wall;	/**< CLOCK_REALTIME at read-out          */
    struct capture_meta meta;	/**< valid iff CAPTURE_F_METADATA        */
};

/**
 * What the ring did, for the report on the way out.
 *
 * @p overrun is the only way a frame this program received is lost here,
 * exactly as it is in probe/src/exchange.c: the producer never refuses a
 * frame, so a consumer that falls more than the ring's depth behind
 * loses the oldest entries and finds out on its next read. A frame lost
 * on the *chip* is a different count, kept by uwb_dw1000.c and reported
 * beside this one.
 *
 * @p high_water is the deepest the ring was ever seen to be, which is
 * what turns its depth from the guess the TODO called it into a
 * measurement: a run that never passes 2 has 14 entries to spare.
 */
struct capture_stats {
    unsigned long captured;	/**< frames read off the chip           */
    unsigned long consumed;	/**< frames handed to the consumer      */
    unsigned long overrun;	/**< frames overwritten before that     */
    unsigned long truncated;	/**< frames longer than a slot          */
    unsigned long high_water;	/**< deepest the ring has been          */
};

/**
 * How the ring reaches the chip.
 *
 * The one reason this is a pair of function pointers rather than direct
 * calls into uwb.h: the ring, its overrun accounting and its truncation
 * are the only real logic in this program, they are provable with no
 * chip, and a direct call to uwb_read_frame_data() drags in bitters and
 * the Linux GPIO and SPI character devices, which is why there was no
 * test. tests/sniffer/capture.c supplies its own pair instead, the same
 * way tests/probe/format.c proves the probe's format with no radio.
 *
 * @p read_meta may be NULL, which is how the metadata read-out is turned
 * off: without it the frames carry no @p CAPTURE_F_METADATA and the
 * callback spends no SPI time on registers nobody asked for.
 */
struct capture_ops {
    void (*read_frame_data)(uint8_t *data, size_t length, size_t offset);
    void (*read_meta)(struct capture_meta *meta);
};

/**
 * Point the ring at the chip, and reset it.
 *
 * MUST be called before the receiver is armed. @p ops is kept by
 * reference and must outlive the ring; @p ops->read_frame_data must not
 * be NULL.
 *
 * @param[in] ops	how to reach the chip, and whether to read metadata
 */
void capture_init(const struct capture_ops *ops);

/**
 * Take the reported frame off the chip and into the ring.
 *
 * MUST be called from the rx_ok callback and MUST NOT be deferred: under
 * double buffering the driver toggles the host side buffer pointer as
 * soon as that callback returns, and every register this reads swings
 * with it. This is the whole reason the ring exists: the read-out
 * happens now, the forwarding happens later.
 *
 * Never fails and never blocks: a full ring drops its oldest entry.
 *
 * @param[in] length	frame length as the driver reported it
 * @param[in] ranging	the frame's ranging bit
 */
void capture_put(size_t length, bool ranging);

/**
 * Take the oldest frame not yet consumed.
 *
 * @param[out] out	the frame, copied out of the ring
 * @return true		a frame was copied into @p out
 * @return false	the ring is empty
 */
bool capture_get(struct capture_frame *out);

/**
 * The counters, for reporting. Never NULL.
 */
const struct capture_stats *capture_stats(void);

/**
 * Flatten a captured frame into the shape a dissector sees.
 *
 * The conversion lives here, next to the struct it reads, rather than in
 * main.c where its only caller is: this is the one place the two
 * representations meet, and putting it here is what lets
 * tests/sniffer/capture.c check the mapping. dissect.h explains why the
 * flat form exists at all (in short: @p capture_frame's size depends on
 * a driver compile-time option, so it must not cross a plugin boundary).
 *
 * @p out->data points into @p in, so @p in must outlive @p out.
 *
 * A frame captured without the metadata read-out leaves the radio fields
 * at their "nothing was read" values (@p rx_time 0, both powers
 * @p DISSECT_POWER_NONE, @p clock_interval 0) rather than at whatever
 * the uninitialised slot held, so that a dissector cannot mistake stale
 * bytes for a reading.
 *
 * @param[out] out	the flat form
 * @param[in]  in	the captured frame
 */
void capture_to_dissect(struct dissect_frame *out,
			const struct capture_frame *in);

#endif

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
