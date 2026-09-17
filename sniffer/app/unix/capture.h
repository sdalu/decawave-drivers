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
#include <dw1000/dw1000.h>

/**
 * One frame, as it was taken off the chip.
 *
 * @p length is what is held in @p data, not what the driver reported: a
 * frame longer than a slot is truncated to fit and counted, and sending
 * the reported length out of a buffer that does not hold it would read
 * past the end. The CRC is included, as it is in the driver's own report
 * (see @p dw1000_rx_get_frame_info(); @p DW1000_CRC_LENGTH is what to
 * subtract for the payload alone) -- a sniffer forwards the frame whole,
 * so nothing is subtracted here.
 */
struct capture_frame {
    uint8_t data[DW1000_FRAME_MAXSIZE];
    size_t  length;
};

/**
 * What the ring did, for the report on the way out.
 *
 * @p overrun is the only way a received frame is lost here, exactly as it
 * is in probe/src/exchange.c: the producer never refuses a frame, so a
 * consumer that falls more than the ring's depth behind loses the oldest
 * entries and finds out on its next read.
 */
struct capture_stats {
    unsigned long captured;	/**< frames read off the chip           */
    unsigned long consumed;	/**< frames handed to the consumer      */
    unsigned long overrun;	/**< frames overwritten before that     */
    unsigned long truncated;	/**< frames longer than a slot          */
};

/**
 * Take the reported frame off the chip and into the ring.
 *
 * MUST be called from the rx_ok callback and MUST NOT be deferred: under
 * double buffering the driver toggles the host side buffer pointer as
 * soon as that callback returns, and RX_BUFFER swings with it. This is
 * the whole reason the ring exists -- the read-out happens now, the
 * forwarding happens later.
 *
 * Never fails and never blocks: a full ring drops its oldest entry.
 *
 * @param[in] length	frame length as the driver reported it
 */
void capture_put(size_t length);

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

#endif
