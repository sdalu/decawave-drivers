/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __WIRE__H
#define __WIRE__H

#include <stdint.h>
#include <stddef.h>

#include "capture.h"

/*
 * What goes in the ethernet payload, ahead of the frame.
 *
 * The payload used to be the frame and nothing else, which loses four
 * things. A frame shorter than 46 bytes cannot be recovered at all: the
 * NIC pads to ETH_ZLEN, so a 5-byte acknowledgement arrives as 60 bytes
 * of frame and garbage with no length to separate them, and on a
 * 802.15.4-shaped network that is most of the short traffic. A frame the
 * ring had to truncate goes out looking like a whole one. There is
 * nowhere to put what the chip knew about the frame (capture.h's
 * @p capture_meta), all of which is read in the callback and would
 * otherwise be dropped on the floor. And a receiver cannot see the gap
 * an overrun left, because nothing numbers the frames.
 *
 * So: a fixed 32-byte header, little endian, with an optional 32-byte
 * metadata block after it, and the frame after that. Every field is
 * naturally aligned at its offset, but the encoder writes byte by byte
 * rather than casting a struct over the buffer, because the sender is an
 * ARM Raspberry Pi and the receiver is whatever the user has: a layout
 * that depends on the compiler's padding is not a wire format.
 *
 * Little endian because both ends of this link are in practice little
 * endian and the DW1000's own registers are, so a big-endian receiver
 * byte-swaps and nothing else has to think about it.
 *
 *   off  size  field
 *     0     4  magic, "UWBS"
 *     4     1  version, WIRE_VERSION
 *     5     1  hdr_len, 32 or 32+32
 *     6     2  flags, WIRE_F_*
 *     8     4  seq, the capture's own frame number, 0-based
 *    12     2  reported_len, what the driver said the frame was
 *    14     2  captured_len, how many frame bytes follow the header
 *    16     4  lost_ring, cumulative, forwarding fell behind
 *    20     4  lost_chip, cumulative, both buffers held (RXOVRR)
 *    24     8  wall_ns, CLOCK_REALTIME at read-out, ns since the epoch
 *
 * then, iff WIRE_F_METADATA:
 *
 *    32     8  rx_time, RMARKER on the 40-bit chip clock
 *    40     4  clock_offset, RX_TTCKO, signed
 *    44     4  clock_interval, RX_TTCKI, 0 when unavailable
 *    48     4  power_signal, milli-dBm, WIRE_POWER_NONE when unavailable
 *    52     4  power_firstpath, milli-dBm, likewise
 *    56     2  first_path
 *    58     2  std_noise
 *    60     2  max_noise
 *    62     2  reserved, written as zero, ignored on read
 *
 * and then captured_len bytes of frame, CRC included.
 *
 * The magic and the version are both there on purpose. The magic is what
 * lets a receiver tell this stream from the bare-frame one this replaces,
 * which `--raw` still produces; the version is what lets a field be added
 * later without the magic having to change. A receiver checks both,
 * skips hdr_len rather than assuming 32, and is then free to ignore a
 * metadata block it does not understand.
 */

#define WIRE_MAGIC0		'U'
#define WIRE_MAGIC1		'W'
#define WIRE_MAGIC2		'B'
#define WIRE_MAGIC3		'S'

#define WIRE_VERSION		1

#define WIRE_HDR_SIZE		32u
#define WIRE_META_SIZE		32u
#define WIRE_MAX_SIZE		(WIRE_HDR_SIZE + WIRE_META_SIZE)

/** @name Header flags, mirroring capture.h's CAPTURE_F_* @{ */
#define WIRE_F_TRUNCATED	(1u << 0)
#define WIRE_F_METADATA		(1u << 1)
#define WIRE_F_RANGING		(1u << 2)
/** @} */

/** No usable power estimate; same sentinel capture.h uses. */
#define WIRE_POWER_NONE		INT32_MIN

/**
 * Encode the header for one captured frame.
 *
 * Writes @p WIRE_HDR_SIZE bytes, plus @p WIRE_META_SIZE more when
 * @p frame carries @p CAPTURE_F_METADATA. The frame bytes are NOT
 * copied: the caller sends header and payload as two iovecs, so that a
 * frame is never copied twice on its way out.
 *
 * @param[out] buf	at least @p WIRE_MAX_SIZE bytes
 * @param[in]  bufsz	how much @p buf holds
 * @param[in]  frame	the captured frame
 * @param[in]  lost_ring  cumulative frames lost to ring overrun
 * @param[in]  lost_chip  cumulative frames lost to chip overrun
 * @return		bytes written, or 0 if @p bufsz is too small
 */
size_t wire_encode(uint8_t *buf, size_t bufsz,
		   const struct capture_frame *frame,
		   unsigned long lost_ring, unsigned long lost_chip);

#endif

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
