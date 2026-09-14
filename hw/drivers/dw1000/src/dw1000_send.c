/*
 * Copyright (c) 2018-2024
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdarg.h>
#include <string.h>

#include "dw1000/osal.h"
#include "dw1000/dw1000.h"
#include "dw1000/dw1000_send.h"


/*===========================================================================*/
/* Local functions                                                           */
/*===========================================================================*/

static inline size_t
_dw1000_tx_prepare_data_sendv(
	dw1000_t *dw, struct iovec *iovec, int iovcnt)
{
    size_t length = 0;

    for ( ; iovcnt > 0 ; iovec++, iovcnt--) {
	dw1000_tx_write_frame_data(dw, iovec->iov_base, iovec->iov_len, length);
	length += iovec->iov_len;
    }

    return length;
}


static inline size_t
_dw1000_tx_prepare_data_send(
	dw1000_t *dw, uint8_t *data, size_t length)
{
    dw1000_tx_write_frame_data(dw, data, length, 0);

    return length;
}

static inline void
_dw1000_tx_prepare_fctrl(dw1000_t *dw, size_t length, int tx_mode)
{
    // Adjust data length if CRC is automatically appended
    if (! (tx_mode & DW1000_TX_NO_AUTO_CRC))
	length += DW1000_CRC_LENGTH;
    // Set frame control parameters
    dw1000_tx_fctrl(dw, length, 0, tx_mode);
}

#if DW1000_WITH_EXTENDED_SEND
/* The two helpers below serve dw1000_tx_extended_vsendv() alone, and use
 * the DW1000_TX_DELAYED_EMBED_TIMESTAMP_* constants that dw1000_send.h
 * only defines for the extended send, so they are guarded with it. */

/* Copy `size` bytes of `data` into the caller's scattered buffer at
 * frame offset `offset`, spanning segments if need be, so the caller's
 * copy of the frame carries the same timestamp as the one in the chip.
 * Bytes falling past the end of the buffer are not written (the caller
 * gave an offset beyond its own data: the chip still got them). */
static inline void
_dw1000_iovec_write(struct iovec *iovec, int iovcnt,
		    size_t offset, const uint8_t *data, size_t size)
{
    for ( ; (iovcnt > 0) && (size > 0) ; iovec++, iovcnt--) {
	if (offset >= iovec->iov_len) {
	    offset -= iovec->iov_len;
	    continue;
	}
	size_t n = iovec->iov_len - offset;
	if (n > size)
	    n = size;
	memcpy((uint8_t *)iovec->iov_base + offset, data, n);
	data   += n;
	size   -= n;
	offset  = 0;
    }
}

static inline bool
_dw1000_tx_prepare_delayed_embed_timestamp(
	dw1000_t *dw, struct iovec *iovec, int iovcnt,
	size_t offset, uint32_t delay, int tx_mode)
{
    // Sanity check
    DW1000_ASSERT(((tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_ENDIAN_MASK)
		   == DW1000_TX_DELAYED_EMBED_TIMESTAMP_BIG_ENDIAN   ) ||
		  ((tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_ENDIAN_MASK)
		   == DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN),
		  "invalid tx_mode for timestamp endianess");
    DW1000_ASSERT(((tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_SIZE_MASK)
		   == DW1000_TX_DELAYED_EMBED_TIMESTAMP_40BIT        ) ||
		  ((tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_SIZE_MASK)
		   == DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT        ),
		  "invalid tx_mode for timestamp size");

    /* The asserts above are compiled out on four of the five ports, so
     * both fields are checked here as well, and before anything is
     * written to the chip: an unused value used to leave `size` at 0 or
     * `data` unfilled, and the frame went out without its timestamp --
     * silently, since the caller's copy was left unfilled too.
     */
    size_t size;
    switch(tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_SIZE_MASK) {
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_40BIT: size = 5; break;
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT: size = 8; break;
    default: return false;
    }
    switch(tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_ENDIAN_MASK) {
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN:
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_BIG_ENDIAN:
	break;
    default: return false;
    }

    uint8_t  data[8] = { 0 };
    uint64_t time;

    // Compute delayed send
    time = dw1000_get_system_time(dw);
    time = DW1000_CLOCK_ROUNDUP(time + delay);
    
    // Set delayed time
    dw1000_txrx_set_time(dw, time);
    
    // Adjust time to take into account antenna delay
    // when embedding timestamp
    time += dw->config->tx_antenna_delay;

    // Keep the value within the 40-bit device clock: the DX_TIME
    // register write truncates naturally, but the 64-bit embed below
    // would otherwise leak the wrap carry into byte 5
    time &= (1ull << DW1000_TIME_CLOCK_BITS) - 1;

    // Build timestamp data
    switch(tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_ENDIAN_MASK) {
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN:
	for (size_t i = 0 ; i < size ; i++)
	    data[i] = (time >> (i     * 8)) & 0xff;
	break;
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_BIG_ENDIAN:
	for (size_t i = 0 ; i < size ; i++)
	    data[i] = (time >> ((size - 1 - i) * 8)) & 0xff;
	break;
    default: return false; // unreachable, validated above
    }
    
    // Embed timestamp, in the chip's transmit buffer and in the caller's
    // copy of the frame alike: the caller then knows the exact time the
    // frame goes out (TX_STAMP will report the same value on completion,
    // a self-check for the host), without waiting for that completion.
    dw1000_tx_write_frame_data(dw, data, size, offset);
    _dw1000_iovec_write(iovec, iovcnt, offset, data, size);

    return true;
}
#endif



/*===========================================================================*/
/* Exported functions                                                        */
/*===========================================================================*/

int
dw1000_tx_sendv(
	dw1000_t *dw, struct iovec *iovec, int iovcnt, int tx_mode)
{
    // Prepare data and frame control
    size_t length = _dw1000_tx_prepare_data_sendv(dw, iovec, iovcnt);
    _dw1000_tx_prepare_fctrl(dw, length, tx_mode);
    // Start trasmit
    return dw1000_tx_start(dw, tx_mode);
}

int
dw1000_tx_send(
	dw1000_t *dw, uint8_t *data, size_t length, int tx_mode)
{
    // Prepare data and frame control
    _dw1000_tx_prepare_data_send(dw, data, length);
    _dw1000_tx_prepare_fctrl(dw, length, tx_mode);
    // Start trasmit
    return dw1000_tx_start(dw, tx_mode);
}



#if DW1000_WITH_EXTENDED_SEND

int
dw1000_tx_extended_vsendv(
	dw1000_t *dw, struct iovec *iovec, int iovcnt, int tx_mode,
	va_list ap)
{
    // Prepare data and frame control
    size_t length = _dw1000_tx_prepare_data_sendv(dw, iovec, iovcnt);
    _dw1000_tx_prepare_fctrl(dw, length, tx_mode);

    // If not embedding timestamp, send it now
    if (! (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP)) {
	return dw1000_tx_start(dw, tx_mode);
    }

    // Embedding a timestamp only makes sense for a delayed send: the
    // value embedded below is DX_TIME plus the antenna delay, and
    // dw1000_tx_start() arms DX_TIME (TXDLYS alongside TXSTRT, UM §3.3
    // p. 26) only when DW1000_TX_DELAYED_START is set. Without it the
    // frame would leave immediately carrying a timestamp a delay into
    // the future, and the late-send check would be skipped. The header
    // states the pairing as a @pre; enforce it rather than trust it.
    tx_mode |= DW1000_TX_DELAYED_START;

    // Default parameters
    int      rc          = 0;
    bool     last_try    = false;
    size_t   offset      = 0;
    uint32_t delay       = DW1000_TX_DELAYED_DEFAULT_DELAY;
    uint32_t retry_delay = DW1000_TX_DELAYED_DEFAULT_RETRY_DELAY;
    
    // Retrieve variadic arguments
    offset = va_arg(ap, size_t);
    if (tx_mode & DW1000_TX_DELAYED_DELAY) {
	delay       = va_arg(ap, uint32_t);
	// The retry follows an attempt the chip reported as too late
	// (HPDWARN), so it must not be given *less* lead time than that
	// attempt had. The #error in dw1000_send.h only relates the two
	// compile time defaults; a caller raising the delay here would
	// otherwise keep a retry sized for the default -- a 600us retry
	// for a 10ms delay, which cannot help precisely on the slow hosts
	// that raised it. Give the retry one and a half times the delay
	// (a first attempt refused by a scheduling blip does not need the
	// lead time doubled, and every extra microsecond of lead time is
	// clock drift the frame carries), saturating rather than wrapping,
	// and keep a compile time default of 0 meaning "retries disabled".
	if (DW1000_TX_DELAYED_DEFAULT_RETRY_DELAY != 0)
	    retry_delay = (delay > (UINT32_MAX - UINT32_MAX / 3))
		        ? UINT32_MAX : (delay + delay / 2);
    }
    // An explicit retry delay is the caller's own choice and is honoured
    // as given.
    if (tx_mode & DW1000_TX_DELAYED_RETRY_DELAY) {
	retry_delay = va_arg(ap, uint32_t);
    }

 retry:
    // Check if zero-delay
    if (delay == 0)
	return -1;
	
    // Prepare embedded timestamp and start transmit
    if (! _dw1000_tx_prepare_delayed_embed_timestamp(dw, iovec, iovcnt,
						     offset, delay, tx_mode))
	return -1;
    rc = dw1000_tx_start(dw, tx_mode);
    if ((rc < 0) && !last_try) {
	delay    = retry_delay;
	last_try = true;
	goto retry;
    }
    return rc;
}

#endif
