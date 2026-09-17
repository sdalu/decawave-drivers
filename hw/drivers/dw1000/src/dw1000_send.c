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

/* Whether the transmitter is free for another frame.
 *
 * Errata 1.4 §3.3 (TX-2): writing the TX buffer while a transmission is
 * in progress corrupts what is being transmitted, and the write happens
 * before anything here could refuse it. So the question is asked first,
 * before a single byte goes to the chip.
 *
 * The send functions document "the DW1000 is in IDLE state" as a
 * precondition and used to take the caller's word for it. A caller that
 * sent faster than the air allows was accepted every time and lost
 * nearly every frame: measured on the bench at 6.8 Mbps, sending a 27
 * byte frame every 0.15 ms against 0.17 ms of airtime delivered 1 frame
 * out of 20000, silently, with every call returning success.
 *
 * dw->tx_pending is the driver's own record of a send it started and has
 * not seen reported done, so this costs no SPI at all. It is cleared
 * wherever a transmission ends: the TXFRS branch of
 * dw1000_process_events(), dw1000_tx_clear_status_done() for a host that
 * polls instead, _dw1000_txrx_off() which stops the transmitter outright,
 * the late path of dw1000_tx_start(), and the soft reset. A host that
 * uses none of those never learns its frames went out either, and is the
 * host this refusal exists to tell.
 */
static inline bool
_dw1000_tx_idle(const dw1000_t *dw)
{
    return dw->tx_pending == 0;
}

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

static inline int
_dw1000_tx_prepare_fctrl(dw1000_t *dw, size_t length, int tx_mode)
{
    /* Errata 1.4 §3.2 (RX-1). "The 129th octet (i.e. buffer offset
     * index[128]) of the second RX buffer (i.e. the one accessed when
     * HSRBP = 1) gets corrupted when the user writes TX data at offsets
     * greater than index 127, and issues a TX send command, before
     * reading the received frame."
     *
     * A payload of more than 128 bytes written from offset 0 reaches
     * index 128, so this is the send the erratum names, and the three
     * workarounds it offers are all "do not do that" (do not answer a
     * long message with one beyond 127 octets; keep messages short in
     * double buffered mode; use single buffering for long messages).
     * Refuse rather than corrupt a frame the host has not read yet.
     *
     * Reachable only with proprietary long frames. Without them
     * dw1000_tx_get_frame_maxsize() is 127, so the largest payload is
     * 125 and a host cannot write past index 124 whatever it does. That
     * is why this has never bitten anyone here, and why the test for it
     * needs its own build.
     *
     * Conservative in one direction, deliberately: the erratum names the
     * second buffer, HSRBP = 1, and this does not test HSRBP. Tracking
     * it would mean shadowing a pointer the overrun path toggles
     * unconditionally, or an SPI read on every send, to halve the
     * refusals of a case the erratum tells you not to build. dw->rx_held
     * is the window where a frame is reported and not yet released,
     * which is what "before reading the received frame" means here.
     */
    if (dw->config->dblbuff && dw->rx_held && (length > 128))
	return DW1000_TX_ERR_BUFFER_HELD;

    // Adjust data length if CRC is automatically appended
    if (! (tx_mode & DW1000_TX_NO_AUTO_CRC))
	length += DW1000_CRC_LENGTH;

    // Refuse a frame the chip cannot carry, rather than let
    // dw1000_tx_fctrl() clamp it. That clamp is the right behaviour for
    // a caller writing TX_FCTRL itself, which has already decided what
    // it means; here the length came from the caller's own payload, and
    // clamping silently drops its last bytes. The assert beside the
    // clamp does not help: it is compiled out on four of the five ports.
    // The frame the chip appends the CRC to is the one that must fit, so
    // the test is on the padded length, which is why it is here and not
    // before the padding above.
    if (length > dw1000_tx_get_frame_maxsize(dw))
	return DW1000_TX_ERR_FRAME_SIZE;

    // Set frame control parameters
    dw1000_tx_fctrl(dw, length, 0, tx_mode);
    return 0;
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

static inline int
_dw1000_tx_prepare_delayed_embed_timestamp(
	dw1000_t *dw, struct iovec *iovec, int iovcnt,
	size_t offset, size_t length, uint32_t delay, int tx_mode)
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
     * `data` unfilled, and the frame went out without its timestamp,
     * silently, since the caller's copy was left unfilled too.
     */
    size_t size;
    switch(tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_SIZE_MASK) {
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_40BIT: size = 5; break;
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT: size = 8; break;
    default: return DW1000_TX_ERR_MODE;
    }
    switch(tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_ENDIAN_MASK) {
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN:
    case DW1000_TX_DELAYED_EMBED_TIMESTAMP_BIG_ENDIAN:
	break;
    default: return DW1000_TX_ERR_MODE;
    }

    /* The timestamp has to fit inside the frame the caller described.
     * Neither write below says otherwise if it does not:
     * dw1000_tx_write_frame_data() clamps to the 1024 byte transmit
     * buffer, not to the frame length already in TX_FCTRL, so bytes
     * past the end are written where nothing transmits them, and
     * _dw1000_iovec_write() drops the same bytes from the caller's copy
     * by design. The frame would go out carrying a truncated timestamp,
     * the caller's copy would be truncated identically, so comparing
     * it against TX_STAMP would agree, and the send would report
     * success. Refuse instead, the way an unusable tx_mode above is
     * refused, and before anything is written to the chip.
     * Computed without overflow: offset alone can exceed the frame.
     */
    if ((offset > length) || ((length - offset) < size))
	return DW1000_TX_ERR_TIMESTAMP;

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
    default: return DW1000_TX_ERR_MODE; // unreachable, validated above
    }
    
    // Embed timestamp, in the chip's transmit buffer and in the caller's
    // copy of the frame alike: the caller then knows the exact time the
    // frame goes out (TX_STAMP will report the same value on completion,
    // a self-check for the host), without waiting for that completion.
    dw1000_tx_write_frame_data(dw, data, size, offset);
    _dw1000_iovec_write(iovec, iovcnt, offset, data, size);

    return 0;
}
#endif



/*===========================================================================*/
/* Exported functions                                                        */
/*===========================================================================*/

int
dw1000_tx_sendv(
	dw1000_t *dw, struct iovec *iovec, int iovcnt, int tx_mode)
{
    // Nothing is written until the transmitter is known free (TX-2)
    if (! _dw1000_tx_idle(dw))
	return DW1000_TX_ERR_BUSY;

    // Prepare data and frame control
    //   The payload is written to the chip before the length is checked;
    //   a refusal leaves it there, unsent, as an unusable tx_mode does.
    size_t length = _dw1000_tx_prepare_data_sendv(dw, iovec, iovcnt);
    int    rc     = _dw1000_tx_prepare_fctrl(dw, length, tx_mode);
    if (rc < 0)
	return rc;
    // Start trasmit
    return dw1000_tx_start(dw, tx_mode);
}

int
dw1000_tx_send(
	dw1000_t *dw, uint8_t *data, size_t length, int tx_mode)
{
    // Nothing is written until the transmitter is known free (TX-2)
    if (! _dw1000_tx_idle(dw))
	return DW1000_TX_ERR_BUSY;

    // Prepare data and frame control
    _dw1000_tx_prepare_data_send(dw, data, length);
    int rc = _dw1000_tx_prepare_fctrl(dw, length, tx_mode);
    if (rc < 0)
	return rc;
    // Start trasmit
    return dw1000_tx_start(dw, tx_mode);
}



#if DW1000_WITH_EXTENDED_SEND

int
dw1000_tx_extended_vsendv(
	dw1000_t *dw, struct iovec *iovec, int iovcnt, int tx_mode,
	va_list ap)
{
    // Nothing is written until the transmitter is known free (TX-2)
    if (! _dw1000_tx_idle(dw))
	return DW1000_TX_ERR_BUSY;

    // Prepare data and frame control
    size_t length = _dw1000_tx_prepare_data_sendv(dw, iovec, iovcnt);
    int    prep   = _dw1000_tx_prepare_fctrl(dw, length, tx_mode);
    if (prep < 0)
	return prep;

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
    //
    // A lead is the lead to the programmed RMARKER, whoever supplies it
    // and whichever way: the pair dw1000_tx_set_default_delay() holds is
    // used exactly as a caller-supplied DW1000_TX_DELAYED_DELAY is,
    // nothing added. Both therefore have to cover the preamble and SFD
    // airtime themselves, which dw1000_tx_get_preamble_airtime()
    // reports, as well as the host's own latency.
    //
    // 0 means the host never set one: the send then has to carry its own
    // delay, or be refused below.
    int      rc          = 0;
    bool     last_try    = false;
    size_t   offset      = 0;
    uint32_t delay       = dw->tx_delay;
    uint32_t retry_delay = dw->tx_retry_delay;
    
    // Retrieve variadic arguments
    offset = va_arg(ap, size_t);
    if (tx_mode & DW1000_TX_DELAYED_DELAY) {
	delay       = va_arg(ap, uint32_t);
	// The retry follows an attempt the chip reported as too late
	// (HPDWARN), so it must not be given *less* lead time than that
	// attempt had. dw1000_tx_set_default_delay() checks that for the
	// pair it holds; a caller raising the delay here would otherwise
	// keep a retry sized for that pair: a 600us retry for a 10ms
	// delay, which cannot help precisely on the slow hosts
	// that raised it. Give the retry one and a half times the delay
	// (a first attempt refused by a scheduling blip does not need the
	// lead time doubled, and every extra microsecond of lead time is
	// clock drift the frame carries), saturating rather than wrapping,
	// and keep a configured retry of 0 meaning "retries disabled".
	if (dw->tx_retry_delay != 0)
	    retry_delay = (delay > (UINT32_MAX - UINT32_MAX / 3))
		        ? UINT32_MAX : (delay + delay / 2);
    }
    // An explicit retry delay is the caller's own choice and is honoured
    // as given.
    if (tx_mode & DW1000_TX_DELAYED_RETRY_DELAY) {
	retry_delay = va_arg(ap, uint32_t);
    }

 retry:
    // Check the lead time is one the chip can honour.
    //
    // Zero means "no delay configured". A lead shorter than the preamble
    // and SFD airtime cannot be met either: the RMARKER the caller is
    // programming marks the *end* of the SFD, so the chip has to be
    // transmitting dw->tx_ton earlier than that (APS022 §5.4, figure 5).
    // Refuse here rather than leave it to the chip, which would report it
    // as HPDWARN one layer down; diagnosable, but only after the frame
    // and its timestamp have been written for nothing.
    //
    // Every lead reaches this the same way, so the test guards them all:
    // the pair set by dw1000_tx_set_default_delay() as much as one the
    // caller passed. A host that reconfigured the radio to a longer
    // preamble without revisiting its default finds out here.
    if ((delay == 0) || (delay <= dw->tx_ton))
	return DW1000_TX_ERR_LEAD;

    // Prepare embedded timestamp and start transmit
    prep = _dw1000_tx_prepare_delayed_embed_timestamp(dw, iovec, iovcnt,
						      offset, length, delay,
						      tx_mode);
    if (prep < 0)
	return prep;
    rc = dw1000_tx_start(dw, tx_mode);
    // Only a refusal more lead time can cure is worth another attempt:
    // the chip having found the time already past is exactly that, and
    // nothing else dw1000_tx_start() reports is.
    if ((rc == DW1000_TX_ERR_TOO_LATE) && !last_try) {
	delay    = retry_delay;
	last_try = true;
	goto retry;
    }
    return rc;
}

#endif
