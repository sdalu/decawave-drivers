/*
 * Copyright (c) 2018-2024
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_SEND_H__
#define __DW1000_SEND_H__

/**
 * @file    dw1000_send.h
 * @brief   DW1000 send functions
 *
 * @addtogroup DW1000
 * @{
 */

#include <stdarg.h>

#include "dw1000/osal.h"
#include "dw1000.h"


/*===========================================================================*/
/* Compile time options                                                      */
/*===========================================================================*/

/**
 * \defgroup Config Compile time configuration
 */
/** @{ */

/**
 * @brief Build with extended send function
 */
#if !defined(DW1000_WITH_EXTENDED_SEND) || defined(__DOXYGEN__)
#define DW1000_WITH_EXTENDED_SEND 1
#endif

/**
 * @brief Microseconds to ticks of the device clock (DW1000_TIME_CLOCK_HZ)
 *
 * @note  Usable in preprocessor conditionals (no cast): the result is an
 *        unsigned long long, to be assigned to a uint32_t delay (fits
 *        below ~67 ms).
 */
#define DW1000_TX_DELAYED_US(us)					\
    ((DW1000_TIME_CLOCK_HZ * (us)) / 1000000)

/** @} */






/*===========================================================================*/
/* Constants                                                                 */
/*===========================================================================*/


#if DW1000_WITH_EXTENDED_SEND

/**
 * @brief Specify an additional parameter for transmit delay
 * @note  The value is the lead to the RMARKER, used exactly as given,
 *        as the default set by @p dw1000_tx_set_default_delay() also is.
 *        It must therefore exceed @p dw1000_tx_get_preamble_airtime() on
 *        its own, or the send is refused with -1.
 */
#define DW1000_TX_DELAYED_DELAY					0x2000
/**
 * @brief Specify an additional parameter for tramist delay when retrying
 * @note  Used exactly as given, as @p DW1000_TX_DELAYED_DELAY is.
 */
#define DW1000_TX_DELAYED_RETRY_DELAY				0x4000

/**
 * @brief Embed timestamp in the trasmitted frame.
 * @note  Implies @p DW1000_TX_DELAYED_START: the embedded value is the
 *        programmed transmission time plus the antenna delay, which is
 *        only meaningful for a delayed send, so the extended send sets
 *        that flag whether or not the caller passed it.
 */
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP			0x1000
/**
 * @brief Use big-endian encoding for timestamp
 */
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP_BIG_ENDIAN		0x0100
/**
 * @brief Use little-endian encoding for timestamp
 */
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN		0x0200
/**
 * @brief Use 40 bit encoding for timestamp
 */
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP_40BIT			0x0400
/**
 * @brief Use 64 bit encoding for timestamp
 */
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT 		0x0800

/**
 * Bitmask for endianess of embedded timestamp
 */
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP_ENDIAN_MASK           0x0300
/**
 * Bitmask for size of embedded timestamp
 */
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP_SIZE_MASK             0x0C00


#endif


/*===========================================================================*/
/* Send                                                                      */
/*===========================================================================*/

/**
 * @brief Set the default leads used by a delayed send
 *
 * @details @p dw1000_tx_extended_sendv() uses these whenever the caller
 *          passes no @p DW1000_TX_DELAYED_DELAY of its own, and uses
 *          them exactly as it uses that one: as the lead to the
 *          programmed RMARKER, with nothing added. A lead means the same
 *          thing here as it does per call.
 *
 * @note  Both are 0 until this is called, and 0 means "no default":
 *        @p dw1000_tx_extended_sendv() then refuses any send that does
 *        not carry its own @p DW1000_TX_DELAYED_DELAY. There is no build
 *        time default to fall back on; the lead is a property of the
 *        host, so the host states it.
 * @note  What to pass: two terms, both the caller's to add up.
 *        @p dw1000_tx_get_preamble_airtime() is the airtime of the
 *        preamble and SFD, which the chip must have on the air before
 *        the programmed RMARKER (APS022 §5.4), about 138 us at a 128
 *        symbol preamble but 4.2 ms at 4096. On top of it goes the
 *        host's own latency: between the read of the system time and
 *        TXSTRT sit the DX_TIME write, the timestamp write and the
 *        SYS_CTRL write, plus the chip's transmit power-up.
 * @note  What the estimators report is the sum, not the second term.
 *        Both spank's delayed-send estimator and the gem's
 *        DW1000#estimate_delayed_send_lead_time binary-search the delay
 *        they pass, which is the whole lead, so their figures already
 *        carry the airtime of whatever preamble was configured.
 *        Measured on 2026-09-14 at a 128 symbol preamble (138 us of
 *        airtime): 0.27 ms on an nRF52 at 8 or 16 MHz SPI, 0.16 ms on a
 *        Raspberry Pi 4 over spidev at 20 MHz, 0.173 ms on a
 *        Raspberry Pi 4B. Subtracting the airtime leaves roughly 132 us,
 *        22 us and 35 us of host latency. Take such a figure as a lead
 *        for that preamble, not as a term to add one to.
 * @note  The airtime term is only known once @p dw1000_configure() has
 *        run, and it changes with the preamble: call this after the
 *        radio is configured, and again if it is reconfigured. A lead
 *        left over from a shorter preamble is refused, so the mistake
 *        shows up as -1 rather than as a lost frame.
 * @note  @p retry follows an attempt the chip called too late, so it must
 *        not be given less lead than that attempt had; one and a half
 *        times @p initial is a reasonable choice, and is the factor
 *        applied at run time to a caller-supplied delay that carries no
 *        retry of its own. A first attempt refused by a scheduling blip
 *        does not need its lead doubled, and every extra microsecond of
 *        lead is clock drift the frame carries.
 * @note  Only a @p DW1000_CLOCK_MIN rounded time is honoured.
 *
 * @param dw       driver context
 * @param initial  whole lead for the first attempt, airtime included.
 *                 0 leaves no default
 * @param retry    the same for the retry, not below @p initial. 0
 *                 disables the retry
 *
 * @retval  0   accepted
 * @retval -1   @p retry is shorter than @p initial, and neither is 0
 */
static inline int
dw1000_tx_set_default_delay(dw1000_t *dw, uint32_t initial, uint32_t retry)
{
    if ((initial != 0) && (retry != 0) && (retry < initial))
	return -1;
    dw->tx_delay       = initial;
    dw->tx_retry_delay = retry;
    return 0;
}


/**
 * @brief Send a frame
 *
 * @pre    The DW1000 is in IDLE state.
 *         The dw1000_txrx_off() function need to be called if necessary.
 *
 * @note   According to the @p DW1000_TX_NO_AUTO_CRC flag, if unset
 *         transmitted frame will have the CRC automatically computed
 *         and appended to the frame so transmitted frame will be length+2; if
 *         set, transmitted frame length will be of the specified length
 *         but a CRC-16-CCITT must be explicitely embedded in the frame data
 *
 * @note   If using @p DW1000_TX_DELAYED_START, the transmission time
 *         should have been previously set using @p dw1000_txrx_set_time
 *
 * @param dw        driver context
 * @param data      data to send
 * @param length    length of the data
 * @param tx_mode   a set of the following flags are supported:
 *                  DW1000_TX_DELAYED_START, DW1000_TX_RESPONSE_EXPECTED,  
 *                  DW1000_TX_RANGING, DW1000_TX_NO_AUTO_CRC
 *
 * @retval  0        Transmission started
 * @retval -1        It was not possible to start transmission.
 *                   (Can happen when @p DW1000_TX_DELAYED_START is set:
 *                   the chip reported the time as already past, or the
 *                   requested lead was at or below the preamble and
 *                   SFD airtime, reported by
 *                   @p dw1000_tx_get_preamble_airtime(), which no host
 *                   could meet)
 */
int dw1000_tx_send(dw1000_t *dw,
		   uint8_t *data, size_t length, int tx_mode);

/**
 * @brief Send a frame
 *
 * @pre    The DW1000 is in IDLE state.
 *         The dw1000_txrx_off() function need to be called if necessary.
 *
 * @note   According to the @p DW1000_TX_NO_AUTO_CRC flag, if unset
 *         transmitted frame will have the CRC automatically computed
 *         and appended to the frame so transmitted frame will be length+2; if
 *         set, transmitted frame length will be of the specified length
 *         but a CRC-16-CCITT must be explicitely embedded in the frame data
 *
 * @note   If using @p DW1000_TX_DELAYED_START, the transmission time
 *         should have been previously set using @p dw1000_txrx_set_time
 *
 * @param dw        driver context
 * @param iovec     io vector
 * @param iovcnt    number of elements in vector
 * @param tx_mode   a set of the following flags are supported:
 *                  DW1000_TX_DELAYED_START, DW1000_TX_RESPONSE_EXPECTED,  
 *                  DW1000_TX_RANGING, DW1000_TX_NO_AUTO_CRC
 *
 * @retval  0        Transmission started
 * @retval -1        It was not possible to start transmission.
 *                   (Can happen when @p DW1000_TX_DELAYED_START is set)
 */
int dw1000_tx_sendv(dw1000_t *dw,
		    struct iovec *iovec, int iovcnt, int tx_mode);


#if DW1000_WITH_EXTENDED_SEND

/**
 * @brief  Start sending a frame
 *
 * @pre    The DW1000 is in IDLE state.
 *         The dw1000_txrx_off() function need to be called if necessary.
 *
 * @note   According to the @p DW1000_TX_NO_AUTO_CRC flag, if unset
 *         transmitted frame will have the CRC automatically computed
 *         and appended to the frame so transmitted frame will be length+2; if
 *         set, transmitted frame length will be of the specified length
 *         but a CRC-16-CCITT must be explicitely embedded in the frame data
 *
 * @note   If using @p DW1000_TX_DELAYED_START, without
 *         @p DW1000_TX_DELAYED_EMBED_TIMESTAMP the transmission time
 *         should have been previously set using dw1000_txrx_set_time().
 *         In this case fonction as the same behaviour as dw1000_tx_send().
 *
 * @note   If using @p DW1000_TX_DELAYED_START with
 *         @p DW1000_TX_DELAYED_EMBED_TIMESTAMP, the driver will perform
 *         a delayed send to automatically embed timestamp into the packet.
 *         An extra paramater is required @a ts_offset (@p size_t).
 *         The following @a tx_mode values specified how the timestamp
 *         shoud be encoded: DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN,
 *                           DW1000_TX_DELAYED_EMBED_TIMESTAMP_BIG_ENDIAN,
 *                           DW1000_TX_DELAYED_EMBED_TIMESTAMP_40BIT,
 *                           DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT.
 *         If specified @p DW1000_TX_DELAYED_DELAY and
 *         @p DW1000_TX_DELAYED_RETRY_DELAY additional
 *         parameter are used to set the necessary delay to perform
 *         frame encoding/processing before transmit, respectively
 *         @a delay (@p uint32_t) and @a retry_delay (@p uint32_t).
 *         Theses delay are highly system specific and should be choosen
 *         carrefully otherwise sending frame will fails.
 *
 * @note   The two forms mean the same thing: a lead to the programmed
 *         RMARKER, used as given. @a delay passed with
 *         @p DW1000_TX_DELAYED_DELAY applies to that send alone, the
 *         pair held by @p dw1000_tx_set_default_delay() to every send
 *         that carries none. Either way the caller owns the whole lead,
 *         the preamble and SFD airtime
 *         (@p dw1000_tx_get_preamble_airtime()) included, and a lead
 *         that cannot be honoured is refused with -1 rather than handed
 *         to the chip. With no default set and no @a delay passed, the
 *         send is refused.
 *
 *
 * @param dw        driver context
 * @param iovec     io vector
 * @param iovcnt    number of elements in vector
 * @param tx_mode   a set of the following flags/values are supported:
 *                  @p DW1000_TX_RESPONSE_EXPECTED,
 *                  @p DW1000_TX_RANGING,
 *                  @p DW1000_TX_NO_AUTO_CRC,
 *                  @p DW1000_TX_DELAYED_START,
 *                  @p DW1000_TX_DELAYED_DELAY,
 *                  @p DW1000_TX_DELAYED_RETRY_DELAY,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_BIG_ENDIAN,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_40BIT,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT
 * @param ts_offset Timestamp placement in the frame (size_t),
 *                  offset is specified from the start of the frame.
 *                  Encoding will be done according to @p tx_mode parameter.
 *                  The timestamp is written both in the chip's transmit
 *                  buffer and, at the same offset, in the caller's @p iovec
 *                  data: on return the caller's frame carries the exact
 *                  time the frame leaves (the value TX_STAMP will report,
 *                  antenna delay included), without waiting for the
 *                  completion. The caller's buffers must therefore be
 *                  writable.
 *                  (parameter only available if @p DW1000_TX_DELAYED_EMBED_TIMESTAMP is specified)
 * @param delay     Scheduling delay (uint32_t)
 *                  (parameter only available if @p DW1000_TX_DELAYED_DELAY is specified)
 * @param retry_delay Scheduling delay for second attempt (uint32_t)
 *                  (parameter only available if @p DW1000_TX_DELAYED_RETRY_DELAY is specified)
 * @param ap        variadic arguments (ts_offset, delay, retry_delay) as
 *                  described above, packed in a @p va_list
 *
 * @retval  0        Transmission started
 * @retval -1        It was not possible to start transmission.
 *                   (Can happen when @p DW1000_TX_DELAYED_START is set)
 */
int dw1000_tx_extended_vsendv(dw1000_t *dw,
			      struct iovec *iovec, int iovcnt,
			      int tx_mode,
			      va_list ap);

static inline
int dw1000_tx_extended_sendv(dw1000_t *dw,
			     struct iovec *iovec, int iovcnt,
			     int tx_mode,
			     ...) {
    va_list ap;
    va_start(ap, tx_mode);
    int rc = dw1000_tx_extended_vsendv(dw, iovec, iovcnt, tx_mode, ap);
    va_end(ap);
    return rc;
}

static inline int
dw1000_tx_extended_send(dw1000_t *dw,
			uint8_t *data, size_t length, int tx_mode, ...) {
    struct iovec iovec = { .iov_base = data,
			   .iov_len  = length };
    va_list ap;
    va_start(ap, tx_mode);
    int rc = dw1000_tx_extended_vsendv(dw, &iovec, 1, tx_mode, ap);
    va_end(ap);
    return rc;
}

#endif


/** @} */

#endif
