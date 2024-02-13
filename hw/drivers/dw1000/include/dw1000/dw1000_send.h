/*
 * Copyright (c) 2018-2024
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_SEND_H__
#define __DW1000_SEND_H__

/**
 * @file    dw1000_send.c
 * @brief   DW1000 send functions
 *
 * @addtogroup DW1000
 * @{
 */

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
 * @brief Value for default delay when embedding timestamp automatically
 *
 * @details Value can be between 0 and 2^32, in DW1000_TIME_CLOCK_HZ steps.
 *          A value of 0 means disabled.
 * @note  The value should be selected to allow enough time for the
 *        system and dw1000 chip to prepare the frame.
 * @note  Setting delay for TX, only consider a DW1000_CLOCK_MIN rounded time
 */
#if !defined(DW1000_TX_DELAYED_DEFAULT_DELAY) || defined(__DOXYGEN__)
#define DW1000_TX_DELAYED_DEFAULT_DELAY 0
#endif

/**
 * @brief Value for default delay when retrying to embed timestamp automatically
 *
 * @details Value can be between 0 and 2^32, in DW1000_TIME_CLOCK_HZ steps.
 *          A value of 0 means disabled.
 * @note  The value should be selected to allow enough time for the
 *        system and dw1000 chip to prepare the frame.
 * @note  Setting delay for TX, only consider a DW1000_CLOCK_MIN rounded time
 */
#if !defined(DW1000_TX_DELAYED_DEFAULT_RETRY_DELAY) || defined(__DOXYGEN__)
#define DW1000_TX_DELAYED_DEFAULT_RETRY_DELAY 0
#endif

/** @} */


/*===========================================================================*/
/* Constants                                                                 */
/*===========================================================================*/


#if DW1000_WITH_EXTENDED_SEND
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP			0x0100
// #define DW1000_TX_DELAYED_EMBED_TIMESTAMP_BIG_ENDIAN		0x0000
#define DW1000_TX_DELAYED_DELAY					0x1000
#define DW1000_TX_DELAYED_RETRY_DELAY				0x2000
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN		0x0200
// #define DW1000_TX_DELAYED_EMBED_TIMESTAMP_40BIT		0x0000
#define DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT 		0x0400
#endif


/*===========================================================================*/
/* Send                                                                      */
/*===========================================================================*/

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
 *                   (Can happen when @p DW1000_TX_DELAYED_START is set)
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
 *         If not specified default to big-endian 40-bit encoding.
 *         If specified DW1000_TX_DELAYED_EMBED_TIMESTAMP_DELAY,
 *         DW1000_TX_DELAYED_EMBED_TIMESTAMP_RETRY_DELAY additional
 *         parameter are used to specifield necessary delay to perform
 *         frame encoding/processing before transmit, respectively
 *         @a delay (@p uint32_t) and @a retry_delay (@p uint32_t).
 *         Theses delay are highly system specific and should be choosen
 *         carrefully otherwise sending frame will fails.
 *         
 *
 * @param dw        driver context
 * @param data      data to send
 * @param length    length of the data
 * @param tx_mode   a set of the following flags/values are supported:
 *                  @p DW1000_TX_RESPONSE_EXPECTED,
 *                  @p DW1000_TX_RANGING,
 *                  @p DW1000_TX_NO_AUTO_CRC,
 *                  @p DW1000_TX_DELAYED_START,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_BIG_ENDIAN,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_40BIT,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_DELAY,
 *                  @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_RETRY_DELAY
 * @param ts_offset Timestamp placement in the frame, offset is specified from
 *                  the start of the frame. Encoding will be done according
 *                  to @p tx_mode parameter.
 *                  (parameter only available if @p DW1000_TX_DELAYED_EMBED_TIMESTAMP is specified)
 * @param delay     Timestamp placement in the frame.
 *                  (parameter only available if @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_DELAY is specified)
 * @param retry_delay
 *                  (parameter only available if @p DW1000_TX_DELAYED_EMBED_TIMESTAMP_RETRY_DELAY is specified)
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
    int rc = dw1000_tx_extended_sendv(dw, iovec, iovcnt, tx_mode, ap);
    va_end(ap);
    return rc;
}

static inline int
dw1000_tx_extended_send(dw1000_t *dw,
			uint8_t *data, size_t length, uint8_t tx_mode, ...) {
    struct iovec iovec = { .iov_base = data,
			   .iov_len  = length };
    return dw1000_tx_sendv(dw, &iovec, 1, tx_mode);
}

#endif


/** @} */

#endif
