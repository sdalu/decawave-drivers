/*
 * Copyright (c) 2018-2024
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdarg.h>

#include "dw1000/osal.h"
#include "dw1000/dw1000.h"
#include "dw1000/dw1000_send.h"



/*===========================================================================*/
/* Local variables and types                                                 */
/*===========================================================================*/

union dw1000_timestamp_encoding {
    uint64_t uint64;
    char raw[sizeof(uint64_t)];
};



/*===========================================================================*/
/* Local functions                                                           */
/*===========================================================================*/

static inline size_t
_dw1000_tx_prepare_data_sendv(
	dw1000_t *dw, struct iovec *iovec, int iovcnt, int tx_mode)
{
    size_t length = 0;

    // Write data to DW TX buffer and compute offset/length
    for ( ; iovcnt > 0 ; iovec++, iovcnt--) {
	dw1000_tx_write_frame_data(dw, iovec->iov_base, iovec->iov_len, length);
	length += iovec->iov_len;
    }

    // Return total length
    return length;
}


static inline size_t
_dw1000_tx_prepare_data_send(
	dw1000_t *dw, uint8_t *data, size_t length, int tx_mode)
{
    // Write data to DW TX buffer
    dw1000_tx_write_frame_data(dw, data, length, 0);

    // Return total length
    return length;
}

static inline void
_dw1000_tx_prepare_fctl(dw1000_t *dw, size_t length, int tx_mode)
{
    // Adjust data length if CRC is automatically appended
    if (! (tx_mode & DW1000_TX_NO_AUTO_CRC))
	length += DW1000_CRC_LENGTH;
    // Set frame control parameters
    dw1000_tx_fctrl(dw, length, 0, tx_mode);
}

static inline void
_dw1000_tx_prepare_delayed_embed_timestamp(
	dw1000_t *dw, size_t offset, uint32_t delay, int tx_mode)
{
    union dw1000_timestamp_encoding data = { 0 };
    size_t                          size = 0;
    uint64_t                        time;

    // Compute delayed send
    time = dw1000_get_system_time(dw);
    time = DW1000_CLOCK_ROUNDUP(time + delay);
    
    // Set delayed time
    dw1000_txrx_set_time(dw, time);
    
    // Adjust time to take into account antenna delay
    // when embedding timestamp
    time += dw->config->tx_antenna_delay;
    
    // Build timestamp data
    if (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT) {
	size = sizeof(uint64_t);
	if (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN) {
	    data.uint64 = dw1000_cpu_to_le64(time); 	   // Little
	} else {
	    data.uint64 = dw1000_cpu_to_be64(time); 	   // Big
	}
    } else {
	size = 5;
	if (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN) {
	    for (int i = 0 ; i < 5 ; i++)
		data.raw[i] = (time >> (i     * 8)) & 0xff; // Little
	} else {
	    for (int i = 0 ; i < 5 ; i++)
		data.raw[i] = (time >> ((4-i) * 8)) & 0xff; // Big
	}
    }

    // Embed timestamp
    dw1000_tx_write_frame_data(dw, data.raw, size, offset);
}



/*===========================================================================*/
/* Exported functions                                                        */
/*===========================================================================*/

int
dw1000_tx_sendv(
	dw1000_t *dw, struct iovec *iovec, int iovcnt, int tx_mode)
{
    // Prepare data and frame control
    size_t length = _dw1000_tx_prepare_data_sendv(dw, iovec, iovcnt, tx_mode);
    _dw1000_tx_prepare_fctl(dw, length, tx_mode);
    // Start trasmit
    return dw1000_tx_start(dw, tx_mode);
}

int
dw1000_tx_send(
	dw1000_t *dw, uint8_t *data, size_t length, int tx_mode)
{
    // Prepare data and frame control
    _dw1000_tx_prepare_data_send(dw, data, length, tx_mode);
    _dw1000_tx_prepare_fctl(dw, length, tx_mode);
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
    size_t length = _dw1000_tx_prepare_data_sendv(dw, iovec, iovcnt, tx_mode);
    _dw1000_tx_prepare_fctl(dw, length, tx_mode);

    // If not embedding timestamp, send it now
    if (! (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP)) {
	return dw1000_tx_start(dw, tx_mode);
    }

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
    }
    if (tx_mode & DW1000_TX_DELAYED_RETRY_DELAY) {
	retry_delay = va_arg(ap, uint32_t);
    }

 retry:
    // Check if zero-delay
    if (delay == 0)
	return -1;
	
    // Prepare embedded timestamp and start transmit
    _dw1000_tx_prepare_delayed_embed_timestamp(dw, offset, delay, tx_mode);
    rc = dw1000_tx_start(dw, tx_mode);
    if ((rc < 0) && !last_try) {
	delay    = retry_delay;
	last_try = true;
	goto retry;
    }
    return rc;
}

#endif
