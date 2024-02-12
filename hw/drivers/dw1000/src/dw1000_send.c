#include <string.h>
#include <stdarg.h>

#include "dw1000/osal.h"
#include "dw1000/dw1000.h"
#include "dw1000/dw1000_send.h"

union dw1000_timestamp_encoding {
    uint64_t uint64;
    char raw[sizeof(uint64_t)];
};


static inline
int _dw1000_tx_prepare_sendv(dw1000_t *dw,
		    struct iovec *iovec, int iovcnt, int tx_mode) {
    size_t length = 0;

    // Write data to DW TX buffer and compute offset/length
    for ( ; iovcnt > 0 ; iovec++, iovcnt--) {
	dw1000_tx_write_frame_data(dw, iovec->iov_base, iovec->iov_len, length);
	length += iovec->iov_len;
    }
    // Adjust data length if CRC is automatically appended
    if (! (tx_mode & DW1000_TX_NO_AUTO_CRC))
	length += DW1000_CRC_LENGTH;
    // Set frame control parameters
    dw1000_tx_fctrl(dw, length, 0, tx_mode);
}

static inline
void _dw1000_tx_prepare_send(dw1000_t *dw,
			   uint8_t *data, size_t length, int tx_mode) {
    // Write data to DW TX buffer
    dw1000_tx_write_frame_data(dw, data, length, 0);
    // Adjust data length if CRC is automatically appended
    if (! (tx_mode & DW1000_TX_NO_AUTO_CRC))
	length += DW1000_CRC_LENGTH;
    // Set frame control parameters
    dw1000_tx_fctrl(dw, length, 0, tx_mode);
}




int dw1000_tx_sendv(dw1000_t *dw,
		    struct iovec *iovec, int iovcnt, int tx_mode) {
    // Prepare data and frame control
    _dw1000_tx_prepare_sendv(dw, iovec, iovcnt, tx_mode);
    // Start transmit
    return dw1000_tx_start(dw, tx_mode);
}

int dw1000_tx_send(dw1000_t *dw,
		   uint8_t *data, size_t length, int tx_mode) {
    // Prepare data and frame control
    _dw1000_tx_prepare_send(dw, data, length, tx_mode);
    // Start transmit
    return dw1000_tx_start(dw, tx_mode);
}


#if DW1000_WITH_EXTENDED_SEND


int dw1000_tx_extended_vsendv(dw1000_t *dw,
			      struct iovec *iovec, int iovcnt,
			      int tx_mode, va_list ap) {
    // Prepare data and frame control
    _dw1000_tx_prepare_sendv(dw, iovec, iovcnt, tx_mode);


    // If not embed timestamp, send it now
    if (! (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP)) {
	return dw1000_tx_start(dw, tx_mode);
    }

    int                             rc             = 0;
    bool                            last_try       = false;
    uint64_t                        time;
    union dw1000_timestamp_encoding ts_data        = { 0 };
    size_t                          ts_size        = 0;
    size_t                          ts_offset      = 0;
    uint32_t                        ts_delay       =
	DW1000_TX_DELAYED_EMBED_TIMESTAMP_DEFAULT_DELAY;
    uint32_t                        ts_retry_delay =
	DW1000_TX_DELAYED_EMBED_TIMESTAMP_DEFAULT_RETRY_DELAY;
    
    // Retrieve variadic arguments
    ts_size = va_arg(ap, size_t);
    if (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_DELAY) {
	ts_delay       = va_arg(ap, uint32_t);
    }
    if (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_RETRY_DELAY) {
	ts_retry_delay = va_arg(ap, uint32_t);
    }
    
 retry:
    // Check if non zero delay
    if (ts_delay == 0)
	return -1;
	
    // Compute delayed send
    time = dw1000_get_system_time(dw);
    time = DW1000_CLOCK_ROUNDUP(time + ts_delay);
    
    // Set delayed time
    dw1000_txrx_set_time(dw, time);
    
    // Adjust time to take into account antenna delay
    // when embedding timestamp
    time += dw->config->tx_antenna_delay;
    
    // Build timestamp data
    if (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_64BIT) {
	ts_size = sizeof(uint64_t);
	if (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN) {
	    ts_data.uint64 = dw1000_cpu_to_le64(time); 	   // Little
	} else {
	    ts_data.uint64 = dw1000_cpu_to_be64(time); 	   // Big
	}
    } else {
	ts_size = 5;
	if (tx_mode & DW1000_TX_DELAYED_EMBED_TIMESTAMP_LITTLE_ENDIAN) {
	    for (int i = 0 ; i < 5 ; i++)
		ts_data.raw[i] = (time >> (i     * 8)) & 0xff; // Little
	} else {
	    for (int i = 0 ; i < 5 ; i++)
		ts_data.raw[i] = (time >> ((4-i) * 8)) & 0xff; // Big
	}
    }

    // Embed timestamp
    dw1000_tx_write_frame_data(dw, ts_data.raw, ts_size, ts_offset);
    
    // Start transmit
    rc = dw1000_tx_start(dw, tx_mode);
    if ((rc < 0) && !last_try) {
	ts_delay = ts_retry_delay;
	last_try = true;
	goto retry;
    }
    return rc;
}

#endif
