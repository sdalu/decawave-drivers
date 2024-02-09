#include <string.h>

#include "dw1000/osal.h"
#include "dw1000/dw1000.h"


int dw1000_tx_send(dw1000_t *dw,
		   uint8_t *data, size_t length, int tx_mode) {

    // Write data to DW TX buffer
    dw1000_tx_write_frame_data(dw, data, length, 0);
    // Adjust data length if CRC is automatically appended
    if (! (tx_mode & DW1000_TX_NO_AUTO_CRC))
	length += DW1000_CRC_LENGTH;
    // Set transmission control parameters
    dw1000_tx_fctrl(dw, length, 0, tx_mode);
    // Start sending
    return dw1000_tx_start(dw, tx_mode);
}


int dw1000_tx_sendv(dw1000_t *dw,
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
    // Set transmission control parameters
    dw1000_tx_fctrl(dw, length, 0, tx_mode);
    // Start sending
    return dw1000_tx_start(dw, tx_mode);
}
