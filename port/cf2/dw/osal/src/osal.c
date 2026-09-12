/*
 * Copyright (c) 2018-2019
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "dw1000/osal.h"


#define DW1000_OSAL_SPI_BUFSIZE 196

// XXX: WTF using a before and not 2 spiExchange ?!
//
// NOTE: This port copies header + payload into a fixed bounce buffer
//       before a single spiExchange(). The buffer is only large enough
//       for standard frames (<= 127 bytes payload + 3 bytes header), so
//       this port does *not* support proprietary long frames. The guards
//       below trap any transfer that would overflow the buffer instead of
//       silently corrupting memory.
//
// NOTE: The bounce buffer is shared by every transfer, but both the fill
//       and the read back happen inside spiBeginTransaction() /
//       spiEndTransaction(), which is the deck SPI mutex, so two tasks
//       cannot be in it at once. Keep it that way: moving the copy out
//       of the transaction would turn it into a race.

static struct {
    uint8_t tx[DW1000_OSAL_SPI_BUFSIZE];
    uint8_t rx[DW1000_OSAL_SPI_BUFSIZE];
} _dw1000_spi_buffer;


/* Keep the first failure for the caller to find (see the error field):
 * asserting here would take the node down on a transient bus error,
 * and returning nothing leaves a zeroed read passing for register
 * content. spiExchange() reports a boolean, so the latched value only
 * says that something failed, not what. */
static inline
void _dw1000_spi_record(dw1000_spi_driver_t *spi, bool ok) {
    if (!ok && (spi->error == 0))
	spi->error = -1;
}

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    DW1000_ASSERT(hdrlen + datalen <= DW1000_OSAL_SPI_BUFSIZE,
		  "SPI transfer too large (long frames unsupported on cf2)");

    spiBeginTransaction(spi->speed);
    /* Chip select goes through the ioline helpers, not digitalWrite():
     * dw1000_ioline_t has two encodings and digitalWrite() only
     * understands the deck pin one, returning silently for anything
     * above DECK_MAX_PIN. Driving it directly left chip select
     * unasserted on a board whose CS is on a plain GPIO. */
    _dw1000_ioline_clear(spi->cs_pin);
    memcpy(_dw1000_spi_buffer.tx, hdr, hdrlen);
    memcpy(_dw1000_spi_buffer.tx+hdrlen, data, datalen);
    bool ok = spiExchange(hdrlen+datalen,
			  _dw1000_spi_buffer.tx, _dw1000_spi_buffer.rx);
    _dw1000_ioline_set(spi->cs_pin);
    spiEndTransaction();

    _dw1000_spi_record(spi, ok);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    DW1000_ASSERT(hdrlen + datalen <= DW1000_OSAL_SPI_BUFSIZE,
		  "SPI transfer too large (long frames unsupported on cf2)");

    spiBeginTransaction(spi->speed);
    _dw1000_ioline_clear(spi->cs_pin);
    memcpy(_dw1000_spi_buffer.tx, hdr, hdrlen);
    memset(_dw1000_spi_buffer.tx+hdrlen, 0, datalen);
    bool ok = spiExchange(hdrlen+datalen,
			  _dw1000_spi_buffer.tx, _dw1000_spi_buffer.rx);
    /* Read back while still inside the transaction: the bounce buffer is
     * only ours until spiEndTransaction(). */
    if (ok)
	memcpy(data, _dw1000_spi_buffer.rx+hdrlen, datalen);
    _dw1000_ioline_set(spi->cs_pin);
    spiEndTransaction();

    _dw1000_spi_record(spi, ok);
    /* The read helpers (_dw1000_reg_read8/16/32/64) hand us an
     * uninitialised stack local and return it whatever happens. Copying
     * the bounce buffer out after a failed exchange would hand back the
     * bytes of the *previous* register read: plausible looking and
     * stable across retries, which is worse than garbage. */
    if (!ok)
	memset(data, 0, datalen);
}
