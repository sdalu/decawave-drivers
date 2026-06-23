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

static struct {
    uint8_t tx[DW1000_OSAL_SPI_BUFSIZE];
    uint8_t rx[DW1000_OSAL_SPI_BUFSIZE];
} _dw1000_spi_buffer;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    DW1000_ASSERT(hdrlen + datalen <= DW1000_OSAL_SPI_BUFSIZE,
		  "SPI transfer too large (long frames unsupported on cf2)");

    spiBeginTransaction(spi->speed);
    digitalWrite(spi->cs_pin, LOW);
    memcpy(_dw1000_spi_buffer.tx, hdr, hdrlen);
    memcpy(_dw1000_spi_buffer.tx+hdrlen, data, datalen);
    spiExchange(hdrlen+datalen, _dw1000_spi_buffer.tx, _dw1000_spi_buffer.rx);
    digitalWrite(spi->cs_pin, HIGH);
    spiEndTransaction();
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    DW1000_ASSERT(hdrlen + datalen <= DW1000_OSAL_SPI_BUFSIZE,
		  "SPI transfer too large (long frames unsupported on cf2)");

    spiBeginTransaction(spi->speed);
    digitalWrite(spi->cs_pin, LOW);
    memcpy(_dw1000_spi_buffer.tx, hdr, hdrlen);
    memset(_dw1000_spi_buffer.tx+hdrlen, 0, datalen);
    spiExchange(hdrlen+datalen, _dw1000_spi_buffer.tx, _dw1000_spi_buffer.rx);
    memcpy(data, _dw1000_spi_buffer.rx+hdrlen, datalen);
    digitalWrite(spi->cs_pin, HIGH);
    spiEndTransaction();
}
