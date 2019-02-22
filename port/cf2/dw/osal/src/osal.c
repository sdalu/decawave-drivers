/*
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 */

#include <string.h>

#include "dw1000/osal.h"


#define DW1000_OSAL_SPI_BUFSIZE 196

// XXX: WTF using a before and not 2 spiExchange ?!

static struct {
    uint8_t tx[DW1000_OSAL_SPI_BUFSIZE];
    uint8_t rx[DW1000_OSAL_SPI_BUFSIZE];
} _dw1000_spi_buffer;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
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
    spiBeginTransaction(spi->speed);
    digitalWrite(spi->cs_pin, LOW);
    memcpy(_dw1000_spi_buffer.tx, hdr, hdrlen);
    memset(_dw1000_spi_buffer.tx+hdrlen, 0, datalen);
    spiExchange(hdrlen+datalen, _dw1000_spi_buffer.tx, _dw1000_spi_buffer.rx);
    memcpy(data, _dw1000_spi_buffer.rx+hdrlen, datalen);
    digitalWrite(spi->cs_pin, HIGH);
    spiEndTransaction();
}
