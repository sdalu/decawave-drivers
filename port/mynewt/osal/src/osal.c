#include "dw1000/osal.h"

#include "hal/hal_spi.h"

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    _dw1000_ioline_clear(spi->cs_pin);
    
    for(uint8_t i = 0; i < hdrlen; i++)
        hal_spi_tx_val(spi->id, hdr[i]);
    for(uint16_t i = 0; i < datalen; i++)
        hal_spi_tx_val(spi->id, data[i]);

    _dw1000_ioline_set(spi->cs_pin);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    _dw1000_ioline_clear(spi->cs_pin);

    for(uint8_t i = 0; i < hdrlen; i++)
        hal_spi_tx_val(inst->id, hdr[i]);
    for(uint16_t i = 0; i < length; i++)
        data[i] = hal_spi_tx_val(inst->id, 0);

    _dw1000_ioline_set(spi->cs_pin);
}
