#include "dw1000/osal.h"

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    spiSelect  (spi->drv);
    spiSend    (spi->drv, hdrlen,  hdr );  // Send register request 
    spiSend    (spi->drv, datalen, data);  // Write data
    spiUnselect(spi->drv);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    spiSelect  (spi->drv);
    spiSend    (spi->drv, hdrlen,  hdr );  // Send register request 
    spiReceive (spi->drv, datalen, data);  // Read data
    spiUnselect(spi->drv);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    spiStart(spi->drv, spi->low_cfg);
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    spiStart(spi->drv, spi->high_cfg);
}
