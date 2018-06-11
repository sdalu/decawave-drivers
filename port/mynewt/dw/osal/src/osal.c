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
        hal_spi_tx_val(spi->id, hdr[i]);
    for(uint16_t i = 0; i < datalen ; i++)
        data[i] = hal_spi_tx_val(spi->id, 0);

    _dw1000_ioline_set(spi->cs_pin);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    int rc;

    spi->settings.baudrate = MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_LOW);

    rc = hal_spi_disable(spi->id);
    DW1000_ASSERT(rc == 0, "failed to disable SPI");
    rc = hal_spi_config(spi->id, &spi->settings);
    DW1000_ASSERT(rc == 0, "failed to configure SPI");
    rc = hal_spi_enable(spi->id);
    DW1000_ASSERT(rc == 0, "failed to enable SPI");

}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    int rc;

    spi->settings.baudrate = MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_HIGH);

    rc = hal_spi_disable(spi->id);
    DW1000_ASSERT(rc == 0, "failed to disable SPI");
    rc = hal_spi_config(spi->id, &spi->settings);
    DW1000_ASSERT(rc == 0, "failed to configure SPI");
    rc = hal_spi_enable(spi->id);
    DW1000_ASSERT(rc == 0, "failed to enable SPI");
}

