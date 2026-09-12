/*
 * Copyright (c) 2018-2019
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dw1000/osal.h"


/*
 * A zero length transfer must not reach spiSend() or spiReceive(): both
 * open with osalDbgCheck((spip != NULL) && (n > 0U) && ...), which halts
 * the system through chSysHalt() whenever CH_DBG_ENABLE_CHECKS is set.
 * Without the check the transfer is started anyway and the calling
 * thread waits on a completion interrupt that never arrives, with chip
 * select held low. The check is in the v1 and the v2 SPI API alike.
 *
 * The header is never empty (_dw1000_spi_header() always builds at
 * least one byte) but the data segment is, for a register access with
 * no payload or an iovec entry of length 0.
 */

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    spiSelect  (spi->drv);
    spiSend    (spi->drv, hdrlen,  hdr );  // Send register request
    if (datalen > 0)
	spiSend(spi->drv, datalen, data);  // Write data
    spiUnselect(spi->drv);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    spiSelect  (spi->drv);
    spiSend    (spi->drv, hdrlen,  hdr );  // Send register request
    if (datalen > 0)
	spiReceive(spi->drv, datalen, data);  // Read data
    spiUnselect(spi->drv);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    spiStart(spi->drv, spi->low_cfg);
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    spiStart(spi->drv, spi->high_cfg);
}
