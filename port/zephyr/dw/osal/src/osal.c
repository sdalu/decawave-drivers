/*
 * Copyright (c) 2018-2020
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dw1000/osal.h"


void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    const struct spi_buf_set tx = {
        .buffers = (struct spi_buf []) { { .buf = hdr,  .len = hdrlen  },
					 { .buf = data, .len = datalen } },
	.count   = 2,
    };

    int rc __attribute__((unused)) =
	spi_write(spi->dev, spi->config, &tx);
    __ASSERT(rc >= 0, "spi write failed (%d)", rc);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    const struct spi_buf_set tx = {
        .buffers = (struct spi_buf []) { { .buf = hdr,  .len = hdrlen  },
					 { .buf = NULL, .len = datalen } },
	.count   = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = (struct spi_buf []) { { .buf = NULL, .len = hdrlen  },
					 { .buf = data, .len = datalen } },
	.count   = 2,
    };

    int rc __attribute__((unused)) =
	spi_transceive(spi->dev, spi->config, &tx, &rx);
    __ASSERT(rc >= 0, "spi transceive failed (%d)", rc);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    spi->config = spi->config_low_speed;
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    spi->config = spi->config_high_speed;
}
