/*
 * Copyright (c) 2018-2020
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dw1000/osal.h"
#include <string.h>


/* Keep the first failure for the caller to find (see the error field):
 * asserting here would take the node down on a transient bus error,
 * and returning nothing leaves a zeroed read passing for register
 * content. */
static inline
void _dw1000_spi_record(dw1000_spi_driver_t *spi, int rc) {
    if (rc < 0 && spi->error == 0)
        spi->error = rc;
}

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    const struct spi_buf_set tx = {
        .buffers = (struct spi_buf []) { { .buf = hdr,  .len = hdrlen  },
					 { .buf = data, .len = datalen } },
	.count   = 2,
    };

    _dw1000_spi_record(spi, spi_write(spi->dev, spi->config, &tx));
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

    int rc = spi_transceive(spi->dev, spi->config, &tx, &rx);
    _dw1000_spi_record(spi, rc);
    /* The read helpers (_dw1000_reg_read8/16/32/64) hand us an
     * uninitialised stack local and return it whatever happens, so a
     * failed transfer must not leave it holding what the stack held:
     * zero it, so it can never pass for register content. */
    if (rc < 0)
        memset(data, 0, datalen);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    spi->config = spi->config_low_speed;
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    spi->config = spi->config_high_speed;
}
