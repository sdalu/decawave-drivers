/*
 * Copyright (c) 2018-2019
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dw1000/osal.h"
#include <string.h>

static inline
void _dw1000_spi_record(dw1000_spi_driver_t *spi, int rc) {
    if (rc < 0 && spi->error == 0)
        spi->error = rc;
}

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    const struct bitters_spi_transfer xfr[] = {
        { .tx = hdr,  .len = hdrlen  },
	{ .tx = data, .len = datalen }
    };

    int rc = bitters_spi_transfer(spi->dev, xfr, 2);
    _dw1000_spi_record(spi, rc);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    const struct bitters_spi_transfer xfr[] = {
        { .tx = hdr,  .len = hdrlen  },
	{ .rx = data, .len = datalen }
    };

    int rc = bitters_spi_transfer(spi->dev, xfr, 2);
    _dw1000_spi_record(spi, rc);
    if (rc < 0)
        memset(data, 0, datalen);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    bitters_spi_set_speed(spi->dev, spi->low_speed);
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    bitters_spi_set_speed(spi->dev, spi->high_speed);
}
