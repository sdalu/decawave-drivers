/*
 * Copyright (c) 2018-2019
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dw1000/osal.h"


void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    const struct bitters_spi_transfert xfr[] = {
        { .tx = hdr,  .len = hdrlen  },
	{ .tx = data, .len = datalen }
    };

    int rc = bitters_spi_transfert(spi->dev, xfr, 2);
    assert(rc >= 0);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    const struct bitters_spi_transfert xfr[] = {
        { .tx = hdr,  .len = hdrlen  },
	{ .rx = data, .len = datalen }
    };

    int rc = bitters_spi_transfert(spi->dev, xfr, 2);
    assert(rc >= 0);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    bitters_spi_set_speed(spi->dev, spi->low_speed);
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    bitters_spi_set_speed(spi->dev, spi->high_speed);
}
