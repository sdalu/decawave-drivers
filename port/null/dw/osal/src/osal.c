/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dw1000/osal.h"
#include <string.h>

/* Latch the first failure only: the core has no error path of its own to
 * carry it, so it is the caller who reads spi->error and clears it, and a
 * later transfer must not overwrite what it has not seen yet.
 */
static inline void
_dw1000_spi_record(dw1000_spi_driver_t *spi, int rc) {
    if (rc < 0 && spi->error == 0)
        spi->error = rc;
}

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    (void)hdr; (void)hdrlen; (void)data; (void)datalen;

    _dw1000_spi_record(spi, -ENODEV);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    (void)hdr; (void)hdrlen;

    _dw1000_spi_record(spi, -ENODEV);

    /* The core's register helpers return an uninitialised local whatever
     * happens, so a buffer left untouched is read back as register
     * content. Zero it.
     */
    memset(data, 0, datalen);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    (void)spi;
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    (void)spi;
}
