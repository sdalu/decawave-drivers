/*
 * Copyright (c) 2018-2019,2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "dw1000/osal.h"

#include "hal/hal_spi.h"


/* Keep the first failure for the caller to find (see the error field):
 * asserting here would take the node down on a transient bus error,
 * and returning nothing leaves a zeroed read passing for register
 * content. */
static inline
void _dw1000_spi_record(dw1000_spi_driver_t *spi, int rc) {
    if ((rc != 0) && (spi->error == 0))
	spi->error = rc;
}

/*
 * Transfers go through hal_spi_txrx(), which moves a whole buffer and
 * returns 0 or a non-zero error code, rather than hal_spi_tx_val() one
 * value at a time. hal_spi_tx_val() returns 0xFFFF on error, which is
 * indistinguishable from real data clocked in, so the byte at a time
 * form cannot report a failure at all.
 *
 * Two constraints from hal_spi.h: txbuf may not be NULL (rxbuf may), and
 * a count of 0 is rejected with an error, so an empty data segment must
 * not be handed over.
 */

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    int rc;

    if (spi->lock)
	spi->lock(spi);

    _dw1000_ioline_clear(spi->cs_pin);

    rc = hal_spi_txrx(spi->id, hdr, NULL, (int)hdrlen);
    if ((rc == 0) && (datalen > 0))
	rc = hal_spi_txrx(spi->id, data, NULL, (int)datalen);

    _dw1000_ioline_set(spi->cs_pin);

    if (spi->unlock)
	spi->unlock(spi);

    _dw1000_spi_record(spi, rc);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    int rc;

    if (spi->lock)
	spi->lock(spi);

    _dw1000_ioline_clear(spi->cs_pin);

    rc = hal_spi_txrx(spi->id, hdr, NULL, (int)hdrlen);
    if ((rc == 0) && (datalen > 0)) {
	/* txbuf may not be NULL, and UM §2.2: "for a read transaction all
	 * octets beyond the transaction header are ignored". So whatever
	 * is clocked out here does not reach the DW1000, and the caller's
	 * buffer can drive MOSI as well as collect MISO, saving a dummy
	 * buffer of its own. Zero it first so what goes out is zeros, as
	 * the previous hal_spi_tx_val(spi->id, 0) loop sent. */
	memset(data, 0, datalen);
	rc = hal_spi_txrx(spi->id, data, data, (int)datalen);
    }

    _dw1000_ioline_set(spi->cs_pin);

    if (spi->unlock)
	spi->unlock(spi);

    _dw1000_spi_record(spi, rc);
    /* The read helpers (_dw1000_reg_read8/16/32/64) hand us an
     * uninitialised stack local and return it whatever happens, so a
     * failed transfer must not leave it holding what the stack held. */
    if (rc != 0)
	memset(data, 0, datalen);
}


/* hal_spi_disable()/config()/enable() each report a failure, and those
 * used to be checked only by DW1000_ASSERT, which four of the five ports
 * compile out. Latch them instead, so a bus that would not reconfigure
 * is visible in a release build too. */
static void _dw1000_spi_set_speed(dw1000_spi_driver_t *spi, uint32_t baudrate) {
    int rc;

    if (spi->lock)
	spi->lock(spi);

    spi->settings.baudrate = baudrate;

    rc = hal_spi_disable(spi->id);
    if (rc == 0)
	rc = hal_spi_config(spi->id, &spi->settings);
    if (rc == 0)
	rc = hal_spi_enable(spi->id);
    _dw1000_spi_record(spi, rc);

    if (spi->unlock)
	spi->unlock(spi);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    _dw1000_spi_set_speed(spi, MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_LOW));
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    _dw1000_spi_set_speed(spi, MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_HIGH));
}
