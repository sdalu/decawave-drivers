/*
 * Copyright (c) 2018-2019,2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "dw1000/osal.h"


/*
 * ChibiOS has two SPI APIs. The low level driver selects between them
 * by defining HAL_LLD_SELECT_SPI_V2, and only the v2 one reports
 * anything: with SPI_USE_SYNCHRONIZATION enabled its spiSend() and
 * spiReceive() return a msg_t (MSG_OK, MSG_TIMEOUT or MSG_RESET). The
 * v1 functions return void, so there a failed transfer cannot be told
 * from a good one at all.
 *
 * Both are wrapped so the transfer code below reads the same either
 * way; on v1 the wrapper evaluates to MSG_OK, which is all "no error
 * was reported" can mean there.
 */
#if defined(HAL_LLD_SELECT_SPI_V2) &&					\
    defined(SPI_USE_SYNCHRONIZATION) && (SPI_USE_SYNCHRONIZATION == TRUE)
#define DW1000_SPI_SEND(drv, n, buf)     spiSend(drv, n, buf)
#define DW1000_SPI_RECEIVE(drv, n, buf)  spiReceive(drv, n, buf)
#define DW1000_SPI_START(drv, cfg)       spiStart(drv, cfg)
#else
#define DW1000_SPI_SEND(drv, n, buf)     (spiSend(drv, n, buf),    MSG_OK)
#define DW1000_SPI_RECEIVE(drv, n, buf)  (spiReceive(drv, n, buf), MSG_OK)
#define DW1000_SPI_START(drv, cfg)       (spiStart(drv, cfg),      MSG_OK)
#endif


/* Keep the first failure for the caller to find (see the error field):
 * asserting here would take the node down on a transient bus error,
 * and returning nothing leaves a zeroed read passing for register
 * content. */
static inline
void _dw1000_spi_record(dw1000_spi_driver_t *spi, msg_t msg) {
    if ((msg != MSG_OK) && (spi->error == 0))
	spi->error = (int)msg;
}


/*
 * A zero length transfer must not reach spiSend() or spiReceive():
 * both open with osalDbgCheck((spip != NULL) && (n > 0U) && ...), which
 * halts the system through chSysHalt() whenever CH_DBG_ENABLE_CHECKS is
 * set. Without the check the transfer is started anyway and the calling
 * thread waits on a completion interrupt that never arrives, with chip
 * select held low. The check is in the v1 and the v2 API alike.
 *
 * The header is never empty (_dw1000_spi_header() always builds at
 * least one byte) but the data segment is, for a register access with
 * no payload or an iovec entry of length 0.
 */

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    msg_t msg;

    spiSelect  (spi->drv);
    msg = DW1000_SPI_SEND(spi->drv, hdrlen,  hdr );  // Send register request
    if ((msg == MSG_OK) && (datalen > 0))
	msg = DW1000_SPI_SEND(spi->drv, datalen, data);  // Write data
    spiUnselect(spi->drv);

    _dw1000_spi_record(spi, msg);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    msg_t msg;

    spiSelect  (spi->drv);
    msg = DW1000_SPI_SEND(spi->drv, hdrlen,  hdr );  // Send register request
    if ((msg == MSG_OK) && (datalen > 0))
	msg = DW1000_SPI_RECEIVE(spi->drv, datalen, data);  // Read data
    spiUnselect(spi->drv);

    _dw1000_spi_record(spi, msg);
    /* The read helpers (_dw1000_reg_read8/16/32/64) hand us an
     * uninitialised stack local and return it whatever happens, so a
     * failed transfer must not leave it holding what the stack held:
     * zero it, so it can never pass for register content. */
    if (msg != MSG_OK)
	memset(data, 0, datalen);
}

/* spiStart() reports a configuration the driver rejected, again only on
 * the v2 API. A bus that would not start is worth the same latch as a
 * transfer that failed on it. */
void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    _dw1000_spi_record(spi, DW1000_SPI_START(spi->drv, spi->low_cfg));
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    _dw1000_spi_record(spi, DW1000_SPI_START(spi->drv, spi->high_cfg));
}
