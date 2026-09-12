/*
 * Copyright (c) 2018-2019
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_OSAL_H__
#define __DW1000_OSAL_H__

#include <ch.h>
#include <hal.h>

#include "sys/_iovec.h"


/*----------------------------------------------------------------------*/
/* Config                                                               */
/*----------------------------------------------------------------------*/


//#define DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH 




/*----------------------------------------------------------------------*/
/* Debug / Assert                                                       */
/*----------------------------------------------------------------------*/

#define DW1000_ASSERT(x, reason) osalDbgAssert(x, reason)



/*----------------------------------------------------------------------*/
/* Time                                                                 */
/*----------------------------------------------------------------------*/

static inline  void
_dw1000_delay_usec(uint16_t us) {
    osalThreadSleepMicroseconds(us);
}

static inline void
_dw1000_delay_msec(uint16_t ms) {
    osalThreadSleepMilliseconds(ms);
}



/*----------------------------------------------------------------------*/
/* IO line                                                              */
/*----------------------------------------------------------------------*/

typedef ioline_t dw1000_ioline_t;

#define DW1000_IOLINE_NONE PAL_NOLINE

static inline void
_dw1000_ioline_set(dw1000_ioline_t line) {
    palSetLine(line);
}

static inline void
_dw1000_ioline_clear(dw1000_ioline_t line) {
    palClearLine(line);
}



/*----------------------------------------------------------------------*/
/* SPI                                                                  */
/*----------------------------------------------------------------------*/

typedef struct dw1000_spi_driver {
    SPIDriver *drv;
    const SPIConfig *low_cfg;
    const SPIConfig *high_cfg;
    /* First failure of a transfer since the field was last cleared
     * (a ChibiOS msg_t, so MSG_TIMEOUT or MSG_RESET), 0 when all
     * transfers went through. The driver has no error path: a failed
     * read zeroes its buffer so it can never pass for data, so the
     * caller must look here to tell a failure from a register that
     * really did read 0. Cleared by whoever reports it.
     *
     * Only ever set when the SPI v2 API is in use: the v1 spiSend()
     * and spiReceive() return void and report nothing. The field is
     * carried in both cases so the interface stays the same as the
     * unix and zephyr ports. */
    int error;
} dw1000_spi_driver_t;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);
void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi);

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi);

#endif
