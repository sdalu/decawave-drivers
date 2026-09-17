/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The null OSAL: the whole port contract, wired to nothing.
 *
 * Not a host. It exists so the core can be compiled where there is no
 * DW1000 and no RTOS, which is what `make check` does, over every
 * combination of the compile-time options, so a combination that stopped
 * compiling is found here rather than by whoever selects it. Every other
 * port needs its vendor headers to say that much, so none of them can.
 *
 * It is also the shortest thing to copy when writing a new port: the
 * symbols below are the entire contract, and the SPI transfers observe
 * the two rules the real ports learned the hard way: a failed read
 * zeroes the caller's buffer, and the first failure is latched in
 * spi->error and left for the caller to clear.
 *
 * Linked into a program it builds a driver that finds no device:
 * every transfer fails with -ENODEV, so dw1000_initialise() returns -1
 * on the device ID and nothing proceeds on a radio that is not there.
 */

#ifndef __DW1000_OSAL_H__
#define __DW1000_OSAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <sys/uio.h>


/*----------------------------------------------------------------------*/
/* Config                                                               */
/*----------------------------------------------------------------------*/

#if  defined(DW1000_SFD_TIMEOUT_DEFAULT) &&	\
    !defined(DW1000_WITH_SFD_TIMEOUT_DEFAULT)
#if DW1000_SFD_TIMEOUT_DEFAULT > 0
#define DW1000_WITH_SFD_TIMEOUT_DEFAULT 1
#else
#define DW1000_WITH_SFD_TIMEOUT_DEFAULT 0
#endif
#endif


/*----------------------------------------------------------------------*/
/* Debug / Assert                                                       */
/*----------------------------------------------------------------------*/

#define DW1000_ASSERT(x, reason) assert(x)


/*----------------------------------------------------------------------*/
/* Time                                                                 */
/*----------------------------------------------------------------------*/

/* Nothing to wait for. Returning at once is honest here in a way it
 * would not be in a port that drives a chip.
 */
static inline void
_dw1000_delay_usec(uint16_t us) {
    (void)us;
}

static inline void
_dw1000_delay_msec(uint16_t ms) {
    (void)ms;
}


/*----------------------------------------------------------------------*/
/* IO line                                                              */
/*----------------------------------------------------------------------*/

typedef int dw1000_ioline_t;

#define DW1000_IOLINE_NONE -1

static inline void
_dw1000_ioline_set(dw1000_ioline_t line) {
    (void)line;
}

static inline void
_dw1000_ioline_clear(dw1000_ioline_t line) {
    (void)line;
}


/*----------------------------------------------------------------------*/
/* SPI                                                                  */
/*----------------------------------------------------------------------*/

/* A real port carries its bus handle here; there is none to carry, so
 * only the error field the contract requires is left.
 */
typedef struct dw1000_spi_driver {
    int  error;
} dw1000_spi_driver_t;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);
void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi);

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi);

#endif
