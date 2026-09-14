/*
 * Copyright (c) 2024-2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The emulation OSAL: the port contract, wired to a register model.
 *
 * No chip and no bus. Every SPI transfer lands in the register model of
 * src/osal.c, and the air interface is a unix socket to a medium server
 * (see include/dw1000/emulation.h for the wire packet). The driver runs
 * whole, unmodified, against it.
 *
 * The delay functions block signals while they sleep: a node driving this
 * port runs its own timers on SIGALRM, and a usleep() cut short by one
 * would shorten a delay the driver asked for.
 */

#ifndef __DW1000_OSAL_H__
#define __DW1000_OSAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <errno.h>
#include <sys/uio.h>
#include <signal.h>

#include "dw1000/dw1000_reg.h"
#include "rsvc.h"


typedef struct dw1000 dw1000_t;


/* The register model behind the lines and the bus. Its constructor and
 * the wire packet it speaks live in dw1000/emulation.h, which is what a
 * medium server includes; a consumer of the port only needs the handle.
 */
struct dw1000_emulation;


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

static inline  void
_dw1000_delay_usec(uint16_t us) {
    sigset_t mask, oldmask;
    sigfillset(&mask);
    pthread_sigmask(SIG_SETMASK, &mask, &oldmask);
    usleep(us);
    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);
}

static inline void
_dw1000_delay_msec(uint16_t ms) {
    sigset_t mask, oldmask;
    sigfillset(&mask);
    pthread_sigmask(SIG_SETMASK, &mask, &oldmask);
    usleep(ms * 1000);
    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);
}


/*----------------------------------------------------------------------*/
/* IO line                                                              */
/*----------------------------------------------------------------------*/

typedef struct dw1000_ioline {
    struct dw1000_emulation *emulation;
    int line;
    struct {
	void (*cb)(void *data);
	void *data;
    } irq;
}  *dw1000_ioline_t;


#define DW1000_IOLINE_NONE	0
#define DW1000_IOLINE_IRQ	1
#define DW1000_IOLINE_RESET	2
#define DW1000_IOLINE_WAKEUP	3


void _dw1000_ioline_set(dw1000_ioline_t line);
void _dw1000_ioline_clear(dw1000_ioline_t line);



/*----------------------------------------------------------------------*/
/* SPI                                                                  */
/*----------------------------------------------------------------------*/

typedef struct dw1000_spi_driver {
    struct dw1000_emulation *emulation;
} dw1000_spi_driver_t;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
              uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);
void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
              uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi);

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi);

#endif
