/*
 * Copyright (c) 2018-2019,2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_OSAL_H__
#define __DW1000_OSAL_H__

#include "FreeRTOS.h"
#include "task.h"
#include "cfassert.h"
#include "sleepus.h"
#include "deck_constants.h"
#include "deck_digital.h"
#include "deck_spi.h"
#include "sys/_iovec.h"


/*----------------------------------------------------------------------*/
/* Debug / Assert                                                       */
/*----------------------------------------------------------------------*/

#define DW1000_ASSERT(x, reason)  do			\
	if (!(x)) { __asm__ __volatile__ ("bkpt #1"); } \
    while(0)



/*----------------------------------------------------------------------*/
/* Time (using CF sleepus.c and FreeRTOS)                               */
/*----------------------------------------------------------------------*/

static inline
void _dw1000_delay_usec(uint16_t us) {
    sleepus(us);
}

static inline
void _dw1000_delay_msec(uint16_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}



/*----------------------------------------------------------------------*/
/* IO line (using CF deck_digital.c wrapper)                            */
/*----------------------------------------------------------------------*/

#define DECK_MAX_PIN 13

typedef uint32_t dw1000_ioline_t;

#define DW1000_IOLINE_NONE 0

static inline void
_dw1000_ioline_set(dw1000_ioline_t line) {
    if (line > 0xFFFF) {
	GPIO_TypeDef *port;
	uint16_t      pin  = line & 0xFFFF;
	switch(line >> 16) {
	case 3: port = GPIOC; break;
	default:
	    /* No mapping for this port index, so there is no pin to drive.
	     * Returning leaves the line alone; falling through would call
	     * GPIO_WriteBit() with an uninitialised pointer, and
	     * DW1000_ASSERT is a bkpt that does not stop a build running
	     * without a debugger attached. */
	    DW1000_ASSERT(0, "unhandled GPIO port");
	    return;
	}
	GPIO_WriteBit(port, pin, 1);
    } else {
	digitalWrite(line, HIGH);
    }
}

static inline void
_dw1000_ioline_clear(dw1000_ioline_t line) {
    if (line > 0xFFFF) {
	GPIO_TypeDef *port;
	uint16_t      pin  = line & 0xFFFF;
	switch(line >> 16) {
	case 3: port = GPIOC; break;
	default:
	    /* No mapping for this port index, so there is no pin to drive.
	     * Returning leaves the line alone; falling through would call
	     * GPIO_WriteBit() with an uninitialised pointer, and
	     * DW1000_ASSERT is a bkpt that does not stop a build running
	     * without a debugger attached. */
	    DW1000_ASSERT(0, "unhandled GPIO port");
	    return;
	}
	GPIO_WriteBit(port, pin, 0);
    } else {
	digitalWrite(line, LOW);
    }
}



/*----------------------------------------------------------------------*/
/* SPI (using CF deck_spi.c wrapper)                                    */
/*----------------------------------------------------------------------*/

typedef struct dw1000_spi_driver {
    dw1000_ioline_t cs_pin;
    uint16_t speed;
    /* First failure of a transfer since the field was last cleared
     * (-1; spiExchange() reports only a boolean), 0 when all transfers
     * went through. The driver has no error path: a failed read zeroes
     * its buffer so it can never pass for data, so the caller must look
     * here to tell a failure from a register that really did read 0.
     * Cleared by whoever reports it. */
    int error;
} dw1000_spi_driver_t;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);
void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);

static inline
void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    spi->speed = SPI_BAUDRATE_2MHZ;
}

static inline
void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    spi->speed = SPI_BAUDRATE_21MHZ;
}

#endif
