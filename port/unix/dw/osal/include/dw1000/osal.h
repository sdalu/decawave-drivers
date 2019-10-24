/*
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
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

#include <bitters/gpio.h>
#include <bitters/spi.h>
#include <bitters/delay.h>


/*----------------------------------------------------------------------*/
/* Config                                                               */
/*----------------------------------------------------------------------*/

#if defined(CONFIG_DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH)
#define DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH		1
#else
#define DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH		0
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
    bitters_delay_usec(us);
}

static inline void
_dw1000_delay_msec(uint16_t ms) {
    bitters_delay_msec(ms);
}


/*----------------------------------------------------------------------*/
/* IO line                                                              */
/*----------------------------------------------------------------------*/

typedef bitters_gpio_pin_t *dw1000_ioline_t;

#define DW1000_IOLINE_NONE BITTERS_GPIO_PIN_NONE

static inline void
_dw1000_ioline_set(dw1000_ioline_t line) {
    bitters_gpio_pin_write(line, 1);
}

static inline void
_dw1000_ioline_clear(dw1000_ioline_t line) {
    bitters_gpio_pin_write(line, 0);
}



/*----------------------------------------------------------------------*/
/* SPI                                                                  */
/*----------------------------------------------------------------------*/

typedef struct dw1000_spi_driver {
    struct bitters_spi      *dev;
    struct bitters_spi_cfg  *config;
    uint32_t      	     high_speed;
    uint32_t	             low_speed;
} dw1000_spi_driver_t;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
              uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);
void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
              uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi);

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi);

#endif
