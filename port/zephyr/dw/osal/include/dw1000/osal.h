/*
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 */

#ifndef __DW1000_OSAL_H__
#define __DW1000_OSAL_H__

#include <kernel.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include "sys/_iovec.h"


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
    k_busy_wait(us);
}

static inline void
_dw1000_delay_msec(uint16_t ms) {
    while (ms != 0) {
        ms = k_sleep(ms);
    }
}


/*----------------------------------------------------------------------*/
/* IO line                                                              */
/*----------------------------------------------------------------------*/

struct dw1000_ioline {
    struct device *gpio_dev;
    u32_t gpio_pin;
};
typedef struct dw1000_ioline *dw1000_ioline_t;

#define DW1000_IOLINE_NONE NULL

static inline void
_dw1000_ioline_set(dw1000_ioline_t line) {
    gpio_pin_write(line->gpio_dev, line->gpio_pin, 1);
}

static inline void
_dw1000_ioline_clear(dw1000_ioline_t line) {
    gpio_pin_write(line->gpio_dev, line->gpio_pin, 0);
}



/*----------------------------------------------------------------------*/
/* SPI                                                                  */
/*----------------------------------------------------------------------*/

typedef struct dw1000_spi_driver {
    struct device     *dev;
    struct spi_config *config_low_speed;
    struct spi_config *config_high_speed;
    struct spi_config *config;
} dw1000_spi_driver_t;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
              uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);
void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
              uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi);

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi);

#endif
