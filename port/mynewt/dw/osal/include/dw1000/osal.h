/*
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 */

#ifndef __DW1000_OSAL_H__
#define __DW1000_OSAL_H__

#include <assert.h>
#include <syscfg/syscfg.h>
#include <os/os.h>
#include <hal/hal_gpio.h>
#include <hal/hal_spi.h>
#include "sys/_iovec.h"


/*----------------------------------------------------------------------*/
/* Endianess                                                            */
/*----------------------------------------------------------------------*/

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define DW1000_ARCH_LITTLE_ENDIAN
#endif

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define DW1000_ARCH_BIG_ENDIAN
#endif



/*----------------------------------------------------------------------*/
/* Config                                                               */
/*----------------------------------------------------------------------*/


#define DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH		\
    MYNEWT_VAL(DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH)



/*----------------------------------------------------------------------*/
/* Debug / Assert                                                       */
/*----------------------------------------------------------------------*/

#define DW1000_ASSERT(x, reason) assert(x)



/*----------------------------------------------------------------------*/
/* Time                                                                 */
/*----------------------------------------------------------------------*/

static inline  void
_dw1000_delay_usec(uint16_t us) {
    os_cputime_delay_usecs(us);
}

static inline void
_dw1000_delay_msec(uint16_t ms) {
    os_time_delay(((ms) * OS_TICKS_PER_SEC + 999) / 1000);
}



/*----------------------------------------------------------------------*/
/* IO line                                                              */
/*----------------------------------------------------------------------*/

typedef int dw1000_ioline_t;

#define DW1000_IOLINE_NONE -1

static inline void
_dw1000_ioline_set(dw1000_ioline_t line) {
    hal_gpio_write(line, 1);
}

static inline void
_dw1000_ioline_clear(dw1000_ioline_t line) {
    hal_gpio_write(line, 0);
}



/*----------------------------------------------------------------------*/
/* SPI                                                                  */
/*----------------------------------------------------------------------*/

typedef struct dw1000_spi_driver {
    int              id;     // SPI device id
    dw1000_ioline_t  cs_pin; // Chip Select Pin
    void (*lock  )(struct dw1000_spi_driver *spi);
    void (*unlock)(struct dw1000_spi_driver *spi);
    struct hal_spi_settings settings;
} dw1000_spi_driver_t;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);
void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi);

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi);

#endif
