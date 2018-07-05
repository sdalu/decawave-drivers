#ifndef __DW1000_OSAL_H__
#define __DW1000_OSAL_H__

#include <ch.h>
#include <hal.h>

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
} dw1000_spi_driver_t;

void _dw1000_spi_send(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);
void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
	      uint8_t *hdr, size_t hdrlen, uint8_t *data, size_t datalen);

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi);

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi);

#endif
