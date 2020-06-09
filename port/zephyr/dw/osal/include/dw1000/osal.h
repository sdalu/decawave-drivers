/*
 * Copyright (c) 2018-2020
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_OSAL_H__
#define __DW1000_OSAL_H__

#include <kernel.h>
#include <version.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include "sys/_iovec.h"


/*----------------------------------------------------------------------*/
/* Config                                                               */
/*----------------------------------------------------------------------*/

#if defined(CONFIG_DW1000_PROPRIETARY_PREAMBLE_LENGTH)
#define DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH		1
#else
#define DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH		0
#endif

#if defined(CONFIG_DW1000_PROPRIETARY_SFD)
#define DW1000_WITH_PROPRIETARY_SFD			1
#else
#define DW1000_WITH_PROPRIETARY_SFD			0
#endif

#if defined(CONFIG_DW1000_PROPRIETARY_LONG_FRAME)
#define DW1000_WITH_PROPRIETARY_LONG_FRAME		1
#else
#define DW1000_WITH_PROPRIETARY_LONG_FRAME		0
#endif

#if defined(CONFIG_DW1000_SFD_TIMEOUT)
#define DW1000_WITH_SFD_TIMEOUT				1
#else
#define DW1000_WITH_SFD_TIMEOUT				0
#endif

#if CONFIG_DW1000_SFD_TIMEOUT_DEFAULT > 0
#define DW1000_WITH_SFD_TIMEOUT_DEFAULT			1
#define DW1000_SFD_TIMEOUT_DEFAULT	CONFIG_DW1000_SFD_TIMEOUT_DEFAULT
#else
#define DW1000_WITH_SFD_TIMEOUT_DEFAULT			0
#endif

#if defined(CONFIG_DW1000_HOTFIX_AAT_IEEE802_15_4_2011)
#define DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011	1
#else
#define DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011	0
#endif



/*----------------------------------------------------------------------*/
/* Debug / Assert                                                       */
/*----------------------------------------------------------------------*/

#define DW1000_ASSERT(x, reason) __ASSERT(x, reason)


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
#if KERNEL_VERSION_NUMBER < 0x020300
        ms = k_sleep(ms);
#else
	ms = k_msleep(ms);
#endif
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
#if KERNEL_VERSION_NUMBER < 0x020200
    gpio_pin_write(line->gpio_dev, line->gpio_pin, 1);
#else
    gpio_pin_set_raw(line->gpio_dev, line->gpio_pin, 1);
#endif
}

static inline void
_dw1000_ioline_clear(dw1000_ioline_t line) {
#if KERNEL_VERSION_NUMBER < 0x020200
    gpio_pin_write(line->gpio_dev, line->gpio_pin, 0);
#else
    gpio_pin_set_raw(line->gpio_dev, line->gpio_pin, 0);
#endif
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
