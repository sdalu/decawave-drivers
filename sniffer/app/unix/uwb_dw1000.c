/*
 * Copyright (c) 2019
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>

#include <bitters.h>
#include <bitters/rpi.h>
#include <bitters/gpio.h>
#include <bitters/spi.h>
#include <dw1000/dw1000.h>

#include "config.h"
#include "uwb.h"
#include "eth.h"


/*======================================================================*/
/* Forward declarations                                                 */
/*======================================================================*/

static void _rx_ok(dw1000_t *drv, uint32_t status, size_t length, bool ranging);




/*======================================================================*/
/* Local variables                                                      */
/*======================================================================*/

/* Callbacks
 */
static void (*uwb_cb_rx_ok)(
	    uint32_t status, size_t length, bool ranging) = NULL;

/* GPIO/SPI for DW1000
 */ 
static bitters_gpio_pin_t
dw1000_reset  = RPI_GPIO_PIN_INITIALIZER(DW1000_RESET );
static bitters_gpio_pin_t
dw1000_wakeup = RPI_GPIO_PIN_INITIALIZER(DW1000_WAKEUP);
static bitters_gpio_pin_t
dw1000_irq    = RPI_GPIO_PIN_INITIALIZER(DW1000_IRQ   );
static bitters_spi_t
dw1000_spi    = RPI_SPI_INITIALIZER(DW1000_SPI, 0);


/* DW1000 driver
 */
static dw1000_t DW0;


/* DW1000 SPI driver configuration (from the MCU point of view)
 * Initialised with baudrate, MSB first, MODE0 (CPOL=0/CPHA=0),
 * 8bit word.
 */
static struct bitters_spi_cfg dw1000_spi_cfg = {
    .mode       = BITTERS_SPI_MODE_0,
    .transfer   = BITTERS_SPI_TRANSFER_MSB,
    .word       = BITTERS_SPI_WORDSIZE(8),
    .speed      =  3000000,
};
static dw1000_spi_driver_t DW0_spi = {
    .dev        = &dw1000_spi,
    .config     = &dw1000_spi_cfg,
    .low_speed  =  3000000,
    .high_speed = 20000000,
};


/* DW1000 Configuration
 * (SPI, IRQ, Reset, callbacks, ....)
 * Remaining config (dw1000_ioline_reset, dw1000_ioline_irq) will
 * be performed at runtime due to device_get_binding()
 */
static dw1000_config_t DW0_config = {
    .spi              = &DW0_spi,
    .irq              = &dw1000_irq,
    .reset            = &dw1000_reset,
    .wakeup           = &dw1000_wakeup,
    .leds             = DW1000_LED_ALL,
    .leds_blink_time  = 3,
    .lde_loading      = 1,     // Loading of LDE microcode
    .rxauto           = 1,     // Automatically re-enable receiver
    .tx_antenna_delay = UWB_ANTENNA_DELAY_METER_TO_CLOCK(154.6)/2,
    .rx_antenna_delay = UWB_ANTENNA_DELAY_METER_TO_CLOCK(154.6)/2,
    .cb               = { .tx_done    = NULL,
			  .rx_timeout = NULL,
			  .rx_error   = NULL,
			  .rx_ok      = NULL, },
};


/*======================================================================*/
/* Local functions                                                      */
/*======================================================================*/

static void
_rx_ok(dw1000_t *drv, uint32_t status, size_t length, bool ranging)
{
    if (uwb_cb_rx_ok == NULL)
	return;

    uwb_cb_rx_ok(status, length, ranging);
}




/*======================================================================*/
/* Exported functions                                                   */
/*======================================================================*/

int
uwb_init(struct uwb_config *uwb_cfg)
{
    /* Some hint about raspberry pi configuration
     */
    printf("Don't forget to run at boot-time:"
	   " raspi-gpio set %d,%d pu\n", dw1000_reset.id, dw1000_wakeup.id);

    
    /*
     * Enable GPIO pins
     */
    struct bitters_gpio_cfg dw1000_reset_cfg  = {
        .dir       = BITTERS_GPIO_DIR_OUTPUT,
	.defval    = 1,
	.label     = "dw1000-reset",
    };
    struct bitters_gpio_cfg dw1000_wakeup_cfg = {
        .dir       = BITTERS_GPIO_DIR_OUTPUT,
	.defval    = 1,
	.label     = "dw1000-wakeup",
    };
    struct bitters_gpio_cfg dw1000_irq_cfg    = {
        .dir       = BITTERS_GPIO_DIR_INPUT,
	.interrupt = BITTERS_GPIO_INTERRUPT_RISING_EDGE,
	.label     = "dw1000-int",
    };
    
    if ((bitters_gpio_pin_enable(&dw1000_reset , &dw1000_reset_cfg ) < 0) ||
	(bitters_gpio_pin_enable(&dw1000_wakeup, &dw1000_wakeup_cfg) < 0) ||
	(bitters_gpio_pin_enable(&dw1000_irq   , &dw1000_irq_cfg   ) < 0)) {
	WARN_ERRNO("unable to configure gpio for dw1000");
	return -errno;
    }

    /*
     * Enable SPI
     */
    if (bitters_spi_enable(&dw1000_spi, &dw1000_spi_cfg) < 0) {
	WARN_ERRNO("unable to configure spi for dw1000");
	return -errno;
    }


    /* Initialisation and basic configuration
     */
    dw1000_t        *drv = &DW0;
    dw1000_config_t *cfg = &DW0_config;

    if (uwb_cfg) {
	if (uwb_cfg->frame_delivery) {
	    uwb_cb_rx_ok  = uwb_cfg->frame_delivery;
	    cfg->cb.rx_ok = _rx_ok;
	}

	if (uwb_cfg->antenna.tx_delay != UINT16_MAX) 
	    cfg->tx_antenna_delay = uwb_cfg->antenna.tx_delay;
	if (uwb_cfg->antenna.rx_delay != UINT16_MAX) 
	    cfg->rx_antenna_delay = uwb_cfg->antenna.rx_delay;
    }

    dw1000_init(drv, cfg);                             // DW creation
    dw1000_hardreset(drv);                             // Reset the DW1000 chip
    if (dw1000_initialise(drv) < 0)                    // Initialise device
	return -ENXIO;
    dw1000_leds_blink(drv, DW1000_LED_ALL);            // Blinking all leds

    return 0;
}



int
uwb_config_dw1000_radio(struct dw1000_radio *radio)
{
    if (! (((radio->prf == DW1000_PRF_64MHZ) &&
	    (radio->tx_pcode >= 9) && (radio->tx_pcode <= 24)) ||
	   ((radio->prf == DW1000_PRF_16MHZ) &&
	    (radio->tx_pcode >= 1) && (radio->tx_pcode <=  8))) )
	return -1;

    dw1000_t *drv = &DW0;
    dw1000_configure(drv,radio); 		       // Configure radio
    dw1000_rx_set_timeout(drv, 0);                     // No RX timeout

    return 0;
}



int
uwb_fill_pollfd(struct pollfd *pollfd)
{
    return bitters_gpio_irq_fill_pollfd(&dw1000_irq, pollfd);
}



int
uwb_process_events(void)
{
    dw1000_t *drv = &DW0;
    return dw1000_process_events(drv) ? 1 : 0;
}



int
uwb_rx_start(void)
{
    dw1000_t *drv = &DW0;
    return dw1000_rx_start(drv, 0);
}



int
uwb_wait_events(void)
{
    return bitters_gpio_irq_wait(&dw1000_irq);
}



int
uwb_read_frame_data(uint8_t *data, size_t length, size_t offset)
{
    dw1000_t *drv = &DW0;
    dw1000_rx_read_frame_data(drv, data, length, offset);
    return 0;
}



/* 
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
