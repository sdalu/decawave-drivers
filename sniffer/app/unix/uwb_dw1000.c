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
#include <math.h>
#include <string.h>

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
static void _rx_error(dw1000_t *drv, uint32_t status);
static void _read_frame_data(uint8_t *data, size_t length, size_t offset);
static void _read_meta(struct capture_meta *meta);




/*======================================================================*/
/* Local variables                                                      */
/*======================================================================*/

/* Callbacks
 */
static void (*uwb_cb_rx_ok)(
	    uint32_t status, size_t length, bool ranging) = NULL;

/* The receive error account, kept here because this is where the status
 * word arrives and nowhere else sees it.
 */
static struct uwb_rx_errors rx_errors;

/* The chip side of capture.c's read-out seam, in both its forms. Which
 * one uwb_capture_ops() hands back is the whole of what `--metadata`
 * and `--no-metadata` select: with read_meta NULL the callback spends no
 * SPI time on registers nobody asked for, and capture.c flags the frames
 * accordingly rather than filling a block with zeroes.
 */
static const struct capture_ops uwb_ops_plain = {
    .read_frame_data = _read_frame_data,
    .read_meta       = NULL,
};
static const struct capture_ops uwb_ops_meta = {
    .read_frame_data = _read_frame_data,
    .read_meta       = _read_meta,
};

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
    /* Double receive buffer (UM 4.3.3): the driver re-enables the
     * receiver in the good-frame path *before* calling rx_ok, so the next
     * frame lands in the other buffer while this one is read out, and it
     * toggles the host side buffer pointer as soon as rx_ok returns. Two
     * obligations follow, both met by _rx_ok() below and by capture.c:
     * the callback must not re-enable the receiver itself, and it must
     * read the frame out before returning: RX_BUFFER swings with the
     * pointer. rxauto stays on beside it, as probe and rpi-redskin both
     * have it. See hw/drivers/dw1000/README.md, "Double buffered
     * receive".
     */
    .dblbuff          = 1,	   // uwb_init() may clear it
    .tx_antenna_delay = DW1000_METER_TO_CLOCK(154.6)/2,
    .rx_antenna_delay = DW1000_METER_TO_CLOCK(154.6)/2,
    /* rx_error is not optional here: dw1000_initialise() refuses dblbuff
     * without one and returns -1, because the overrun recovery re-arms
     * nothing by itself and the receiver would stop for good on the first
     * overrun. rx_timeout stays NULL: uwb_config_dw1000_radio() sets
     * the receive timeout to 0, so no timeout is ever reported.
     */
    .cb               = { .tx_done    = NULL,
			  .rx_timeout = NULL,
			  .rx_error   = _rx_error,
			  .rx_ok      = NULL, },
};


/*======================================================================*/
/* Local functions                                                      */
/*======================================================================*/

static void
_rx_ok(dw1000_t *drv, uint32_t status, size_t length, bool ranging)
{
    (void)drv;

    if (uwb_cb_rx_ok == NULL)
	return;

    uwb_cb_rx_ok(status, length, ranging);
}


/* Every receive error, the overrun included, arrives here, and re-arming
 * is the host's job for all of them: the driver's overrun recovery puts
 * the chip back in order and reports, but enables nothing.
 *
 * The response is the same whatever the reason, which is why this used to
 * discard the status word. Counting it is not about the response but
 * about the report: an overrun (RXOVRR) means frames were received and
 * then lost on the chip because both buffers were held, which is the
 * host being too slow and is the number that says whether the ring and
 * the read-out are keeping up; the rest mean a frame never decoded,
 * which is the link. A sniffer that cannot tell the user which of those
 * happened is asking to be trusted about the frames it did not report.
 */
static void
_rx_error(dw1000_t *drv, uint32_t status)
{
    rx_errors.total++;

    /* Not mutually exclusive, and not counted as if they were: one
     * recovery can carry several of these bits at once, and each is a
     * separate thing that happened. So the counts sum to more than
     * total, deliberately.
     */
    if (status & DW1000_FLG_SYS_STATUS_RXOVRR ) rx_errors.overrun++;
    if (status & DW1000_FLG_SYS_STATUS_RXPHE  ) rx_errors.phy++;
    if (status & DW1000_FLG_SYS_STATUS_RXFCE  ) rx_errors.fcs++;
    if (status & DW1000_FLG_SYS_STATUS_RXRFSL ) rx_errors.sync++;
    if (status & DW1000_FLG_SYS_STATUS_LDEERR ) rx_errors.lde++;
    if (status & DW1000_FLG_SYS_STATUS_RXSFDTO) rx_errors.sfd_timeout++;
    if (status & DW1000_FLG_SYS_STATUS_AFFREJ ) rx_errors.rejected++;

    /* A call with none of the bits above is worth its own count rather
     * than being dropped: it means the driver reported an error this
     * program does not know how to name, and a run where that number is
     * not zero is a run to look at the driver for.
     */
    if (!(status & (DW1000_MSK_SYS_STATUS_ALL_RX_ERR |
		    DW1000_FLG_SYS_STATUS_RXOVRR)))
	rx_errors.unexplained++;

    dw1000_rx_start(drv, DW1000_RX_IMMEDIATE);
}


/* The two read-outs capture.c reaches the chip through. Both MUST run
 * inside the rx_ok callback: RX_BUFFER, RX_TIME, RX_FQUAL, RX_TTCKI and
 * RX_TTCKO all swing with the double buffer pointer, which the driver
 * toggles the moment the callback returns (hw/drivers/dw1000/README.md,
 * "Double buffered receive"). capture.h says the same thing from the
 * other side; this is the end that actually touches the bus.
 */
static void
_read_frame_data(uint8_t *data, size_t length, size_t offset)
{
    dw1000_rx_read_frame_data(&DW0, data, length, offset);
}


/* dBm as the driver reports it, into the milli-dBm capture.h carries.
 *
 * The driver gives -INFINITY when the preamble accumulation count is 0,
 * which leaves no usable estimate, and that has to stay distinguishable
 * from a reading: CAPTURE_POWER_NONE is what says so, and a genuine
 * value is clamped away from it rather than allowed to collide with it.
 * Received powers are in the -100..0 dBm region, so the clamp never
 * fires in practice; it is there because a sentinel a real reading can
 * reach is not a sentinel.
 */
static int32_t
_milli_dbm(double dbm)
{
    if (!isfinite(dbm))
	return CAPTURE_POWER_NONE;

    double milli = dbm * 1000.0;
    if (milli >= (double)INT32_MAX)         return INT32_MAX;
    if (milli <= (double)(CAPTURE_POWER_NONE + 1)) return CAPTURE_POWER_NONE + 1;
    return (int32_t)lround(milli);
}


static void
_read_meta(struct capture_meta *meta)
{
    dw1000_t       *drv = &DW0;
    dw1000_rxinfo_t rxinfo;
    double          signal, firstpath;
    int32_t         offset;
    uint32_t        interval;

    meta->rx_time = dw1000_rx_get_rmarker_time(drv);

    /* Reported raw, not through dw1000_rx_power_correction(). The
     * correction is a curve out of UM §4.7 fig 22 mapping the estimate
     * onto the actual level, and applying it here would leave a consumer
     * unable to get back to what the chip said. A sniffer's job is to
     * report; the curve is the consumer's to apply.
     */
    dw1000_rx_get_power_estimate(drv, &signal, &firstpath);
    meta->power_signal    = _milli_dbm(signal);
    meta->power_firstpath = _milli_dbm(firstpath);

    /* Offset and interval rather than the ratio: the ratio is what
     * dw1000_rx_get_clock_drift() returns, and it is a double whose
     * zero means both "no drift" and "no reading yet" (RX_TTCKI reads 0
     * before the first frame is demodulated). Carrying both numbers
     * keeps those apart, and dividing is the consumer's business.
     */
    dw1000_rx_get_time_tracking(drv, &offset, &interval);
    meta->clock_offset   = offset;
    meta->clock_interval = interval;

    dw1000_rx_get_info(drv, &rxinfo);
    meta->first_path = rxinfo.first_path;
    meta->std_noise  = rxinfo.std_noise;
    meta->max_noise  = rxinfo.max_noise;
}




/*======================================================================*/
/* Exported functions                                                   */
/*======================================================================*/

int
uwb_init(struct uwb_config *uwb_cfg)
{
    /* Some hint about raspberry pi configuration.
     *
     * Through INFO, so it lands on stderr: with `-w -` stdout is the
     * pcapng stream, and this runs before the first block of it is
     * written, so a printf here would put a line of English in front of
     * the file's magic number.
     */
    INFO("Don't forget to run at boot-time:"
	 " raspi-gpio set %d,%d pu", dw1000_reset.id, dw1000_wakeup.id);

    
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

	if (uwb_cfg->antenna.tx_delay >= 0)
	    cfg->tx_antenna_delay = (uint16_t)uwb_cfg->antenna.tx_delay;
	if (uwb_cfg->antenna.rx_delay >= 0)
	    cfg->rx_antenna_delay = (uint16_t)uwb_cfg->antenna.rx_delay;

	/* rx_error stays registered either way: without double buffering
	 * dw1000_initialise() no longer requires it, but every receive
	 * error still leaves the receiver off and needs re-arming, and it
	 * is still the only place the error counts come from. rxauto stays
	 * set either way too, and in the single buffered case it is what
	 * re-enables the receiver after a good frame.
	 */
	cfg->dblbuff = uwb_cfg->dblbuff ? 1 : 0;
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
    dw1000_t *drv = &DW0;

    /* The hand-written PRF/preamble-code check that used to be here is
     * gone: dw1000_configure() runs dw1000_radio_is_valid() on every
     * call, which checks the same rule and more (it looks at rx_pcode
     * too, which this did not, and at the preamble length against the
     * build's options). Its return value used to be discarded, so a
     * configuration it refused left the chip untouched while this
     * reported success. It is checked now.
     */
    if (dw1000_configure(drv, radio) < 0)              // Configure radio
	return -1;

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



const struct capture_ops *
uwb_capture_ops(bool metadata)
{
    return metadata ? &uwb_ops_meta : &uwb_ops_plain;
}



void
uwb_rx_set_frame_filtering(uint16_t bitmask)
{
    dw1000_rx_set_frame_filtering(&DW0, bitmask);
}



const struct uwb_rx_errors *
uwb_rx_errors(void)
{
    return &rx_errors;
}



/* 
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
