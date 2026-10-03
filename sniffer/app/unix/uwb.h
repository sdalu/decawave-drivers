/*
 * Copyright (c) 2019
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __UWB_DW1000__H
#define __UWB_DW1000__H

#include <stdbool.h>
#include <dw1000/dw1000.h>

#include "capture.h"

struct uwb_config {
    struct {
	int32_t tx_delay;	/* device ticks, one way; -1 leaves the */
	int32_t rx_delay;	/*   driver's own                       */
    } antenna;

    void (*frame_delivery)(uint32_t status, size_t length, bool ranging);

    /**
     * Double buffered receive (UM §4.3.3), on unless told otherwise.
     *
     * Turning it off is what `--no-dblbuff` is for, and it is a fallback
     * rather than a mode: without it the chip is deaf for the whole
     * read-out of every frame, which is what a sniffer can least afford
     * (hw/drivers/dw1000/README.md says so, and adds that rxauto does
     * not cover that window).
     *
     * Nothing else in this program changes with it. The receiver still
     * re-arms without the loop's help, because rxauto stays set either
     * way and RXAUTR is what re-enables after a good frame when the
     * driver does not; the read-out still happens inside the callback,
     * because there is no reason to move it out; and rx_error is still
     * registered, because every receive error leaves the receiver off
     * whatever the buffering, and it is the only place the error counts
     * come from.
     */
    bool dblbuff;
};

/**
 * Frames the chip reported it could not give us, by reason.
 *
 * This is the other half of the loss account, and it used to be thrown
 * away: the rx_error callback took the status word and dropped it, so a
 * run could lose frames on the chip and say nothing. @p overrun is the
 * one that means frames were received and then lost (both buffers held
 * and a third frame arriving, UM §4.3.3), as opposed to the rest, which
 * mean a frame never decoded. Distinguishing them is the difference
 * between "the host was too slow" and "the link is bad".
 */
struct uwb_rx_errors {
    unsigned long total;	/**< rx_error callbacks, whatever the cause */
    unsigned long overrun;	/**< RXOVRR: frames lost on the chip        */
    unsigned long phy;		/**< RXPHE: PHY header error                */
    unsigned long fcs;		/**< RXFCE: bad frame check sequence        */
    unsigned long sync;		/**< RXRFSL: reed-solomon sync loss         */
    unsigned long lde;		/**< LDEERR: leading edge detection failed  */
    unsigned long sfd_timeout;	/**< RXSFDTO: SFD never arrived             */
    unsigned long rejected;	/**< AFFREJ: frame filtering rejected it    */
    unsigned long unexplained;	/**< an error none of the above accounts for */
};


int uwb_init(struct uwb_config *cfg);
int uwb_config_dw1000_radio(struct dw1000_radio *radio);

int uwb_fill_pollfd(struct pollfd *pollfd);
int uwb_wait_events(void);
int uwb_process_events(void);
int uwb_rx_start(void);

/**
 * The chip side of capture.c's read-out seam.
 *
 * @param[in] metadata	also read the timestamp, the power estimates and
 *			the clock tracking for every frame, which costs
 *			SPI time inside the rx_ok callback
 * @return		ops for capture_init(), never NULL
 */
const struct capture_ops *uwb_capture_ops(bool metadata);

/**
 * Turn hardware frame filtering on or off (UM §5.2.1).
 *
 * @param[in] bitmask	0 disables filtering entirely, so every frame on
 *			the channel is reported; otherwise the
 *			@p DW1000_FLG_SYS_CFG_FFA* set to accept
 */
void uwb_rx_set_frame_filtering(uint16_t bitmask);

/**
 * The receive error counts. Never NULL.
 */
const struct uwb_rx_errors *uwb_rx_errors(void);

/* Nothing is validated here any more. The radio fields and the antenna
 * delay are all the driver's, in <dw1000/dw1000_validate.h>, because
 * which values the chip accepts, and what a distance in metres is in
 * ticks, is the chip's business rather than this program's.
 */


#endif
