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

struct uwb_config {
    struct {
	uint16_t tx_delay;
	uint16_t rx_delay;
    } antenna;

    void (*frame_delivery)(uint32_t status, size_t length, bool ranging);
};


int uwb_init(struct uwb_config *cfg);
int uwb_config_dw1000_radio(struct dw1000_radio *radio);

int uwb_fill_pollfd(struct pollfd *pollfd);
int uwb_wait_events(void);
int uwb_process_events(void);
int uwb_rx_start(void);
int uwb_read_frame_data(uint8_t *data, size_t length, size_t offset);

/* Nothing is validated here any more. The radio fields and the antenna
 * delay are all the driver's, in <dw1000/dw1000_validate.h>, because
 * which values the chip accepts, and what a distance in metres is in
 * ticks, is the chip's business rather than this program's.
 */


#endif
