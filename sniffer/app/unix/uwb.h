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

#define UWB_ANTENNA_DELAY_METER_TO_CLOCK(x)				\
    (((x) * DW1000_TIME_CLOCK_HZ) / SPEED_OF_LIGHT)

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

bool uwb_validate_channel(int channel, uint8_t *val, const char **errmsg);
bool uwb_validate_bitrate(int bitrate, uint8_t *val, const char **errmsg);
bool uwb_validate_prf    (int prf,     uint8_t *val, const char **errmsg);
bool uwb_validate_pcode  (int pcode,   uint8_t *val, const char **errmsg);
bool uwb_validate_plen   (int plen,    uint8_t *val, const char **errmsg);
bool uwb_validate_pac    (int pac,     uint8_t *val, const char **errmsg);
bool uwb_validate_delay  (float dist, uint16_t *val, const char **errmsg);


#endif
