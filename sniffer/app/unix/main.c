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

#include "config.h"
#include "cmdline.h"
#include "uwb.h"
#include "eth.h"

#ifndef __arraycount
#define	__arraycount(__x)	(sizeof(__x) / sizeof(__x[0]))
#endif



static void
_rx_ok(uint32_t status, size_t length, bool ranging) {
    printf("GOT RX_OK: size=%ld\n", length);
    uint8_t data[length];
    uwb_read_frame_data(data, length, 0);
    
    uint8_t addr[] = { 0xdc, 0x4a, 0x3e, 0x06, 0x6f, 0x7b };
    if (eth_send(addr, data, length) < 0) {
	WARN_ERRNO("failed sending ethernet frame");
    }
}




struct config config = {
    .ifname_default = "eth0",
    .ifname         = config.ifname_default,
    .verbose        = 0,
    .proto          = 9999,
    .tx_delay       = UINT16_MAX,
    .rx_delay       = UINT16_MAX,
    .channel        = 5,
    .bitrate        = 6800,
    .prf            = 64,
    .tx_pcode       = 10,
    .rx_pcode       = 10,
    .tx_plen        = 128,
    .rx_pac         = 8,
};





/* Radio configuration of the DW1000 
 *
 * UM §9.3  : Data rate, preamble length, PRF
 * UM §4.1.1: Preamble detection
 * UM §10.5 : UWB channels and preamble codes
 *
 * Recommanded preamble length for the following bitrates:
 *  6800 kbps :   64 or  128 or  256
 *   850 kbps :  256 or  512 or 1024
 *   110 kbps : 2048 or 4096
 *
 * PLEN (Preamble length) / PAC (Preamble Acquisition Chunk)
 *   tx_plen: 64 | 128 | 256 | 512 | 1024 | 1536 | 2048 | 4096
 *   rx_pac : 8  | 8   | 16  | 16  | 32   | 64   | 64   | 64
 *
 * Recommanded preamble codes according to selected channel and PRF:
 *  Channel | Preamble codes | Preamble codes 
 *          | for 16MHz PRF  | for 64 MHz PRF
 * ---------+----------------+----------------
 *     1    |     1, 2       |  9, 10, 11, 12
 *     2    |     3, 4       |  9, 10, 11, 12
 *     3    |     5, 6       |  9, 10, 11, 12
 *     4    |     7, 8       | 17, 18, 19, 20
 *     5    |     3, 4       |  9, 10, 11, 12
 *     7    |     7, 8       | 17, 18, 19, 20
 */
static struct dw1000_radio dw1000_radio = {
    .channel          = 5, // Possible to use 2 or 5 with the same parameters
    .bitrate          = DW1000_BITRATE_6800KBPS,
    .prf              = DW1000_PRF_64MHZ,
    .tx_plen          = DW1000_PLEN_128, // UM §9.3
    .rx_pac           = DW1000_PAC8,     // UM §4.1
    .tx_pcode         = 10,              // UM §10.5
    .rx_pcode         = 10,              // UM §10.5
#if DW1000_WITH_PROPRIETARY_SFD || DW1000_WITH_PROPRIETARY_LONG_FRAME
    .proprietary      = {
#if DW1000_WITH_PROPRIETARY_LONG_FRAME
       .long_frames  = 0,
#endif
#if DW1000_WITH_PROPRIETARY_SFD
       .sfd          = 1,
#endif
    },
#endif
#if DW1000_WITH_SFD_TIMEOUT
    .sfd_timeout      = DW1000_SFD_TIMEOUT_MAX
#endif
};


/* UWB hardware configuration
 * Antenna delay (UINT16_MAX -> using default value)
 */
static struct uwb_config uwb_config = {
    .antenna        = { .tx_delay = UINT16_MAX, .rx_delay = UINT16_MAX },
    .frame_delivery = _rx_ok,
};





int loop(void) {
    struct pollfd pollfd[1];
    uwb_fill_pollfd(&pollfd[0]);

    uwb_rx_start();
    
    while(1) {
	int n = poll(pollfd, __arraycount(pollfd), -1);
	if (n <= 0) continue;

	if (pollfd[0].revents) {
	    uwb_wait_events();
	    uwb_process_events();
	    uwb_rx_start();
	}
    }
}


int main(int argc, const char* argv[]) {
    uint8_t hwaddr[ETH_HWADDR_SIZE];
    
    /* Lookup for default ethernet interface 
     */
    if (eth_get_first_interface(config.ifname_default) < 0) {
	DIE("No valid ethernet interface available");
    }

    /* Parse command line
     */
    if (cmdline_parse(&config, argc, argv) < 0) {
	DIE("Failed to parse command line");
    }
    
    /* Initialize bitters library
     */ 
    if (bitters_init() < 0) {
	DIE_ERRNO("bitters library initialisation failed");
    }

    /* Try to tune program for reduced latency
     */
    if (bitters_reduced_latency() < 0) {
	WARN_ERRNO("unable to tune program for reduced latency");
    }

    /* Initialize network interface
     */
    if (eth_init(config.ifname, config.proto, hwaddr) < 0) {
	DIE_ERRNO("failed to initialize ethernet interface");
    }
    
    /* Initialize UWB
     */
    uwb_config.antenna.tx_delay = config.tx_delay;
    uwb_config.antenna.rx_delay = config.rx_delay;
    if (uwb_init(&uwb_config) < 0) {
	DIE("failed to initialise dw1000 driver");
    }

    uwb_validate_channel(config.channel,  &dw1000_radio.channel,  NULL);
    uwb_validate_bitrate(config.bitrate,  &dw1000_radio.bitrate,  NULL);
    uwb_validate_prf    (config.prf,      &dw1000_radio.prf,      NULL);
    uwb_validate_pcode  (config.tx_pcode, &dw1000_radio.tx_pcode, NULL);
    uwb_validate_pcode  (config.rx_pcode, &dw1000_radio.rx_pcode, NULL);
    uwb_validate_plen   (config.tx_plen,  &dw1000_radio.tx_plen,  NULL);
    uwb_validate_pac    (config.rx_pac,   &dw1000_radio.rx_pac,   NULL);

    if (uwb_config_dw1000_radio(&dw1000_radio) < 0) {
	DIE("selected UWB configuration is invalid");
    }

    /*
     */
    printf("To capture packets on the remote side you can use:\n");
    printf("# tcpdump ether proto 0x%04x"
	        " and ether src %02x:%02x:%02x:%02x:%02x:%02x"
	   "\n",
	   config.proto,
	   hwaddr[0], hwaddr[1], hwaddr[2], hwaddr[3], hwaddr[4], hwaddr[5]
	   );
    
    /* Loop for incoming packets
     */
    if (loop() < 0) {
	DIE("failed to run");
    }

    /* Job's done...
     *  (it's a lie, wa never finish)
     */
    return 0;
}




/* 
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
