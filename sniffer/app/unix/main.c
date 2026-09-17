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
#include <dw1000/dw1000_validate.h>

#include "config.h"
#include "cmdline.h"
#include "uwb.h"
#include "eth.h"
#include "capture.h"

#ifndef __arraycount
#define	__arraycount(__x)	(sizeof(__x) / sizeof(__x[0]))
#endif



/* Producer half of the ring, and nothing else.
 *
 * Under double buffering this runs with the receiver already armed and the
 * next frame landing in the other buffer, so it does the one thing that
 * cannot wait -- the read-out, inside capture_put() -- and returns. What
 * used to be here, a per-frame printf and a blocking sendmsg(2), now
 * happens in loop() below, on what the ring holds. It must not re-enable
 * the receiver: the driver already did. See capture.c.
 */
static void
_rx_ok(uint32_t status, size_t length, bool ranging) {
    (void)status;
    (void)ranging;

    capture_put(length);
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

    /* Armed once, here, and not again per iteration. Under double
     * buffering the driver re-enables the receiver itself in the
     * good-frame path, and errors re-arm from _rx_error() in
     * uwb_dw1000.c, so there is nothing left for this loop to arm. The
     * re-arm that used to sit at the bottom of it is the mistake probe
     * removed in "probe: receive double-buffered, and stop arming the
     * receiver off": an unconditional re-arm around a wait turns the
     * receiver off for exactly the window a frame arrives in.
     */
    uwb_rx_start();

    unsigned long reported_lost = 0;

    while(1) {
	int n = poll(pollfd, __arraycount(pollfd), -1);
	if (n <= 0) continue;

	if (pollfd[0].revents) {
	    uwb_wait_events();
	    uwb_process_events();

	    /* Drain what the callbacks captured. This is where the frame
	     * leaves the program, deliberately outside the callback that
	     * received it -- eth_send() is a blocking sendmsg(2).
	     */
	    struct capture_frame frame;
	    while (capture_get(&frame)) {
		if (config.verbose) {
		    INFO("GOT RX_OK: size=%zu", frame.length);
		}
		if (eth_send(config.dst_addr, frame.data, frame.length) < 0) {
		    WARN_ERRNO("failed sending ethernet frame");
		}
	    }

	    /* Frames the ring had to overwrite before they could be sent.
	     * Reported when the count moves rather than per frame, so a
	     * burst of losses does not itself cost time in this loop.
	     */
	    const struct capture_stats *stats = capture_stats();
	    if (stats->overrun != reported_lost) {
		WARN("lost %lu frame(s): forwarding fell behind the radio",
		     stats->overrun - reported_lost);
		reported_lost = stats->overrun;
	    }
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
    /* Metres on the command line, ticks in the chip -- and the conversion
     * used to be computed and thrown away. cmdline_parse() validates with
     * a NULL out parameter, so what landed here was the raw metres: an
     * explicit --tx_delay 154.6 set 154 ticks, about 0.7 m, instead of
     * 32951. The defaults (UINT16_MAX) mean "leave the driver's own
     * alone" and must not be converted, which is what the guard is for.
     */
    if (config.tx_delay != UINT16_MAX)
	dw1000_validate_antenna_delay(config.tx_delay,
				      &uwb_config.antenna.tx_delay, NULL);
    if (config.rx_delay != UINT16_MAX)
	dw1000_validate_antenna_delay(config.rx_delay,
				      &uwb_config.antenna.rx_delay, NULL);
    if (uwb_init(&uwb_config) < 0) {
	DIE("failed to initialise dw1000 driver");
    }

    /* Already checked, with the message, in cmdline_parse(); this is the
     * pass that writes the encoded values into the radio. The combination
     * they make is not checked here and must not be -- that is
     * dw1000_configure()'s job, by way of uwb_config_dw1000_radio() below.
     */
    dw1000_validate_channel(config.channel,  &dw1000_radio.channel,  NULL);
    dw1000_validate_bitrate(config.bitrate,  &dw1000_radio.bitrate,  NULL);
    dw1000_validate_prf    (config.prf,      &dw1000_radio.prf,      NULL);
    dw1000_validate_pcode  (config.tx_pcode, &dw1000_radio.tx_pcode, NULL);
    dw1000_validate_pcode  (config.rx_pcode, &dw1000_radio.rx_pcode, NULL);
    dw1000_validate_plen   (config.tx_plen,  &dw1000_radio.tx_plen,  NULL);
    dw1000_validate_pac    (config.rx_pac,   &dw1000_radio.rx_pac,   NULL);

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
