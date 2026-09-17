#ifndef __CONFIG__H
#define __CONFIG__H

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <bitters/rpi.h>


#define EXIT_OK		0
#define EXIT_USAGE	1
#define EXIT_ERROR	2


#define RPI_DW1000_MOSI         BITTERS_RPI_SPI0_MOSI	// RPI_P1_19
#define RPI_DW1000_MISO         BITTERS_RPI_SPI0_MISO	// RPI_P1_21
#define RPI_DW1000_SCLK         BITTERS_RPI_SPI0_SCLK	// RPI_P1_23
#define RPI_DW1000_SS           BITTERS_RPI_SPI0_CE0	// RPI_P1_24

/*
 * One pin map, the one wired on the bench: SPI0/CE0, reset on P1_18,
 * wakeup on P1_16, interrupt on P1_15. Identical to rpi-redskin/config.h
 * and to probe/app/unix/config.h, which took it from there -- so a Pi
 * that already runs either of those runs this with nothing rewired.
 *
 * This program used to differ (wakeup on P1_22, interrupt on P1_16),
 * which meant a bench could run redskin or the sniffer but not both
 * without moving two jumpers.
 */
#define RPI_DW1000_SPI		BITTERS_RPI_SPI0
#define RPI_DW1000_WAKEUP       BITTERS_RPI_P1_16
#define RPI_DW1000_RESET        BITTERS_RPI_P1_18
#define RPI_DW1000_IRQ          BITTERS_RPI_P1_15

#define RPI_GPIO_PIN_INITIALIZER(name)					\
    BITTERS_GPIO_PIN_INITIALIZER(BITTERS_RPI_GPIO_CHIP, RPI_##name)

#define RPI_SPI_INITIALIZER(name, ce)					\
    BITTERS_SPI_INITIALIZER(RPI_##name, ce)


#define INFO(x, ...)					\
    fprintf(stdout, x "\n", ##__VA_ARGS__)

#define WARN(x, ...)					\
    fprintf(stderr, x "\n", ##__VA_ARGS__)

#define DIE(x, ...)					\
    do {						\
	fprintf(stderr, x "\n", ##__VA_ARGS__);		\
	exit(EXIT_ERROR);				\
    } while(0)

#define WARN_ERRNO(x, ...)				\
    WARN(x " (%s)", ##__VA_ARGS__, strerror(errno))

#define DIE_ERRNO(x, ...)				\
    DIE(x " (%s)", ##__VA_ARGS__, strerror(errno))


#include "eth.h"
    
struct config {
    char    ifname_default[ETH_NAME_SIZE];
    char   *ifname;
    int     verbose;
    int     proto;
    uint8_t dst_addr[ETH_HWADDR_SIZE];
    float   tx_delay;
    float   rx_delay;
    int     channel;
    int     bitrate;
    int     prf;
    int     tx_pcode;
    int     rx_pcode;
    int     tx_plen;
    int     rx_pac;
}; 

extern struct config config;

#endif
