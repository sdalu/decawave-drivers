#ifndef __CONFIG__H
#define __CONFIG__H

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <bitters/rpi.h>


/* How many `--dissector=` may be given. Eight, to match dissect.c's
 * table: a ninth would be refused there anyway, and refusing it at the
 * command line instead says so before anything is loaded. */
#define SNIFFER_DISSECT_MAX	8

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
 * and to probe/app/unix/config.h, which took it from there, so a Pi
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


/* Everything a person reads goes to stderr, INFO included.
 *
 * INFO used to be stdout, which was harmless while the only thing this
 * program produced was ethernet frames. It is not harmless now: `-w -`
 * writes the pcapng capture to stdout, and one informational line in the
 * middle of it makes the file unreadable from that byte on. So stdout
 * carries capture data and nothing else, which is the convention
 * tcpdump(1) has always followed for the same reason.
 */
#define INFO(x, ...)					\
    fprintf(stderr, x "\n", ##__VA_ARGS__)

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
    
/*
 * The three negative flags below are `int` set by POPT_ARG_NONE and read
 * as "the user asked for the non-default", rather than positive fields
 * pre-set to 1 and cleared. POPT_ARG_VAL would express the positive form
 * in one line, but what it stores and what poptGetNextOpt() then returns
 * differ between the two shapes, and a command line flag whose behaviour
 * depends on a subtlety of the option table is not worth the line it
 * saves. main.c turns them the right way round once, where it is visible.
 */
struct config {
    char    ifname_default[ETH_NAME_SIZE];
    char   *ifname;
    int     verbose;
    int     proto;
    uint8_t dst_addr[ETH_HWADDR_SIZE];
    int     dst_valid;		/* a destination was given on the line  */
    long    tx_antd;		/* antenna delays, device ticks, one way; */
    long    rx_antd;		/*   -1 leaves the driver's own           */
    long    antd_arg;		/* where popt lands one, before it is kept */
    float   delay_m_arg;	/* the same, in metres (the old options)  */
    int     channel;
    int     bitrate;
    int     prf;
    int     tx_pcode;
    int     rx_pcode;
    int     code_arg;		/* --code, before it is kept as both      */
    char   *sfd_arg;		/* --sfd, before it is kept               */
    int     sfd_decawave;	/* the proprietary SFD, else the standard */
    int     tx_plen;
    int     rx_pac;
    char   *pcapng;		/* -w: file to write, "-" for stdout    */
    long    count;		/* --count: stop after N frames, 0 = no */
    int     stats;		/* --stats: report every N seconds, 0 = no */
    int     raw;		/* --raw: no sniffer header on the wire */
    int     no_metadata;	/* --no-metadata                        */
    int     no_dblbuff;		/* --no-dblbuff, cleared by --dblbuff   */
    char   *dissector[SNIFFER_DISSECT_MAX];	/* --dissector=PATH[:args]  */
    int     dissectors;		/* how many of the above are set        */
    char   *dissector_arg;	/* where popt lands one, before it is kept */
    int     dissect_filter;	/* --dissect-filter                     */
    int     list_dissectors;	/* --list-dissectors                    */
    int     no_dissect;		/* --no-dissect                         */
}; 

extern struct config config;

#endif
