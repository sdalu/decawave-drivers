/*
 * Copyright (c) 2019
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <popt.h>

#include <dw1000/dw1000_validate.h>

#include "config.h"
#include "cmdline.h"
#include "eth.h"
#include "uwb.h"

/* Every value on this command line is the driver's to judge, radio
 * fields and antenna delay alike; see <dw1000/dw1000_validate.h>.
 */
#define CMDLINE_DW1000_VALIDATE(_name, _var, _msg)		\
    do {							\
	if (! dw1000_validate_##_name((_var), NULL, (_msg))) {	\
	    DIE("uwb: %s", *(_msg));				\
	}							\
    } while(0)

/* A string kept past poptFreeContext(), so copied.
 *
 * popt allocates the value of a POPT_ARG_STRING itself rather than
 * pointing into argv, and in practice does not free it in
 * poptFreeContext(): a value still reads correctly afterwards even with
 * the allocator poisoning what it frees. But popt(3) does not say so,
 * and the example it gives reads its string before freeing the context
 * rather than after, so it settles nothing. Three fields in `config`
 * outlive that call (the interface, the capture file and every
 * --dissector), and a copy costs a few bytes once and removes the
 * question.
 */
static char *
_keep(const char *s)
{
    char *copy = strdup(s);

    if (copy == NULL) {
	DIE("out of memory");
    }
    return copy;
}

int
cmdline_parse(struct config *config, int argc, const char* argv[])
{
    /* Parse command line 
     */
    struct poptOption optionsTable[] = {
        // Ethernet configuration
        { "prototype",       'P', POPT_ARG_INT | POPT_ARGFLAG_SHOW_DEFAULT,
	  &config->proto,    'P', "ethernet prototype", NULL },
        { "interface",       'i', POPT_ARG_STRING | POPT_ARGFLAG_SHOW_DEFAULT,
	  &config->ifname,   'i', "ethernet interface", NULL },

	// Radio configuration
	{ "channel",         'c', POPT_ARG_INT,
	  &config->channel,   1 , "channel", NULL },
	{ "bitrate",         'b', POPT_ARG_INT,
	  &config->bitrate,   2 , "bitrate (in kbps)", NULL },
	{ "prf",             'p', POPT_ARG_INT,
	  &config->prf,       3 , "pulse rate frequency (in MHz)", NULL },
	{ "tx_plen",          0 , POPT_ARG_INT,
	  &config->tx_plen,   4 , "preamble length", NULL },
	{ "rx_pac",           0 , POPT_ARG_INT,
	  &config->rx_pac,    5 , "preamble accumulation", NULL },
	{ "tx_pcode",         0 , POPT_ARG_INT,
	  &config->tx_pcode,  6 , "TX preamble code", NULL },
	{ "rx_pcode",         0 , POPT_ARG_INT,
	  &config->rx_pcode,  7 , "RX preamble code", NULL },

	// UWB hardware configuration
	{ "tx_delay",         0 , POPT_ARG_FLOAT,
	  &config->tx_delay, 21 , "antenna TX delay (in meters)", NULL },
	{ "rx_delay",         0 , POPT_ARG_FLOAT,
	  &config->rx_delay, 22 , "antenna RX delay (in meters)", NULL },

	// Capture and output
	{ "write",           'w', POPT_ARG_STRING,
	  &config->pcapng,  'w' , "write pcapng to FILE (- for stdout)", NULL },
	{ "raw",              0 , POPT_ARG_NONE,
	  &config->raw,       0 , "send bare frames, without the header", NULL },
	{ "no-metadata",      0 , POPT_ARG_NONE,
	  &config->no_metadata, 0, "do not read timestamp/power per frame", NULL },
	{ "no-dblbuff",       0 , POPT_ARG_NONE,
	  &config->no_dblbuff,  0, "single buffered receive", NULL },
	{ "count",            0 , POPT_ARG_LONG,
	  &config->count,    'n', "stop after N frames", NULL },
	{ "stats",            0 , POPT_ARG_INT,
	  &config->stats,    's', "report counters every N seconds", NULL },

	// Dissectors
	{ "dissector",        0 , POPT_ARG_STRING,
	  &config->dissector_arg, 'D',
	  "load a dissector from FILE[:args] (repeatable)", NULL },
	{ "dissect-filter",   0 , POPT_ARG_NONE,
	  &config->dissect_filter, 0,
	  "forward only frames a dissector accepts", NULL },
	{ "list-dissectors",  0 , POPT_ARG_NONE,
	  &config->list_dissectors, 0,
	  "print the dissectors and exit", NULL },
	{ "no-dissect",       0 , POPT_ARG_NONE,
	  &config->no_dissect, 0, "ignore every dissector", NULL },

	// Misc.
        { "verbose",         'v', POPT_ARG_NONE,
	  &config->verbose,   0 , "verbose mode", NULL },
        { "version",         'V', POPT_ARG_NONE,
	  NULL,              'V', "show version information", NULL },
	POPT_AUTOHELP
	POPT_TABLEEND
    };

    poptContext popt_ctx;
    popt_ctx = poptGetContext(NULL, argc, argv, optionsTable, 0);
    poptSetOtherOptionHelp(popt_ctx, "[OPTIONS]* [<dst_macaddr>]");

    int c;
    while ((c = poptGetNextOpt(popt_ctx)) >= 0) {
	const char *errmsg = NULL;
	switch (c) {
	case 'i':
	    if ((strlen(config->ifname) + 1) > ETH_NAME_SIZE) {
		DIE("inteface name too long");
	    }
	    if (eth_validate_interface(config->ifname, &errmsg) <= 0) {
		DIE("invalid interface (%s)", errmsg);
	    }
	    config->ifname = _keep(config->ifname);
	    break;

	case 'P':
	    if (((config->proto > 0) && (config->proto <= 0x05DC)) ||
 		(config->proto > 0xFFFF)) {
		DIE("prototype value must be 0 or 0x05DD..0xFFFF");
	    }
	    break;
	    
	case 'w':
	    if (config->pcapng[0] == '\0') {
		DIE("empty file name for --write");
	    }
	    config->pcapng = _keep(config->pcapng);
	    break;

	case 'n':
	    if (config->count < 0) {
		DIE("--count must not be negative");
	    }
	    break;

	case 's':
	    if (config->stats < 0) {
		DIE("--stats must not be negative");
	    }
	    break;

	/* Kept one at a time as popt hands them over: POPT_ARG_STRING
	 * writes each occurrence over the same pointer, so a repeatable
	 * option has to be collected here or every one but the last is
	 * lost. */
	case 'D':
	    if (config->dissector_arg[0] == '\0') {
		DIE("empty --dissector");
	    }
	    if (config->dissectors >= SNIFFER_DISSECT_MAX) {
		DIE("at most %d --dissector options", SNIFFER_DISSECT_MAX);
	    }
	    config->dissector[config->dissectors++] =
		_keep(config->dissector_arg);
	    break;

	case 'V':
	    printf("Version: %s\n", VERSION);
	    exit(EXIT_OK);

	case  1:
	    CMDLINE_DW1000_VALIDATE(channel, config->channel,  &errmsg);
	    break;
	case  2:
	    CMDLINE_DW1000_VALIDATE(bitrate, config->bitrate,  &errmsg);
	    break;
        case  3:
	    CMDLINE_DW1000_VALIDATE(prf,     config->prf,      &errmsg);
	    break;
	case  4:
	    CMDLINE_DW1000_VALIDATE(plen,    config->tx_plen,  &errmsg);
	    break;
	case  5:
	    CMDLINE_DW1000_VALIDATE(pac,     config->rx_pac,   &errmsg);
	    break;
	case  6:
	    CMDLINE_DW1000_VALIDATE(pcode,   config->tx_pcode, &errmsg);
	    break;
	case  7:
	    CMDLINE_DW1000_VALIDATE(pcode,   config->rx_pcode, &errmsg);
	    break;

	case 21:
	    CMDLINE_DW1000_VALIDATE(antenna_delay, config->tx_delay, &errmsg);
	    break;
	case 22:
	    CMDLINE_DW1000_VALIDATE(antenna_delay, config->rx_delay, &errmsg);
	    break;
	}

    }

    /* The destination is required, unless -w gives the capture somewhere
     * else to go: a run that writes pcapng locally has no second host
     * and nothing to address, which is what makes
     * `uwb-sniffer -w - | wireshark -k -i -` possible at all. With
     * neither, the program would capture and then discard, so that is
     * the usage error.
     */
    const char *addr = poptGetArg(popt_ctx);
    if (addr == NULL) {
	/* --list-dissectors prints and exits without capturing, so it is
	 * the one mode that needs neither a destination nor -w. */
	if ((config->pcapng == NULL) && !config->list_dissectors) {
	usage:
	    poptPrintUsage(popt_ctx, stderr, 0);
	    exit(EXIT_USAGE);
	}
    } else {
	/* Checked, which it was not: a destination that does not parse
	 * left dst_addr as it was, all zeroes, and the program then ran
	 * happily sending every frame to 00:00:00:00:00:00. A typo in a
	 * MAC address is the single most likely thing to be wrong on this
	 * command line, and it was the one thing not reported.
	 */
	if (eth_parse_addr(addr, config->dst_addr) < 0) {
	    DIE("not an ethernet address: %s", addr);
	}
	config->dst_valid = 1;
    }

    if (poptGetArg(popt_ctx) != NULL) {
	goto usage;
    }

    poptFreeContext(popt_ctx);

    return 0;
}
