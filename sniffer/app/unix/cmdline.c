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
 * fields and antenna delay alike -- see <dw1000/dw1000_validate.h>.
 */
#define CMDLINE_DW1000_VALIDATE(_name, _var, _msg)		\
    do {							\
	if (! dw1000_validate_##_name((_var), NULL, (_msg))) {	\
	    DIE("uwb: %s", *(_msg));				\
	}							\
    } while(0)

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
    poptSetOtherOptionHelp(popt_ctx, "[OPTIONS]* <dst_macaddr>");

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
	    break;

	case 'P':
	    if (((config->proto > 0) && (config->proto <= 0x05DC)) ||
 		(config->proto > 0xFFFF)) {
		DIE("prototype value must be 0 or 0x05DD..0xFFFF");
	    }
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

    const char *addr = poptGetArg(popt_ctx);
    if (addr == NULL) {
    usage:
	poptPrintUsage(popt_ctx, stderr, 0);
	exit(EXIT_USAGE);
    }
    eth_parse_addr(addr, config->dst_addr);

    if (poptGetArg(popt_ctx) != NULL) {
	goto usage;
    }

    poptFreeContext(popt_ctx);

    return 0;
}
