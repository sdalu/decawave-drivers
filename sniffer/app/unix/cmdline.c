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

	// Radio configuration: the probe's spelling and units, option for
	// option (probe/include/dw1000/probe/radio.h), which
	// tests/check-radio-options.sh holds this table to.
	{ "channel",         'c', POPT_ARG_INT,
	  &config->channel,   1 , "channel (default: 5)", NULL },
	{ "bitrate",         'b', POPT_ARG_INT,
	  &config->bitrate,   2 , "bitrate, in kbps (default: 6800)", NULL },
	{ "prf",             'p', POPT_ARG_INT,
	  &config->prf,       3 , "pulse repetition frequency, in MHz"
	  " (default: 64)", NULL },
	{ "preamble",         0 , POPT_ARG_INT,
	  &config->tx_plen,   4 , "preamble length, in symbols"
	  " (default: 128)", NULL },
	{ "pac",              0 , POPT_ARG_INT,
	  &config->rx_pac,    5 , "preamble acquisition chunk, in symbols"
	  " (default: 8)", NULL },
	{ "code",             0 , POPT_ARG_INT,
	  &config->code_arg,  8 , "preamble code, both ways (default: 10)",
	  NULL },
	{ "tx-code",          0 , POPT_ARG_INT,
	  &config->tx_pcode,  6 , "TX preamble code", NULL },
	{ "rx-code",          0 , POPT_ARG_INT,
	  &config->rx_pcode,  7 , "RX preamble code", NULL },
	{ "sfd",              0 , POPT_ARG_STRING,
	  &config->sfd_arg,   9 , "decawave or standard (default: decawave)",
	  NULL },

	// UWB hardware configuration
	{ "antenna-delay",    0 , POPT_ARG_LONG,
	  &config->antd_arg, 23 , "antenna delay, both ways, in device ticks"
	  " (default: 16475)", NULL },
	{ "tx-antenna-delay", 0 , POPT_ARG_LONG,
	  &config->antd_arg, 24 , "antenna TX delay, in device ticks", NULL },
	{ "rx-antenna-delay", 0 , POPT_ARG_LONG,
	  &config->antd_arg, 25 , "antenna RX delay, in device ticks", NULL },

	// The spellings this program had before it took the probe's,
	// kept working and kept out of --help.
	{ "tx_plen",          0 , POPT_ARG_INT | POPT_ARGFLAG_DOC_HIDDEN,
	  &config->tx_plen,   4 , NULL, NULL },
	{ "rx_pac",           0 , POPT_ARG_INT | POPT_ARGFLAG_DOC_HIDDEN,
	  &config->rx_pac,    5 , NULL, NULL },
	{ "tx_pcode",         0 , POPT_ARG_INT | POPT_ARGFLAG_DOC_HIDDEN,
	  &config->tx_pcode,  6 , NULL, NULL },
	{ "rx_pcode",         0 , POPT_ARG_INT | POPT_ARGFLAG_DOC_HIDDEN,
	  &config->rx_pcode,  7 , NULL, NULL },
	{ "tx_delay",         0 , POPT_ARG_FLOAT | POPT_ARGFLAG_DOC_HIDDEN,
	  &config->delay_m_arg, 21, NULL, NULL },
	{ "rx_delay",         0 , POPT_ARG_FLOAT | POPT_ARGFLAG_DOC_HIDDEN,
	  &config->delay_m_arg, 22, NULL, NULL },

	// Capture and output
	{ "write",           'w', POPT_ARG_STRING,
	  &config->pcapng,  'w' , "write pcapng to FILE (- for stdout)", NULL },
	{ "raw",              0 , POPT_ARG_NONE,
	  &config->raw,       0 , "send bare frames, without the header", NULL },
	{ "no-metadata",      0 , POPT_ARG_NONE,
	  &config->no_metadata, 0, "do not read timestamp/power per frame", NULL },
	{ "dblbuff",          0 , POPT_ARG_VAL,
	  &config->no_dblbuff,  0, "double buffered receive (default)", NULL },
	{ "no-dblbuff",       0 , POPT_ARG_VAL,
	  &config->no_dblbuff,  1, "single buffered receive", NULL },
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
	case  8:
	    CMDLINE_DW1000_VALIDATE(pcode,   config->code_arg, &errmsg);
	    config->tx_pcode = config->rx_pcode = config->code_arg;
	    break;
	case  9:
	    if (strcmp(config->sfd_arg, "standard") == 0) {
		config->sfd_decawave = 0;
	    } else if (strcmp(config->sfd_arg, "decawave") == 0) {
#if DW1000_WITH_PROPRIETARY_SFD
		config->sfd_decawave = 1;
#else
		DIE("uwb: the Decawave SFD needs a driver built with "
		    "DW1000_WITH_PROPRIETARY_SFD");
#endif
	    } else {
		DIE("uwb: --sfd is decawave or standard");
	    }
	    break;

	/* Ticks, one way, any 16-bit value: what the chip holds, and what
	 * the probe takes, so a delay read off one tool's output can be
	 * given to the other unchanged. */
	case 23:
	case 24:
	case 25:
	    if (config->antd_arg < 0 || config->antd_arg > 65535) {
		DIE("uwb: an antenna delay is device ticks, 0 .. 65535");
	    }
	    if (c != 25) config->tx_antd = config->antd_arg;
	    if (c != 24) config->rx_antd = config->antd_arg;
	    break;

	/* The old spellings, in metres, converted here so that everything
	 * after this sees ticks. */
	case 21:
	case 22: {
	    uint16_t ticks;

	    if (! dw1000_validate_antenna_delay(config->delay_m_arg, &ticks,
						&errmsg)) {
		DIE("uwb: %s", errmsg);
	    }
	    if (c == 21) config->tx_antd = ticks;
	    else         config->rx_antd = ticks;
	    break;
	}
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
