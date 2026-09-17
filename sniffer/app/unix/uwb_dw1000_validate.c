/*
 * Copyright (c) 2019
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <dw1000/dw1000.h>
#include "config.h"
#include "uwb.h"


bool
uwb_validate_channel(int channel, uint8_t *val, const char **errmsg)
{
    char *msg = NULL;
    switch(channel) {
    case 1: case 2: case 3: case 4: case 5: case 7:
	if (val) *val = channel;
	break;
    case 6:
	msg = "channel 6 not supported by dw1000 hardware";
	break;
    default:
	msg = "channel must be 1, 2, 3, 4, 5, or 7";
	break;
    }
    if (errmsg) *errmsg = msg;

    return msg == NULL;
}


bool
uwb_validate_bitrate(int bitrate, uint8_t *val, const char **errmsg)
{
    char *msg = NULL;
    switch(bitrate) {
    case 6800:
	if (val) *val = DW1000_BITRATE_6800KBPS;
	break;
    case 850:
	if (val) *val = DW1000_BITRATE_850KBPS;
	break;
    case 110:
	if (val) *val = DW1000_BITRATE_110KBPS;
	break;
    default:
	msg = "only 6800kbps, 850kps, and 110kps are supported";
	break;
    }
    if (errmsg) *errmsg = msg;

    return msg == NULL;
}


bool
uwb_validate_prf(int prf, uint8_t *val, const char **errmsg)
{
    char *msg = NULL;
    switch(prf) {
    case 4:
	msg = "PRF of 4MHz not supported by dw1000 hardware";
	break;
    case 16:
	if (val) *val = DW1000_PRF_16MHZ;
	break;
    case 64:
	if (val) *val = DW1000_PRF_64MHZ;
	break;
    default:
	msg = "only PRF od 16MHz or 64MHz are supported";
	break;
    }
    if (errmsg) *errmsg = msg;

    return msg == NULL;
}


bool
uwb_validate_pcode(int pcode, uint8_t *val, const char **errmsg)
{
    char *msg = NULL;
    switch(pcode) {
    case  1: case  2: case  3: case  4: case  5:
    case  6: case  7: case  8: case  9: case 10:
    case 11: case 12: 
             case 17: case 18: case 19: case 20:
	if (val) *val = pcode;
	break;

                      case 13: case 14: case 15:
    case 16: 
        msg = "preamble code 13, 14, 15, 16 are not supported"
	      " in this implementation";		  
	break;
    default:
	msg = "supported preamble code are 1..12 and 17..20";
	break;
    }
    if (errmsg) *errmsg = msg;

    return msg == NULL;
}


bool
uwb_validate_plen(int plen, uint8_t *val, const char **errmsg)
{
    char *msg = NULL;
    switch(plen) {
    case   64:
	if (val) *val = DW1000_PLEN_64;
	break;
    case  128:
	if (val) *val = DW1000_PLEN_128;
	break;
    case  256:
	if (val) *val = DW1000_PLEN_256;
	break;
    case  512:
	if (val) *val = DW1000_PLEN_512;
	break;
    case 1024:
	if (val) *val = DW1000_PLEN_1024;
	break;
    case 1536:
	if (val) *val = DW1000_PLEN_1536;
	break;
    case 2048:
	if (val) *val = DW1000_PLEN_2048;
	break;
    case 4096:
	if (val) *val = DW1000_PLEN_4096;
	break;
    default:
	msg = "supported preamble length are"
	      " 64, 128, 256, 512, 1024, 1536, 2048, 4096";
	break;
    }
    if (errmsg) *errmsg = msg;

    return msg == NULL;
}


bool
uwb_validate_pac(int pac, uint8_t *val, const char **errmsg)
{
    char *msg = NULL;
    switch(pac) {
    case  8:
	if (val) *val = DW1000_PAC8;
	break;
    case 16: 
	if (val) *val = DW1000_PAC16;
	break;
   case 32:
	if (val) *val = DW1000_PAC32;
	break;
    case 64:
	if (val) *val = DW1000_PAC64;
	break;
    default:
	msg = "supported preamblelength are 8, 16, 32, and 64";
	break;
    }
    if (errmsg) *errmsg = msg;

    return msg == NULL;
}


bool
uwb_validate_delay(float dist, uint16_t *val, const char **errmsg)
{
    char *msg  = NULL;
    float time = UWB_ANTENNA_DELAY_METER_TO_CLOCK(dist);
    if (time < 0) {
	msg = "delay doesn't support negative value";
    } else if (time > (UINT16_MAX-1)) {
	msg = "delay is too big";
    } else {
	if (val) *val = (uint16_t)time;
    }
    if (errmsg) *errmsg = msg;

    return msg == NULL;
}


/* 
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
