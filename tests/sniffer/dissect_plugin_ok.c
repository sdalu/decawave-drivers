/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * A fixture for tests/sniffer/dissect.c: the smallest thing that is a
 * valid loadable dissector. Built into a shared object by
 * tests/check-sniffer.sh.
 *
 * It references nothing in the sniffer, which is not incidental: a plugin
 * that does will not load, because dissect.c uses RTLD_NOW and the
 * executable exports nothing. That is exactly why the example dissector
 * keeps its dissect_register() call in a separate file, and this fixture
 * is what proves the rule holds.
 */

#include <stdio.h>
#include <string.h>

#include "dissect.h"

static int
_open(const char *args)
{
    /* Echoed back through describe() below, so the test can check that
     * --dissector=PATH:args really reaches the plugin. */
    static char seen[64];

    snprintf(seen, sizeof(seen), "%s", (args != NULL) ? args : "(none)");
    return 0;
}

static size_t
_describe(const struct dissect_frame *frame, char *out, size_t outsz)
{
    int n;

    /* Claims only frames whose first byte is 0xAA, so that the test can
     * hand it a frame it will refuse and check that 0 means "not mine". */
    if ((frame->length == 0) || (frame->data[0] != 0xAA))
	return 0;

    n = snprintf(out, outsz, "plugin-ok len=%zu seq=%u",
		 frame->length, (unsigned)frame->seq);
    return (n < 0) ? 0 : (size_t)n;
}

static const struct dissector plugin_ok = {
    .abi      = DISSECT_ABI,
    .name     = "plugin-ok",
    .version  = "fixture",
    .open     = _open,
    .accept   = NULL,
    .describe = _describe,
    .close    = NULL,
};

DISSECT_EXPORT const struct dissector *uwb_dissector_v1(void);

DISSECT_EXPORT const struct dissector *
uwb_dissector_v1(void)
{
    return &plugin_ok;
}
