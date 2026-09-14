/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Logging for the emulation port.
 *
 * The port is a library inside somebody else's program, so it owns no log
 * configuration: everything goes to stderr, prefixed, and the trace is
 * compiled out unless DW1000_EMULATION_DEBUG is defined. The quiet form
 * still expands its arguments inside an `if (0)`, so that the format
 * string keeps being checked and a variable that only feeds a trace does
 * not turn into an unused-variable warning.
 */

#ifndef __DW1000_EMU_LOG_H__
#define __DW1000_EMU_LOG_H__

#include <stdio.h>
#include <stdlib.h>

#define EMU_LOG_PREFIX	"dw1000-emulation: "

#if defined(DW1000_EMULATION_DEBUG)
#define EMU_DEBUG(fmt, ...)						\
    fprintf(stderr, EMU_LOG_PREFIX fmt "\n", ##__VA_ARGS__)
#else
#define EMU_DEBUG(fmt, ...)						\
    do {								\
	if (0)								\
	    fprintf(stderr, EMU_LOG_PREFIX fmt "\n", ##__VA_ARGS__);	\
    } while (0)
#endif

#define EMU_WARNING(fmt, ...)						\
    fprintf(stderr, EMU_LOG_PREFIX fmt "\n", ##__VA_ARGS__)

#define EMU_FATAL(fmt, ...)						\
    do {								\
	fprintf(stderr, EMU_LOG_PREFIX fmt "\n", ##__VA_ARGS__);	\
	abort();							\
    } while (0)

#endif
