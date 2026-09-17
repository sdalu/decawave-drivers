/*
 * Copyright (c) 2019
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _CMDLINE__H
#define _CMDLINE__H

#include "config.h"

int cmdline_parse(struct config *config, int argc, const char* argv[]);

#endif
