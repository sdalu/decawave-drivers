/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IEEE802154_DISSECTOR__H
#define __IEEE802154_DISSECTOR__H

#include "dissect.h"

/*
 * Shared between the dissector itself and register.c, which is the only
 * reason it is not static. The split is not tidiness: see register.c.
 */
extern const struct dissector ieee802154_dissector;

#endif
