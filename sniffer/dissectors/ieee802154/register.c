/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The compile-time route's registration, in a file of its own, and this
 * is the part worth copying rather than the dissector next to it.
 *
 * dissect_register() lives in the sniffer. A shared object that calls it
 * has an undefined symbol the sniffer does not export, because the
 * dissector interface is deliberately one way (the sniffer calls the
 * dissector, never the reverse, which is also why the executable is not
 * linked -rdynamic). dissect.c loads plugins with RTLD_NOW, so such an
 * object does not load at all:
 *
 *     Undefined symbol "dissect_register"
 *
 * which is a puzzling way to discover an architectural rule. So the call
 * sits here, in a translation unit that DISSECT_SOURCES includes and that
 * `make plugin` leaves out. The dissector itself references nothing in
 * the sniffer and builds both ways unchanged.
 *
 * A tree providing several dissectors calls dissect_register() once per
 * dissector from this one function.
 */

#include "ieee802154.h"

void dissect_ieee802154_register(void);

void
dissect_ieee802154_register(void)
{
    dissect_register(&ieee802154_dissector);
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
