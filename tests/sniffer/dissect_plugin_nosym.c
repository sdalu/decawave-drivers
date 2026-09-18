/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * A fixture for tests/sniffer/dissect.c: a shared object that loads
 * perfectly well and is not a dissector, because it exports no
 * uwb_dissector_v1. dissect_load() must refuse it and say so, rather
 * than crash on a NULL entry point.
 *
 * Deliberately exporting something, so that the object is not empty and
 * the refusal is about the missing symbol rather than about the file.
 */

int uwb_not_a_dissector(void);

int
uwb_not_a_dissector(void)
{
    return 1;
}
