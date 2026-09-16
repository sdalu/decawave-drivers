/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_PROBE_ROLE_H__
#define __DW1000_PROBE_ROLE_H__

/**
 * @file    role.h
 * @brief   What the instrument can be asked to do.
 *
 * The set of roles is the instrument's definition, so it is here. How a
 * particular host spells the verb that selects one -- a shell subcommand,
 * an argv word, a key in a config file -- is the application's, and may
 * differ. What must not differ is the identifier printed in `role=`:
 * that is read by one parser, and a board saying `resp` where a Linux
 * host says `twr_resp` would make the parser need to know which it was
 * reading.
 *
 * @addtogroup PROBE
 * @{
 */

#include <stdbool.h>

/**
 * @brief The roles.
 */
typedef enum {
    DW1000_PROBE_ROLE_TWR_INIT = 0,    /**< initiator of the four-frame exchange  */
    DW1000_PROBE_ROLE_TWR_RESP,        /**< responder; the end that emits records */
    DW1000_PROBE_ROLE_TX,              /**< transmit only: heats the far end      */
    DW1000_PROBE_ROLE_RX,              /**< receive only: heats this end          */
    DW1000_PROBE_ROLE_TEMPERATURE,     /**< radio stopped, sample the die         */
    DW1000_PROBE_ROLE__COUNT
} dw1000_probe_role_t;

/**
 * @brief The canonical spelling of a role, as printed in `role=`.
 *
 * These five spellings and no others, matching the reference instrument's
 * own role names so that a record from either names its role the same
 * way:
 *
 *     "twr_init"  "twr_resp"  "tx"  "rx"  "temperature"
 *
 * They are written out here because this is where a `resp`-versus-
 * `twr_resp` drift would otherwise happen unseen, inside an
 * implementation nobody re-reads.
 *
 * Never returns NULL; an out-of-range role gives "invalid".
 */
const char *dw1000_probe_role_name(dw1000_probe_role_t role);

/**
 * @brief The role a canonical name denotes.
 *
 * The inverse of dw1000_probe_role_name(). Matching is exact: a shell that wants
 * to accept abbreviations does that itself, because which abbreviations
 * are acceptable is a question about that shell's users.
 *
 * @param[in]  name  a canonical name
 * @param[out] role  set only on success
 * @return     true if the name was recognised
 */
bool dw1000_probe_role_lookup(const char *name, dw1000_probe_role_t *role);

/** @} */

#endif
