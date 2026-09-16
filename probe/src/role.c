/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    role.c
 * @brief   The canonical role names, and their inverse.
 *
 * @addtogroup PROBE
 * @{
 */

#include <stddef.h>
#include <string.h>

#include <dw1000/probe/role.h>

/*===========================================================================*/
/* The table                                                                 */
/*===========================================================================*/

/* The one place the five spellings are written down; dw1000_probe_role_name()
 * and dw1000_probe_role_lookup() are both driven off it, so the two cannot
 * drift apart from one another the way they could if each held its own
 * switch.
 */
struct role_entry {
    const char  *name;
    dw1000_probe_role_t role;
};

static const struct role_entry role_table[] = {
    { "twr_init",    DW1000_PROBE_ROLE_TWR_INIT    },
    { "twr_resp",    DW1000_PROBE_ROLE_TWR_RESP    },
    { "tx",          DW1000_PROBE_ROLE_TX          },
    { "rx",          DW1000_PROBE_ROLE_RX          },
    { "temperature", DW1000_PROBE_ROLE_TEMPERATURE },
};

#define ROLE_TABLE_COUNT (sizeof(role_table) / sizeof(role_table[0]))

/*===========================================================================*/
/* API                                                                       */
/*===========================================================================*/

const char *
dw1000_probe_role_name(dw1000_probe_role_t role)
{
    size_t i;

    for (i = 0; i < ROLE_TABLE_COUNT; i++) {
	if (role_table[i].role == role)
	    return role_table[i].name;
    }
    return "invalid";
}

bool
dw1000_probe_role_lookup(const char *name, dw1000_probe_role_t *role)
{
    size_t i;

    if (name == NULL || name[0] == '\0')
	return false;

    for (i = 0; i < ROLE_TABLE_COUNT; i++) {
	if (strcmp(role_table[i].name, name) == 0) {
	    *role = role_table[i].role;
	    return true;
	}
    }
    return false;
}

/** @} */
