/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The dissector registry, and the loader for the shared-object route.
 *
 * Both routes end in the same place: a fixed table of pointers to
 * `struct dissector`, walked by the forwarding loop. What differs is who
 * fills it. A tree named by `DISSECTORS=` at build time provides one
 * function that calls dissect_register() (main.c calls it through the
 * SNIFFER_DISSECT_INIT macro build.sh defines), and `--dissector=PATH`
 * dlopen()s an object and asks it for the same struct.
 *
 * This file touches neither the chip nor Linux: dlopen(3) is POSIX and
 * the interface in dissect.h carries no driver type, deliberately (the
 * header says why). So it joins capture.c, wire.c and pcapng.c in the
 * part of this program that tests/tests-sniffer.sh can build and run on
 * any host, which is where the loader most wants to be tested: getting
 * an ABI refusal wrong is the kind of mistake that otherwise surfaces
 * only on a Pi, with a plugin, at a bench.
 *
 * Everything here runs on the forwarding loop's thread. Nothing in it is
 * reachable from the receive callback, and that is not an accident: a
 * dissector is exactly the sort of unbounded work that must not sit
 * inside rx_ok while the next frame lands in the other buffer. capture.c
 * carries the full version of that argument.
 */

#include <dlfcn.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "dissect.h"

/* Eight is not a measured number: it is "more than anyone has wanted",
 * and a fixed table so that registration cannot fail for want of memory
 * at a moment when there is nothing useful to do about it. A ninth
 * dissector is refused with a message rather than silently dropped. */
#define DISSECT_MAX	8u

struct entry {
    const struct dissector *d;
    void                   *handle;	/* dlopen handle, NULL if linked in */
    const char             *args;	/* from PATH:args, NULL if none     */
    char                    argbuf[128];
};

static struct entry        table[DISSECT_MAX];
static unsigned            count;
static struct dissect_stats stats;

/* Reported through this file rather than config.h's WARN, so that
 * dissect.c stays free of everything in config.h (which reaches bitters,
 * and so Linux). The test links this file without config.h at all. */
static void
_warn(const char *fmt, ...)
{
    va_list ap;

    fputs("dissect: ", stderr);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

int
dissect_register(const struct dissector *d)
{
    if (d == NULL)
	return -EINVAL;

    /* Checked, not assumed, and named in the message: a plugin built
     * against another revision of dissect.h is the failure this number
     * exists to catch, and "refused" with both numbers is the only
     * report that tells the author what to do about it. */
    if (d->abi != DISSECT_ABI) {
	_warn("refusing '%s': ABI %u, this sniffer speaks %u",
	      d->name ? d->name : "(unnamed)",
	      (unsigned)d->abi, (unsigned)DISSECT_ABI);
	return -EPROTO;
    }

    if ((d->name == NULL) || (d->name[0] == '\0')) {
	_warn("refusing a dissector with no name");
	return -EINVAL;
    }

    /* A dissector with neither hook cannot do anything. Registering it
     * would put a name in --list-dissectors that never runs, which reads
     * as "installed and working". */
    if ((d->accept == NULL) && (d->describe == NULL)) {
	_warn("refusing '%s': it has neither accept() nor describe()",
	      d->name);
	return -EINVAL;
    }

    if (count >= DISSECT_MAX) {
	_warn("refusing '%s': already holding %u dissectors",
	      d->name, DISSECT_MAX);
	return -ENOSPC;
    }

    table[count].d      = d;
    table[count].handle = NULL;
    table[count].args   = NULL;
    count++;

    return 0;
}

int
dissect_load(const char *spec)
{
    char              path[512];
    const char       *args = NULL;
    const char       *colon;
    void             *handle;
    dissect_plugin_fn entry;
    const struct dissector *d;
    unsigned          slot;

    if ((spec == NULL) || (spec[0] == '\0'))
	return -EINVAL;

    if (count >= DISSECT_MAX) {
	_warn("cannot load '%s': already holding %u dissectors",
	      spec, DISSECT_MAX);
	return -ENOSPC;
    }

    /* PATH:args, split at the LAST colon, so that an absolute path on a
     * system that permits colons in file names still works when no args
     * are given. A path that really contains a colon and wants args is
     * not supported, and says so rather than guessing. */
    colon = strrchr(spec, ':');
    if (colon != NULL) {
	size_t plen = (size_t)(colon - spec);
	if (plen >= sizeof(path)) {
	    _warn("cannot load: path longer than %zu bytes", sizeof(path) - 1);
	    return -ENAMETOOLONG;
	}
	memcpy(path, spec, plen);
	path[plen] = '\0';
	args = colon + 1;
    } else {
	if (strlen(spec) >= sizeof(path)) {
	    _warn("cannot load: path longer than %zu bytes", sizeof(path) - 1);
	    return -ENAMETOOLONG;
	}
	strcpy(path, spec);
    }

    /* A name with no slash in it is not a path: dlopen(3) searches the
     * library path for it. That is surprising when you have just built
     * ./thing.so and asked for "thing.so", and it is worse than
     * surprising here, because this program runs privileged on a Pi
     * (SCHED_FIFO and mlockall both need it), so a bare name is an
     * invitation to load whatever a search path turns up first. Refused,
     * with the fix in the message.
     */
    if (strchr(path, '/') == NULL) {
	_warn("'%s' is a library name, not a path: dlopen(3) would search"
	      " the library path for it. Give a path, as in ./%s", path, path);
	return -EINVAL;
    }

    /* RTLD_NOW so that an unresolved symbol is a load failure here,
     * before the receiver is armed, rather than a crash on the first
     * frame. RTLD_LOCAL because nothing else should see the plugin's
     * symbols: the interface is one way, the sniffer calls the dissector,
     * which is also why the executable is not linked -rdynamic. */
    handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (handle == NULL) {
	/* -ENOEXEC for every "this file is not a usable dissector" case
	 * below as well. The errno value only tells main.c that the load
	 * failed; dlerror()'s text above is the actual diagnosis, and
	 * ELIBACC, which says this more precisely, is a Linux extension
	 * and would cost this file its portability for a number nobody
	 * reads. */
	_warn("cannot load '%s': %s", path, dlerror());
	return -ENOEXEC;
    }

    /* Cleared first: dlerror() is sticky, and a NULL from dlsym() is a
     * legitimate value for a symbol, so the error string is the only way
     * to tell "absent" from "present and null". */
    (void)dlerror();
    entry = (dissect_plugin_fn)(uintptr_t)dlsym(handle, DISSECT_PLUGIN_SYMBOL);
    if (entry == NULL) {
	_warn("'%s' exports no %s: not a dissector for this sniffer, or one"
	      " built against another interface version",
	      path, DISSECT_PLUGIN_SYMBOL);
	dlclose(handle);
	return -ENOEXEC;
    }

    d = entry();
    if (d == NULL) {
	_warn("'%s': %s returned nothing", path, DISSECT_PLUGIN_SYMBOL);
	dlclose(handle);
	return -ENOEXEC;
    }

    slot = count;
    if (dissect_register(d) < 0) {
	dlclose(handle);
	return -EPROTO;
    }

    /* The handle goes on the entry dissect_register() just made, so that
     * dissect_close_all() unloads exactly what was loaded. Copied rather
     * than pointed at: `spec` is argv memory in the program but need not
     * be in a test. */
    table[slot].handle = handle;
    if (args != NULL) {
	snprintf(table[slot].argbuf, sizeof(table[slot].argbuf), "%s", args);
	table[slot].args = table[slot].argbuf;
    }

    return 0;
}

int
dissect_open_all(void)
{
    int rc = 0;

    for (unsigned i = 0; i < count; i++) {
	if (table[i].d->open == NULL)
	    continue;
	if (table[i].d->open(table[i].args) < 0) {
	    _warn("'%s' refused to open", table[i].d->name);
	    rc = -1;
	}
    }

    return rc;
}

void
dissect_close_all(void)
{
    for (unsigned i = 0; i < count; i++) {
	if (table[i].d->close != NULL)
	    table[i].d->close();
    }

    /* Unloaded in a second pass, after every close() has run: a dlclose()
     * in the first pass would unmap one plugin's code while a later
     * entry's close() had not been called yet, and the table holds
     * pointers into both. */
    for (unsigned i = 0; i < count; i++) {
	if (table[i].handle != NULL)
	    dlclose(table[i].handle);
    }

    memset(table, 0, sizeof(table));
    count = 0;
}

unsigned
dissect_count(void)
{
    return count;
}

const char *
dissect_name(unsigned i)
{
    return (i < count) ? table[i].d->name : NULL;
}

const char *
dissect_version(unsigned i)
{
    if (i >= count)
	return NULL;
    return table[i].d->version ? table[i].d->version : "";
}

bool
dissect_accept(const struct dissect_frame *frame)
{
    bool asked = false;

    for (unsigned i = 0; i < count; i++) {
	if (table[i].d->accept == NULL)
	    continue;
	asked = true;
	if (table[i].d->accept(frame))
	    return true;
    }

    /* Nobody was in a position to judge, so the frame goes out. Turning
     * --dissect-filter on with dissectors that do not filter is then
     * harmless, rather than a capture that silently forwards nothing. */
    if (!asked)
	return true;

    stats.dropped++;
    return false;
}

size_t
dissect_describe(const struct dissect_frame *frame, char *out, size_t outsz)
{
    if ((out == NULL) || (outsz == 0))
	return 0;

    out[0] = '\0';

    /* First to claim it wins, and the rest are not called. Frames are
     * one protocol at a time and this runs once per frame forwarded, so
     * gathering every dissector's opinion would cost real time to
     * produce a line nobody asked for. */
    for (unsigned i = 0; i < count; i++) {
	size_t n;

	if (table[i].d->describe == NULL)
	    continue;

	n = table[i].d->describe(frame, out, outsz);
	if (n > 0) {
	    stats.described++;
	    return n;
	}
    }

    stats.unrecognised++;
    out[0] = '\0';
    return 0;
}

const struct dissect_stats *
dissect_stats(void)
{
    return &stats;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
