/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The dissector registry and the shared-object loader, with no chip, no
 * radio and no Linux.
 *
 * dissect.c was written to be reachable from here: the interface in
 * dissect.h carries no driver type (it says why: struct capture_frame's
 * size depends on a driver compile-time option and must not cross a
 * plugin boundary), and dlopen(3) is POSIX. So the whole of both routes
 * can be exercised on whatever host the tree is checked out on, which
 * matters most for the refusals: getting an ABI check or a missing-symbol
 * report wrong is the sort of thing that otherwise surfaces on a Pi, with
 * somebody's plugin, at a bench.
 *
 * Covers, in the registry:
 *  - a well-formed dissector registers, and its name and version come
 *    back out;
 *  - an ABI that is not DISSECT_ABI is refused, and the count does not
 *    move;
 *  - so are a dissector with no name, and one with neither accept() nor
 *    describe() (which could do nothing, so registering it would put a
 *    working-looking name in --list-dissectors);
 *  - the table's limit is enforced: the ninth is refused, the first eight
 *    are not disturbed;
 *  - describe(): the first dissector to claim a frame wins and the rest
 *    are not consulted, a frame nobody claims yields 0 and leaves the
 *    output string empty, and both outcomes move the right counter;
 *  - accept(): with no accept() hook anywhere every frame is forwarded
 *    (so --dissect-filter with dissectors that do not filter is harmless
 *    rather than a capture that silently forwards nothing), one hook
 *    accepting is enough, and a frame every hook refuses is dropped and
 *    counted;
 *  - open_all() passes NULL args to a dissector registered in-process,
 *    and reports a dissector whose open() refuses;
 *  - close_all() calls every close() and empties the table.
 *
 * And in the loader:
 *  - a bare library name is refused rather than searched for on the
 *    library path (this program runs privileged on a Pi);
 *  - so is a path that does not exist;
 *  - so is a shared object that loads but exports no uwb_dissector_v1;
 *  - a real shared object loads, its describe() runs, its open() is
 *    handed the args from PATH:args, and close_all() unloads it.
 *
 * The loader cases need two shared objects, which tests/tests-sniffer.sh
 * builds and passes as argv[1]. Run without that argument, they are
 * skipped and say so, so that a host which cannot build a shared object
 * still gets the registry checked.
 *
 * Several cases here make dissect.c report a refusal on stderr. Those
 * lines are expected output, not noise from a failure;
 * tests/tests-sniffer.sh keeps them out of sight unless a case fails.
 *
 * One line per case; a failing case says why on its own line. Run by
 * tests/tests-sniffer.sh.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dissect.h"

static int failures;

static void
step(const char *what, const char *reason)
{
    if (reason == NULL) {
	printf("ok: %s\n", what);
    } else {
	printf("FAIL: %s (%s)\n", what, reason);
	failures++;
    }
}

static char reason[256];

#define REASON(...) (snprintf(reason, sizeof(reason), __VA_ARGS__), reason)


/*----------------------------------------------------------------------*/
/* Fixtures                                                              */
/*----------------------------------------------------------------------*/

/* Which hooks ran, so that a case can check that the one that should not
 * have been called was not. */
static int  called_describe_a, called_describe_b;
static int  called_accept_a, called_accept_b;
static int  called_close_a, called_close_b;
static char open_args_a[64];

static size_t
_describe_a(const struct dissect_frame *f, char *out, size_t outsz)
{
    called_describe_a++;
    /* Claims a frame whose first byte is 1, and nothing else. */
    if ((f->length == 0) || (f->data[0] != 1))
	return 0;
    return (size_t)snprintf(out, outsz, "A seq=%u", (unsigned)f->seq);
}

static size_t
_describe_b(const struct dissect_frame *f, char *out, size_t outsz)
{
    called_describe_b++;
    if ((f->length == 0) || (f->data[0] != 2))
	return 0;
    return (size_t)snprintf(out, outsz, "B seq=%u", (unsigned)f->seq);
}

static bool
_accept_a(const struct dissect_frame *f)
{
    called_accept_a++;
    return (f->length > 0) && (f->data[0] == 1);
}

static bool
_accept_b(const struct dissect_frame *f)
{
    called_accept_b++;
    return (f->length > 0) && (f->data[0] == 2);
}

static int
_open_a(const char *args)
{
    snprintf(open_args_a, sizeof(open_args_a), "%s",
	     (args != NULL) ? args : "(null)");
    return 0;
}

static int
_open_refuses(const char *args)
{
    (void)args;
    return -1;
}

static void _close_a(void) { called_close_a++; }
static void _close_b(void) { called_close_b++; }

static const struct dissector dis_a = {
    .abi = DISSECT_ABI, .name = "A", .version = "1",
    .open = _open_a, .accept = _accept_a, .describe = _describe_a,
    .close = _close_a,
};

static const struct dissector dis_b = {
    .abi = DISSECT_ABI, .name = "B", .version = "2",
    .open = NULL, .accept = _accept_b, .describe = _describe_b,
    .close = _close_b,
};

/* describe only, no accept: the shape that makes --dissect-filter a
 * no-op rather than a blackout. */
static const struct dissector dis_describe_only = {
    .abi = DISSECT_ABI, .name = "describe-only", .version = "1",
    .describe = _describe_a,
};

static const struct dissector dis_bad_abi = {
    .abi = DISSECT_ABI + 1u, .name = "wrong-abi", .describe = _describe_a,
};

static const struct dissector dis_no_name = {
    .abi = DISSECT_ABI, .name = NULL, .describe = _describe_a,
};

static const struct dissector dis_empty_name = {
    .abi = DISSECT_ABI, .name = "", .describe = _describe_a,
};

static const struct dissector dis_no_hooks = {
    .abi = DISSECT_ABI, .name = "inert", .version = "1",
};

static const struct dissector dis_open_refuses = {
    .abi = DISSECT_ABI, .name = "refuses", .describe = _describe_a,
    .open = _open_refuses,
};

/* A frame whose first byte selects which fixture claims it. */
static void
frame_init(struct dissect_frame *f, const uint8_t *data, size_t len)
{
    memset(f, 0, sizeof(*f));
    f->struct_size     = sizeof(*f);
    f->data            = data;
    f->length          = len;
    f->reported        = len;
    f->seq             = 7;
    f->power_signal    = DISSECT_POWER_NONE;
    f->power_firstpath = DISSECT_POWER_NONE;
}

static void
reset(void)
{
    dissect_close_all();
    called_describe_a = called_describe_b = 0;
    called_accept_a   = called_accept_b   = 0;
    called_close_a    = called_close_b    = 0;
    open_args_a[0]    = '\0';
}


/*----------------------------------------------------------------------*/
/* 1. Registration, and what it refuses                                  */
/*----------------------------------------------------------------------*/

static const char *
case_register(void)
{
    reset();

    if (dissect_count() != 0)
	return REASON("after close_all, count is %u, want 0", dissect_count());

    if (dissect_register(&dis_a) != 0)
	return "a well-formed dissector was refused";
    if (dissect_count() != 1)
	return REASON("count %u after one register, want 1", dissect_count());
    if ((dissect_name(0) == NULL) || (strcmp(dissect_name(0), "A") != 0))
	return REASON("name(0) is '%s', want 'A'",
		      dissect_name(0) ? dissect_name(0) : "(null)");
    if ((dissect_version(0) == NULL) ||
	(strcmp(dissect_version(0), "1") != 0))
	return "version(0) is not '1'";

    /* Out of range, rather than off the end of the table. */
    if (dissect_name(1) != NULL)
	return "name(1) is not NULL with one dissector registered";
    if (dissect_name(99) != NULL)
	return "name(99) is not NULL";

    if (dissect_register(NULL) >= 0)
	return "a NULL dissector was accepted";
    if (dissect_count() != 1)
	return "a refused registration changed the count";

    return NULL;
}

static const char *
case_register_refusals(void)
{
    reset();

    if (dissect_register(&dis_bad_abi) >= 0)
	return "a dissector with the wrong ABI was accepted";
    if (dissect_count() != 0)
	return "a wrong-ABI dissector was counted";

    if (dissect_register(&dis_no_name) >= 0)
	return "a dissector with a NULL name was accepted";
    if (dissect_register(&dis_empty_name) >= 0)
	return "a dissector with an empty name was accepted";

    if (dissect_register(&dis_no_hooks) >= 0)
	return "a dissector with neither accept() nor describe() was accepted";

    if (dissect_count() != 0)
	return REASON("count %u after four refusals, want 0",
		      dissect_count());

    return NULL;
}

static const char *
case_table_full(void)
{
    /* Eight distinct structs, because dissect_register() keeps the
     * pointer: registering one struct eight times would not fill a table
     * that holds pointers to different things in the real program. */
    static struct dissector many[9];
    unsigned i;

    reset();

    for (i = 0; i < 9; i++) {
	many[i].abi      = DISSECT_ABI;
	many[i].name     = "many";
	many[i].version  = "1";
	many[i].describe = _describe_a;
    }

    for (i = 0; i < 8; i++) {
	if (dissect_register(&many[i]) != 0)
	    return REASON("dissector %u of 8 was refused", i + 1);
    }
    if (dissect_count() != 8)
	return REASON("count %u after eight, want 8", dissect_count());

    if (dissect_register(&many[8]) >= 0)
	return "a ninth dissector was accepted";
    if (dissect_count() != 8)
	return REASON("count %u after the ninth was refused, want 8",
		      dissect_count());

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 2. describe(): first to claim wins                                    */
/*----------------------------------------------------------------------*/

static const char *
case_describe(void)
{
    const uint8_t          for_b[] = { 2, 0, 0 };
    const uint8_t          for_none[] = { 9, 9, 9 };
    struct dissect_frame   f;
    char                   out[64];
    size_t                 n;
    struct dissect_stats   before;

    reset();
    if ((dissect_register(&dis_a) != 0) || (dissect_register(&dis_b) != 0))
	return "registering the two fixtures failed";

    before = *dissect_stats();

    /* A claims nothing here, B claims it: both are asked, in order. */
    frame_init(&f, for_b, sizeof(for_b));
    n = dissect_describe(&f, out, sizeof(out));
    if (n == 0)
	return "B did not claim a frame it should have";
    if (strcmp(out, "B seq=7") != 0)
	return REASON("describe wrote '%s', want 'B seq=7'", out);
    if (n != strlen("B seq=7"))
	return REASON("describe returned %zu, want %zu", n,
		      strlen("B seq=7"));
    if (called_describe_a != 1)
	return REASON("A's describe ran %d times, want 1", called_describe_a);
    if (dissect_stats()->described != before.described + 1)
	return "a claimed frame did not move the described counter";

    /* A claims it, so B must not be asked at all. */
    called_describe_a = called_describe_b = 0;
    {
	const uint8_t for_a[] = { 1, 0, 0 };

	frame_init(&f, for_a, sizeof(for_a));
	n = dissect_describe(&f, out, sizeof(out));
	if ((n == 0) || (strcmp(out, "A seq=7") != 0))
	    return REASON("describe wrote '%s', want 'A seq=7'", out);
	if (called_describe_b != 0)
	    return "B's describe ran although A had already claimed the frame";
    }

    /* Nobody claims it: 0, an empty string, and the other counter. */
    before = *dissect_stats();
    frame_init(&f, for_none, sizeof(for_none));
    snprintf(out, sizeof(out), "not cleared");
    n = dissect_describe(&f, out, sizeof(out));
    if (n != 0)
	return REASON("describe returned %zu for an unclaimed frame", n);
    if (out[0] != '\0')
	return REASON("describe left '%s' in the buffer, want it empty", out);
    if (dissect_stats()->unrecognised != before.unrecognised + 1)
	return "an unclaimed frame did not move the unrecognised counter";

    /* No room, and no dissectors consulted. */
    called_describe_a = 0;
    if (dissect_describe(&f, out, 0) != 0)
	return "describe with outsz 0 did not return 0";
    if (dissect_describe(&f, NULL, sizeof(out)) != 0)
	return "describe with a NULL buffer did not return 0";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 3. accept(): any accepts, or nobody judges                            */
/*----------------------------------------------------------------------*/

static const char *
case_accept(void)
{
    const uint8_t        for_a[]  = { 1, 0, 0 };
    const uint8_t        none[]   = { 9, 9, 9 };
    struct dissect_frame f;
    struct dissect_stats before;

    /* (a) Nothing has an accept() hook: every frame is forwarded, and
     * the dropped counter does not move. This is the case that keeps
     * --dissect-filter from turning a capture into silence when the
     * loaded dissectors only describe. */
    reset();
    if (dissect_register(&dis_describe_only) != 0)
	return "registering the describe-only fixture failed";
    before = *dissect_stats();
    frame_init(&f, none, sizeof(none));
    if (!dissect_accept(&f))
	return "a frame was dropped although no dissector filters";
    if (dissect_stats()->dropped != before.dropped)
	return "the dropped counter moved with no accept() hook anywhere";

    /* (b) Two hooks, one of which accepts. */
    reset();
    if ((dissect_register(&dis_a) != 0) || (dissect_register(&dis_b) != 0))
	return "registering the two fixtures failed";
    frame_init(&f, for_a, sizeof(for_a));
    if (!dissect_accept(&f))
	return "a frame A accepts was dropped";
    if (called_accept_a != 1)
	return "A's accept did not run";
    if (called_accept_b != 0)
	return "B's accept ran although A had already accepted";

    /* (c) Nobody accepts: dropped, and counted. */
    before = *dissect_stats();
    called_accept_a = called_accept_b = 0;
    frame_init(&f, none, sizeof(none));
    if (dissect_accept(&f))
	return "a frame no dissector accepts was forwarded";
    if ((called_accept_a != 1) || (called_accept_b != 1))
	return REASON("accept ran %d/%d times, want 1/1",
		      called_accept_a, called_accept_b);
    if (dissect_stats()->dropped != before.dropped + 1)
	return "a dropped frame did not move the dropped counter";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 4. open_all() and close_all()                                         */
/*----------------------------------------------------------------------*/

static const char *
case_open_close(void)
{
    reset();

    if ((dissect_register(&dis_a) != 0) || (dissect_register(&dis_b) != 0))
	return "registering the two fixtures failed";

    if (dissect_open_all() != 0)
	return "open_all refused with two openable dissectors";
    /* A dissector registered in process has no PATH:args to be given, so
     * it must see NULL rather than an empty string or a stale pointer. */
    if (strcmp(open_args_a, "(null)") != 0)
	return REASON("open() saw args '%s', want NULL", open_args_a);

    if (dissect_close_all(), (dissect_count() != 0))
	return REASON("count %u after close_all, want 0", dissect_count());
    if ((called_close_a != 1) || (called_close_b != 1))
	return REASON("close ran %d/%d times, want 1/1",
		      called_close_a, called_close_b);

    /* A refusing open() is reported, and the others still run. */
    reset();
    if ((dissect_register(&dis_open_refuses) != 0) ||
	(dissect_register(&dis_a) != 0))
	return "registering the refusing fixture failed";
    if (dissect_open_all() >= 0)
	return "open_all did not report a dissector whose open() refused";
    if (strcmp(open_args_a, "(null)") != 0)
	return "a later dissector's open() was skipped after an earlier"
	       " refusal";

    /* close_all on an empty table is not an error. */
    reset();
    dissect_close_all();
    if (dissect_count() != 0)
	return "count is not 0 after close_all on an empty table";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 5. The loader's refusals                                              */
/*----------------------------------------------------------------------*/

static const char *
case_load_refusals(const char *dir)
{
    char path[512];

    reset();

    /* A name with no slash is a library name, which dlopen(3) would look
     * for on the library path. Refused: this program runs privileged on
     * a Pi, and a bare name is an invitation to load whatever a search
     * path turns up first. */
    if (dissect_load("libc.so") >= 0)
	return "a bare library name was loaded";
    if (dissect_load("dissect_plugin_ok.so") >= 0)
	return "a bare object name was loaded";
    if (dissect_count() != 0)
	return "a refused load was counted";

    if (dissect_load(NULL) >= 0)
	return "a NULL spec was accepted";
    if (dissect_load("") >= 0)
	return "an empty spec was accepted";

    snprintf(path, sizeof(path), "%s/there-is-no-such-plugin.so", dir);
    if (dissect_load(path) >= 0)
	return "a path that does not exist was loaded";

    /* Loads as a shared object, is not a dissector. */
    snprintf(path, sizeof(path), "%s/dissect_plugin_nosym.so", dir);
    if (dissect_load(path) >= 0)
	return "an object exporting no uwb_dissector_v1 was accepted";
    if (dissect_count() != 0)
	return "an object with no entry point was counted";

    return NULL;
}


/*----------------------------------------------------------------------*/
/* 6. Loading a real shared object, args and all                         */
/*----------------------------------------------------------------------*/

static const char *
case_load(const char *dir)
{
    const uint8_t        claimed[] = { 0xAA, 1, 2, 3 };
    const uint8_t        other[]   = { 0x55, 1, 2, 3 };
    struct dissect_frame f;
    char                 path[512];
    char                 out[64];
    size_t               n;

    reset();

    snprintf(path, sizeof(path), "%s/dissect_plugin_ok.so", dir);
    if (dissect_load(path) != 0)
	return REASON("loading %s failed", path);
    if (dissect_count() != 1)
	return REASON("count %u after one load, want 1", dissect_count());
    if ((dissect_name(0) == NULL) ||
	(strcmp(dissect_name(0), "plugin-ok") != 0))
	return REASON("loaded dissector is named '%s', want 'plugin-ok'",
		      dissect_name(0) ? dissect_name(0) : "(null)");

    if (dissect_open_all() != 0)
	return "the loaded plugin's open() refused";

    /* Its describe() really runs, across the dlopen boundary. */
    frame_init(&f, claimed, sizeof(claimed));
    n = dissect_describe(&f, out, sizeof(out));
    if (n == 0)
	return "the loaded plugin did not claim a frame it should have";
    if (strcmp(out, "plugin-ok len=4 seq=7") != 0)
	return REASON("plugin wrote '%s'", out);

    frame_init(&f, other, sizeof(other));
    if (dissect_describe(&f, out, sizeof(out)) != 0)
	return "the loaded plugin claimed a frame it should not have";

    /* PATH:args reaches the plugin. Loaded a second time, with args, so
     * that the same object is proven to take them. */
    reset();
    snprintf(path, sizeof(path), "%s/dissect_plugin_ok.so:hello=1",
	     dir);
    if (dissect_load(path) != 0)
	return REASON("loading with args failed: %s", path);
    if (dissect_open_all() != 0)
	return "the loaded plugin's open() refused when given args";

    /* And unloading it is clean: nothing in the table, and a second
     * close_all() does not touch a handle it has already dropped. */
    dissect_close_all();
    if (dissect_count() != 0)
	return "count is not 0 after unloading the plugin";
    dissect_close_all();

    return NULL;
}


int
main(int argc, char **argv)
{
    const char *dir = (argc > 1) ? argv[1] : NULL;

    step("register: a well-formed dissector, and its name",
					     case_register());
    step("register: bad ABI, no name, no hooks all refused",
					     case_register_refusals());
    step("register: the table's limit is enforced",
					     case_table_full());
    step("describe: first to claim wins, and the counters move",
					     case_describe());
    step("accept: any accepts, and nobody judging means forward",
					     case_accept());
    step("open_all and close_all",           case_open_close());

    if (dir != NULL) {
	step("load: bare names, missing files and non-dissectors refused",
					     case_load_refusals(dir));
	step("load: a real shared object, its args and its unloading",
					     case_load(dir));
    } else {
	printf("skip: the two loader cases (no fixture directory given)\n");
    }

    return failures == 0 ? 0 : 1;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
