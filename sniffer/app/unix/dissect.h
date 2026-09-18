/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DISSECT__H
#define __DISSECT__H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * The dissector interface: what a frame looks like to code that did not
 * come with this program.
 *
 * Two things can supply a dissector. A tree named by `DISSECTORS=` at
 * build time is compiled and linked straight in (build.sh reads its
 * manifest the way it already reads the driver's and bitters'), and a
 * shared object named by `--dissector=PATH` is loaded at start-up. The
 * interface below is the same either way; only how the registration
 * happens differs.
 *
 * NOTHING IN THIS HEADER MAY MENTION A DRIVER TYPE. That is not tidiness,
 * it is the whole reason the interface looks like this. `struct
 * capture_frame` embeds `uint8_t data[DW1000_FRAME_MAXSIZE]`, which is
 * 127 bytes or 1023 depending on DW1000_WITH_PROPRIETARY_LONG_FRAME, so
 * handing one across a dlopen boundary would mean a plugin built with a
 * different option set reading a struct of a different shape, with no
 * symptom until the numbers came out wrong. The root README refuses to
 * ship the driver as a shared library for exactly this reason. So a
 * dissector sees a flat struct of fixed-width fields, a pointer and a
 * length, and can be compiled in complete ignorance of the driver's
 * options.
 *
 * A dissector may not change anything. The frame is const, and nothing
 * here lets one alter what is forwarded or what is written; the most it
 * can do is say "do not forward this one" through accept(). A capture
 * tool whose plugins can rewrite the capture is not a capture tool.
 */

/** Bumped when anything below changes shape. Checked, not assumed. */
#define DISSECT_ABI		1

/** @name Per-frame flags, the subset of capture.h's that a dissector needs
 * @{ */
#define DISSECT_F_TRUNCATED	(1u << 0)
#define DISSECT_F_RANGING	(1u << 2)
/** @} */

/** No usable power estimate. Same value capture.h uses, restated here so
 *  that a plugin needs no driver header to test for it. */
#define DISSECT_POWER_NONE	INT32_MIN

/**
 * One captured frame, as a dissector sees it.
 *
 * @p data is @p length bytes and no more. @p reported is what the driver
 * said the frame was, which is larger exactly when the capture had to
 * truncate it (@p DISSECT_F_TRUNCATED): a dissector that walks a
 * variable-length payload MUST bound itself by @p length, and should not
 * report a clean parse of a frame carrying that flag. The CRC is
 * included in both, as it is everywhere else in this program.
 *
 * @p struct_size is what the caller thinks this struct is. It is here so
 * that a plugin built against an older header, handed a longer struct by
 * a newer sniffer, can notice rather than read a field that has moved.
 * Check it if you read anything past @p flags.
 */
struct dissect_frame {
    size_t          struct_size;
    const uint8_t  *data;
    size_t          length;
    size_t          reported;
    uint32_t        seq;
    uint32_t        flags;
    /* The radio's account of the frame, valid only when the capture was
     * run with the metadata read-out on; rx_time is 0 and both powers are
     * DISSECT_POWER_NONE when it was not. */
    uint64_t        rx_time;
    int32_t         clock_offset;
    uint32_t        clock_interval;
    int32_t         power_signal;
    int32_t         power_firstpath;
    uint16_t        first_path;
    uint16_t        std_noise;
    uint16_t        max_noise;
};

/**
 * What a dissector is.
 *
 * @p abi must be @p DISSECT_ABI or the dissector is refused with a
 * message naming both numbers. @p name is what `--list-dissectors`
 * prints and must not be NULL. Every function pointer is optional except
 * that a dissector with neither @p accept nor @p describe does nothing
 * and is refused, since silently registering a dissector that cannot act
 * is worse than saying so.
 *
 * All of these run on the forwarding loop's thread, never in the receive
 * callback, and never concurrently with each other.
 */
struct dissector {
    uint32_t    abi;
    const char *name;
    const char *version;

    /**
     * Called once before the receiver is armed. @p args is whatever
     * followed a colon in `--dissector=PATH:args`, or NULL.
     * @return 0 to accept, negative to refuse registration
     */
    int  (*open)(const char *args);

    /**
     * Should this frame be forwarded? Only consulted when the capture was
     * started with `--dissect-filter`; without it every frame is
     * forwarded and this is never called.
     */
    bool (*accept)(const struct dissect_frame *frame);

    /**
     * One line of human-readable text about the frame, for `-v` and for
     * the pcapng packet comment. Write at most @p outsz bytes including
     * the terminator.
     * @return the length written, or 0 to say "this is not my frame"
     */
    size_t (*describe)(const struct dissect_frame *frame,
		       char *out, size_t outsz);

    /** Called once on the way out, after the loop has stopped. */
    void (*close)(void);
};

/**
 * The symbol a `--dissector=PATH` shared object must export.
 *
 * Versioned in the NAME rather than only in the struct: a plugin built
 * against an incompatible interface then fails to load, instead of
 * loading and being misread.
 */
#define DISSECT_PLUGIN_SYMBOL	"uwb_dissector_v1"
typedef const struct dissector *(*dissect_plugin_fn)(void);

/**
 * Put this on the entry point's definition.
 *
 * A plugin wants to be built with -fvisibility=hidden, so that the one
 * symbol it exports is the entry point and two plugins cannot collide
 * over a helper they happen to have named alike. That switch hides the
 * entry point too unless it is marked, and a plugin whose entry point is
 * hidden loads without error and is then refused for "exporting no
 * uwb_dissector_v1", which is a confusing way to find out. So the macro
 * lives here rather than in each plugin's author's memory.
 */
#if defined(__GNUC__) || defined(__clang__)
#define DISSECT_EXPORT	__attribute__((visibility("default")))
#else
#define DISSECT_EXPORT
#endif

/**
 * What the dissectors did, for the reception account.
 */
struct dissect_stats {
    unsigned long described;	/**< frames some dissector recognised   */
    unsigned long unrecognised;	/**< frames no dissector claimed        */
    unsigned long dropped;	/**< frames accept() refused to forward */
};


/**
 * Register a dissector. The compile-time route's entry point: a tree
 * named by `DISSECTORS=` exports one function that calls this once per
 * dissector it provides.
 *
 * @p d is kept by reference and must outlive the program.
 *
 * @return 0 registered, negative refused (bad ABI, no hooks, table full)
 */
int dissect_register(const struct dissector *d);

/**
 * Load a dissector from a shared object.
 *
 * @param[in] spec	PATH, or PATH:args
 * @return 0 loaded, negative on failure (the reason is reported)
 */
int dissect_load(const char *spec);

/**
 * Run every registered dissector's open(). MUST be called once, after
 * all registration and before the receiver is armed.
 *
 * @return 0 if all opened, negative if any refused
 */
int dissect_open_all(void);

/** Run every registered dissector's close(), and unload what was dlopen'd. */
void dissect_close_all(void);

/** How many dissectors are registered, and the name of one. */
unsigned dissect_count(void);
const char *dissect_name(unsigned i);
const char *dissect_version(unsigned i);

/**
 * Should this frame be forwarded?
 *
 * True when no registered dissector has an @p accept hook, so turning
 * filtering on with dissectors that do not filter is harmless rather
 * than silent death. A frame is forwarded if ANY dissector accepts it.
 */
bool dissect_accept(const struct dissect_frame *frame);

/**
 * First dissector that claims the frame writes its line into @p out.
 *
 * @return the length written, or 0 if nothing claimed it
 */
size_t dissect_describe(const struct dissect_frame *frame,
			char *out, size_t outsz);

/** The counters, for reporting. Never NULL. */
const struct dissect_stats *dissect_stats(void);

#endif

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
