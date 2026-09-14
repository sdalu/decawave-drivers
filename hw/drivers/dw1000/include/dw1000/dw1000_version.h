/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_VERSION_H__
#define __DW1000_VERSION_H__

/**
 * @file  dw1000_version.h
 * @brief Driver version
 *
 * @addtogroup DW1000
 * @{
 */

/* The release, and the one place it is written.
 *
 * A C header can read no other file, so if a consumer is to have the
 * version without a build step, the version has to be written where the
 * header can see it -- here. Everything else reads it from here:
 * dw1000.cmake parses these three lines, and the Makefile asks
 * scripts/manifest.sh, which parses them too. So `make version`,
 * DW1000_VERSION and DW1000_VERSION_STRING cannot disagree, there being
 * nothing left to disagree with.
 *
 * Bumping a release is editing these three numbers and tagging v<M>.<m>.<p>.
 */

/** Major version of the release */
#define DW1000_VERSION_MAJOR 1
/** Minor version of the release */
#define DW1000_VERSION_MINOR 3
/** Patch version of the release */
#define DW1000_VERSION_PATCH 0

/* Composed rather than spelled out, so the numbers above stay the only
 * copy. The two levels are the usual stringify dance: the inner one is
 * what expands DW1000_VERSION_MAJOR before # freezes it.
 */
#define __DW1000_VERSION_STR(x)  #x
#define __DW1000_VERSION_XSTR(x) __DW1000_VERSION_STR(x)

/** The release as a string, @c "1.2.3" */
#define DW1000_VERSION_STRING						\
    __DW1000_VERSION_XSTR(DW1000_VERSION_MAJOR) "."			\
    __DW1000_VERSION_XSTR(DW1000_VERSION_MINOR) "."			\
    __DW1000_VERSION_XSTR(DW1000_VERSION_PATCH)

/**
 * The release as one comparable integer, for @c \#if -- 1.2.3 is 10203.
 * Each field is given two digits, so a field never reaches the next one
 * (1.1.0 is 10100, well under 1.2.0's 10200).
 */
#define DW1000_VERSION_NUMBER						\
    (DW1000_VERSION_MAJOR * 10000 +					\
     DW1000_VERSION_MINOR * 100 +					\
     DW1000_VERSION_PATCH)

/**
 * Whether this driver is release @p maj.@p min.@p pat or newer, for
 * conditional compilation:
 *
 * @code
 * #if !DW1000_VERSION_AT_LEAST(1, 1, 0)
 * #error this application needs dw1000 1.1.0 or newer
 * #endif
 * @endcode
 */
#define DW1000_VERSION_AT_LEAST(maj, min, pat)				\
    (DW1000_VERSION_NUMBER >= ((maj) * 10000 + (min) * 100 + (pat)))

/**
 * What a build between releases adds to the version, and nothing (@c "")
 * for a release or wherever it could not be known.
 *
 * Passed on the compiler command line -- this tree's Makefile does it
 * from @c scripts/gitversion.sh, and @c make @c sources hands a vendoring
 * build the same answer as @c DW1000_VERSION_GIT for it to pass on if it
 * wants to. A tree built from a tarball, or vendored into somebody else's
 * repository, leaves it empty rather than reporting that repository's git
 * state as the driver's.
 *
 * The shape is SemVer build metadata: @c "+58.g3403fe0" is fifty-eight
 * commits past the release tag at that commit, and @c ".dirty" is
 * appended when the worktree had uncommitted changes.
 *
 * Unlike the @c DW1000_WITH_* options this changes no structure and no
 * entry point, so it is the one define that need not reach every
 * translation unit -- passing it to some and not others is harmless.
 */
#ifndef DW1000_VERSION_GIT
#define DW1000_VERSION_GIT ""
#endif

/**
 * The version this build actually is: the release, plus the commit when
 * it was built between releases -- @c "1.1.0+58.g3403fe0".
 *
 * A string and not a function on purpose. There is no installed library
 * here to have been replaced underneath you: the driver is vendored, so
 * its headers and its sources are compiled together out of one tree, and
 * a call would cost an MCU a symbol and a string it may not want. Log it
 * where you log your own firmware version.
 */
#define DW1000_VERSION_FULL DW1000_VERSION_STRING DW1000_VERSION_GIT

/** @} */

#endif
