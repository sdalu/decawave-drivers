#!/bin/sh
# build.sh: compile the Linux/Raspberry Pi UWB sniffer
# usage: build.sh [-n] [-o outfile]
#
#   -n   print the compiler command and exit; compile nothing
#   -o   executable to write (default: build/uwb-sniffer)
#
# Modelled on probe/app/unix/build.sh, which is modelled in turn on
# spank/simulation/build.sh and rpi-redskin/build.sh: it reads
# `make -s sources` from every vendored tree and evals the result rather
# than naming files, so a source this application did not add itself is
# never missed and never hand-copied out of step with the manifest that
# actually describes it. The build.sh this replaces globbed
# hw/drivers/dw1000/src/*.c, which now silently takes dw1000_state.c,
# a file the driver holds back on purpose (see dw1000.cmake's note on
# DW1000_SOURCES_STATE), and had had its bitters half narrowed from
# src/*.c to four named files by hand, which is the maintenance this
# removes rather than repeats: the narrowing is the manifest's job, and
# a hand-kept list is wrong again as soon as either tree adds a file.
#
# Unlike the probe there is nothing here to ask the manifest for: the
# sniffer is an application and nothing else, so it exports no sources,
# has no port layer, and appears nowhere in dw1000.cmake. Its eight
# translation units all sit in this directory. Four of them (capture.c,
# wire.c, pcapng.c and dissect.c) touch neither the chip nor Linux, which
# is what lets tests/check-sniffer.sh compile and run them on any host;
# the other four build only for the Pi.
#
# A ninth set of sources can come from elsewhere: see DISSECTORS= below.
#
# Environment, each overridable:
#   DW1000     this tree (decawave-drivers); default: three directories
#              above this script, i.e. the tree this script itself lives
#              in; set it only to point at a *different* checkout
#   BITTERS    the bitters tree; default $HOME/Repos/bitters
#   DISSECTORS a tree of frame dissectors to compile in; default none.
#              It must export `make -s sources` the way this tree and
#              bitters do, setting DISSECT_SOURCES, DISSECT_INIT (the
#              name of its registration function) and optionally
#              DISSECT_CFLAGS, DISSECT_LIBS and DISSECT_VERSION.
#              sniffer/dissectors/ieee802154 is a worked example and the
#              interface's reference; sniffer/DESIGN.md says why the
#              dissector interface is shaped as it is.
#   CC         compiler binary                                  (cc)
#   CCEXTRA    flags that particular binary needs
#   OPTFLAGS   optimisation and debug flags                  (-O2 -g)
#
# Exit: 0 built, 1 build failed, 2 usage error.
set -eu

progname=${0##*/}
die()   { printf '%s: %s\n' "$progname" "$*" >&2; exit 1; }
usage() { printf 'usage: %s [-n] [-o outfile]\n' "$progname" >&2; exit 2; }

# This script lives in $top/sniffer/app/unix; three levels up is the tree.
appdir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd) || exit 1
top=$(CDPATH='' cd -- "$appdir/../../.." && pwd)      || exit 1

: "${DW1000:=$top}"
: "${BITTERS:=$HOME/Repos/bitters}"
: "${CC:=cc}"
: "${CCEXTRA:=}"
: "${DISSECTORS:=}"

# -O2, not the -O0 this used to carry. Between the SPI read-out in the
# rx_ok callback and the sendmsg(2) that follows it there is nothing else
# running, and how much of the radio's frame rate the host can keep up
# with is the one number this program is judged on: shipping it
# unoptimised spends that for nothing. -g stays, because a sniffer that
# stops sniffing on a bench is debugged where it stands. Overridable, so
# that `OPTFLAGS='-O0 -g' sh build.sh` still gives a build to step
# through.
: "${OPTFLAGS:=-O2 -g}"

dryrun=0 out=build/uwb-sniffer
while getopts no: opt; do
    case $opt in
    n)  dryrun=1      ;;
    o)  out=$OPTARG   ;;
    *)  usage         ;;
    esac
done
shift $((OPTIND - 1))
[ $# -eq 0 ] || usage

command -v "$CC" >/dev/null || die "no such compiler: $CC"
command -v make  >/dev/null || die "make(1) is needed to read the manifests"
[ -f "$DW1000/dw1000.cmake" ] \
    || die "no dw1000.cmake under $DW1000; wrong DW1000= ?"
[ -d "$BITTERS" ] \
    || die "no bitters tree at $BITTERS; set BITTERS="
[ -z "$DISSECTORS" ] || [ -d "$DISSECTORS" ] \
    || die "no dissector tree at $DISSECTORS; wrong DISSECTORS= ?"

# The driver core and its unix OSAL, from decawave-drivers' own manifest
# interface (dw1000.cmake, read through `make sources`). Same call
# probe/app/unix/build.sh and rpi-redskin/build.sh both make.
dw1000_vars=$(make -s -C "$DW1000" sources OSAL=unix) \
    || die "cannot read the source manifest from $DW1000"
eval "$dw1000_vars"

# bitters, taken by subsystem: this application uses gpio, spi and delay
# (core is always required), not i2c. Same choice probe/app/unix/build.sh
# and rpi-redskin/build.sh make, for the same reason: which subsystems a
# program takes is its own choice, not the module's, and asking the
# manifest for them by name is what keeps that choice from going stale,
# which the hand-narrowed list this replaces could not.
bitters_vars=$(make -s -C "$BITTERS" sources) \
    || die "cannot read the source manifest from $BITTERS"
eval "$bitters_vars"
BITTERS_TAKEN="$BITTERS_SOURCES_CORE $BITTERS_SOURCES_DELAY \
               $BITTERS_SOURCES_GPIO $BITTERS_SOURCES_SPI"

# A tree of dissectors, if one was named. Read through the same
# `make -s sources` interface as the driver and bitters, for the same
# reason: the tree that owns the files is the tree that lists them. What
# this one must export beyond its sources is DISSECT_INIT, the name of
# the function that calls dissect_register() once per dissector; passing
# the NAME through to the compiler is what keeps build.sh, and main.c,
# from knowing anything protocol-specific.
DISSECT_SOURCES= DISSECT_CFLAGS= DISSECT_LIBS= DISSECT_INIT= DISSECT_VERSION=
if [ -n "$DISSECTORS" ]; then
    dissect_vars=$(make -s -C "$DISSECTORS" sources) \
        || die "cannot read the source manifest from $DISSECTORS"
    eval "$dissect_vars"
    [ -n "$DISSECT_SOURCES" ] \
        || die "$DISSECTORS exports no DISSECT_SOURCES"
    [ -n "$DISSECT_INIT" ] \
        || die "$DISSECTORS exports no DISSECT_INIT (the registration symbol)"
fi

mkdir -p -- "$(dirname -- "$out")" || die "cannot create $(dirname -- "$out")"

# One argument list, built up and then handed to the compiler, so -n can
# print exactly what would run. Deliberately unquoted below: CCEXTRA and
# every *_SOURCES/*_CFLAGS/*_LIBS variable is a list, and splitting it
# into separate arguments is the point.
# shellcheck disable=SC2086
{
    set -- $CCEXTRA -std=c17 -D_GNU_SOURCE
    set -- "$@" $OPTFLAGS -Wall -Wextra -pthread

    set -- "$@" -I "$appdir"
    set -- "$@" $DW1000_CFLAGS
    set -- "$@" $BITTERS_CFLAGS

    # The compile-time options this program's radio configuration
    # (main.c's dw1000_radio) assumes: the driver's own defaults,
    # spelled out rather than left implicit, exactly as
    # probe/app/unix/build.sh and rpi-redskin/build.sh do, and the same
    # four the build.sh this replaces set.
    set -- "$@" -DDW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH=1
    set -- "$@" -DDW1000_WITH_PROPRIETARY_SFD=1
    set -- "$@" -DDW1000_WITH_PROPRIETARY_LONG_FRAME=0
    set -- "$@" -DDW1000_WITH_SFD_TIMEOUT=0

    # Kept from the build.sh this replaces, and deliberately more than
    # the probe asks for: the asserts check every SPI transfer and GPIO
    # call this program makes against the shapes bitters documents, and
    # SILENCE_RPI_WARNING suppresses the runtime complaint bitters makes
    # when it finds itself somewhere that is not a Raspberry Pi, which
    # is a warning worth having on a bench and not worth having here.
    set -- "$@" -DBITTERS_WITH_GPIO -DBITTERS_WITH_SPI
    set -- "$@" -DBITTERS_WITH_GPIO_IRQ -DBITTERS_WITH_THREADS
    set -- "$@" -DBITTERS_SPI_WITH_ASSERT -DBITTERS_GPIO_WITH_ASSERT
    set -- "$@" -DBITTERS_SILENCE_RPI_WARNING

    # cmdline.c prints this for -V. It used to be the sniffer's own git
    # description, from the repository it had to itself; now that it
    # lives here, what it honestly reports is the driver it was built
    # against: release plus whatever a between-releases build adds.
    set -- "$@" "-DVERSION=\"$DW1000_VERSION$DW1000_VERSION_GIT\""
    set -- "$@" "-DDW1000_VERSION_GIT=\"$DW1000_VERSION_GIT\""
    set -- "$@" "-DBITTERS_VERSION_GIT=\"$BITTERS_VERSION_GIT\""

    set -- "$@" "$appdir/main.c"     "$appdir/cmdline.c"
    set -- "$@" "$appdir/eth.c"      "$appdir/capture.c"
    set -- "$@" "$appdir/wire.c"     "$appdir/pcapng.c"
    set -- "$@" "$appdir/dissect.c"  "$appdir/uwb_dw1000.c"

    # The compile-time dissector route. SNIFFER_DISSECT_INIT is the name
    # of the registration function, which main.c declares extern and
    # calls under #ifdef; with no DISSECTORS= the macro is never defined
    # and that code is not compiled at all.
    if [ -n "$DISSECTORS" ]; then
	set -- "$@" $DISSECT_CFLAGS
	set -- "$@" "-DSNIFFER_DISSECT_INIT=$DISSECT_INIT"
	set -- "$@" $DISSECT_SOURCES
    fi

    # DW1000_SOURCES_CORE without _SEND: the sniffer never transmits, and
    # dw1000.c never calls into dw1000_send.c, so the send half is left
    # out: the receive-only case dw1000.cmake documents. The build.sh
    # this replaces took both, because a glob cannot tell them apart.
    set -- "$@" $DW1000_SOURCES_CORE $DW1000_OSAL_SOURCES

    # DW1000_SOURCES_VALIDATE: what turns "--channel 5" into the field
    # dw1000_radio_t wants, with the message when it will not. This program
    # used to carry its own copy of that table, and the copy was wrong in
    # three places.
    #
    # It is part of DW1000_SOURCES, so most consumers get it without
    # asking, but DW1000_SOURCES carries _SEND too, and the line above
    # takes _CORE precisely to leave that out. So it is named here.
    set -- "$@" $DW1000_SOURCES_VALIDATE
    set -- "$@" $BITTERS_TAKEN

    set -- "$@" $DW1000_LIBS $BITTERS_LIBS $DISSECT_LIBS -lpopt -lm

    # dissect.c calls dlopen(3) for the --dissector route. Harmless where
    # it is part of libc already (glibc 2.34 and later, and the BSDs);
    # needed on anything older.
    set -- "$@" -ldl
    set -- "$@" -o "$out"
}

if [ "$dryrun" -eq 1 ]; then
    printf '%s' "$CC"
    for arg do printf ' %s' "$arg"; done
    printf '\n'
    exit 0
fi

rm -f -- "$out"

if [ -n "$DISSECTORS" ]; then
    printf '%s: dissectors from %s (%s), registered by %s()\n' \
        "$progname" "$DISSECTORS" "${DISSECT_VERSION:-no version}" \
        "$DISSECT_INIT" >&2
fi

printf '%s: building %s (dw1000 %s%s, bitters %s%s, %s)\n' \
    "$progname" "$out" \
    "$DW1000_VERSION" "$DW1000_VERSION_GIT" \
    "$BITTERS_VERSION" "$BITTERS_VERSION_GIT" "$CC" >&2
"$CC" "$@" || die "build failed"
printf '%s: built %s\n' "$progname" "$out" >&2
