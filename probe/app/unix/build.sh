#!/bin/sh
# build.sh -- compile the Linux/Raspberry Pi probe application
# usage: build.sh [-n] [-o outfile]
#
#   -n   print the compiler command and exit; compile nothing
#   -o   executable to write (default: build/probe)
#
# Modelled on spank/simulation/build.sh: it reads `make -s sources` from
# every vendored tree and evals the result rather than naming files, so
# a source this application did not add itself is never missed and never
# hand-copied out of step with the manifest that actually describes it.
# rpi-redskin/build.sh is the second half of the model -- it is the
# working example of doing this for bitters AND decawave-drivers
# together, in one gcc invocation, for exactly this hardware.
#
# `make sources` does not know about the probe -- it is decawave-drivers'
# own addition, sitting beside the driver rather than folded into it (see
# dw1000.cmake's own note on why) -- so its three pieces (the probe API,
# probe/src, and this port) are asked for directly from scripts/
# manifest.sh, the same way tests/check-probe.sh already does.
#
# Environment, each overridable:
#   DW1000     this tree (decawave-drivers); default: three directories
#              above this script, i.e. the tree this script itself lives
#              in -- set it only to point at a *different* checkout
#   BITTERS    the bitters tree; default $HOME/Repos/bitters
#   CC         compiler binary                                  (cc)
#   CCEXTRA    flags that particular binary needs
#
# Exit: 0 built, 1 build failed, 2 usage error.
set -eu

progname=${0##*/}
die()   { printf '%s: %s\n' "$progname" "$*" >&2; exit 1; }
usage() { printf 'usage: %s [-n] [-o outfile]\n' "$progname" >&2; exit 2; }

# This script lives in $top/probe/app/unix; three levels up is the tree.
appdir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd) || exit 1
top=$(CDPATH='' cd -- "$appdir/../../.." && pwd)      || exit 1

: "${DW1000:=$top}"
: "${BITTERS:=$HOME/Repos/bitters}"
: "${CC:=cc}"
: "${CCEXTRA:=}"

dryrun=0 out=build/probe
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
[ -d "$DW1000/probe/port/unix" ] \
    || die "no probe/port/unix under $DW1000 -- wrong DW1000= ?"
[ -d "$BITTERS" ] \
    || die "no bitters tree at $BITTERS -- set BITTERS="

# The driver core and its unix OSAL, from decawave-drivers' own manifest
# interface (dw1000.cmake, read through `make sources`). Same call
# tests/check-probe.sh and rpi-redskin/build.sh both make.
dw1000_vars=$(make -s -C "$DW1000" sources OSAL=unix) \
    || die "cannot read the source manifest from $DW1000"
eval "$dw1000_vars"

# bitters, taken by subsystem: this application uses gpio, spi and delay
# (core is always required), not i2c -- same choice rpi-redskin/build.sh
# makes, for the same reason: which subsystems a program takes is its own
# choice, not the module's.
bitters_vars=$(make -s -C "$BITTERS" sources) \
    || die "cannot read the source manifest from $BITTERS"
eval "$bitters_vars"
BITTERS_TAKEN="$BITTERS_SOURCES_CORE $BITTERS_SOURCES_DELAY \
               $BITTERS_SOURCES_GPIO $BITTERS_SOURCES_SPI"

# The probe: its own API, probe/src (role, record, exchange.c -- the
# state machine this application drives), and this port. Asked for
# directly, the way tests/check-probe.sh does, because `make sources`
# above is the driver's own interface and does not carry these.
m="sh $DW1000/scripts/manifest.sh"
# Prefixed like every source below: manifest.sh answers relative to the
# tree top, and this script does not run from there.
PROBE_INCLUDE=$DW1000/$($m probeincdir)
PROBE_SOURCES=
for f in $($m probesrc) $($m probeportsrc unix); do
    PROBE_SOURCES="$PROBE_SOURCES $DW1000/$f"
done

mkdir -p -- "$(dirname -- "$out")" || die "cannot create $(dirname -- "$out")"

# One argument list, built up and then handed to the compiler, so -n can
# print exactly what would run. Deliberately unquoted below: CCEXTRA and
# every *_SOURCES/*_CFLAGS/*_LIBS variable is a list, and splitting it
# into separate arguments is the point (see spank/simulation/build.sh's
# own note on the same thing).
# shellcheck disable=SC2086
{
    set -- $CCEXTRA -std=c17 -D_GNU_SOURCE
    set -- "$@" -g -O0 -Wall -Wextra -pthread

    set -- "$@" -I "$appdir"
    set -- "$@" -I "$PROBE_INCLUDE"
    set -- "$@" $DW1000_CFLAGS
    set -- "$@" $BITTERS_CFLAGS

    # The compile-time options this instrument's radio configuration
    # (probe/app/unix/main.c's probe_radio) assumes -- the driver's own
    # defaults, spelled out rather than left implicit, exactly as
    # rpi-redskin/build.sh and its Makefile do.
    set -- "$@" -DDW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH=1
    set -- "$@" -DDW1000_WITH_PROPRIETARY_SFD=1
    set -- "$@" -DDW1000_WITH_PROPRIETARY_LONG_FRAME=0
    set -- "$@" -DDW1000_WITH_SFD_TIMEOUT=0

    set -- "$@" -DBITTERS_WITH_GPIO -DBITTERS_WITH_SPI
    set -- "$@" -DBITTERS_WITH_GPIO_IRQ -DBITTERS_WITH_THREADS

    set -- "$@" "-DDW1000_VERSION_GIT=\"$DW1000_VERSION_GIT\""
    set -- "$@" "-DBITTERS_VERSION_GIT=\"$BITTERS_VERSION_GIT\""

    set -- "$@" "$appdir/main.c"
    set -- "$@" $DW1000_SOURCES $DW1000_OSAL_SOURCES
    set -- "$@" $PROBE_SOURCES
    set -- "$@" $BITTERS_TAKEN

    set -- "$@" $DW1000_LIBS $BITTERS_LIBS -lm
    set -- "$@" -o "$out"
}

if [ "$dryrun" -eq 1 ]; then
    printf '%s' "$CC"
    for arg do printf ' %s' "$arg"; done
    printf '\n'
    exit 0
fi

rm -f -- "$out"

printf '%s: building %s (dw1000 %s%s, bitters %s%s, %s)\n' \
    "$progname" "$out" \
    "$DW1000_VERSION" "$DW1000_VERSION_GIT" \
    "$BITTERS_VERSION" "$BITTERS_VERSION_GIT" "$CC" >&2
"$CC" "$@" || die "build failed"
printf '%s: built %s\n' "$progname" "$out" >&2
