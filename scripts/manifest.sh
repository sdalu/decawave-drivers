#!/bin/sh
# Read dw1000.cmake, which is the one place the file list lives, and
# <dw1000/dw1000_version.h>, which is the one place the release is written.
#
# CMake consumers include it directly. The Makefile cannot, so it asks
# here instead (`SRC_CORE != sh scripts/manifest.sh core` and so on),
# and shell-driven builds get the whole answer at once from
# `make sources`, which is `vars` below. Nothing keeps a second copy, so
# there is no second copy to drift.
#
# Paths come out relative to the top of the tree, since that is what the
# Makefile wants; `vars` takes a directory to make them absolute with,
# because that is what a consumer elsewhere wants.
#
# POSIX sh and awk only.

set -e

top=`dirname "$0"`/..
cm="$top/dw1000.cmake"

if [ ! -f "$cm" ]; then
    echo "manifest: no dw1000.cmake next to $0" >&2
    exit 1
fi

# The values of one set(NAME ...), whether it is written on one line or
# spread over several, with the ${CMAKE_CURRENT_LIST_DIR}/ prefix taken
# off and the whitespace squeezed to single spaces.
cmvar() {
    awk -v want="$1" '
	/^set\(/         { collecting = 1; buf = "" }
	collecting       { buf = buf " " $0 }
	collecting && /\)/ {
	    collecting = 0
	    sub(/^[ \t]*set\(/, "", buf)
	    sub(/\)[ \t]*$/,    "", buf)
	    name = buf
	    sub(/[ \t].*$/, "", name)
	    if (name == want) {
		sub(/^[^ \t]+[ \t]*/, "", buf)
		gsub(/\$\{CMAKE_CURRENT_LIST_DIR\}\//, "", buf)
		gsub(/[ \t]+/, " ", buf)
		sub(/^ /, "", buf); sub(/ $/, "", buf)
		print buf
	    }
	}
    ' "$cm"
}

# The release, from <dw1000/dw1000_version.h>. It is written there rather
# than here because a C header can read no other file: a consumer must have
# the version without running anything, so the header is where it lives and
# this is one of the two readers (dw1000.cmake parses the same three lines).
hdrversion() {
    h="$top/$(cmvar DW1000_INCLUDE_DIR)/dw1000/dw1000_version.h"
    awk '
	/^#define[ \t]+DW1000_VERSION_MAJOR[ \t]/ { maj = $3 }
	/^#define[ \t]+DW1000_VERSION_MINOR[ \t]/ { min = $3 }
	/^#define[ \t]+DW1000_VERSION_PATCH[ \t]/ { pat = $3 }
	END {
	    if (maj == "" || min == "" || pat == "") exit 1
	    print maj "." min "." pat
	}
    ' "$h" || {
	echo "manifest: no version in $h" >&2
	exit 1
    }
}

# set(DW1000_OSAL_CF2_INCLUDE_DIR ...) for port cf2. An unknown port
# gives an empty answer rather than an error: the Makefile's portcheck
# turns that into the message that names the ports there are.
portvar() {
    up=`echo "$1" | tr 'a-z' 'A-Z'`
    cmvar "DW1000_OSAL_${up}_$2"
}

# set(DW1000_PROBE_EMULATION_SOURCES ...) for probe port emulation. Shallower
# than portvar(): a probe port implements probe/include/dw1000/probe/port.h
# alone, so there is no per-port include directory to ask for.
probeportvar() {
    up=`echo "$1" | tr 'a-z' 'A-Z'`
    cmvar "DW1000_PROBE_${up}_SOURCES"
}

what=$1
[ $# -gt 0 ] && shift

case $what in
version)  hdrversion ;;
incdir)   cmvar DW1000_INCLUDE_DIR ;;
core)     cmvar DW1000_SOURCES_CORE ;;
send)     cmvar DW1000_SOURCES_SEND ;;
# Optional, and absent from `sources` for that reason: a consumer asks
# for it by name or does not compile it. See dw1000/dw1000_state.h.
state)    cmvar DW1000_SOURCES_STATE ;;
# Part of `sources`, unlike state. Answered on its own as well, for a
# consumer that takes `core` rather than `sources` because it does not
# transmit. See dw1000/dw1000_validate.h.
validate) cmvar DW1000_SOURCES_VALIDATE ;;
# DW1000_SOURCES is composed of the two in cmake syntax the Makefile
# cannot expand, so compose it here from the same two pieces.
sources)  echo "`cmvar DW1000_SOURCES_CORE` `cmvar DW1000_SOURCES_SEND`" \
	       "`cmvar DW1000_SOURCES_VALIDATE`" ;;
libs)     out=; for l in `cmvar DW1000_LIBS`; do out="$out -l$l"; done
	  echo $out ;;
ports)    cmvar DW1000_OSAL_PORTS ;;
inc)      portvar "$1" INCLUDE_DIR ;;
src)      portvar "$1" SOURCES ;;

# The probe: record and role, free of <dw1000/dw1000.h>, and its ports.
# tests/check-probe.sh asks here rather than naming probe/src or
# probe/port/* itself, the same discipline check-emulation.sh keeps for
# the driver.
probeincdir)  cmvar DW1000_PROBE_INCLUDE_DIR ;;
# Composed here from its two pieces, exactly as `sources` is for the
# driver: the cmake composite is ${...} references the awk cannot expand.
probecore)    cmvar DW1000_PROBE_SOURCES_CORE ;;
probeexchange) cmvar DW1000_PROBE_SOURCES_EXCHANGE ;;
probesrc)     echo "`cmvar DW1000_PROBE_SOURCES_CORE` `cmvar DW1000_PROBE_SOURCES_EXCHANGE`" ;;
probeports)   cmvar DW1000_PROBE_PORTS ;;
probeportsrc) probeportvar "$1" ;;

# Every port's OSAL object, for `make clean`: which port was built is not
# recorded anywhere, so clean removes them all.
objs)
    for p in `cmvar DW1000_OSAL_PORTS`; do
	echo "`portvar "$p" SOURCES`" | tr ' ' '\n' | sed 's/\.c$/.o/'
    done | tr '\n' ' '
    echo
    ;;

# What `make sources` prints: everything a build script needs, as shell
# variables, with $2 (the tree, absolutely) prefixed onto every path.
#
# No option flags are emitted. Which ones you want is your choice, and a
# default printed here would be a second place they are decided.
#
# The version comes in two pieces for the same reason. DW1000_VERSION is
# the release; DW1000_VERSION_GIT is what a build between releases adds to
# it, worked out here where the driver's own git tree is (it is empty for a
# tarball, and for a tree copied into your repository rather than cloned).
# Pass it on with -DDW1000_VERSION_GIT if you want the driver to know
# it: unlike the options it changes no structure, so it need not reach
# every translation unit.
vars)
    port=$1
    d=$2
    [ -n "$port" ] || { echo "manifest: vars needs a port" >&2; exit 1; }
    [ -n "$d"    ] || { echo "manifest: vars needs a directory" >&2; exit 1; }

    core=$d/`cmvar DW1000_SOURCES_CORE`
    send=$d/`cmvar DW1000_SOURCES_SEND`
    # Optional, so it is NOT folded into DW1000_SOURCES: a consumer
    # opts in by naming DW1000_SOURCES_STATE, and one that does not
    # simply ignores it. Emitted here rather than left to `make state`
    # so that a single `eval $(make sources)` gets it: a consumer that
    # has to ask twice fails differently against a driver too old to
    # answer, which is exactly what happened to rpi-redskin's build.sh.
    state=$d/`cmvar DW1000_SOURCES_STATE`
    validate=$d/`cmvar DW1000_SOURCES_VALIDATE`
    inc=$d/`cmvar DW1000_INCLUDE_DIR`
    oinc=$d/`portvar "$port" INCLUDE_DIR`
    osrc=$d/`portvar "$port" SOURCES`
    libs=
    for l in `cmvar DW1000_LIBS`; do libs="$libs -l$l"; done
    libs=`echo $libs`

    printf "DW1000_SOURCES_CORE='%s'\n" "$core"
    printf "DW1000_SOURCES_SEND='%s'\n" "$send"
    printf "DW1000_SOURCES='%s'\n"      "$core $send $validate"
    printf "DW1000_SOURCES_STATE='%s'\n" "$state"
    printf "DW1000_SOURCES_VALIDATE='%s'\n" "$validate"
    printf "DW1000_OSAL='%s'\n"         "$port"
    printf "DW1000_OSAL_SOURCES='%s'\n" "$osrc"
    printf "DW1000_INCLUDE='%s'\n"      "$inc"
    printf "DW1000_OSAL_INCLUDE='%s'\n" "$oinc"
    printf "DW1000_CFLAGS='%s'\n"       "-I$inc -I$oinc"
    printf "DW1000_LIBS='%s'\n"         "$libs"
    printf "DW1000_VERSION='%s'\n"      "$(hdrversion)"
    printf "DW1000_VERSION_GIT='%s'\n"  "$(sh "$top/scripts/gitversion.sh")"
    ;;

*)
    echo "usage: manifest.sh version|incdir|core|send|sources|libs|ports|objs" >&2
    echo "       manifest.sh inc|src <port>" >&2
    echo "       manifest.sh vars <port> <directory>" >&2
    echo "       manifest.sh validate" >&2
    echo "       manifest.sh probeincdir|probecore|probeexchange|probesrc|probeports" >&2
    echo "       manifest.sh probeportsrc <port>" >&2
    exit 1
    ;;
esac
