#!/bin/sh
# Stage 0 of the probe: record and role are free of <dw1000/dw1000.h>, so
# the format can be proved with no chip, no driver and no radio -- just
# probe/src, the emulation port, and tests/probe/format.c. So this is the
# one probe check the tree can actually *run*, the same reason
# check-emulation.sh exists for the driver: build the three into a
# temporary directory and run it.
#
# probe/src also holds exchange.c, which is NOT free of <dw1000/dw1000.h>
# -- it is chip mechanism, and drives the chip directly -- so proving it
# compiles and links needs the driver core and an OSAL port alongside the
# probe port, exactly like check-emulation.sh needs them for the driver
# itself. Pulling those in here, from the manifest, with no Zephyr tree
# anywhere in sight, is what proves exchange.c's Zephyr coupling really
# is gone: if it still referenced a Zephyr header or type, this would not
# link. format.c itself calls nothing in exchange.c -- linking it in is
# the check, the same way format.c already links port/emulation's clock
# in without calling it just to prove that builds too.
#
# What the test covers is in its own header comment. Here: one summary
# line, and the test's own lines kept when something failed. Run by
# `make check`.
#
# POSIX sh only.
set -e
top=`dirname "$0"`/..
cc=${CC:-cc}
cflags=${CFLAGS:--Wall -Wextra}

# From dw1000.cmake, like everything else that needs to know the files --
# never hardcoded here, which is what tests/check-manifest.sh punishes.
m="sh $top/scripts/manifest.sh"
inc="-I$top/`$m probeincdir` -I$top/`$m incdir` -I$top/`$m inc emulation`"
src=
for f in `$m probesrc` `$m probeportsrc emulation` `$m sources` `$m src emulation`; do
    src="$src $top/$f"
done
libs=`$m libs`

# DW1000_VERSION_FULL (used by exchange.c's STATS line) needs this the
# same way the driver's own version string does; empty is the right
# answer for a tree with no git part, same as everywhere else that asks.
gitver=`sh "$top/scripts/gitversion.sh"`

tmp=`mktemp -d`
trap 'rm -rf "$tmp"' EXIT INT TERM

echo "probe: building the format test, port/emulation, exchange.c, $cc"

if ! $cc $cflags $inc -DDW1000_VERSION_GIT="\"$gitver\"" -pthread \
	-o "$tmp/format" \
	"$top/tests/probe/format.c" $src $libs \
	> "$tmp/build.log" 2>&1; then
    sed 's/^/  /' "$tmp/build.log"
    echo "probe: the format test does not build"
    exit 1
fi

if "$tmp/format" > "$tmp/run.log" 2>&1; then
    echo "probe: format test passed"
    exit 0
fi

sed 's/^/  /' "$tmp/run.log"
echo "probe: format test FAILED"
exit 1
