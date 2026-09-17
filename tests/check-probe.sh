#!/bin/sh
# Stage 0 of the probe: record and role are free of <dw1000/dw1000.h>, so
# the format can be proved with no chip, no driver and no radio: just
# probe/src, the emulation port, and tests/probe/format.c. So this is the
# one probe check the tree can actually *run*, the same reason
# check-emulation.sh exists for the driver: build the three into a
# temporary directory and run it.
#
# probe/src also holds exchange.c, which is NOT free of <dw1000/dw1000.h>
# (it is chip mechanism, and drives the chip directly), so proving it
# compiles and links needs the driver core and an OSAL port alongside the
# probe port, exactly like check-emulation.sh needs them for the driver
# itself. Pulling those in here, from the manifest, with no Zephyr tree
# anywhere in sight, is what proves exchange.c's Zephyr coupling really
# is gone: if it still referenced a Zephyr header or type, this would not
# link. format.c itself calls nothing in exchange.c; linking it in is
# the check, the same way format.c already links port/emulation's clock
# in without calling it just to prove that builds too.
#
# TWO TESTS, each saying in its own header comment what it covers:
#   format    the four lines and the arithmetic behind them, no radio
#   exchange  the responder against port/emulation: that a run nobody
#             answers ends rather than hanging, and that its STATS line
#             says what the receiver heard and why none of it was used
#
# The second is why exchange.c being linked in is no longer merely a
# link check. It overrides the two POLL budgets, because a gate cannot
# prove a wait ends by waiting out a sixty-second one; the defaults are
# a property of the bench's harness and probe/src/exchange.c makes them
# overridable for exactly this.
#
# Here: one summary line per test, and the test's own lines kept when
# something failed. Run by `make check`.
#
# POSIX sh only.
set -e
top=`dirname "$0"`/..
cc=${CC:-cc}
cflags=${CFLAGS:--Wall -Wextra}

# From dw1000.cmake, like everything else that needs to know the files;
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

# 0.4 s for the first POLL and 0.2 s for each one after it, against 60 s
# and 2 s in the field. Both steps of the exchange test spend their whole
# budget by construction (nothing ever answers), so the run is about a
# second and a half, and the test's own alarm(20) is the backstop.
budgets='-DPROBE_EXCHANGE_FIRST_POLL_TIMEOUT_US=400000
	 -DPROBE_EXCHANGE_POLL_TIMEOUT_US=200000'

# Not a glob over tests/probe, for the reason check-emulation.sh gives:
# a build step that discovers its own inputs is what check-manifest.sh
# exists to punish.
status=0
for t in format exchange; do
    case $t in
    exchange) extra=$budgets ;;
    *)        extra=         ;;
    esac

    echo "probe: building the $t test, port/emulation, exchange.c, $cc"

    if ! $cc $cflags $inc $extra -DDW1000_VERSION_GIT="\"$gitver\"" -pthread \
	    -o "$tmp/$t" \
	    "$top/tests/probe/$t.c" $src $libs \
	    > "$tmp/build.log" 2>&1; then
	sed 's/^/  /' "$tmp/build.log"
	echo "probe: the $t test does not build"
	status=1
	continue
    fi

    if "$tmp/$t" > "$tmp/run.log" 2>&1; then
	echo "probe: $t test passed"
    else
	sed 's/^/  /' "$tmp/run.log"
	echo "probe: $t test FAILED"
	status=1
    fi
done

exit $status
