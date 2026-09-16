#!/bin/sh
# port/emulation is the one port that needs nothing installed -- no
# vendor tree, no chip, and (since each test under tests/emulation
# carries its own medium) no server either. So it is the one port this
# tree can actually *run* the driver against, rather than merely
# compile, and that is what this does: build the core, the port and each
# test into a temporary directory, and run it.
#
# Three tests, each saying in its own header comment what it covers:
#   smoke   a frame out and a frame back, the four callbacks, a bad FCS
#   timing  the parts that need a clock -- SYS_TIME, delayed send and
#           receive, RXRFTO and RXPTO -- and the create/destroy cycle
#   dblbuf  the double receive buffer: the swinging set, HRBPT, overrun
#
# dblbuf is built twice. Errata 1.4 RX-1 needs a TX write past index 127,
# which the 127 byte standard frame cannot reach, so the second build
# turns proprietary long frames on and the file compiles one extra step.
#
# Here: one summary line per test, and the test's own lines kept when
# something failed. Run by `make check`.
#
# POSIX sh only.
set -e
top=`dirname "$0"`/..
cc=${CC:-cc}
cflags=${CFLAGS:--Wall -Wextra}

# From dw1000.cmake, like everything else that needs to know the files.
m="sh $top/scripts/manifest.sh"
inc="-I$top/`$m incdir` -I$top/`$m inc emulation`"
src=
for f in `$m sources` `$m src emulation`; do src="$src $top/$f"; done
libs=`$m libs`

# The one thing no file in the tree holds; the Makefile passes it the
# same way, and the version header defaults it to "" for everyone who
# does not.
gitver=`sh "$top/scripts/gitversion.sh"`

tmp=`mktemp -d`
trap 'rm -rf "$tmp"' EXIT INT TERM

# Not a glob over tests/emulation: tests/check-manifest.sh exists to
# punish a build step that discovers its own inputs, and a test added
# here should be added here deliberately.
tests="smoke timing dblbuf dblbuf_longframe"

status=0
for t in $tests; do
    echo "emulation: building the $t test, port/emulation, $cc"

    # One file, two builds: see the note at the top.
    case $t in
    dblbuf_longframe) file=dblbuf
		      extra=-DDW1000_WITH_PROPRIETARY_LONG_FRAME=1 ;;
    *)		      file=$t
		      extra= ;;
    esac

    if ! $cc $cflags $extra $inc -DDW1000_VERSION_GIT="\"$gitver\"" \
	    -pthread -o "$tmp/$t" "$top/tests/emulation/$file.c" $src \
	    $libs > "$tmp/build.log" 2>&1; then
	sed 's/^/  /' "$tmp/build.log"
	echo "emulation: the $t test does not build"
	status=1
	continue
    fi

    if "$tmp/$t" > "$tmp/run.log" 2>&1; then
	echo "emulation: $t test passed"
    else
	sed 's/^/  /' "$tmp/run.log"
	echo "emulation: $t test FAILED"
	status=1
    fi
done

exit $status
