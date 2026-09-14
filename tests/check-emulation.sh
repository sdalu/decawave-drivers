#!/bin/sh
# port/emulation is the one port that needs nothing installed -- no
# vendor tree, no chip, and (since tests/emulation/smoke.c carries its
# own medium) no server either. So it is the one port this tree can
# actually *run* the driver against, rather than merely compile, and
# that is what this does: build the core, the port and the smoke test
# into a temporary directory, and run it.
#
# What the smoke test covers is in its own header comment. Here: one
# summary line, and the test's own lines kept when something failed.
# Run by `make check`.
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

echo "emulation: building the smoke test, port/emulation, $cc"

if ! $cc $cflags $inc -DDW1000_VERSION_GIT="\"$gitver\"" \
	-pthread -o "$tmp/smoke" "$top/tests/emulation/smoke.c" $src \
	$libs > "$tmp/build.log" 2>&1; then
    sed 's/^/  /' "$tmp/build.log"
    echo "emulation: the smoke test does not build"
    exit 1
fi

if "$tmp/smoke" > "$tmp/run.log" 2>&1; then
    echo "emulation: smoke test passed"
    exit 0
fi

sed 's/^/  /' "$tmp/run.log"
echo "emulation: smoke test FAILED"
exit 1
