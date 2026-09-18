#!/bin/sh
# Run sniffer/wireshark/uwbs.lua, outside wireshark, against a header the
# real wire.c encoded.
#
# tests/check-wireshark.sh compares the script's offsets against wire.h's
# table without running anything, and needs nothing installed. This goes
# further and actually executes the dissector, which catches what a
# static comparison cannot: the endianness of each read, the arithmetic
# on the nanosecond clock, the magic and version checks, how much of the
# payload is handed on. It needs a Lua interpreter, so it skips cleanly
# where there is none.
#
# NOT a C program linked against liblua, deliberately, which is why no
# Lua include directory is wanted anywhere below. wireshark's Lua API is
# faked in Lua (tests/lua/wsstub.lua), so what this needs is an
# interpreter to run, not headers to compile against. Embedding liblua in
# a C harness would mean writing the same stub twice, once in C against
# the Lua C API, for no more coverage.
#
# Lua 5.4 is the minimum, for the dissector and for this harness alike.
# The dissector uses the bitwise operators, which arrived in 5.3 and
# which an older wireshark cannot even parse; 5.4 rather than 5.3 is the
# line because that is what wireshark 4.4 and later build against, and
# supporting a version nothing here runs would be a claim nobody checks.
# A candidate below it is not used, and where that leaves nothing this
# check skips rather than failing: an old Lua on the machine the tree
# happens to sit on says nothing about the sniffer.
#
# Where Lua lives differs by system and by version, so nothing here is
# hardcoded:
#
#   LUA    the interpreter. Default: the first of
#          lua5.4 lua54 lua5.5 lua55 lua
#          on PATH that reports 5.4 or later, which covers Linux's
#          lua5.4 naming and FreeBSD's lua54.
#   LUAC   the syntax checker, or a list of them. Default: every
#          luac5.4 luac54 luac5.5 luac55 luac
#          on PATH that reports 5.4 or later.
#
# Each candidate is asked its version rather than trusted for its name,
# so an unversioned `lua` or `luac` that turns out to be 5.2 is passed
# over instead of reporting a failure that is really just an old
# interpreter. Setting either variable by hand overrides the search, and
# then a version below the minimum is reported as an error rather than
# skipped, because it was asked for:
#
#   LUA=/usr/local/bin/lua55 LUAC=/usr/local/bin/luac55 sh tests/check-lua.sh
#
# POSIX sh.
set -e
top=`dirname "$0"`/..
cc=${CC:-cc}
cflags=${CFLAGS:--Wall -Wextra}

LUA_MIN_MAJOR=5
LUA_MIN_MINOR=4
LUA_MIN=$LUA_MIN_MAJOR.$LUA_MIN_MINOR

dissector="$top/sniffer/wireshark/uwbs.lua"
harness="$top/tests/lua/uwbs_test.lua"

if [ ! -f "$dissector" ]; then
    echo "lua: $dissector is missing"
    exit 1
fi

# "Lua 5.4.8  Copyright ..." on stdout for 5.3 and later, on stderr for
# older ones, hence the redirection: an old interpreter must still be
# identifiable in order to be passed over.
lua_version() {
    "$1" -v 2>&1 | sed -n 's/^Lua \([0-9][0-9]*\)\.\([0-9][0-9]*\).*/\1 \2/p' \
	| head -1
}

# $1 major, $2 minor
version_ok() {
    [ -n "$1" ] || return 1
    [ "$1" -gt "$LUA_MIN_MAJOR" ] && return 0
    [ "$1" -eq "$LUA_MIN_MAJOR" ] && [ "$2" -ge "$LUA_MIN_MINOR" ] && return 0
    return 1
}

# --- the interpreter ---------------------------------------------------
lua=
if [ -n "${LUA:-}" ]; then
    if ! command -v "$LUA" > /dev/null 2>&1; then
	echo "lua: LUA=$LUA is not executable"
	exit 1
    fi
    set -- `lua_version "$LUA"`
    if ! version_ok "${1:-}" "${2:-}"; then
	echo "lua: LUA=$LUA is Lua ${1:-unknown}.${2:-}, below the $LUA_MIN"
	echo "lua: this dissector requires (5.2 cannot parse it at all; 5.3"
	echo "lua: can, but is not a version anything here is run against)"
	exit 1
    fi
    lua=$LUA
    luaver=$1.$2
else
    for c in lua5.4 lua54 lua5.5 lua55 lua; do
	command -v "$c" > /dev/null 2>&1 || continue
	set -- `lua_version "$c"`
	if version_ok "${1:-}" "${2:-}"; then
	    lua=$c
	    luaver=$1.$2
	    break
	fi
    done
fi

# --- the syntax checkers -----------------------------------------------
luacs=
if [ -n "${LUAC:-}" ]; then
    for c in $LUAC; do
	if ! command -v "$c" > /dev/null 2>&1; then
	    echo "lua: LUAC names $c, which is not executable"
	    exit 1
	fi
	set -- `lua_version "$c"`
	if ! version_ok "${1:-}" "${2:-}"; then
	    echo "lua: LUAC names $c, which is Lua ${1:-unknown}.${2:-};"
	    echo "lua: $LUA_MIN is the minimum"
	    exit 1
	fi
	luacs="$luacs $c"
    done
else
    for c in luac5.4 luac54 luac5.5 luac55 luac; do
	command -v "$c" > /dev/null 2>&1 || continue
	set -- `lua_version "$c"`
	if version_ok "${1:-}" "${2:-}"; then
	    luacs="$luacs $c"
	fi
    done
fi

if [ -z "$lua" ] && [ -z "$luacs" ]; then
    echo "lua: no Lua $LUA_MIN or later found; skipping."
    echo "lua: set LUA= and/or LUAC= to check sniffer/wireshark/uwbs.lua"
    echo "lua: (tests/check-wireshark.sh still checked its offsets)"
    exit 0
fi

status=0

# --- 1. does it parse? -------------------------------------------------
# Every checker found, not just the first: 5.4 and 5.5 are both in use,
# and a construct one accepts and the other does not is worth knowing
# about here rather than from somebody's wireshark.
for lc in $luacs; do
    if "$lc" -p "$dissector" > /dev/null 2>&1; then
	echo "lua: uwbs.lua parses with $lc"
    else
	echo "lua: uwbs.lua does NOT parse with $lc:"
	"$lc" -p "$dissector" 2>&1 | sed 's/^/  /'
	status=1
    fi
done

if [ -z "$lua" ]; then
    echo "lua: no interpreter of $LUA_MIN or later, so it was not run"
    exit $status
fi

if [ ! -f "$harness" ]; then
    echo "lua: $harness is missing"
    exit 1
fi

tmp=`mktemp -d`
trap 'rm -rf "$tmp"' EXIT INT TERM

# --- 2. the header the dissector is checked against --------------------
# Encoded by the real wire.c, which is the point: see tests/lua/emit.c.
# Only the driver's include path is wanted here, and no Lua one.
m="sh $top/scripts/manifest.sh"
inc="-I$top/`$m incdir` -I$top/`$m inc emulation` -I$top/sniffer/app/unix"

if ! $cc $cflags $inc -o "$tmp/emit" \
	"$top/tests/lua/emit.c" "$top/sniffer/app/unix/wire.c" \
	> "$tmp/build.log" 2>&1; then
    sed 's/^/  /' "$tmp/build.log"
    echo "lua: the header emitter does not build"
    exit 1
fi

# --- 3. run it ---------------------------------------------------------
echo "lua: running uwbs.lua under $lua (Lua $luaver)"
if "$lua" "$harness" "$dissector" "$tmp/emit" > "$tmp/run.log" 2>&1; then
    sed 's/^/  /' "$tmp/run.log"
    echo "lua: uwbs.lua behaved as wire.h says it should"
else
    sed 's/^/  /' "$tmp/run.log"
    echo "lua: uwbs.lua test FAILED"
    status=1
fi

exit $status
