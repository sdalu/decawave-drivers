#!/bin/sh
# sniffer/wireshark/uwbs.lua reads the sniffer's wire header; this checks
# that it reads it at the offsets sniffer/app/unix/wire.h documents.
#
# It exists because that Lua is the one piece of this tree nothing can
# run here. It needs wireshark, or at least a Lua interpreter with
# wireshark's Proto, ProtoField, Tvb and Dissector globals, and `make
# check` is run on machines that have none of those. So the script was
# written with its offsets as a transcription of wire.h's table, and this
# is what checks the transcription: not that the dissector works, which
# only wireshark can say, but that the two files agree about where every
# field is. That is the half of "works" most likely to rot, because
# wire.h can gain a field without anybody opening the Lua.
#
# What it does NOT check, and what therefore still has to be done by
# hand in wireshark at least once: that the Lua is syntactically valid,
# that the API calls are the right ones, and that the payload really
# reaches the 802.15.4 dissector. sniffer/DESIGN.md says as much.
#
# POSIX sh and POSIX awk, deliberately: adding an interpreter to the
# suite's dependencies to check one file would cost more than it buys.
set -e
top=`dirname "$0"`/..

hdr="$top/sniffer/app/unix/wire.h"
lua="$top/sniffer/wireshark/uwbs.lua"

for f in "$hdr" "$lua"; do
    if [ ! -f "$f" ]; then
	echo "wireshark: $f is missing"
	exit 1
    fi
done

# wire.h's table is the reference. Its lines look like
#
#    *     0     4  magic, "UWBS"
#
# so with the default field separator $2 is the offset and $3 the size.
# The Lua's reads are `tvb(OFF, SIZE)`, where OFF is a number for the
# fixed header and `o` or `o + N` inside the metadata block, o being
# WIRE_HDR_SIZE. Anything else in a tvb() call (the payload hand-off
# uses variables) is not an offset into the header and is skipped.
awk '
function trim(s) { sub(/^[ \t]+/, "", s); sub(/[ \t]+$/, "", s); return s }

BEGIN { meta = 32; bad = 0; nused = 0 }

# --- the reference -----------------------------------------------------
FILENAME == hdrfile && $1 == "*" && $2 ~ /^[0-9]+$/ && $3 ~ /^[0-9]+$/ {
    key = $2 "/" $3
    doc[key] = 1
    name[key] = $4
    ndoc++
    next
}

# --- what the Lua reads ------------------------------------------------
FILENAME == luafile {
    line = $0
    while (match(line, /tvb\([^)]*\)/)) {
	inside = substr(line, RSTART + 4, RLENGTH - 5)
	line   = substr(line, RSTART + RLENGTH)

	if (split(inside, a, ",") != 2)
	    continue

	off_s = trim(a[1])
	sz_s  = trim(a[2])

	if (sz_s !~ /^[0-9]+$/)
	    continue

	if (off_s ~ /^[0-9]+$/) {
	    off = off_s + 0
	} else if (off_s == "o") {
	    off = meta
	} else if (off_s ~ /^o[ \t]*\+[ \t]*[0-9]+$/) {
	    sub(/^o[ \t]*\+[ \t]*/, "", off_s)
	    off = meta + off_s + 0
	} else {
	    continue
	}

	key = off "/" sz_s
	if (!(key in used)) { used[key] = 1; nused++ }
    }
    next
}

END {
    # Every span the Lua reads must be a field wire.h documents. The one
    # exception is the offset and interval together (8 bytes at the top of
    # the metadata block), which the Lua highlights as one range for the
    # clock drift it computes from the pair.
    derived = (meta + 8) "/8"
    for (k in used) {
	if ((k in doc) || (k == derived))
	    continue
	split(k, p, "/")
	printf "wireshark: the Lua reads %s bytes at offset %s;" \
	       " wire.h documents no such field\n", p[2], p[1]
	bad = 1
    }

    # And every field wire.h documents must be read, except the reserved
    # padding, which wire.h itself says is ignored on read. A field added
    # to the header and not to the Lua is the drift this catches.
    for (k in doc) {
	if (k in used)
	    continue
	if (name[k] ~ /^reserved/)
	    continue
	split(k, p, "/")
	printf "wireshark: wire.h documents %s (%s bytes at offset %s)" \
	       " and the Lua never reads it\n", name[k], p[2], p[1]
	bad = 1
    }

    if (bad) {
	print "wireshark: uwbs.lua and wire.h disagree"
	exit 1
    }

    printf "wireshark: uwbs.lua reads %d spans, all at the offsets" \
	   " wire.h documents (%d fields)\n", nused, ndoc
}
' hdrfile="$hdr" luafile="$lua" "$hdr" "$lua"
