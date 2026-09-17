#!/bin/sh
# The README lists seven compile-time options, and nothing in this tree
# selects any of them: whoever vendors the driver does, in their own build.
# So an option that stops compiling stops compiling silently, and is found
# by the one consumer who wanted it. DW1000_WITH_EXTENDED_SEND=0 spent an
# unknown length of time giving fourteen errors that way.
#
# Compile the core over all 128 combinations, against the null port, the
# one OSAL that needs no vendor tree and no hardware. Run by `make check`.
#
# Only -fsyntax-only: this is about the options being coherent, not about
# codegen, and 128 real compiles would cost more than the answer is worth.
#
# POSIX sh and awk only.
set -e
top=`dirname "$0"`/..
cc=${CC:-cc}
cflags=${CFLAGS:--Wall -Wextra}

# From dw1000.cmake, like everything else that needs to know the files.
m="sh $top/scripts/manifest.sh"
inc="-I$top/`$m incdir` -I$top/`$m inc null`"
src=
# `state` is optional and absent from `sources`, so nothing else would
# ever compile it, which is exactly how an optional file stops
# compiling without anyone noticing. Named here on purpose.
for f in `$m sources` `$m state` `$m src null`; do src="$src $top/$f"; done

tmp=`mktemp -d`
trap 'rm -rf "$tmp"' EXIT INT TERM

# Ordered least to most surprising, so that the bit pattern reported for a
# failure reads left to right the way the README table does.
OPTS='DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH
DW1000_WITH_PROPRIETARY_SFD
DW1000_WITH_PROPRIETARY_LONG_FRAME
DW1000_WITH_EXTENDED_SEND
DW1000_WITH_SFD_TIMEOUT
DW1000_WITH_SFD_TIMEOUT_DEFAULT
DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011'

n=`echo "$OPTS" | wc -l | tr -d ' '`
total=`awk -v n="$n" 'BEGIN{print 2^n}'`
row=64          # dots per progress line
bad=0
run=0

# One option breaking the build breaks it in half the matrix, so failures
# are grouped by the compiler output they produced rather than printed as
# they happen: 64 copies of the same fourteen errors is not a report.
# The group's file names come from a checksum of that output.
: > "$tmp/order"

# Say what is about to happen, and then show it happening. This takes tens
# of seconds, and a silent minute is indistinguishable from a hang.
echo "options: $total combinations of $n options, port/null, $cc"
printf '  '

# Counting to 2^n and reading the bits off the counter, rather than
# nesting seven loops. `expr` and `test` are all this needs, so it stays
# in POSIX sh.
i=0
while [ "$i" -lt "$total" ]; do
    defs=
    label=
    bit=1
    for opt in $OPTS; do
        if [ $(( (i / bit) % 2 )) -eq 1 ]; then
            defs="$defs -D$opt=1"; label="$label"1
        else
            defs="$defs -D$opt=0"; label="$label"0
        fi
        bit=$(( bit * 2 ))
    done

    if $cc $cflags $inc $defs -fsyntax-only $src > "$tmp/log" 2>&1; then
        printf '.'
    else
        printf 'X'
        sig=`cksum < "$tmp/log" | tr -d ' \t'`
        if [ ! -f "$tmp/g.$sig.log" ]; then
            cp "$tmp/log" "$tmp/g.$sig.log"
            echo "$sig" >> "$tmp/order"
        fi
        echo "$label" >> "$tmp/g.$sig.labels"
        bad=1
    fi
    run=$(( run + 1 ))
    i=$(( i + 1 ))

    if [ $(( run % row )) -eq 0 ]; then
        printf ' %4d/%d\n' "$run" "$total"
        if [ "$run" -lt "$total" ]; then printf '  '; fi
    fi
done

# A total that is not a whole number of rows would leave the last row
# unterminated. It always is here, but the script should not depend on
# seven being the number of options.
if [ $(( run % row )) -ne 0 ]; then
    printf ' %4d/%d\n' "$run" "$total"
fi

# The options every member of a group agrees on are the ones that caused
# it; the ones that vary across the group are along for the ride. Saying
# so turns "64 combinations failed" into the name of the option to look
# at. Positions where the group disagrees print nothing.
common() {
    awk -v opts="`echo $OPTS`" '
	{
	    for (i = 1; i <= length($0); i++) {
		c = substr($0, i, 1)
		if (NR == 1)        v[i] = c
		else if (v[i] != c) v[i] = "?"
	    }
	}
	END {
	    n = split(opts, o, " ")
	    for (i = 1; i <= n; i++)
		if (v[i] != "?") printf " %s=%s", o[i], v[i]
	    printf "\n"
	}
    ' "$1"
}

nf=0
for sig in `cat "$tmp/order"`; do
    labels="$tmp/g.$sig.labels"
    cnt=`wc -l < "$labels" | tr -d ' '`
    nf=$(( nf + cnt ))
    echo ""
    if [ "$cnt" -eq 1 ]; then
	echo "  FAILED  one combination:`common "$labels"`"
    else
	echo "  FAILED  $cnt combinations, all with these errors."
	echo "          What they have in common:`common "$labels"`"
    fi
    sed 's/^/      /' "$tmp/g.$sig.log"
done

echo ""
if [ "$bad" -eq 0 ]; then
    echo "options: all $run compile"
else
    echo "options: $nf of $run FAILED, reported above"
fi
exit $bad
