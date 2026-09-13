#!/bin/sh
# The README lists eight compile-time options, and nothing in this tree
# selects any of them: whoever vendors the driver does, in their own build.
# So an option that stops compiling stops compiling silently, and is found
# by the one consumer who wanted it. DW1000_WITH_EXTENDED_SEND=0 spent an
# unknown length of time giving fourteen errors that way.
#
# Compile the core over all 256 combinations, against the null port -- the
# one OSAL that needs no vendor tree and no hardware. Run by `make check`.
#
# Only -fsyntax-only: this is about the options being coherent, not about
# codegen, and 256 real compiles would cost more than the answer is worth.
set -e
top=`dirname "$0"`/..
cc=${CC:-cc}
cflags=${CFLAGS:--Wall -Wextra}

# From dw1000.cmake, like everything else that needs to know the files.
m="sh $top/scripts/manifest.sh"
inc="-I$top/`$m incdir` -I$top/`$m inc null`"
src=
for f in `$m sources` `$m src null`; do src="$src $top/$f"; done

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
DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011
DW1000_WITH_DWM1000_EVK_COMPATIBILITY'

n=`echo "$OPTS" | wc -l | tr -d ' '`
total=`awk -v n="$n" 'BEGIN{print 2^n}'`
bad=0
run=0

# Counting to 2^n and reading the bits off the counter, rather than
# nesting eight loops. `expr` and `test` are all this needs, so it stays
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
        :
    else
        echo "  FAILED  $label"
        # Spell the combination out: the bit string says which, but not
        # what, and the point of the report is to be pasteable.
        echo "    `echo $defs`"
        sed 's/^/      /' "$tmp/log"
        bad=1
    fi
    run=$(( run + 1 ))
    i=$(( i + 1 ))
done

if [ "$bad" -eq 0 ]; then
    echo "options: $run combinations of $n options compile"
else
    echo "options: FAILURES above, out of $run combinations"
fi
exit $bad
