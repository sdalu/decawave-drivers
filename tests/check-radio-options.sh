#!/bin/sh
# The probe and the sniffer take the same radio options, spelt the same
# way: probe/src/radio.c parses them for both probe shells, and the
# sniffer's popt table (sniffer/app/unix/cmdline.c) names them again,
# because it is a popt program and the probe's parser is not. Two tables
# drift unless something holds one to the other, so this holds the
# sniffer's to the list the probe publishes in its help macro,
# DW1000_PROBE_RADIO_OPTIONS_HELP, and adds the two buffering switches
# both programs take. Run by `make check`.
#
# POSIX sh, sed and grep only.
set -e
top=`dirname "$0"`/..
hdr=$top/probe/include/dw1000/probe/radio.h
cmd=$top/sniffer/app/unix/cmdline.c
bad=0

# The option names inside the macro's string literals: from the #define
# to the first line that does not continue it.
names=`sed -n '/^#define DW1000_PROBE_RADIO_OPTIONS_HELP/,/[^\\]$/p' "$hdr" \
       | grep -o -- '--[a-z][a-z-]*' | sed 's/^--//'`

if [ -z "$names" ]; then
    echo "  radio options: none found in $hdr; has the macro moved?"
    bad=1
fi

n=0
for o in $names dblbuff no-dblbuff; do
    n=`expr $n + 1`
    if ! grep -q "{ \"$o\"," "$cmd"; then
	echo "  radio options: the sniffer does not take --$o"
	bad=1
    fi
done

if [ $bad -eq 0 ]; then
    echo "radio options: $n, the same in the probe and the sniffer"
else
    echo "radio options: DIFFER"
fi
exit $bad
