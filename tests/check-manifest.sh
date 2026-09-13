#!/bin/sh
# dw1000.cmake is the manifest: the Makefile reads it through
# scripts/manifest.sh, and CMake consumers include it. There is no second
# copy to compare it against any more -- what is left to check is that it
# still describes the tree, and that nothing has quietly gone back to
# keeping its own list. Run by `make check`.
set -e
top=`dirname "$0"`/..
m="sh $top/scripts/manifest.sh"
bad=0

# --- the manifest parses at all ---------------------------------------
# Every query below returns the empty string when the awk stops matching,
# and an empty source list would make `make check` pass by compiling
# nothing. Fail loudly instead.
version=`$m version`
case $version in
    [0-9]*.[0-9]*) ;;
    *) echo "  version is not a version: '$version'"; bad=1 ;;
esac

for q in incdir core send; do
    v=`$m $q`
    if [ -z "$v" ]; then
	echo "  manifest: $q is empty"; bad=1
    elif [ ! -e "$top/$v" ]; then
	echo "  manifest: $q names nothing: $v"; bad=1
    fi
done

ports=`$m ports`
if [ -z "$ports" ]; then
    echo "  manifest: no ports"; bad=1
fi

# --- every port is really there ---------------------------------------
# A port whose osal.c is called something else -- chibios's is dw_osal.c
# -- is exactly what a check on the names alone would miss.
for p in $ports; do
    i=`$m inc $p`
    s=`$m src $p`
    if [ -z "$i" ] || [ -z "$s" ]; then
	echo "  port $p: listed in DW1000_OSAL_PORTS but not defined"
	bad=1
	continue
    fi
    [ -d "$top/$i" ]                || { echo "  port $p: no directory $i"; bad=1; }
    [ -f "$top/$i/dw1000/osal.h" ]  || { echo "  port $p: no <dw1000/osal.h> under $i"; bad=1; }
    [ -f "$top/$s" ]                || { echo "  port $p: no file $s"; bad=1; }
done

# --- nothing keeps its own copy ---------------------------------------
# zephyr/CMakeLists.txt used to name the same five paths a second time,
# and the Makefile used to hold the whole list. Both read the manifest
# now; fail if either starts spelling a source path itself again.
if grep -q 'hw/drivers/dw1000/src' "$top/zephyr/CMakeLists.txt"; then
    echo "  zephyr/CMakeLists.txt names core sources directly again"
    bad=1
fi
if grep -q 'hw/drivers/dw1000/src\|port/[a-z0-9]*/dw/osal' "$top/Makefile"; then
    echo "  Makefile names driver or port paths directly again"
    bad=1
fi

if [ $bad -eq 0 ]; then
    echo "manifest: $version, `echo $ports | wc -w | tr -d ' '` ports, all present"
else
    echo "manifest: BROKEN"
fi
exit $bad
