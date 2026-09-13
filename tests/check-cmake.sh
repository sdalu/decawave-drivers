#!/bin/sh
# dw1000.cmake repeats what the Makefile already knows: the version, the
# core sources, and the include directory and source of every port. Two
# copies drift -- repository.yml in this tree still says 0.0.1 while the
# tags say v1.1.0 -- so check they agree. Run by `make check`.
set -e
top=`dirname "$0"`/..
bad=0

# --- version ----------------------------------------------------------
mk_ver=`sed -n 's/^VERSION *= *//p' "$top/Makefile" | tr -d ' \r'`
cm_ver=`sed -n 's/^set(DW1000_VERSION *\([0-9.]*\))/\1/p' "$top/dw1000.cmake"`
if [ "$mk_ver" != "$cm_ver" ]; then
    echo "  version differs: Makefile=$mk_ver dw1000.cmake=$cm_ver"
    bad=1
fi

# --- core sources -----------------------------------------------------
# Reduced to bare file names: the Makefile spells them relative to the
# tree, dw1000.cmake through ${CMAKE_CURRENT_LIST_DIR}, and it is the set
# that has to match, not the spelling.
mk_src=`{ sed -n 's/^SRC_CORE *= *//p' "$top/Makefile"; \
          sed -n 's/^SRC_SEND *= *//p' "$top/Makefile"; } \
        | sed 's|.*/||' | sed '/^$/d' | sort | tr -d '\r'`
cm_src=`sed -n 's|.*/hw/drivers/dw1000/src/\([a-z0-9_]*\.c\))|\1|p' \
        "$top/dw1000.cmake" | sort`
if [ "$mk_src" != "$cm_src" ]; then
    echo "  core sources differ between Makefile and dw1000.cmake:"
    echo "    Makefile    : `echo $mk_src`"
    echo "    dw1000.cmake: `echo $cm_src`"
    bad=1
fi

# --- ports ------------------------------------------------------------
mk_ports=`sed -n 's/^OSAL_PORTS *= *//p' "$top/Makefile" | tr ' ' '\n' \
          | sed '/^$/d' | sort | tr -d '\r'`
cm_ports=`sed -n 's/^set(DW1000_OSAL_PORTS *\(.*\))/\1/p' "$top/dw1000.cmake" \
          | tr ' ' '\n' | sed '/^$/d' | sort`
if [ "$mk_ports" != "$cm_ports" ]; then
    echo "  port lists differ between Makefile and dw1000.cmake:"
    echo "    Makefile    : `echo $mk_ports`"
    echo "    dw1000.cmake: `echo $cm_ports`"
    bad=1
fi

# The port list is only a list of names; what matters is that both files
# point each name at the same two paths, and that those paths are there.
# A port whose osal.c is called something else -- chibios's is dw_osal.c
# -- is exactly what a name-only check would miss.
for p in $mk_ports; do
    P=`echo "$p" | tr 'a-z' 'A-Z'`
    mk_i=`sed -n "s|^OSAL_INC_$p *= *||p" "$top/Makefile" | tr -d ' \r'`
    mk_s=`sed -n "s|^OSAL_SRC_$p *= *||p" "$top/Makefile" | tr -d ' \r'`
    cm_i=`sed -n "/^set(DW1000_OSAL_${P}_INCLUDE_DIR\$/,/)/s|.*CMAKE_CURRENT_LIST_DIR}/\(.*\))|\1|p" \
          "$top/dw1000.cmake"`
    cm_s=`sed -n "/^set(DW1000_OSAL_${P}_SOURCES\$/,/)/s|.*CMAKE_CURRENT_LIST_DIR}/\(.*\))|\1|p" \
          "$top/dw1000.cmake"`
    if [ "$mk_i" != "$cm_i" ] || [ "$mk_s" != "$cm_s" ]; then
        echo "  port $p differs:"
        echo "    Makefile    : $mk_i $mk_s"
        echo "    dw1000.cmake: $cm_i $cm_s"
        bad=1
        continue
    fi
    if [ ! -d "$top/$mk_i" ]; then
        echo "  port $p: no such directory: $mk_i"; bad=1
    fi
    if [ ! -f "$top/$mk_i/dw1000/osal.h" ]; then
        echo "  port $p: no <dw1000/osal.h> under $mk_i"; bad=1
    fi
    if [ ! -f "$top/$mk_s" ]; then
        echo "  port $p: no such file: $mk_s"; bad=1
    fi
done

# --- the Zephyr module ------------------------------------------------
# zephyr/CMakeLists.txt used to name the same five paths a second time.
# It consumes dw1000.cmake now; make sure it has not gone back.
if grep -q 'hw/drivers/dw1000/src' "$top/zephyr/CMakeLists.txt"; then
    echo "  zephyr/CMakeLists.txt names core sources directly again"
    bad=1
fi

if [ $bad -eq 0 ]; then
    echo "cmake: agrees with the Makefile: $mk_ver, `echo $mk_ports | wc -w | tr -d ' '` ports"
else
    echo "cmake: MISMATCH"
fi
exit $bad
