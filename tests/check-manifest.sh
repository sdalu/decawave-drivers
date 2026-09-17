#!/bin/sh
# dw1000.cmake is the manifest: the Makefile reads it through
# scripts/manifest.sh, and CMake consumers include it. The release is the
# same arrangement one file over -- written in <dw1000/dw1000_version.h>,
# because a C header can read no other file, and parsed from there by
# dw1000.cmake and by manifest.sh. There is no second copy of either to
# compare against any more -- what is left to check is that they still
# describe the tree, that the two parses of the version agree with the
# compiler's, and that nothing has quietly gone back to keeping its own
# list. Run by `make check`.
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

# --- and the compiler reads the same version --------------------------
# The manifest parses the header with awk and dw1000.cmake with a regex,
# but what a consumer actually gets is what the *preprocessor* makes of
# it. Ask it, so that a second #define, a comment in the wrong place or a
# clever macro cannot make the three disagree.
#
# DW1000_VERSION_FULL is asked with a git part passed in, which is how a
# build between releases gets one: it has to append to the release and not
# replace it, and it must not need the header edited to do so.
cc=${CC:-cc}
inc="-I$top/$($m incdir)"
# $cc and $1 are left unquoted deliberately: CC may carry flags of its own
# (CC='gcc --sysroot=/x'), and $1 is a flag list, empty on the first call.
# shellcheck disable=SC2086
cppver() {
    printf '#include <dw1000/dw1000_version.h>\n%s\n' "$2" \
	| $cc -E $1 $inc -x c - 2>/dev/null \
	| tr -d '" \t' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+' | tail -n 1
}

cppversion=$(cppver "" DW1000_VERSION_STRING)
if [ -z "$cppversion" ]; then
    echo "  version: the preprocessor makes nothing of DW1000_VERSION_STRING"
    bad=1
elif [ "$cppversion" != "$version" ]; then
    echo "  version: the manifest says $version, the preprocessor $cppversion"
    bad=1
fi

cppfull=$(cppver '-DDW1000_VERSION_GIT="+7.gdeadbee"' DW1000_VERSION_FULL)
if [ "$cppfull" != "$version+7.gdeadbee" ]; then
    echo "  version: DW1000_VERSION_FULL is '$cppfull', want"\
	 "'$version+7.gdeadbee' when a git part is passed"
    bad=1
fi

# --- and the tags say the same thing ----------------------------------
# The header is one half of naming a release; `git tag` is the other, and
# nothing in the tree makes them agree. `make tag` makes the tag from the
# header so that they cannot disagree, but a tag made by hand can, and a
# release build has no later chance to notice. Silent when there is no git,
# or when the repository around this tree is somebody else's.
if out=$(sh "$top/scripts/checktag.sh" "$version"); then
    :
else
    echo "$out"
    bad=1
fi

# --- the git part, when there is one ----------------------------------
# It is appended to the release, so it has to be an addition and not a
# replacement: SemVer build metadata, starting with '+'. Empty is the
# right answer for a release, a tarball, or a tree copied into another
# project's repository.
gitver=$(sh "$top/scripts/gitversion.sh")
case $gitver in
    "") ;;
    +*) ;;
    *) echo "  git part does not start with '+': '$gitver'"; bad=1 ;;
esac

for q in incdir core send validate; do
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
    for f in $s; do
	[ -f "$top/$f" ]            || { echo "  port $p: no file $f"; bad=1; }
    done
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

# ... and the version is the same arrangement: written in the header, read
# from there. A `set(DW1000_VERSION 1.2.3)` back in dw1000.cmake would be
# the second copy this checks the absence of.
if grep -qE '^set\(DW1000_VERSION[ \t]+[0-9]' "$top/dw1000.cmake"; then
    echo "  dw1000.cmake writes the version itself again"
    bad=1
fi

# Counted with awk rather than `echo $ports | wc -w`, which would need the
# expansion left unquoted in order to split.
nports=$(printf '%s\n' "$ports" | awk '{print NF}')

if [ $bad -eq 0 ]; then
    echo "manifest: $version$gitver, $nports ports, all present"
else
    echo "manifest: BROKEN"
fi
exit $bad
