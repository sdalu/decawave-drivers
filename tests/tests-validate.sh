#!/bin/sh
# <dw1000/dw1000_validate.h> turns a human radio value (a channel
# number, a bitrate in kbps, a PRF in MHz, a preamble length in symbols)
# into the field dw1000_radio_t wants. It has to accept exactly what
# _dw1000_radio_is_valid() accepts in dw1000.c, and the API exists because
# a hand-written copy of that table got it wrong in three places (see
# tests/validate/values.c, which names them).
#
# Nothing here touches a chip, a driver or a port: the source is a value
# mapping and links on its own, so this is the cheapest test in the tree:
# two compiles and two runs, no hardware, no threads, no clock.
#
# TWICE, on purpose. Five of the eight preamble lengths are proprietary
# and dw1000_configure() refuses them in a build without
# DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH, so what this API may accept
# depends on how the driver was compiled. A single run would check one
# half of that and the wrong half is the one that used to be broken.
#
# The paths come from the manifest, like everything else here;
# tests/check-manifest.sh fails a test that spells them itself.
#
# POSIX sh only.
set -e
top=`dirname "$0"`/..
cc=${CC:-cc}
cflags=${CFLAGS:--Wall -Wextra}

m="sh $top/scripts/manifest.sh"
# The null port, because it is the one that needs nothing installed and
# all that is wanted from it is a <dw1000/osal.h> for dw1000.h to find.
inc="-I$top/`$m incdir` -I$top/`$m inc null`"
src="$top/`$m validate`"

tmp=`mktemp -d`
trap 'rm -rf "$tmp"' EXIT INT TERM

# The third pass overrides DW1000_SPEED_OF_LIGHT_MPS, which dw1000.h
# leaves alone when it is already defined. Halved: light at half speed
# takes twice as long over a metre, so the conversion must double. Only
# the override case runs in that build: every other number in values.c
# is against the real constant.
status=0
for proprietary in 1 0 half-c; do
    extra=
    case $proprietary in
    1)      what="with proprietary preamble lengths"    ;;
    0)      what="without proprietary preamble lengths" ;;
    half-c) what="with the speed of light overridden"
	    proprietary=1
	    extra="-DDW1000_SPEED_OF_LIGHT_MPS=149896229.0
		   -DVALUES_SPEED_OF_LIGHT_OVERRIDDEN" ;;
    esac

    echo "validate: building the values test, $what, $cc"

    # shellcheck disable=SC2086
    if ! $cc $cflags $inc $extra \
	    -DDW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH=$proprietary \
	    -o "$tmp/values" \
	    "$top/tests/validate/values.c" $src \
	    > "$tmp/build.log" 2>&1; then
	sed 's/^/  /' "$tmp/build.log"
	echo "validate: the values test does not build $what"
	status=1
	continue
    fi

    if "$tmp/values" > "$tmp/run.log" 2>&1; then
	sed 's/^/  /' "$tmp/run.log"
	echo "validate: values test passed $what"
    else
	sed 's/^/  /' "$tmp/run.log"
	echo "validate: values test FAILED $what"
	status=1
    fi
done

exit $status
