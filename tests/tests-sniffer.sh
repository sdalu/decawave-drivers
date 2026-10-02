#!/bin/sh
# The sniffer's host tests: the parts of sniffer/app/unix that can be run
# with no chip, no radio and no Raspberry Pi.
#
# Most of that application cannot be built here at all. eth.c wants
# Linux's AF_PACKET and its struct ifreq, uwb_dw1000.c and main.c want
# bitters and the Linux GPIO and SPI character devices, and cmdline.c
# wants libpopt: a sniffer is a Linux program for a Pi, and this script
# is run wherever the tree is. So three of its seven translation units
# were written to need none of that, and those three are what this
# builds:
#
#   capture   the frame ring: the read-out seam, oldest-first delivery,
#             the overrun accounting, truncation, the high-water mark,
#             wire.c's header byte by byte at its documented offsets, and
#             the flattening of a captured frame for a dissector
#   pcapng    the pcapng writer: the three block types, their lengths and
#             their padding, and a file handed to tcpdump(1) at the end
#   dissect   the dissector registry and the shared-object loader: what
#             registration refuses, which hook wins, and what the loader
#             makes of a bare name, a missing file and a non-dissector
#
# and then two checks on sniffer/wireshark/uwbs.lua, which is not C and
# not compiled: tests/check-wireshark.sh compares its offsets against
# wire.h needing nothing installed, and tests/check-lua.sh runs it
# against a header wire.c encoded, where a Lua interpreter can be found.
#
# That split is the point of capture.c reaching the chip through
# capture_ops rather than calling uwb_read_frame_data() directly: the
# sniffer's only real logic used to be unreachable from any test, which
# is why it had none. Same reason tests/tests-probe.sh exists for the
# probe and tests/tests-emulation.sh for the driver. dissect.c is in the
# portable set for the same reason and one more: the interface it
# implements is what other people's code compiles against, so its
# refusals are worth checking somewhere other than on a Pi with
# somebody's plugin.
#
# The dissect test needs two shared objects to try loading. They are
# built here into the temporary directory, and if this host cannot build
# a shared object at all the test is still run, without them: it skips
# its two loader cases and says so, rather than failing for something
# that is not the sniffer's fault.
#
# The driver include path and the emulation OSAL come from the manifest,
# like everything else that needs to know where things are. The
# sniffer's own files are named here, because the sniffer is an
# application and appears nowhere in dw1000.cmake: there is no manifest
# to ask.
#
# Here: one summary line per test, and the test's own lines kept when
# something failed. Run by `make tests`.
#
# POSIX sh only.
set -e
top=`dirname "$0"`/..
cc=${CC:-cc}
cflags=${CFLAGS:--Wall -Wextra}

m="sh $top/scripts/manifest.sh"
inc="-I$top/`$m incdir` -I$top/`$m inc emulation` -I$top/sniffer/app/unix"

app="$top/sniffer/app/unix"

tmp=`mktemp -d`
trap 'rm -rf "$tmp"' EXIT INT TERM

# The pcapng test writes real files and hands one to tcpdump(1), so it
# needs somewhere to put them. Pointing TMPDIR at the directory the trap
# above already removes means `make tests` leaves nothing behind, which
# a fixed name under /tmp would not: the test names its files itself and
# keeps them deliberately, so that the one it feeds to tcpdump is the
# one it wrote.
TMPDIR="$tmp"
export TMPDIR

# Not a glob over tests/sniffer, for the reason tests-emulation.sh gives:
# a build step that discovers its own inputs is what check-manifest.sh
# exists to punish.
# The dissect test's fixtures. -fvisibility=hidden on the first, to prove
# that DISSECT_EXPORT really is what keeps the entry point visible; the
# second deliberately exports no entry point at all.
fixtures=
if $cc -shared -fPIC -fvisibility=hidden $inc \
	-o "$tmp/dissect_plugin_ok.so" \
	"$top/tests/sniffer/dissect_plugin_ok.c" > "$tmp/fx.log" 2>&1 &&
   $cc -shared -fPIC $inc \
	-o "$tmp/dissect_plugin_nosym.so" \
	"$top/tests/sniffer/dissect_plugin_nosym.c" >> "$tmp/fx.log" 2>&1
then
    fixtures=$tmp
else
    echo "sniffer: cannot build a shared object here; the dissect test"
    echo "sniffer: will skip its loader cases (see below)"
    sed 's/^/  /' "$tmp/fx.log"
fi

status=0
for t in capture pcapng dissect; do
    args=
    case $t in
    capture) src="$app/capture.c $app/wire.c" ;;
    pcapng)  src="$app/pcapng.c"              ;;
    dissect) src="$app/dissect.c"; args=$fixtures ;;
    esac

    echo "sniffer: building the $t test, $cc"

    # VERSION is compiled into the application by build.sh, from the
    # driver's release; the writer puts it in the pcapng file's
    # shb_userappl option. A test build has no such thing to report and
    # says so rather than failing to compile.
    if ! $cc $cflags $inc -DVERSION='"uwb-sniffer (test build)"' \
	    -o "$tmp/$t" \
	    "$top/tests/sniffer/$t.c" $src \
	    > "$tmp/build.log" 2>&1; then
	sed 's/^/  /' "$tmp/build.log"
	echo "sniffer: the $t test does not build"
	status=1
	continue
    fi

    # $args is deliberately unquoted: empty means "no argument at all",
    # which is how the dissect test is told to skip its loader cases.
    # shellcheck disable=SC2086
    if "$tmp/$t" $args > "$tmp/run.log" 2>&1; then
	echo "sniffer: $t test passed"
    else
	sed 's/^/  /' "$tmp/run.log"
	echo "sniffer: $t test FAILED"
	status=1
    fi
done

# The wireshark dissector, which is neither compiled nor run: it is Lua,
# and this host may have neither wireshark nor an interpreter. What can be
# checked without either is that it reads the wire header at the offsets
# wire.h documents, which is the half most likely to rot. Its own script
# says what that does and does not prove.
if sh "$top/tests/check-wireshark.sh"; then
    :
else
    status=1
fi

# And then run it, where there is a Lua to run it with. That catches what
# comparing offsets cannot: endianness, the arithmetic on the nanosecond
# clock, and how much of the payload is handed on. It skips cleanly with
# no Lua installed, so this is not a new dependency for `make tests`.
if CC="$cc" CFLAGS="$cflags" sh "$top/tests/check-lua.sh"; then
    :
else
    status=1
fi

exit $status
