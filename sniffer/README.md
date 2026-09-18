# UWB sniffer

Capture UWB frames off a DW1000 and forward each one, whole, either
inside an ethernet frame to another host or as a local pcapng file, so
that wireshark or tcpdump can look at it. The radio is configured from
the command line, the receiver runs with no timeout and re-arms after
every frame, and nothing is transmitted: this is a listener.

It is an application and nothing else: no library part, no port layer,
and no entry in `dw1000.cmake`, for the same reason `probe/app/unix` has
none: nothing else consumes it. What it does consume is the driver
(receive half only) and [bitters][bitters] for the Raspberry Pi's GPIO
and SPI.

[bitters]: https://gitlab.inria.fr/dalu/bitters

## Usage

```text
Usage: uwb-sniffer [OPTIONS]* [<dst_macaddr>]
  -P, --prototype=INT        ethernet prototype (default: 9999)
  -i, --interface=STRING     ethernet interface (default: first valid)
  -c, --channel=INT          channel                     (default: 5)
  -b, --bitrate=INT          bitrate (in kbps)           (default: 6800)
  -p, --prf=INT              pulse rate frequency in MHz (default: 64)
      --tx_plen=INT          preamble length             (default: 128)
      --rx_pac=INT           preamble accumulation       (default: 8)
      --tx_pcode=INT         TX preamble code            (default: 10)
      --rx_pcode=INT         RX preamble code            (default: 10)
      --tx_delay=FLOAT       antenna TX delay (in meters)
      --rx_delay=FLOAT       antenna RX delay (in meters)
  -w, --write=STRING         write pcapng to FILE (- for stdout)
      --raw                  send bare frames, without the header
      --no-metadata          do not read timestamp/power per frame
      --no-dblbuff           single buffered receive
      --count=LONG           stop after N frames (default: no limit)
      --stats=INT            report counters every N seconds (default: off)
      --dissector=STRING     load a dissector from FILE[:args] (repeatable)
      --dissect-filter       forward only frames a dissector accepts
      --list-dissectors      print the dissectors and exit
      --no-dissect           ignore every dissector
  -v, --verbose              verbose mode
  -V, --version              show version information
```

`<dst_macaddr>` is now optional: give it to send frames over ethernet, `-w`
to write pcapng, or both. At least one of the two is required, since a
run with neither would capture and then discard every frame.

Every radio value is checked before the chip is touched, and a bad one
ends the program with the reason rather than being quietly clamped. The
accepted values are the driver's, in `<dw1000/dw1000_validate.h>`, and
`cmdline.c` calls straight into it: nothing here keeps a second copy of
the rules, which is what this program used to do and how it came to
accept preamble codes the driver rejects. An antenna delay left unset
takes the driver's own; `-V` reports the driver release this was built
against, since that is now the version this program has.

`-P` (long form --prototype) must be 0 or in 0x05DD..0xFFFF: anything at or below
0x05DC would be read as a length by a receiving stack rather than as a
type, which is why the default is 9999 (0x270F) and not something
smaller.

`-v` prints a line per frame, with a dissector's reading of it appended
when one is loaded. Without it the program is quiet once
running, which is what you want at any real frame rate. That print
used to happen inside the receive callback, where it cost the radio time
on every frame.

Everything a person reads (`-v`'s lines, the reception account, every
warning, the boot-time `raspi-gpio` hint and the tcpdump hint) goes to
stderr, never stdout. Stdout carries only the pcapng stream when `-w -`
is given, or `-V`'s version line, which prints and exits before any
capture is opened. That is what makes
`uwb-sniffer -w - <dst_macaddr> | wireshark -k -i -` work: nothing this
program prints for a person can land in the middle of the file.

## Output: ethernet, pcapng, or both

Each captured frame carries a 32-byte header ahead of it on the wire,
little endian, starting with the magic `UWBS` and a version byte: a
sequence number that a receiver can use to see a gap, the driver's own
reported length (so a frame the ring had to truncate does not look like
a whole one), and the two loss counts below. The --raw flag (below)
sends the bare frame instead, exactly as every earlier build of this
program did, for
a receiver written against that output; the magic is what lets a
receiver on the other end tell which it is being handed. `wire.h` is the
header's byte-for-byte layout.

`-w FILE` (or `-w -` for stdout) writes the same captured frames as a
pcapng file instead of, or as well as, sending them over ethernet:
wireshark or `tcpdump -r` read it with no dissector needed, unlike the
ethernet-wrapped output. `<dst_macaddr>` becomes optional once `-w` is
given: a run that only writes pcapng opens no ethernet socket at all.

Every frame also carries what the chip knew about it beyond its bytes:
the arrival timestamp, the two power estimates, the transmitter's clock
offset and interval, the first-path index and the two noise figures.
This costs about ten register reads a frame, on by default; --no-metadata
turns it off for a run that wants the highest frame rate instead. On
the wire it is the header's optional second 32 bytes; in a pcapng file
it is the packet's comment, which wireshark shows alongside the frame.

--count N stops the run after N frames have been forwarded; --stats N
prints the reception account below every N seconds without stopping.
Both are unset (no limit, no periodic report) unless given.

Ctrl-C (or a `TERM`) stops the loop rather than killing the program
outright, and the run prints its reception account on the way out:
how many frames were captured and forwarded, how deep the ring ever
got, how many were lost to the ring falling behind or to the chip
overrunning, and how many receive errors the chip reported, broken
down by cause. Those two loss counts are kept apart because they are
fixed by different things: a ring loss means the host fell behind its
own forwarding, a chip overrun means the host fell behind the radio
itself.

Frame filtering is always off. There is no flag to turn it on: this
program never writes the PAN id or the addresses hardware filtering
matches against, so the only effect a filter switch could have is to
make the sniffer deaf to everything, silently.

## Dissectors

A dissector turns a frame's bytes into one line of text, and can say
whether a frame should be forwarded at all. It affects three things: the
`-v` line, the pcapng packet comment, and (only with
`--dissect-filter`) whether the frame goes out. It cannot change what is
captured or written.

Two ways to supply one. Compiled in, from a tree named at build time:

```sh
DISSECTORS=$PWD/sniffer/dissectors/ieee802154 \
    sh sniffer/app/unix/build.sh
```

or loaded at start-up, from a shared object:

```sh
make -C sniffer/dissectors/ieee802154 plugin
uwb-sniffer --dissector=./sniffer/dissectors/ieee802154/ieee802154.so \
    -w capture.pcapng
```

`--dissector` may be given more than once, and takes arguments after a
colon: `--dissector=./ieee802154.so:types=data,ack`. The path must
contain a `/`; a bare name is refused rather than looked for on the
library path, since this program normally runs privileged.
`--list-dissectors` prints what is registered and exits without touching
the radio.

`sniffer/dissectors/ieee802154` is a worked example, useful in its own
right (every frame here is 802.15.4-shaped) and the reference for
writing another: it registers both ways from one source file, uses all
three hooks, and its `Makefile` documents the interface a dissector tree
must present. A dissector for a protocol *inside* the frame belongs in
that protocol's own repository, so that it includes the layout's real
definition instead of keeping a second copy; `sniffer/DESIGN.md` says
why the interface is shaped to allow that.

`--dissect-filter` is off by default. Dropping a frame before the
`sendmsg(2)` that would have carried it is the one thing a dissector can
do that wireshark on the far side cannot, but whether it pays depends on
the traffic: it spends dissector time to save a syscall. The reception
account is how to decide, not this paragraph. With `--dissect-filter` and
dissectors that do not filter, every frame is forwarded, so turning it on
by mistake costs nothing.

## Reading the ethernet output in wireshark

The pcapng output needs nothing: link type 195 is IEEE 802.15.4 with FCS
and wireshark dissects it already. The ethernet output is the one that
used to arrive as bytes nothing understood, and
[`wireshark/uwbs.lua`](wireshark/uwbs.lua) is for that: it reads the
`UWBS` header, makes every field filterable (`uwbs.lost_ring > 0`,
`uwbs.power_signal < -90000`, `uwbs.truncated`), and hands the frame
itself to wireshark's 802.15.4 dissector.

```sh
cp sniffer/wireshark/uwbs.lua ~/.local/lib/wireshark/plugins/
```

then Ctrl-Shift-L in wireshark to reload, and

```sh
tcpdump -i eth1 -w - ether proto 0x270f | wireshark -k -i -
```

It binds to ethertype 9999, which is `--prototype`'s default; a capture
made with another value wants that number set in Preferences > Protocols
> UWBS.

`make check` does test it, in two layers.
`tests/check-wireshark.sh` needs nothing installed and checks that the
script reads the header at the offsets `wire.h` documents.
`tests/check-lua.sh` parses it with every `luac` it finds and then runs
it against a header the real `wire.c` encoded, skipping cleanly where no
Lua is installed. `LUA=` and `LUAC=` override which interpreter is used,
the defaults covering both `lua5.4` (Linux) and `lua54` (FreeBSD)
naming:

```sh
LUA=lua55 LUAC=luac55 sh tests/check-lua.sh
```

**The script needs Lua 5.4 or later, and so wireshark 4.4 or later.** It
uses the bitwise operators, which arrived in Lua 5.3; a wireshark built
against 5.2 does not merely misread it, it fails to parse it, and the
error names the operator rather than the version. A complaint about
`')' expected near '&'` means exactly that.

What neither test layer can prove is that wireshark agrees with the
harness about its own API, so it is still worth opening a capture in
wireshark once by hand.

## How it receives

The radio runs double buffered by default, and frames are forwarded from
a ring rather than from the callback that received them. Both are
lessons taken from `probe`, which arrived at them the hard way; the
driver's own guide
([`../hw/drivers/dw1000/README.md`](../hw/drivers/dw1000/README.md),
"Double buffered receive") is where they are written down.
--no-dblbuff falls back to single buffering without a rebuild, which
costs the chip being deaf for the whole read-out of every frame; nothing
else about how a frame is handled changes with it.

`dblbuff` means the driver re-enables the receiver in the good-frame
path *before* it calls back, so the next frame lands in the other buffer
while this one is read out, and it toggles the host side buffer pointer
as soon as the callback returns. Two things follow. The callback must
not re-enable the receiver, so the receiver is armed exactly once,
before the loop. And everything the frame is wanted for must be read out
before the callback returns, because `RX_BUFFER` swings with the
pointer.

That second obligation is what the ring is for. `eth_send()` is a
blocking `sendmsg(2)`; doing it inside the callback would hold the
callback open across the kernel's network path while the next frame is
already arriving. So the callback copies the frame into a ring slot and
returns, and the loop forwards what the ring holds afterwards.
`capture.c` carries the detail, including why it needs no lock where
probe's equivalent does.

A frame that reached the host can still be lost, in one place: if
forwarding falls more than the ring's depth (16) behind the radio, the
oldest entries are overwritten. That is counted, and the loop says so on
stderr when the count moves: `lost N frame(s): forwarding fell behind
the radio`. An overrun on the *chip* (both buffers held and a third
frame arriving) loses frames too, but earlier and for a different
reason, so it is counted and reported separately: the driver recovers it
and reports it through the `rx_error` callback, which re-arms. That
callback is not optional under double buffering, as
`dw1000_initialise()` refuses `dblbuff` without one.

Whether the ring's depth of 16 is enough for a given run is no longer a
guess: the reception account (`--stats N`, or when the run ends) reports
the high-water mark alongside the overrun count, so a run that never
passed 2 has 14 entries to spare and a run that overran knows by how
much.

## Example

```sh
uwb-sniffer -i eth1 -P 6666                                 \
    -c 5 -b 6800 -p 64                                      \
    --tx_pcode 10 --rx_pcode 10 --tx_plen 128 --rx_pac 8     \
    dc:4a:3e:06:6f:7b
```

On the receiving host, filtering on the type and on the Pi's own address
(the program prints the tcpdump line to use, with its real address
filled in, once the interface is up):

```sh
tcpdump ether proto 6666 and ether src aa:bb:cc:dd:ee:ff
```

That captures the ethernet frames, header and payload alike; nothing on
the receiving host understands the `UWBS` header or the 802.15.4 frame
inside it, since no dissector exists for either. For that, capture
locally instead and hand the file straight to wireshark, with no
ethernet destination and no socket opened:

```sh
uwb-sniffer -c 5 -b 6800 -p 64 -w - | wireshark -k -i -
```

## Wiring

On a Raspberry Pi, connect the DecaWave module to these pins:

| RPI | DWM1000 | Meaning           |
| --- | ------- | ----------------- |
| 14  | GND     | Ground            |
| 15  | IRQ     | DWM1000 interrupt |
| 16  | WAKEUP  | DWM1000 wakeup    |
| 17  | 3V3     | Power 3.3V        |
| 18  | RESET   | DWM1000 reset     |
| 19  | MOSI    | SPI MOSI          |
| 21  | MISO    | SPI MISO          |
| 23  | SCLK    | SPI clock         |
| 24  | CS      | SPI chip select   |

Reset and wakeup want a pull-up held across boot, which the Pi does not
do by itself; `uwb_init()` prints the `raspi-gpio set` line to put in a
boot-time script.

This is the one pin map in use, shared with `probe/app/unix/config.h`
and with the production Linux application, so a Pi that already runs
either of those runs this with nothing rewired. It lives in
`app/unix/config.h`, and the SPI settings and the three GPIO
configurations are identical too.

## Building

```sh
sh sniffer/app/unix/build.sh                  # -> build/uwb-sniffer
```

The script reads the file list out of each tree's own manifest rather
than naming sources itself (`make -s sources` here and in bitters),
so neither list can go stale. `-n` prints the compiler command and
compiles nothing; `-o` names the executable.

It needs bitters, and looks for it at `$HOME/Repos/bitters` unless
`BITTERS=` says otherwise:

```sh
BITTERS=/elsewhere/bitters sh sniffer/app/unix/build.sh -o /tmp/sniffer
```

`DISSECTORS=` compiles a tree of dissectors in; it is read through the
same `make -s sources` interface, and
`sniffer/dissectors/ieee802154/Makefile` documents what such a tree must
export. Without it, no dissector is compiled in and the code that would
have called one is not compiled either.

`libpopt` (headers included) is the one other dependency, used by
`cmdline.c`. The target is Linux: bitters reaches the GPIO and SPI character
devices directly, so this does not build on anything else, and it is the
Pi that it is meant for.

`dissect.c` calls `dlopen(3)`, so the link takes `-ldl`. That is part of
libc on glibc 2.34 and later and on the BSDs, where the flag is harmless.

Only the receive half of the driver is compiled in
(`DW1000_SOURCES_CORE`, not `_SEND`, plus `DW1000_SOURCES_VALIDATE`):
`hw/drivers/dw1000/src/dw1000.c` never calls into
`hw/drivers/dw1000/src/dw1000_send.c`, and a sniffer never transmits.
That is the receive-only case `dw1000.cmake` documents.

The build is optimised (`-O2 -g`) rather than debug (`-g -O0`), because
how much of the radio's frame rate the host keeps up with is the one
number this program is judged on.
`OPTFLAGS='-O0 -g' sh sniffer/app/unix/build.sh` overrides it for a
build to step through.
