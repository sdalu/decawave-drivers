# Design of the UWB sniffer

Why the sniffer is shaped the way it is: the ring, the wire format, where
the metadata is read, which files are portable and which are not, and
what is deliberately left out. For building and running it, read
[`README.md`](README.md). The code this describes lives beside this
file, in `app/unix/`.

Code is pointed at by the name of a function or a rule rather than by a
line number. Line numbers into a file this document does not own rot
without anybody touching this one: two of them here were stale within a
day of being written, because the driver moved underneath them.

## Why a ring, not a slot

Frames are held in a ring (`capture.c`, sixteen entries) rather than in
one slot, and are forwarded from the ring by the loop in `main.c`,
never from the callback that received them.

The design is `probe/src/exchange.c`'s, which arrived at it the hard
way (commit `c3483f3`, "probe/exchange: hold received frames in a
ring, not one slot"): a monotonic count of frames captured, the slot
being that count modulo the ring's depth, and a producer that never
refuses. The sniffer's reason for wanting it is not probe's (probe had
gaps between waits in which a peer's answer could be discarded); it is
that forwarding costs far more than receiving. `eth_send()` is a
blocking `sendmsg(2)` and `pcapng_write()` a `write(2)`; a single slot
would mean the frame arriving while either of those is in flight
overwrites the one not yet sent. A ring of sixteen absorbs that instead
of losing it, and only falls behind when forwarding is more than
sixteen frames slower than the radio.

Unlike probe's ring, this one needs no lock. Probe's producer runs on
its port's event thread while a role thread consumes, so it takes the
bus lock around both ends. Here, producer and consumer are the same
thread: `capture_put()` runs inside the `rx_ok` callback, itself called
from `dw1000_process_events()`, and `capture_get()` runs from the loop
in `main.c` that called `dw1000_process_events()` and has returned from
it. bitters does run a GPIO IRQ thread, but it only dispatches
registered callbacks, and this program registers none: it polls the
interrupt's file descriptor itself. Were that ever to change,
`capture.c` is where the lock would go.

The ring's depth of sixteen was, until this change, a guess with no way
to check it: the TODO this document replaces said so outright. It no
longer is. `capture_stats()` reports `high_water`, the deepest the ring
was ever seen to be, alongside `overrun`, the count of frames the ring
lost by wrapping over them unread. A run whose high-water mark never
passes two has fourteen entries to spare; a run that overran knows by
how much. Sixteen is chosen to absorb "a scheduling hiccup on the
forwarding side", not tuned to a measured worst case, and deeper than
probe's four because the two rings absorb different things: probe
covers the one or two frames a two-node exchange makes in a gap, this
covers however many frames arrive while the host is inside `sendmsg(2)`
or `write(2)`, with every frame on the channel wanted.

## Why the read-out is in the callback, and the output is not

Under double buffering (the default; `--no-dblbuff` falls back to
single buffering) the driver re-enables the receiver in the good-frame
path *before* calling `rx_ok`, and toggles the host-side buffer pointer
the moment `rx_ok` returns. Two obligations follow, and they are the
whole shape of `capture_put()`: the callback must not re-enable the
receiver itself, since the driver already has; and everything the frame
is wanted for must be read out before the callback returns, because
`RX_BUFFER` swings with the pointer, and so, in double-buffered mode, do
`RX_TIME`, `RX_FQUAL`, `RX_TTCKI` and `RX_TTCKO`.

"Everything the frame is wanted for" used to mean only the payload. It
now also means the metadata below, which is why `capture_ops::read_meta`
exists as a second, separate read-out rather than being folded into
`read_frame_data()`: a run that wants the highest frame rate can leave
it out (`--no-metadata`) and spend no time on registers nobody asked
for.

The output is the opposite case: `eth_send()` and `pcapng_write()` are
blocking calls into the kernel, and doing either inside `rx_ok` would
hold the callback open across that call while the next frame is already
landing in the other buffer. So the callback does only the read-outs and
returns; the loop in `main.c` drains the ring afterwards, outside the
callback, where a blocking call costs the forwarding path time and
nothing else.

```text
        DW1000, double buffered receiver
                  │
                  │  rx_ok(status, length, ranging)
                  ▾
        ┌───────────────────┐
        │   capture_put()   │
        └───────────────────┘  inside the callback; must not block
                  │
                  ▾
        ┌───────────────────┐
        │    rx_ring[16]    │
        └───────────────────┘  capture.c
                  │
                  │  capture_get(), oldest first
                  ▾
        ┌───────────────────┐
        │ loop()  (main.c)  │
        └───┬──────────────┬┘
            │              │
            ▾              ▾
      wire_encode() pcapng_write()
            │              │
            ▾              ▾
       eth_send()    file / stdout
```

## The wire format, and why it looks like that

The ethernet payload used to be the captured frame and nothing else.
That loses four things. A frame shorter than 46 bytes cannot be
recovered at all, because the NIC pads to `ETH_ZLEN` and nothing says
where the frame actually stopped, which on 802.15.4-shaped traffic is
most of the short frames. A frame the ring had to truncate goes out
looking whole. There is nowhere to put what the chip knew about the
frame. And nothing numbers the frames, so a receiver cannot see the gap
an overrun left.

So captured frames now carry a fixed 32-byte header ahead of them,
little endian, with the magic `UWBS`, a version byte, and an optional
32-byte metadata block after it (`wire.h` gives the byte-for-byte
layout: `wire_encode()` in `wire.c` is the encoder, one field at a time). Two
choices in it are deliberate rather than obvious.

It is written byte by byte, never by writing a packed struct straight
into the buffer. A struct says the same thing on the two machines this
happens to run between today and a different thing on the next one:
padding, alignment and byte order are all the compiler's to choose, and
a wire format is exactly what must not depend on them.

The two power estimates are milli-dBm integers, not the doubles the
driver itself reports them as. A receiver on the other end would have
to share this program's NaN and infinity conventions to make sense of a
float; an integer sentinel (`CAPTURE_POWER_NONE`, distinguishing "no
usable estimate" from a genuine reading of zero) needs no such
agreement.

Nothing about this format is a one-way door. `--raw` still sends the
bare frame, exactly as every earlier build of this program did, for a
receiver already written against that output; the magic is what lets a
receiver on the other end tell which of the two it has been handed, and
the version byte is what lets a field be added later without the magic
having to change.

## Where the metadata is read, and why

Per-frame metadata (the RMARKER arrival timestamp, the two power
estimates, the transmitter's clock offset and interval, the first-path
index and the two noise figures) is read inside the `rx_ok` callback,
through `capture_ops::read_meta`, and carried alongside the frame in
`struct capture_meta`.

It has to be read there or not at all. `RX_TIME`, `RX_FQUAL`,
`RX_TTCKI` and `RX_TTCKO` all swing with the double buffer pointer,
which the driver toggles the moment the callback returns
(`hw/drivers/dw1000/README.md`, "Double buffered receive"; the same
note is in `capture.h` and in `app/unix/uwb_dw1000.c`'s `_read_meta()`). It used
to be read nowhere, so the sniffer could say what a frame contained and
nothing about how it arrived. It is on by default, and costs about ten
register reads a frame; `--no-metadata` buys that back for a run that
wants the highest frame rate instead, and the frame then carries no
`CAPTURE_F_METADATA` rather than a block of zeroes a consumer could
mistake for a reading.

Two things this program could report and deliberately does not.
`dw1000_rx_power_correction()` is not applied before reporting: it maps
the raw estimate onto UM §4.7 fig 22's curve, and applying it here
would leave a consumer with no way back to what the chip actually said.
Reporting is this program's job; the curve is the consumer's to apply.
And the clock drift is not reported as the ratio
`dw1000_rx_get_clock_drift()` returns: that is a double whose zero
means both "no drift" and "no reading yet" (`RX_TTCKI` reads 0 before
the first frame is demodulated), so the offset and the interval are
carried separately instead, and dividing is left to whoever reads the
capture.

## Four portable files, four Pi-only ones

`capture.c` reaches the chip only through `struct capture_ops`, a pair
of function pointers, rather than calling `uwb_read_frame_data()`
directly. That seam is what makes `capture.c`, `wire.c`, `pcapng.c` and
`dissect.c` buildable and testable with no chip and no Linux: none of the three
calls into the driver, into bitters, or onto an `AF_PACKET` socket. They
do reach `<dw1000/dw1000.h>`, by way of `capture.h`, for
`DW1000_FRAME_MAXSIZE` and nothing else, which is why
`tests/check-sniffer.sh` passes the driver's include directory and the
emulation port's while linking not one driver source file. `tests/sniffer/capture.c` drives the ring and the
wire encoder with its own fake `capture_ops` pair; `tests/sniffer/pcapng.c`
drives the pcapng writer with frames built by hand;
`tests/sniffer/dissect.c` drives the dissector registry and loads two
shared objects it builds for the purpose; all three are run by
`tests/check-sniffer.sh`, itself run by `make check`'s `check-sniffer`
target (the `check` and `check-sniffer` rules in the `Makefile`). The sniffer's only real logic (the
ring, its overrun accounting, its truncation, the wire encoding, the
pcapng block layout) used to be unreachable from any test, and now is;
`tests/check-emulation.sh` buys the driver the same thing, and
`tests/probe/format.c` the probe's line format, for the same reason.

The other four translation units (`main.c`, `cmdline.c`, `eth.c`,
`app/unix/uwb_dw1000.c`) build only on Linux, and only make sense on a Pi:
`eth.c` wants `AF_PACKET` and Linux's `struct ifreq`, `cmdline.c` wants
`libpopt`, and `app/unix/uwb_dw1000.c` and `main.c` want bitters and the Linux
GPIO and SPI character devices. `sh sniffer/app/unix/build.sh -n` shows
the real compiler invocation; only a Raspberry Pi can run it for real.

## Two loss accounts, kept separate

A run reports two counts of lost frames, not one, because they are
fixed by different things.

| Count | Kept in | What it means |
| --- | --- | --- |
| ring overrun | `capture_stats()->overrun` | forwarding (this program) fell behind the radio: the ring wrapped over frames not yet sent |
| chip overrun (`RXOVRR`) | `uwb_rx_errors()->overrun` | the host fell behind the *chip*: both buffers were still held when a third frame arrived, so the chip dropped it |
| the rest of the receive errors (`phy`, `fcs`, `sync`, `lde`, `sfd_timeout`, `rejected`) | `uwb_rx_errors()` | a frame never decoded at all; a link problem, not a host-speed one |
| `unexplained` | `uwb_rx_errors()->unexplained` | a status word matching none of the bits above; a run where this is not zero is a run to go and look at the driver for |

A ring loss is the host being too slow to *forward*; a chip overrun is
the host being too slow to *read out*. They are fixed by different
things, so a sniffer that folded them into one count would be asking to
be trusted about the frames it did not report without saying why they
went missing. The receive-error status word used to be discarded
outright in the `rx_error` callback; classifying it
(`app/unix/uwb_dw1000.c`'s `_rx_error()`) is what makes the second half of this
table possible at all. `capture_stats()->high_water`, alongside
`overrun`, is the same idea applied to the ring: together they turn
"is sixteen deep enough" from a guess into a measurement.

## pcapng, not classic pcap

`-w FILE` (`-w -` for stdout) writes captured frames locally as pcapng
instead of, or as well as, sending them over ethernet. Before this
change the only output was an ethernet frame that no dissector
understood; pcapng is what wireshark and `tcpdump -r` read with no
plugin needed.

pcapng, not classic pcap, because classic pcap has no per-packet option
space, and the metadata above would have had nowhere to go. In pcapng
it rides in each Enhanced Packet Block's `opt_comment` (option code 1),
which wireshark shows as the packet's comment (`pcapng.c`'s
`format_meta_comment()` is the one line it formats); a frame with no
metadata carries no option list at all rather than an empty one.

The link type this writer declares is 195
(`LINKTYPE_IEEE802_15_4_WITH_FCS`), not 230 (the `_NOFCS` variant),
because the frame the driver reports, and this program forwards, includes
the CRC: nothing here strips it, matching `capture.h`'s note that a
sniffer forwards the frame whole. `if_tsresol` is written as 9
(`write_idb()` in `pcapng.c`), because the timestamps this writer gives
every packet
are nanoseconds (`frame->wall`, `CLOCK_REALTIME` at read-out) and
pcapng's own default resolution is microseconds; leaving it unstated
would silently understate the precision actually written.

Because `-w` gives the capture somewhere to go, `<dst_macaddr>` is now
optional rather than required (`cmdline.c`'s destination check, and the
`dst_valid` flag `main.c` reads before opening a socket at all). That is
what makes `uwb-sniffer -w - <dst_macaddr> | wireshark -k -i -` a
one-host affair with no ethernet socket opened.

The writer emits exactly the three block types pcapng-spec.md describes
(Section Header, Interface Description, Enhanced Packet), in the host's
own byte order, and nothing else; `pcapng.c`'s own header comment names
this explicitly, and it is what keeps the second half of `check-sniffer`
provable with no chip.

## stdout carries capture data, and nothing else

Every informational line this program prints (`-v`'s per-frame line,
the reception account, every warning, the boot-time `raspi-gpio` hint,
the tcpdump hint) goes to stderr, never stdout; `config.h`'s `INFO` and
`WARN` macros are both defined that way. That used to be harmless, when
the only thing this program produced was ethernet frames; it is not
harmless with `-w -`, where stdout *is* the pcapng stream, and one line
of English in the middle of it makes the file unreadable from that byte
on. This is the convention `tcpdump(1)` has always followed, for the
same reason. The one exception is `-V`'s version line, which is printed
and the process exits before any capture file is opened, so it can
never land inside one.

## The receive loop: bounded draining and a clean stop

`dw1000_process_events()` takes one snapshot of the status register and
handles at most one frame from it, returning whether it did
(`dw1000_process_events()` in `hw/drivers/dw1000/src/dw1000.c`). The loop
used to call it once
per wake-up and discard that return value; with two frames sitting in
the two buffers, one of them would wait for the next interrupt. It is
now called until it returns false, bounded at `DRAIN_MAX` (16)
iterations. Bounded, not a bare `while`, because the condition being
waited on is a bit in a register on the other end of an SPI bus: a
status bit nothing here knows how to clear would turn an unbounded loop
into a program that never returns to `poll(2)`. Sixteen is chosen
because it is the ring's own depth, so one wake-up can never drain more
than the ring could hold regardless.

The `poll(2)` timeout went from infinite to one second
(`POLL_TIMEOUT_MS`), the same figure and the same reasoning as
`probe/app/unix/main.c`'s `PROBE_EVENT_WAIT_TIMEOUT_US`
(`PROBE_EVENT_WAIT_TIMEOUT_US` in `probe/app/unix/main.c`, one second). It is not how a wake-up is
expected to arrive; every real one arrives on the interrupt's file
descriptor. It is a bound on how long a missed edge could go unnoticed.

`SIGINT` and `SIGTERM` now set a flag the loop reads (`_on_signal()`,
without `SA_RESTART` so that `poll(2)`'s `EINTR` return is how the loop
notices promptly), rather than killing the process outright. Ctrl-C
used to discard the whole reception account with the process; now the
run prints it on the way out regardless of how it stopped. `--count N`
stops the loop itself after `N` frames have been forwarded; `--stats N`
prints the same account periodically, on a count moving rather than on
every frame, so a burst of losses does not itself cost the loop time.

## Frame filtering: off, and no switch to turn it on

`main.c` disables hardware frame filtering explicitly
(`uwb_rx_set_frame_filtering(0)`), where before it was simply never
enabled, relying on the chip's own reset state. It is the one setting
that, if it were ever on, would make this program silently deaf to
every frame not addressed to it: a sniffer that reports nothing and no
error, which is the worst failure a capture tool can have.

A `--filter` flag was written, and then removed before it shipped.
Hardware filtering matches against the PAN id and the addresses in
`PANADR` and `EUI` (UM §5.2.1), and this program never writes either,
so the flag's only possible effect was to make the sniffer deaf.

## The build: optimised by default

`build.sh` now compiles at `-O2 -g`, where it used to compile at
`-g -O0`. Between the SPI read-out inside `rx_ok` and the `sendmsg(2)`
or `write(2)` that follows it there is nothing else running, and how
much of the radio's frame rate the host keeps up with is the one number
this program is judged on; shipping it unoptimised spends that for
nothing. `-g` stays, because a sniffer that stops sniffing on a bench is
debugged where it stands; `OPTFLAGS` overrides both for a build to step
through.

## Dissectors: two routes, one interface

A dissector is code from outside this program that is shown each captured
frame and may do two things with it: write one line about it (for `-v` and
for the pcapng packet comment) and say whether it should be forwarded at
all. It may do nothing else. The frame is `const`, there is no hook that
alters what is written or sent, and a capture tool whose plugins can
rewrite the capture is not a capture tool.

Two routes supply one, and the interface (`dissect.h`) is the same either
way:

| Route | Selected by | Good for |
| --- | --- | --- |
| compiled in | `DISSECTORS=<tree>` at build time | the dissector you always want, in the repository that owns the protocol |
| loaded | `--dissector=PATH[:args]` at start-up | iterating without rebuilding, and shipping one binary |

`DISSECTORS=` names a tree that exports `make -s sources`, exactly as the
driver and bitters already do for `build.sh`; it sets `DISSECT_SOURCES`
and `DISSECT_INIT`, the *name* of the function that calls
`dissect_register()`. `build.sh` passes that name through as
`-DSNIFFER_DISSECT_INIT=`, and `main.c` declares it `extern` and calls it
under `#ifdef`. So neither `build.sh` nor `main.c` knows a protocol, and
the tree that owns the frame layout is the tree that lists its own files.
`sniffer/dissectors/ieee802154` is a worked example and the interface's
reference.

### Why the interface mentions no driver type

This is the constraint the whole of `dissect.h` is shaped by.
`struct capture_frame` embeds `uint8_t data[DW1000_FRAME_MAXSIZE]`, which
is 127 bytes or 1023 depending on `DW1000_WITH_PROPRIETARY_LONG_FRAME`.
Hand one of those across a `dlopen` boundary and a plugin built with a
different option set reads a struct of a different shape, with no symptom
until the numbers come out wrong. The root `README.md` refuses to ship the
driver as a shared library for exactly this reason, and a dissector
interface carrying a driver type would have walked into it.

So a dissector sees `struct dissect_frame`: a pointer, a length, and
fixed-width fields. `capture_to_dissect()` in `capture.c` is the one place
the two representations meet, which is why it is there rather than in
`main.c` where its only caller is, and why `tests/sniffer/capture.c`
checks the mapping. It carries both lengths, because a dissector walking a
variable-length payload must bound itself by what arrived and not by what
the driver said arrived, and it sets the radio fields to explicit
"nothing was read" values when the capture ran without the metadata
read-out, so that a plugin cannot mistake a stale slot for a measurement.

### Why the registration call is in a file of its own

`dissect_register()` lives in the sniffer. `dissect.c` loads plugins with
`RTLD_NOW`, and the executable exports nothing (the interface is one way,
so there is no `-rdynamic`). A shared object that calls
`dissect_register()` therefore does not load at all:

```text
Undefined symbol "dissect_register"
```

which is a puzzling way to discover an architectural rule. The example
keeps that one call in `register.c`, which `DISSECT_SOURCES` includes and
`make plugin` leaves out; the dissector itself references nothing in the
sniffer and builds both ways unchanged. `register.c`'s own comment is the
place this is written down for whoever copies it.

Two smaller decisions in the loader, both for the same reason. The symbol
a plugin must export is versioned in its *name*
(`uwb_dissector_v1`), so a plugin built against a later interface fails to
load rather than loading and being misread; and a `--dissector=` argument
with no `/` in it is refused rather than passed to `dlopen(3)`, which
would search the library path for it. This program runs privileged on a
Pi, so a bare name is an invitation to load whatever a search path turns
up first.

### Where a dissector runs, and what it costs

In `_forward()`, on the forwarding loop's thread, and nowhere else. Never
in the receive callback: a dissector is exactly the unbounded work the
ring exists to keep out of `rx_ok` while the next frame lands in the other
buffer. `accept()` runs first, so a frame nothing wants costs neither a
`describe()` nor a syscall, and `describe()` runs only when its line has
somewhere to go.

A slow dissector is therefore measurable rather than mysterious: it shows
up as the ring's high-water mark climbing toward sixteen, and then as the
overrun count moving. That is the number to decide by, and it is the
reason `--dissect-filter` is off by default. Filtering trades dissector
time for the `sendmsg(2)` it avoids, and which way that trade goes depends
on the traffic; the counters are how a bench answers it rather than a
guess written here.

With `--dissect-filter` and dissectors that have no `accept()` hook, every
frame is forwarded. That is deliberate: the alternative reading, "nobody
accepted it, so drop it", turns a capture into silence for a
misconfiguration, and silence is the worst failure a capture tool has.

## The wireshark dissector, and what it is not

`sniffer/wireshark/uwbs.lua` is a wireshark dissector for the *ethernet*
output: it reads the `UWBS` header, exposes every field as something to
filter and sort on, and hands the frame to wireshark's own 802.15.4
dissector. The pcapng output needs nothing of the sort, because link type
195 is already understood; this closes the gap on the other path, which
`README.md` used to describe as bytes nothing understood.

It reads the payload's own protocol no further than 802.15.4, and
deliberately. A Lua dissector for a protocol whose layout is defined in C
somewhere else is a second copy of that layout in a second language,
which is a copy that drifts. `DISSECTORS=` exists so that the dissector
for such a protocol can `#include` the definition instead.

### How it is tested without wireshark

In two layers, because they need different things installed.

`tests/check-wireshark.sh` needs nothing at all. It reads `wire.h`'s
table and the Lua's `tvb()` calls with awk and checks that every offset
the script reads is a documented field and every documented field is
read. That catches the drift that actually happens, a field added to the
header and not to the script.

`tests/check-lua.sh` needs a Lua interpreter, and skips cleanly without
one. It parses the script with every `luac` it can find, then runs it:
`tests/lua/wsstub.lua` fakes the part of wireshark's Lua API the script
uses, `tests/lua/emit.c` encodes a header with the real `wire_encode()`
and prints both the bytes and the values that went into them, and
`tests/lua/uwbs_test.lua` feeds the bytes to the dissector and compares
field by field. So what is compared is the C encoder against the Lua
decoder, with nothing transcribed from `wire.h` twice; a test with a
hand-typed header would have agreed with itself.

What makes that worth the code is the endianness. The stub honours
wireshark's distinction between `tree:add()` (big endian) and
`tree:add_le()` (little endian), so reading a field the wrong way round
on a little-endian wire format fails here rather than silently in the
field. Nothing else in the harness catches a whole class of bug the way
that one detail does.

### Lua 5.4 is the minimum, deliberately

The script uses the bitwise operators, so it needs Lua 5.3 or later, and
the line is drawn at 5.4 because that is what wireshark 4.4 and later
build against. Supporting 5.2 is possible (a single-bit test is
`(v % (m + m)) >= m` in arithmetic, and the script was written that way
first) and it was given up on purpose: it costs a helper and a paragraph
explaining why the helper exists, to keep a promise nothing here runs
against and so nothing here checks. A version claim no test covers is
worse than no claim.

The cost is worth stating plainly, because the failure is not graceful.
A wireshark built against Lua 5.2 does not misread the script, it fails
to parse it, and the error it reports names the operator rather than the
version. The script's header comment says so, so that whoever reads
`')' expected near '&'` finds the answer next to it.

`tests/check-lua.sh` enforces the same minimum, and asks each candidate
its version rather than trusting its name: an unversioned `lua` that
turns out to be 5.2 is passed over rather than reported as a failure,
since an old interpreter on the machine the tree happens to sit on says
nothing about the sniffer. A version named explicitly through `LUA=` or
`LUAC=` is different, and one below the minimum is an error there,
because it was asked for.

Neither layer is the real thing. The stub was written from the same
documentation the dissector was, so a misunderstanding shared by both
passes here and fails in wireshark: testing against a mock proves the
mock agrees. What is still owed is opening a capture in wireshark once by
hand.

Where Lua lives differs by system, so nothing is hardcoded: `LUA` and
`LUAC` override the search, whose defaults cover both Linux's `lua5.4`
naming and FreeBSD's `lua54`. No Lua include directory is wanted
anywhere, because the harness is Lua rather than a C program linked
against liblua; writing the stub twice, once against the Lua C API, would
buy no coverage.

## Deliberately out of scope

**This program never transmits.** `build.sh` takes
`DW1000_SOURCES_CORE` without `_SEND`:
`hw/drivers/dw1000/src/dw1000.c` never calls into
`hw/drivers/dw1000/src/dw1000_send.c`, so the send half of the driver
is simply not in the binary. That is the receive-only case
`dw1000.cmake` documents.

**It does not itself know any protocol.** Every path through this
program (`capture.c`, `wire.c`, `pcapng.c`, `eth.c`) treats a frame's
bytes as an opaque blob to copy, count and forward. Dissection happens
only through the interface below, in code that came from somewhere else,
and the full decode is still the receiving wireshark's job, which is most
of why the pcapng path exists.

**It does not capture frames that failed their CRC.** The driver's
`rx_error` callback carries only a status word
(in `hw/drivers/dw1000/include/dw1000/dw1000.h`,
`void (*rx_error)(dw1000_t *dw, uint32_t status)`); it
is never handed the buffer a bad frame arrived in, so there is nothing
here to forward for one. A count of *how many* such frames arrived is
available (the loss-account table above); the frames themselves are
not.

**It is not a ranging instrument.** It carries no exchange protocol, no
two-way timing, and no distance computation; `probe/` is the instrument
built for that, and its own design document explains why that is kept
separate from the production stack as well.
