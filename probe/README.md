# Probe

An instrument, not a protocol. It runs one two-way-ranging exchange
between two nodes, writes down every timestamp the chip reported and
what those timestamps make of the distance, and exits. Or it runs one
node alone (transmitting, listening, or idle) and writes down what its
die temperature does meanwhile. Where a ranging
stack decides what to do about a measurement, this only records it, so
that when a stack disagrees with the bench there is something to compare
against that has no opinion.

This file is the *how*. [`DESIGN.md`](DESIGN.md) is the *why*: what the
exchange looks like, what each timestamp means, and why the pieces are
split the way they are.

## What is here

```text
probe/
  DESIGN.md              the reasoning
  include/dw1000/probe/  the API: record, role, settle, exchange, solo,
                         port
  src/                   role.c, record.c, settle.c
                                     (no <dw1000/dw1000.h>)
                         exchange.c  the two-node exchange
                         solo.c      tx, rx, temperature, and the
                                     read-backs (both drive the chip)
  port/{unix,zephyr,emulation}/src/port.c
  app/unix/              the Raspberry Pi application, and build.sh
```

The split is the point. `record.c`, `role.c` and `settle.c` are free of
`<dw1000/dw1000.h>`, so the line format and the settle rule can be
proved with no chip, no driver and no radio at all, which is what
`make tests` does. `exchange.c` and `solo.c` are not free of it: they
drive the chip. A port implements `include/dw1000/probe/port.h` and
nothing else.

The same `src/` runs under Zephyr: the Zephyr application
(`zephyr-redskin/probe/`) is this code with a different port and a
shell around it, not a second implementation. Every role and every
read-back lives here, so the two applications differ only in how a
value reaches the role; see [Two platforms, one instrument](#two-platforms-one-instrument).

## Running it

Build for a Raspberry Pi with a DW1000 attached:

```sh
sh probe/app/unix/build.sh                    # -> build/probe
BITTERS=/elsewhere/bitters sh probe/app/unix/build.sh -o /tmp/probe
```

`-n` prints the compiler command without compiling. It needs
[bitters][bitters] (default `$HOME/Repos/bitters`) and Linux, since
bitters reaches the GPIO and SPI character devices directly.

```text
probe [options] twr_init <own_addr> <peer_addr> <count>
probe [options] twr_resp <own_addr> <peer_addr> <count>
probe [options] tx <count>
probe [options] rx <seconds>
probe [options] --settle rx
probe [options] temperature [<seconds>]
probe [options] info | config | power

  own_addr    this node's address, hex accepted (0xc939)
  peer_addr   the peer's; twr_resp answers whoever polled it, so its
              value is unused, but required for symmetry
  count       exchanges (twr_*) or frames (tx) to run
  seconds     how long to listen (rx) or sample (temperature); a
              temperature run with none takes one reading

  every role:
  --power=<dB>|auto transmit power, 0..30.5 on the 0.5 dB grid
  --node=NAME       the name emitted lines carry (default: hostname)
  --dblbuff         double-buffered receive (default)
  --no-dblbuff      single-buffered, to compare against

  the radio, every role (defaults in brackets):
  --channel=N              1 2 3 4 5 7                     [5]
  --bitrate=KBPS           110 850 6800                    [6800]
  --prf=MHZ                16 64                           [64]
  --preamble=SYMBOLS       64 128 256 512 1024 1536 2048 4096  [128]
  --pac=SYMBOLS            8 16 32 64                      [8]
  --code=N                 preamble code, both ways        [10]
  --tx-code=N, --rx-code=N one way only
  --sfd=decawave|standard                                  [decawave]
  --antenna-delay=TICKS    both ways, device ticks         [16475]
  --tx-antenna-delay=TICKS, --rx-antenna-delay=TICKS

  one role (refused by the others):
  --ss              twr_*: two-frame single-sided estimate only
  --warmup=N        twr_init: uncounted exchanges first (default 5)
  --gap=MS          tx: between frames, on an absolute schedule
                    (default 10)
  --settle          rx: listen until the die settles, not for <seconds>
  --settle-window=S       window length (default 30)
  --settle-windows=N      windows whose means must agree (default 4)
  --settle-threshold=DEG  the spread of those means below (default 0.2)
  --settle-give-up=S      stop trying after (default 900)
                    each --settle-* implies --settle
```

The options come before the role; `--help` prints them and `--version`
the driver and bitters releases. The radio options are the sniffer's
too, spelt and valued the same (`make check` holds the two together),
so a radio set up for a probe run is given to the sniffer unchanged. The radio is validated by the
driver's own rules before the chip is touched, so a preamble code that
does not suit the PRF, say, is a usage error that names the reason. An
antenna delay is calibrated on one channel: on another, give the run
its own. `info`, `config` and `power` bring
the radio up, print what the chip holds and exit: the chip and lot id
with the OTP calibration references, the radio configuration read back,
and the applied transmit power. A role run prints the configuration
and the power first, so every run's log says what the radio was.

Start the responder first; it waits, so the order matters. Its count
is a ceiling: it stops when the initiator does.

```sh
# on the responder
probe --node=rpi-d twr_resp 0xd000 0xc000 1000

# on the initiator: it records
probe --node=rpi-c twr_init 0xc000 0xd000 30
```

Settling a node needs no peer, since its receiver being on is what heats
it; a transmitter heats the far end instead:

```sh
probe --node=rpi-d --settle rx                 # until settled, or 15 min
probe --node=rpi-c --gap=10 tx 3000            # 30 s of frames
```

[bitters]: https://gitlab.inria.fr/dalu/bitters

## Reading the output

**The initiator emits the records, not the responder.** It is the end
that ends up holding all six timestamps: it has its own three, and the
responder's arrive in the REPORT, the last frame, sent from responder
to initiator. A `TWR` line per counted exchange, then one `STATS` line:

```text
TWR seq=0 t_sp=… t_rp=… t_sr=… t_rr=… t_sf=… t_rf=…
    sym_mm=927.9 asym_mm=949.1 … exchange=ds ss_mm=1402.3
    status=ok node=rpi-c role=twr_init run=-
STATS role=twr_init … completed=30 of=30 heard=70 drop_unwatched=0 …
```

The responder prints `READY` before it listens and its own `STATS` line
at the end, `of=` being the POLLs it answered (the warm-up included)
and `completed=` the REPORTs it sent.

Three distance estimates, all in millimetres, and they do not agree to
the millimetre, which is the useful part:

| Field     | What it is                                         |
| :-------- | :------------------------------------------------- |
| `sym_mm`  | symmetric double-sided; assumes reply delays match |
| `asym_mm` | asymmetric double-sided; does not, and is tightest |
| `ss_mm`   | single-sided, from two frames; drift-sensitive     |

Every role's first line is `SETUP`, the radio the run had, read back
off the chip, so that runs at different settings stay apart in a
capture:

```text
SETUP channel=5 bitrate=6800 prf=64 preamble=128 pac=8 tx_code=10 rx_code=10 sfd=decawave tx_antd=16475 rx_antd=16475 tx_power_db=7.5 node=rpi-d role=twr_resp run=-
```

The roles a node runs alone emit `TEMP` lines instead: elapsed seconds,
the die in hundredths of a degree, the supply in millivolts, at the
start, once a second and at the end. Under `--settle`, `rx` emits a
`SETTLE` line per window in their place, closing on `state=settled` or
`state=gave-up`:

```text
TEMP 0.0 3012 3298 node=rpi-d run=-
SETTLE window=1 elapsed=30.0 mean=3105 spread=- threshold=20 state=unsettled node=rpi-d role=rx run=-
…
SETTLE window=9 elapsed=270.0 mean=3388 spread=14 threshold=20 state=settled node=rpi-d role=rx run=-
TEMP 270.1 3378 3297 node=rpi-d run=-
```

`spread` is the largest minus the smallest of the last four window
means, and the die is settled when it is below the threshold; why
means and not readings, and why the spread and not each step, is in
[`DESIGN.md`](DESIGN.md) and `include/dw1000/probe/settle.h`. Each role
ends with a summary line of its own: `tx: 3000/3000 started, 3000
completed`; `rx: N frames received over S s` and `rx: N frames REJECTED
by the chip`, the second separating corrupt frames from no frames.

A `status` other than `ok` says what was missing rather than dropping
the exchange silently: `no-response` (the POLL went, nothing came
back), `no-report` (the exchange went through FINAL and the REPORT
never arrived, which leaves no distance at all, every estimate needing
the responder's instants). The `drop_*` counters
on the `STATS` line separate "not heard" from "heard and rejected", and
`drop_overrun` is the only way a frame this program received is lost;
see the ring in `src/exchange.c`.

## Two platforms, one instrument

Both applications run the roles in `src/`; what differs is the shell
around them.

| | Raspberry Pi (`app/unix`) | Zephyr board (`probe` shell noun) |
| :-- | :-- | :-- |
| roles | `twr_init` `twr_resp` `tx` `rx` `temperature` | the same, plus `temp` as an old name for `temperature` |
| read-backs | `info` `config` `power` | the same, plus `radio`; `power <dB>` and `radio <options>` also set |
| radio | the options above | the same options, on a role or on `probe radio`; antenna delays default to 16433 |
| settings | node, addresses, radio, power, buffering hold for the process | `probe node`, `probe addr`, `probe radio`, `probe power`, `probe dblbuff` |
| role options | before the role | after it; also `--own=` `--peer=` |
| a setting lasts | one invocation, defaults every time | until changed: settings stick |
| run length | unbounded: the line buffer grows | the 64-line buffer: twr counts up to 60, `rx`/`temperature` up to 62 s, the settle give-up up to 62 windows |

A board refuses a run its buffer cannot hold rather than lose its end.
`tx` lasts as long as its frames take, so a board checks it against the
gaps alone and says afterwards if lines were lost.

## What `make tests` proves

`make tests-probe` runs two tests, neither needing hardware:

- **format**: the line format, the arithmetic behind it and the settle
  rule (a still die, a die dithering a whole LSB between readings, a
  steady climb the rule must refuse), against `port/emulation`, with no
  radio at all.
- **exchange**: the roles against `port/emulation`: that a responder
  run nobody polls *ends* rather than hanging, and that its `STATS`
  line says what it heard and why none of it was used; that an
  initiator, against a stand-in responder, resolves a four-frame and a
  single-sided exchange and records the responder's numbers as REPORT
  brought them, and records `no-report` and `no-response` when those
  frames do not come; that `tx` is
  told of every frame it sent, that `rx` counts received and rejected
  frames apart, that `rx --settle` ends on the rule's verdict, and that
  `temperature` samples as asked; and that the radio options take what
  the driver takes and refuse what it refuses, the combination
  included, and that the `SETUP` line follows a reconfiguration. It
  overrides the POLL budgets,
  because a gate cannot prove a wait terminates by waiting out a
  sixty-second one.

Neither application is built by `make tests`: the Pi's needs bitters and
Linux, the board's a Zephyr workspace.

Linking `exchange.c` into those is itself the check that its Zephyr
coupling is gone: it builds with no Zephyr tree in sight.

## Two things to know before trusting a number

**Buffering moves the answer.** Every combination of single and double
buffering at the two ends resolves, and each end's buffering moves
`asym_mm` by about two centimetres, about five with both ends switched
together ([`../AUDIT.md`](../AUDIT.md) has the measurement). Which mode
is right is not established, so the probe's absolute distances are not
calibrated to better than a few centimetres; compare runs taken in the
same mode.

**An antenna delay holds for one channel.** A run on another channel
needs its own `--antenna-delay` for its absolute distances to mean
anything.
