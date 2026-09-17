# Probe

An instrument, not a protocol. It runs one two-way-ranging exchange
between two nodes, writes down every timestamp the chip reported and
what those timestamps make of the distance, and exits. Where a ranging
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
  include/dw1000/probe/  the API: record, role, exchange, port
  src/                   role.c, record.c   (no <dw1000/dw1000.h>)
                         exchange.c         (drives the chip)
  port/{unix,zephyr,emulation}/src/port.c
  app/unix/              the Raspberry Pi application, and build.sh
```

The split is the point. `record.c` and `role.c` are free of
`<dw1000/dw1000.h>`, so the line format can be proved with no chip, no
driver and no radio at all, which is what `make check` does.
`exchange.c` is not free of it: it drives the chip, so it is its own
translation unit. A port implements
`include/dw1000/probe/port.h` and nothing else.

The same `src/` runs under Zephyr: the Zephyr application is this code
with a different port and a shell around it, not a second
implementation.

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
probe [options] <role> <own_addr> <peer_addr> <count>

  role        twr_init | twr_resp
  own_addr    this node's address, hex accepted (0xc939)
  peer_addr   the peer's; twr_resp answers whoever polled it, so its
              value is unused, but required for symmetry
  count       exchanges to run

  --ss              two-frame single-sided estimate only
  --warmup=N        twr_init: uncounted exchanges first (default 5)
  --power=<dB>|auto transmit power, 0..30.5 on the 0.5 dB grid
  --node=NAME       the name emitted lines carry (default: hostname)
  --dblbuff         double-buffered receive (default)
  --no-dblbuff      single-buffered, to compare against
```

Start the responder first; it waits, so the order matters:

```sh
# on the responder
probe --node=rpi-d twr_resp 0xd000 0xc000 30

# on the initiator
probe --node=rpi-c twr_init 0xc000 0xd000 30
```

[bitters]: https://gitlab.inria.fr/dalu/bitters

## Reading the output

**The responder emits the records, not the initiator.** It is the end
that ends up holding all six timestamps: it has its own three, and the
initiator's arrive in the REPORT. A `TWR` line per exchange, then one
`STATS` line:

```text
TWR seq=0 t_sp=… t_rp=… t_sr=… t_rr=… t_sf=… t_rf=…
    sym_mm=882.1 asym_mm=766.7 … exchange=ds ss_mm=2263.8
    status=ok node=rpi-d role=twr_resp run=-
STATS role=twr_resp … completed=20 of=20 heard=60 drop_unwatched=0 …
```

Three distance estimates, all in millimetres, and they do not agree to
the millimetre, which is the useful part:

| Field     | What it is                                         |
| :-------- | :------------------------------------------------- |
| `sym_mm`  | symmetric double-sided; assumes reply delays match |
| `asym_mm` | asymmetric double-sided; does not, and is tightest |
| `ss_mm`   | single-sided, from two frames; drift-sensitive     |

A `status` other than `ok` says what was missing rather than dropping
the exchange silently. `no-report` is the common one: the responder
heard the POLL and the FINAL and never the REPORT. The `drop_*` counters
on the `STATS` line separate "not heard" from "heard and rejected", and
`drop_overrun` is the only way a frame this program received is lost;
see the ring in `src/exchange.c`.

## What `make check` proves

`make check-probe` runs two tests, neither needing hardware:

- **format**: the line format and the arithmetic behind it, against
  `port/emulation`, with no radio at all.
- **exchange**: the responder against `port/emulation`: that a run
  nobody answers *ends* rather than hanging, and that its `STATS` line
  says what it heard and why none of it was used. It overrides the POLL
  budgets, because a gate cannot prove a wait terminates by waiting out
  a sixty-second one.

Linking `exchange.c` into those is itself the check that its Zephyr
coupling is gone: it builds with no Zephyr tree in sight.

## Two things to know before trusting a number

**The responder must be double buffered.** Single-buffered it hears the
POLL and the FINAL and never the REPORT, so nothing resolves and every
record reads `no-report`. This is not a preference; see
[`../AUDIT.md`](../AUDIT.md).

**The initiator's buffering moves the answer**, by a couple of
centimetres on `asym_mm`. Small, consistent, and not yet attributed, so
the probe's absolute distances are not calibrated to better than a
few centimetres. `../doc/bench/` holds the raw output behind both.
