# The probe: a measurement-only instrument

What the probe is, why it exists apart from the production stack, and
the contract its record and its exchange keep. The code it describes
lives beside this file: `probe/src/`,
`probe/include/dw1000/probe/`, and the Unix application in
`probe/app/unix/`. For building and running it, read
[`README.md`](README.md).

This document is design and contract. The campaign that used the
instrument (the stages, the measured results and the bench-side
findings) is `PROBE-CAMPAIGN.md` in the bench repository. Numbers
quoted here are quoted because a rule rests on them; the record of what
was measured when is there, and the driver-side record is
[`../AUDIT.md`](../AUDIT.md).

The Zephyr application's own hard-won facts (the CDC-ACM console
wedge, one console attachment per role, the console baud in the board
overlay, stack sizes, immediate logging) are properties of that
application and live with it, not here.

## Why there is an instrument at all

A pair-baseline measurement wants the same numbers from every link:
received signal strength, first-path strength, both dies' temperature,
the distance as measured from each end separately, and the link's
asymmetry. Ruby roles on a Linux host with a DW1000 can produce all of
that. A board running the production stack can produce one number, the
count of exchanges that resolved, because no board speaks the
instrument's exchange.

That gap is what makes counts alone useless as evidence, and three
effects show why:

- **Resolved counts cannot explain themselves.** Two pairs at
  essentially the same distance, with every other radio off, can differ
  by an order of magnitude in exchanges resolved. Nothing in a count
  says whether that is the antenna, the placement or the stack.
- **Transmit power matters enormously at short range**, and not
  monotonically: on a pair at bench distances, completions can fall to
  zero in both directions at the top of the power range while a low
  setting completes most of them. Received power tracks the setting, so
  the effect is real. A board whose transmit power is fixed at build
  time cannot be swept at all.
- **Die temperature is not stationary during a measurement.** Antenna
  delay drifts with it, so a measurement taken on a chip that is still
  warming is not the measurement you wanted. Boards report no
  temperature.

What heats the chip is the **receiver being enabled**, not traffic. A
chip with its receiver on and nothing at all arriving climbs by several
degrees a minute; the same chip with the radio off cools; a transmitter
sending thousands of frames barely moves. The rate is
board-dependent, and by more than a factor of two between board types,
so the two ends of a cross-type pair do not merely settle at different
temperatures; they approach them at different speeds.

Two consequences follow, and they are why the settle rule below is what
it is. A node can settle itself, alone: no peer, no transmitter, no
coordination, each end simply enables its receiver and waits for its own
temperature to stop moving. And receiver-on time is a systematic error
source for every ranging measurement on the bench, not only the probe's.

So the probe exists to make every node answer the same questions, by
running the same instrument everywhere.

## What it is, and what it is not

**It talks only to other probes.** It does not implement the production
ranging protocol and does not interoperate with the production firmware
on air. This is settled, not open, and because the probe core runs on
a Linux host as well as on a board, it costs nothing: every node can be
a probe, so every pair is a probe pair. The Ruby TWR roles stop being
the thing to imitate and become the independent cross-check, which is a
better job for them.

The consequence is deliberate. The probe measures the *radio path*
(antennas, placement, power, temperature) and not the production
firmware's behaviour on that path. For "is this link any good" that is
the right instrument. For "why does the production stack do badly on
this link" it is the wrong one, and such questions stay with the stacks.

That constraint is what keeps the probe small, and small is the point.
The production stack is the thing under test, and a single bench session
has found a protocol defect costing a third of all exchanges between two
nodes, a received-power estimate off by a fraction of a dB, and a stale
transmit completion handing back the previous frame's timestamp. An
instrument sharing that code cannot tell you whether a bad number is the
link or the stack. A probe implementing one fixed exchange has very
little to be wrong about.

So it carries no ranging protocol, no UAV layer, no host-link library,
no accelerometer, no GUI and no sniffer.

### The independence boundary, stated honestly

The probe shares **the DW1000 driver, and nothing above it.**

- **Shared, and unavoidably so:** the register layout, the timestamp
  path, the receive-power estimate, the transmit power encoding. Any
  instrument for this chip goes through a driver, and writing a second
  one would be a larger thing to be wrong about than the one being
  avoided.
- **Not shared:** the production stack in its entirety. No protocol
  core, no ranging state machine, no ranging history, no OSAL of its
  own, no syslog of its own.

The honest limitation that follows: **a driver-level defect is invisible
to this instrument.** When the probe and the production stack disagree,
the probe is evidence; when they agree, they may be agreeing on a driver
bug. [`../AUDIT.md`](../AUDIT.md) is where that risk is managed, not
here.

## Where the probe lives

The probe core lives in this repository, beside the driver it
characterises, with one thin shell per platform outside it.

```text
  decawave-drivers/
      probe/                     the core. Chip-generic mechanism.
      probe/port/<os>/           host glue: clock, wait, line sink
      probe/app/unix/            the Linux shell: pin wiring, argv
      hw/drivers/dw1000/         the driver (shared, deliberately)
      port/<os>/                 the driver's own OSAL ports
      docs/                      aps011, aps013, aps014, aps006 …
```

Three reasons, all of them already true of the repository:

1. **The subject matter is already here.** `docs/` holds the
   application notes on sources of error in two-way ranging, on two-way
   ranging itself, on antenna delay calibration and on channel effects.
   That is the probe's entire bibliography, sitting next to an
   `AUDIT.md` that audits the driver against the User Manual.
   Everything the probe measures is chip characterisation, not swarm
   protocol.

2. **The portability is already here.** `dw1000.cmake` declares seven
   OSAL ports, written and in use, and the driver already builds
   standalone against `OSAL=unix`. So a Linux port of the probe costs a
   `main.c`, not a port.

3. **One commit is one instrument.** A measurement is interpretable
   only if you know which driver produced it, and the driver is partly
   what is under suspicion. Co-located, the pairing cannot be forged.

### Why not in the production stack's tree

Investigated and rejected on evidence: that tree's lower layers cannot
be taken without its core. Its OSAL calls into the core's syslog, its
hardware-abstraction contract includes a core header, and its Zephyr
build wraps everything in one config symbol with the core sources
unconditional. No configuration takes the OSAL without the core.

And this is policy, not accident: that tree's own design document calls
the core indivisible. Detangling it to host an instrument would mean
overturning that, in the tree where the protocol under test is actively
being changed. This repository's Zephyr module is all-or-nothing in the
same way, but here all-or-nothing yields exactly the driver the probe
wants, so the same mechanism has the opposite consequence.

### Naming: `dw1000_probe`, not `probe`

The component is `probe/` as a directory, but everything it exports
carries the driver's prefix: `<dw1000/probe/record.h>` on the include
path, `dw1000_probe_*` for C symbols and types, `DW1000_PROBE_*` for
CMake variables, `CONFIG_DW1000_PROBE` for Kconfig. That mirrors what
the driver's ports already do, and it is not only a convention point.

A bare `probe` collides with the bench's own vocabulary, where *probe*
means the **debug adapter**, the CMSIS-DAP or J-Link. Naming the
instrument `probe` too would produce sentences like "flash the probe
onto the board through the probe". There is no compile-level clash in
Zephyr, which is exactly what makes this worth writing down: the
collision is in comprehension, so nothing would ever have errored.

`.probe` being the universal driver-init verb in Zephyr and Linux idiom
is the second reason a `probe_*` prefix inside a *driver* library reads
wrong.

The board's shell noun stays `probe`: at a board's console there is no
debug adapter to confuse it with, and the ambiguity lives in the
flashing vocabulary, not the console one.

### The split line: mechanism, host, values

This repository is a general library with a licence, version tags and
seven ports. Bench specifics must not leak into it. But a two-way split
of "chip-generic versus bench-specific" is the wrong shape, for two
reasons:

- Most of what a second bench would change is neither code nor format
  but **values**: addresses, radio configuration, antenna delays,
  counts, gaps, timeouts, settle thresholds. Their type, unit, default
  and validation are chip facts; only the value is the bench's.
- The driver's port contract offers a delay, the IO lines, SPI and an
  assert, and **nothing else**. No monotonic clock, no
  wait-for-interrupt-or-timeout. The exchange state machine needs both,
  and every consumer that has needed them hand-rolled its own condition
  variable and `clock_gettime` loop. That is **host glue**, not bench
  knowledge, and putting it in the shells would mean a second lab
  rewriting plumbing that has nothing to do with its bench.

So the line is three-way:

> **Mechanism in `probe/`; host glue in `probe/port/<os>/` beside the
> driver's ports; values in the shell.** Every state machine, formula,
> format, buffer and rule is chip-generic. Every value it needs is a
> parameter whose type, unit, default and validation live in `probe/`
> and whose value comes from the shell. What the host must supply to run
> it (a clock, a wait, a line sink) is a port and knows nothing
> about the bench. The shell is what is left: choosing values, wiring
> pins, and naming things for the people who run the bench.

**`probe/` holds the mechanism**: the exchange state machine, the frame
layout and the filter on its peer field; timestamp capture, the six
instants and the distance formulas; applying the radio configuration and
the antenna delays, reading transmit power back off the chip, and
logging the configuration at boot; temperature and voltage sampling; the
record, with its field set, units, provenance slots and line formatting,
`-` for a missing value included; the record buffer, bounded by the
exchange count, and the dump-after-run ordering; the settle algorithm's
windowed means, consecutive-window test and give-up; the set of roles
and the identifiers they print; the parameter struct's fields, units,
defaults and validation; and the driver version on the `STATS` line.

**`probe/port/<os>/` holds the host glue**, and knows nothing about any
bench: wait for an interrupt or a timeout, a monotonic clock, a sleep,
and the line sink that a record is written to.

**The shell holds the values**: own and peer address; channel, PRF, PAC,
preamble length, both codes, bitrate, the SFD bit, both antenna delays
and the requested power; the sampling interval and duration; the node
name and run id; the exchange count; the settle window length and count,
its threshold and its give-up time; the command surface and the mapping
of verbs to roles; how the parameter struct gets filled; and the pin and
build wiring.

Two rows are worth their own sentence, because the obvious placement is
the wrong one:

- **Formatting is core, not shell.** The line must be byte-identical
  everywhere, and the record is proved under emulation, where no shell
  exists. A formatter in the shells would be written twice and would
  drift.
- **Buffering is core, not transport.** Buffer-then-dump is a
  correctness rule of the instrument, not a convenience of the console.
  In the shells it would be duplicated, and nothing would stop a third
  consumer writing an inline-emitting shell that manufactures exactly
  the artifact the rule exists to prevent.

This split line is the one judgment call here that no test will catch.
If it is wrong, the symptom is slow: either bench assumptions accumulate
in a general library, or the two shells start duplicating what `probe/`
should have held.

### Devicetree: the probe writes its own overlays

An instrument whose devicetree moves when the thing under test moves is
a bad instrument, so the probe does not share the production
application's board overlays. It writes near-empty ones for a board
whose DW1000 comes from upstream Zephyr's own board definition, and for
a board that needs it, the SPI pinctrl and the `dw1000@0` node, and
nothing of the accelerometer, the LEDs or the button aliases that the
production overlays carry.

**The console speed is the exception the rule has to name.** Where a
board's console runs at other than its Zephyr default, the overlay must
set it, because that is how the bench reaches the board at all: an image
without it boots correctly, runs correctly, and answers in mojibake. The
distinction to carry: board wiring and application plumbing are the
application's own, but anything the bench's side of the wire has to
agree with is shared infrastructure.

## What it does

Each item is something the bench needed and could not get from a board.

1. **One record per exchange attempt, machine-readable.** One line,
   `key=value` pairs, mirroring the Ruby roles' `TWR seq=…` format, so
   that one parser serves both. An attempt that does not resolve still
   emits a line, with a status field and whatever instants did arrive.

2. **Transmit power settable at run time, in dB, with readback of what
   was applied.** The readback is not optional: the chip's "automatic"
   setting resolves to a particular value, so a measurement that
   records the request rather than the result is recording its own
   intention.

3. **Both exchanges**, single- and double-sided, initiator and
   responder, at a fixed exchange count, so that every pair yields the
   same fields. The double-sided one reports all three estimates; the
   single-sided one reports the estimate it can.

4. **One-way roles: transmit-only and receive-only.** These are what
   separated transmit heating from receive heating, and the answer
   changed how settling has to work. They are not optional extras.

5. **Idle temperature sampling**, radio stopped, for baselines and for
   the settle-until-stable phase.

6. **The same core on Zephyr and on Linux.** Not a reimplementation:
   the same `probe/` sources, over `OSAL=zephyr` and `OSAL=unix`. This
   is what makes a board-to-host pair measurable at all. It holds for
   every role and every read-back: the one-way and idle roles were
   first written as Zephyr shell commands, which made items 4 and 5
   board-only, and they moved into `probe/src/solo.c` for that reason.

## The exchange

**Two exchanges, not one.** Single-sided two-way ranging (two frames)
and double-sided (four frames), selected by a parameter, with the record
naming which produced it in `exchange=ss` or `exchange=ds`.

The four-frame exchange is the Ruby roles' `twr_init` / `twr_resp`,
specified by reference to a working implementation, which is the point
of choosing it. The two-frame one has no counterpart there and is this
instrument's own.

```text
   initiator                                   responder
      │ ── POLL ────────────────────────────────▸ │   t_sp / t_rp
      │ ◂──────────────────────────── RESPONSE ── │   t_sr / t_rr
      │ ── FINAL ───────────────────────────────▸ │   t_sf / t_rf
      │      carries t_sp, t_rr, echo of t_sr     │
      │ ── REPORT ──────────────────────────────▸ │
      │      carries t_sf + the initiator's sensors
```

Three consequences that are built in, not discovered:

- **Six instants, three of them carried in frames.** `t_rp` and `t_rf`
  are the responder's own receive timestamps and `t_sr` its own send
  timestamp; `t_sp` and `t_rr` travel in the FINAL payload (offsets 0
  and 1, with an echo of `t_sr` at offset 2), and `t_sf` travels in the
  REPORT payload at offset 0.
- **The REPORT frame is a sensor carrier.** The initiator's
  temperature, voltage, receive power and first-path power ride in the
  REPORT payload at offsets 1, 2, 3 and 4. Reporting *both* dies'
  readings is therefore a wire-format requirement, not two local reads.
- **Only the responder emits the record.** The initiator emits none.
  The responder holds both ends' numbers because the REPORT brought
  them.

### The single-sided exchange, and why three estimates

```text
   initiator                                   responder
      │ ── POLL ────────────────────────────────▸ │   t_sp / t_rp
      │ ◂──────────────────────────── RESPONSE ── │   t_sr / t_rr
```

Four instants, one estimate. It is worth having for two reasons: it is
half the airtime and half the chances to lose a frame, and it is the
estimator whose error is *most* informative, because what wrecks it is
exactly the thing double-sided ranging exists to cancel.

The four intervals of an exchange are

```text
   round1 = t_rr - t_sp      reply1 = t_sr - t_rp
   round2 = t_rf - t_sr      reply2 = t_sf - t_rr
```

and `round1` and `reply2` are measured on the **initiator's** clock
while `reply1` and `round2` are measured on the **responder's**. A
crystal offset between the two nodes therefore biases them differently,
and the three estimators absorb that differently:

```text
   single-sided          (round1 - reply1) / 2

   symmetric, "classic"  ((round1 - reply1) + (round2 - reply2)) / 4

                          round1·round2 - reply1·reply2
   asymmetric, Neirynck  -------------------------------
                         round1 + reply1 + round2 + reply2
```

On a true 1032.0 mm link with the responder's crystal 20 ppm fast (an
ordinary part-to-part offset, not a fault):

| Estimator    | Reports       | Error       |
| ------------ | ------------- | ----------- |
| single-sided | **-140.8 mm** | -1172.8 mm  |
| symmetric    | 1172.9 mm     | +140.9 mm   |
| Neirynck     | 1032.2 mm     | **+0.2 mm** |

Single-sided does not merely degrade, it goes negative. That is why the
distance fields are signed, and it is why a **double-sided exchange
emits all three** rather than picking one: `round1` and `reply1` are
present in a four-frame exchange too, so the single-sided estimate comes
free, over *identical* timestamps. Comparing estimators on one exchange
is a different and much better experiment than comparing them across
separate runs, where the link has moved underneath.

So the spread between the three is not noise to be averaged away. It is
a measurement of the clock offset between the pair, which is a property
of that pair.

**Semantic parity, not just format parity.** Matching field names while
computing them differently would be worse than not matching at all: the
mismatch would stop being visible. The probe computes `sym_mm` and
`asym_mm` by the same two formulas over the same six instants as its
reference implementation.

**Addressing is a deliberate departure.** The Ruby exchange carries no
address: it filters on a mark, a type and a sequence byte alone. Taken
verbatim, any probe on the channel would answer any other probe's POLL,
and the requirement not to collide with the production nodes would be
unfulfillable. So the probe adds a peer field to the frame header and
filters on it. Like the other departure below, it is stated in the
record rather than discovered from a distance that makes no sense.

**Start ordering.** The responder prints `READY` before listening, and
the initiator is not started until it appears. The initiator then
discards a stated number of warm-up exchanges before the counted run
begins. A responder that is not yet listening loses the opening
exchanges, which is precisely the shape of result that made counts
untrustworthy in the first place.

**The responder's wait is bounded.** The first POLL gets sixty seconds
and each one after it two, both overridable at compile time
(`PROBE_EXCHANGE_FIRST_POLL_TIMEOUT_US`,
`PROBE_EXCHANGE_POLL_TIMEOUT_US`), which is how the gate proves a run
nobody answers ends rather than hangs. An unbounded first wait is the
one failure this instrument cannot report: every other produces a
record with a status, while that one produces absence, and a responder
that is never polled is then indistinguishable from one that was never
started.

## The record

The field list, in order, as the Ruby roles emit it:

```text
TWR seq= t_sp= t_rp= t_sr= t_rr= t_sf= t_rf= sym_mm= asym_mm=
    resp_temp= resp_vbat= init_temp= init_vbat=
    resp_rx= resp_fp= init_rx= init_fp=
```

Units: instants in device clock ticks; distances in millimetres;
temperature in hundredths of a degree Celsius; voltage in millivolts;
receive and first-path power in hundredths of a dBm, negated. A missing
value is rendered `-`.

Two deliberate departures from the reference:

- **A `status=` field, and a line for every attempt.** The Ruby roles
  emit nothing at all for an exchange that does not resolve. Mirroring
  that exactly would make the probe a *regression* on the one statistic
  boards already provide, the resolved count. The probe emits a line
  per attempt, carrying whatever instants arrived.
- **Provenance.** The record says where it came from: `node`, `role`
  and `run`. Transmit power is not on the TWR line; it rides the
  `STATS` summary, as in the reference, so that one parser still serves
  both.

### What a transmit power read-back is worth

Not what one might hope. The driver's cached transmit power is **not**
the request: the clamp and the encode happen first, and the automatic
setting is resolved out of the power table, so the cache already holds
the exact word that goes on the wire. Nor does reading back catch an
encoder defect: the decode is the exact inverse of the encode, so a
wrong encoder round-trips cleanly through its matching decoder.

What a read-back is genuinely worth is narrower, and it is enough:
**it asserts that the chip is in the state the driver believes.** A
write that did not land, a reset, a brownout, a radio that came up
wrong: none of those are visible in the cache, and all of them
invalidate every number in the run. For a production stack that is not
worth a bus transaction. For an instrument it is, because nothing else
in the record would reveal it, and the bench has a documented instance
of exactly that class: a board coming back from a flash with a DW1000
that reports no transmissions until it is power-cycled.

So: one SPI read per run, as a cheap assertion that the radio is
configured as assumed. `dw1000_tx_get_power()` and
`dw1000_tx_power_to_05db()` exist in the driver for it. The `STATS` line
reports the read-back value and says it was read, so a future reader
knows which it is.

### Radio configuration parity is not a formality

It has already failed twice, and the difference in how says what to
build.

**Antenna delay** was configured for the wrong distance, giving a delay
tens of ticks out, about a tenth of a metre of range. Caught in
*seconds*, because both firmwares log the delay at boot and the two
lines sit next to each other.

**The proprietary SFD bit** was left clear where the production stack
sets it. That is not cosmetic: it selects the `CHAN_CTRL` SFD mode and
with it the `RXPACC` adjustment, which is the `N` of the UM §4.7.1
power formula, so the probe's received power would have sat a few tenths
of a dB from the production stack's, the same magnitude as the
estimate error this instrument exists to detect. Caught only by reading
two source trees side by side and noticing a field the probe did not
set.

The second took far longer, and the reason is that the boot log carried
the antenna delay and nothing else. So the probe logs its **full** radio
configuration at boot (channel, PRF, bitrate, preamble length, PAC,
both codes, the SFD bit, and both antenna delays) in one line, in a
fixed order. A difference then costs a diff rather than an afternoon,
and the next such field cannot hide by being one nobody thought to
check.

Antenna delay alone is not enough for the same reason: channel, PRF,
preamble length, PAC size, data rate, SFD mode and preamble code all
move both range and received power. The receive-power estimate in
particular takes a PRF-dependent constant and the preamble accumulation
count, and choosing the wrong constant is an error of several dB,
larger than the effects being chased. **The probe does not re-implement
it**: `dw1000_rx_get_power_estimate()` already does, citing UM §4.7.1
and §4.7.2. That is the shared-driver boundary working as intended,
and it is also why a defect in that estimate is one the probe cannot
see.

**Temperature is absolute, by construction.** DW1000 die temperature is
absolute only after the OTP calibration reading is applied, and the
driver applies it, so reporting one node's reading alongside another's
is a claim the driver already supports. Note that
`dw1000_read_temp_vbat()` is `void` and has no failure path, so `-` in a
temperature field can only mean the value never travelled (a peer's
reading absent from a REPORT frame), never that a sensor read failed.

## Interface

The console shell on a board; argv and stdout on a Linux host. A
measurement is: attach with the command that selects the role and the
power, then capture the console for the duration. Set-then-measure is
the order the bench already uses. On Zephyr the commands register under
one top-level noun, `probe`, with subcommands named after the roles.

The two shells differ in one way worth stating, because it is a
difference of behaviour and not of spelling: **a board's settings
stick, a host's do not.** On a board, node name, addresses, transmit
power and double buffering hold until changed, whether by their own
command or by a role's option, because a board is a long-lived shell
and every console attachment is expensive. On a host every invocation
starts from the defaults. A host's line buffer grows; a board's holds
64 lines, and a board refuses a run that would not fit rather than lose
its end.

**Records are buffered and dumped after the run, never emitted inline.**
The reference does this because a write that blocked between the frames
of an exchange would land in the measurement. On a board the channel is
a UART, and a record of a few hundred bytes costs milliseconds, an
amount that differs between board types, because their console speeds
differ. Emitting inline would therefore backpressure the protocol
*differently on the two board types*, manufacturing a board-type
-dependent artifact in an instrument built to compare board types. The
fixed exchange count has a second job: bounding the buffer.

Do **not** reach for the alternatives to a run-time setting. A build
option plus a reflash costs a build and a flash per value, and
reflashing power-cycles the board, which resets the die and restarts
the thermal settling. Writing memory over SWD does not work at all:
transmit power reaches the chip from exactly one write site inside the
driver's radio tuning, reached only from `dw1000_configure()`, reading
the driver's own copy of the configuration, which it holds by value.
Poking the application's struct reaches neither the copy nor the chip.

## Facts worth not rediscovering

**Transmit power encoding.** `DW1000_TX_POWER_AUTO` is 0,
`DW1000_TX_POWER_05DB(v)` is `v | 0x80`, and `DW1000_TX_POWER(v)` is
that of `2*v`. The range is 61 half-dB steps, 30.5 dB, per UM
§7.2.31.1: 7 coarse steps of 2.5 dB and 32 fine of 0.5 dB. An earlier
driver used 3 dB coarse steps and 33.5 dB. This is a driver-level claim
the probe cannot independently check.

**Antenna delay: read it, do not transcribe it.** The delay is
calibrated per platform, differs between a board and a host adapter,
and moves with temperature by a per-module amount. A figure copied into
a document goes stale silently, and a tenth of a metre of calibration
distance is centimetres of range, the size of the effects being
chased. So the probe logs its own at boot and the two ends are compared
by inspection, never by a constant written down here.

**Settling needs the receiver enabled, not traffic, and not a timer.**
A node with its receiver on and nothing arriving heats at several
degrees a minute; with the radio off it cools. So a node settles itself,
alone, and the settle phase of a pair is two independent settle phases
that can run before the peer is even flashed. A soak with the radio off
settles nothing, and a timer alone is not a settle: the criterion has to
watch the temperature stop moving *between* windows, not within one,
because a chip climbing steadily has a small spread inside any short
window. Concretely, and adjustable once measured: **four consecutive
30 s windows whose means differ by less than 0.2 °C, giving up at
15 minutes.**

"Differ by less than" is read as the **spread** of the four means, the
largest minus the smallest, not as each step between neighbours. A die
climbing 0.15 °C a window passes a step test indefinitely while moving
0.45 °C across the four, and a steady climb is exactly what the rule is
there to refuse. The rule is `probe/src/settle.c`, free of the driver
and tested on readings alone; it runs inside the rx role
(`rx --settle`), because the receiver being on is what settles a die,
and its windows are counted in readings, thirty at 1 Hz, so that it is
a pure function of what it was fed. Its output is a `SETTLE` line per
window rather than a `TEMP` line per reading, which bounds a settle
run's lines by its give-up time: thirty-two at the defaults, which a
board's buffer holds.

**The threshold is below what one reading can resolve, and that is not
a mistake.** Consecutive samples of a chip sitting still read one of
two adjacent codes and nothing between; that step is about 1.1 °C, one
LSB of the DW1000's 8-bit SAR. Two consequences:

- **Compare window means, never raw readings.** A raw-reading
  comparison against 0.2 °C can only pass by coincidence, and an
  implementer who writes it that way will see a settle phase that never
  converges and conclude the chip never settles.
- The mean works because the readings **dither** between adjacent
  codes. Thirty samples at 1 Hz put the standard error of the mean well
  under the threshold. Sample fewer, or at a rate where the readings
  stop dithering, and the resolution goes away with it.

It also recalibrates the temperature figures above: a change of about
1 °C is one LSB, at the edge of what a single reading can see. Read
small deltas accordingly.

### What emulation gives you, and what it does not

Running the probe against `OSAL=emulation`, which is how the chip-free
tests work, comes with standing facts, all deliberate. The port's own
contract is in
[`../port/emulation/README.md`](../port/emulation/README.md).

**Received power reads as absent.** `RX_FINFO`'s accumulation and rate
fields are unwritten and `RX_FQUAL` is all zero, because there is no
signal model behind them. So `dw1000_rx_get_power_estimate()` has
nothing to work from and the record prints `-` for the power fields.
That is the model being honest: a schema-valid fabricated power estimate
would be far worse than an obviously absent one.

**The line callback must never call the driver.** The callback runs on
the port's reader thread, and a driver call from it that enables the
receiver or starts a transmit issues a request and waits for a reply,
and the only thread that reads replies is the one now sitting inside the
callback. The wait is bounded, so this is a five-second stall and an
error rather than a permanent hang, but a node that spends those seconds
has already lost whatever timing it had.
`dw1000_process_events()` takes that path in double-buffered mode, so
this is the ordinary case and not a corner of it. The probe is immune by
construction (`<dw1000/probe/port.h>` forbids touching the chip from
the wake context), and the rule must survive anyone who thinks it is
merely tidy. The port has had two independent reasons for the
prohibition; one was fixed and the other was not, and for a while both
were believed gone. That is why the contract states the rule without
resting it on any port's internals.

**The same hazard has an on-board form.** Anything that
read-modify-writes the status register from both an interrupt handler
and the main loop can lose an edge: the derived interrupt state ends up
describing neither read, and a rising edge disappears. The symptom is a
transmission whose completion the host is simply never told about,
although the register says it happened. The probe's protection is the
same port rule, the status register having one reader, but it is
protection by discipline, and on hardware a lost edge costs a
measurement.

**Emulation can tell you the exchange is logically correct; it cannot
tell you it is fast enough.** A sub-millisecond responder against a
software re-arm that cannot beat it is a race between real latencies,
and the model has neither: it has no notion of how long a host takes to
get from `tx_done` to re-arming the receiver. Correctness under
emulation is worth having; "fast enough" remains a hardware
measurement, per platform, and a cross-platform pair is what finds it.

**A model owns a thread and must be stopped, not dropped.** The shutdown
order does not commute: stop the model, close the connection, then
destroy the model, and none of the three from the line callback. A test
that creates one model and exits is fine (threads die with the
process), but anything that creates and discards models in a loop,
sweeping a
parameter say, leaks a thread apiece. **This is the one to watch, and it
will not look like itself**: it surfaces as a failure to create the nth
model, far from the call that leaked the first.

## Out of scope

Anything that makes the probe resemble the production firmware: the
ranging protocol, multi-node rounds, initiator election, the UAV layer,
the host-link library, the accelerometer, the GUI, sniffing.

Anything bench-specific inside `probe/`. If a change there would need to
know a node's address, a hub port, or the shape of a bench configuration
file, it belongs in a shell. See the split line above.
