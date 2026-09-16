# The probe: a measurement-only instrument

What the probe is, why it exists apart from the production stack, and
the contract its record and its exchange keep. The code it describes
lives beside this file: `probe/src/`, `probe/include/dw1000/probe/`, and
the Unix application in `probe/app/unix/`.

**This is one of two documents.** The campaign that used the instrument
-- the stages, every measured result, and the bench-side findings --
is `PROBE-CAMPAIGN.md` in the bench repository (`~/Thesis/dev/hardware`).
Design and contract here; what was measured and how it went wrong there.
They were one file until 2026-09-16 and split because half of it
described code in this repository and half described a testbed in
another, so neither half lived where its reader would look.

The Zephyr application's own hard-won facts -- the CDC-ACM console wedge,
one console attachment per role, the console baud in the board overlay,
4096 stacks, immediate logging -- are not here either. They are
properties of that application and live with it, in
`zephyr-redskin/probe/` and in its commit history.

## 1. Why

`bench pair-baseline` measures every link on the bench one pair at a
time. For a pair of Pi nodes it drives `ruby-dw1000`'s roles and reports
received signal strength, first-path strength, both dies' temperature,
the distance as measured from each end separately, and the link's
asymmetry. For any pair involving a board it can only run the production
stacks and count how many exchanges resolved.

The reason is in the bench's own code (`lib/bench/cli.rb:101-106`): those
roles are Ruby against a DW1000 on a Linux host, and *"a board runs
redskin and speaks FTM-BC, **not the gem's TWR exchange**"*. The obstacle
was never that a board cannot run Ruby. It is that no board speaks the
instrument's exchange. That is a thing an instrument gets to choose.

The gap matters. Measured on this bench:

- Two nodes 36 cm apart resolved 56 and 57 exchanges where another pair
  at 37 cm resolved 413 and 323, with every other radio powered off.
  Counts alone could not say why, and the pair in question had a board
  at one end — so it fell on the count-only side of exactly this split.
- Transmit power turns out to matter enormously at these ranges. On a Pi
  pair, 40 exchanges offered: 23 and 39 completed at 5 dB, 17 and 31 at
  the chip's automatic setting, and **zero in both directions at 20 dB**.
  Received power tracked the setting, so the effect is real. No board can
  be swept this way today, because its transmit power is fixed at build
  time.
- Die temperature is not stationary during a measurement, and the
  asymmetry is large: a node flooded with 2000 frames rose 1.1 °C at the
  transmitter and 8.0 °C at the listener. Antenna delay drifts with die
  temperature, so a measurement taken on a chip that is still warming is
  not the measurement you wanted. Boards report no temperature at all.

  **Stage 2 measured this on boards and found the mechanism is not what
  was written here.** It is not *reception* that heats the chip. It is an
  **enabled receiver**. Measured on D4, three conditions, same chip, same
  interval:

  | condition | change over ~40 s | rate |
  |---|---|---|
  | radio off, idle (D4) | -1.1 °C | about -1.6 °C/min, i.e. cooling |
  | receiver enabled, **zero frames** (D4, DWM1001) | **+6.5 °C** | **+9.8 °C/min** |
  | receiver enabled, **zero frames** (A2, nRF52840-MDK) | **+14.8 °C** | **+22.2 °C/min** |
  | transmitting 2000 frames (A2) | none visible (inside one LSB) | about 0 |

  Both board types, both with the channel verified silent and zero frames
  received. A2 reached **51.5 °C** from 36.7 in forty seconds.

  The receiver-enabled rows are the ones that matter: nothing arrived,
  and the chips still heated — one of them at over 20 °C a minute. Frames
  are incidental; what costs power is the receiver being on. The original
  figures are not wrong — a listener does end up much hotter — but the
  cause was misattributed, and the difference is practical rather than
  academic (see §9's settle rule).

  **The rate is board-dependent and that is itself a result**: the MDK
  climbs more than twice as fast as the DWM1001, so the two ends of a
  cross-type pair do not merely settle at different temperatures, they
  approach them at different speeds. A fixed settle *timer* would
  therefore be wrong for one of the two even if it were right for the
  other, which is the second reason §9 asks for a measured criterion
  rather than a duration.

  **And it makes receiver-on time a systematic error source for every
  ranging measurement on this bench**, not just for the probe's. Antenna
  delay drifts with die temperature; a node whose receiver has been
  enabled for a minute is at a materially different operating point from
  one just switched on. Any pair measured shortly after being brought up
  is being measured while both ends move — and they move at different
  rates.

So the probe exists to make every node on the bench answer the same
questions, by running the same instrument everywhere.


## 2. What it is, and what it is not

**It talks only to other probes.** It does not implement FTM-BC and does
not interoperate with `redskin` on air. This is settled, not open — and
because the probe core runs on the Pi as well as on a board (§3), it
costs nothing: every node can be a probe, so every pair is a probe pair.
`ruby-dw1000`'s TWR roles stop being the thing to imitate and become the
independent cross-check, which is a better job for them.

The consequence is deliberate and must be understood by whoever builds
it: the probe measures the *radio path* — antennas, placement, power,
temperature — and not the production firmware's behaviour on that path.
For "is this link any good", which is what `pair-baseline` asks, that is
the right instrument. For "why does redskin do badly on this link", it is
the wrong one, and such questions stay with the stacks.

That constraint is what keeps the probe small, and small is the point.
`redskin` is the thing under test. In one session this bench found a
protocol defect costing a third of all exchanges between two nodes, a
received-power estimate 0.75 dB low, and a stale transmit completion
handing back the previous frame's timestamp. An instrument sharing that
code cannot tell you whether a bad number is the link or the stack. A
probe implementing one fixed exchange has very little to be wrong about.

So it carries no FTM-BC, no UAV, no ExtIO, no accelerometer, no GUI and
no sniffer.

### The independence boundary, stated honestly

The probe shares **the DW1000 driver, and nothing above it.**

- **Shared, and unavoidably so:** `decawave-drivers` — the register
  layout, the timestamp path, the receive-power estimate, the transmit
  power encoding. Any instrument for this chip goes through a driver, and
  writing a second one would be a larger thing to be wrong about than the
  one being avoided.
- **Not shared:** `spank` in its entirety. No protocol core, no FTM-BC
  state machine, no ranging history, no spank OSAL, no spank syslog.

The honest limitation that follows: **a driver-level defect is invisible
to this instrument.** Two of the three defects listed above are driver
matters, not `redskin` matters. When the probe and `redskin` disagree,
the probe is evidence; when they agree, they may be agreeing on a driver
bug. `AUDIT.md` in `decawave-drivers` is where that risk is managed, not
here.


## 3. Where the probe lives

The probe core lives **in `decawave-drivers`**, beside the driver it
characterises, with one thin shell per platform outside it.

```
  decawave-drivers/
      probe/                     ← the core. Chip-generic mechanism.
      probe/port/<os>/           ← host glue: clock, wait, line sink
      hw/drivers/dw1000/           the driver (shared, deliberately)
      port/<os>/dw/osal/           seven driver ports, already written
      docs/                        aps011, aps013, aps014, aps006 …
              ▲                          ▲
              │                          │
  zephyr-redskin/probe/          rpi-probe/
      Zephyr shell:                  Linux shell:
      devicetree, shell commands     bitters pin wiring, argv
```

Three reasons this is the right home, all of them already true of the
repo:

1. **The subject matter is already there.** `decawave-drivers/docs/`
   holds `aps011_sources_of_error_in_twr.pdf`,
   `aps013_two_way_ranging_v2.0.pdf`,
   `aps014_antenna_delay_calibration_v1.01.pdf` and
   `aps006_part1_channel_effects.pdf`. That is the probe's entire
   bibliography, sitting next to an `AUDIT.md` that audits the driver
   against the user manual. Everything the probe measures is chip
   characterisation, not swarm protocol.

2. **The portability is already there.** `dw1000.cmake` declares
   `DW1000_OSAL_PORTS cf2 chibios emulation mynewt null unix zephyr` —
   seven ports, written and in use. `rpi-redskin` already builds the
   driver standalone as `${DW1000_SOURCES} ${DW1000_OSAL_SOURCES}` with
   `OSAL=unix`. The repo includes nothing from `spank`; the dependency is
   zero. So the Pi port of the probe costs a `main.c`, not a port.

3. **One commit is one instrument.** A measurement is interpretable only
   if you know which driver produced it, and the driver is partly what is
   under suspicion. Co-located, the pairing is unforgeable. It also makes
   the drift recorded in `nodes.conf:36-38` structurally impossible for
   the probe — see §9.

### Why not in `spank`

Investigated and rejected on evidence. `spank`'s lower layers cannot be
taken without its core:

- `port/unix/osal/src/osal.c:366,378` and `port/zephyr/osal/src/osal.c:45,49`
  **call** `spank_syslog_string()` and `spank_syslog_level_lookup()`,
  defined only in `spank/src/syslog.c`. That is a link-time dependency of
  the OSAL on core object code.
- `port/hal/io/dw1000/include/spank/driver.h:16` includes
  `<spank/types.h>` — a core header, inside the HAL's own contract.
- `spank/zephyr/CMakeLists.txt:9` wraps everything in one
  `if (CONFIG_SPANK)`, with `zephyr_library_sources(${SPANK_SOURCES})`
  unconditional at line 83. No Kconfig path takes the OSAL without the
  core.

And this is policy, not accident: `spank.cmake:73-88` calls the core
*"All of it: nothing here is optional, and the pieces reach into one
another"*, and `DESIGN.md` says *"The core is not divisible: everything
under spank/src reaches into the rest of it."* Detangling it to host an
instrument would mean overturning that, in the tree where the protocol
under test is actively being changed. `decawave-drivers/zephyr/CMakeLists.txt`
is all-or-nothing in the same way — but there all-or-nothing yields
exactly the driver the probe wants, so the same mechanism has the
opposite consequence.

### Naming: `dw1000_probe`, not `probe`

The component is `probe/` as a directory, but everything it exports
carries the driver's prefix: `<dw1000/probe/record.h>` on the include
path, `dw1000_probe_*` for C symbols and types, `DW1000_PROBE_*` for
CMake variables, `CONFIG_DW1000_PROBE` for Kconfig. That mirrors how the
ports already do it -- `port/emulation/dw/osal/include/dw1000/osal.h` --
and it is not only a convention point.

A bare `probe` collides with the bench's own vocabulary. `devlist.conf`
warns that a mistake means *"a flash goes to the wrong probe"* and
`nodes.conf:30` notes a board *"where the probe enumerates"* -- in both,
`probe` is the **debug adapter**, the CMSIS-DAP or J-Link. Naming the
instrument `probe` too would produce sentences like "flash the probe onto
the board through the probe". There is no compile-level clash in Zephyr
(`CONFIG_PROBE`, `include/probe/`, a `probe` shell command and `probe_*()`
are all free), which is exactly what makes this worth writing down: the
collision is in comprehension, so nothing would ever have errored.

`.probe` being the universal driver-init verb in Zephyr and Linux idiom
is the second reason a `probe_*` prefix inside a *driver* library reads
wrong.

The board's shell noun stays `probe`, deliberately: at a board's console
there is no debug adapter to confuse it with, and the ambiguity above
lives in the flashing vocabulary, not the console one. Revisit if that
stops being true.

### The split line — mechanism, host, values

`decawave-drivers` is a general library with a licence, version tags and
seven ports. Bench specifics must not leak into it. But a two-way split
of "chip-generic versus bench-specific" is the wrong shape, for two
reasons found by review before any of this was built:

- Most of what a second bench would change is neither code nor format but
  **values** — addresses, radio configuration, antenna delays, counts,
  gaps, timeouts, settle thresholds. Their type, unit, default and
  validation are chip facts; only the value is the bench's.
- The driver's port contract offers `_dw1000_delay_usec/msec`, the io
  lines, SPI and an assert, and **nothing else**
  (`port/unix/dw/osal/include/dw1000/osal.h`). No monotonic clock, no
  wait-for-interrupt-or-timeout. The exchange state machine needs both,
  and every existing consumer has hand-rolled them: `tests/emulation/smoke.c`
  and `rpi-redskin/main.c:219-332` each invented their own condition
  variable and `clock_gettime` loop. That is **host glue**, not bench
  knowledge, and putting it in the shells would mean a second lab on
  Zephyr rewriting plumbing that has nothing to do with its bench.

So the line is three-way:

> **Mechanism in `probe/`; host glue in `probe/port/<os>/` beside the
> driver's ports; values in the shell.** Every state machine, formula,
> format, buffer and rule is chip-generic. Every value it needs is a
> parameter whose type, unit, default and validation live in `probe/` and
> whose value comes from the shell. What the host must supply to run it —
> a clock, a wait, a line sink — is a port and knows nothing about the
> bench. The shell is what is left: choosing values, wiring pins, and
> naming things for the people who run this bench.

| in `probe/` (mechanism) | in `probe/port/<os>/` (host, bench-free) | in the shells (this bench) |
|---|---|---|
| the exchange state machine; the frame layout, including the peer field and the filter on it | wait for IRQ or timeout; monotonic clock; sleep | own and peer address values |
| timestamp capture, the six instants, the `sym_mm`/`asym_mm` formulas | — | — |
| apply the radio configuration and antenna delays; set transmit power and read it back from the chip; log the configuration at boot | — | channel, PRF, PAC, preamble length, codes, bitrate, SFD, antenna delays, requested power |
| temperature and voltage sampling | — | sampling interval and duration |
| the record: field set, units, provenance slots (`node`, `role`, `run`), and the `TWR` / `STATS` / `READY` line formatting, `-` for a missing value included | the line sink (UART write, stdout write) | node name and run id values |
| the record buffer, bounded by the exchange count, and the dump-after-run ordering | — | the exchange count |
| the settle algorithm: windowed means, consecutive-window test, give-up | clock | window length, window count, threshold, give-up time |
| the set of roles and their canonical identifiers as printed in `role=` | — | the command surface / argv, and the mapping of verbs to roles |
| `struct probe_params`: fields, units, defaults, validation | — | filling it from argv, the shell command, `nodes.conf` / `devlist.conf` |
| the driver version on the STATS line | — | hub and port wiring, `build-zephyr`, pin wiring (`config.h`, devicetree overlay) |

Two rows are worth their own sentence, because the obvious placement is
the wrong one:

- **Formatting is core, not shell.** §5 item 1 requires the line to be
  byte-identical everywhere, and stage 0 proves the record under
  emulation, where no shell exists. A formatter in the shells would be
  written twice and would drift — the format-level version of the
  semantic-parity failure §6 warns against.
- **Buffering is core, not transport.** §8 makes buffer-then-dump a
  correctness rule of the instrument, not a convenience of the console.
  In the shells it would be duplicated, and nothing would stop a third
  consumer writing an inline-emitting shell that manufactures exactly the
  artifact §8 exists to prevent.

> **This split line was the one judgment call in this plan that no stage
> proof will catch**, and it has had one adversarial review, which moved
> three rows and added a column. If it is wrong again, the symptom is
> slow: either bench assumptions accumulate in a general library, or the
> two shells start duplicating what `probe/` should have held. Revisit it
> at stage 5, which is the first time two shells exist to disagree.


## 4. Where everything is

| what | where |
|---|---|
| the driver, and the probe core | `~/Repos/decawave-drivers` (symlinked as `~/ZephyrProjects/modules/decawave-drivers`) |
| the workspace | `~/ZephyrProjects` (holds `zephyr/`, `spank/`, `spank-extio/`, `modules/`) |
| the existing Zephyr app | `~/ZephyrProjects/zephyr-redskin/redskin/` |
| the new Zephyr shell | `~/ZephyrProjects/zephyr-redskin/probe/` |
| the existing Linux app | `~/Repos/rpi-redskin` (its `main.c`, `config.h` and bitters wiring are the template) |
| the instrument being replaced | `~/Repos/ruby-dw1000/test/roles/node.rb` |
| the bench | `~/Thesis/dev/hardware` |
| the build script | `~/Thesis/dev/hardware/scripts/build-zephyr` |
| node and board config | `~/Thesis/dev/hardware/nodes.conf`, `devlist.conf` |

`redskin/` shows the shape a Zephyr app takes here: `CMakeLists.txt`,
`Kconfig`, `prj.conf`, `boards/<board>.conf`, `boards/<board>.overlay`,
`src/`, and shell commands under `src/shell/` registered as one
`SHELL_STATIC_SUBCMD_SET_CREATE` table per top-level noun, each closed by
`SHELL_CMD_REGISTER` (`redskin/src/shell/spank.c:532-550`).

`tests/emulation/smoke.c` in `decawave-drivers`, built by
`tests/check-emulation.sh` with a plain `cc` line against the core and
the emulation port, is the template for a standalone probe binary that
needs no chip. It is what makes stage 0 possible.

### Devicetree: do not reuse `redskin`'s overlays

The two board types are an nRF52840-MDK (`nrf52840_mdk`) and a
DWM1001-DEV (`decawave_dwm1001_dev`), and the previous version of this
plan said to reuse `redskin/boards/`. That is wrong in both directions:

- `redskin/boards/decawave_dwm1001_dev.overlay` is 32 lines and contains
  **no DW1000 at all** — it is an accelerometer `chosen` node, UI LED and
  button aliases, an i2c pinctrl, and `&spi1 { status = "disabled" }`.
  The DW1000 comes from upstream Zephyr's board definition
  (`zephyr/boards/qorvo/decawave_dwm1001_dev/decawave_dwm1001_dev.dts:142`).
  Reusing it imports exactly the three things §11 puts out of scope.
- `redskin/boards/nrf52840_mdk.overlay` is 106 lines and *does* carry the
  DW1000 (`&spi2`, `dw1000@0`, plus `spi2_default`/`spi2_sleep` pinctrl),
  mixed in with i2c0, the accelerometer and the same UI aliases.

So the probe writes its own: near-empty for the DWM1001, and for the MDK
the spi2 pinctrl, the `&spi2`/`dw1000@0` block — **and the console
speed**. Beyond tidiness, an instrument whose devicetree moves when the
thing under test moves is a bad instrument.

**The console speed is the exception that the rule has to name**, because
leaving it out cost a flash and a capture to discover. `redskin`'s MDK
overlay carries `&uart0 { current-speed = <230400>; }`, which is not the
board's default. It looks like more of the application plumbing being
deliberately left behind — it is not. `devlist.conf` records a speed per
board and `tribe-control` connects at it, so that line is how the bench
reaches the board at all; an image without it boots correctly, runs
correctly, and answers in mojibake. The DWM1001 is the opposite case and
its overlay is right to set nothing: `devlist.conf` has it at the stock
115200.

The distinction to carry: **board wiring and application plumbing are the
app's own, but anything the bench's side of the wire has to agree with is
shared infrastructure.** Today that is exactly one line.


## 5. What it must do

Each item is something the bench needed and could not get from a board.

1. **One record per exchange attempt, machine-readable.** One line,
   `key=value` pairs, mirroring `ruby-dw1000`'s `TWR seq=…` format
   exactly, so that one parser serves both. (That parser is not in
   `~/Thesis/dev/hardware`; it lives with the gem's pair harness. Name it
   here once found — a format contract whose consumer is unnamed is a
   contract with nobody.) Fields in §7. An attempt that
   does not resolve **still emits a line**, with a status field and
   whatever instants did arrive — see §7.

2. **Transmit power settable at run time, in dB, with readback of what
   was applied.** The readback is not optional. On these chips the
   "automatic" setting turns out to be 7.5 dB, so a measurement that
   records the request rather than the result is recording its own
   intention.

3. **Both exchanges of §6**, single- and double-sided, initiator and
   responder, fixed exchange count, so that every pair on the bench
   yields the same fields. The double-sided one reports all three
   estimates; the single-sided one reports the estimate it can.

4. **One-way roles: transmit-only and receive-only.** These are what
   separated transmit heating from receive heating, and the answer
   changed how settling has to work. They are not optional extras.

5. **Idle temperature sampling**, radio stopped, for baselines and for
   the settle-until-stable phase of §9.

6. **The same core on Zephyr and on Linux.** Not a reimplementation — the
   same `probe/` sources, over `OSAL=zephyr` and `OSAL=unix`. This is
   what makes a board-to-Pi pair measurable at all.


## 6. The exchange

**Two exchanges, not one.** Single-sided two-way ranging (two frames) and
double-sided (four frames), selected by a parameter, with the record
naming which produced it in `exchange=ss` or `exchange=ds`.

The four-frame exchange is exactly `ruby-dw1000`'s `twr_init` /
`twr_resp` (`test/roles/node.rb:280,298`). The two-frame one has no
counterpart there and is this instrument's own.

The previous version of this plan rested on *"one fixed exchange"*
without ever naming it; these are the exchanges, and specifying the
four-frame one by reference to a working implementation is the point of
choosing it.

```
   initiator                                   responder
      │ ── POLL ────────────────────────────────▶ │   t_sp / t_rp
      │ ◀──────────────────────────── RESPONSE ── │   t_sr / t_rr
      │ ── FINAL ───────────────────────────────▶ │   t_sf / t_rf
      │      carries t_sp, t_rr, echo of t_sr        │
      │ ── REPORT ──────────────────────────────▶ │
      │      carries t_sf + the initiator's sensors  │
```

Three consequences that must be built in, not discovered:

- **Six instants, three of them carried in frames.** `t_rp` and `t_rf`
  are the responder's own receive timestamps and `t_sr` its own send
  timestamp; `t_sp` and `t_rr` travel in the FINAL payload (offsets 0 and
  1, with an echo of `t_sr` at offset 2), and `t_sf` travels in the
  REPORT payload at offset 0.
- **The REPORT frame is a sensor carrier.** The initiator's temperature,
  voltage, receive power and first-path power ride in the REPORT payload
  at offsets 1, 2, 3 and 4. §5 item 1 asking for *both* dies' readings is
  therefore a wire-format requirement, not two local reads.
- **Only the responder emits the record.** The initiator emits none. The
  responder holds both ends' numbers because the REPORT brought them.

### The single-sided exchange, and why three estimates

```
   initiator                                   responder
      │ ── POLL ────────────────────────────────▶ │   t_sp / t_rp
      │ ◀──────────────────────────── RESPONSE ── │   t_sr / t_rr
```

Four instants, one estimate. It is worth having for two reasons: it is
half the airtime and half the chances to lose a frame, and it is the
estimator whose error is *most* informative, because what wrecks it is
exactly the thing double-sided ranging exists to cancel.

The four intervals of an exchange are

```
   round1 = t_rr - t_sp      reply1 = t_sr - t_rp
   round2 = t_rf - t_sr      reply2 = t_sf - t_rr
```

and `round1` and `reply2` are measured on the **initiator's** clock while
`reply1` and `round2` are measured on the **responder's**. A crystal
offset between the two nodes therefore biases them differently, and the
three estimators absorb that differently:

| estimator | formula |
|---|---|
| single-sided | `(round1 - reply1) / 2` |
| symmetric, "classic" | `((round1 - reply1) + (round2 - reply2)) / 4` |
| asymmetric, **Neirynck** | `(round1·round2 - reply1·reply2) / (round1+reply1+round2+reply2)` |

On a true **1032.0 mm** link with the responder's crystal 20 ppm fast --
an ordinary part-to-part offset, not a fault:

| estimator | reports | error |
|---|---|---|
| single-sided | **-140.8 mm** | -1172.8 mm |
| symmetric | 1172.9 mm | +140.9 mm |
| Neirynck | 1032.2 mm | **+0.2 mm** |

Single-sided does not merely degrade, it goes negative. That is why the
distance fields are signed, and it is why a **double-sided exchange emits
all three** rather than picking one: `round1` and `reply1` are present in
a four-frame exchange too, so the single-sided estimate comes free, over
*identical* timestamps. Comparing estimators on one exchange is a
different and much better experiment than comparing them across separate
runs, where the link has moved underneath.

So the spread between the three is not noise to be averaged away. It is a
measurement of the clock offset between the pair, which is a property of
that pair, and `ruby-dw1000` already carries the two double-sided
estimates for the same reason -- `measurements/temperature-effects/`
analyses the Neirynck figure and reports the symmetric one alongside.

**Semantic parity, not just format parity.** Matching field names while
computing them differently would be worse than not matching at all: the
mismatch would stop being visible. The probe computes `sym_mm` and
`asym_mm` by the same two formulas over the same six instants.

**Addressing is a deliberate departure.** The gem's exchange carries no
address: `TWR.frame` builds `"twr" + type + seq` and `TWR.match?` filters
on the mark, the type and the sequence byte alone
(`ruby-dw1000/test/support/twr.rb:86-99`). Taken verbatim, any probe on
the channel would answer any other probe's POLL, and §9's requirement not
to collide with the FTM-BC nodes would be unfulfillable. So the probe
adds a peer field to the frame header and filters on it. This is the
second of the two places the probe departs from its reference — see §7
for the first — and like that one it must be stated in the record, not
discovered from a distance that makes no sense.

**Start ordering.** The responder prints `READY` before listening —
`ruby-dw1000` does this at `node.rb:701` — and the bench does not attach
the initiator until it appears. The initiator then discards a stated
number of warm-up exchanges before the counted run begins. A responder
that is not yet listening loses the opening exchanges, which is precisely
the shape of result that produced §1's "56 and 57".


## 7. The record

The exact field list, in order, as `ruby-dw1000` emits it
(`test/roles/node.rb:712-714`, `twr_instants` at 663, `twr_sensors` at 679):

```
TWR seq= t_sp= t_rp= t_sr= t_rr= t_sf= t_rf= sym_mm= asym_mm=
    resp_temp= resp_vbat= init_temp= init_vbat=
    resp_rx= resp_fp= init_rx= init_fp=
```

Units, as the gem uses them: instants in device clock ticks; distances in
millimetres; temperature in hundredths of a degree Celsius; voltage in
millivolts; receive and first-path power in hundredths of a dBm,
negated. A sensor value that is missing is rendered `-`.

Two deliberate departures from the gem:

- **A `status=` field, and a line for every attempt.** `ruby-dw1000` does
  `TWR.distances(ts) or return nil` — an exchange that does not resolve
  emits nothing at all. Mirroring that exactly would make the probe a
  *regression* on the one statistic boards already provide today, the
  resolved count of §1. The probe emits a line per attempt, carrying
  whatever instants arrived.
- **Provenance.** The record must say where it came from: node, role, and
  run. In the gem the transmit power is not on the TWR line at all — it
  rides a separate `STATS role=… tx_power_db=…` summary
  (`node.rb:765`). The probe emits the same STATS line, so that one
  parser still serves both.

**Transmit power readback, and what it is actually worth.** An earlier
version of this section claimed the probe beats its reference here, on
the grounds that the gem records the request while the probe would record
the result. That was wrong, and worth recording as wrong because the
mistake is easy to repeat.

The driver's cached `dw->tx_power` is **not** the request. Look at
`dw1000.c:622-634`: the clamp to 61 half-dB steps happens first, the
encode second, and only then is `dw->tx_power` assigned; the automatic
setting is resolved out of the power table in the same way. So the cache
already holds the exact word that goes on the wire, clamp and
auto-resolution included. The gem decodes that same kind of word. On this
axis the two instruments were always equal.

Nor does reading back catch an encoder defect, which is the other thing
one might hope for: the decode is the exact inverse of the encode, so a
wrong encoder round-trips cleanly through its matching decoder. Verified
over all 62 steps. The 3 dB / 2.5 dB coarse-step error corrected in
`0544d9c` would have been invisible to a read-back.

What a read-back is genuinely worth is narrower: **it asserts that the
chip is in the state the driver believes.** A write that did not land, a
reset, a brownout, a radio that came up wrong — none of those are visible
in the cache, and all of them invalidate every number in the run. For a
production stack that is not worth a bus transaction. For an instrument
it is, because nothing else in the record would reveal it, and this bench
has a documented instance of exactly that class: D4 comes back from an
openocd flash with a DW1000 that reports no transmissions until it is
power-cycled (§9).

So: one SPI read per run, as a cheap assertion that the radio is
configured as assumed. `dw1000_tx_get_power()` and
`dw1000_tx_power_to_05db()` were added to the driver for it — the first
and so far only change this work has made to `hw/drivers/`. The STATS
line reports the read-back value and says it was read, so a future reader
knows which it is.

**Radio configuration parity is not a formality — it has already failed
twice.** Both were caught on the first hardware run, and the difference
in how says what to build:

- **Antenna delay** was configured at 154.0 m instead of 154.2, giving
  16411 against redskin's 16433: 22 ticks, about 103 mm. Caught in
  *seconds*, because both firmwares log the delay at boot and the two
  lines sit next to each other.
- **The proprietary SFD bit** was left at 0 where spank sets it to 1
  (`spank/port/hal/io/dw1000/src/driver.c`). That is not cosmetic: it
  sets the CHAN_CTRL SFD mode and selects `rxpacc_adj` (`dw1000.c:770-781`,
  -10 against -5), which is the `N` of the UM §4.7.1 power formula
  (`dw1000.c:926`). The probe's received power would have sat roughly
  0.3-0.4 dB from redskin's — the same magnitude as the 0.75 dB estimate
  error §2 cites as a reason this instrument exists. Caught only by
  reading two source trees side by side and noticing a field the probe
  did not set.

The second took far longer than the first, and the reason is that **the
boot log carries the antenna delay and nothing else**. So: the probe
logs its FULL radio configuration at boot — channel, PRF, bitrate,
preamble length, PAC, both codes, the SFD bit, and both antenna delays —
in one line, in a fixed order. A difference against redskin then costs a
diff rather than an afternoon, and the next such field cannot hide by
being one nobody thought to check.

**Antenna delay alone is not enough.** The
channel, PRF, preamble length, PAC size, data rate, SFD mode and preamble
code all move both range and received power. The probe logs its full
radio configuration at boot, and it must match `redskin`'s. The
receive-power estimate in particular takes a PRF-dependent constant
(≈113.77 dB at 16 MHz, ≈121.74 at 64 MHz) and the preamble accumulation
count; choosing the wrong constant is an ~8 dB error, which is the size
of the effects §1 is chasing. **The probe does not re-implement it**: the
driver already does, in `dw1000_rx_get_power_estimate()`
(`hw/drivers/dw1000/src/dw1000.c:2246`), citing UM §4.7.1/§4.7.2 for both
constants. That is the shared-driver boundary of §2 working as intended —
and it is also why a defect in that estimate is one the probe cannot
see.

**Temperature: absolute or relative?** §1's evidence is deltas (1.1 °C
versus 8.0 °C), which need no calibration. Reporting a die temperature
alongside another node's is an absolute claim, and DW1000 die temperature
is absolute only after the OTP calibration reading is applied. The driver
applies it (`dw1000.c:1159-1161`, `:1532`), so the probe reports absolute
temperature by construction. Note that `dw1000_read_temp_vbat()` is
`void` and has no failure path (`dw1000.h:912`), so `-` in a temperature
field can only mean the value never travelled — a peer's reading absent
from a REPORT frame — never that a sensor read failed.


## 8. Interface

The console shell on a board; argv and stdout on a Pi. The console
channel already exists and is exercised on every run: `tribe-control
connect --command=…` types a line into the board's shell
(`scripts/tribe-control:864`), and the bench sends `spank report all` to
every board at attach because the firmware logs at warning and would
otherwise stay silent. Each node in `nodes.conf` can override that line
with a `command` key (`lib/bench/backend/zephyr.rb:56`, default
`"spank report all"`) — a mechanism that exists but which no node
currently uses, so the probe will be its first consumer.

So a measurement is: attach with the command that selects the role and
the power, then capture the console for the duration. Set-then-measure is
already the order the bench uses.

Register the Zephyr shell's commands the way `redskin/src/shell/` does,
under one top-level noun (`probe`), with subcommands named after the
roles.

**Records are buffered and dumped after the run, never emitted inline.**
`ruby-dw1000` does this and says why (`node.rb:703-706`): *"Kept back
until the run is over, so that nothing writes to the pipe the harness
reads between the frames of an exchange — a write that blocked there
would land in the measurement."* That is a blocking pipe on a Linux host.
The probe's board-side channel is a UART, and a ~150-byte record is
~13 ms at 115200 and ~6.5 ms at 230400 — and per `devlist.conf` the
DWM1001-DEV runs at 115200 while every MDK runs at 230400. Emitting
inline would backpressure the protocol *differently on the two board
types*, manufacturing a board-type-dependent artifact in an instrument
built to compare board types. The fixed exchange count therefore has a
second job: bounding the buffer.

Do **not** reach for the alternatives to a run-time setting. Kconfig plus
a reflash costs a build and a flash per value, and reflashing
power-cycles the board, which resets the die and restarts the thermal
settling that §5 item 4 exists to respect. Writing memory over SWD does
not work at all: transmit power reaches the chip from exactly one write
site, `_dw1000_reg_write32(dw, DW1000_REG_TX_POWER, …)` inside
`_dw1000_radio_tuning()` (`hw/drivers/dw1000/src/dw1000.c:832`), reached
only from `dw1000_configure()`, reading the driver's own copy of the
configuration — which it has held by value since commit `2433928`
(`dw1000.h:438`, *"Copied: caller keeps nothing alive"*). Poking the
application's struct reaches neither the copy nor the chip.



## Facts worth not rediscovering

**Transmit power encoding.** `DW1000_TX_POWER_AUTO` is 0.
`DW1000_TX_POWER_05DB(v)` is `v | 0x80`, and `DW1000_TX_POWER(v)` is that
of `2*v` (`dw1000.h:181,190,197`). The range is 61 half-dB steps,
30.5 dB, per UM 2.18 §7.2.31.1: 7 coarse steps of 2.5 dB and 32 fine of
0.5 dB — stated at `dw1000.h:184-188` and enforced in the encoder at
`dw1000.c:615-633`. An earlier driver used 3 dB coarse steps and 33.5 dB,
corrected in `0544d9c`. Note this is a driver-level claim the probe
cannot independently check (§2).

**Antenna delay: read it, do not transcribe it.** The previous version of
this plan recorded `tx=16475 / rx=16475`. That is wrong for a board:
`redskin` configures `SPANK_ANTENNA_DELAY_METER_TO_CLOCK(154.2) / 2`
(`redskin/src/redskin.c:192-193`), which is **16433**. 16475 is the value
for 154.6 m — the figure the comment three lines above attributes to *the
Raspberry Pi hats*. The 0.4 m difference is 20 cm of range, an error the
size of the effect §1 is chasing. The delay is calibrated per platform
and *"moves with temperature by a per-module amount"*, so the probe logs
its own at boot as `redskin` does (`redskin.c:538`) and the two are
compared by inspection, not by a constant copied into a document.

**Settling needs the RECEIVER ENABLED, not traffic, and not a timer.**
An earlier version of this section said traffic, on the assumption that
heating came from receiving frames. Stage 2 measured otherwise (§1): a
node with its receiver on and *nothing at all arriving* heats at nearly
10 °C a minute, while the same node with the radio off cools.

That is a large practical simplification, and it is why the correction is
worth the space: **a node can settle itself, alone.** No peer, no
transmitter, no coordination between the two ends of a pair — each end
enables its receiver and waits for its own temperature to stop moving.
The settle phase of a pair is two independent settle phases, not a joint
one, and it can run before the peer is even flashed.

A soak with the radio **off** still settles nothing, so the original
warning stands in its practical form: a timer alone is not a settle. The settle phase
must run traffic and watch the temperature stop moving *between* windows,
not within one: a chip climbing steadily has a small spread inside any
short window. Concretely, and adjustable once measured: **four
consecutive 30 s windows whose means differ by less than 0.2 °C, giving
up at 15 minutes.** A rule without numbers is not a rule an implementer
can follow.

**The threshold is below what one reading can resolve, and that is not a
mistake — but it has to be understood or it will be implemented wrongly.**
Measured on D4: consecutive samples of a chip sitting still read either
3668 or 3782 and nothing between. That is 1.14 °C, one LSB of the
DW1000's 8-bit SAR. Two consequences:

- **Compare window MEANS, never raw readings.** A raw-reading comparison
  against 0.2 °C can only pass by coincidence, and an implementer who
  writes it that way will see a settle phase that never converges and
  conclude the chip never settles.
- The mean works because the readings **dither** between adjacent codes.
  Thirty samples at 1 Hz put the standard error of the mean near 0.06 °C,
  comfortably under the threshold. Sample fewer, or at a rate where the
  readings stop dithering, and the resolution goes away with it.

It also recalibrates §1's own figures: the 1.1 °C measured at the
transmitter is **one LSB**, i.e. at the edge of what a single reading can
see, while the 8.0 °C at the listener is about seven. The asymmetry is
real, but only one side of it is comfortably above the noise floor, and
that is worth knowing before anybody treats the transmitter figure as
precise.

**What the emulation model does and does not give you.** Running the
probe against `OSAL=emulation` -- which is how stage 0 works and how any
later chip-free test of the exchange would work -- comes with two
standing facts, both deliberate:

- **Received power reads as absent.** `RX_FINFO`'s `RXPACC`, `RXPSR`,
  `RXBR` and `RXNSPL` are unwritten and `RX_FQUAL` is all zero, because
  there is no signal model behind them. So `dw1000_rx_get_power_estimate()`
  has nothing to work from and the record prints `-` for the power
  fields. That is the model being honest rather than a fault to chase: a
  schema-valid fabricated power estimate would be far worse than an
  obviously absent one, and it is exactly the class of error §2 says an
  instrument cannot afford.
- **The line callback must never call the driver, and this is a hang
  rather than an error.** The callback runs on the rsvc reader thread. A
  driver call from it that enables the receiver or starts a transmit
  issues a request and blocks in `sem_wait()` for a reply -- and the only
  thread that reads replies is the one now sitting inside the callback.
  Nothing can wake it. `tests/emulation/smoke.c:32-35` states it: *"the
  rsvc client blocks on a semaphore for a reply, with no timeout. A
  request left unanswered is a hang, not an error."*

  `dw1000_process_events()` takes that path in double-buffer mode, so
  this is the ordinary case and not a corner. The probe is already
  immune -- `<dw1000/probe/port.h>` forbids touching the chip from the
  wake context -- but the rule must survive anyone who thinks it is
  merely tidy. Note also that this port had two independent reasons for
  the prohibition, a mutex and the reader thread; one was fixed and the
  other was not, and for a while both were believed gone. That is why the
  contract states the rule without resting it on any port's internals.

- **The same hazard has an on-board form, and stage 2 is where it
  appears.** The emulation model carried a real data race -- the
  interrupt line was recomputed from `SYS_STATUS` and `SYS_MASK` outside
  the model's lock on one path and inside it on another, so a frame
  arriving between the two reads left the derived state describing
  neither, and the rising edge was lost. Symptom: a transmission whose
  completion the host is simply never told about, although the register
  says it happened. Measured at about 6% before it was fixed.

  On a real board the same shape is available: anything that
  read-modify-writes the status register from both an interrupt handler
  and the main loop can lose an edge the same way. The probe's protection
  is already written down -- `<dw1000/probe/port.h>` forbids touching the
  chip from the wake context, so the status register has one reader --
  but it is protection by discipline, and stage 2 is the first time it
  runs against hardware where an edge actually costs a measurement.

- **What emulation can and cannot prove about the exchange.** The
  model now implements the mechanism the exchange depends on: a
  completed transmission with `WAIT4RESP` hands the chip straight to
  receive, and -- since 389a118 -- the receive timeout armed before
  `TXSTRT` counts from that turn-on, as UM §7.2.14 says it must ("each
  time the receiver is enabled"). Before that fix a POLL sent with
  `WAIT4RESP` and answered by nobody would have hung for ever under
  emulation, wearing a different hat than the board-to-Pi failure but
  the same shape. It is a test step with a mutant now.

  But the limitation is the more important fact, and it bears directly
  on the bug stage 5 found. **Emulation can tell you the exchange is
  logically correct; it cannot tell you it is fast enough.** A 0.92 ms
  responder against a software re-arm that cannot beat it is a race
  between real latencies, and the model has neither -- it has no notion
  of how long a host takes to get from `tx_done` to `rx_arm()`. A trace
  of spank's own simulation against the model made this concrete: 2266
  IDLE-to-RX transitions and not one TX-to-RX, no delayed send, no
  `WAIT4RESP`, no timeout, no double buffering. The simulation exercises
  the core model and none of the timing-dependent paths, and would never
  have exercised the probe's either. Correctness under emulation is
  worth having; the "fast enough" of §5 item 6 remains a hardware
  measurement, per platform, and the cross-platform pair is what finds
  it.

- **A model owns a thread and must be stopped, not dropped.**
  `dw1000_emulation_destroy()` joins it, must be called before
  `rsvc_close()`, and must never be called from the line callback. A test
  that creates one model and exits is fine -- threads die with the
  process -- but anything long-running that creates and discards models
  leaks a thread apiece.

  **This is the one to watch, and it will not look like itself.** The
  ordering is currently exercised only by tests that create a single
  model and destroy it at exit, so a defect in it is unexercised. A probe
  harness that creates and discards models in a loop -- sweeping a
  parameter, say, which is exactly what stage 4 does -- is the first
  thing that would find one, and it would surface as *thread exhaustion*:
  a failure to create the nth model, far from the call that leaked the
  first. Anyone debugging that from the symptom will not be looking
  here.

**Silence is not a result the responder can report.** The responder's
wait for the first `POLL` has no deadline (`UINT64_MAX`), on the stated
premise that *"the bench waits for READY before starting the
initiator"*. The bench cannot do that and never could -- console output
does not exist until the session ends. A responder that is never polled
therefore hangs, prints nothing, and is indistinguishable from one that
was never started. Every other failure in this instrument produces a
record with a status; this one produces absence, which is why it cost a
stage to characterise.

## Out of scope

Anything that makes the probe resemble the production firmware: FTM-BC,
multi-node rounds, initiator election, the UAV layer, ExtIO, the
accelerometer, the GUI, sniffing. If a stage seems to need one of them,
the stage is wrong.

Anything bench-specific inside `decawave-drivers/probe/`. If a change
there would need to know a node's address, a hub port, or the shape of
`nodes.conf`, it belongs in a shell. See the split line in §3.