# What the campaign owes the rest of the tree

The bench campaign of 2026-09-14 to 18 established what `DW1000.md`
records and changed what `hw/drivers/dw1000/README.md` requires of a
host. Eight consumer trees were surveyed against those findings on
2026-09-18. This file says what each is exposed to and what is owed,
so that the findings reach the code that is wrong rather than staying
in the document that is right.

Every pointer below was checked at the surveyed tree's HEAD on
2026-09-18. `DW1000.md` holds the evidence for each finding; this file
holds only where it lands.

## Where each consumer stands

| Tree | How it reaches the chip | Driver it has | Standing |
| :--- | :--- | :--- | :--- |
| ruby-dw1000 | its own extension, directly | `65b0339` (HEAD) | current |
| mynewt-redskin | spank shim | `65b0339` (HEAD) | ported 2026-09-19, unbuilt |
| zephyr-redskin, in `~/Repos` | spank shim | symlink to HEAD | has the fix, not the work |
| zephyr-redskin, in `~/ZephyrProjects` | spank shim, and `probe/` directly | symlink to HEAD | **the flashed tree, and it lacks the fix** |
| rpi-redskin | spank shim, and a forked second consumer | `88f3351` (8 behind) | behind |
| spank | the portable shim itself | consumer supplied | current |
| ruby-ftmbc | the ruby-dw1000 gem, directly | via the gem | current |
| rpi-uwb-sniffer | its own copy | `f9a2e2c` (44 behind) | superseded |
| uwb-sniffer | nothing: 2018 sources, untracked | a symlink | dead |

## 1. The boards in the field do not have the RXAUTR fix

`49ce758` established that `RXAUTR` set under a sender costs one send
in about seven thousand, and the fix went out to all three firmwares
on 2026-09-18 within ten minutes of each other: `cc3e6b1` in spank,
`40e84ac` in rpi-redskin, `d40f9a6` in mynewt-redskin, `8a5b528` in
zephyr-redskin.

**It went to the wrong zephyr tree.** There are two, and they have
diverged in both directions:

- `~/Repos/zephyr-redskin` is at `8a5b528` and carries the fix.
- `~/ZephyrProjects/zephyr-redskin` is the west workspace: it holds
  `probe/`, `doc/`, `flash-all.rb` and every build directory, and it
  is what is compiled and flashed. Its HEAD is `d8e00c4`, nine
  unpushed commits on top of `493797b`, the parent of the fix. The
  commit object `8a5b528` does not exist in it at all, and its
  `origin/ftm-bc` ref is stale at `493797b`.

So the flashed firmware still reads `.rxauto = 1`, in both of its
DW1000 consumers: `redskin/src/redskin.c:188` and
`probe/src/main.c:122`.

The nine commits the workspace holds and `~/Repos` does not are the
whole of the Zephyr probe (its shell, its double-buffered receive,
its error counting, its `config` command) plus a stack-overflow fix.
Neither tree is a superset of the other.

**The reconcile is trivial, though.** The two sides do not overlap:
`8a5b528` is one line, at `redskin/src/redskin.c:188`; the nine
workspace commits are 1638 insertions and no deletions, of which
`probe/` is a directory `~/Repos` does not have at all, and the only
`redskin.c` edit among them is at line 506, three hundred lines away.
Their merge base is `493797b` and the merge is conflict free, tested
on a scratch clone of both. So this is a fetch and a rebase, or a
single cherry-pick of the one-line fix; the history diverges but the
content does not.

**Owed, and first:** fetch in the workspace, reconcile the two trees,
and re-flash. Everything else in this file can wait; this cannot,
because it is the one finding that is currently costing frames on
hardware.

No measurement is in doubt because of it. Every ruby-dw1000 bench run
of 2026-09-18, duplex, pair suite and jam alike, used Raspberry Pi
peers running the ruby host built from this tree, so no counted result
sits on a Zephyr receiver. The divergence costs frames in the field,
not confidence in `DW1000.md`.

## 2. The driver's version cannot express any of this

`1.3.1` has been the answer since `61f1d34` on 2026-09-14, and 28
commits have touched `hw/drivers/dw1000/` since: the transmit error
taxonomy (`28125d7`), `rx_keep_on` (`2084ca2`), the channel 5 analogue
values (`0050df5`), the stale-frame guard (`222e9f8`), the enforced
IDLE precondition (`06cee4f`). The last tag in the tree is `v1.2.0`;
neither 1.3.0 nor 1.3.1 was ever tagged.

A consumer at `f9a2e2c`, 44 commits stale and still writing the old
channel 5 values, answers `DW1000_VERSION_AT_LEAST(1, 3, 1)` with yes,
exactly as HEAD does. `hw/drivers/dw1000/README.md`, "Guard against
the version you need", therefore tells applications to guard on a
number that has not moved, and no consumer can express what it needs.

Every tree surveyed was found without a version guard, which until now
was the rational choice.

**Owed:** bump, tag, and say in the README which release carries which
contract change. Nothing else here can be asked of a consumer first.

**And push.** `master` is two commits ahead of `origin/master`
(`052066c` and `65b0339`), so the release a consumer would be asked to
pin does not exist on the remote yet. A submodule bumped to an
unpushed SHA breaks every fresh clone, and a tag on one helps nobody.
mynewt-redskin's vendored clone is two ahead of its own origin in the
same way, which is how it came to be at HEAD.

## 3. The proprietary SFD is on by a default nobody chose

`DW1000_WITH_PROPRIETARY_SFD` defaults to 1 in
`hw/drivers/dw1000/include/dw1000/dw1000.h:47`, Zephyr's
`zephyr/Kconfig:31` defaults its Kconfig to `y` to match, and every
consumer tests it with `#if`. So `.sfd = 1` compiles in everywhere
without anyone deciding it: `probe`, `sniffer/app/unix/main.c:184`,
`spank/port/hal/io/dw1000/src/driver.c:69` (and so all three
firmwares), the Zephyr probe, `rpi-uwb-sniffer`.

`DW1000.md` measured the cost: 40 of 40 frames heard when two nodes
agree on the bit, 0 when they differ, in either direction. The fleet
is therefore internally consistent, and deaf to every standard-SFD
DW1000 there is: a DWM1001 on factory firmware, a commercial tag,
another lab's node.

The sniffer is the acute case, because pointing it at traffic nobody
here configured is its whole purpose, and `sniffer/app/unix/cmdline.c`
exposes channel, bitrate, PRF, preamble length, PAC and both preamble
codes but not this bit. A mismatch reads as silence with every drop
counter at zero, which is the hardest failure on the bench to
diagnose.

The one place that reasons about the bit is the Zephyr probe
(`probe/src/main.c:178-190`), and it is about the RXPACC adjustment
and the power estimate, not about who can hear whom.

**Owed:** a `--[no-]sfd` on the sniffer, and the choice recorded where
each radio is configured.

## 4. One uncalibrated antenna delay, seven sites

`INVESTIGATE.md` entry 3 holds the recalibration owed after the
channel 5 analogue values changed on 2026-09-16, worth roughly 4 cm.
The constant it applies to is written out seven times in five trees:

| Tree | Site | Value |
| :--- | :--- | :--- |
| decawave-drivers | `probe/app/unix/main.c:117` | 154.6 |
| decawave-drivers | `sniffer/app/unix/uwb_dw1000.c:128` | 154.6 |
| spank | `simulation/node/main.c:408-409` | 154.6 |
| rpi-redskin | `main.c:488-489` | 154.6 |
| mynewt-redskin | `apps/redskin/src/redskin.c:88-89` | 154.6, untouched since 2018-06-19 |
| ruby-ftmbc | `bin/node-run:129` | 154.6 |
| zephyr-redskin | `redskin/src/redskin.c:192-193`, `probe/src/main.c:96` | 154.2 |

The entry reads as one bench session with a tape measure. It is that,
plus a seven-site edit across five repositories, with two different
values already in circulation. Until it happens every one of these
trees quotes distances carrying the same bias.

The Zephyr probe records its own value as "a neutral round-trip
placeholder, not calibrated for this instrument"
(`probe/src/main.c:79-81`), and notes that it must equal what redskin
configures or the two are not comparable. That constraint is the
reason this has to be one edit rather than seven.

## 5. The delayed send's launch gain is larger than the bias being chased

`DW1000.md` puts the counter's gain at 2.5 to 3.4 ppm of the lead,
taken at the launch and carried by every later timestamp.

`ruby-ftmbc/lib/ftmbc.rb:76` and `spank/include/spank/config.h:66`
both use a 400 microsecond lead. At 3 ppm that is 1.2 ns, about 36 cm
of range: an order of magnitude above the 4 cm question of section 4.
All three of ruby-ftmbc, spank and zephyr-redskin already default
embedded transmit off, which is the right call, and
`zephyr-redskin/redskin/Kconfig:54-59` states the reason in the help
text.

Two of those statements are now wrong in the same way.
`ruby-ftmbc/bin/node-run:46-48` and `zephyr-redskin/redskin/Kconfig`
both say the clock runs fast *while a delayed send is pending*. That
was refuted on 2026-09-18: a send armed and taken back by `TRXOFF`
gains 0.3 ns whatever the lead, against 5.4 to 46.1 ns when it fires.
The gain comes with the launch. The defaults are right; the reason
recorded beside them is not.

**Owed:** correct both comments to point at `INVESTIGATE.md` entry 2,
which already carries the refutation under "Ruled out ... the wait
itself, on 2026-09-18", and carry the 36 cm into whatever error budget
quotes a distance. That entry is also what would let embedded transmit
be turned on at all.

## 6. Per tree, what is left

### decawave-drivers, here

- `sniffer/app/unix/uwb_dw1000.c:123` justifies `rxauto = 1` as what
  "probe and rpi-redskin both have". Neither does
  (`probe/app/unix/main.c:137` sets 0; rpi-redskin took `40e84ac`).
  The setting is right for a pure listener and the README says so; the
  reason cited is stale.
- `probe` and the SPANK firmwares still drive the receiver themselves.
  `cfg->rx_keep_on` (`2084ca2`) is what would replace that, and
  ruby-dw1000's engine already runs under it with no re-arm left.
  Nothing is broken; the migration is simply not done, and the README
  already says so.

### rpi-redskin

**A second DW1000 consumer the fix missed.** `40e84ac` put
`.rxauto = 0` in `main.c:487`, but `ext/dw1000.c` is an independent
Ruby extension with its own `dw1000_config_t`, still at
`.rxauto = 1` with `.dblbuff = 0` (`ext/dw1000.c:426-428`) while
exposing `tx_start` and `tx_send` (`:639`, `:678`).

It is a 1150-line stale ancestor of the gem's extension
(`ruby-dw1000/ext/dw1000.c`, 3105 lines), last touched 2026-08-26,
before the campaign. The gem now defaults `rxauto: false`,
`dblbuff: true` and `rx_keep_on: true`, and its comment that "dblbuff
is not fully implemented" (`ext/dw1000.c:417`) has been overtaken. It
builds separately and nothing in `build.sh` or the top `Makefile`
reaches it, so it is a side tool rather than shipped code. **Owed:**
retire it in favour of the gem, or say in it that it is superseded.

**The submodule is 8 commits behind**, and cannot be bumped until the
two unpushed commits of section 2 reach the remote. The bump forces no
code change
(`git diff --stat 88f3351..65b0339 -- hw/drivers/dw1000/include` adds
fields only). Three sites want re-reading after it: the manual re-arm
in every callback (`main.c:142,163,177,196,385,400`) against
`rx_keep_on`; `_rx_error`'s assumption that the frames it held are
gone (`main.c:181-197`) against `052066c`, which keeps a frame that
landed during `rx_ok`; and `ext/dw1000.c:639,678`, whose
caller-reaches-IDLE precondition the driver now enforces itself.

### mynewt-redskin

Not a museum piece: its vendored driver is at `65b0339`, HEAD, which
makes it the best synchronised of the three firmwares, and it has used
decawave-drivers rather than Decawave's own since 2018 (`935ffbf`).

One item owed beyond the common ones:
`apps/redskin/src/redskin.c:80-96` sets no `.dblbuff`, so it is single
buffered, alone among the live consumers, and `DW1000.md` measured a
single-buffered receiver deaf while a frame is read out. Whether
FTM-BC's broadcast rounds ever put two peers' frames inside that
window is a timing question no survey can settle; the `rx_error`
callback the pairing needs is already there (`redskin.c:93`).

### ruby-ftmbc

Drives the chip directly through the gem, not through spank over
ExtIO: it is a wire-compatible reimplementation of spank's FTM-BC, not
a client of it (`bin/node-run:11`). Owed: the corrected comment of
section 5 and the antenna delay of section 4.

### spank, spank-extio

spank carries almost nothing of its own: the shim is thin and correct,
and the exposure is in what its consumers hand it. Two small things:
`SPANK_DRIVER_TX_POWER` (`port/hal/io/dw1000/include/spank/driver.h:145`)
returns the cached context word while `spank_driver_info()` reads the
chip, which is an inconsistency between two accessors rather than a
defect; and `spank.cmake:119` still says the simulation runs against
decawave-drivers 1.3.0.

spank-extio and spank-extio-demo owe nothing: pure transport for
values spank computed on the MCU, no register write, no estimator, no
calibration constant, and the demo's checked-in campaign data are from
2023.

### The bench harnesses

`DW1000.md`, "The bench, for scale", settled that 299 of 355 frames
counted as lost were prefixes the later node never listened for, and
that the apparent link asymmetry was whichever node the harness
brought up second. Two harnesses still have the shape that caused it:

- rpi-redskin's synchronisation hook is dead.
  `main.c:601-609` defines
  `spank_ftmbc_defer_initiator_schedule_send()` entirely inside a
  comment, so spank's weak default (never defer) is what links, and
  the `I_ULOCK` flag it was to read is written and never consulted
  (`main.c:588-593`). `test.sh:23` starts the two nodes with a
  `sleep 1` between them and no confirmation that both are listening.
- the Zephyr probe paces a burst from the end of each send.
  `probe/src/shell.c:324-350` sleeps `gap_ms` after each
  `dw1000_tx_start()`, which is the shape `DW1000.md` found drifts two
  nodes into each other within a few dozen frames. The fix is the
  absolute schedule, the i-th frame at start plus i times the gap.

ruby-dw1000's harness already says GO to both roles once both are
listening (`test/support/remote.rb`, `Job#go`) and paces absolutely;
it is the model for the other two.

### The sniffers

`rpi-uwb-sniffer` is superseded. `8ce5fdd` folded it into this tree as
`sniffer/`, which carries its pin map corrected, its options extended
and its own unclosed TODO ("use dw1000 double buffer") closed, and
adds pcapng, dissectors and loss accounting it never had. Its every
exposure traces either to the 44-commit-stale pin (the old channel 5
values at `decawave-drivers/hw/drivers/dw1000/src/dw1000.c:210`, no
stale-frame guard) or to shapes the in-tree sniffer documents having
corrected: single buffered, a blocking `sendmsg(2)` inside `rx_ok`,
and no `rx_error` callback, so the receive error bits are never
unmasked at all. Nothing is worth rescuing. **Owed:** retire it.

`uwb-sniffer` is neither a repository nor a sniffer: an untracked
directory of 2018 ChibiOS two-way-ranging firmware for an STM32F042K6,
every file dated 2018-07-05, whose `decawave-drivers` is a symlink to
the sibling checkout rather than a pin. Its apparent recent commits
belong to the home directory's own repository above it.

## 7. The frame a node's own send cut is now a receive error

A `TRXOFF` issued before a send can terminate a peer's frame after its
payload and its CRC are in and before the leading-edge run, which is
what writes `RX_TIME`: the chip posts `RXFCG` for it, never posts
`LDEDONE`, and leaves `RX_TIME` holding the stamp of an earlier
reception into the same buffer (`DW1000.md`, "A TRXOFF between RXFCG
and LDEDONE leaves the frame without its timestamp, and the IC pointer
where it was"; 5 of 4482, 11 of 4415 and 13 of 4507 deliveries over the
three duplex soaks of 2026-09-19, `doc/bench/2026-09-19-lde`). Until
now the driver handed such a frame to `rx_ok` with `LDEDONE` clear in
the status and left the bit for the host to test. It now reports it
through `rx_error` instead, with `RXFCG` set and `LDEDONE` clear in the
word it hands over, and offers no payload for it.

A survey of every consumer on 2026-09-19 is why. Not one of them tests
`LDEDONE`, and every ranging host used the stale stamp as the frame's
own, silently:

- **spank**: `spank/src/io.c:233`, in `spank_io_get_from_driver()`,
  reads the timestamp. The status word does not reach that function at
  all: the callbacks hand the io layer an event, not a word, so the bit
  could not be tested there even by a host that wanted to.
- **ruby-dw1000**: `ext/dw1000.c:516-529` reads the stamp inside the
  callback, where it must be read, and `lib/dw1000/io.rb:479-502` puts
  it into `Meta#ts` beside `Meta#status`. The word is carried; nothing
  looks at this bit in it.
- **ruby-ftmbc**: `lib/ftmbc.rb:540-542` takes `meta.ts` as the round's
  reception instant.
- **the probe**: `probe/src/exchange.c:320` stores
  `dw1000_rx_get_rmarker_time()` into the capture ring.
- **the three redskin firmwares**: their `rx_ok` callbacks drop the
  status word (`zephyr-redskin/redskin/src/redskin.c:243-247` names it
  `ARG_UNUSED`; `rpi-redskin/main.c:124-130` ignores it), and the stamp
  is then read through spank, above.

**Owed: nothing beyond rebuilding on this driver.** Every one of these
already has an `rx_error` callback, re-arms from it and counts it, so
a cut frame arrives as one more receive error and is accounted as one.
A host that wants to count these apart tests `RXFCG` set with
`LDEDONE` clear in the word `rx_error` is handed, which no other error
carries. A node that only listens never sees one: the cut is its own
`TRXOFF`.

The same survey found one thing that is not about this change:
`mynewt-redskin/apps/redskin/src/redskin.c:120-123`, `_rx_ok()`, called
`spank_io_on_event()`, which no longer exists anywhere in spank (the
event entry points are `_spank_io_evt_rx_received()` and its
siblings, `spank/include/spank/io.h:241`), so that tree did not build
against the spank it links.

**That port was done on 2026-09-19.** `apps/redskin/src/redskin.c` now
has rpi-redskin's shape: the four callbacks take `dw1000_t *`, call the
`_spank_io_evt_*` entry points and re-arm the receiver themselves; the
chip is driven from one task only, reached by an `os_eventq` carrying
three request kinds (IRQ, TX, TX-ABORT), which is where the app's
`spank_driver_post_tx()` and `spank_driver_tx_abort()`
(`spank/port/hal/io/dw1000/include/spank/driver.h:131,139`) now put
their work. It stays single buffered and keeps `rxauto 0`. SPANK's
mynewt osal was brought up with it: `SPANK_STAT`/`SPANK_STAT_INC`,
`spank_printf`, `spank_output_lock`/`unlock`, the `SPANK_FMT*` systime
formats and `__SPANK_CONSTRUCTOR` were all missing, and portable SPANK
requires every one of them. The driver's own mynewt osal
(`port/mynewt/dw/osal`) was found current and untouched.

The rest of the application went with it. `apps/redskin/src/cmd.c`
loses the two `spank_config_t.orchestration` keys of its `spank config`
command, which went with the token SDS-TWR protocol (`a3c50f7`) and
which nothing replaced, and `apps/redskin/src/spank_neighbours_table.c`
is deleted: its `SPANK_SWARM_SIZE`, `SPANK_NODE_DEF_UAV` and
`SPANK_NODE_DEF_ANCHOR` are all gone from spank, nothing referenced the
table, and the current node API discovers peers into a pool rather than
declaring them.

**Two lines of spank's core were touched, and they travel.** Mynewt 1.6
has no `LIST_FOREACH_SAFE` in its `sys/queue.h`, so `spank/src/node.c`
now carries the BSD form under `#ifndef` above its first use, and
`spank/src/spank.c` now includes `<errno.h>` for the `EOPNOTSUPP` it
already used. Neither changes behaviour on any other port, but both
land in every tree that rebuilds spank.

**Owed:** this has been compiled, never built and never run.
`helpers/check-syntax.sh` in that tree type-checks the application and
the whole of `SPANK_SOURCES` against the real headers without newt or a
cross compiler, 21 files, and passes with no error; it links nothing and
touches no board. A `newt build redskin`, a flash and a pair run on the
DWM1001 are all still owed.

## What was checked and found already right

Worth recording, so it is not re-surveyed: the IDLE-before-send
discipline and the CRC-stripping receive path in spank
(`port/hal/io/dw1000/include/spank/driver.h:221`, `spank/src/io.c:199`);
the interrupt masks in rpi-redskin (`main.c:91-96`), zephyr-redskin
(`redskin.c:103-108`) and spank's simulation, all matching their
registered callbacks exactly; the `dblbuff` plus `rx_error` pairing
everywhere it is set; the do-not-re-arm-under-dblbuff rule in
rpi-redskin (`main.c:136`) and zephyr-redskin (`redskin.c:254`); the
lead-time check at start-up in rpi-redskin (`main.c:674-694`) and
zephyr-redskin (`redskin.c:520-531`); the transmit power read-back in
the Zephyr probe (`probe/src/shell.c:252`); and the frame ring in
`probe/src/exchange.c:306`.
