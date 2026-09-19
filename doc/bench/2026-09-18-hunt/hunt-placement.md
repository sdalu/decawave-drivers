# Bug hunt: the two items left open by the 2026-09-18 hunts

Scope: `INVESTIGATE.md` entry 1 (one frame in a hundred more lost with
the receiver enabled inside the pass) and `hunt-driver.md`'s fourth
finding (a receive error beside a good frame in one status word wiping
the next frame). Evidence: the 480 duplex logs kept under
`soak-traces/` — runs 200–447 on the driver at `2084ca2`, before the
stale frame was told apart, and runs 600–647 on `222e9f8` after it; the
runs from 300 on carry the three `SYS_STATUS` words of every event pass
(`DW1000_DUPLEX_TRACE=1`, extension built with `DW1000_DEBUG=1`), every
run carries each frame's transmit and receive timestamps. Nothing on the
bench was run; nothing in the driver was changed. The scripts that
produced every number below are beside this file (`analyse-*.rb`,
`dump-run.rb`), each taking the trace directory and a run range.

---

## [Severity: Low] Entry 1 is answered by the traces: the frames the in-pass enable lost were lost by the stale pass's toggle, not by the chip's enable timing — and it stopped with `222e9f8`

**Location**
`INVESTIGATE.md` entry 1 ("Not ruled out", "Next"); the comment at
`hw/drivers/dw1000/src/dw1000.c:2005-2009` ("Measured: … with one
written here, rpi-d lost about one frame in fifty … Whether the instant
is the cause is not established"); `DW1000.md` wherever it repeats the
one-in-a-hundred figure.

**What the entry claims** that the enable's placement costs about one
frame in a hundred (z = 3.7 on rpi-d), mechanism unknown, candidate an
enable written within fifty microseconds of a frame end, and that a
host able to place the enable at chosen offsets is needed to answer.

**What the traces say**

1. *The knob's code path almost never ran on the node that showed the
   effect.* `cfg->rx_enable_early` changes one thing: in a pass whose
   status word carries a frame beside this node's own completion
   (`RXFCG | TXFRS`), it writes `RXENAB` before the read-out instead of
   deferring to the end of the pass. Every received frame's status word
   is in its `RXSEQ` line, so such passes are countable without the
   pass trace: rpi-d had **2** of them in 120 early runs (rpi-c 65).
   Whatever cost rpi-d 49 frames against 30 was not the enable's
   instant on rpi-d. (`analyse-loss-context.rb soak-traces`)

2. *The losses were not one in a hundred at random.* Prefix losses
   (start skew) removed, the rest come in episodes: on rpi-d, 45 frames
   in 29 episodes over 96 early runs, with episodes of 8, 4 and six of
   2, against 27 in 18 episodes with the end-of-pass placement. The z
   on frames overstates the effect; the unit is the episode.
   (`analyse-loss-timing.rb soak-traces 200 447`)

3. *Every lost frame can be placed against the loser's own sends.* Each
   node prints the transmit timestamp of every frame it sends (`TXSEQ`)
   and the receive timestamp of every frame it hears (`RXSEQ`); a
   linear fit of the peer's transmit instants to this node's receive
   instants, per run, has a residual of 1 ns rms, and puts the arrival
   of a frame this node never heard into this node's clock. Received
   frames never sit within about ±250 µs of the loser's own transmit
   RMARKER: that is the receiver's dead window around a send (`TRXOFF`
   before `TXSTRT`, `RXENAB` at the end of the completion's pass).
   Lost frames split in two:

   | runs 200–447, 96 per cell | inside the dead window | outside it (> 500 µs) |
   | :-- | :-- | :-- |
   | rpi-c, enable inside the pass | 17 | **9** |
   | rpi-c, enable at the end | 19 | 1 |
   | rpi-d, enable inside the pass | 30 | **15** |
   | rpi-d, enable at the end | 27 | 0 |

   The dead-window losses are the same with either placement; they are
   the collisions `DW1000.md` now describes ("two bursts paced from the
   end of each send drift into collision"). The whole of the
   placement effect is the second column: 24 frames, all in eight early
   runs (211, 261, 263, 267, 277, 335, 415, 447), arriving 1 to 4 ms
   *before* the loser's send — nowhere near any enable — and every
   other frame lost in a row (seq 9, 11, 13, 15 of run 415 on rpi-d;
   29, 31, 33, 35 on rpi-c).

4. *The pass trace names the mechanism.* `dump-run.rb soak-traces 415
   rpi-d 4 16` shows each episode beginning the same way, on the
   pre-`222e9f8` driver:

   - a frame arrives while this node's send is on the air (`TXFRB` set,
     `TXFRS` not); the `RXFCG` branch reads it out with the receiver
     off and toggles `HRBPT`, which lands the host pointer on the buffer
     the chip parked on — pointers aligned;
   - the completion's pass then sees the aligned pointers' live `RXFCG`
     and, on that driver, reports the previous frame a second time (the
     duplicate `222e9f8` fixed). **With the knob set, this stale pass
     writes `RXENAB` first**, reads out, and toggles again: the host
     pointer moves *off* the chip's buffer under a live receiver
     (after-toggle word `…|TXFRS|HSRBP`, `ICRBP` clear);
   - the next frame completes into the chip's buffer — `ICRBP` moves,
     which UM §4.3.2 reserves for a good CRC — but the host, sitting on
     the other buffer, never sees its `RXFCG`: the next completion pass
     carries `TXFRS|RXPRD|RXSFDD|RXPHD` and nothing else. That frame is
     lost unread;
   - the frame after it goes into the other buffer and is received; the
     one after that finds both buffers full: `RXOVRR`, the overrun
     recovery realigns the pointers, and the cycle waits for the phase
     drift to produce the next stale pass.

   With the default placement the stale pass toggles the same way, but
   `dw->state` is `TX` and `rx_deferred` is set, so the pass ends in
   `_dw1000_rx_start()`, whose `_dw1000_rx_sync_dblbuff()` finds
   `ICRBP != HSRBP` with no frame latched and issues `HRBPT` **before**
   `RXENAB` — `dump-run.rb soak-traces 404 rpi-d 15 18` shows the
   crossed pointers after the stale toggle and the aligned pair on the
   very next pass. The end-of-pass placement did not win by being
   later; it won by passing through the sync the early write skipped.

5. *The counts match.* Over the traced pre-fix runs (300–447, 48 per
   cell): receiver overruns **40 and 28** with the enable inside the
   pass, **0 and 0** with it at the end; completion passes showing a
   detected frame and no `RXFCG` 75 and 73 against 45 and 41 (the
   baseline being frames cut by this node's own `TRXOFF`). Stale passes
   themselves occur equally with either placement (40/41 on rpi-c,
   39/53 on rpi-d), as they must. (`analyse-stale.rb`,
   `analyse-classify.rb`)

6. *It is over.* On `222e9f8` the stale `RXFCG` is stripped before the
   branch, so the stale pass neither reads out nor toggles, and the
   knob has nothing to act on but a real frame beside the completion,
   after which the toggle lands the host on the chip's buffer as in any
   ordinary pass. Runs 600–647: 0 stale passes, 0 overruns, losses 3–4
   per cell in both placements, all in the dead window.

**Trigger** (reproduced from the traces, not on the bench) driver
`2084ca2`, double buffered, `rx_keep_on`, `rxauto` clear,
`rx_enable_early` set; a peer frame completing during this node's send,
then the completion's pass. Run 415 on rpi-d, seqs 4–16, is the
canonical trace.

**Impact** None on the driver as it stands: the default placement was
right for the wrong reason and the defect it masked is fixed. What is
wrong is the record: entry 1's "not ruled out" (an enable inside fifty
microseconds of a frame end) is ruled out — the lost frames arrived
milliseconds from any enable — and its "next" (firmware to sweep the
enable offset) is not needed. The comment in the driver and the
one-in-fifty figure in `DW1000.md` describe a measurement whose cause
was elsewhere.

**Fix** Close entry 1 into `DW1000.md` with the mechanism above;
rewrite the comment at `dw1000.c:2000-2010` to say what the early
enable actually broke (a toggle under a live receiver with the pointers
crossed, on the pre-`222e9f8` stale pass) and that the knob is kept as
a bench diagnostic only. Optionally retire `rx_enable_early`. A rule
worth writing down in `DESIGN.md`: never toggle `HRBPT` off the chip's
buffer with the receiver enabled; every `RXENAB` after a toggle goes
through `_dw1000_rx_sync_dblbuff()`.

**Test** The mechanism is fixed by `222e9f8`, whose guard has no
emulation step of its own (the model does not reproduce the aligned
pointers' live flags). Not testable here as a regression; the trace
analysis is replayable: `ruby analyse-loss-timing.rb soak-traces 200
447` must keep reporting 0 and 1 far losses for the end placement.

---

## The second item: a receive error beside a good frame

**What was open** `hunt-driver.md`'s fourth finding: in a pass whose
snapshot carries `RXFCG` with a non-swinging error (`RXPHE`, `RXRFSL`,
`RXSFDTO`, `AFFREJ`, `LDEERR`, `RXRFTO`, `RXPTO`), the `RXFCG` branch
re-enables the receiver, reads out and toggles; a second frame landing
during `rx_ok` becomes visible after the toggle; the error branch's
`_dw1000_txrx_off()` then clears `ALL_RX_GOOD` and its sync toggles the
second frame's buffer away unread. Reasoned; 0 such snapshots in the
traces; the emulation could not raise the case.

**Why the bench count is zero by construction, not by luck** The
finding's trigger says `rxauto = 0`. With `RXAUTR` clear the receiver
is idle after a good frame *and* after an error (UM §7.2.6, §5.3.2), so
after `RXFCG` no error can be raised until something writes `RXENAB` —
and in this host every `RXENAB` is written either inside the pass,
after its snapshot, or by `dw1000_rx_start()` after a
`dw1000_txrx_off()` that has already dropped the status. A snapshot
carrying both therefore needs two receiver runs with no pass between
them, which `RXAUTR` clear forbids. The 43 000 traced passes hold 5
non-overrun errors (`RXSFDTO`, `RXPHE`), none beside `RXFCG`
(`analyse-stale.rb`, "other err bits"). The traced roles are not where
this case lives; the raw `overrun` role (`rxauto` set) is.

**What would raise it** `RXAUTR` set: frame 1 completes into buffer A
and the receiver re-arms itself; a frame then fails in the PHY header
(`RXPHE`, no `ICRBP` move, buffer B reused) and the receiver re-arms
again; the host's snapshot reads `RXFCG_A | RXPHE`. The finding's walk
then holds as written, provided a third frame lands during `rx_ok`.
Both conditions are deterministic in the emulation once it can fail a
frame on demand; on the bench they need `rxauto`, a stalled host and an
error source (a second flooder for collisions), which is a session of
its own and would still be statistics.

**Delegated** an emulation knob `dw1000_emulation_fail_next_frame()`
and a step `step_error_beside_a_good_frame` in
`tests/emulation/dblbuff.c`, in a worktree of this repository on branch
`hunt/emulation-rx-error`; the result is appended below when it lands.

**Result** Reproduced in the emulation, deterministically (five repeat
runs, identical words). The patch is `emulation-rx-error.patch` beside
this file (three files, nothing under `hw/`), the delegate's report
`emulation-rx-error-report.md`, the gate's output
`emulation-rx-error-gate.log`; the patch is applied to the working tree of
this repository (the delegate's worktree and branch are retired). `sh tests/check-emulation.sh` there, re-run by the
delegator: smoke, timing and timing_dblbuff pass; dblbuff and
dblbuff_longframe fail on the new step and on nothing else:

```
status after frame 1        : 0x80006f03   RXFCG, ICRBP 1, HSRBP 0
status after frame 2        : 0x80007f03   the same with RXPHE
status inside rx_ok, entry  : 0x80007f03
status inside rx_ok, frame 3: 0x00000b02   ICRBP moved 1 -> 0: frame 3 is in
status handed to rx_error   : 0x80007f03   the snapshot, predating frame 3
status after the first pass : 0x00000002   no RXFCG, pointers (0,0)
second pass                 : 0 rx_ok, 0 rx_error
```

The walk is the finding's: after `rx_ok` the branch toggles `HRBPT`,
which lands the host on frame 3's buffer with its `RXFCG` latched; the
error branch's `_dw1000_txrx_off(ALL_RX_ERR|ALL_RX_TO|ALL_RX_GOOD)`
clears that latch, and its `_dw1000_rx_sync_dblbuff()`, whose guard
against "a frame the host has not been told about" reads the latch it
has just cleared, issues `HRBPT` and hands frame 3 back to the chip.
The second pass has nothing to report. **Finding 4 of `hunt-driver.md`
is confirmed**, with the correction that it needs `RXAUTR` set; its
severity stands at Medium (a silent lost frame, in a configuration the
driver keeps off for senders and the raw roles use for receiving).

**Fix** as `hunt-driver.md` proposes: give the error and timeout
branches a clear mask of `ALL_RX_ERR | ALL_RX_TO` plus only the
good-frame bits that were in the snapshot, and skip the `TRXOFF`/reset
when the `RXFCG` branch of the same pass has already re-armed the
receiver. **Applied** the mask half on 2026-09-18 (`rx_drop`, defined just above the timeout branch of `dw1000_process_events()`): `ALL_RX_GOOD` is dropped only when no good frame was reported in the same pass. Cycled: all five emulation builds pass with it, `dblbuff` and `dblbuff_longframe` fail on the new step with it reverted, pass again restored. The `TRXOFF` and the UM §4.1.6 reset are kept. **Then the second half,
the same day, on request:** once the reset is applied and before the
`rx_error` callback, both branches put the receiver back through
`_dw1000_rx_start()` when the policy wants it (`rx_keep_on` and
`rx_wanted`), instead of leaving it to the end of the pass; the reset
is not skipped. Second emulation step
`step_receiver_back_before_rx_error` (policy on for its run): `rx_error`
finds `PMSC_STATE` 5 with a fourth frame already complete, and the two
passes after it report frames 3 and 4 in order. Cycled: all five builds
pass with it; with only the mask half in place, the step fails on
`PMSC_STATE 1` inside `rx_error`.

**On the bench**, rpi-b, rpi-c and rpi-d, 2026-09-18 evening (the
scripts named `DW1000_DRIVERS_DIR`, which nothing reads, so the nodes
built ruby-dw1000's submodule pin rather than the working tree; the pin
was this tree's HEAD, `65b0339`, so the driver measured is the one
described; the scripts now set `DECAWAVE_DRIVERS_DIR`, the variable
`ext/vendoring.rb` reads):
- `rake test:pair` with `DW1000_MIN_DELIVERY=0.9`: 9 runs, 78
  assertions, 0 failures — both duplex directions 50 of 50, the overrun
  role's recovery clean (2 overruns, 2281 frames after the first), the
  floods complete.
- `bench-errbranch.rb jam` (beside this file): rpi-c counts a gap-less
  3000-frame flood from rpi-d while rpi-b floods 3000 frames of a
  foreign prefix on top, the event passes traced. Three runs: 18, 56 and
  19 passes took the error branch (collisions: CRC and header errors),
  0 the timeout branch; no event discrepancy, no warning, no corrupt
  frame; in the kept run (`errbranch-jam-rpi-c.log`) 2943 receptions
  over 2962 passes and 42 of them after the last error. The receiver
  came back from every error through the new path. The baseline without
  the jammer received 1969 of 3000 (the Pi's gap-less ceiling,
  `DW1000.md` "The bench, for scale"); the jammed runs 848 to 2077 of
  the 3000 counted frames, the rest being the jammer's, received but not
  counted.

**Test** `tests/emulation/dblbuff.c:step_error_beside_a_good_frame`,
in the patch, both builds; it fails on `7226204` for the reason above
and is the regression test for the fix. The knob raises `RXPHE` only;
`RXRFSL`, `AFFREJ`, `LDEERR` and `RXSFDTO` are the same class in the
driver's branches and would need the knob widened to be covered.

---

## Tooling

**Ran**
- `ruby -wc` on the five analysis scripts: clean.
- `analyse-loss-context.rb`, `analyse-loss-timing.rb`, `analyse-stale.rb`,
  `analyse-classify.rb` over `soak-traces/`, for the run ranges 200–447,
  300–447 and 600–647; their output is quoted above verbatim.
- `dump-run.rb` on runs 415 (rpi-d, seqs 0–16; rpi-c, 27–36), 447,
  335, 261, 277 and 404.
- Nothing on the bench. All four nodes were idle at 17:55 (no `ruby` or
  `node-run` process, `pgrep` over ssh), so the raw-role experiment
  above could be run; it was not, because the emulation settles the
  same question deterministically.

**Install to widen the hunt** nothing missing for this hunt. The
emulation gains one knob from the delegated piece; a
`dw1000_emulation_park_aligned()` that reproduces the aligned pointers'
live flags would let `222e9f8`'s guard carry a step of its own, which it
does not today.

## Coverage

**Examined** every kept duplex log (240 runs, both nodes): the
per-frame status words, the transmit and receive timestamps, and, from
run 300 on, the three status words of every pass; the driver's
`dw1000_process_events()` at `7226204` branch by branch for what
`rx_enable_early` changes and for every path that writes `RXENAB` or
`HRBPT` (`_dw1000_rx_start()`, `_dw1000_rx_sync_dblbuff()`,
`_dw1000_rx_overrun_recover()`); `INVESTIGATE.md` entries 1 and 2's
"bench numbers"; `hunt-driver.md` finding 4 against UM §4.3.2, §5.3.2,
§7.2.6 as cited in the driver's own comments.

**Not examined** the 24 absolute-schedule runs `DW1000.md` cites (not
under `soak-traces/`, no traces kept); the chip's per-buffer state
behind "ICRBP moved but the host never saw RXFCG" — the trace shows the
effect, not the chip's bookkeeping; the ruby-dw1000 host outside the
duplex role's trace lines.

**Stop condition** the two items were the scope; item 1 is answered
from data already collected and item 2 is reduced to one deterministic
reproduction, pending in the emulation. A pass over the checklist for
siblings of the confirmed defect — a toggle off the chip's buffer under
a live receiver — found no non-debug path in the driver at `7226204`
that does it: every `RXENAB` outside the `RXFCG` branch follows the
sync, and the branch's own enable-then-toggle lands the host on the
chip's buffer.

## Summary

Entry 1's effect was real and is explained: with the enable written
inside the pass, the pre-`222e9f8` stale pass toggled the host pointer
off the chip's buffer under a live receiver and skipped the sync that
the end-of-pass placement runs, losing every other frame until an
overrun realigned the pointers — 24 frames outside the dead window
against 1, all in eight early runs, none since the stale frame was told
apart. The chip's enable timing is not involved and no firmware
experiment is needed. The second item cannot occur with `RXAUTR`
clear, which is why the bench never showed it; with `RXAUTR` set it is
reproducible on demand, and that reproduction is being built in the
emulation.
