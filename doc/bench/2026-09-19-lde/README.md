# 2026-09-19: the frame whose LDE run a TRXOFF cut

The instrumented duplex soak that settled INVESTIGATE.md entry 6:
rpi-c and rpi-d through ruby-dw1000, `DW1000_DEBUG=1` (the driver's
`DW1000_WITH_DEBUG` three-point status trace compiled in), 60 runs at
the end-of-pass receiver enable, the driver being the restructured
event pass of this same day.

- `soak.jsonl`: one JSON line per node per run. 4481 of 6000 received,
  1 duplicate (run 41, rpi-c).
- `traces/run-041-rpi-{c,d}.log`: the run with the duplicate, every
  pass with its three words (entry, after the masked clear, after the
  toggle). Lines 148 to 151 of the rpi-c log are the failing pass and
  the one after it.
- `traces/run-{014,019,020}-rpi-c.log`: the three other deliveries
  with `LDEDONE` clear at entry, each with a stale stamp and no
  duplicate after it.
- `analyse-lde.rb`: the count. Over the 60 runs: 4482 deliveries, 5
  with `LDEDONE` clear at entry, all 5 with `HSRBP == ICRBP` and one of
  the node's own transmit bits set, 4 of them carrying the `RX_TIME`
  stamp of an earlier frame received into the same buffer and the
  fifth, the first frame of its run, a stamp of 0; 0 of the other
  4477 had `LDEDONE` clear, and the run's one duplicate follows one of
  the five. What it means is `DW1000.md`, "A TRXOFF
  between RXFCG and LDEDONE leaves the frame without its timestamp".

The driver reached the nodes through `DECAWAVE_DRIVERS_DIR`.

## The wait that never ended, 2026-09-19 later

Two more instrumented soaks of 60 runs, with a driver that waited for
`LDEDONE` before the `TRXOFF` of its own send and once more in the
event pass, each wait bounded at 1 ms of the chip's own clock, and
the wait counting itself (`dbg_lde_waits`, `dbg_lde_wait_done`,
`dbg_lde_wait_max`, the last three words of every `PASS` line):

- `soak-wait.jsonl`, `traces/wait-run-013-rpi-c.log`: the wait on any
  word with `RXFCG` and no `LDEDONE`. 4415 deliveries, 11 with
  `LDEDONE` clear at entry, 27 waits, every one of them run to the
  bound (1010 to 1046 us), 0 ended by `LDEDONE`, 1 duplicate. In run
  13 (line 60) the cut frame is reported in the pass that follows its
  send's `TXSTRT`, the wait counter moving from 0 to 1 on that very
  pass: the wait before the `TRXOFF` had not fired, the frame's
  `RXFCG` not being posted yet when the send path read the status.
- `soak-wait2.jsonl`, `traces/wait2-run-004-rpi-c.log`: the wait only
  on words carrying a detect bit (`RXPRD|RXSFDD|RXPHD`), so never on
  the stale record of an aligned pair. 4971 deliveries, 19 waits, all
  at the bound, 0 ended by `LDEDONE`, 0 duplicates. Every wait sits on
  a pass that reported a cut frame (lines 297 and 539 of run 4, two
  frames of the filler burst, which the harness does not print as
  `RXSEQ`), and none on the send path.

So a frame that shows `RXFCG` without `LDEDONE` never gets it: the
`TRXOFF` terminated the reception after the payload and its CRC were in
and before the leading-edge run, `RXFCG` is posted for it all the same,
and `RX_TIME` and `ICRBP` stay as they were. No wait of any length
helps, the wait before the `TRXOFF` cannot see the frame, and the two
waits were removed; what stays is the pass reporting such a frame with
`LDEDONE` clear and toggling nothing, which is what took the duplicate
from 1 in 6000 to 0 in 4971.

## Without the waits, 2026-09-19 later still

`soak-nowait.jsonl`, `traces/nowait-run-{004,043,048}-rpi-c.log`: the
driver with both waits removed and the cut handling kept (a cut frame
reported with `LDEDONE` clear, no early enable, no toggle). 60 runs,
4504 of 6000 received, 36 cut frames reported (the filler frames
included, `dbg_lde_cuts`), 13 of them counted frames with a stale
stamp, and 3 duplicates. Two of the three follow a cut frame: run 4,
the cut seq 7 at line 21 reported with the pointers aligned and left
so, then seq 5 delivered a second time at line 109 on the word
`0x40806402`, the pointers apart (`HSRBP` 1, `ICRBP` 0) and no detect
bit; run 48, the cut seq 18 at line 81, then seq 16 again at line 280
on the same shape. Run 43's is the older shape, after a receive error.
So the toggle after the callback is not the only route to the host
standing apart on a buffer whose latch it never cleared: with the cut
frame left untoggled the pointers still come apart later, by a move
these words do not show (`ICRBP` advancing late, or a sync toggle on
a word the driver read as clean), and the redelivery follows. What
tells a redelivery apart in every case seen, the aligned ones of
2026-09-18 and the apart ones of 2026-09-19 alike, is the stamp: it is
the `RX_TIME` of a frame already delivered, and the word carries no
detect bit. `analyse-lde.rb` counts a duplicate that way.

## As a receive error, 2026-09-19 afternoon

`soak-error.jsonl`: the driver reporting a cut frame through `rx_error`
(`RXFCG` set, `LDEDONE` clear in the word it hands over, no payload
offered, no toggle), 60 runs with the counters on. 4649 of 6000
received, 25 cut frames reported as errors (the `dbg_lde_cuts` sum over
the runs and the count of `rx_error` passes whose entry word carries
`RXFCG` without `LDEDONE` agree), 0 deliveries with `LDEDONE` clear,
0 stale stamps, 0 duplicates. The pair suite the same afternoon: 9 runs,
78 assertions, 0 failures.
