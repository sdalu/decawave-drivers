# What is still to be investigated

The open questions about the chip and the bench, as of 2026-09-18:
what was seen, what has been ruled out, what could explain it, and the
experiment that would settle it. `DW1000.md` holds what is
established; `AUDIT.md` what was checked against the manual. This
file holds what neither can claim yet, so that the next session
starts from the last one's evidence rather than from the symptom.

An entry leaves this file when its experiment has been run, whichever
way it went: a settled cause goes to `DW1000.md`, a refuted one stays
there under "Unexplained" with the refutation attached.

## The bench's own numbers, for reading the rest

Two nodes, rpi-c and rpi-d, DWM1000 modules on Raspberry Pis, 6.8
Mbps, 128-symbol preamble, PRF 64 MHz, channel 5, double buffered,
receiver auto re-enable off. Sending at each other every 30 ms, 50
frames a run each way, twelve runs a soak. Where things stand after
the driver as of `2084ca2`:

| | Result | Runs |
| :-- | :-- | :-- |
| Sends written and never completed | 0 in 440 000 frames | six soaks |
| rpi-d hearing rpi-c | 47 to 50 of 50, mostly 49 or 50 | 24 |
| rpi-c hearing rpi-d | 27 to 50 of 50, typically 33 to 38 | 24 |

The two receive rows were taken with a harness that started its two
nodes one after the other, so they mix link loss with the prefix of
the peer's burst the later node never listened for (`DW1000.md`, "The
bench, for scale"; settled 2026-09-18). With the nodes started
together the scattered loss is under one frame in fifty on either node.

The transmit side is closed. Everything below is about receiving, or
about a claim in `DW1000.md` that rests on one bench and would be
stronger for a second.

## Settled on 2026-09-18

Five entries left this file that day; their answers are in
`DW1000.md`:

- the 2 of 50 on rpi-c, and the link that seemed to favour rpi-d: the
  harness's start skew, not the chip and not the antennas ("The bench,
  for scale");
- the `SYS_STATE` readings against 2.18 §7.2.27 and APS022 §4.3–4.6:
  all five agree, PMSC 3 is `RX_WAIT` ("SYS_STATE (0x19)");
- errata TX-1 inside the reported region: 0 lost in 8440 delayed
  sends, every refusal flagged, `TXPUTE` from 168 to 173.4 µs of lead
  on rpi-b and `HPDWARN` below ("Errata TX-1");
- a reception delivered twice, one in 300, found the same day once the
  traces existed: with the two buffer pointers on one buffer the
  swinging status bits are the chip's live flags, not the buffer's;
  the driver now tells the stale frame apart and 0 of 48 runs
  duplicate against 13 and 11 before ("With the buffer pointers
  aligned").

## 1. One frame in fifty less, by where the receiver enable sits

**Seen.** With the receiver enabled inside the event pass that
carries both a received frame and the host's own send completion,
rpi-d received 47 or fewer of 50 in five runs of seventeen; with the
enable moved to the end of that pass, none of twelve below 48
(2026-09-17). The statistical experiment below was run on 2026-09-18:
288 duplex runs, the placement alternated run by run (a bench knob,
`cfg->rx_enable_early`, under `DW1000_WITH_DEBUG`), every frame's arrival traced so that the
harness's start skew could be taken out (`DW1000.md`, "The bench").
Scattered losses, the skew removed:

| | inside the pass | end of the pass |
| :-- | :-- | :-- |
| rpi-d | 64 of 4557 frames | 28 of 4520 |
| rpi-c | 72 of 4672 | 49 of 4675 |

z = 3.7 on rpi-d, 2.1 on rpi-c: the effect is real, about one frame
in a hundred, and the driver keeps the end-of-pass placement. Not
chance, then; the mechanism is still not established.

**Ruled out.** That the enable itself goes unheard (300 of 300 at
fifty microseconds and beyond, `DW1000.md`); chance.

**Not ruled out.** An enable written *inside* fifty microseconds of
the frame end, which the pass reaches when the completion is already
in the status word it has just read; the Pi cannot place a write
there on purpose.

**Next.** The mechanistic experiment only: a host fast enough to write
`RXENAB` at a chosen offset from `TXFRS`, sweeping 0 to 50 µs, with a
scope on the IRQ line or the redskin firmware in place of the Pi.
Cost: firmware work; not a ruby-dw1000 job.

## 2. The counter gains 2.5 to 3.4 ppm of the lead at a delayed send, and not during its wait

**Seen.** ruby-dw1000, `measurements/delayed-send/README.md` and
`measurements/errata-tx1/README.md`: the clock-rate excess while a
delayed send waits, 2.57 to 3.01 ppm with the TX clock forced on
against 2.58 to 3.35 ppm without, two runs each.

**Ruled out.** The TX-1 workaround as cause or cure: the excess is the
same with and without it. The wait itself, on 2026-09-18: with the
delayed send armed and taken back by `TRXOFF` a millisecond before its
instant (`sender.rb`, `CANCEL=1`, 150 cycles at 2, 4, 8 and 16 ms,
rpi-b to rpi-d), the counter gains 0.3 ns whatever the lead, against
5.4, 11.0, 22.7 and 46.1 ns when the same sends fire (2.7 to 2.9 ppm,
the same day, same pair). So the counter does not run fast while the
send is pending; the excess comes with the launch, and every later
timestamp carries it. The delayed frame's own carrier is tracked by
the receiver 3.0 ppm off the immediate frames' in the same run.

**Candidates.** Something the chip does at the programmed instant of a
delayed send, and not during the wait: the manual describes only the
8 ns grid of `DX_TIME`.

**Next.** Two leads, one wait: arm at 16 ms, cancel, then a real
delayed send at 2 ms in the same cycle, to see whether the excess is a
fixed cost of a launch or scales with the lead of the send that fires
(the table above scales with the lead, and the cancelled control says
the wait is not where it accrues; the two together want a launch whose
cost depends on how long it waited). Cost: an hour on the two nodes.
The temperature half of the old plan is done by the harness as it
stands: every cycle logs the SAR reading, and in the pending run the
sender warmed by about four steps over the 150 cycles.

## 3. Antenna delays are owed a re-calibration on channel 5

**Seen.** The channel 5 analogue values were changed to 2.18's on
2026-09-16, and the measured SDS-TWR distance moved by 4.6 cm with
them; `TC_PGDELAY` sets the pulse width, and pulse width sets the
effective antenna delay.

**Not done.** Re-calibrating `tx_antenna_delay` and `rx_antenna_delay`
on every node against the new values. Every distance measured on
channel 5 since then carries a bias of roughly 4 cm, and measurements
taken across the change are two configurations.

**Next.** APS014's procedure on each node, at the reference distance
the previous calibration used, and the values recorded with the
`RF_TXCTRL`/`TC_PGDELAY` pair they belong to. Cost: a bench session
with a tape measure; blocks nothing in the driver, but blocks any
distance being quoted.

## 4. Which receive mode carries the double-buffer distance bias

**Seen.** With the responder double buffered, switching the initiator
from double to single buffered moves the asymmetric estimator by
+23 mm at 9.7 sigma, consistent across four rounds. The 6 to 10 cm
figure remembered from earlier was not reproduced.

**Blocked.** The responder cannot be varied: single buffered, it is
deaf during read-out and resolves no exchange at all (0 of 240). So
the both-ends-single comparison the earlier figure presumably came
from cannot be taken with the probe as it stands.

**Next.** Either accept the 2.3 cm as the initiator's share and stop,
or give the probe a responder mode with a longer slot before the
REPORT so a single-buffered responder can hear it, which changes the
exchange timing and so the thing being measured. The first is
recommended; the entry stays open only because the earlier figure has
not been accounted for.

## 5. RXAUTR's re-enable latency

**Seen.** With auto re-enable set, a send's `TXSTRT` found the receiver
back up 14 times in the lost sends, and the losses vanish with the bit
clear. The mechanism written in `DW1000.md` (the `TRXOFF` cuts a
reception, the error re-enables the receiver, the start lands in it)
is inferred; the latency of the re-enable is not measured.

**Why it matters little.** Every sender now keeps the bit clear, so
nothing depends on the figure. It is here because the inference is
the one place in `DW1000.md` where a mechanism is stated without a
direct reading.

**Next.** Only if a host ever needs the bit set while sending: read
`SYS_STATE` at a fixed short delay after a `TRXOFF` that cuts a
reception in progress, sweeping the delay, and record the first
offset at which it reads RX. Cost: an hour on the two nodes.

## Housekeeping, not investigation

- The shelf in `docs/` is behind Qorvo's current revisions for seven
  application notes (README, "The vendor documents"). Only APS022 and
  the manual are cited by section, and both are current, so nothing
  cited is stale; the newer files are in the session's scratchpad if
  wanted.
- Every entry above that needs bench time needs the bench free of
  ruby-ftmbc and of other sessions' probes, which has bitten once
  already.
