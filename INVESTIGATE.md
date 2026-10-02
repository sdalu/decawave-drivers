# What is still to be investigated

The open questions about the chip and the bench, as of 2026-09-18:
what was seen, what has been ruled out, what could explain it, and the
experiment that would settle it. `DW1000.md` holds what is
established; `AUDIT.md` what was checked against the manual. This
file holds what neither can claim yet, so that the next session
starts from the last one's evidence rather than from the symptom.

An entry leaves this file when its experiment has been run, or when
evidence already collected settles it, whichever way it went: a
settled cause goes to `DW1000.md`, a refuted one stays there under
"Unexplained" with the refutation attached.

## The bench's own numbers, for reading the rest

Two nodes, rpi-c and rpi-d, DWM1000 modules on Raspberry Pis, 6.8
Mbps, 128-symbol preamble, PRF 64 MHz, channel 5, double buffered,
receiver auto re-enable off. Sending at each other every 30 ms, 50
frames a run each way, twelve runs a soak. Where things stand after
the driver as of `10d1882`:

| | Result | Runs |
| :-- | :-- | :-- |
| Sends written and never completed | 0 in 440 000 frames | six soaks |
| rpi-d hearing rpi-c | 47 to 50 of 50, mostly 49 or 50 | 24 |
| rpi-c hearing rpi-d | 27 to 50 of 50, typically 33 to 38 | 24 |

The two receive rows were taken with a harness that started its two
nodes one after the other, so they mix link loss with the prefix of
the peer's burst the later node never listened for (`DW1000.md`, "The
bench, for scale"; settled 2026-09-18). With the nodes started
together, and their bursts kept half a gap apart on an absolute
schedule, the same soak loses nothing at all.

The transmit side is closed. Everything below is about receiving, or
about a claim in `DW1000.md` that rests on one bench and would be
stronger for a second.

## Settled on 2026-09-18

Six entries left this file that day; their answers are in
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
  aligned");
- the frame in a hundred lost by the receiver enable sitting inside
  the event pass, entry 1 of this file: not the chip's enable timing
  but the pre-`273fb9d` stale pass's `HRBPT` toggle under a receiver
  the early enable had already brought up, which skipped the
  buffer-pointer sync the end-of-pass enable runs first; the driver
  keeps the end-of-pass placement and keeps `cfg->rx_enable_early` as
  a `DW1000_WITH_DEBUG` bench knob, and the firmware experiment that
  was owed is not needed ("A buffer toggle that moves the host off the
  chip's buffer under a live receiver"; settled by trace analysis over
  the kept duplex logs).

## Settled on 2026-09-19

One entry left this file that day; its answer is in `DW1000.md`:

- the reception delivered twice with the buffer pointers apart, entry
  6 of this file, opened the same day: a `TRXOFF` issued between a
  frame's `RXFCG` and its `LDEDONE` cuts the LDE run, so the frame is
  reported with an earlier frame's `RX_TIME` and `ICRBP` never moves,
  and the unconditional `HRBPT` after `rx_ok` then lands the host on
  the buffer the chip still owns, where the next pass reads a standing
  latch as a frame; 5 of 4482 deliveries carried `RXFCG` without
  `LDEDONE`, every one of them beside this node's own transmit bits,
  and four of the five carried the stamp of an earlier delivery into
  the same buffer. The driver reports such a frame through `rx_error`,
  with `RXFCG` set and `LDEDONE` clear in the word it hands over, offers
  no payload for it and toggles nothing: no consumer surveyed on
  2026-09-19 tested the bit in its `rx_ok`, and in SPANK the status word
  never reaches the place the timestamp is read, so a frame handed over
  as a good one was a wrong distance or clock offset in every ranging
  host, silently (`PROPAGATE.md`, "The frame a node's own send cut is
  now a receive error"). It waits for the bit nowhere: two
  further soaks of the same day spent 46 waits of up to 1 ms on exactly
  these words and not one ever saw it come up, the frame's `RXFCG`
  being posted only after the `TRXOFF` that cut its run ("A TRXOFF
  between RXFCG and LDEDONE leaves the frame without its timestamp, and
  the IC pointer where it was"; three instrumented duplex soaks, and a
  reconstruction that pointed at the intermediate pass).

## Settled on 2026-10-01

One entry left this file that day; its answer is in `DW1000.md`:

- the counter's gain at a delayed send, entry 2 of this file: not the
  send at all, neither its wait nor its launch, but the sender's
  receiver. The clock runs 2.0 to 2.6 ppm faster with the receiver off
  than on on rpi-b, 0.55 to 0.68 ppm on rpi-c and 0.43 to 0.69 ppm on
  rpi-d; an immediate send after
  16 ms of idle gains what a 16 ms delayed send gains, and with the
  receiver never on neither gains more than 0.05 ppm. The 2026-09-18
  control that put the gain at the launch compared fired sends made
  through ruby-dw1000's engine, which keeps the receiver on between
  frames, with cancelled ones made by hand, which never turned it on,
  and is withdrawn ("The clock runs at one rate with the receiver on
  and another with it off"; ruby-dw1000 `measurements/delayed-send/`,
  `MANUAL=1`, twenty-seven runs over the three chips and two radio
  configurations, repeated the next day, and confirmed in SDS-TWR to
  within 3 % of the bias it predicts). What the chip does to tie
  the two is entry 7.

The entries below keep the numbers they were given, so that the
references to them elsewhere in the tree still point where they did;
the numbering starts at 3 because entries 1 and 2 are settled above,
and skips 6, opened on 2026-09-19 and settled the same day.

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

## 7. Why the clock rate follows the radio's state

**Seen.** 2026-10-01, all three chips (`DW1000.md`, "The clock runs at
one rate with the receiver on and another with it off"): an interval
the sender spends with its receiver off, idle or with a send armed,
gains 2.0 to 2.6 ppm on rpi-b, 0.55 to 0.68 ppm on rpi-c and 0.43 to
0.69 ppm on rpi-d against frames sent straight after the receiver;
with the receiver never on, 0.02 to 0.10 ppm. The step is complete
within 0.2 ms of the `TRXOFF` and undone within 0.23 ms of the enable;
it is the same at PRF 16 MHz with a 1024-symbol preamble, and single
buffered; two receivers timestamping the sender at once agree to
0.25 ns, receiving a third node's traffic shrinks it by 7 to 9 %, and
it repeats on 2026-10-02 to within 3 %. In SDS-TWR it shortens a
delayed reply's distance by 52.3 cm at 3 ms and 186.5 cm at 10 ms,
against 53.5 and 184.9 cm predicted. A delayed send's preamble
airtime seems to count with the receiver rather than with idle, so the
two rates may be the radio active and the radio idle (inferred, one
run per configuration).

**Ruled out.** The delayed send itself; the ranging bit; SPI traffic
during the interval; the driver's `dw1000_tx_send()` against the bare
`TXSTRT`; the TX-1 workaround (`TXCLKS` forced on or not, ruby-dw1000
`measurements/errata-tx1/README.md`); the chip's self-heating, the step
being complete in 0.2 ms; the partner, rpi-d's step being the same
heard by rpi-b or by rpi-c, and two observers of one sender agreeing;
the radio configuration; the receive buffering; the chip's digital
clock gating, `RXCLKS`, `SYSCLKS` or both forced to the PLL for a
whole run changing nothing (`CLKFORCE=`, through the debug build's new
`reg_write8`); the RF synthesiser and the regulators being switched
on, `RF_CONF`'s `PLLFEN` and `LDOFEN` forced for a whole run changing
nothing either (`RFFORCE=`, 2026-10-02). The receiving chip's carrier
tracking was read for a while as evidence about the oscillator; it
moves with the radio settings (1.9 ppm on rpi-b by default, 0.2 at PRF
16 MHz, and of either sign from chip to chip), so it is not evidence
either way.

**Candidates.** The supply: receiving and transmitting both draw an
order of magnitude more than idle, and a crystal or its oscillator
pulled by the module's supply or by the current through it would
follow the radio's state with a fast time constant; rpi-b, whose step
is four times the others', reads 2.59 V on its SAR against 2.9 V on
rpi-d (the temperature run of ruby-dw1000's
`measurements/delayed-send/README.md`). Or the receive and transmit
signal chains themselves, coupling into the reference some other way.
The digital clock selects, the synthesiser and the regulators are out
(above), so a setting would have to be one no documented register
forces, and the chains are the one part of the radio left that runs in
both active states and not in idle: their current through the supply
remains the simplest account. The SAR cannot separate them: read with
the receiver running it returns values about 565 steps off in supply
and 200 in temperature, which is not a reading of either.

**Next.** Both need hands on the bench. The decisive one: a frequency
counter on rpi-b's 38.4 MHz oscillator, referenced to GPS or rubidium,
picked up with a near-field loop, across receiver on/off edges; a
2.5 ppm step is about 96 Hz there. A crystal that steps is the
oscillator moving, the supply the likely cause; one that does not puts
the step inside the chip, after the crystal. Then the supply itself: a
scope on rpi-b's module VDD across the same edges, and the
`RXON=1 IDLEOFF=1` run repeated with the module fed from a bench supply
at 3.3 V, where a step that shrinks towards rpi-c's size is the supply.
Little is left to try from the host: of the documented forces only
`RF_CONF`'s `TXFEN` has not been tried, deliberately, since the
transmit chain forced on outside a frame could radiate into the
measurement. The sessions of mid-September that put rpi-b at 3.1 to 3.6 ppm
are the one figure the two days of October do not repeat; they went
through the engine on another build and may have carried that day's
temperature or supply, which the supply test would bear on.

## Housekeeping, not investigation

- The shelf in `docs/` is behind Qorvo's current revisions for seven
  application notes (README, "The vendor documents"). Only APS022 and
  the manual are cited by section, and both are current, so nothing
  cited is stale. The newer files were fetched once, into a session's
  scratchpad that is gone; they are to be fetched again if wanted.
- Every entry above that needs bench time needs the bench free of
  ruby-ftmbc and of other sessions' probes, which has bitten once
  already.
