# Inside the driver core

How `hw/drivers/dw1000` is put together, and why it behaves the way it
does where that is a choice rather than the manual's instruction. For
using the driver, read [`../../../README.md`](../../../README.md); for
the traps an application falls into, [`README.md`](README.md); for the
repository-wide shape (the port contract, the build, the checks),
[`../../../DESIGN.md`](../../../DESIGN.md).

The audit against the User Manual and the Errata, which is where every
register value was checked and where the open questions live, is
[`../../../AUDIT.md`](../../../AUDIT.md), and the chip's established
behaviour, graded by source, is [`../../../DW1000.md`](../../../DW1000.md).
This file repeats neither.

## The four sources

| Source              | Holds                                        |
| ------------------- | -------------------------------------------- |
| `dw1000.c`          | registers, tables, bring-up, receive, events |
| `dw1000_send.c`     | the transmit entry points and their checks   |
| `dw1000_validate.c` | human radio values to encoded fields         |
| `dw1000_state.c`    | reading the radio back off the chip          |

`dw1000.c` never calls into `dw1000_send.c`, which is what lets a
receive-only application leave the latter out. `dw1000_state.c` is not
part of `DW1000_SOURCES` at all: reading the configuration back is
diagnostic, and it is the only source here that formats strings, so an
image that does not want them does not carry them.

`dw1000_validate.c` *is* part of `DW1000_SOURCES`, and the reason it is
a separate list anyway is that a receive-only consumer takes
`DW1000_SOURCES_CORE` and has to name it. It validates one field at a
time and deliberately does not check combinations: `dw1000_configure()`
already refuses an inconsistent radio, and re-stating those rules in a
caller is how two copies come to disagree. That is not hypothetical:
the sniffer this file was lifted out of had a hand-written check that
accepted preamble codes the driver rejects and rejected ones it accepts.

Inside `dw1000.c` the layout is conventional: local definitions, then
the tables, then the static helpers, then the exported functions. The
`_dw1000_` prefix marks what is internal. A handful of `dw1000_rx_*`
functions live among the helpers because the overrun recovery needs
them; only those declared in a header are API.

## The tables are the expensive part

Most of `dw1000.c` above the function bodies is tuning tables indexed by
channel, PRF, bitrate or PAC size, each carrying the User Manual table
number it was transcribed from. They are the reason the core is worth
sharing across hosts, and the reason every value in them is cited: a
disagreement between the code and the chip is then settled by reading
the manual rather than by experiment.

Two properties follow, and both are load-bearing:

**An out-of-range radio value would be an out-of-range read.** Because
the configuration indexes tables, a bad channel number is not merely a
wrong setting; it is a read past the end of an array. So
`dw1000_radio_is_valid()` runs inside `dw1000_configure()` in every
build, not under an assert, and `dw1000_configure()` returns `-1`
without touching the chip. This is the one validation the driver cannot
delegate to the caller.

**Channel 5 carries the current manual's values, not 2.12's.**
`RF_TXCTRL` and `TC_PGDELAY` changed between manual revisions and the
vendor driver kept the old ones. Taking the new values obliges an
antenna-delay re-calibration for anyone who calibrated against the old
ones, which is recorded in AUDIT.md rather than here.

## Bring-up, and the order that matters

`dw1000_initialise()` follows UM table 4. The steps whose order is not
obvious:

- SPI runs below 3 MHz until the PLL is up, then fast. The port supplies
  both speeds; the driver switches between them.
- The LDE microcode load needs the clocks forced, and `LDECLK` must be
  released afterwards (table 4 step L-3). Leaving it forced is one of
  the vendor's defects listed in AUDIT.md.
- The OTP read comes early, while the clocks still run at the crystal
  rate, which is what makes it safe: it yields the chip and lot IDs, the
  reference voltage and temperature, the crystal trim and the LDO tune.
  The antenna delays are not in it: those come from the
  configuration, written near the end of the sequence.
- The crystal trim takes the OTP value first, a configuration override
  second, and `0x10` only as a fallback.
- `DIS_STXP` is set here, turning smart transmit power off, because it
  perturbs the ranging bias correction.

`dw1000_initialise()` returns `-1` and leaves the chip untouched when
the device ID is not a DW1000, and it refuses `cfg->dblbuff` with no
`rx_error` callback before it issues any SPI traffic at all. Rejecting
early is deliberate: a half-configured radio that reports success is the
failure mode worth spending a check on.

The interrupt mask is then derived from the callbacks the configuration
carries, one bit per callback, plus `MRXOVRR` when `dblbuff` is set.
That last one is conditioned on the flag rather than on a callback,
which is the single exception in that block, and it exists because
`RXOVRR` is absent from `ALL_RX_ERR`: a double-buffered host that does
not unmask it stays in the errored state of UM §4.3.5 instead of
recovering.

## `dw1000_process_events()` is the only place events become callbacks

It reads `SYS_STATUS`, dispatches, and clears. Three rules govern it.

**It clears the status bits it reports on, and no others.** A bit it
does not handle stays set, the IRQ line stays asserted, and the host
spins. That is why the guide tells an application not to unmask bits the
function does not handle, and it is a property of the masked-clear
approach rather than an oversight: clearing the whole word would drop
events the host has not been told about.

**It returns whether anything was handled, not whether the status word
was non-zero.** An overrun is handled, reported through `rx_error`, and
stripped from the status, and the call still returns `true`, so a host
driving its interrupt acknowledgement off the return value gets the
overrun case right.

**It performs SPI transfers, so it cannot run in an interrupt handler.**
The IRQ line only says there is something to do; the work happens on a
thread or in a loop. `dw1000_pending_interrupt()` exists so a host can
poll the line instead, which is also the Errata IRQ-1 workaround.

The good-frame path is where the double-buffer difference lives. Single
buffered, the receiver is off when `rx_ok` runs. Double buffered, the
function writes `RXENAB` *before* calling `rx_ok`, so the next frame
lands in the other buffer while this one is read out, and toggles the
host-side buffer pointer as soon as the callback returns.

That ordering is a policy choice, and the vendor made the opposite one:
it reads the frame, the timestamps and the diagnostics into its own
buffers inside the ISR, toggles `HRBPT`, then calls the user. Ours calls
the user first and expects the callback to read the frame. Both are
internally consistent; ours avoids a copy and a fixed-size buffer inside
the driver, at the cost of the two obligations the guide states.

Two naming rules meet here, and the boundary between them is the
register map. **Register and field names are the manual's**, spelled as
it spells them and never adjusted to fit anything here: the bit that
selects this mode is `DIS_DRXB`, the pointers are `HSRBP` and `ICRBP`,
the toggle is `HRBPT`, and the overrun is `RXOVRR`. **Everything the
driver names itself uses one spelling**, `dblbuff`, with two `f`s,
after the `dw1000_config_t` field: the flag
(`DW1000_RX_NO_DBLBUFF_SYNC`), the internals
(`_dw1000_rx_sync_dblbuff()`), the emulation model's own field and
helpers, and the test (`tests/emulation/dblbuff.c`). So `DBLBUFF`
appears in `dw1000_reg.h` exactly once, in
`DW1000_MSK_SYS_STATUS_ALL_DBLBUFF`, which is this driver's aggregate
over two manual-named bits (`RXDFR | RXFCG`) and takes the house
spelling because the manual names no such group.

### The flags that are not the buffer's

With `HSRBP == ICRBP` the swinging status bits read as the chip's
record of its last reception, not as the host's buffer's latch: no
status write clears them, the next receiver enable does, and the
receive registers meanwhile select the host's buffer (`DW1000.md`,
"With the buffer pointers aligned"). A frame read out with the
receiver off is followed by exactly that state, and the pass after it
would report the previous frame again. `dw1000_process_events()`
therefore takes `RXFCG` with the pointers aligned and none of
`RXPRD|RXSFDD|RXPHD` set as stale: it strips the bits from the word it
works on, reports nothing, toggles nothing, and lets the enable that
ends the pass reset them. Measured, not read (24 of 24 duplicates over
96 runs had that shape, and 0 of 48 runs duplicate with the check), and
since reproduced in `port/emulation`, which shows the chip's flags with
the pointers aligned: `tests/emulation/dblbuff.c`,
`step_stale_frame_told_apart`, where the pass over a send on the air
reads the frame out, toggles, and the completion's pass finds that word
and reports nothing.

### The frame whose LDE run was cut

A pass can find `RXFCG` set with `LDEDONE` clear and a detect bit
standing: the driver's own send issues a `TRXOFF` through
`dw1000_txrx_idle()`, and a host issues one of its own through
`dw1000_txrx_off()`. Either terminates a reception whose payload and
CRC are already in and whose leading edge run has not run, and the chip
posts that frame's `RXFCG` afterwards all the same, without `RX_TIME`
and without moving `ICRBP` (`DW1000.md`, "A TRXOFF between RXFCG and
LDEDONE leaves the frame without its timestamp, and the IC pointer
where it was", measured 2026-09-19, 5 of 4482, 11 of 4415 and 13 of
4507 deliveries over three duplex soaks).

Nothing is waited for, and the measurement is why. Two soaks of the
same day ran a driver that polled for `LDEDONE` on exactly these words,
before the `TRXOFF` of its own send and again in the pass: 46 waits of
up to 1 ms of the chip's own clock, every one of them run to the bound,
0 ended by the bit. The wait before the `TRXOFF` never fired at all,
the frame's `RXFCG` not being posted yet when the send path read the
status. So `dw1000_process_events()` reads the word once, and
`_dw1000_rx_lde_pending()` on it is the whole of the decision: the
detect bits are what tell such a frame from the chip's standing record
of an earlier reception, which carries `RXFCG` without `LDEDONE` too
and is stripped a few lines below.

Such a frame is reported through `rx_error`, and no payload is offered
for it. The reason is a survey of every consumer of this driver on
2026-09-19 (`PROPAGATE.md`, "The frame a node's own send cut is now a
receive error"): not one of them tests `LDEDONE` in its `rx_ok`, and in
SPANK the status word never reaches `spank_io_get_from_driver()`
(`spank/src/io.c:233`), where the timestamp is read, so the bit could
not be tested there even by a host that wanted to. What the frame
carries is the `RX_TIME` of an earlier reception into the same buffer,
and a distance or a clock offset computed from it is plausible and
wrong, in every ranging host, silently. Reported as an error it costs
those hosts nothing: every one of them re-arms from `rx_error` and
counts it, and a host that wants these apart tests `RXFCG` set with
`LDEDONE` clear in the word `rx_error` is handed, which no other error
carries.

The route is one strip and one condition. With `_dw1000_rx_lde_pending()`
true the pass takes `RXFCG`, `RXDFR` and `LDEDONE` out of the word it
works on, so neither the stale strip below it nor the `RXFCG` branch
runs for it, keeps the word it was entered with for the callback, and
counts the frame as processed; the error section's condition is that
strip's flag or `ALL_RX_ERR`, and its two arms do for the cut frame
what they do for an `RXFCE`. The bits the chip still shows go with
`rx_drop`, which carries `ALL_RX_GOOD` here because no good frame was
reported in the pass. Two things that happen for an ordinary frame
therefore do not happen at all: no early `RXENAB` over a buffer the
chip still owns, and no `HRBPT`, a toggle with `ICRBP` where it was
being exactly the move that lands the host off the chip's buffer and
delivers the next pass a standing latch as a frame (`DW1000.md`, "A
buffer toggle that moves the host off the chip's buffer under a live
receiver"). Single buffered there is nothing to differ: the frame is
reported the same way, through `rx_error`.

What the chip is left showing is the `RXFCG` and `RXDFR` of that
reception with the two pointers aligned, which no status write clears
and the next receiver enable resets (`DW1000.md`, "With the buffer
pointers aligned, the swinging status bits are the chip's flags, not
the buffer's"). That is the stale record the strip below discards, so a
pass run before any enable reports nothing for it.

The emulation step is `tests/emulation/dblbuff.c`,
`step_lde_cut_frame_is_an_error`: a frame delivered with the model's
LDE run armed long, `dw1000_txrx_idle()` inside the run, and one pass,
which must call `rx_error` once with `RXFCG` set and `LDEDONE` clear,
call `rx_ok` not at all, leave both buffer pointers where they were,
and report the next frame normally with its own stamp.

### The registers that do not swing

UM table 7 lists what the double buffer duplicates. `DRX_RXPACC_NOSAT`
and `LDE_THRESH` are absent from it, so there is one live instance of
each and the next frame's LDE run overwrites them. The driver therefore
samples both for the frame it is about to report, and the accessors that
need them read the sample when `dblbuff` is set and the register when it
is not. `_dw1000_rx_get_pacc_count()` is the accessor for the first. It
is `static inline` in `dw1000.c` and tagged `@internal`, like every
`_dw1000_` helper here, so an application cannot reach it; what it
reports arrives through `dw1000_rx_get_power_estimate()`.

### What the error and timeout branches drop

Both branches drop one mask, `rx_drop`, built once above them:
`ALL_RX_ERR` and `ALL_RX_TO` always, and the good-frame bits
(`ALL_RX_GOOD`) only when no good frame was reported in the same pass.
A frame whose LDE run was cut is on the other side of that condition
rather than an exception to it: its `RXFCG` was stripped from the word
before either branch, no branch reported it as a frame, and the drop is
what takes its bits away (the section above).
The `TRXOFF` and the UM 4.1.6 receiver reset are unchanged by that
condition: they still happen unless a transmission is in flight, in
which case the status is dropped without the `TRXOFF` and the reset is
owed to `dw1000_rx_start()`. The reset after an error is a requirement
of the manual, not a matter of what the pass saw.

Once the reset is applied, and before the callback, both branches run
`_dw1000_rx_reconcile()`, the same helper that closes the pass: the end
of the pass is only a few register accesses later, but every microsecond
the receiver is down after an error is a preamble missed, and the sync
inside the start keeps whatever was completed into the other buffer
meanwhile. Not over a send in flight, which leaves the state in TX or
TX_W4R and the reconcile with nothing to do; that send's completion
brings the receiver back in its own pass. A host without the policy
still finds the receiver down in `rx_error` and re-arms it there, as
before, with one exception: a `dw1000_rx_start()` recorded over a send
whose completion is in this same status word is honoured here rather
than at the end of the pass, so the enable is written before the
callback instead of after it. The receiver ends the pass in the same
place either way.

The condition is the double buffer's. When the `RXFCG` branch of the
same pass has run it has already cleared the good-frame bits,
re-enabled the receiver and toggled `HRBPT`, so the swinging bits read
in the error branch are the *next* buffer's, and a frame that landed
there while `rx_ok` ran carries an `RXFCG` no branch has reported.
Written over, it is gone: `_dw1000_txrx_off()`'s own
`_dw1000_rx_sync_dblbuff()`, whose guard is exactly a frame the host
has not been told about, then reads the latch the clear has just
removed, issues `HRBPT` and hands that buffer back to the chip unread.
One frame lost with no callback and no counter.

Reaching it takes `RXAUTR` set. With the bit clear the receiver is idle
after a good frame and after an error alike, so no error can stand
beside an `RXFCG` in one snapshot; the bit is kept clear for senders
and set by the raw receiving roles, which is why 43 000 traced duplex
passes never showed the case. The reproduction is deterministic
in the emulation instead: `tests/emulation/dblbuff.c`,
`step_error_beside_a_good_frame`, delivers a good frame, then one
failed in its PHY header through `dw1000_emulation_fail_next_frame()`,
then a third from inside `rx_ok`, and asks that the third still be
reported. A second step, `step_receiver_back_before_rx_error`, runs
the same word with the policy on and asks that `rx_error` already
finds the receiver listening (`PMSC_STATE` 5) with a fourth frame
complete, and that the two passes after it report the third and the
fourth in order.

The rule both of these obey, and the one the receiver-enable placement
was measuring without knowing it (`DW1000.md`, "A buffer toggle that
moves the host off the chip's buffer under a live receiver"): never
toggle `HRBPT` off the chip's buffer with the receiver enabled, and let
every `RXENAB` after a toggle go through `_dw1000_rx_sync_dblbuff()`.
The end-of-pass enable is the driver's because it does; an enable
written before the toggle skips it, and the pass that did that lost the
next frame unread and the one after to an overrun.

### Overrun recovery

A third frame arriving while both buffers are held is an overrun.
`_dw1000_rx_overrun_recover()` turns the transceiver off, resets the
receiver, issues `HRBPT` unconditionally, re-aligns the pointers, and
then lets the caller report `rx_error`.

`HRBPT` is issued unconditionally rather than through the ordinary
conditional toggle because on overrun the IC has wrapped back onto the
buffer the host still holds (§4.3.5), so `ICRBP == HSRBP` and the
conditional toggle would issue nothing, and `HRBPT` is the only thing
that clears `RXOVRR`. `RXOVRR` is never written: §7.2.17 makes it read
only, and the vendor writing 1 to it is another of the defects AUDIT.md
lists.

Re-arming is left to the callback, as it is for every other error. The
driver could re-arm here, and does not, because that would hide the
frames an overrun means were lost. The price is that `dblbuff` without
an `rx_error` callback is unusable, which is why it is refused outright.

## Transmit: one frame at a time

`dw1000_send.c` keeps a flag saying whether a transmission is in flight
and refuses a send while one is, with `DW1000_TX_ERR_BUSY`, before
anything is written to the chip. The reason is Errata TX-2: writing the
transmit buffer while a transmission is in progress corrupts what is on
the air, and that write happens before any later check could stop it.
The flag costs no SPI: the driver already maintains it.

The flag is a shadow, and a stale one used to refuse: a completion left
standing, or a send the chip dropped. `_dw1000_tx_idle()` now asks the
chip before refusing, once: a completion standing there is consumed by
the new send, a send with no flag goes through `_dw1000_tx_dropped()`,
and only a frame on the air refuses. The consequence for the driver is
that IDLE is enforced rather than documented, the way the vendor does
it, but with the receiver's events kept and only when the receiver may
be up (the state says so, or `rxauto` is set, which re-enables it
unseen): a
`TRXOFF` a host already issued is not issued again, and no `SYS_STATE`
is read for a send that is where it should be. `dw1000_tx_start()`, the
raw primitive, keeps the precondition.

The failure codes are distinguished because the right response differs,
and the driver uses its own distinction: the extended send retries only
`DW1000_TX_ERR_TOO_LATE`, that being the one more lead time can cure.
The retry lead is scaled from the delay actually passed, not from the
configured default, so a long send is never retried with less lead than
the attempt the chip just refused.

Errata TX-1 is handled inside the delayed-send path:
`_dw1000_tx_clock_force()` forces the TX clock on before `TXDLYS|TXSTRT`
and `_dw1000_tx_clock_release()` releases it on completion. Without it a
send whose programmed time falls just after the TXPUTE window is dropped
with no flag raised and no TX-done event, and the driver reports success
for a frame that never left.

Errata RX-1 is handled by refusing a transmit that would write past TX
index 127 while a received frame is held. It is unreachable without
proprietary long frames, and `dw1000_tx_write_frame_data()` does not
enforce it, because it returns void and cannot know a send will follow.

## One state, one table

The transceiver is one field, `dw->state`, in place of the two flags
the driver kept until 1.4 (`tx_pending`, `wait4resp`), and every public
operation and every event branch is a cell of the table below.
`rx_held` (a frame being read out of the double buffer),
`rx_reset_due` (a receiver reset owed, UM 4.1.6) and `rx_want` (what the
host has asked of the receiver: NONE, ONCE for a start that came over a
send, KEEP for the standing ask of `rx_keep_on`) ride alongside; none of
them says what the transceiver is doing.

| Event or call            | IDLE                | RX / RX_W4R                    | TX / TX_W4R                            |
| :----------------------- | :------------------ | :----------------------------- | :------------------------------------- |
| `dw1000_tx_send()`       | start, TX or TX_W4R | TRXOFF keeping events, start   | busy; over or dropped: consume, start  |
| `dw1000_rx_start()`      | RXENAB, RX          | RXENAB again, RX               | recorded (`rx_want`), stay             |
| good frame               | (cannot happen)     | report; RX (dblbuff RXENAB) or IDLE | report, stay; no RXENAB; beside TXFRS: enable at the end of the pass |
| error, timeout, overrun  | (cannot happen)     | TRXOFF, reset, report; IDLE    | drop status, owe reset, report; stay   |
| TXFRS                    | stale, cleared      | (cannot happen)                | report; IDLE, or RX_W4R from TX_W4R    |
| `dw1000_txrx_stop()`     | IDLE                | IDLE                           | abort; IDLE                            |
| end of the pass          | `rx_want` is ONCE or KEEP: RXENAB, RX; ONCE spent | nothing | nothing                    |

A completion is booked into the state before any branch reads it.
`dw1000_process_events()` reads `SYS_STATUS` once, and a `TXFRS` in that
word means the chip left TX before the pass began: the first thing the
pass does, after the dropped-send check, is apply that send's state
transition (`_dw1000_tx_done_state()`, and the suspicion it carried
dropped). What the completion owes the chip and the host stays in the
`TXFRS` branch, in the order it always had: the `TX` status group
cleared, the forced TX clock released, the two hotfixes, the receiver
reset owed under UM 4.1.6, and `tx_done`. What the booking buys is that
"is a send on the air" is a question about the state alone
(`_dw1000_tx_pending()`), asked seven times a pass, instead of a
question about the state *and* the status word that every branch had to
carry around and correct for.

RX_W4R is the receiver the chip put up itself at the end of a send
that expected a response; it is what `dw1000_tx_is_expecting_response()`
answers from inside `tx_done`, and it turns into RX on the first good
frame. "Report, stay" holds for a good frame beside a TX_W4R in single
buffered receive as well: that branch takes the transceiver to IDLE once
the transmitter is off the air, but not over a send that expects a
response, whose RX_W4R the booking above has already produced and which
the branch therefore leaves alone (with the two "RX_W4R to RX" lines of
that branch, which speak for the frame being reported and not for a
receiver put up after it). The chip has its own receiver up by then, so a
state written to IDLE there had `dw1000_tx_is_expecting_response()`
answering false, left an owed receiver reset unapplied, and cost the next
send, whose start went into a listening receiver. A start recorded over a send is honoured
at the end of the completion's pass, whatever the policy, and a host
that tests for `DW1000_RX_ERR_BUSY` never sees it any more.
`tests/emulation/timing.c` runs every one of its steps in both receive
modes, `TIMING_DBLBUFF` selecting the build.

## The receiver policy, and the send the chip never began

Two things the driver keeps for a host that asks. `cfg->rx_keep_on` makes
`dw1000_process_events()` reconcile the receiver from `rx_want` and the
state: IDLE means nothing has the receiver up, and `rx_want` is what the
host has asked for, in one field with three settings rather than the two
flags (`rx_wanted` and `rx_deferred`) it replaces. NONE is nothing
asked; KEEP is the policy's standing ask, which a `dw1000_rx_start()`
sets under `rx_keep_on` and only `dw1000_txrx_off()` ends, and which no
enable spends; ONCE is a single start, recorded because it came over a
send on the air, and spent the moment the receiver is back up. The
fourth combination the two flags could hold, a start recorded with the
policy also asking, was never anything but KEEP, and saying so removes
the pair of writes that had to keep the two flags agreeing. That
reconcile is one function, `_dw1000_rx_reconcile()`, and it is the only
thing in the pass that writes `RXENAB` on the policy's behalf, which is
what makes the order of the branches irrelevant to the receiver; the
same function honours a start that came over a send. It runs at the end
of the pass, after every callback, and once more in each of the timeout
and error branches, where a receiver left down costs preambles (see
above). Neither of the two `RXENAB`s that are not the
policy's goes through it: the good-frame branch's belongs to the double
buffer, and the re-enable the `TXFRS` branch owes a WAIT4RESP receiver
belongs to that receiver.

`_dw1000_tx_dropped()` is the one read of `SYS_STATE` in the driver, and
it happens only where a stale `tx_pending` would otherwise refuse
something: a send with no transmit flag and the transceiver neither in
TX nor TX_WAIT is suspected, the chip's time noted, and called dropped on
a later sighting past its airtime and a millisecond. The two sightings
are what keep the moment after `TXSTRT`, when the chip still reads IDLE,
from being mistaken for a drop, without a clock read on every send. The
airtime is computed at the start from the frame length and the bitrate,
as a bound rather than a timestamp.

## The formulas, and where their constants come from

Four computations in the core are not register shuffling, and each cites
the manual section it implements:

- **Receive power** (`dw1000_rx_get_power_estimate()`, UM §4.7.1/§4.7.2)
  takes a PRF-dependent constant and divides by the preamble
  accumulation count, with the `RXPACC` SFD adjustment applied. The
  count can legitimately be zero, and a failed SPI read produces zero
  too, so both give `-INFINITY` rather than a plausible number.
- **The power correction curve** (`dw1000_rx_power_correction()`)
  applies the published correction to that estimate.
- **Clock drift** (`dw1000_rx_get_clock_drift()`) is `RXTOFS` over the
  interval, with the sign of the §7.2.22 worked example. The vendor's
  has the opposite sign.
- **The antenna calibration table** (`dw1000_get_calibration()`) is a
  setup helper rather than part of the operating path: given a channel
  and a PRF it hands back the reference receiver input power and the
  antenna separation an antenna-delay calibration is run at. Nothing
  inside the driver calls it.

The last three are ours alone; the vendor has none of them. That is
worth knowing when comparing numbers with anything built on the vendor
driver, and it is the reason an instrument sharing this driver cannot
independently check them; see
[`../../../probe/DESIGN.md`](../../../probe/DESIGN.md).

## Where this driver departs from Decawave's

The full function-by-function comparison, with file and line on both
sides, is in AUDIT.md under "Comparison with the Decawave driver". The
register map, every field, bit and mask, and every tuning table are
byte-identical; the divergences fall into three groups.

**Vendor defects, not to be adopted.** Ten of them, from `AGC_TUNE3`
never being written to the crystal trim never taking its OTP value.
AUDIT.md tabulates each against the manual section that settles it.

**Deliberate policy differences**, which this driver documents rather
than changes:

| Question                  | Vendor             | Here               |
| ------------------------- | ------------------ | ------------------ |
| Re-arm after a good frame | inside its ISR     | the callback's job |
| Re-arm after an error     | inside its ISR     | the callback's job |
| Before `TXSTRT`           | writes `TRXOFF`    | IDLE is required   |
| Smart TX power            | left on            | `DIS_STXP` at init |
| Reported frame length     | excludes the FCS   | includes it        |
| Read-out order            | driver reads first | callback reads     |
| Radio validation          | debug only         | every build        |

**What only one side has.** The vendor has the carrier-integrator clock
offset (`DRX_CAR_INT`, not even mapped here), frame-duration helpers,
sleep and wake, and the event counters at 0x2F. This driver has the
delayed-send retry, `SFCST`, the `RXPACC` SFD adjustment, the antenna
calibration table, the receive power correction curve, and the
configuration validation in every build.

## Adding what is missing

Three absences are worth naming with what completing them costs, because
each has pieces already in the tree that look like support and are
inert.

**Sleep and deep sleep.** `dw->sleep_mode` is accumulated and never
written to `AON_WCFG`; `AON_CFG1` is cleared and never uploaded with
`UPL_CFG`; and `TX_ANTD` is not preserved across a sleep (UM §7.2.26).
An entry point means finishing all three, and Errata PMSC-1 (a wake-up
event longer than 500 µs) becomes applicable the moment one exists.

**The carrier-integrator clock offset.** `DRX_CAR_INT` is not in
`dw1000_reg.h`, so this starts with the register map. The driver already
reports a clock drift from `RXTOFS`, which answers a related question by
a different route; whether both are wanted is a design question, not an
implementation one.

**The event counters.** Register 0x2F is unmapped and the `EVC_*` fields
are undefined. They are cheap to add and would give a host a drop count
the driver currently cannot provide.

`SYS_STATE` (0x19) is a fourth, smaller case: mapped, and read in one
place only, the dropped-send diagnosis, when a send is pending with no
transmit flag to show for it. The manual (2.12) leaves that register
file reserved and documents none of its fields; what the driver reads
into it is the bench's measurement: 1 idle, 2 while a delayed send
waits, 4 while a frame goes out, 5 with the receiver enabled.
