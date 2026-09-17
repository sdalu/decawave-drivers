# Inside the driver core

How `hw/drivers/dw1000` is put together, and why it behaves the way it
does where that is a choice rather than the manual's instruction. For
using the driver, read [`../../../README.md`](../../../README.md); for
the traps an application falls into, [`README.md`](README.md); for the
repository-wide shape (the port contract, the build, the checks),
[`../../../DESIGN.md`](../../../DESIGN.md).

The audit against the User Manual and the Errata, which is where every
register value was checked and where the open questions live, is
[`../../../AUDIT.md`](../../../AUDIT.md). This file does not repeat it.

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
`_dw1000_radio_is_valid()` runs inside `dw1000_configure()` in every
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

The consequence for a host is the completion obligation the guide
states. The consequence for the driver is that IDLE is a documented
precondition rather than something it enforces by writing `TRXOFF`
before every `TXSTRT`, which is what the vendor does. Enforcing it would
mean a `SYS_STATE` read on every send, or throwing away status the host
may still want.

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

`SYS_STATE` (0x19) is a fourth, smaller case: mapped, never read. The
IDLE preconditions the guide states in prose could be checked against it
instead of asserted, which would turn a documented contract into an
enforced one at the cost of an SPI read per send.
