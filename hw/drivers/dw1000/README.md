# Writing an application against this driver

Every entry below is something that has actually gone wrong: in this
tree, in an application built on it, or on the bench. Each says what
bites, why, and where the behaviour is prescribed, so it can be checked
against the User Manual rather than taken on trust. Read it before
writing an application, not after.

This is not the API reference. The headers are the contract, and they
are where a detail this file only summarises gets settled. The other
documents: [`../../../README.md`](../../../README.md) for the API, the
build and the options; [`DESIGN.md`](DESIGN.md) for how the core works
inside and why it departs from Decawave's driver;
[`../../../AUDIT.md`](../../../AUDIT.md) for the audit against the User
Manual and the Errata, including what is still open.

## Four facts that shape everything else

**Asserts are not validation.** `DW1000_ASSERT` maps to `assert()`,
`__ASSERT` or `osalDbgAssert`, and in six of the seven ports it vanishes
in a shipping build: under `NDEBUG`, without `CONFIG_ASSERT`, or
without `CH_DBG_ENABLE_ASSERTS`. Only `port/cf2` traps unconditionally.
Anything the driver must reject in a release build is checked separately
and reported through a return value. Never rely on an assert to catch a
bad argument in production.

**The configuration must outlive the driver; the radio settings need
not.** `dw1000_init()` stores the `dw1000_config_t *` you hand it and
reads it for the life of the context, callbacks and SPI handle included,
so it cannot be a local. `dw1000_radio_t` is different: it is copied by
value into the context, so a local struct passed to `dw1000_configure()`
is fine.

**Return values are not decoration.** `dw1000_initialise()`,
`dw1000_configure()`, `dw1000_tx_send()`, the extended sends and
`dw1000_rx_start()` all report failure, and in each case the failure is
one an ordinary mistake provokes. Transmitting after `dw1000_configure()`
returns `-1` is a programming error: the radio keeps whatever
configuration it had, which on a first call is none.

The transmit failures are distinguished further, because what to do
about them differs. All are negative, so `rc < 0` still works:

| Failure                     | What to do                           |
| --------------------------- | ------------------------------------ |
| `DW1000_TX_ERR_TOO_LATE`    | transient; the same call may succeed |
| `DW1000_TX_ERR_BUSY`        | transient; consume the completion    |
| `DW1000_TX_ERR_LEAD`        | a setup to fix; raise the lead       |
| `DW1000_TX_ERR_FRAME_SIZE`  | permanent; the frame is too long     |
| `DW1000_TX_ERR_MODE`        | permanent; the flags disagree        |
| `DW1000_TX_ERR_TIMESTAMP`   | permanent; no room at that offset    |
| `DW1000_TX_ERR_BUFFER_HELD` | permanent while a frame is unread    |

The driver uses the distinction itself: the extended send retries only
`DW1000_TX_ERR_TOO_LATE`, that being the one more lead time can cure.

**The reported frame length includes the FCS, and so does the buffer.**
Two bytes of every length `rx_ok` is given, and of every
`dw1000_rx_get_frame_length()`, are the CRC, and the bytes are really
there: `RX_BUFFER` holds the payload followed by the FCS, little endian,
at offset `length - 2`. Subtract `DW1000_CRC_LENGTH` for the payload.
Decawave's own driver excludes them from the length it reports, so code
ported from it is off by two.

The two directions are opposite conventions, deliberately, and this is
the single easiest thing to get wrong here:

| Direction | What the length means                      |
| :-------- | :----------------------------------------- |
| Send      | payload only; the driver adds the two      |
| Receive   | payload **plus** FCS; you subtract the two |

Receive reports what the chip reports (`RXFLEN`), which is what the rest
of the driver does with every register. Send is the one that is helpful.
A round trip written symmetrically is wrong at one end.

The driver computes no CRC of its own, in either direction. On transmit
it adds `DW1000_CRC_LENGTH` to the length it writes to `TX_FCTRL` and
the chip appends the FCS itself; with `DW1000_TX_NO_AUTO_CRC` it does
not, and the frame you supply must already carry a CRC-16-CCITT. On
receive the chip has already checked the FCS, and a bad one raises
`RXFCE` and never reaches `rx_ok`, so re-checking it is redundant. The
FCS is exposed for a sniffer or a log, not for validation.

## Bring-up and configuration

### Set the default delay after every `dw1000_configure()`

`dw1000_tx_set_default_delay()` must be called after
`dw1000_configure()`, and again after every reconfiguration. The
delayed-send lead includes the preamble and SFD airtime, which is only
known once the radio is configured and which changes with the
preamble length: about 138 µs at 128 symbols and 4.2 ms at 4096. A lead
left over from a shorter preamble is refused with `DW1000_TX_ERR_LEAD`
rather than silently lost.

### Double buffering requires an `rx_error` callback

`dw1000_initialise()` refuses `cfg->dblbuff` when `cfg->cb.rx_error` is
`NULL`, and returns `-1`. The overrun recovery puts the chip back in
order and then reports a receive error; re-arming the receiver is the
host's job, as it is for every other error. With no callback the
recovery would run to completion and nothing would ever enable the
receiver again, silently and permanently. The driver refuses the pairing
rather than re-arm by itself, which would hide the frames an overrun
means were lost.

### The interrupt mask is derived from the callbacks you register

`dw1000_initialise()` unmasks `MTXFRS`, `ALL_RX_TO`, `ALL_RX_ERR` and
`MRXFCG` only for the callbacks that are present, plus `MRXOVRR` when
`dblbuff` is set. If you unmask more with `dw1000_interrupt()`, unmask
only bits `dw1000_process_events()` handles. It clears the status bits
it reports on and no others, so a bit it does not handle stays set, the
IRQ line stays asserted, and the host spins. `CPLOCK`, `GPIOIRQ`,
`TXBERR`, `RFPLL_LL`, `CLKPLL_LL` and any intermediate TX or RX bit
without its terminal bit are all in that category.

### Guard against the version you need

The release is written in one place and is usable from the preprocessor:

```c
#include <dw1000/dw1000.h>          /* or <dw1000/dw1000_version.h> alone */

#if !DW1000_VERSION_AT_LEAST(1, 1, 0)
#error this application needs dw1000 1.1.0 or newer
#endif
```

### Smart transmit power is off

The driver sets `DIS_STXP` at initialisation, where Decawave's leaves
smart power on, because it perturbs the ranging bias correction.
Transmit power is named outright through `DW1000_TX_POWER(dB)` in
`dw1000_radio_t`; start from `manual_tx_power[]` and UM §7.2.31.4 for a
value. There is no board compensation flag, and a board calibration is
not the driver's business.

## Transmitting

### The chip must be IDLE before a send, and which call gets you there matters

| Situation                    | Call                 |
| :--------------------------- | :------------------- |
| Shutting the radio down      | `dw1000_txrx_off()`  |
| Events pending that you want | `dw1000_txrx_idle()` |
| From inside `rx_ok`, dblbuff | `dw1000_txrx_idle()` |

`dw1000_txrx_off()` clears every pending TX and RX event along with
stopping the transceiver; `dw1000_txrx_idle()` stops the transceiver and
leaves the status untouched.

The third row is the one that has bitten. In double buffered mode the
driver re-enables the receiver before calling `rx_ok`, so a responder
answering from its callback is not in IDLE and has to stop the
transceiver first. `dw1000_txrx_off()` there throws away the status of
the frame being read out. The frame itself survives, the driver holding
its buffer against exactly this, but there is no reason to ask for the
loss.

### One frame at a time, and you must consume the completion

A send issued while a previous one has not been reported done is refused
with `DW1000_TX_ERR_BUSY`, and nothing is written to the chip. Errata
1.4 §3.3 (TX-2) is the reason: writing the transmit buffer while a
transmission is in progress corrupts what is on the air, and that write
happens before any later check could stop it.

So every completion has to be consumed, by one of:

| Call                            | When                        |
| :------------------------------ | :-------------------------- |
| `dw1000_process_events()`       | the ordinary path           |
| `dw1000_tx_clear_status_done()` | a host that polls           |
| `dw1000_txrx_idle()` / `_off()` | abandoning the transmission |

A host that does none of these will see every send after its first
refused. That is not a new restriction so much as a newly visible one:
such a host never learned whether its frames left, and at a send
interval below the frame's airtime almost none of them did, with every
call returning success. The check costs no SPI: it is a flag the driver
already maintains. AUDIT.md carries the measurement.

### A delayed-send delay is the whole lead, airtime included

Both `DW1000_TX_DELAYED_DELAY` and the pair held by
`dw1000_tx_set_default_delay()` are the lead to the programmed RMARKER,
used exactly as given, with nothing added. The RMARKER is the *end* of
the SFD (APS022 §5.4), so the chip must already be transmitting before
it: a lead at or below `dw1000_tx_get_preamble_airtime()` cannot be met
and is refused with `DW1000_TX_ERR_LEAD`.

A lead of `0` means "no default set", and every send that carries no
delay of its own is then refused.

Lead figures reported by the estimators in the surrounding tooling are
**totals**, not the host term. Do not add the airtime to them a second
time. Measured across a thirty-fold range of preamble length, the
minimum workable lead tracks the computed airtime with a flat residual
for the host's own register writes and the transmitter power-up: tens
of microseconds, and a property of the host rather than of the radio.

### The retry gets one and a half times the delay, unless you name it

When you pass `DW1000_TX_DELAYED_DELAY` without
`DW1000_TX_DELAYED_RETRY_DELAY`, the retry is scaled from *your* delay,
not from the configured default. A retry must never get less lead than
the attempt the chip just called too late, which is what used to happen:
a 10 ms send retried at a 4 ms default, failing precisely on the slow
hosts that raised the delay. An explicit retry delay is honoured as
given, and a configured retry of `0` disables retries.

### Embedding a transmit timestamp

- It implies `DW1000_TX_DELAYED_START`, whether or not you pass it. The
  value embedded is a programmed time, so sending immediately would put
  a future instant in a frame already gone.
- The timestamp must fit inside the frame. An offset leaving fewer than
  5 bytes (or 8, at `_64BIT`) is refused with `DW1000_TX_ERR_TIMESTAMP`.
- The frame you hand in is **written in place**, at the same offset, so
  it must be writable. A Ruby string or any shared or frozen buffer
  needs a copy first.
- The embedded value equals what `TX_STAMP` reports on completion,
  antenna delay included. Comparing the two is a genuine self-check, and
  it is what lets a receiver learn the transmit instant without a second
  exchange.

### The payload ceiling is the frame ceiling less the CRC

`dw1000_tx_get_frame_maxsize()` gives the largest frame the chip can
carry: 127, or 1023 only when the build has proprietary long frames
*and* the radio is configured for them. With the automatic CRC the
largest payload you may pass is that less `DW1000_CRC_LENGTH`, because
the driver adds the two bytes before the ceiling applies. A longer
payload is refused with `DW1000_TX_ERR_FRAME_SIZE`.

`dw1000_tx_fctrl()` still clamps, for a caller writing `TX_FCTRL` itself
and having already decided what it means. The refusal sits in the send
path instead.

### Two errata worth knowing

**TX-1, handled.** A delayed send whose time falls just after the TXPUTE
window used to be dropped with neither `HPDWARN` nor `TXPUTE` raised and
no TX-done event, so the driver reported success for a frame that never
left. The driver now forces the TX clock on before `TXDLYS|TXSTRT` and
releases it on completion. Nothing for a caller to do.

**RX-1, handled, and unreachable by default anyway.** A TX buffer write
past index 127, followed by a send, while a frame sits unread in the
second receive buffer, corrupts that frame's 129th octet. The send
functions refuse such a transmit while a frame is held, so an ordinary
caller cannot hit it.

Two things worth knowing if you drive the transmitter yourself.
`dw1000_tx_write_frame_data()` does **not** enforce it: it returns void
and cannot know a send will follow, so a caller using it with
`dw1000_tx_fctrl()` and `dw1000_tx_start()` owns the constraint. And it
cannot arise at all without proprietary long frames, since
`dw1000_tx_get_frame_maxsize()` is otherwise 127, which caps the payload
at 125 and the highest index written at 124.

## Receiving

### The driver does not re-arm the receiver for you

After an error or a timeout the receiver is left off. Re-enabling it is
the callback's job, or `RXAUTR`'s if you set `cfg->rxauto`. Decawave's
driver re-enables inside its ISR; this one does not, deliberately, so
that a host decides when to listen again.

### `dw1000_process_events()` returns whether anything was handled

Not whether the status word was non-zero. An overrun is handled,
reported through `rx_error`, and stripped from the status, and the call
still returns `true`. A host driving its interrupt acknowledgement off
that value gets the overrun case right.

### The preamble timeout is not a hard deadline

UM §7.2.40.9: an unconfirmed preamble detection suspends the countdown
by at least one PAC plus 32 symbols. Back
`dw1000_rx_set_timeout_preamble()` with the SFD and frame-wait timeouts
rather than relying on it alone.

### `-INFINITY` means "no estimate", not "very weak"

`dw1000_rx_get_power_estimate()` divides by the preamble accumulation
count, which can legitimately be zero, and which a failed SPI read also
produces. Both outputs are then set to `-INFINITY`: it cannot be
mistaken for a reading, it orders correctly against any threshold, and
it survives `dw1000_rx_power_correction()` unchanged. Returning a
plausible number instead would hand you a believable lie.

## Double buffered receive

Worth reading as a whole before turning `cfg->dblbuff` on.

> [!IMPORTANT]
> For a three-frame exchange, the node that must hear the last frame
> needs this. A single-buffered responder hears the first and the
> middle frame and never the last, with every drop counter at zero: the
> read-out window is simply deaf, and `rxauto` does not cover it. A
> double-buffered one resolves every exchange. AUDIT.md carries the
> measurement.

Which mode carries a ranging bias is not established. Enabling it moves
double-sided distances by a small amount (a couple of centimetres on
the asymmetric estimate), and the obvious experiment is not available,
because the both-ends-single-buffered case does not resolve at all. The
estimators disagree in sign, so the number to expect is small and the
direction is unsettled. See AUDIT.md.

### `rxauto` is a separate bit, and both applications here keep it on

`cfg->dblbuff` and `cfg->rxauto` are independent. `rxauto` becomes
`SYS_CFG`'s `RXAUTR`, written once by `dw1000_initialise()`; `dblbuff`
is what makes `dw1000_process_events()` write `RXENAB` itself in the
good-frame path, before calling `rx_ok`. Both applications in this tree
that turn double buffering on leave `rxauto` set as well
(`probe/app/unix/main.c` and `sniffer/app/unix/uwb_dw1000.c`), so that
pairing is what the bench results were obtained with, and the one to
start from.

What is *not* established is whether `rxauto` is load-bearing in this
mode or merely harmless: nothing here has been measured with `dblbuff`
set and `rxauto` clear. Neither bit covers a timeout, so an `rx_timeout`
callback re-arms for itself either way.

### The `rx_ok` contract changes

Two obligations, both absolute:

1. **Do not re-enable the receiver.** The driver already did, before
   calling you, so that the next frame lands in the other buffer while
   you read this one out.
2. **Read everything you need before returning.** `RX_BUFFER`,
   `RX_TIME`, `RX_FQUAL`, `RX_TTCKI` and `RX_TTCKO` all swing with the
   buffer pointer, which the driver toggles as soon as the callback
   returns. A host that only queues the event and reads the frame later
   gets the previous frame's data with the current frame's length.

### Read out in the callback, but do the work outside it

Obligation 2 says read everything before returning; it does not say
*process* it there, and the two pull in opposite directions. The
callback is open while the receiver is already armed and the next frame
is landing in the other buffer, so anything slow inside it is time the
following frame's read-out is waiting on, and a third frame arriving
while both buffers are held is the overrun below.

The shape that resolves it, in `probe/src/exchange.c` and again in
`sniffer/app/unix/capture.c`: the callback does the SPI read-out into a
ring of slots and returns; whatever costs real time (a `sendmsg(2)`, a
classification, a write to a file) happens afterwards, from the loop,
on what the ring holds. A monotonic count of frames captured indexes the
ring (`seq % depth`), the producer never refuses, and a consumer that
has fallen more than the depth behind detects it on its next read and
counts the loss.

A single slot instead of a ring is not enough: a peer's answer arriving
between two waits is silently discarded, which costs the whole exchange.
`probe` needs a lock around both ends because its producer is on the
port's event thread; an application whose producer and consumer are the
same thread needs none.

### Two registers the read-out needs do not swing

`DRX_RXPACC_NOSAT` and `LDE_THRESH` are absent from UM table 7, so there
is one live instance of each and the next frame's LDE run overwrites
them. The driver samples both for the frame being reported, and the
accessors that need them (`dw1000_rx_get_power_estimate()` and
`dw1000_rx_get_info()`) use the sampled values. Nothing for a caller
to do, but it explains why `max_noise` is not a live read in this mode.

### An overrun is reported as a receive error

A third frame arriving while both buffers are held is an overrun. The
driver turns the transceiver off, resets the receiver, re-aligns the
pointers, issues `HRBPT` (which is the only thing that clears `RXOVRR`,
UM §4.3.5) and calls `rx_error` with `RXOVRR` in the status. Re-arm as
you would for any other error.

The recovery works, and a node that takes an overrun goes on receiving.
A test that reported otherwise was measuring itself: its flood finished
before the receiver's stall ended, and it outran the
one-frame-at-a-time send discipline so almost none of it transmitted.
See AUDIT.md, "Settled: the overrun recovery works, and the test could
not see it".

### `dw1000_rx_start()` syncs the buffer pointers

Pass `DW1000_RX_NO_DBLBUFF_SYNC` when you are enabling the receiver under
a frame you still hold. The driver suppresses the sync by itself for the
window between reporting a frame and releasing it, so the ordinary paths
are safe; the flag is for a host doing its own sequencing.

## What the driver does not do

Absences worth knowing before you design around them:

- **No sleep or deep sleep.** The pieces that look like support are
  inert: `dw->sleep_mode` is accumulated and never written to
  `AON_WCFG`, `AON_CFG1` is cleared but never uploaded with `UPL_CFG`,
  and `TX_ANTD` is not preserved across a sleep (UM §7.2.26). Adding an
  entry point means finishing all three.
- **No carrier-integrator clock offset** (`DRX_CAR_INT`, which is not
  even mapped), no frame-duration helpers, and no event counters
  (register 0x2F). Decawave's driver has all three.
- **`SYS_STATE` (0x19) is mapped and never read.** The IDLE
  preconditions this file states in prose could be checked against it
  rather than asserted.

## Where the reasons live

| Question                               | Look in                           |
| :------------------------------------- | :-------------------------------- |
| What a register write is for           | the comment beside it             |
| How the core is built, and why         | `DESIGN.md`                       |
| Why the driver differs from Decawave's | `DESIGN.md`, and `AUDIT.md`       |
| What is still wrong or unverified      | `AUDIT.md`, "Open"                |
| What a measurement actually was        | `AUDIT.md`                        |
| What the API promises                  | the header, which is the contract |
