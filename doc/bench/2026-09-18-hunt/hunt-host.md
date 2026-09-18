# Bug hunt: ruby-dw1000's host layer over the DW1000 driver

Frozen copy at `.../scratchpad/gem`; driver `decawave-drivers` at `2084ca2`.
Paths below are relative to that copy. Findings are ordered by
severity, cheapest reproduction first on ties; the one that bears on
`INVESTIGATE.md` entries 2 and 3 is the second.

---

## [Severity: High] A failure in either half of `:tx_abort` leaves the receiver off for the life of the process, with transmit still working

**Location** `lib/dw1000/io.rb:381-390`

**Bug**

```ruby
when :tx_abort
    if @tx_pending&.first.equal?(done)
        @tx_pending = nil
        @dw.txrx_off
        @dw.rx_start(0)
    end
```

`txrx_off` and `rx_start` are a pair -- turn the radio off, then put the
receiver back -- but nothing makes them atomic and nothing retries. Both
raise `DW1000::Error::IOError` on a failed SPI transfer: `_spi_check` runs
on entry to every binding via `DW1000_RB_DATA` (`ext/dw1000.c:239-244`)
and again on the way out of `dw1000_m_rx_start` (`ext/dw1000.c:1643`). The
command loop's `rescue StandardError` (`io.rb:130-134`) logs and goes on to
the next command, and **nothing in `DW1000::IO` ever re-arms the receiver
again**: `#process_event` deliberately re-arms nothing (`io.rb:79-84`,
`:436-437`, `:447-449`), and `#start` is the only other `rx_start` and
cannot be called twice.

On the chip the second case is unrecoverable rather than merely untidy:
`dw1000_txrx_off` clears `rx_wanted` (`decawave-drivers/.../dw1000.h:1407`),
which is the flag the driver's end-of-pass `rx_keep_on` re-enable tests
(`src/dw1000.c:2222-2227`, the `rx_keep_on` block). With `rx_wanted` at 0 and the host never
calling `rx_start` again, the receiver stays off permanently. Sends are
unaffected: `tx_send` and the `tx_done` events keep working.

**Trigger**
1. A `#transmit` reaches `TX_TIMEOUT` and `#disown` queues `:tx_abort`
   (`io.rb:214`, `:334-338`).
2. One SPI transfer inside `@dw.txrx_off` fails; the Unix osal records it
   in `spi_drv.error` instead of asserting.
3. `@dw.rx_start(0)` raises on entry from `_spi_check`, before touching
   the chip. The receiver is off, `rx_wanted` is 0, the command thread
   logs and carries on.
4. Every later frame is missed; every later `#transmit` succeeds.

The `txrx_off`-raises variant (step 2 raising instead) is the same shape
with a different end state.

**Impact** Silent, permanent deafness with a healthy-looking transmitter
-- the exact signature `INVESTIGATE.md` entry 2 describes, though *not*
its cause in that particular run (see the elimination in the next
finding: nothing was refused there, so no abort ran).

**Fix** Wrap the pair so the re-arm is owed unconditionally and retried --
e.g. `begin; @dw.txrx_off; ensure; retry_rx_start; end`, with a
`@rx_owed` flag the next `:interrupt` honours.

**Test** Proven. `hunt-host/test_probe.rb`,
`TestProbe#test_p1_tx_abort_with_a_failing_txrx_off_leaves_the_receiver_off`
and `#test_p2_tx_abort_with_a_failing_rx_start_never_retries`, both
failing against the tree:

```
P1: receiver left off after a failed abort: [:rx_start, :txrx_off]
P2: nothing ever re-arms the receiver after a failed abort:
    [:rx_start, :txrx_off, :rx_start]   (a later :rx_timeout/:rx_error
                                         adds no further :rx_start)
```

These belong in `test/unit/test_io.rb` beside
`test_stop_survives_a_failing_txrx_off`, which covers the same failure on
the `#stop` path only.

---

## [Severity: High] The two duplex nodes never rendezvous, so the node that comes up last loses a contiguous prefix of the peer's burst

**Location** `test/pair/test_stress.rb:114-118`, with `test/roles/node.rb:188-191`

**Bug**
`test_both_nodes_can_transmit_at_the_same_time` spawns both roles back to
back and only then waits for either to announce itself:

```ruby
jobs = [ [ HOST_A, "dup-a", "dup-b" ], [ HOST_B, "dup-b", "dup-a" ] ]
       .map { |host, me, peer| Remote.role(host, "duplex", DUPLEX, me, peer) }
jobs.each(&:await_ready)
```

`Node#duplex` bursts on its **own** READY and on nothing else:

```ruby
collector = Thread.new { collect(peer, deadline) }
ready
sent = burst(count, mine, gap: gap)
```

There is no cross-node rendezvous anywhere: not in `duplex`, not in
`Remote.role`, not in `Job#await_ready` (which only tells the *harness*
that a node is up, long after that node has started sending). Every other
pair test in the file uses the correct shape -- `Remote.role(HOST_B,
"count", ...).await_ready` is fully up before `Remote.role(HOST_A,
"flood", ...)` is even spawned (`test_stress.rb:30-32`, `:59-62`,
`:85-89`). `duplex` is the single exception.

**Trigger**
Let d be the skew between the two nodes' READY instants (ssh handshake,
ruby boot, `DW1000.new` -- hardreset, LDE load, radio configuration --
plus the `Remote.cleanup(host)` ssh that `Remote.role` runs per role,
which polls for up to 3 s when a previous role is still dying). The
default burst is `DUPLEX = 50` frames at `gap = 0.03`, i.e. **1.5 s
long**.

- The node that is ready first has its collector running for the whole of
  the peer's burst; `collect` cannot give up early on it, because
  `quiet_at` stays `nil` until the first peer frame and the fallback
  deadline is `count*gap + din + 30` = 34.5 s (`node.rb:184-187`, `:536-550`).
  So the early node hears everything the link delivers.
- The node that is ready last starts its collector d seconds into the
  peer's burst and can only ever see the last `50 - d/0.03` frames. The
  losses are a **contiguous prefix** of the sequence numbers, not a
  scatter.
- d >= 1.5 s and it hears nothing at all.

The run in `INVESTIGATE.md` entry 2 -- rpi-c received 2 of 50, all 50 of
its own sends completed, nothing refused -- is exactly d ~= 1.44 s:
received 2 means seq 48 and 49 only. And "nothing refused" *rules out*
the `:tx_abort` path above as the cause there: a refused or abandoned
send would have made `sent < count` and `Node#duplex` would have exited
non-zero (`node.rb:195`, `burst` at `:479-486`).

The same mechanism fits entry 3: a systematic d of 0.36-0.51 s in one
direction gives "rpi-c hears 33 to 38 of 50" while "rpi-d hears 47 to 50",
with no antenna, sensitivity or transmit-power difference needed.

**Impact**
The two numbers `INVESTIGATE.md` entries 2 and 3 rest on are not a
measurement of the link: they conflate link loss with start skew. The
suite cannot catch it -- `MIN_DELIVERY` defaults to 0, so
`assert_clean_delivery` only requires `received > 0`, which 2 satisfies
(`test_stress.rb:26`, `:139-143`). A driver hunt for entry 2 is chasing a
harness artefact.

**Fix** Have `duplex` wait for its first frame from the peer (or for a
peer "go" frame) before starting its own burst, and have the harness
`await_ready` both jobs before either can send.

**Test** Not reachable from `test/unit` against `fake_device.rb`: it is a
property of two processes on two hosts. *Suspected, needs the bench* --
but the decisive observation is free and needs no new run if the
per-run logs are kept:

- Add the sequence numbers to the role's own output: `stats("duplex",
  ..., first: received.min, last: received.max)` at `node.rb:193-194`.
  Re-run one soak. If the misses are the prefix `0..k` on the node with
  the lower count, this finding is the cause and entries 2 and 3 close.
  If they are scattered, it is the radio and this finding is only a
  confound.
- Cheaper still, as a one-off: print `Process.clock_gettime(CLOCK_REALTIME)`
  beside READY on both nodes and read d off two logs.

---

## [Severity: Medium] `#process_event` is not exception-safe around the chip's interrupt mask, and `_event_add` can raise from inside a driver callback

**Location** `ext/dw1000.c:2132-2172` (the mask/unmask window), `ext/dw1000.c:406-416` (`_event_add`)

**Bug**

```c
dw1000_interrupt(&w->dw, irq_mask, false);   /* SYS_MASK -> 0 */
...
dw1000_interrupt(&w->dw, irq_mask, true);    /* restored */
```

There is no `rb_ensure` and no `rb_protect` around the body. The design
depends on that restore: the DW1000 asserts its IRQ line on
`SYS_STATUS & SYS_MASK`, so with `SYS_MASK` at 0 the line is low, and it
is the *restore* that produces the rising edge the host polls for
(`interrupt_wait` requests `BITTERS_GPIO_INTERRUPT_RISING_EDGE`,
`ext/dw1000.c:812-817`). If a Ruby exception longjmps out of that window,
`SYS_MASK` stays 0, the line never rises again, and the host receives no
further interrupt -- no frames, no completions -- for the life of the
process.

`rb_protect` guards the five callback funcalls, and `_spi_check` is
deliberately after the restore. What is *not* guarded:

- `rb_warn("dw1000 event buffer full, event dropped")` at `:409`, which
  runs **inside a driver callback**, mid-`dw1000_process_events`. This is
  the very thing the file's own rule for that context forbids: "Nothing
  may raise: this runs inside the driver, which would be left half-way
  through its event processing" (`ext/dw1000.c:427-429`). `rb_warn` goes
  through `Warning.warn`, i.e. Ruby code writing to `$stderr` -- an
  overridden `Warning.warn`, a full or broken stderr (`Errno::ENOSPC`,
  `Errno::EPIPE`) all raise. In the roles stderr is a harness tempfile,
  so `ENOSPC` is the live one.
- `rb_warn("discrepancy in event processing reporting")` at `:2147`, same
  mechanism.
- Allocation inside `_rx_frame_value` and the array build (`NoMemoryError`).

**Trigger** The event buffer fills (see the next finding), `_event_add`
returns NULL and calls `rb_warn`, and the warning write fails. The
callback longjmps out of `dw1000_process_events`, past the driver's
end-of-pass receiver re-enable *and* past the unmask. Receiver off,
interrupt masked, permanently.

**Impact** Permanent deafness, same class as the previous finding but
reached without any SPI failure.

**Fix** Run the body under `rb_protect`/`rb_ensure` so the unmask always
happens and the exception is re-raised after it, and drop `rb_warn` from
`_event_add` in favour of a counter the caller reports after the window.

**Test** Not reachable from `test/unit`: it is inside the C extension, and
the unit suite runs against `FakeDevice`, which has no interrupt mask. The
probability, not the mechanism, is what is unproven -- mark *suspected* on
frequency only. A bench confirmation: run a role with a `Warning.warn`
that raises and a forced buffer overflow, and check the node stops
reporting events while `#state` still answers.

---

## [Severity: Medium] The event buffer holds four entries but five callbacks are reachable in one pass, and the overflow is invisible to the cross-check

**Location** `ext/dw1000.c:134-159` (`entry[4]` at `:158`), `ext/dw1000.c:406-416`

**Bug** The sizing comment claims "the driver checks the TX-done, RX-ok,
RX-timeout and RX-error status groups independently and fires each
callback at most once per call, so up to 4 events can accumulate", and
`_event_add` calls the 4-slot bound "unreachable in practice". Both are
wrong on `2084ca2`. Walking `dw1000_process_events` (`src/dw1000.c:1846-2233`)
in order, with `tx_dropped` counted as the fifth callback the binding
registers:

| order | callback | condition |
| :-- | :-- | :-- |
| 1 | `tx_dropped` | `_dw1000_tx_pending && _dw1000_tx_dropped` (`:1863`, callback at `:1841`) |
| 2 | `rx_error` | entry `RXOVRR` (`:1872`, callback at `:1761`) -- excludes 3..7, it strips the RX bits |
| 3 | `rx_ok` | `RXFCG` (`:1891`, callback at `:2016`) |
| 4 | `rx_error` | **second** overrun test after the read-out, before HRBPT (`:2038`) |
| 5 | `tx_done` | `TXFRS` (callback at `:2120`) |
| 6 | `rx_timeout` | `ALL_RX_TO` (callback at `:2164`) |
| 7 | `rx_error` | `ALL_RX_ERR` (callback at `:2210`) |

1 and 5 are mutually exclusive (`_dw1000_tx_dropped` returns false when
`TXFRS` is set, `src/dw1000.c:1796-1800`), and 2 excludes 3,4,6,7. The worst case is
therefore **3+4+5+6+7 = five events into four slots**: a frame read out,
an overrun raised while it was being read, the host's own completion, a
timeout and an error in one status word.

**Trigger** The fifth `_event_add` returns NULL. The lost event is always
the tail one (`rx_timeout` or `rx_error`), so `DW1000::IO` loses only the
overrun warning at `io.rb:452-461` -- `rx_ok` can never be the casualty,
since the entry-overrun branch that would push it to index 4 excludes it.
What makes this worth reporting is that the loss is *silent*: the
extension's own self-check is `processed != has_event` (`:2144-2148`),
which is true-vs-true here and says nothing, and the pair suite only
greps for `discrepancy in event processing`
(`test_stress.rb:152-155`) -- never for `event buffer full`.

Note also that `_rx_ok` skips `_rx_ok_read_frame` entirely when
`_event_add` returns NULL (`ext/dw1000.c:453-458`), so an `rx_ok` that
*did* land in a full buffer would be reported to the Ruby callback with
the frame never read out of the receive buffer before HRBPT hands it
back. Unreachable today by the index argument above; one more callback in
the driver makes it reachable.

**Fix** Size `entry[]` from the callback count the binding registers (5,
or 8 with headroom) and assert on overflow at build time rather than
warning at run time.

**Test** Not reachable from `test/unit` (the buffer is in the extension;
`FakeDevice#process_event` hands back whatever the test queued). Reachable
on the bench from the `overrun` role, which is the only thing that
provokes the double-overrun path: *suspected, needs the bench* -- run
`ruby test/roles/node.rb overrun 40 stress` against a `flood 3000 stress 0`
peer and grep the node's output for `event buffer full`.

---

## [Severity: Medium] `#stop` talks to the chip while the command thread may still be inside a device call

**Location** `lib/dw1000/io.rb:265-295` (`join(1)` at `:272`, `txrx_off` at `:288`, `@tx_pending` at `:281`)

**Bug** `#stop` kills the command thread and joins it with a 1 s bound,
then proceeds unconditionally. The bound is deliberate ("in case it sits
in a chip call", `:271`) but there is no branch for the case it was put
there for: when the join times out, `@dw.txrx_off` runs on the main
thread while the command thread is still inside `#process_event` or
`#tx_send`. `Thread#kill` cannot land while the extension holds the GVL,
which it does for the whole of `dw1000_m_process_event` and
`dw1000_m_tx_send` -- only `interrupt_wait` releases it. Two threads then
drive the same `dw1000_t` and the same SPI device: interleaved transfers,
and `_dw1000_txrx_off`'s save/restore of `SYS_MASK`
(`src/dw1000.c:2246-2284`) racing `#process_event`'s own mask/unmask, which
can leave `SYS_MASK` at 0.

`@tx_pending = nil` at `:281` is the same race in Ruby: the comment at
`:86-88` says it is "written by the command thread only (and by #stop,
once it is dead)", and after a timed-out join it is not dead.

**Trigger** A chip that has stopped answering, or an SPI transfer longer
than 1 s -- the same failure that usually prompts the `#stop` in the
first place.

**Impact** Bounded to teardown, so it cannot produce a mid-run symptom.
It can leave the chip's interrupt masked for whatever object opens it
next, and it can corrupt a final transfer.

**Fix** Loop the join, or keep a flag so the post-join work is skipped
when the thread is still alive and reported to the logger instead.

**Test** Proven. `hunt-host/test_probe.rb`,
`TestProbe#test_p3_stop_touches_the_device_concurrently_with_a_stuck_command`:
a `read_temp_vbat` that stalls under `Thread.handle_interrupt(Object =>
:never)` (modelling the GVL held across a device call) makes `#stop`
return after exactly 1.0 s while the command thread is still inside it.
Note `test_io.rb:44-54` stalls for 0.2 s only, i.e. below the join bound,
so the existing suite never reaches this branch.

---

## [Severity: Low] `interrupt_wait` ignores the result of `bitters_gpio_irq_wait`, so a failed consume spins the IRQ thread

**Location** `ext/dw1000.c:2069-2073`

**Bug**

```c
assert(w->pollfd[0].revents);
bitters_gpio_irq_wait(&w->irq.pin);
return rb_id2sym(id_interrupt);
```

`bitters_gpio_irq_wait` returns `-errno` on a failed `read(2)` and
`-ERANGE` on a short one (`bitters/src/gpio.c:911-926`); the return value
is discarded. The `assert` above it is compiled out under `NDEBUG`, which
is how the gem ships. When the read fails, the queued GPIO line event is
**not consumed**, so the next `poll(2)` returns ready immediately and
forever: `DW1000::IO`'s IRQ thread (`io.rb:106-120`) spins pushing
`:interrupt`, and the command thread burns SPI on `#process_event` calls
that find nothing -- the host is effectively not reading for the rest of
the run.

**Trigger** `POLLERR`/`POLLNVAL` on the line descriptor, or `EAGAIN`
after something set `O_NONBLOCK` on it (`bitters_gpio_irq_callback` does
exactly that, `bitters/src/gpio.c:971-977`; nothing in this gem calls it,
so the live path is the poll-error one).

**Impact** A livelock that looks like a wedged node, with the CPU pinned.

**Fix** Check the return and `rb_syserr_fail` on a negative one, the way
the `poll` error is handled two lines above.

**Test** Not reachable from `test/unit`: `FakeDevice#interrupt_wait` is a
queue pop. *Suspected, needs the bench* -- confirmable only by injecting a
read failure; more cheaply, confirmable by reading, since the missing
check is on the face of it.

---

## [Severity: Low] Every received frame costs an unrequested SAR read on the chip, with the receiver live

**Location** `lib/dw1000/io.rb:411`, `decawave-drivers/hw/drivers/dw1000/src/dw1000.c:1595-1626`

**Bug** `#process_event`'s `:rx_ok` branch always calls
`@dw.read_temp_vbat`, whether or not the caller ever looks at
`meta.temp`/`meta.vbat` -- `Node#collect`, `#count` and `#rx` never do.
The call is not a register read: it writes `RF_CONF` sub-registers 0x11
and 0x12 to bring up the TLD and ADC biases and enable the SAR outputs,
writes `TC_SARC` twice, busy-waits 4 us inside the driver
(`_dw1000_delay_usec`), reads `TC_SARL`, and writes `TC_SARC` again --
seven SPI transactions plus a spin, per frame, on the command thread,
with the receiver already re-enabled by the driver and another frame
possibly landing in the other buffer. The bias enables are never
restored.

**Trigger** Any received frame. At `duplex`'s 30 ms cadence it is small;
under the `flood` roles it is once per frame at the radio's rate, and it
is serialised ahead of the next `:interrupt` and any queued `:tx`.

**Impact** Two separable ones, neither established here: a per-frame
latency the command thread adds between one event pass and the next, and
a possible RF-front-end perturbation from driving the SAR biases while
the receiver is enabled (DecaWave's own `dwt_readtempvbat` is used with
the transceiver idle). Either would be a host-side contribution to the
receive-side losses of `INVESTIGATE.md` entries 1 and 3.

**Fix** Read the sensors only when asked -- an option on `DW1000::IO.new`,
or a lazily-evaluated `Meta#temp`.

**Test** Not reachable from `test/unit`: `FakeDevice#read_temp_vbat`
returns a constant. *Suspected, needs the bench.* The experiment is one
run: patch `io.rb:411` to `m.temp, m.vbat = nil, nil`, re-run one
duplex soak, and compare the received distributions against the numbers
already in `INVESTIGATE.md`. If the loss moves, this is a host-side
contributor to entry 3 and the antenna hypothesis there is not the whole
story.

---

## [Severity: Low] Driver callbacks can append to the event buffer outside any `#process_event` window, where the entry is never reported

**Location** `ext/dw1000.c:2141` (`w->event.count = 0`, the only
reset), `decawave-drivers/hw/drivers/dw1000/src/dw1000.c:2574` (`_dw1000_tx_dropped`
called from `_dw1000_rx_start`)

**Bug** `w->event` is treated as a per-pass buffer -- zeroed at the top of
`dw1000_m_process_event` and drained at the bottom -- but the callbacks
that fill it are the driver's, and the driver fires `tx_dropped` from
`dw1000_rx_start` as well, which the binding exposes as `#rx_start`. An
event added there is written into `w->event`, counted, and then discarded
by the next `#process_event`'s `count = 0`. The Ruby-level `tx_dropped`
handler still runs (a no-op), so nothing clears `@tx_pending` and the
caller waits out the full `TX_TIMEOUT` -- precisely what the
`:tx_dropped` event exists to avoid (`ext/dw1000.c:497-508`).

**Trigger** `#rx_start` called with a send pending on the chip. Not
reachable through `DW1000::IO` as it stands: `#start` runs before any
send, and `:tx_abort` calls `txrx_off` first, which clears the pending
send -- and in the one ordering where it would not (a raising
`txrx_off`), `#rx_start` is never reached at all (the `:tx_abort` finding). Reachable
from any other host that uses `#rx_start` over a send, which the driver
documents as supported.

**Impact** A dropped send reported to nobody, and one caller waiting
`TX_TIMEOUT` instead of failing at once. Latent today.

**Fix** Drain or reject out-of-window callbacks: set a flag around the
`dw1000_process_events` call and have `_event_add` refuse (or report
separately) when it is clear.

**Test** Not reachable from `test/unit` (extension-internal, and no IO
path leads there). Reachable in C only; report as latent.

---

## Tooling

**Ran**
- `rake test:unit` in `.../scratchpad/gem` -- 145 runs, 431 assertions,
  0 failures, 0 errors, 3 skips (baseline green). 1 of the 3 permitted runs used.
- `ruby -w -I gem/lib -I gem/test hunt-host/test_probe.rb` -- 3 probes,
  3 failures, all as predicted (P1, P2, P3 above). Two runs.
- `ruby -w -c` on `lib/dw1000/io.rb`, `test/roles/node.rb`,
  `test/support/remote.rb` -- syntax OK. (The Rakefile already runs the
  suites under `-w`; no warnings from the host files.)
- `rubocop --only Lint,Security --force-default-config` on the same three
  files -- 3 files inspected, no offenses detected.
- Manual read of `ext/dw1000.c` (whole file), `lib/dw1000/io.rb` (whole),
  `test/roles/node.rb` (roles and radio helpers), `test/support/remote.rb`
  (whole), `test/support/fake_device.rb`, `test/unit/test_io.rb`,
  `test/pair/test_stress.rb`, `bitters/src/gpio.c` (IRQ paths), and
  `decawave-drivers/hw/drivers/dw1000/src/dw1000.c`
  (`dw1000_process_events`, `_dw1000_rx_start`, `_dw1000_txrx_off`,
  `_dw1000_tx_dropped`, `dw1000_read_temp_vbat`) plus `dw1000_reg.h` masks
  and `INVESTIGATE.md`.

**Skipped, per the brief** `clang -fsyntax-only` on `ext/dw1000.c`: it
needs the Ruby headers and the vendored include tree configured, and the
extension is not built in this copy.

**Install to widen the hunt**
- A `clang`/`gcc` syntax-and-warning pass would want `rake compile` to
  have run once so `ext/Makefile`'s `-I` set is real; `-Wall -Wextra
  -Wmissing-field-initializers` on `ext/dw1000.c` would also flag the
  unused `VALUE r` results of the four `rb_protect` calls at `:486`,
  `:511`, `:532` and `:553`.
- `helgrind`/`ThreadSanitizer` over a built extension would settle the
  `#stop` race of finding 5 mechanically rather than by the Ruby-level
  proxy used here.

---

## Coverage

**Examined** `lib/dw1000/io.rb` in full (both threads, `#start`, `#stop`,
`#transmit`/`#disown`, `#receive`, `#process_command`, `#process_event`,
`#check_payload`, `#estimate_delayed_send_lead_time`); `ext/dw1000.c` in
full, with close attention to `dw1000_m_initialize`'s configuration, the
five callbacks and their `rb_protect` wrappers, `_event_add`,
`_rx_ok_read_frame`, `dw1000_m_process_event`, `dw1000_m_interrupt_wait`,
`dw1000_m_tx_send`, `dw1000_m_rx_start`, `dw1000_m_txrx_off` and
`_spi_check`; `test/roles/node.rb`'s `duplex`, `collect`, `burst`,
`receive_until`, `count`, `flood`, `overrun`/`collect_overrun`;
`test/support/remote.rb` in full; `test/pair/test_stress.rb`;
`test/unit/test_io.rb` and `test/support/fake_device.rb` for what is
already covered; and enough of the driver and of `bitters/src/gpio.c` to
settle each host-side question against them.

**Checked and dropped** (a guard, an invariant or an existing test
refutes them, so they are not reported):
- `SYS_MASK` drift between `_irq_mask` and what `dw1000_initialise`
  enables -- the two sets are identical (`ext/dw1000.c:316-325` vs
  `src/dw1000.c:1387-1398`), so the masked window really does drive the
  IRQ line low and the restore really does regenerate the edge.
- `:tx_abort` racing a late completion, and a late completion answering
  the *next* `#transmit` -- both ways round are handled
  (`io.rb:386`, `:423-435`) and covered by
  `test_a_completion_arriving_after_the_timeout_is_not_reused` and
  `test_an_abandoned_transmit_does_not_complete_the_next_one`. `disown`
  runs inside `@tx_mutex`, so `:tx_abort` is always queued ahead of the
  next `:tx`; FIFO does the rest.
- `done, want_ts, embedded = @tx_pending` with `@tx_pending` nil -- Ruby
  destructures nil to all-nil; the `if done` branch at `io.rb:425` is correct.
- A `nil` from `#process_event` on a real event -- `|| []` at `io.rb:350`,
  and the extension's `processed != has_event` cross-check covers the
  driver side.
- An exception escaping one event's handler taking the rest of the batch
  -- the per-event `rescue` at `io.rb:352-358` and
  `test_a_failing_event_does_not_skip_the_rest_of_the_batch`.
- The IRQ thread dying on a failed wait -- `rescue StandardError` plus the
  0.1 s pause at `io.rb:111-118`, and
  `test_a_failing_interrupt_wait_does_not_kill_the_engine`.
- The command thread spinning on `pop` returning nil from a closed empty
  queue -- unreachable: `#stop` kills the thread before closing
  (`io.rb:266-278`).
- `#collect`'s idle window truncating the peer's burst on its own -- the
  window (3 s) is twice the burst (1.5 s) and `quiet_at` stays nil until
  the first peer frame, so it cannot end a burst early; what it *does*
  truncate is a burst the collector joined late, which is the duplex-rendezvous finding.
- `#estimate_delayed_send_lead_time`'s bisection invariants -- correct,
  and covered by three tests.
- `#check_payload`'s `room` arithmetic for payloads shorter than the
  timestamp -- negative `room` makes `between?` false, which raises.
- The `begin`/`end` with no rescue at `io.rb:405-412` -- vestigial, but the
  behaviour it leaves (a frame whose sensor read failed is dropped, the
  driver re-arms) is what the comment claims and what
  `test_a_failing_read_drops_the_frame_and_the_engine_goes_on` asserts.
  Style, not correctness, so not reported.

**Not examined** `lib/dw1000/sniffer.rb` and `lib/dw1000/sniffer/`,
`bin/dw1000-sniff`, `test/unit/test_sniffer.rb`, the TWR roles and
`test/support/twr.rb`, `test/hw`, `ext/vendoring.rb` and the packaging
tests, `bitters` outside the GPIO IRQ paths, and the driver outside the
functions the host calls -- all outside the brief's target.

**Stop condition** A full pass over the checklist (logic, edge cases,
null, numeric, concurrency, resources, error handling, API contracts)
over the named targets added nothing after the eighth; the remaining
candidates were all refuted by a guard or an existing test and are listed
above rather than padded into Low findings.

---

**Summary** Eight findings: the duplex pair test never makes the two
nodes start together, which reproduces `INVESTIGATE.md` entry 2's "2 of
50" exactly and offers a complete alternative to entry 3's antenna
hypothesis; and `:tx_abort` can leave the receiver off permanently while
transmit keeps working -- proven with failing probes, though eliminated as
the cause of that particular run.
