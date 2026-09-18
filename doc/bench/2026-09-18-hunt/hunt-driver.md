# Bug hunt: the DW1000 driver's transceiver state machine at 2084ca2

Scope: `hw/drivers/dw1000/src/dw1000.c`, `src/dw1000_send.c`,
`include/dw1000/dw1000.h`, read against `DESIGN.md` ("One state, one
table", "The receiver policy, and the send the chip never began"),
`README.md`, `DW1000.md` and `INVESTIGATE.md`. Every finding below was
reproduced against `port/emulation` unless it says otherwise. Nothing
was changed in the tree.

---

## [Severity: High] A consumed or polled-away completion leaves the other three transmit flags standing, and a start the chip drops inherits them

**Location**
`hw/drivers/dw1000/include/dw1000/dw1000.h:1658` (`dw1000_tx_clear_status_done()`),
`hw/drivers/dw1000/src/dw1000.c:1787` (`_dw1000_tx_release()`),
`hw/drivers/dw1000/src/dw1000.c:1796` (the flag test in `_dw1000_tx_dropped()`),
`hw/drivers/dw1000/src/dw1000.c:2467` (the status clear in `dw1000_tx_start()`, delayed sends only).

**Bug**
`dw1000_tx_start()` clears `TXFRB|TXPRS|TXPHS|TXFRS` for a delayed send
only, and relies otherwise on UM §7.2.17's "automatically cleared at the
next transmitter enable". A `TXSTRT` the chip drops — into a listening
receiver (`DW1000.md`, measured), or under Errata TX-1 — is never a
transmitter enable, so nothing clears them. The two paths that end a
send without going through the `TXFRS` branch of
`dw1000_process_events()` write no status at all
(`_dw1000_tx_release()`), or write `TXFRS` alone
(`dw1000_tx_clear_status_done()`, which is the acknowledgement the
header offers a polling host). The next send therefore starts with the
previous send's flags on the chip, and both of the driver's tests for "is
this send real" read them:

- `dw1000_process_events()` at :2058 takes a standing `TXFRS` for the
  current send's completion;
- `_dw1000_tx_dropped()` at :1796 takes any of `TXFRB|TXPRS|TXPHS|TXFRS`
  as proof the send is on the air and returns `false` before it reads
  `SYS_STATE` at all.

**Trigger** (two, one per path; both reproduced)

*A — a phantom completion.* `dw->state` = IDLE.
1. `dw1000_tx_send(IMMEDIATE)` → `dw->state` = TX. The frame goes out;
   the chip sets `AAT|TXFRB|TXPRS|TXPHS|TXFRS`. The host does not call
   `dw1000_process_events()` (it polls, or it simply sends again).
2. `dw1000_tx_send(IMMEDIATE)` again. `_dw1000_tx_idle()`
   (`dw1000_send.c:45`) reads `TXFRS`, calls `_dw1000_tx_release()` →
   `dw->state` = IDLE, **status untouched**. `dw1000_tx_start()` writes
   `TXSTRT`; the chip drops it (nothing is transmitted, no flag moves).
   `dw->state` = TX, return 0.
3. `dw1000_process_events()`: `status` still carries the *first* send's
   `TXFRS`, so the `TXFRS` branch books a completion, clears `ALL_TX`,
   `_dw1000_tx_done_state()` → `dw->state` = IDLE, and `cb.tx_done`
   fires. The host is told a frame went out that never left, and
   `_dw1000_tx_dropped()` never gets a chance to say otherwise.
   Observed: `status 0x000000f3 (TXFRS=1), tx_done=1 tx_dropped=0,
   dw->state=0`, with the stub medium's TX request count unchanged.

*B — the detector blinded, and a permanent refusal.* `dw->state` = IDLE.
1. `dw1000_tx_send(IMMEDIATE)`; poll `dw1000_tx_is_status_done()`;
   `dw1000_tx_clear_status_done()` → `dw->state` = IDLE, and the chip is
   left at `TXFRB=1 TXPRS=1 TXPHS=1 TXFRS=0` (observed: `0x00000072`).
2. `dw1000_tx_send(IMMEDIATE)`; the chip drops the start. `dw->state` =
   TX, `tx_suspect` = 0, return 0.
3. Any number of `dw1000_process_events()` passes, any amount of time:
   `_dw1000_tx_dropped()` sees `TXFRB` at :1796, clears `tx_suspect` and
   returns `false` every time. `cb.tx_dropped` never fires, `dw->state`
   stays TX.
4. `dw1000_tx_send()` → `_dw1000_tx_idle()` → same test → `false` →
   `DW1000_TX_ERR_BUSY`, for ever. Observed: `tx_dropped=0, dw->state=3,
   next send rc=-2`.

**Impact**
A: the host books a frame the chip never began — the exact failure the
dropped-send detector exists to prevent, and the one `DESIGN.md` says
`_dw1000_tx_dropped()` closes. B: the driver is wedged for a polling
host — every later `dw1000_tx_send()` refused, every
`dw1000_rx_start()` deferred (`_dw1000_rx_start()` at :2573 takes the
same two answers), so the receiver never comes up again either. Both are
mode-independent (reproduced single and double buffered). The
ruby-dw1000 host is event driven and calls `dw1000_process_events()` for
every completion, so it is exposed to A only in the window where it
sends again before processing; it never calls
`dw1000_tx_clear_status_done()`, so B is not its path.

**Fix**
Clear `DW1000_MSK_SYS_STATUS_ALL_TX` wherever a send's flags stop being
this send's — in `_dw1000_tx_release()`, in
`dw1000_tx_clear_status_done()`, and unconditionally in
`dw1000_tx_start()` rather than for a delayed send alone.

**Test**
`tests/emulation/timing.c`, one step run in both builds, e.g.
`step_dropped_start_after_a_consumed_completion`: send, poll
`dw1000_tx_is_status_done()`, do not process; then
`dw1000_emulation_drop_next_start()`, send again, then
`dw1000_process_events()` — `cb.tx_done` must not fire and the stub's
`tx_count` must confirm nothing reached the medium. Second half: the same
with `dw1000_tx_clear_status_done()` in place of the unprocessed
completion — `cb.tx_dropped` must fire within a few passes and the send
after it must be accepted, not `DW1000_TX_ERR_BUSY`. Working step in
`/compat/linux/tmp/claude-1058/-home-sdalu-Repos-ruby-dw1000/fa68e2a8-fd82-4c34-b16d-4e6a3e9a4256/scratchpad/hunt-driver/steps.c`, `hunt_stale_tx_flags()`.

---

## [Severity: High] Single buffered, a good frame beside the completion writes IDLE over TX_W4R while the chip's own WAIT4RESP receiver is up

**Location**
`hw/drivers/dw1000/src/dw1000.c:2011-2012`
(`if (! cfg->dblbuff && ! _dw1000_tx_inflight(dw, status)) dw->state = DW1000_STATE_IDLE;`),
with `_dw1000_tx_done_state()` at `include/dw1000/dw1000.h:668`.

**Bug**
In the `RXFCG` branch, the single-buffered line at :2011 sets
`dw->state` = IDLE whenever the transmitter is not still on the air —
which includes the case where the status word carries this send's own
`TXFRS`. The state at that moment may be `DW1000_STATE_TX_W4R`, and that
is written over. The `RX_W4R` rewrite eighteen lines earlier (:1993) does not
catch it: it tests for `RX_W4R`, not `TX_W4R`. `_dw1000_tx_done_state()`
is then a no-op ("a state the good-frame branch of the same pass has
already moved on … is left where it is" — it assumes RX or RX_W4R, and
IDLE is neither), so the `TX_W4R → RX_W4R` transition the table
promises never happens, while the chip has put the WAIT4RESP receiver
up at the end of the frame.

The double-buffered path handles the same case deliberately (:1928-1941,
`dw->state = DW1000_STATE_TX; dw->rx_deferred = 1;`), which is why
`tests/emulation/timing.c:step_state_beside_completion` passes — it
returns early `if (!TIMING_DBLBUFF)`, so this cell is untested.

**Trigger** (reproduced, single-buffered build)
`cfg->dblbuff` = 0, `rx_keep_on` = 0.
1. A frame arrives and is reported by the chip; the host does not call
   `dw1000_process_events()`. `RXFCG` stands.
2. `dw1000_txrx_idle()` (or `_dw1000_tx_idle()` doing it), which keeps
   the event by design. `dw->state` = IDLE.
3. `dw1000_tx_send(IMMEDIATE | RESPONSE_EXPECTED)` → `dw->state` =
   TX_W4R. The frame goes out; the chip sets `TXFRS` and enables its own
   receiver. Status now carries `RXFCG | TXFRS`.
4. `dw1000_process_events()`. `RXFCG` branch: `_dw1000_tx_inflight()` is
   false (`TXFRS` present), so :2012 writes IDLE over TX_W4R. `TXFRS`
   branch: `_dw1000_tx_done_state()` does nothing, the AUTOACK/AAT
   teardown at :2098 is skipped (it requires `RX_W4R`), and the owed
   receiver reset at :2112 is skipped (same test). End of pass:
   `dw->state` = IDLE, `rx_deferred` = 0.
   Observed: `dw->state=0 (IDLE)`, `SYS_STATE` PMSC = 5 (receiving),
   `dw1000_tx_is_expecting_response() = 0`.
5. `dw1000_tx_send(IMMEDIATE)`. `_dw1000_tx_idle()` asks
   `_dw1000_rx_up(dw)`, reads IDLE, issues no `TRXOFF`, and
   `dw1000_tx_start()` writes `TXSTRT` into a listening receiver. The
   chip drops it. Observed: rc = 0, stub TX requests unchanged, no
   `tx_done`.

**Impact**
Three at once, for any single-buffered host that uses
`DW1000_TX_RESPONSE_EXPECTED` and whose responder is fast enough to
answer before the host polls — which is precisely the case WAIT4RESP
exists for, and which needs no unusual timing at all: the response lands
in the same status word as the completion.
(a) `dw1000_tx_is_expecting_response()`, the documented way to learn
this from inside `tx_done`, answers false while the receiver is up.
(b) A receiver reset owed under UM §4.1.6 (`rx_reset_due`) is dropped on
the floor: neither :2112 nor a later `dw1000_rx_start()` applies it,
because the state says the receiver is not up.
(c) The next send is silently lost — the driver returns 0 and the chip
drops the start — until `_dw1000_tx_dropped()` notices a millisecond
later, and only if a pass runs. With `rx_keep_on` set the receiver is
re-enabled at the end of the pass instead, which masks (c) at the cost
of a second `RXENAB` on an already-listening receiver.
Deviation from `DESIGN.md`'s table in two cells: "good frame / TX_W4R"
says *report, stay*, and "TXFRS / TX_W4R" says *RX_W4R from TX_W4R*.

**Fix**
Make :2011 leave a WAIT4RESP send alone — test `_dw1000_w4r(dw)` (or
`dw->state == DW1000_STATE_TX_W4R`) before writing IDLE, so
`_dw1000_tx_done_state()` gets its TX_W4R and produces RX_W4R.

**Test**
`tests/emulation/timing.c`, a single-buffered twin of
`step_state_beside_completion` (drop its `if (!TIMING_DBLBUFF) return
NULL;` and give the second half a single-buffered expectation of RX_W4R
or RX, never IDLE); assert `sys_state_pmsc(dw) == 5` and that the send
that follows reaches the stub medium. Working step in
`/compat/linux/tmp/claude-1058/-home-sdalu-Repos-ruby-dw1000/fa68e2a8-fd82-4c34-b16d-4e6a3e9a4256/scratchpad/hunt-driver/steps.c`, `hunt_w4r_single_buffer_state()`.

---

## [Severity: Medium] `dw1000_rx_start()` hands an unread received frame back to the chip

**Location**
`hw/drivers/dw1000/src/dw1000.c:2596-2597` (the sync in
`_dw1000_rx_start()`), `hw/drivers/dw1000/src/dw1000.c:1003` (the
`rx_held` guard in `_dw1000_rx_sync_dblbuff()`).

**Bug**
In double buffered receive, `ICRBP != HSRBP` is the *normal* state
whenever the chip has completed a frame the host has not released — the
driver says so itself at :994-1002. `_dw1000_rx_sync_dblbuff()` guards
only `rx_held`, which is set between the `RXFCG` branch and the `HRBPT`
after the `rx_ok` callback (:1951, :2023). A frame the chip has
completed and reported in `SYS_STATUS` but that the host has not yet
processed is *not* held by that flag, so the sync reads the misalignment
as a fault, issues `HRBPT`, and the frame is handed back to the chip
unread. Its `RXFCG` swings out with the buffer, so nothing is left to
show it existed.

**Trigger** (reproduced, double-buffered build)
`cfg->dblbuff` = 1.
1. `dw1000_rx_start(DW1000_RX_IMMEDIATE)` → `dw->state` = RX.
2. A frame arrives. `RXFCG` set, `ICRBP != HSRBP`. Host has not called
   `dw1000_process_events()` yet.
3. `dw1000_rx_start(DW1000_RX_IMMEDIATE)` again — the table's cell for
   `dw1000_rx_start()` in RX is "RXENAB again, RX", and the call returns
   0. `_dw1000_rx_sync_dblbuff()` sees `ic != host` and writes `HRBPT`.
   Observed: `status 0x80006f03 → 0xc0000b02` — `RXFCG` 1 → 0,
   `HSRBP` 0 → 1, `ICRBP` unchanged.
4. `dw1000_process_events()`: no `rx_ok`, no error, nothing. The frame is
   gone.

**Impact**
A silent lost frame on a call the table lists as legal and which reports
success. `README.md` ("`dw1000_rx_start()` syncs the buffer pointers")
tells a host to pass `DW1000_RX_NO_DBLBUFF_SYNC` "when you are enabling
the receiver under a frame you still hold" — but a host cannot know it
holds one until `dw1000_process_events()` has told it, which is exactly
the window this bug lives in, and the sentence after it ("the ordinary
paths are safe") is what does not hold.
The driver's own end-of-pass start (`_dw1000_rx_start(dw,
DW1000_RX_IMMEDIATE, false)` at :2225) passes no flag either. With
`cfg->rxauto` clear it is safe, because the chip stops receiving after
each frame, so at most one frame is ever queued and it is always in the
status word the pass has just read; with `cfg->rxauto` set a second
frame can be queued behind the one being reported, and the end-of-pass
start then discards it. That makes this a reason not to combine
`rxauto` with `rx_keep_on`, on top of the reasons `INVESTIGATE.md` entry
9 gives.
Not a candidate for entry 2: it loses one frame, not a run's worth, and
the ruby-dw1000 host only calls `dw1000_rx_start()` after a
`dw1000_txrx_off()` that has already dropped everything on purpose.

**Fix**
Track "a frame is reported and not yet released" rather than only "a
frame is being read out" — set `rx_held` (or a second flag) when the
chip's `RXFCG` is seen and clear it at the `HRBPT`, and have
`_dw1000_rx_sync_dblbuff()` refuse on that.

**Test**
`tests/emulation/dblbuff.c`, `step_rx_start_keeps_a_queued_frame`:
deliver one frame, wait for the line without processing, call
`dw1000_rx_start(DW1000_RX_IMMEDIATE)`, then
`dw1000_process_events()` — `rx_ok` must fire. Working step in
`/compat/linux/tmp/claude-1058/-home-sdalu-Repos-ruby-dw1000/fa68e2a8-fd82-4c34-b16d-4e6a3e9a4256/scratchpad/hunt-driver/steps.c`, `hunt_rx_start_drops_queued_frame()`.

---

## [Severity: Medium] Suspected: the error and timeout branches undo the good-frame branch's re-enable and clear frame bits that swung in after the snapshot

**Location**
`hw/drivers/dw1000/src/dw1000.c:2149` and `:2195` (the
`_dw1000_txrx_off(dw, ALL_RX_ERR | ALL_RX_TO | ALL_RX_GOOD)` in the
timeout and error branches), against `:1943-1945` (the `RXENAB` in the
double-buffered `RXFCG` branch) and `:2023-2048` (the `HRBPT` after the
callback).

**Bug**
`dw1000_process_events()` runs its branches over one snapshot, in a
fixed order, and the receive-side bits that are *not* in the double
buffered swinging set (`RXPHE`, `RXRFSL`, `RXSFDTO`, `AFFREJ`,
`LDEERR`, `RXRFTO`, `RXPTO`) can stand in the same word as a good
frame's `RXFCG`. When they do, the `RXFCG` branch runs first: it writes
`RXENAB`, reports the frame, and toggles `HRBPT` — after which the
*next* buffer's swinging bits, including a second frame's `RXFCG`,
become visible. The error branch then issues `TRXOFF` (killing the
receive just armed) and writes `ALL_RX_GOOD` into `SYS_STATUS`, which
clears that second frame's `RXFCG` and `RXDFR` although no branch has
reported it, and `_dw1000_txrx_off()`'s own
`_dw1000_rx_sync_dblbuff()` (:2271) then issues `HRBPT` and releases
the buffer. The frame is lost with no error raised for it. The
comment at :2133 and :2179 justifies clearing `ALL_RX_GOOD` as "the
frame-ready flags the failed frame leaves behind" — which is right for
the snapshot, and wrong for what swung in since.

**Trigger** (reasoned from the source; not reproducible in the emulation)
`cfg->dblbuff` = 1, `rx_keep_on` = 1, `rxauto` = 0, no send in flight.
1. A reception fails on a PHY header error (`RXPHE`) — or hits
   `RXRFSL`, `AFFREJ`, `LDEERR`, `RXSFDTO` — after a good frame has
   already been completed into the host-side buffer. The status word
   carries `RXFCG | RXPHE`.
2. `dw1000_process_events()`. `RXFCG` branch: `RXENAB`, `dw->state` =
   RX, `rx_held` = 1, read out, `rx_ok`, `rx_held` = 0.
3. A second frame lands in the other buffer while `rx_ok` runs (the
   receiver was re-enabled in step 2). `HRBPT` → its `RXFCG` is now
   visible.
4. Error branch: `TRXOFF`, `SYS_STATUS ← ALL_RX_ERR|ALL_RX_TO|
   ALL_RX_GOOD` (the second frame's `RXFCG`/`RXDFR` go with it), sync →
   `HRBPT` (the second frame's buffer is released), `_dw1000_rx_reset()`,
   `cb.rx_error`.
5. End of pass: `dw->state` = IDLE, `rx_keep_on && rx_wanted` →
   `RXENAB`. The receiver is back; the second frame is not.

**Impact**
One extra lost frame for each pass that carries a receive error beside a
good frame, in exactly the configuration `INVESTIGATE.md` entries 1 and
2 are about, and invisible to the host: no callback, no status, no
counter. It is not a stuck receiver (`rx_keep_on` re-enables at the end
of the pass), so it does not by itself explain entry 2; it is a
candidate for the one-in-fifty of entry 1, which is about where the
enable sits in the pass.

**Why not proven**
`port/emulation` raises none of the non-swinging receive errors: the
model's receive path explicitly writes `RXPHE`, `RXRFSL`, `RXSFDTO`,
`RXPTO` and `RXRFTO` to zero (`port/emulation/dw/osal/src/osal.c:2041`
onward), and its frame-wait timeout is disarmed the moment a frame
arrives, so a status word carrying both a good frame and a receive error
cannot be produced. The one error it does model, a bad FCS (`RXFCE`), is
part of the swinging set and so cannot coexist with `RXFCG`.

**What would confirm it, on the bench**
On rpi-c or rpi-d, double buffered with `rx_keep_on`, log the raw
`SYS_STATUS` snapshot at the top of every `dw1000_process_events()` pass
and count the passes where `RXFCG` stands together with any of
`RXPHE (1<<12)`, `RXRFSL (1<<16)`, `LDEERR (1<<18)`, `AFFREJ (1<<29)`,
`RXSFDTO (1<<26)`, `RXRFTO (1<<17)`, `RXPTO (1<<21)`. If that count is
non-zero over a soak, the path is live; then read `SYS_STATUS` again
immediately after the `HRBPT` at :2044 and before the error branch, and
compare `RXFCG` there with what the branch clears. (ruby-dw1000 needs a
raw `SYS_STATUS` read for this — the same accessor `INVESTIGATE.md`
entry 6 asks for.)

**Fix**
Give the error and timeout branches a clear mask of
`ALL_RX_ERR | ALL_RX_TO` plus only the good-frame bits that were in the
snapshot, and skip the `TRXOFF`/reset entirely when the `RXFCG` branch
of the same pass has already re-armed the receiver.

**Test**
Not testable against `port/emulation` as it stands. It becomes testable
once the model can raise a non-swinging receive error on demand (an
`dw1000_emulation_raise_rx_error()` knob beside
`dw1000_emulation_drop_next_start()`); the step would then be
`tests/emulation/dblbuff.c:step_error_beside_a_good_frame`.

---

## Tooling

**Ran**
- `clang -fsyntax-only -Wall -Wextra -Wshadow` over `dw1000.c`,
  `dw1000_send.c`, `dw1000_validate.c`, `dw1000_state.c` with the
  emulation port's includes, and again over `dw1000.c` and `dw1000_send.c` with
  `-DDW1000_WITH_PROPRIETARY_LONG_FRAME=1`: **no diagnostics**.
- `clang --analyze -Xanalyzer -analyzer-output=text -Wall -Wextra` over
  the same four sources: **no reports**.
- `sh tests/check-emulation.sh` from the frozen tree, once: all five
  builds (smoke, timing, timing_dblbuff, dblbuff, dblbuff_longframe)
  pass. Every finding above is a gap in that suite, not a regression in
  it.
- A purpose-built test binary under the scratch directory —
  `tests/emulation/timing.c` with three steps added and its own step
  list — built against the frozen tree with the compile line
  `tests/check-emulation.sh` uses, in both receive modes, and once more
  with `clang -fsanitize=address,undefined`: **no sanitizer reports**,
  all three steps reproduce. Sources: `/compat/linux/tmp/claude-1058/-home-sdalu-Repos-ruby-dw1000/fa68e2a8-fd82-4c34-b16d-4e6a3e9a4256/scratchpad/hunt-driver/steps.c`,
  `probe.c`, assembled from `part1.c`/`part2.c`/`part3.c`/`calls.c`; binaries `probe_single`, `probe_dbl`, `probe_san`.

**Install to widen the hunt**
`cppcheck` (`--enable=style,portability`, for the
sign and width mixing in the `SYS_STATUS` bit arithmetic that clang says
nothing about, unchecked here), `clang-tidy` with
`clang-analyzer-*,bugprone-*,cert-*` (for the read-modify-write register
helpers), and `scan-build` for a whole-program view across
`dw1000.c` ↔ `dw1000_send.c`. None is present on this host. A
`-fsanitize=thread` build of the emulation tests would also be worth
having: the driver's callbacks, the model's deadline thread and the
rsvc reader thread meet in `dw1000_process_events()`, and nothing here
checked that boundary.

## Coverage

**Examined**: `dw1000_process_events()` branch by branch (dropped send,
overrun, `RXFCG` single and double buffered, `TXFRS`, timeout, error,
the end-of-pass receiver reconcile) against `DESIGN.md`'s transition
table cell by cell; `_dw1000_tx_dropped()` (the two-sighting rule, the
`tx_airtime` bound, the 40-bit wrap, the `DX_TIME` test for a delayed
send); `_dw1000_txrx_off()`, `dw1000_txrx_stop/off/idle`,
`_dw1000_tx_idle()` and every send entry point; `_dw1000_rx_start()`
including both delayed-start failure paths; `_dw1000_rx_sync_dblbuff()`,
`_dw1000_rx_drop_status()`, `_dw1000_rx_clear_status_dblbuff()` and
`_dw1000_rx_overrun_recover()` for a send in flight and for a held
frame; `_dw1000_tx_clock_force/release` across every path that arms or
cancels a delayed send; every status-clear mask against the snapshot it
is written from; `rx_wanted`, `rx_deferred`, `rx_reset_due`, `rx_held`,
`tx_suspect` at each site that sets or clears them; the interrupt mask
`dw1000_initialise()` derives against the bits the function handles.

**Not examined**: the tuning tables and `dw1000_configure()`'s register
values (AUDIT.md's ground, not the state machine), the OTP and bring-up
sequence, the four formulas (`dw1000_rx_get_power_estimate()`,
`dw1000_rx_power_correction()`, `dw1000_rx_get_clock_drift()`,
`dw1000_get_calibration()`), `dw1000_state.c`, `dw1000_validate.c`, the
probe and sniffer applications, and the ports other than
`port/emulation`.

**On the open symptom (INVESTIGATE.md entry 2)**: no path was found that
leaves rpi-c's receiver off for a run in its configuration (double
buffered, `rx_keep_on` = 1, `rxauto` = 0, callbacks that re-arm nothing,
`dw1000_rx_start()` only after a `dw1000_txrx_off()`). Every branch that
leaves `dw->state` at something other than IDLE with the chip idle —
the `TX`/`TX_W4R` cells, the overrun with a send in flight, the deferred
start — is resolved by the completion's pass, which the host's 1 s
`tx_done` timeout guarantees will come. The one place `dw->state` can
read RX with the chip not receiving, after a good frame with `rxauto`
clear, is between the chip finishing the frame and the pass that reports
it, and that pass writes `RXENAB` before it reports. The three findings
above are all frame-loss or send-loss, none a dead receiver. Finding 4
is the closest candidate and is unproven.

**Stop condition**: a full pass over the checklist (logic, edge cases,
null, numeric, concurrency, resources, error handling, API contracts)
added nothing beyond the four above. Null and resource checks are empty
by construction here — the driver allocates nothing and the only
pointers are the caller's context and the callback table, which
`dw1000_initialise()` refuses to proceed without where it matters
(`dblbuff` with no `rx_error`). The numeric pass found the 40-bit
arithmetic in `_dw1000_tx_dropped()`, `_dw1000_tx_airtime()` and
`_dw1000_tx_prepare_delayed_embed_timestamp()` correct, each with its
wrap handled and its comment saying how. The concurrency pass is
bounded by the driver's own contract (`dw1000_process_events()` performs
SPI and cannot run in an interrupt handler), and everything inside one
pass is single-threaded by that contract.

## Summary

Four findings: two High in the transmit path — a dropped `TXSTRT`
inheriting the previous send's status flags, which turns into either a
completion for a frame that never left or a permanently refusing driver,
and a single-buffered `TX_W4R` overwritten with IDLE beside its own
completion, which costs the send that follows — one Medium where
`dw1000_rx_start()` discards a received frame the host has not been told
about, and one Medium suspected where a receive error in the same pass
as a good frame wipes the next frame unreported.
