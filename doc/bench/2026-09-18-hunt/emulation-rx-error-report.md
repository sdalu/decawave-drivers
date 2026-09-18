DONE_WITH_CONCERNS

Asked:
  One emulation knob, `dw1000_emulation_fail_next_frame()`, making the next
  frame delivered to the model end as a PHY header error (RXPHE) instead of a
  good frame; and one step, `step_error_beside_a_good_frame`, in
  tests/emulation/dblbuff.c reproducing finding 4 of
  doc/bench/2026-09-18-hunt/hunt-driver.md. Patch to knob.patch, report here,
  the driver under hw/ untouched.

Attempts:
  Read first: the finding (hunt-driver.md:256-342), the driver's double
  buffered RXFCG branch (dw1000.c:1938-2137), its error branch
  (dw1000.c:2259-2301), `_dw1000_txrx_off()` (:2336) and
  `_dw1000_rx_sync_dblbuff()` (:994); the model's receive path
  (osal.c:2041-2112 before this patch), its passthrough and swinging masks
  (osal.c:200, :934), `e_rx_auto_reenable()` (:491),
  `e_dblbuff_advance_ic()` (:529); and `dw1000_emulation_drop_next_start()`
  (osal.c:84, :1140, :1604), whose shape
  the new knob copies.

  Three files changed, all inside the permitted directories:

  - port/emulation/dw/osal/include/dw1000/emulation.h: the declaration and its
    header comment, beside `dw1000_emulation_drop_next_start()`.
  - port/emulation/dw/osal/src/osal.c: a one-shot `fail_next_frame` flag in
    `struct dw1000_emulation` beside `drop_next_start`; the knob itself beside
    `dw1000_emulation_drop_next_start()`; and, in the DW1000_RSVC_RX case of
    `rsvc_uwb_handler()`, after the receive timeout is disarmed and before the
    overrun test, the PHY-header-error path: RXPRD, RXSFDD, RXPHD and RXPHE
    set, nothing else touched, no write to RX_FINFO / RX_BUFFER / the
    timestamps, `taken` left false so ICRBP does not move, the receiver
    re-enabled through `e_rx_auto_reenable(e, false)` (UM 7.2.6: a *failure*
    re-enables in both buffering modes), the interrupt recomputed, done. It is
    a failure, not a frame, so it never asks for a buffer and the overrun path
    below it is deliberately not reached.

    Why RXPHE works where the model's own RXFCE cannot: SYS_STATUS's
    passthrough mask (osal.c:200, byte 1 = 0x1b) makes bits 8, 9, 11 and 12 --
    RXPRD, RXSFDD, RXPHD, RXPHE -- single instances written into *both*
    register sets, while 10, 13, 14, 15 (LDEDONE, RXDFR, RXFCG, RXFCE) are the
    swinging set of UM table 7. So the error is readable whichever buffer the
    host is on, and stands beside another buffer's RXFCG; RXFCE never can.

  - tests/emulation/dblbuff.c: `step_error_beside_a_good_frame`, an `err_probe`
    record, an rx_error callback (`cb_rx_error_probe`, now the config's
    `.cb.rx_error` in place of `cb_nothing`; `step_dblbuff_needs_rx_error`
    sets its own and is unaffected), a dispatch at the head of the existing
    `cb_rx_probe`, two bounded waits (`wait_for_status`, `wait_for_ic_move`;
    1000 x 2 ms = 2 s, a timeout becoming a REASON naming what was waited for,
    never a hang), `deliver_from_callback()`, and file statics `g_stub` and
    `g_emulation` beside `g_config` / `g_radio`.

  The step, in order: `restart()`; deliver frame 1 (good, unreported); call the
  knob and deliver frame 2 (RXPHE); check the premise (RXFCG and RXPHE
  together, HSRBP != ICRBP, no RXOVRR); one `dw1000_process_events()` whose
  rx_ok asks the medium for frame 3 and waits until the model has completed it
  (ICRBP moved again); record SYS_STATUS after the pass; a second
  `dw1000_process_events()`; then assert the correct behaviour -- frame 3 still
  visible in SYS_STATUS when the first pass ends, or reported by rx_ok in the
  second pass.

  `deliver_from_callback()` writes RXENAB straight into SYS_CTRL rather than
  calling `dw1000_rx_start()`, the way the file's own `hrbpt()` writes HRBPT:
  inside rx_ok the driver is mid-pass, holds the buffer being read out and has
  already re-enabled the receiver, and a driver call there would put the
  driver's receive policy on top of the very pass under test. The model asks
  the medium for frames on every receiver enable, which is what makes the burst
  arrive.

  One correction to the finding's own trigger, as the brief established: it
  says `rxauto` = 0, and with RXAUTR clear the receiver is idle after a good
  frame, so no error can arrive to stand beside its RXFCG. The step runs with
  RXAUTR set, which is what dblbuff.c's config already has and what UM 4.3.1
  recommends alongside the double buffer. `rx_keep_on` is left as the other
  dblbuff steps have it (unset); it governs only whether the receiver comes
  back at the end of the pass, not the loss of frame 3.

Checks:
  `sh tests/check-emulation.sh` from the worktree root: run once, of the four
  allowed. Exit status 1. Expected final state met exactly: smoke, timing and
  timing_dblbuff pass; dblbuff and dblbuff_longframe fail on the new step and
  on nothing else. Own compile lines copied from that script (build/one.sh
  under Scratch) were used for the single builds and for five repeat runs of
  the dblbuff binary: the failure and every recorded word are identical each
  time, so it is not a race. No new compiler warning; the one warning printed
  (`unused function 'cb_rx_nothing'`) is pre-existing at 7226204.

  Exact output of the final `sh tests/check-emulation.sh` run:

    emulation: building the smoke test, port/emulation, cc
    emulation: smoke test passed
    emulation: building the timing test, port/emulation, cc
    emulation: timing test passed
    emulation: building the timing_dblbuff test, port/emulation, cc
    emulation: timing_dblbuff test passed
    emulation: building the dblbuff test, port/emulation, cc
      ok: aligned at reset
      ok: one frame moves ICRBP
      ok: two frames, two sets
      ok: second frame interrupts
      ok: bad crc keeps buffer
      dw1000-emulation: receiver overrun: both buffers hold a frame the host has not read out
      ok: three frames overrun
      dw1000-emulation: packet lost (receiver not enabled: IDLE)
      ok: single buffered
      ok: dblbuff needs rx_error
      ok: frame held across txrx_off
      ok: a queued frame survives rx_start
          status after frame 1        : 0x80006f03
      dw1000-emulation: frame failed in its PHY header on request (RXPHE)
          status after frame 2        : 0x80007f03
          status inside rx_ok, entry  : 0x80007f03
          status inside rx_ok, frame 3: 0x00000b02
          status handed to rx_error   : 0x80007f03
          status after the first pass : 0x00000002 (HSRBP 0, ICRBP 0)
          second pass                 : 0 rx_ok, 0 rx_error
      FAIL: an error beside a good frame (frame 16 is gone: RXFCG stood with RXPHE (0x80007f03), the pass left 0x00000002 with no RXFCG and the pointers aligned (0,0), and the second pass raised 0 rx_ok and 0 rx_error)
      ok: cleanup
    emulation: dblbuff test FAILED
    emulation: building the dblbuff_longframe test, port/emulation, cc
      ok: aligned at reset
      ok: one frame moves ICRBP
      ok: two frames, two sets
      ok: second frame interrupts
      ok: bad crc keeps buffer
      dw1000-emulation: receiver overrun: both buffers hold a frame the host has not read out
      ok: three frames overrun
      dw1000-emulation: packet lost (receiver not enabled: IDLE)
      ok: single buffered
      ok: dblbuff needs rx_error
      ok: errata RX-1 long send
      ok: frame held across txrx_off
      ok: a queued frame survives rx_start
          status after frame 1        : 0x80006f03
      dw1000-emulation: frame failed in its PHY header on request (RXPHE)
          status after frame 2        : 0x80007f03
          status inside rx_ok, entry  : 0x80007f03
          status inside rx_ok, frame 3: 0x00000b02
          status handed to rx_error   : 0x80007f03
          status after the first pass : 0x00000002 (HSRBP 0, ICRBP 0)
          second pass                 : 0 rx_ok, 0 rx_error
      FAIL: an error beside a good frame (frame 17 is gone: RXFCG stood with RXPHE (0x80007f03), the pass left 0x00000002 with no RXFCG and the pointers aligned (0,0), and the second pass raised 0 rx_ok and 0 rx_error)
      ok: cleanup
    emulation: dblbuff_longframe test FAILED

Case:
  Finding 4 reproduced, and for the reason it predicts. The recorded words,
  decoded (the two builds differ only in the burst numbering, 16 vs 17):

    after frame 1            0x80006f03
                             IRQS | CPLOCK | RXPRD | RXSFDD | LDEDONE |
                             RXPHD | RXDFR | RXFCG | ICRBP
                             HSRBP 0, ICRBP 1: a good frame completed into the
                             host's buffer and not yet reported.

    after frame 2 (the knob) 0x80007f03
                             IRQS | CPLOCK | RXPRD | RXSFDD | LDEDONE |
                             RXPHD | RXPHE | RXDFR | RXFCG | ICRBP
                             RXPHE (1<<12) standing in the same word as the
                             good frame's RXFCG (1<<14), with the pointers
                             still apart. This is the word the finding says
                             cannot be produced against port/emulation; it is
                             the knob's whole purpose.

    entering rx_ok           0x80007f03
                             the snapshot dw1000_process_events() took and
                             handed to the callback: identical, so the pass
                             runs its branches over a word carrying both.

    inside rx_ok, once the   0x00000b02
    model had completed      CPLOCK | RXPRD | RXSFDD | RXPHD
    frame 3                  ICRBP moved 1 -> 0, back onto the host's own
                             pointer, so frame 3 is complete in the *other*
                             buffer; the host's set has had its RXFCG, RXDFR
                             and LDEDONE cleared by the RXFCG branch's
                             ALL_RX_GOOD write, and frame 3's arrival cleared
                             RXPHE from the passthrough bits. Frame 3's own
                             RXFCG becomes visible only after the HRBPT the
                             driver issues when this callback returns -- which
                             is exactly step 3 of the finding's trigger.

    handed to rx_error       0x80007f03
                             the same snapshot: the error branch is acting on
                             a word that predates frame 3 entirely.

    after the first pass     0x00000002  (HSRBP 0, ICRBP 0)
                             CPLOCK alone. No RXFCG, no RXDFR, and the two
                             pointers aligned again. Both halves of the
                             finding, visible: the error branch's
                             `_dw1000_txrx_off(ALL_RX_ERR|ALL_RX_TO|
                             ALL_RX_GOOD)` cleared frame 3's RXFCG and RXDFR
                             although no branch reported them, and its
                             `_dw1000_rx_sync_dblbuff()` -- finding those bits
                             gone, so its "a frame the host has not been told
                             about" guard (dw1000.c:1031) does not hold --
                             issued HRBPT and handed frame 3's buffer back to
                             the chip.

    second pass              0 rx_ok, 0 rx_error
                             Nothing is left to report. Frame 16 (17 in the
                             long frame build) was received by the chip, never
                             given to the host, and never counted: one lost
                             frame, invisible, per pass that carries a receive
                             error beside a good frame.

  The step asserts the correct behaviour -- frame 3 still in SYS_STATUS when
  the pass ends, or an rx_ok for it from the pass after -- so it fails on the
  driver at 7226204, as intended. It is not a driver change and it is not a
  model change beyond the knob: every other step of smoke, timing,
  timing_dblbuff, dblbuff and dblbuff_longframe still passes.

Unblock:
  Nothing blocked. No escalation was needed.

Options:
  Not applicable.

Completed:
  - `dw1000_emulation_fail_next_frame()` in the emulation port, declared in
    port/emulation/dw/osal/include/dw1000/emulation.h and implemented in
    port/emulation/dw/osal/src/osal.c. One-shot, exactly one frame, the next
    delivered after the call, in the style of
    `dw1000_emulation_drop_next_start()`. Like that knob it is not cleared by
    `dw1000_emulation_reset()`.
  - `step_error_beside_a_good_frame` in tests/emulation/dblbuff.c, last in the
    step list before cleanup, in both builds.
  - The patch: knob.patch (git diff from the worktree root, 456 lines, three
    files, nothing under hw/). Not committed; the worktree stays dirty on
    branch hunt/emulation-rx-error.
  - This report.

Undone:
  Nothing.

Concerns (the DONE_WITH_CONCERNS, for the delegator, not defects in the work):
  1. The one thing not directly observed is the instant *between* the driver's
     post-rx_ok HRBPT and the error branch, where the finding says frame 3's
     RXFCG is briefly visible. There is no hook there that does not touch the
     driver (`dw->dbg_pass[2]` records it, but only under DW1000_WITH_DEBUG,
     which this build does not set), so it is inferred rather than recorded:
     the model had frame 3 complete in the other buffer before rx_ok returned
     (ICRBP moved, recorded), and the word after the pass shows that buffer
     both cleared and released. If the delegator wants that word recorded too,
     a DW1000_WITH_DEBUG build of the same step would print it with no further
     change to the test.
  2. The knob raises RXPHE only. The finding names RXRFSL, AFFREJ, LDEERR and
     RXSFDTO as the same class; the fix it proposes covers all of them. A knob
     taking the bit as an argument would cover the class, but the brief asked
     for one knob failing the next frame in the PHY header, so that is what
     this is.
  3. dblbuff.c's config now carries `cb_rx_error_probe` rather than
     `cb_nothing` as `.cb.rx_error`. It is inert unless `err_probe.active`, but
     it is a shared object touched by one step, the same way `cb_rx_probe`
     already is.
