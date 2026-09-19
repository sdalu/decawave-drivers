# 2026-09-19: the event pass restructured, checked on the bench

The driver at `10e459d` ("head") against the same tree with
`dw1000_process_events()` restructured (the completion booked first,
one receiver reconcile, one receive-intent field; "new"), on rpi-c and
rpi-d through ruby-dw1000's engine, the bench of `DW1000.md`, "The
bench, for scale".

- `pair-head.log`, `pair-new.log`: `DW1000_MIN_DELIVERY=0.9 rake
  test:pair`, one run each, 9 runs and 78 assertions, 0 failures both.
- `soak-head.jsonl`, `soak-new.jsonl`: 60 duplex runs each (12, then
  24, then 24, alternating drivers), one JSON line per node per run,
  all at the end-of-pass receiver enable (`soak-end.rb`, the
  `soak.rb` of the 2026-09-18 hunt with the placement alternation
  removed). Totals: head 3892 of 6000 received, 1 duplicate; new 4336
  of 6000, 2 duplicates. The losses are the harness's bursts colliding
  (INVESTIGATE.md, "The bench's own numbers"), not the driver's.
- `traces/`: the three runs that carry a duplicate, both nodes' logs,
  `DW1000_DUPLEX_TRACE=1`: `new-run-006` (rpi-c, seq 17),
  `new-run-019` (rpi-d, seq 45), `head-run-010` (rpi-c, seq 26).
  INVESTIGATE.md entry 6 is what they show.

The driver reached the nodes through `DECAWAVE_DRIVERS_DIR`, the
variable ruby-dw1000's `ext/vendoring.rb` reads, exported by hand for
these runs. `soak.rb` and `bench-errbranch.rb` used to set
`DW1000_DRIVERS_DIR`, which nothing reads; both, and `soak-end.rb`
here, were corrected on 2026-09-19.
