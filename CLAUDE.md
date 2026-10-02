# decawave-drivers

A portable driver for the DW1000 ultra-wideband transceiver, plus two
applications that consume it: a ranging probe and a UWB sniffer.

## Where things are

- `README.md` — what it does, how to build it, how to vendor it, the
  compile-time options, the API
- `DESIGN.md` — why it is shaped this way, the checks and the suite
- `AUDIT.md` — the audit against the DW1000 User Manual and Errata
- `DW1000.md` — what the chip itself has been found to do
- `INVESTIGATE.md` — open questions about the chip and the bench
- `PROPAGATE.md` — what a bench finding still owes other trees
- `probe/README.md`, `probe/DESIGN.md` — the ranging probe
- `sniffer/README.md`, `sniffer/DESIGN.md` — the UWB sniffer
- `port/emulation/README.md` — the chip model the suite runs against
- `dw1000.cmake` — the source list. It is **not** in the Makefile; a new
  `src/*.c` is added there, and `scripts/manifest.sh` is how make reads it
- `hw/drivers/dw1000/include/dw1000/dw1000_version.h` — the release, and
  the only file that holds it

## Gate

Gate: `make check && make tests` — `check` is preflight (the option
matrix and the manifest) and runs none of the project's code; `tests`
builds `tests-validate`, `tests-emulation`, `tests-probe` and
`tests-sniffer` and runs them. `make check WERROR=yes
&& make tests WERROR=yes` is the CI form (`DESIGN.md`).

## Traps

- **`make tests` needs no chip and no Pi.** It runs entirely against
  `port/emulation`, which needs nothing installed. Only the probe and
  sniffer *applications themselves* (`probe/app/unix`,
  `sniffer/app/unix`) want bitters and a Raspberry Pi's Linux; a green
  `make tests` says nothing about those builds.
- **`tests-validate` is cheap but it is the suite**: it compiles and runs
  the driver's own `dw1000_validate.c`, so it moved from `check` to
  `tests` in the 2026-10-02 split, cheap as it is. `check` runs none of
  the driver's code.
- **`make tag` now runs both `check` and `tests` before tagging**, not
  only `check` — the split moved the emulation, probe and sniffer runs
  out of `check`, and the full pre-tag suite still has to cover them.
- **There is no `make install`.** The compile-time options change the
  public headers, so the driver is vendored into each consumer instead
  (`make sources`, `dw1000.cmake`); see the note atop the `Makefile`.
