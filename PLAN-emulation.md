# Plan: move the DW1000 emulation out of spank into this driver

The register-level DW1000 emulation that spank uses for its simulation
is a DW1000 OSAL port, and nothing in it is about spank. Living here it
becomes a port like the others, `make check` compiles it, and the driver
gets its first way to run without a chip. This plan moves the C half,
leaves the Ruby medium server and the spank node in spank, and pins the
socket protocol between them as the one contract.

- **Moves**: `spank/port/unix/dw1000-rsvc/` (the emulation OSAL, 1 530
  lines) and `spank/port/unix/rsvc/` (the socket client it needs, 726
  lines).
- **Stays in spank**: `simulation/main.c` (the spank node),
  `simulation/lib/spank/simulator*.rb` (the medium server) and the
  launch scripts.
- **Not in scope**: extending the model. Ten registers are attached as
  `TODO`, the double-buffer swing set is marked incomplete, and RXPTO,
  RXSFDTO, RXRFTO and delayed send are not emulated. That is follow-up
  work, listed at the end.

## 1. Layout in this tree

Follow the existing shape, one directory per port, one include
directory, so `dw1000.cmake` and the Makefile need no new mechanism:

```text
port/emulation/dw/osal/include/dw1000/osal.h      the port contract
port/emulation/dw/osal/include/dw1000/emulation.h public: create, reset,
                                                  the wire packet, the
                                                  DW1000_RSVC_* types
port/emulation/dw/osal/include/rsvc.h             socket client API
port/emulation/dw/osal/src/osal.c                 register model, SPI,
                                                  IRQ line, TX/RX
port/emulation/dw/osal/src/emulation.h            private macros
port/emulation/dw/osal/src/rsvc.c                 socket client
```

`rsvc.h` sits in the same include directory because the Makefile passes
a single `-I$(OSAL_INC)`. The wire packet (`struct dw1000_driver_iopkt`)
and the `DW1000_RSVC_RX`/`TX`/`TX_DONE`/`RX_CONFIG` constants currently
live inside `osal.c`; they move to the public `emulation.h` so that a
server implementer and the port share one definition.

## 2. Remove the spank helpers

Every spank symbol the two directories use, with its replacement:

| Symbol | Uses | Replacement |
| :----- | ---: | :---------- |
| `SPANK_DEBUG`, `SPANK_WARNING`, `SPANK_SYSLOG_PREFIX`, `SPANK_NO_SYSLOG` | 24 | `EMU_DEBUG(...)` to stderr, compiled in under `DW1000_EMULATION_DEBUG`, in a new `src/emu_log.h` |
| `SPANK_FATAL`, `SPANK_UNIMPLEMENTED` | 11 | `EMU_FATAL(...)`: message to stderr, then `abort()` |
| `SPANK_ASSERT` | 15 | `DW1000_ASSERT(x, "...")`, the port's own macro |
| `spank_mutex_t`, `_init`, `_acquire`, `_release`, `SPANK_SYSTICKS_INFINITE` | 17 | `pthread_mutex_t` and `pthread_mutex_lock/unlock` |
| `spank_crc16_ccitt` | 2 | copy the routine into `osal.c` as a static; it is the 802.15.4 FCS and the driver has no CRC of its own |
| `spank_popcount32` | 1 | `(sys_status & sys_mask) != 0` |
| `spank_cksum16`, `spank_printf`, `spank_output_lock/unlock` | 6 | debug-only; delete with the `#if 0` block, or print with `fprintf(stderr, ...)` |
| `SPANK_MIN`, `SPANK_MAX` | 4 | local macros |
| `#include "spank/osal.h"` in `osal.h` | 1 | delete; nothing in the header uses it once the mutex type is in `osal.c` |
| `RSVC_WITH_SPANK_SYSLOG` in `rsvc.c` | 1 | `RSVC_DEBUG` to stderr under `RSVC_WITH_DEBUG`, which `build.sh` already defines |

Two portability items surface at the same time:

- `rsvc.c` includes `<sys/tree.h>` and `<pthread_np.h>`, which glibc
  does not ship. The red-black tree holds the registered handlers, and
  exactly one is ever registered (RSVC_UWB_IO). Replace it with a short
  array and drop both headers, so the port builds on Linux too.
- `emulation.h` defines a private `E_DEFINE_PASSTHROUGH` that names
  `reg_p_SYS_STATUS` regardless of its argument. Harmless today, but
  worth fixing while the file is open.

Keep the file header's `TODO` on the swing set; it is true.

## 3. Register the port

- `dw1000.cmake`: add `emulation` to `DW1000_OSAL_PORTS`, then
  `DW1000_OSAL_EMULATION_INCLUDE_DIR` and `DW1000_OSAL_EMULATION_SOURCES`
  listing `osal.c` and `rsvc.c`. `tests/check-manifest.sh` then checks
  the tree matches.
- Makefile: nothing to add; `make lib OSAL=emulation` works from the
  manifest. Add `emulation` to what `make check` compiles, next to the
  option matrix over `port/null`, since this port has no vendor tree to
  wait for and should never rot silently the way
  `DW1000_WITH_EXTENDED_SEND=0` did.
- `README.md`: a row in "Supported hosts": emulation, `port/emulation`,
  "no chip: a register model talking to a medium server over a unix
  socket". A paragraph under "Writing a port" is not needed; the port
  is a consumer of the contract, not a template.
- `docs/emulation.md`: the wire protocol in one page. The three
  RSVC service types (OPEN, CLOSE, SEED_GET), the UWB channel
  (UWB_OPEN, UWB_CLOSE, UWB_IO) and the four UWB messages with their
  fields, the interrupt flag on server-to-node frames, and the timestamp
  convention (the server returns RX and TX times already adjusted for
  the antenna delay it was given; the port derives RX_RAWST and
  TX_RAWST by rounding to the 512-tick grid). Say plainly what the model
  does not do. This page is what the Ruby server is written against; it
  replaces the "see port/unix/hal/io/dw1000/src/driver.c" comment in
  `simulator.rb`, which points at a file that no longer exists.
- Version: bump to 1.3.0. A new port is a feature, and spank will pin
  on it.

## 4. Update spank

- Delete `port/unix/dw1000-rsvc/` and `port/unix/rsvc/`; remove
  `unix-rsvc` and `unix-dw1000-rsvc` from the module list and the two
  blocks in `spank.cmake`.
- `simulation/build.sh`: replace the two `port/unix/...` include and
  source lines with `${DW1000}/port/emulation/dw/osal/include` and
  `${DW1000}/port/emulation/dw/osal/src/*.c`. The `DW1000` variable
  already points at a checkout of this driver. Drop
  `-DRSVC_WITH_SPANK_SYSLOG`.
- `simulation/main.c`: includes stay (`dw1000/osal.h`, `rsvc.h`); add
  `dw1000/emulation.h` for `dw1000_emulation_create`. The
  `.emulation` fields on the ioline and SPI structs are unchanged.
- `simulation/lib/spank/simulator.rb`: point the protocol comment at
  `decawave-drivers/docs/emulation.md`.
- `port/README.md` and the "Unix, only for simulation" row: say the
  emulation now comes from decawave-drivers ≥ 1.3.0.
- `spank.0` and `rpi-redskin/spank` carry the same two directories
  (`build.sh` is the only file that differs). They are copies, not
  forks; nothing to merge, but note it in the commit so nobody goes
  looking for a lost change.

The 17 files spank tracks under `simulation/` do not include the vendored
gems, the logs or `a.out`, so nothing there needs untracking.

## 5. Verify

Gates, cheapest first:

1. `make check` here: manifest, the 256-configuration matrix, and the
   emulation port compiling with `-Wall -Wextra`, on FreeBSD and on
   rpi-a (Linux), which is where the `<sys/tree.h>` removal is proven.
2. `cppcheck` and `clang-tidy` on rpi-a over the moved files, with the
   defines recorded in `AUDIT.md`.
3. spank's `simulation/build.sh`, then `simulation/test.sh`, the
   two-node ranging run, with a fixed seed. The medium server logs
   every frame; a diff of that log before and after the move, modulo
   wall-clock timestamps, is the behavioural check. Do the "before" run
   first, on the current tree, and keep its log.
4. ruby-dw1000 is untouched by the move and needs no run.

## 6. Order of work

1. Before-run of the simulation on the current spank tree; keep the log.
2. Copy the two directories in, restructure per §1, strip helpers per
   §2, fix the two portability items. One commit: "port/emulation: the
   DW1000 register model from spank, without spank".
3. Register, README, docs page, version bump. Second commit.
4. spank: delete, re-point, comment fix. Third commit, in spank.
5. Gates 1 to 3.

About a day, most of it in §2 and the docs page.

## 7. Follow-ups this move makes possible

- Done: `tests/emulation/smoke.c`, run by `make check`. It covers
  bring-up, configure, TX, RX, a bad FCS and a ranging frame, and is
  the place to turn several `AUDIT.md` findings into failing tests.
- ruby-dw1000 could build against `port/emulation` for an offline
  suite.
- Model gaps, in the order the audit findings need them: DX_TIME with
  TXDLYS and HPDWARN, RX_FWTO with RXRFTO, the double-buffer swing set
  with HSRBP/ICRBP/HRBPT, RXPTO and RXSFDTO.
