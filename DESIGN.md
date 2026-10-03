# Design of the DW1000 driver

Why the driver is shaped the way it is, what the port contract asks of a
new host, and how the build and the checks fit together. For using the
driver, read [`README.md`](README.md); for the traps an application
falls into, [`hw/drivers/dw1000/README.md`](hw/drivers/dw1000/README.md);
for what the core does inside,
[`hw/drivers/dw1000/DESIGN.md`](hw/drivers/dw1000/DESIGN.md).

## One core, one port, and nothing between them

The driver is split so that the part which knows about the chip knows
nothing about the system it runs on. The core holds the register map,
the tuning tables and the transmit and receive sequencing, and contains
no RTOS call and no vendor header. Everything host-specific sits behind
`<dw1000/osal.h>`, a contract small enough to state in a table, with one
implementation per supported system.

The reason is that the chip facts are the expensive part. A register
sequence, a tuning table indexed by channel and PRF, the order in which
the LDE microcode has to be loaded: each of those cost a reading of the
User Manual and, often, a measurement. They are worth writing once. What
a host contributes is an SPI transfer, a GPIO write and a delay, which
is worth writing per host and nothing more.

Two consequences that shape the code. The core never allocates and never
blocks on anything but a delay, because it cannot know what either would
mean on the host. And every register access carries a comment naming the
User Manual section or table that prescribes it, so that a disagreement
between the code and the chip can be settled by reading rather than by
experiment. [`AUDIT.md`](AUDIT.md) is the record of that reading.

The names come from the manual too. A register, a sub-register offset, a
field or a bit is spelled the way the User Manual spells it, and is
never adjusted to fit a convention of this driver's: `DIS_DRXB` stays
`DIS_DRXB` even though the configuration field that drives it is called
`dblbuff`, and stays it even though the bit is inverted (setting it
turns double buffering off). That is what makes `dw1000_reg.h` checkable
against §7.2 definition by definition, which is how it was audited, and
it is why grep on a name out of the manual finds the code that
implements it. The house conventions apply to everything the driver
names itself, which is where the boundary falls: the `_dw1000_` prefix
for internals, and one spelling per concept for the rest.

## The tree

```text
decawave-drivers
├── hw/drivers/dw1000              the portable core
│   ├── include/dw1000
│   │   ├── dw1000.h               driver API, config and radio structs
│   │   ├── dw1000_send.h          frame transmission helpers
│   │   ├── dw1000_validate.h      human radio values to radio fields
│   │   ├── dw1000_state.h         read the radio back off the chip
│   │   ├── dw1000_reg.h           register, field and flag definitions
│   │   ├── dw1000_otp.h           OTP memory map
│   │   ├── dw1000_bswap.h         endian helpers
│   │   └── dw1000_version.h       the release, and nothing else
│   ├── src                        dw1000.c, dw1000_send.c,
│   │                              dw1000_validate.c, and dw1000_state.c
│   │                              which is the one optional source
│   ├── README.md                  the pitfalls guide, for applications
│   └── DESIGN.md                  how the core works inside
│
├── port                           one OSAL per host, pick one
│   ├── unix                       through the bitters library
│   ├── zephyr                     Zephyr 2 and later
│   ├── chibios                    ChibiOS, SPI API v1 or v2
│   ├── mynewt                     Apache MyNewt
│   ├── cf2                        Crazyflie 2.x, on FreeRTOS
│   ├── emulation                  a register model, no chip
│   └── null                       no hardware, for compile checks
│
├── probe                          a two-way-ranging instrument, built
│                                  on the driver rather than part of it
├── sniffer                        forwards captured frames over ethernet,
│                                  or writes them as pcapng
│
├── dw1000.cmake                   the file list, read by both of these
├── Makefile                       compile checks, vendoring, doxygen
├── scripts                        reads dw1000.cmake for the Makefile
├── tests                          what `make check` runs
├── zephyr                         Kconfig and CMakeLists, as a module
├── docs                           the vendor shelf: manuals, errata
└── doc                            generated doxygen output, and bench/
```

`probe/` and `sniffer/` are applications that consume the driver, not
parts of it. They live here because they are built against this driver
and nothing else, and because a measurement is interpretable only if the
driver that produced it is known; co-located, that pairing cannot be
forged.

## Writing a port

An OSAL is a header and a source file. The header defines the types and
the macros, the source implements the SPI transfers. The whole contract:

| Symbol                               | Purpose                      |
| ------------------------------------ | ---------------------------- |
| `dw1000_ioline_t`                    | a GPIO line                  |
| `DW1000_IOLINE_NONE`                 | sentinel for "not wired"     |
| `_dw1000_ioline_set/clear()`         | drive it high or low         |
| `_dw1000_delay_usec/msec()`          | busy wait, or sleep          |
| `DW1000_ASSERT(cond, reason)`        | programming-error trap       |
| `dw1000_spi_driver_t`                | bus handle, plus `int error` |
| `_dw1000_spi_send/recv()`            | header then data, CS around  |
| `_dw1000_spi_low_speed/high_speed()` | probe slowly, then run fast  |

Copy `port/null`, which implements all of it wired to nothing, and fill
it in. Three rules the existing ports learned the hard way.

**A failed read must zero the caller's buffer.** The core's register
helpers return an uninitialised local whatever happens, so a buffer left
untouched is read back as register content.

**The first failure is latched in `spi->error` and left for the caller
to clear.** The core has no error path of its own to carry it, so a port
that clears the field itself loses the only report there is.

**`DW1000_ASSERT` is not validation, and the port decides whether it
survives.** Six of the seven ports map it to something that vanishes in
a release build: `assert()` under `NDEBUG`, `__ASSERT` without
`CONFIG_ASSERT`, `osalDbgAssert` without `CH_DBG_ENABLE_ASSERTS`. Only
`port/cf2` traps unconditionally, with a breakpoint. Anything the driver
must reject in a shipping build is therefore checked separately and
reported through a return value; the asserts catch programming errors
during development and are not a safety net.

The two speeds are a chip requirement, not a convenience: the DW1000
must be probed below 3 MHz until its PLL is up, and run fast afterwards.
`dw1000_initialise()` calls both.

## Why the options are compile-time

The seven `DW1000_WITH_*` options change the public headers. Two add
fields to `dw1000_config_t`, one adds entry points to
`<dw1000/dw1000_send.h>`, and one changes `DW1000_FRAME_MAXSIZE` from
127 to 1023 and with it every buffer sized from it. None of that can be
decided after the driver is compiled, and none of it can differ between
the driver and its caller without the two disagreeing about a struct
layout.

That is also why they are options at all rather than always-on. The
proprietary modes are outside 802.15.4, the long-frame mode quadruples
the frame buffers, and the AAT hotfix is a workaround whose cost is a
register read per receive. An MCU with 32 KB of RAM should not pay for
features it does not use, and a deployment that must stay standard
should not be able to reach the ones that leave the standard.

The consequence for whoever vendors the driver is the one warning worth
repeating: define the set in one place the whole build sees. Nothing in
this tree selects any of them, which is deliberate (the consumer
decides), and it is why `make check` exists.

## Why the delayed-send lead is a run-time argument

It used to be a compile-time option, and it cannot be, because it has
two parts and no single build-time number can carry both.

The first part is the airtime of the preamble and SFD. What a delayed
send programs is the RMARKER, the *end* of the SFD (APS022 §5.4), so the
chip has to be transmitting well before it: about 138 µs at a 128 symbol
preamble, 1.05 ms at 1024, and 4.2 ms at 4096. That depends on the radio
configuration, so the driver computes it at `dw1000_configure()` and
`dw1000_tx_get_preamble_airtime()` reports it.

The second part is the host's own latency between reading the system
time and writing `TXSTRT`, which only the host knows.

Both parts are the caller's to add up. A lead always means the lead to
the RMARKER and is programmed as given, whether it is a default set once
through `dw1000_tx_set_default_delay()` or a `DW1000_TX_DELAYED_DELAY`
passed with one send. Until a default is set there is none, and every
delayed send has to carry its own. A lead at or below the airtime cannot
be met and is refused, so a default left over from a shorter preamble
shows up at the next send rather than as a lost frame.

For scale, and as a warning: the lead figures reported by the
estimators in the surrounding tooling are **totals**, airtime included,
not the host term. At a 128 symbol preamble (138 µs of airtime) an
nRF52 at 8 or 16 MHz SPI needs about 0.27 ms and a Raspberry Pi 4 over
spidev at 20 MHz about 0.16 ms, which puts the host term alone at
roughly 130 µs and 30 µs. Read such a figure as a lead for that
preamble, never as something to add the airtime to.

Transmit power is left out of the options for a related reason: it is
part of `dw1000_radio_t` and `DW1000_TX_POWER(dB)` names it outright, so
a board whose RF path differs from the reference one is corrected where
the radio is configured rather than by a build switch. The usual case is
a DWM1000 module driven by software written for the EVB1000 evaluation
board, which wants roughly 3 dB above the calibrated default for its
channel and PRF; those defaults are `manual_tx_power[]` in `dw1000.c`,
encoded as UM §7.2.31.4 describes.

## Why vendored, not installed

There is no `make install` and no `pkg-config` file. Since the options
change the public headers, a shared library could not carry them: two
consumers of one `libdw1000.so` with different option sets would
disagree about `dw1000_config_t`, and nothing would say so. A
`pkg-config` file naming one option set would be a trap of the same
shape.

So the driver is compiled into its consumer, once per project, against
whichever OSAL is on the include path, and the top-level `Makefile` does
the other half of the job: it answers what to compile and with what
flags. `make lib` builds `libdw1000.a` locally, which is useful for a
quick check and is not how a project should consume the driver.

## One file list

`dw1000.cmake` is the single place the file list lives. A CMake consumer
includes it directly; the `Makefile` reads it through
`scripts/manifest.sh`, which parses the `set()` calls. Adding a source
or a port means editing that file and nothing else.

It defines variables rather than targets, for the reason above: a target
would bake in one option set and hide it from the consumer. It also
parses the three numbers out of `dw1000_version.h` into `DW1000_VERSION`,
so the version cannot be written twice.

`tests/check-manifest.sh` fails if either side starts keeping a list of
its own again, which is the failure this arrangement exists to prevent
and the one that would otherwise be found months later by a build that
silently stopped compiling a file.

## The checks

`make check` is preflight: it runs none of the suite below, and needs
nothing built.

| Check            | Proves                                  |
| ---------------- | ---------------------------------------- |
| `check-options`  | every option combination still compiles |
| `check-manifest` | `dw1000.cmake` still describes the tree |

None of them needs hardware, which is the point: a check that needs a
DW1000 on a bench is a check that does not run.

`check-options` compiles the core over all 128 combinations of the seven
boolean options, against `port/null` and with `-fsyntax-only`. Nothing
in this tree selects any of them (whoever vendors the driver does),
so without this an option that stopped compiling would be found by the
one consumer who wanted it, which is how `DW1000_WITH_EXTENDED_SEND=0`
came to spend an unknown length of time broken. It also names
`dw1000_state.c` explicitly, because that source is absent from
`DW1000_SOURCES` and nothing else would ever compile it.

It takes about a minute and prints a row of dots so that it is visibly
working. One broken option breaks half the matrix, so failures are
grouped by the errors they produced and reported once per group, with
the options every member of the group shares, which is the option to
go and look at:

```text
  FAILED  64 combinations, all with these errors.
          What they have in common: DW1000_WITH_EXTENDED_SEND=0
```

`make check WERROR=yes` is the CI form.

## The suite

`make tests` builds and runs what `check` does not: four targets, each
its own program built and actually run, none needing hardware.

| Test              | Proves                                                  |
| ----------------- | -------------------------------------------------------- |
| `tests-validate`  | the radio-value validation matches `dw1000_configure()` |
| `tests-emulation` | the driver runs against a model of the chip             |
| `tests-probe`     | the probe's record format, settle rule and roles        |
| `tests-sniffer`   | the sniffer's host-buildable logic                      |

`tests-emulation` needs `port/emulation`, which is the one port requiring
nothing installed and no server: it is what lets this be run rather than
only compiled. `tests-probe` and `tests-sniffer` build against the same
port for the same reason.

`make tests WERROR=yes` is the CI form, same as `check`.

## The version scheme

The release is written in exactly one place, `dw1000_version.h`, because
a C header can read no other file: a consumer must be able to have the
version without running anything. Everything else parses those three
lines (`dw1000.cmake` for `DW1000_VERSION`, `scripts/manifest.sh` for
the `Makefile`), so nothing can disagree with the header.

The git part reaches the code only through `-DDW1000_VERSION_GIT`, which
`make lib` passes and `make sources` hands over for a vendoring build to
pass on. `scripts/gitversion.sh` works it out, and it deliberately
answers nothing when the answer would be somebody else's: a tarball has
no repository, and a tree copied into another project's repository would
otherwise report that project's tags and dirt as the driver's.

Bumping a release is editing the three numbers, committing, and
`make tag`, which reads the number out of the header rather than having
it typed again:

```sh
$EDITOR hw/drivers/dw1000/include/dw1000/dw1000_version.h
git commit -am 'Bump the version to 1.1.1.'
make tag                             # v1.1.1, from the header
git push origin v1.1.1
```

`make tag` refuses an unclean worktree or an existing tag, then asks,
then runs the whole check suite before it tags, so declining costs
nothing and nothing gets tagged that the suite has not passed. `YES=1`
answers yes for a script, and a non-interactive run without it declines,
which is the safe way round. It pushes nothing.

A tag made by hand can still disagree, so `scripts/checktag.sh` gates
the other direction from inside `make check`. On a tag with a clean
worktree, a release build, which has no later chance to be wrong,
the header must say what the tag says; anywhere else it must be at or
ahead of the nearest tag. Bumped-but-not-yet-tagged is the normal state
between releases and passes; behind a tag that exists does not. It says
nothing at all where the answer would be somebody else's.
