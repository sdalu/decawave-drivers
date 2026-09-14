Decawave DW1000 driver
======================

A driver for the [DW1000][6] ultra-wideband transceiver, written so that
the part of it that knows about the chip knows nothing about the system
it runs on. The register map, the tuning tables and the transmit and
receive sequencing live in one portable core; everything host-specific
is a small OS abstraction layer (OSAL) behind a fixed contract, and
there is one of those per supported system.

Register accesses are annotated with the section or table of the
[DW1000 User Manual][7] that prescribes them, so a reader can check the
code against the source it came from.


Architecture
============

```text
        application
   ┌─────────────────────────────────────────────────────────────┐
   │  dw1000_configure()   dw1000_tx_send()   dw1000_rx_start()  │
   └──────────────────────────────┬──────────────────────────────┘
                                  │  <dw1000/dw1000.h>
   ┌──────────────────────────────┴──────────────────────────────┐
   │  driver core                              hw/drivers/dw1000 │
   │                                                             │
   │  register map, tuning tables, TX/RX sequencing.             │
   │  No RTOS call and no vendor header anywhere in it.          │
   └──────────────────────────────┬──────────────────────────────┘
                                  │  <dw1000/osal.h>  (port contract)
   ┌──────────────────────────────┴──────────────────────────────┐
   │  OSAL port                port/{unix,zephyr,chibios,...}    │
   │                                                             │
   │  SPI transfer, GPIO line, delay, assert.                    │
   └──────────────────────────────┬──────────────────────────────┘
                                  │
   ┌──────────────────────────────┴──────────────────────────────┐
   │  RTOS and vendor HAL                                        │
   └──────────────────────────────┬──────────────────────────────┘
                                  │  SPI   IRQ   RSTn   WAKEUP
                           ┌──────┴──────┐
                           │   DW1000    │
                           └─────────────┘
```

The core is compiled once per project, against whichever OSAL is on the
include path. Swapping hosts means swapping `port/`, not touching
`hw/`.


Supported hosts
===============

| Host                      | OSAL          | Notes                    |
| ------------------------- | ------------- | ------------------------ |
| [Zephyr][2] 2 and later   | `port/zephyr` | also a Zephyr module     |
| [Crazyflie][5] 2.x        | `port/cf2`    | FreeRTOS, deck SPI       |
| Unix                      | `port/unix`   | via the [bitters][1] lib |
| [ChibiOS][3]              | `port/chibios`| SPI API v1 or v2         |
| [Apache MyNewt][4]        | `port/mynewt` | via `hal_spi_txrx()`     |
| none                      | `port/null`   | no hardware; see below   |

ChibiOS and MyNewt are built against their vendor headers but have not
been exercised on hardware recently.

`port/null` is not a host. It implements the whole port contract wired to
nothing, so the core can be compiled where there is no DW1000 and no
vendor tree -- which is what `make check` does. It is also the shortest
thing to copy when writing a port of your own.


Layout
======

```text
decawave-drivers
├── hw/drivers/dw1000              the portable core
│   ├── include/dw1000
│   │   ├── dw1000.h               driver API, config and radio structs
│   │   ├── dw1000_send.h          frame transmission helpers
│   │   ├── dw1000_reg.h           register, field and flag definitions
│   │   ├── dw1000_otp.h           OTP memory map
│   │   └── dw1000_bswap.h         endian helpers
│   └── src                        dw1000.c, dw1000_send.c
│
├── port                           one OSAL per host, pick one
│   ├── unix                       through the bitters library
│   ├── zephyr                     Zephyr 2 and later
│   ├── chibios                    ChibiOS, SPI API v1 or v2
│   ├── mynewt                     Apache MyNewt
│   ├── cf2                        Crazyflie 2.x, on FreeRTOS
│   └── null                       no hardware, for compile checks
│
├── dw1000.cmake                   the file list, read by both of these
├── Makefile                       compile checks, vendoring, doxygen
├── scripts                        reads dw1000.cmake for the Makefile
├── tests                          what `make check` runs
├── zephyr                         Kconfig and CMakeLists, as a module
└── doc                            generated doxygen output
```


Bringing a device up
====================

```text
   call                       does                    returns
   ────────────────────────   ─────────────────────   ─────────────────

   dw1000_init()              binds the config        void
        │
        ▾
   dw1000_hardreset()         pulses RSTn, if wired   void
        │
        ▾
   dw1000_initialise()        probes the device ID,   0, or -1 when the
        │                     loads the LDE code,     device ID is not
        │                     reads OTP calibration   a DW1000
        ▾
   dw1000_configure()         channel, PRF, preamble  0, or -1 when a
        │                     code, bitrate, power    setting is out of
        │                     and every tuning table  range
        ▾
   dw1000_rx_start()   or   dw1000_tx_send()
```

`dw1000_initialise()` and `dw1000_configure()` both report failure and
both leave the chip untouched when they do, so a caller that checks
them never proceeds on a half-configured radio. Most of the radio
configuration indexes a tuning table, so an out-of-range value would be
an out-of-range read rather than merely a wrong setting; the check that
catches it runs in every build, not only where assertions are enabled.


Handling events
===============

Register a callback for what you care about, then call
`dw1000_process_events()` from the interrupt handler or from a thread:

```text
        IRQ line, or a poll of dw1000_pending_interrupt()
                          │
                          ▾
                 dw1000_process_events()
                          │
     ┌────────────────┬───┴────────────┬────────────────┐
     ▾                ▾                ▾                ▾
     cb.rx_ok         cb.rx_error      cb.rx_timeout    cb.tx_done

     frame is         CRC, PHY or      nothing heard    transmission
     ready to         SFD error, or    before the       finished
     be read          frame rejected   timeout
```

In double buffered mode the receiver is re-enabled before the frame is
read out, so `cb.rx_ok` must consume the frame before returning and must
not re-enable the receiver itself.


Writing a port
==============

An OSAL is a header and a source file. The header defines the types and
the macros, the source implements the SPI transfers:

| Symbol                               | Purpose                       |
| ------------------------------------ | ----------------------------- |
| `dw1000_ioline_t`                    | a GPIO line                   |
| `DW1000_IOLINE_NONE`                 | sentinel for "not wired"      |
| `_dw1000_ioline_set/clear()`         | drive it high or low          |
| `_dw1000_delay_usec/msec()`          | busy wait, or sleep           |
| `DW1000_ASSERT(cond, reason)`        | programming-error trap        |
| `dw1000_spi_driver_t`                | bus handle, plus `int error`  |
| `_dw1000_spi_send/recv()`            | header then data, CS around   |
| `_dw1000_spi_low_speed/high_speed()` | probe slowly, then run fast   |

Two rules the existing ports learned the hard way. A failed read must
zero the caller's buffer: the core's register helpers return an
uninitialised local whatever happens, so a buffer left untouched is read
back as register content. And the first failure is latched in
`spi->error` and left for the caller to clear, because the core has no
error path of its own to carry it.


Compile-time options
====================

Undefined takes the default in the table, which is not always off. Under
Zephyr they are `CONFIG_DW1000_*` in `Kconfig` and under MyNewt they are
`syscfg` settings; elsewhere define them on the compiler command line.
`make options` prints the table below.

| Option                                     | Default | Effect         |
| ------------------------------------------ | ------- | -------------- |
| `DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH`  | 1 | preamble lengths outside the standard set |
| `DW1000_WITH_PROPRIETARY_SFD`              | 1 | the Decawave non-standard SFD |
| `DW1000_WITH_PROPRIETARY_LONG_FRAME`       | 0 | frames up to 1023 bytes, not 127 |
| `DW1000_WITH_EXTENDED_SEND`                | 1 | delayed send with an embedded timestamp |
| `DW1000_WITH_SFD_TIMEOUT`                  | 0 | caller-chosen SFD timeout |
| `DW1000_WITH_SFD_TIMEOUT_DEFAULT`          | 0 | a fixed SFD timeout, not a computed one |
| `DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011` | 1 | work around a spurious AAT on receive |
| `DW1000_WITH_DWM1000_EVK_COMPATIBILITY`    | 0 | +3 dB, DWM1000 under EVB1000 software |

Three more take a value rather than a flag:
`DW1000_SFD_TIMEOUT_DEFAULT` (default `DW1000_SFD_TIMEOUT_MAX`),
`DW1000_TX_DELAYED_DEFAULT_DELAY` (400 us, in `DW1000_TIME_CLOCK_HZ` steps;
an nRF52 measured a need of 0.27 ms, a Raspberry Pi 4 over spidev 0.16 ms)
and `DW1000_TX_DELAYED_DEFAULT_RETRY_DELAY` (1.5 x the delay, 600 us).

**These are not internal to the driver.** `DW1000_WITH_SFD_TIMEOUT` and
`DW1000_WITH_PROPRIETARY_SFD` add fields to `dw1000_config_t`, and
`DW1000_WITH_EXTENDED_SEND` adds entry points to `<dw1000/dw1000_send.h>`,
so the same set has to reach the driver and every translation unit that
includes `<dw1000/dw1000.h>`. Define them in one place your whole build
sees, not per file.


Building
========

The driver is vendored into a project rather than installed: since the
options above change the public headers, a shared library and a
`pkg-config` file that could not carry them would be a trap. So there is
no `make install`, and the top-level `Makefile` does the other half of
the job. It works with both GNU make and BSD make.

```text
make                print the targets; nothing is built by default
make check          compile the option matrix, and check the manifest
make sources        print the files and flags to vendor, as shell variables
make lib            build libdw1000.a locally, against OSAL=<port>
make options        print the option table above
make doc            run doxygen
```

`make check` compiles the core over all 256 combinations of the eight
boolean options, against `port/null`. Nothing in this tree selects any
of them -- whoever vendors the driver does -- so without this an option
that stopped compiling would be found by the one consumer who wanted it,
which is how `DW1000_WITH_EXTENDED_SEND=0` came to spend an unknown
length of time broken. `make check WERROR=yes` is the CI form.

It takes about a minute, and prints a row of dots as it goes so that it
is visibly working. One broken option breaks half the matrix, so failures
are grouped by the errors they produced and reported once per group, with
the options every member of the group shares -- which is the option to go
and look at:

```text
  FAILED  128 combinations, all with these errors.
          What they have in common: DW1000_WITH_EXTENDED_SEND=0
```

To vendor from a shell-driven build:

```sh
eval "$(make -s -C 3rd/decawave-drivers sources OSAL=unix)"
cc $DW1000_CFLAGS -DDW1000_WITH_PROPRIETARY_LONG_FRAME=1 \
   -c $DW1000_SOURCES $DW1000_OSAL_SOURCES
```

That answer comes out of `dw1000.cmake`, which is the one place the file
list lives. A CMake consumer includes it; the Makefile reads it through
`scripts/manifest.sh`. Adding a source or a port means editing that file
and nothing else, and `make check` fails if either side starts keeping a
list of its own again.

From CMake it is used directly. It defines variables rather than targets,
for the reason above -- a target would bake in one option set and hide it
from you:

```cmake
include(3rd/decawave-drivers/dw1000.cmake)
add_library(dw1000 INTERFACE)
target_include_directories(dw1000 INTERFACE
    ${DW1000_INCLUDE_DIR} ${DW1000_OSAL_UNIX_INCLUDE_DIR})
target_sources(dw1000 INTERFACE
    ${DW1000_SOURCES} ${DW1000_OSAL_UNIX_SOURCES})
target_compile_definitions(dw1000 INTERFACE
    DW1000_WITH_PROPRIETARY_LONG_FRAME=1)
target_link_libraries(ranger PRIVATE dw1000 bitters m)
```

`DW1000_SOURCES_CORE` and `DW1000_SOURCES_SEND` are separable --
`dw1000.c` never calls into `dw1000_send.c` -- so an application that
only receives can leave the latter out. `dw1000.c` uses `<math.h>`, so a
hosted link wants `DW1000_LIBS`, which is `m`.

Zephyr and MyNewt need none of this: `zephyr/CMakeLists.txt` is a module
that reads `dw1000.cmake` itself, and `hw/drivers/dw1000/pkg.yml` is a
MyNewt package.


Version
=======

`<dw1000/dw1000_version.h>` carries the release, and is the one place it
is written -- a C header can read no other file, so a consumer must be
able to have the version without running anything. `dw1000.cmake` parses
those three lines for `DW1000_VERSION`, and the Makefile asks
`scripts/manifest.sh`, which parses them too, so nothing can disagree
with the header.

`<dw1000/dw1000.h>` includes it, so a consumer of the driver API already
has these; include it directly if the version is all you want.

```c
#include <dw1000/dw1000.h>          /* or <dw1000/dw1000_version.h> alone */

#if !DW1000_VERSION_AT_LEAST(1, 1, 0)
#error this application needs dw1000 1.1.0 or newer
#endif

printf("dw1000 %s\n", DW1000_VERSION_FULL);
```

| **Macro** | **What it says** |
|---|---|
| `DW1000_VERSION_MAJOR` / `_MINOR` / `_PATCH` | the release, as numbers |
| `DW1000_VERSION_STRING` | the release, as `"1.1.0"` |
| `DW1000_VERSION_NUMBER` | the release as one comparable integer -- 1.2.3 is `10203` |
| `DW1000_VERSION_AT_LEAST(maj, min, pat)` | for `#if` |
| `DW1000_VERSION_FULL` | the release, plus the commit a build between releases came from |

All macros, and no function: there is no library here to have been
replaced underneath you -- the driver is vendored, so its headers and its
sources are compiled together out of one tree -- and a call would cost an
MCU a symbol and a string it may not want.

A release is the release alone, `1.1.0`. Anything else appends SemVer
build metadata: `1.1.0+58.g3403fe0`, fifty-eight commits past the tag,
plus `.dirty` for uncommitted changes. `scripts/gitversion.sh` works that
out, and it is empty when the answer would be somebody else's -- a tarball
has no repository, and a tree copied into your repository rather than
cloned would otherwise report *your* tags and dirt as the driver's. An
empty answer is never wrong, only less precise.

`make version` prints the release, `make version-full` what this tree
builds as, the two being equal exactly when it is a release tree. The git
part reaches the code only through `-DDW1000_VERSION_GIT`, which this
tree's `make lib` passes and `make sources` hands over for a vendoring
build to pass on:

```sh
eval "$(make -s -C 3rd/decawave-drivers sources OSAL=unix)"
cc $DW1000_CFLAGS -DDW1000_VERSION_GIT="\"$DW1000_VERSION_GIT\"" \
   -c $DW1000_SOURCES $DW1000_OSAL_SOURCES
```

Unlike the `DW1000_WITH_*` options it changes no structure and no entry
point, so it is the one define that need not reach every translation unit.

Bumping a release is editing the three numbers in the header, committing,
and `make tag`, which reads the number out of the header rather than
having it typed again -- so the tag and the header cannot end up saying
different things. It refuses on an uncommitted worktree or an existing
tag, and pushes nothing.

```sh
$EDITOR hw/drivers/dw1000/include/dw1000/dw1000_version.h
git commit -am 'Bump the version to 1.1.1.'
make tag                             # v1.1.1, from the header
git push origin v1.1.1
```

`make tag` refuses an unclean worktree or an existing tag, then asks, then
runs the whole check suite before it tags — so declining costs nothing and
nothing gets tagged that the suite has not passed. `YES=1` answers yes for
a script; a non-interactive run without it declines.

A tag made by hand can still disagree, so `make check` gates the other
direction (`scripts/checktag.sh`): on a tag with a clean worktree -- a
release build, which has no later chance to be wrong -- the header must
say what the tag says, and anywhere else it must be at or ahead of the
nearest tag. Bumped-but-not-yet-tagged is the normal state between
releases and passes; behind a tag that exists does not. It says nothing at
all where the answer would be somebody else's: no git, a tarball, or a
tree copied into another project's repository.


Documentation
=============

Running `doxygen` at the top of the tree writes HTML and LaTeX into
`doc/generated`, which is not tracked. The [DW1000 User Manual][7] is the
reference the code is annotated against, by section and table number at
the point each register sequence is issued.


License
=======

Apache-2.0. See `LICENSE`.


[1]: https://gitlab.inria.fr/dalu/bitters
[2]: https://www.zephyrproject.org/
[3]: http://www.chibios.org/
[4]: http://mynewt.apache.org/
[5]: https://www.bitcraze.io/products/crazyflie-2-1/
[6]: https://www.qorvo.com/products/p/DW1000
[7]: https://www.qorvo.com/products/d/da007967
