# Decawave DW1000 driver

A driver for the [DW1000][6] ultra-wideband transceiver. It configures a
radio, transmits and receives frames, and reports the timestamps that
two-way ranging needs, from an application on Zephyr, ChibiOS, MyNewt,
FreeRTOS or Linux.

The part that knows about the chip knows nothing about the system it
runs on. The register map, the tuning tables and the transmit and
receive sequencing live in one portable core; everything host-specific
is a small OS abstraction layer (OSAL) behind a fixed contract, and
there is one of those per supported system. Register accesses cite the
section or table of the [DW1000 User Manual][7] that prescribes them, so
the code can be checked against the source it came from.

This file is how to use the driver. [`DESIGN.md`](DESIGN.md) is the
other half: why it is shaped this way, what the port contract asks of a
new host, and how the build fits together.

## What you supply, and what you get

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

Pick the OSAL port for your host and the core compiles against it
unchanged. Moving to another host means changing `port/`, not `hw/`.

## Supported hosts

| Host                    | OSAL             | Notes                    |
| ----------------------- | ---------------- | ------------------------ |
| [Zephyr][2] 2 and later | `port/zephyr`    | also a Zephyr module     |
| [Crazyflie][5] 2.x      | `port/cf2`       | FreeRTOS, deck SPI       |
| Unix                    | `port/unix`      | via the [bitters][1] lib |
| [ChibiOS][3]            | `port/chibios`   | SPI API v1 or v2         |
| [Apache MyNewt][4]      | `port/mynewt`    | via `hal_spi_txrx()`     |
| Emulation               | `port/emulation` | a chip model, no chip    |
| none                    | `port/null`      | compiles, does nothing   |

ChibiOS and MyNewt are built against their vendor headers but have not
been exercised on hardware recently.

Neither `port/null` nor `port/emulation` is a host, and they are not the
same kind of stand-in. `null` implements the whole port contract wired
to nothing, so the core compiles where there is no DW1000 and no vendor
tree; it is what `make check` builds against, and the shortest thing to
copy when writing a port of your own. `emulation` runs the driver
against a model of the chip, which needs a medium server on the other
end of its socket; see
[`port/emulation/README.md`](port/emulation/README.md).

## Bringing a device up

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

`dw1000_initialise()` and `dw1000_configure()` both report failure, and
both leave the chip untouched when they do, so a caller that checks them
never proceeds on a half-configured radio. Most of the radio
configuration indexes a tuning table, so an out-of-range value would be
an out-of-range read rather than merely a wrong setting; the check that
catches it runs in every build, not only where assertions are enabled.

`dw1000_config_t` must outlive the driver (`dw1000_init()` keeps the
pointer), while `dw1000_radio_t` is copied by value, so a local is
fine. That and the rest of the traps are in
[`hw/drivers/dw1000/README.md`](hw/drivers/dw1000/README.md), which is
worth reading before writing an application, not after.

## Handling events

Register a callback for what you care about, then call
`dw1000_process_events()` from a thread woken by the interrupt handler,
or from a polling loop. It performs SPI transfers, so it cannot run in
the interrupt handler itself:

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

The interrupt mask is derived from the callbacks you register, so a
callback left `NULL` also leaves its event masked.

The receiver is not re-armed for you after an error or a timeout: that
is the callback's job, or `RXAUTR`'s if you set `cfg->rxauto`. In
double-buffered mode the receiver *is* re-enabled before the frame is
read out, so `cb.rx_ok` must consume the frame before returning and must
not re-enable the receiver itself.

## Compile-time options

An option left undefined takes the default below, which is not always
off. Under Zephyr they are the `CONFIG_DW1000_*` symbols in `Kconfig`
and under MyNewt they are `syscfg` settings; elsewhere define them on
the compiler command line. `make options` prints this table.

| Option (prefix `DW1000_WITH_`) | Default | Effect                    |
| ------------------------------ | ------- | ------------------------- |
| `PROPRIETARY_PREAMBLE_LENGTH`  | 1       | non-standard lengths      |
| `PROPRIETARY_SFD`              | 1       | the Decawave SFD          |
| `PROPRIETARY_LONG_FRAME`       | 0       | frames to 1023, not 127   |
| `EXTENDED_SEND`                | 1       | delayed and stamped sends |
| `SFD_TIMEOUT`                  | 0       | caller-chosen SFD timeout |
| `SFD_TIMEOUT_DEFAULT`          | 0       | a fixed, not computed one |
| `EVENT_COUNTERS`               | 0       | the 0x2F diagnostic bank  |
| `ACCUMULATOR`                  | 0       | the CIR accumulator read  |
| `TEMP_COMPENSATION`            | 0       | TX power and bandwidth    |
| `DEBUG`                        | 0       | the bench diagnostics     |

One more takes a value rather than a flag:
`DW1000_SFD_TIMEOUT_DEFAULT` (default `DW1000_SFD_TIMEOUT_MAX`).

> [!IMPORTANT]
> These are not internal to the driver. `DW1000_WITH_SFD_TIMEOUT` and
> `DW1000_WITH_PROPRIETARY_SFD` add fields to `dw1000_config_t`, and
> `DW1000_WITH_EXTENDED_SEND` adds entry points to
> `<dw1000/dw1000_send.h>`, so the same set has to reach the driver and
> every translation unit that includes `<dw1000/dw1000.h>`. Define them
> in one place your whole build sees, not per file.

Two things that look like options and are not. Transmit power is part of
`dw1000_radio_t`, named outright by `DW1000_TX_POWER(dB)`, so a board
whose RF path differs from the reference one is corrected where the
radio is configured. And the lead time of a delayed send is a run-time
argument, because half of it is the host's own latency and the other
half changes with the preamble length; `dw1000_tx_get_preamble_airtime()`
reports the part the driver knows. `DESIGN.md` says why for both.

## Adding it to a build

The driver is vendored into a project rather than installed, so there is
no `make install`: the options above change the public headers, and a
shared library could not carry them. The top-level `Makefile` does the
other half of the job, with either GNU make or BSD make.

```text
make                print the targets; nothing is built by default
make sources        print the files and flags to vendor
make lib            build libdw1000.a locally, against OSAL=<port>
make check          preflight: options compile, the manifest
make tests          run the validation, emulation, probe and sniffer tests
make options        print the option table above
make version        print the release
make doc            run doxygen into doc/generated
```

From a shell-driven build, ask `make sources` and use what it sets:

```sh
eval "$(make -s -C 3rd/decawave-drivers sources OSAL=unix)"
cc $DW1000_CFLAGS -DDW1000_WITH_PROPRIETARY_LONG_FRAME=1 \
   -c $DW1000_SOURCES $DW1000_OSAL_SOURCES
```

From CMake, include `dw1000.cmake` and read its variables. It defines
variables rather than targets on purpose: a target would bake in one
option set and hide it from you.

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

The variables worth knowing:

| Variable                  | Holds                                     |
| ------------------------- | ----------------------------------------- |
| `DW1000_SOURCES`          | core, send and radio-value validation     |
| `DW1000_SOURCES_CORE`     | `dw1000.c` alone; receive-only builds     |
| `DW1000_SOURCES_SEND`     | `dw1000_send.c`; nothing in core calls it |
| `DW1000_SOURCES_VALIDATE` | human radio values to radio fields        |
| `DW1000_SOURCES_STATE`    | read the radio back; opt-in, see below    |
| `DW1000_LIBS`             | `m`, for `<math.h>` on a hosted link      |
| `DW1000_OSAL_<PORT>_*`    | include dir and sources for one port      |

A receive-only application can leave `DW1000_SOURCES_SEND` out;
`sniffer/app/unix` does exactly that. `DW1000_SOURCES_STATE` is opt-in
because reading the configuration back off the chip is diagnostic, and
it is the only source here that formats strings.

Zephyr and MyNewt need none of this: `zephyr/CMakeLists.txt` is a module
that reads `dw1000.cmake` itself, and `hw/drivers/dw1000/pkg.yml` is a
MyNewt package.

## Asking for a version

`<dw1000/dw1000_version.h>` carries the release and is the one place it
is written. `<dw1000/dw1000.h>` includes it, so an application already
has these; include it directly if the version is all you want.

```c
#include <dw1000/dw1000.h>          /* or <dw1000/dw1000_version.h> alone */

#if !DW1000_VERSION_AT_LEAST(1, 1, 0)
#error this application needs dw1000 1.1.0 or newer
#endif

printf("dw1000 %s\n", DW1000_VERSION_FULL);
```

| Macro                            | What it says                  |
| -------------------------------- | ----------------------------- |
| `DW1000_VERSION_MAJOR`           | the release's major number    |
| `DW1000_VERSION_MINOR`           | its minor                     |
| `DW1000_VERSION_PATCH`           | its patch                     |
| `DW1000_VERSION_STRING`          | the release, as `"1.1.0"`     |
| `DW1000_VERSION_NUMBER`          | one integer; 1.2.3 is `10203` |
| `DW1000_VERSION_AT_LEAST(m,n,p)` | for `#if`                     |
| `DW1000_VERSION_FULL`            | the release, plus the commit  |

All macros and no function, so a version costs an MCU neither a symbol
nor a string it may not want.

A release is the release alone, `1.1.0`. Anything else appends SemVer
build metadata: `1.1.0+58.g3403fe0` is fifty-eight commits past the tag,
plus `.dirty` for uncommitted changes. That part is empty when the
answer would be somebody else's. A tarball has no repository, and a
tree copied into your repository would otherwise report *your* tags as
the driver's. An empty answer is never wrong, only less precise.

To get the git part into a vendoring build, pass what `make sources`
hands you:

```sh
eval "$(make -s -C 3rd/decawave-drivers sources OSAL=unix)"
cc $DW1000_CFLAGS -DDW1000_VERSION_GIT="\"$DW1000_VERSION_GIT\"" \
   -c $DW1000_SOURCES $DW1000_OSAL_SOURCES
```

Unlike the `DW1000_WITH_*` options it changes no structure and no entry
point, so it is the one define that need not reach every translation
unit.

## Documentation

Each part of the tree is documented beside itself. This is the index:

- [`DESIGN.md`](DESIGN.md): why the driver is shaped this way, the
  port contract, and how the build and the checks fit together.
- [`hw/drivers/dw1000/README.md`](hw/drivers/dw1000/README.md): the
  traps an application falls into: the `rx_ok` contract, double
  buffering, what the driver does *not* do.
- [`hw/drivers/dw1000/DESIGN.md`](hw/drivers/dw1000/DESIGN.md): how
  the core works inside, and where it departs from Decawave's driver.
- [`port/emulation/README.md`](port/emulation/README.md): the wire
  protocol between a node and a medium server, for running the driver
  with no chip.
- [`probe/README.md`](probe/README.md): building and running the
  two-way-ranging instrument, and how to read its lines.
- [`probe/DESIGN.md`](probe/DESIGN.md): the probe's exchange, record
  contract and split line.
- [`sniffer/README.md`](sniffer/README.md): building and running the
  UWB sniffer, which forwards captured frames over ethernet or writes
  them as pcapng for wireshark.
- [`sniffer/DESIGN.md`](sniffer/DESIGN.md): the sniffer's frame ring,
  its wire format, its dissector interface, and why four of its files
  are portable.
- [`AUDIT.md`](AUDIT.md): what was checked against the manual, what
  was wrong, what was measured, and what is still open.
- [`DW1000.md`](DW1000.md): what the chip does, as established from
  the manual, the errata and the bench, each finding graded by its
  source.
- [`INVESTIGATE.md`](INVESTIGATE.md): what is still open about the
  chip and the bench, with the evidence so far and the experiment
  that would settle each.

Running `doxygen` at the top of the tree writes HTML and LaTeX into
`doc/generated`, which is not tracked. `bench/` holds the scripts that
drive and read a bench run.

### The vendor documents

`docs/` is the vendor shelf: the manuals, the errata and the
application notes, as published. It is ignored by git, because
Decawave's documents carry a copyright notice and no licence to
redistribute them; fetch them from Qorvo, which now owns Decawave.
Each link below serves the PDF directly, and was checked on
2026-09-18 by downloading it and reading the document's own title
page. "On the shelf" is the revision AUDIT.md and DW1000.md cite;
"served" is what the link returned that day. Qorvo serves only the
current revision, so an older manual (2.12, 2.15) can no longer be had
from them.

| Document | On the shelf | Served | Where |
| :------- | :----------- | :----- | :---- |
| DW1000 User Manual | 2.18 (also 2.12 and 2.15, cited by page) | 2.18 | [da007967](https://www.qorvo.com/products/d/da007967) |
| DW1000 Errata | 1.4, April 2021 | 1.4 | [da007968](https://www.qorvo.com/products/d/da007968) |
| DWM1000 module data sheet | | 2.0, March 2026 | [da007948](https://www.qorvo.com/products/d/da007948), which Qorvo lists as the DW1000 datasheet but which serves the module's |
| DW1000 Device Driver API Guide | 2.1 | 2.14, inside "DW1000 API with STM32F10x Application Examples" | [ra006746](https://www.qorvo.com/products/r/ra006746), a software package, not a PDF |
| APH001, DW1000 hardware design guide | 1.1 | 1.2 | [da008428](https://www.qorvo.com/products/d/da008428) |
| APH005, power source selection | 1.00 | 1.3 | [da008429](https://www.qorvo.com/products/d/da008429) |
| APS006 part 1, channel effects on range and timestamp accuracy | 1.03 | 1.04 | [da008440](https://www.qorvo.com/products/d/da008440) |
| APS011, sources of error in two-way ranging | 1.0 | 1.2 | [da008446](https://www.qorvo.com/products/d/da008446) |
| APS012, production tests | 1.5 | 1.8 | [da008447](https://www.qorvo.com/products/d/da008447) |
| APS013, the implementation of two-way ranging | 2.0 | 2.4 | [ra007039](https://www.qorvo.com/products/r/ra007039) |
| APS014, antenna delay calibration | 1.01 | 1.3 | [da008449](https://www.qorvo.com/products/d/da008449) |
| APS022, debugging DW1000-based products and systems | 1.4 | 1.4 | [da008452](https://www.qorvo.com/products/d/da008452) |
| APS023 part 1, transmit power calibration and management | 1.4 | 1.4 | [da008453](https://www.qorvo.com/products/d/da008453) |
| APS023 part 2, TX bandwidth and channel power compensation | 1.4 | 1.4 | [da008454](https://www.qorvo.com/products/d/da008454) |

The [product page][6] lists them all under Documents and Software,
should an identifier above move.

Qorvo's site now answers `curl` and `wget` with a Vercel bot challenge
(HTTP 429, `x-vercel-mitigated: challenge`), whatever headers are sent.
A real browser passes it: headless Chromium against the link above
lands the PDF in the download directory.

## License

Apache-2.0. See `LICENSE`.


[1]: https://gitlab.inria.fr/dalu/bitters
[2]: https://www.zephyrproject.org/
[3]: http://www.chibios.org/
[4]: http://mynewt.apache.org/
[5]: https://www.bitcraze.io/products/crazyflie-2-1/
[6]: https://www.qorvo.com/products/p/DW1000
[7]: https://www.qorvo.com/products/d/da007967
