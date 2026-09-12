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

ChibiOS and MyNewt are built against their vendor headers but have not
been exercised on hardware recently.


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
│   └── cf2                        Crazyflie 2.x, on FreeRTOS
│
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

Undefined means off. Under Zephyr they are `CONFIG_DW1000_*` in
`Kconfig`; elsewhere define them on the compiler command line.

| Option                                     | Effect                   |
| ------------------------------------------ | ------------------------ |
| `DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH`  | preamble lengths outside the standard set |
| `DW1000_WITH_PROPRIETARY_SFD`              | the Decawave non-standard SFD |
| `DW1000_WITH_PROPRIETARY_LONG_FRAME`       | frames up to 1023 bytes, not 127 |
| `DW1000_WITH_EXTENDED_SEND`                | delayed send with an embedded timestamp |
| `DW1000_WITH_SFD_TIMEOUT`                  | caller-chosen SFD timeout |
| `DW1000_WITH_SFD_TIMEOUT_DEFAULT`          | a fixed SFD timeout, not a computed one |
| `DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011` | work around a spurious AAT on receive |
| `DW1000_WITH_DWM1000_EVK_COMPATIBILITY`    | +3 dB, DWM1000 under EVB1000 software |


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
