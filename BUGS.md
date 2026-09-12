# Known bugs

Open defects, most severe first. Each entry names a concrete condition
that triggers wrong behaviour; anything that could not be confirmed from
the sources available here is marked **suspected**, with the check that
would settle it.

Found by an audit of `hw/drivers/dw1000` and the five OSAL ports, against
the DW1000 User Manual v2.05. Line numbers refer to the state of the tree
when the entry was written.

**Severity** — *high*: silent wrong results, corruption, or a crash on
input an ordinary caller can supply. *medium*: real misbehaviour with a
bounded blast radius. *low*: genuine but slight.


## Open

### 1. cf2: chip select bypasses the dual pin encoding — *medium*, suspected

`port/cf2/dw/osal/src/osal.c:36,40,51,56`

`spi->cs_pin` is typed `dw1000_ioline_t`, and that type has two
encodings, both handled by `_dw1000_ioline_set()` / `_dw1000_ioline_clear()`
(`port/cf2/dw/osal/include/dw1000/osal.h:57-85`): a deck pin number
(`<= 0xFFFF`, driven through `digitalWrite`) and a direct-GPIO form
`(port << 16) | pin_mask` (driven through `GPIO_WriteBit`). The SPI paths
call `digitalWrite(spi->cs_pin, …)` directly, which only understands the
first.

*Trigger*: a board whose DW1000 chip select is on a GPIO that is not a
deck pin — the case the encoded form exists for, and the form this port
already supports for the reset and wakeup lines.

*Impact*: chip select is never asserted. Every header and every data byte
is clocked out with the chip deselected: register writes are lost, reads
return garbage.

*Fix*: use `_dw1000_ioline_clear(spi->cs_pin)` / `_dw1000_ioline_set(spi->cs_pin)`,
as the mynewt port does (`port/mynewt/dw/osal/src/osal.c:18,25,37,44`).

*Suspected on*: whether Crazyflie's `digitalWrite()` traps or silently
returns for a pin above `DECK_MAX_PIN`. Read `deck_digital.c` in
crazyflie-firmware to settle it. Either way chip select is not asserted;
only the failure mode differs.


### 2. chibios, cf2 and mynewt drop SPI failures silently — *medium*

Missing `error` field:
`port/chibios/dw/osal/include/dw1000/osal.h`,
`port/cf2/dw/osal/include/dw1000/osal.h`,
`port/mynewt/dw/osal/include/dw1000/osal.h`.
Discarded status: `port/cf2/dw/osal/src/osal.c:39,54`.

The unix and zephyr ports carry a latched `int error` on
`dw1000_spi_driver_t` — first failure kept, cleared by whoever reports
it. The other three have no such field. cf2 is the concrete case:
`spiExchange()` returns a status and it is discarded on both paths.

This matters because the driver core has no error path of its own: the
register read helpers (`_dw1000_reg_read8/16/32/64`,
`hw/drivers/dw1000/include/dw1000/dw1000.h:655,674,693,712`) declare an
uninitialised local, hand it to the OSAL and return it unconditionally.

*Trigger*: any failing transfer on cf2.

*Impact*: the failure is invisible at every layer. cf2's receive path then
does `memcpy(data, _dw1000_spi_buffer.rx + hdrlen, datalen)` from a
**static** bounce buffer, handing back the bytes of the *previous*
register read — plausible-looking and stable across retries, which is
worse than garbage. An application written against the unix/zephyr
contract does not even compile against these three ports.

*Fix*: add `int error;` to the three structs plus a shared
`_dw1000_spi_record()` helper; on cf2, feed `spiExchange()`'s return into
it and zero `data` on failure. chibios (`spiSend`/`spiReceive` return
`void`) and mynewt (`hal_spi_tx_val` returns `0xFFFF` on error) need an
API-level change to detect anything, but should carry the field so the
interface stays uniform.


### 3. chibios cannot survive a zero-length data segment — *medium*, suspected

`port/chibios/dw/osal/src/dw_osal.c:15,24`

`spiSend()` / `spiReceive()` are called unconditionally, with no
`datalen != 0` guard.

*Trigger*: `dw1000_tx_sendv()` with an iovec entry whose `iov_len` is 0.
`_dw1000_tx_prepare_data_sendv()` forwards each segment verbatim
(`hw/drivers/dw1000/src/dw1000_send.c:27`) and
`dw1000_tx_write_frame_data()` clamps only the upper bound, so a
zero-length payload reaches the OSAL.

*Impact*: with `CH_DBG_ENABLE_CHECKS`, the `osalDbgCheck(n > 0U)` inside
`spiSend`/`spiReceive` halts the system; without it, a zero-length DMA is
started and the calling thread blocks on a completion interrupt that
never arrives, with chip select held low. The other four ports degrade
gracefully.

*Fix*: guard both calls (`if (datalen) spiSend(…)`), keeping the
select / header / unselect sequence intact.

*Suspected on*: the exact `osalDbgCheck` in the ChibiOS HAL version in
use. Read `spiSend`/`spiReceive` in `os/hal/src/hal_spi.c`.


### 4. cf2: `DW1000_IOLINE_NONE` is `0`, a valid deck pin — *low*, suspected

`port/cf2/dw/osal/include/dw1000/osal.h:55`, with `DECK_MAX_PIN 13` at `:51`

Every other port picks a sentinel outside the valid range (`PAL_NOLINE`,
`-1`, `NULL`, `BITTERS_GPIO_PIN_NONE`). cf2 picks `0`, while deck pins
appear to be numbered from 0.

*Trigger*: a board wiring RSTn, WAKEUP or IRQ to deck pin 0.

*Impact*: the core's three `DW1000_IOLINE_NONE` comparisons all mis-fire.
`dw1000_hardreset()` returns without pulsing RSTn
(`hw/drivers/dw1000/src/dw1000.c:993`), the wakeup line is never raised
(`:1003`), and GPIO8 is programmed as a plain GPIO instead of the IRQ
output (`:1169`) — so the chip never raises an interrupt and a
callback-driven application stalls forever.

*Fix*: use a sentinel outside the pin range, e.g. `UINT32_MAX`.

*Suspected on*: whether deck pins are numbered from 0. Check
`DECK_GPIO_IO1` in crazyflie-firmware.


### 5. Input validation is assert-only, so release builds index out of range — *low*

`hw/drivers/dw1000/src/dw1000.c:484,545`, `:1713`,
`hw/drivers/dw1000/src/dw1000_send.c:28`

`DW1000_ASSERT` maps to `assert()` / `__ASSERT` / `osalDbgAssert` on four
of the five ports, so it is compiled out under `NDEBUG`, without
`CONFIG_ASSERT`, or without `CH_DBG_ENABLE_ASSERTS`. Only cf2
(`bkpt #1`) is always on. The checks in `dw1000_configure()` are then the
only thing between a bad value and an out-of-range table index.

*Trigger*: `radio->channel` of 0 or 6 makes `channel_table_mapping[]`
yield `-1`, so `channel_tunning[-1]` and `manual_tx_power[-1]` are read
out of bounds.

Separately, `_dw1000_tx_prepare_data_sendv()` accumulates the
**unclamped** iovec total (`hw/drivers/dw1000/src/dw1000_send.c:28`)
while
`dw1000_tx_write_frame_data()` silently clamps the write at 1024 bytes.
`TFLEN`+`TFLE` occupy bits 0-9 of `TX_FCTRL`
(`hw/drivers/dw1000/src/dw1000.c:1713`), so a total
of 8192 or more shifts into `TXBR` (bits 13-14) and silently changes the
on-air bitrate.

*What the contract protects*: these asserts are what turns a misconfigured
radio into a loud failure during bring-up, instead of a node that
transmits at the wrong bitrate on the wrong channel. Relaxing them is not
the fix.

*Fix*: either map `DW1000_ASSERT` to an always-on trap for the
config-validation sites (as cf2 does), or return an error from
`dw1000_configure()` instead of asserting. The latter changes the API.


## Fixed

- **Double buffered status bits were cleared with the interrupt live.**
  UM §4.3.3 (figure 14) and §4.3.4 (figure 15) require the doubly-buffered
  status bits (RXDFR, RXFCG, RXFCE, LDEDONE) to be masked while they are
  cleared, "to prevent glitch when cleared". `_dw1000_txrx_off()` always
  did; the per-frame RXFCG path did not, and every received frame went
  through it. Fixed by `dw1000_rx_clear_status_dblbuf()` in
  `hw/drivers/dw1000/src/dw1000.c`, applied only in double buffered mode.

- **zephyr: a failed SPI read left the caller's buffer untouched.** Since
  the core's read helpers return an uninitialised local unconditionally,
  a failed read fed the driver stack garbage: the device-ID check in
  `dw1000_initialise()` passed or failed at random, and
  `dw1000_process_events()` dispatched on a made-up status. The unix port
  zeroes on failure; zephyr now does too
  (`port/zephyr/dw/osal/src/osal.c`).


## Investigated, not bugs

Recorded so they are not raised again:

- **The HPDWARN retry loop** in `dw1000_tx_extended_vsendv()` does not
  wedge on a stale flag. UM v2.05 p.90: *"The HPDWARN event status flag is
  READ ONLY. It will clear when the delayed TX/RX is cancelled…"* — and
  the cancel path issues TRXOFF.
- **`DW1000_CLOCK_ROUNDUP` on a 64-bit time.** `~0x1FF` is `int` −512, but
  the usual arithmetic conversions sign-extend it to `0xFFFF…FE00`
  against a `uint64_t`. No truncation.
- **`dw1000_get_system_time()` / `dw1000_rx_get_rmarker_time()`** read 5
  bytes into a `uint64_t` and byte-swap; correct on big-endian too.
- **Antenna delay in the embedded timestamp.** `DX_TIME + TX_ANTD` is what
  `TX_STAMP` reports, so the self-check the API documents holds.
- **`dw1000_bswap_n()`** is correct for lengths 3, 4, 5 and 8.
- **Re-enabling the receiver before read-out in double buffered mode** is
  explicitly permitted by UM §4.3.1.
- **The RXOVRR recovery** (TRXOFF, receiver-only reset, discard frames) is
  what UM figure 14 prescribes.
- **Register definitions**: a scripted pass over `dw1000_reg.h` checked
  every `MSK_*`/`SFT_*` pair for contiguity and every `FLG_*` for being a
  single bit — 246 definitions, none inconsistent. Register *values* (the
  tuning tables, PLL and RF constants) were not checked against the
  datasheet.


## Not audited

`hw/drivers/dw1000/include/dw1000/dw1000_otp.h` beyond its use sites, the
Kconfig / CMake / `syscfg.yml` build glue, and `doc/generated`.
