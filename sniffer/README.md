UWB sniffer
===========

Capture UWB frames off a DW1000 and forward each one, whole, inside an
ethernet frame to another host -- where wireshark or tcpdump can look at
it. The radio is configured from the command line, the receiver runs with
no timeout and re-arms after every frame, and nothing is transmitted: this
is a listener.

It is an application and nothing else -- no library part, no port layer,
and no entry in `dw1000.cmake`, for the same reason `probe/app/unix` has
none: nothing else consumes it. What it does consume is the driver
(receive half only) and [bitters][bitters] for the Raspberry Pi's GPIO and
SPI.

It arrived here from `rpi-uwb-sniffer`, which was a repository of its own
holding this program plus submodule copies of the two trees it needs. Here
it is one directory beside the driver it was always built against.

[bitters]: https://gitlab.inria.fr/dalu/bitters


Usage
=====

~~~
Usage: uwb-sniffer [OPTIONS]* <dst_macaddr>
  -P, --prototype=INT        ethernet prototype (default: 9999)
  -i, --interface=STRING     ethernet interface (default: the first valid one)
  -c, --channel=INT          channel                     (default: 5)
  -b, --bitrate=INT          bitrate (in kbps)           (default: 6800)
  -p, --prf=INT              pulse rate frequency (in MHz)  (default: 64)
      --tx_plen=INT          preamble length             (default: 128)
      --rx_pac=INT           preamble accumulation       (default: 8)
      --tx_pcode=INT         TX preamble code            (default: 10)
      --rx_pcode=INT         RX preamble code            (default: 10)
      --tx_delay=FLOAT       antenna TX delay (in meters)
      --rx_delay=FLOAT       antenna RX delay (in meters)
  -v, --verbose              verbose mode
  -V, --version              show version information
~~~

Every radio value is checked before the chip is touched, and a bad one
ends the program with the reason rather than being quietly clamped --
`uwb_dw1000_validate.c` is where the accepted values are, and it is the
one place that knows them. An antenna delay left unset takes the driver's
own; `-V` reports the driver release this was built against, since that
is now the version this program has.

`--prototype` must be 0 or in 0x05DD..0xFFFF: anything at or below 0x05DC
would be read as a length by a receiving stack rather than as a type,
which is why the default is 9999 (0x270F) and not something smaller.

`-v` prints a line per frame. Without it the program is quiet once running,
which is what you want at any real frame rate -- that print used to happen
inside the receive callback, where it cost the radio time on every frame.


How it receives
===============

The radio runs double buffered, and frames are forwarded from a ring
rather than from the callback that received them. Both are lessons taken
from `probe`, which arrived at them the hard way; the driver's own guide
(`hw/drivers/dw1000/README.md`, "Double buffered receive") is the place
they are written down.

`dblbuff` means the driver re-enables the receiver in the good-frame path
*before* it calls back, so the next frame lands in the other buffer while
this one is read out, and it toggles the host side buffer pointer as soon
as the callback returns. Two things follow. The callback must not re-enable
the receiver -- so the `uwb_rx_start()` that used to sit at the bottom of
the poll loop is gone, and the receiver is armed exactly once, before the
loop. And everything the frame is wanted for must be read out before the
callback returns, because `RX_BUFFER` swings with the pointer.

That second obligation is what the ring is for. `eth_send()` is a blocking
`sendmsg(2)`; doing it inside the callback would hold the callback open
across the kernel's network path while the next frame is already arriving.
So the callback copies the frame into a ring slot and returns, and the loop
forwards what the ring holds afterwards. `capture.c` carries the detail,
including why it needs no lock where probe's equivalent does.

A frame can still be lost, in exactly one place: if forwarding falls more
than the ring's depth (16) behind the radio, the oldest entries are
overwritten. That is counted, and the loop says so on stderr when the count
moves -- `lost N frame(s): forwarding fell behind the radio`. An overrun on
the *chip* (both buffers held and a third frame arriving) is a separate
thing, recovered by the driver and reported through the `rx_error` callback,
which re-arms; that callback is not optional, as `dw1000_initialise()`
refuses `dblbuff` without one.


Example
=======

~~~sh
uwb-sniffer -i eth1 -P 6666                                 \
    -c 5 -b 6800 -p 64                                      \
    --tx_pcode 10 --rx_pcode 10 --tx_plen 128 --rx_pac 8     \
    dc:4a:3e:06:6f:7b
~~~

On the receiving host, filtering on the type and on the Pi's own address
(the program prints the tcpdump line to use, with its real address filled
in, once the interface is up):

~~~sh
tcpdump ether proto 6666 and ether src aa:bb:cc:dd:ee:ff
~~~


Wiring
======

On a Raspberry Pi, connect the DecaWave module to these pins:

| RPI | DWM1000 | Meaning
|-----|---------|-------------------
|  14 | GND     | Ground
|  16 | IRQ     | DWM1000 interrupt
|  17 | 3V3     | Power 3.3V
|  18 | RESET   | DWM1000 reset
|  19 | MOSI    | SPI MOSI
|  21 | MISO    | SPI MISO
|  22 | WAKEUP  | DWM1000 wakeup
|  23 | SCLK    | SPI clock
|  24 | CS      | SPI chip select

Reset and wakeup want a pull-up held across boot, which the Pi does not
do by itself; `uwb_init()` prints the `raspi-gpio set` line to put in a
boot-time script.

**This is not the pin map the rest of the tree uses.** `rpi-redskin`, and
`probe/app/unix/config.h` after it, put wakeup on P1_16 and the interrupt
on P1_15; this program has wakeup on P1_22 and the interrupt on P1_16
(reset is P1_18 for all three). A bench wired for one will not run the
other. The map lives in `app/unix/config.h`, and it was left as it was
found -- rewiring somebody's bench is not a thing a file move should do.


Building
========

~~~sh
sh sniffer/app/unix/build.sh                  # -> build/uwb-sniffer
~~~

The script reads the file list out of each tree's own manifest rather than
naming sources itself -- `make -s sources` here and in bitters -- so
neither list can go stale. `-n` prints the compiler command and compiles
nothing; `-o` names the executable.

It needs bitters, and looks for it at `$HOME/Repos/bitters` unless
`BITTERS=` says otherwise:

~~~sh
BITTERS=/elsewhere/bitters sh sniffer/app/unix/build.sh -o /tmp/sniffer
~~~

`libpopt` (headers included) is the one other dependency -- `cmdline.c`
uses it. The target is Linux: bitters reaches the GPIO and SPI character
devices directly, so this does not build on anything else, and it is the
Pi that it is meant for.

Only the receive half of the driver is compiled in
(`DW1000_SOURCES_CORE`, not `_SEND`): `dw1000.c` never calls into
`dw1000_send.c`, and a sniffer never transmits. That is the receive-only
case `dw1000.cmake` documents.


TODO
====
* the ring depth (16) is a guess, not a measurement; the overrun counter is
  what would say whether it is enough under real traffic
* a `--no-dblbuff` switch, as `rpi-redskin` has, to fall back without a
  rebuild -- double buffering is recent enough that probe still marks it an
  experiment
