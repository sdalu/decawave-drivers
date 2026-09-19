# DW1000 emulation wire protocol

`port/emulation` is a DW1000 OSAL port with no chip behind it.
`_dw1000_spi_send()` and `_dw1000_spi_recv()` decode the same SPI
header byte a real transfer would carry and apply it to an in-memory
register model (`struct dw1000_emulation`) instead of a bus, and the
IRQ and RESET GPIO lines are calls into that same model rather than
real pins. The model cannot stand alone: a running node needs a
medium server reachable over a Unix domain socket, which relays TX/RX
frames between every node attached to it and works out reception
timestamps, since that is not something a register model can compute
by itself. This is what sets it apart from `port/null`: `null` is
wired to nothing and cannot exchange a frame at all, while
`emulation` runs the whole driver, against a model of the chip, for
as long as a medium server is listening on the other end of the
socket.

This page is the contract between the two: what a node sends and
receives on the wire, so that a medium server can be written against
it without reading the port's C.


## Where a medium server comes from

There is none in this tree, by design: a register model cannot work out
a reception timestamp, so the medium is somebody else's program and this
page is the seam. The one in use is Ruby, in spank
(`simulation/lib/spank/simulator.rb`), which is where this port came
from in the first place: the C half (the emulation OSAL and its socket
client) moved here so it could be a port like the others and be
compiled by `make check`, while the medium server and the spank node
stayed behind. That split is why this file exists rather than a shared
header: the two halves are in different repositories and different
languages, and the wire is all they have in common.


## Socket and framing

The transport is a connected `AF_UNIX SOCK_DGRAM` socket: the node
binds a private path (`<socket_path>.<pid>`), connects it to the
server's `socket_path`, and every request or reply is one datagram.
Every message starts with a fixed header, packed with no padding:

```c
struct rsvc_inhdr {                 /* node -> server */
    uint16_t type;                  /* service type, e.g. RSVC_OPEN */
    uint64_t id;                    /* sequence id, matches the reply */
    uint8_t  flags;
} __attribute__((packed));

struct rsvc_outhdr {                /* server -> node */
    uint16_t type;
    uint64_t id;
    int8_t   status;                /* 0 on success */
    uint8_t  flags;
} __attribute__((packed));
```

11 bytes in, 12 bytes out. Neither struct declares a byte order, and
nothing on either side byte-swaps them: the node's C `pack`ing and
the server's Ruby `unpack("SQCa*")`/`unpack("SQCCa*")` are both
native-endian, which only works because node and server share a
host. `id` is a per-connection sequence counter the node assigns; the
server echoes it back unchanged so a reply can be matched to its
request, and sends `id = 0` on every unsolicited frame.

Two flag bits, both in the last header byte:

```c
#define RSVC_HDR_FLG_INCLUDE_NICKNAME  0x01
#define RSVC_HDR_FLG_INTERRUPT         0x02
```

`FLG_INCLUDE_NICKNAME` appends the node's nickname, a NUL-terminated
string up to `RSVC_NICKNAME_MAXLEN` (62) bytes, right after the
header and before the payload; only `RSVC_OPEN` sets it, and the
server uses the nickname (rather than the socket path) to look the
node up. `FLG_INTERRUPT` marks a frame the server sends unprompted
(the `RX` and `TX_DONE` messages described below), and the node
dispatches those to a registered handler instead of waking a
caller blocked on a matching reply `id`. The port registers exactly
one such handler, for `RSVC_UWB_IO`; registering a second for the
same type fails. Payload data, in either direction, is capped at
`RSVC_DATA_MAXLEN` (500) bytes.

Every service type the protocol defines:

```c
#define RSVC_OPEN       0x0001
#define RSVC_CLOSE      0x0002
#define RSVC_SEED_GET   0x0003
#define RSVC_UWB_OPEN   0x0301
#define RSVC_UWB_CLOSE  0x0302
#define RSVC_UWB_IO     0x0303
```


## Service messages: OPEN, CLOSE, SEED_GET

**RSVC_OPEN**: sent once, with `FLG_INCLUDE_NICKNAME` set and no
payload, to register the node with the server under its nickname.
No reply payload.

**RSVC_CLOSE**: sent once, no payload, no `FLG_INCLUDE_NICKNAME`
(the server looks the node up by its socket path instead); tells the
server to unregister it. No reply payload.

**RSVC_SEED_GET**: sent with no payload. The reply carries one
4-byte, native-endian `uint32_t`: a random seed for the node to draw
on.


## The UWB channel: UWB_OPEN, UWB_CLOSE, UWB_IO

`RSVC_UWB_OPEN` and `RSVC_UWB_CLOSE` are defined in the protocol, and
the reference medium server implements handlers for both (each
takes an 8-byte `drvid` as its only payload field and replies with no
data), but nothing in this port ever sends either one. In this
codebase the UWB channel is opened implicitly, by registering the
`RSVC_UWB_IO` interrupt handler in `dw1000_emulation_create()`.

`RSVC_UWB_IO` is the one type the port actually uses, in both
directions. The node sends it as an ordinary request/reply call for
`TX` and `RX_CONFIG`; the server sends it back as an unsolicited,
`FLG_INTERRUPT` frame for `RX` and `TX_DONE`. Which of the four it is
sits inside the payload, as a one-byte packet type ahead of a union:

```c
struct __attribute__((packed,aligned(1))) dw1000_driver_iopkt {
    uintptr_t drvid;
    uint8_t   type;
    union {
        struct {
            uint8_t  flags;
            uint16_t antenna_delay;
            uint8_t  frame[DW1000_FRAME_MAXSIZE];
        } __attribute__((packed)) tx;
        struct {
            uint64_t timestamp;
        } __attribute__((packed)) tx_done;
        struct {
            uint8_t  flags;
            uint64_t timestamp;
            uint8_t  frame[DW1000_FRAME_MAXSIZE];
        } __attribute__((packed)) rx;
        struct {
            uint8_t  flags;
        } __attribute__((packed)) rx_config;
    } __attribute__((packed));
};

#define DW1000_RSVC_TX          0x03
#define DW1000_RSVC_RX          0x04
#define DW1000_RSVC_TX_DONE     0x05
#define DW1000_RSVC_RX_CONFIG   0x06

#define DW1000_RSVC_FLG_RANGING 0x02
```

`drvid` is the node's own pointer to its `struct dw1000_emulation`,
native word size and byte order; the server only threads it back
through on a `TX_DONE` reply (the `RX` interrupt always carries
`drvid = 0`, since the server has no node-local pointer to give).
`DW1000_FRAME_MAXSIZE` is 127, or 1023 with
`DW1000_WITH_PROPRIETARY_LONG_FRAME=1`; only as many `frame` bytes as
the actual frame length are put on the wire, the rest of the array is
never sent. As with the outer header, none of these fields are
byte-swapped for the wire: native order throughout.

**TX** (node -> server, request). `flags` bit `0x02`
(`DW1000_RSVC_FLG_RANGING`) is set when the frame's `TX_FCTRL.TR` bit
was set. `antenna_delay` is the node's `TX_ANTD` register, verbatim,
in device clock ticks (see Timestamp convention below). `frame` is
the 802.15.4 frame, FCS included. The reply is a synchronous status 0
with no data; completion is reported later, asynchronously, as a
`TX_DONE` interrupt.

**RX_CONFIG** (node -> server, request). One `flags` byte, always
sent as 0 by this port (`dw1000_emulation_recv()`); the reference
server logs it and applies no behaviour. Reply: synchronous status 0,
no data.

**RX** (server -> node, `FLG_INTERRUPT`). `flags` bit `0x02` is set
when the *sender's* frame was a ranging frame. `timestamp` is an
8-byte field, low 40 bits meaningful, the frame's arrival instant in
the *receiving* node's own clock. `frame` is the frame as the sender
transmitted it.

**TX_DONE** (server -> node, `FLG_INTERRUPT`). `timestamp` is an
8-byte field, low 40 bits meaningful: the node's own send instant,
already including the antenna delay it supplied in its `TX` request.


## Timestamp convention

Every timestamp is in units of the device clock,
`DW1000_TIME_CLOCK_HZ = 499200000ull * 128` Hz (~15.65 ps per tick);
the node's `dw1000.h` and the reference server's Ruby define the same
number. A DW1000 timestamp is a 40-bit quantity, the register
fields it lands in (`RX_TIME.RX_STAMP`, `TX_TIME.TX_STAMP`, and the
`_RAWST` pair) being written 40 bits at a time; it is carried on the
wire in a full 8-byte field, upper 24 bits unused.

For **TX**: the server rounds its own send instant up to the next
512-tick boundary, adds the `antenna_delay` the node supplied in its
`TX` request, and returns that sum as the `TX_DONE` timestamp. The
port writes it straight into `TX_TIME.TX_STAMP`, then derives
`TX_TIME.TX_RAWST` by subtracting the node's own `TX_ANTD` register
from it, recovering exactly the pre-antenna-delay, already-rounded
value, which the port asserts:

```c
#define DW1000_CLOCK_ROUNDUP(x) (((x) + 511) & (~0x1FF))
/* ... */
DW1000_ASSERT(tx_time == DW1000_CLOCK_ROUNDUP(tx_time),
              "transmit timestamp on a 512-tick boundary");
```

For **RX**: the server computes the arrival instant from propagation
delay and the receiving node's own clock (adjusted for its configured
clock drift, if any), and returns it as the `RX` timestamp with no
antenna-delay adjustment of its own. The port writes that value
straight into `RX_TIME.RX_STAMP`, then derives `RX_TIME.RX_RAWST` by
adding the node's own receive antenna delay (`LDE_IF.LDE_RXANTD`) and
rounding up with `DW1000_CLOCK_ROUNDUP()`. The port's own comment
calls this "building a fake one": it lands `RX_RAWST` on the right
512-tick grid, but it is not computed the way real DW1000 hardware
derives a raw first-path timestamp.

**Not established**: nothing in the source pins down what `RX_RAWST`
*should* be instead, beyond "on the 512-tick grid": the comment
flags it as needing improvement without saying what a correct value
would look like.


## What the model does, and what it does not

### The clock

The model keeps a device clock, and a medium server sharing the host
must keep the same one or the two halves cannot be compared. It is
`CLOCK_REALTIME` scaled to `DW1000_TIME_CLOCK_HZ` and truncated to 40
bits; in C,

```c
uint64_t ticks = ((uint64_t)ts.tv_sec * DW1000_TIME_CLOCK_HZ)
               + ((uint64_t)ts.tv_nsec * 638976ull) / 10000ull;
return ticks & ((1ull << 40) - 1);
```

and in the reference server's Ruby, `(Time.now.to_r * DW1000_HZ).to_i &
DW1000_MASK`, which is the same number. `dw1000_emulation_clock()`
exports it, so a medium in the same process (as the tests are) can call
it rather than reimplement it.

`SYS_TIME` reads that clock, sampled on each host read. The reference
server applies a per-node clock *drift* that it is told about by its
controller and the node never learns, so a node configured with a
non-zero drift has a `SYS_TIME` running at a slightly different rate
from the timestamps it is handed. Nothing in the protocol carries that
figure to the node.

### Delayed send and receive

`DX_TIME` works, for both `TXDLYS` and `RXDLYE`, with `HPDWARN` and
`TXPUTE` decided the way UM §3.3 describes: on the internal
transmitter start time, which is the programmed time less the preamble
and SFD airtime, and not on the programmed time itself. Neither flag
cancels anything; the manual is explicit that a long delay may be
intended, and stopping it is the host's move by `TRXOFF`. A programmed
time already gone by is not refused either: the model waits for the
counter to come round to it, almost a whole period, exactly as the chip
does.

Both flags are **read-only and derived**, not latched (UM §7.2.17).
`HPDWARN` reads set while an armed delayed operation is still more than
half a clock period from starting, and clears by itself when the
operation is cancelled or when the counter catches up; `TXPUTE` reads
set only inside the few microseconds of transmitter power-up, which the
manual notes a host is unlikely ever to catch. Writing 1 to either does
nothing, here as on the chip. A model that latched them would refuse
every delayed operation after the first late one.

A delayed send's own timestamps are exact. UM §3.3 makes the RMARKER
the programmed time by construction, so `TX_TIME.TX_RAWST` is `DX_TIME`
with its low nine bits cleared and `TX_STAMP` is that plus `TX_ANTD`;
the model writes both itself and discards the server's stamp.

**The other nodes' view is not exact.** The server works out every
receiving node's arrival time from the instant the `TX` request reached
it, and that instant carries however long the sending node's deadline
thread took to wake up and get the datagram out, tens of
microseconds, which is metres. A node's own registers are corrected for
this and its peers' are not, so a two-way ranging run over delayed
sends will not close as tightly here as on hardware. Closing it needs
the node to tell the server the RMARKER it programmed, and no message
in this protocol has a field for it. That is the one extension worth
making, and it has not been made: it would change the contract this
page defines, and the reference server would have to change with it.

### Receive timeouts

`RXRFTO` and `RXPTO` both work, counting from the moment the receiver
turns on, which for a delayed receive is when the counter reaches
`DX_TIME`, per UM §7.2.40.9. `RX_FWTO`'s unit is exactly 65536 device
ticks (512 counts of the 499.2 MHz clock, UM §7.2.14) and `DRX_PRETOC`
is `(value + 1)` PACs, the PAC size recovered by matching `DRX_TUNE2`
against the eight tuning words the manual documents.

`RXSFDTO` is **not** modelled, and cannot be. UM §7.2.40.7 starts that
counter at preamble detection and ends it at SFD detection, and this
model has neither: a frame arrives whole from the medium server or does
not arrive. For the same reason `DRX_PRETOC`'s countdown here is never
suspended by an unconfirmed preamble detection, which UM §7.2.40.9 says
real hardware does for at least 1 PAC + 32 symbols.

### The double receive buffer

The swinging set of UM table 7 works: `HSRBP` and `ICRBP`, the `HRBPT`
command, per-buffer `LDEDONE`/`RXDFR`/`RXFCG`/`RXFCE`, a good CRC
moving the IC pointer and a bad one not, and `RXOVRR` when a frame
arrives with both buffers still held by the host. `DIS_DRXB` selects it
and `RXAUTR` re-enables the receiver between frames, with the two
meanings UM §7.2.6 gives it, which are not the same: double buffered it
re-enables after "a frame reception event or failure", single buffered
only after "a frame reception failure". So a *good* frame stops a
single-buffered receiver and does not stop a double-buffered one. A
frame wait timeout never re-enables, in either mode.

The swinging bits are not simply that buffer's latch (DW1000.md, "With
the buffer pointers aligned, the swinging status bits are the chip's
flags, not the buffer's", measured 2026-09-18). Whenever double
buffering is on and the two pointers are on one buffer with nothing
outstanding, a host read of `SYS_STATUS` shows in `LDEDONE`, `RXDFR`,
`RXFCG` and `RXFCE` the chip's own record of its last completed
reception rather than the set the host is on, and a write-one-to-clear
of those bits leaves that record standing: it is set when a reception
completes, reset by the next receiver enable, and untouched by `TRXOFF`,
by a status write and by `HRBPT`. Nothing outstanding is the measured
case, the IC parked on the buffer a toggle has just released; the other
way the pointers come together is the wrap-around of UM §4.3.5, both
buffers held by the host, and there the host's set is that buffer's own
latch as before. It is in `tests/emulation/dblbuff.c`, step
`step_stale_frame_told_apart`.

The LDE run is a phase of its own when
`dw1000_emulation_lde_delay(e, ticks)` is given a non-zero length, and
atomic (the default, and what a reset restores) when it is not. With a
length set, a reception completes in two steps: at the frame's arrival
the payload reaches `RX_BUFFER`, `RX_FINFO` is written and
`RXPRD|RXSFDD|RXPHD|RXDFR` stand with `RXFCG` or `RXFCE`, while
`LDEDONE` stays clear, `RX_TIME`, `RX_TTCKI` and `RX_TTCKO` keep what
they held, and `ICRBP` does not move; `ticks` device clock ticks later
the run finishes and writes all of that. A `TRXOFF` in between cancels
it like every other deadline the model holds, and none of it ever
happens, which is the cut of DW1000.md, "A TRXOFF between RXFCG and
LDEDONE leaves the frame without its timestamp, and the IC pointer
where it was" (measured 2026-09-19).

The cut is all the knob claims. What the bench measured is that a
reception a `TRXOFF` terminates after its payload and its CRC are in
posts `RXFCG` and never posts `LDEDONE`; whether an untouched run ends
before or after `RXFCG` is not measured, and the step is not a claim
that it ends after. The knob gives the run a length only so that a
`TRXOFF` has somewhere to fall inside it. It is in
`tests/emulation/dblbuff.c`, `step_lde_cut_frame_is_an_error`, which
asks that the driver report such a frame through `rx_error` with
`RXFCG` set and `LDEDONE` clear, call `rx_ok` not at all, and leave
both buffer pointers where they were.

Nothing is sent to the medium server when the receiver comes back
through `RXAUTR`. A server that treats `RX_CONFIG` as "the receiver is
now on" will therefore think a node has stopped listening after its
first frame; the reference server applies no behaviour to `RX_CONFIG`,
which is why this is sound here. A host re-enabling the receiver
explicitly does send one, even if the receiver was already on.

### Still not modelled

- **The masked clear that waits for the toggle.** DW1000.md, "With the
  buffer pointers aligned, the swinging status bits are the chip's
  flags, not the buffer's" (measured 2026-09-18): with the receiver
  enabled and the two pointers apart, a host's masked clear of `RXDFR`,
  `RXFCG` and `LDEDONE` does not take, the bits read back set and go
  only with the `HRBPT` toggle, while `RXPRD`, `RXSFDD` and `RXPHD`
  clear as written. Here that clear takes, as it always did. What the
  measurement does not say is what becomes of the interrupt line, and
  `IRQS` being the OR of the masked status bits (UM §7.2.17) the model
  cannot hold the bits set without also holding the line high: a model
  that did so stopped raising edges for later frames, since nothing
  ever cleared them again.

- **Frame filtering.** `FFEN` and the `FF*` bits are stored and
  ignored, so `AFFREJ` is never raised and no frame is ever rejected.
  `EUI` and `PANADR` have no storage at all. UM §5.2.2 makes filtering
  interact with the timeouts ("any frames rejected will stop the
  reception ... timeout will not trigger") and with the double buffer
  (a filtered frame does not move `ICRBP`), so this is the largest
  single gap left.
- **Automatic acknowledgement**: `AUTOACK`, `AAT`, `ACK_RESP_T`.
- **Sleep and wake**: the `WAKEUP` IO line still aborts the process,
  and `AON` is storage only.
- `RX_TTCKI` and `RX_TTCKO` are still the constants `0x1000` and `0`.
- `RX_FINFO`'s `RXPACC`, `RXPSR`, `RXBR` and `RXNSPL`, and all of
  `RX_FQUAL`: there is no signal model behind them, so a receive power
  estimate computed from this port is meaningless rather than merely
  imprecise.
- **The LDE itself.** `dw1000_emulation_lde_delay()` gives the run a
  length and nothing else: there is no leading-edge algorithm behind
  it, `LDEERR` is never raised, `LDE_THRESH` and `LDE_RXANTD` are read
  but only the antenna delay is used, and the length is whatever the
  knob was told rather than anything derived from the frame. What the
  knob models is a window between `RXFCG` and the run's two products
  (`RX_TIME` and `ICRBP`), which is what a `TRXOFF` across it takes
  away; the window's existence on the chip is not itself a measurement.
- `RX_TIME.RX_RAWST` is still derived by adding the receive antenna
  delay to `RX_STAMP` and rounding to the 512-tick grid: on the grid,
  but not how hardware derives a first-path timestamp.
- Registers attached as placeholders, with storage but no behaviour:
  `EUI`, `PANADR`, `SYS_STATE`, `RX_SNIFF`, `ACK_RESP_T`, `ACC_MEM`,
  `DIAG` (so none of the `EVC_*` event counters count).
- Registers stored and read back but driving nothing: `CHAN_CTRL`
  (except `RXPRF`, which sizes a preamble symbol), `USR_SFD`,
  `AGC_CTRL`, `EXT_SYNC`, `RF_CONF`, `FS_CTRL`, `AON`, `PMSC`,
  `TX_POWER`, `OTP_IF`, and `DRX_CONF` except `DRX_PRETOC`,
  `DRX_TUNE2` and `RXPACC_NOSAT`.
- Errata 1.4 is not modelled at all: neither the TX-1 silent window,
  nor the RX-1 corruption of the second buffer's 129th octet, nor the
  IRQ-1 glitch.
- **Overrun corruption.** UM §4.3.5 says an overrun corrupts the frames
  already received (`RX_FINFO`, `RX_TIME` and `RX_FQUAL`), and that
  they must be discarded. Here the two buffered frames survive an
  overrun intact, so a host that reads them anyway gets away with it.
- `SYS_TIME`'s low 9 bits read as zero on the chip (UM §7.2.8); here
  they carry the full resolution of the host clock.
- `W4R_TIM`, the programmable turnaround delay of `ACK_RESP_T`: a
  `WAIT4RESP` receiver comes on immediately rather than after it. The
  receive timeouts do start at that turn-on, as they should.

### Threads

Three threads reach the model: the host's, through the SPI and IO line
calls; the rsvc reader's, delivering frames from the server; and the
model's own deadline thread, which is what meets a programmed time. All
three take the model's mutex for register access, and none of them
holds it across the line callback.

**The line callback must not call the driver.** Not "should not": it
hangs. The callback runs on the rsvc reader thread, and a driver call
that enables the receiver or starts a transmit reaches `rsvc_i()`, which
blocks waiting for a reply datagram. The only thread that reads
datagrams and matches replies is the rsvc reader: the thread now sitting
inside the callback. Nothing can wake it.
`dw1000_process_events()` takes that path in double-buffered mode, so
this is the ordinary case and not a corner of it. Signal a condition
variable from the callback and call the driver from the node's own
thread, as all three tests do.

An earlier revision of this page said the opposite, on the strength of a
change that removed a *different* deadlock: the callback used to run
while the model held its own mutex, and no longer does. Removing one of
two independent reasons for a prohibition does not lift it.

**A slow medium server stalls every deadline.** One thread serves them
all, and when it delivers a delayed send it blocks in `rsvc_i()` until
the server replies. A server that takes a millisecond to answer a `TX`
delays every timeout and every other programmed time by that much. Reply
promptly and do the work afterwards.

**Every request must be answered.** A node sends `TX` and `RX_CONFIG` as
ordinary request/reply calls and waits for the reply; there is no
fire-and-forget. A server that does not recognise a service type must
answer it with a non-zero status rather than ignore it; the reference
server does, by rescuing the exception its dispatch raises and replying
with an error. The wait is bounded (five seconds by default,
`rsvc_set_reply_timeout()` to change it), so silence is now reported
rather than fatal, but the bound exists to diagnose a broken server and
not to tolerate a slow one: a node that reaches it has already stopped
behaving like a radio. On a timeout the call returns `RSVC_ERR_TIMEOUT`,
the model returns to idle, and it says so once until the server answers
again.

Because the model owns a thread, shutting down has an order, and none of
it commutes: the model's thread calls the connection and the
connection's thread calls the model:

1. `dw1000_emulation_stop(e)` joins the model's deadline thread;
   after it, the model makes no further calls on the connection.
2. `rsvc_close(rsvc)` joins the reader; after it, no frame can arrive
   in the model.
3. `dw1000_emulation_destroy(e)` frees it.

Step 1 is done for you if you skip it, which is safe only when nothing
is arriving. Step 2 is not, and destroying the model with the connection
still open leaves the reader able to deliver a frame into freed memory.
None of the three may be called from the line callback.

## Using it from a node

`dw1000_emulation_create(rsvc, line_cb, line_args)` builds the
register model over an already-open `rsvc_t *` and registers the
`RSVC_UWB_IO` handler described above. The returned
`struct dw1000_emulation *` is what ties the OSAL glue to one model
instance: a node assigns it to the `.emulation` field of its IRQ
ioline, its RESET ioline, and its SPI driver struct, e.g. (from
`simulation/main.c`):

```c
emulation = dw1000_emulation_create(rsvc, dw1000_line_cb, NULL);

dw1000_ioline_irq  .emulation = emulation;
dw1000_ioline_reset.emulation = emulation;
DW0_spi            .emulation = emulation;
```

`_dw1000_ioline_set/clear()` and `_dw1000_spi_send/recv()` read that
field to find which model instance a call is for. `line_cb` is
called with `DW1000_IOLINE_IRQ` whenever the model sees
`SYS_STATUS.IRQS` transition low to high: it stands in for a real IRQ
line's edge, and a node is expected to schedule its interrupt
processing from there rather than do it on the callback's own stack.
It may arrive on the rsvc reader's thread or on the model's own
deadline thread; see Threads above.

Before any of this, a node typically opens the connection with
`rsvc_open()` (which performs the `RSVC_OPEN` call itself) and then
calls `RSVC_SEED_GET` once to obtain a seed for its own random number
generator, as `simulation/main.c` does:

```c
rsvc = rsvc_open(socket_path, nickname, NULL);
/* ... */
uint32_t seed;
size_t   seedlen = sizeof(seed);
rsvc_o(rsvc, RSVC_SEED_GET, &seed, &seedlen);
```

`tests/emulation/` holds three programs, each both halves in one:

- `smoke.c`: a medium that answers every request and loops a node's
  own frame back at it, and a node that runs the driver against it:
  transmit, receive, a damaged FCS, a ranging frame. The shortest
  worked example of the protocol above.
- `timing.c`: a medium that stamps with `dw1000_emulation_clock()`
  and holds its frames until asked, for `SYS_TIME`, delayed send and
  receive, and the two receive timeouts.
- `dblbuff.c`: a medium that sends a burst of numbered frames on one
  `RX_CONFIG`, for the swinging set and the overrun.

`sh tests/check-emulation.sh` builds and runs four tests out of those
three files, needing nothing installed and no server started: `dblbuff.c`
is built twice, the second time with proprietary long frames on, because
errata 1.4 RX-1 needs a TX write past index 127 and the 127 byte
standard frame cannot reach it.

## The two traces

`DW1000_EMULATION_DEBUG` turns on the model's running commentary
(`EMU_DEBUG` in `emu_log.h`): state changes, buffer toggles, deadlines,
frames in and out. `DW1000_EMULATION_SPI_TRACE` is separate and prints
one line per host SPI transfer instead, reads and writes both, in the
fixed shape `dw1000-emulation: spi W reg=0x0f off=0 len=4 data=...`
(`W` for a write, `R` for a read, the bytes as the host wrote or got
them, the live view of `SYS_STATUS` included). Both are off by default;
each is set on the compiler line on its own, `-DDW1000_EMULATION_SPI_TRACE=1`
for this one. What it is for is the equivalence check of a driver
refactor: the writes are everything the driver does *to* the chip, so
two builds whose `spi W` lines match, in order and in bytes, do the same
thing to it, and a restructured `dw1000_process_events()` can be held to
that rather than to the tests alone.
