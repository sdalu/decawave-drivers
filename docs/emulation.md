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
node up. `FLG_INTERRUPT` marks a frame the server sends unprompted --
the `RX` and `TX_DONE` messages described below -- and the node
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

**RSVC_OPEN** -- sent once, with `FLG_INCLUDE_NICKNAME` set and no
payload, to register the node with the server under its nickname.
No reply payload.

**RSVC_CLOSE** -- sent once, no payload, no `FLG_INCLUDE_NICKNAME`
(the server looks the node up by its socket path instead); tells the
server to unregister it. No reply payload.

**RSVC_SEED_GET** -- sent with no payload. The reply carries one
4-byte, native-endian `uint32_t`: a random seed for the node to draw
on.


## The UWB channel: UWB_OPEN, UWB_CLOSE, UWB_IO

`RSVC_UWB_OPEN` and `RSVC_UWB_CLOSE` are defined in the protocol, and
the reference medium server implements handlers for both -- each
takes an 8-byte `drvid` as its only payload field and replies with no
data -- but nothing in this port ever sends either one. In this
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
byte-swapped for the wire -- native order throughout.

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
number. A DW1000 timestamp is a 40-bit quantity -- the register
fields it lands in (`RX_TIME.RX_STAMP`, `TX_TIME.TX_STAMP`, and the
`_RAWST` pair) are written 40 bits at a time -- carried on the wire
in a full 8-byte field, upper 24 bits unused.

For **TX**: the server rounds its own send instant up to the next
512-tick boundary, adds the `antenna_delay` the node supplied in its
`TX` request, and returns that sum as the `TX_DONE` timestamp. The
port writes it straight into `TX_TIME.TX_STAMP`, then derives
`TX_TIME.TX_RAWST` by subtracting the node's own `TX_ANTD` register
from it -- recovering exactly the pre-antenna-delay, already-rounded
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
*should* be instead, beyond "on the 512-tick grid" -- the comment
flags it as needing improvement without saying what a correct value
would look like.


## What the register model does not do

Taken from the comments in the source, not inferred:

- The double-buffer swing set: the file's own header comment is
  `TODO: correctly implement swing-set for double buffer use.`
- On every received frame, `SYS_STATUS.RXPTO`, `RXSFDTO` and `RXRFTO`
  are unconditionally cleared, each commented `// Not emulated`.
- Also on receive: `AFFREJ`, `AAT`, `RXOVRR`, `RXRSCS` and `RXPREJ`
  are commented `// Not handled: AFFREJ AAT RXOVRR RXRSCS RXPREJ`.
- `RX_TTCKI` and `RX_TTCKO` are written as fixed constants
  (`0x1000` and `0`, respectively), each commented
  `// Fake value, need improvement`.
- `RX_FINFO`'s `RXPACC`, `RXPSR`, `RXBR` and `RXNSPL` fields are
  commented `// Not supported`.
- `RX_TIME.RX_RAWST` is a "fake one" on the right clock grid, not a
  true raw timestamp -- see Timestamp convention above.
- Delayed send: `dw1000_emulation_send()` ends with the comment
  `// Delayed send: TXPUTE HPDWARN` and no code behind it.
- The `WAKEUP` IO line is unimplemented: `_dw1000_ioline_set()`
  calls `EMU_FATAL("unimplemented: wakeup line not implemented")`, so
  driving it aborts the process.
- Ten registers are attached only as placeholders
  (`E_REG_ATTACH(e, ..., TODO)`), storage exists but nothing gives
  them behaviour: `EUI`, `PANADR`, `SYS_TIME`, `SYS_STATE`,
  `DX_TIME`, `RX_FWTO`, `RX_SNIFF`, `ACK_RESP_T`, `ACC_MEM`, `DIAG`.
- Several more are attached and readable/writable as plain storage,
  but their content drives nothing in the model, each marked
  `// ignored` at the attach site: `CHAN_CTRL`, `USR_SFD`,
  `AGC_CTRL`, `EXT_SYNC`, `RF_CONF`, `FS_CTRL`, `AON`, `PMSC` --
  plus `DRX_CONF` ("ignored except `DW1000_OFF_DRX_RXPACC_NOSAT`")
  and `OTP_IF` ("ignored except...", unfinished in the source
  comment itself). `TX_FCTRL`, `TX_ANTD` and `TX_POWER` carry an
  inline `// todo` at their attach line.


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
called with `DW1000_IOLINE_IRQ` whenever the model's
`e_raise_interrupt()` sees `SYS_STATUS.IRQS` transition low to high
-- it stands in for a real IRQ line's edge, and a node is expected to
schedule its interrupt processing from there rather than do it on the
callback's own stack.

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

`tests/emulation/smoke.c` is the whole of both halves in one program: a
medium that binds the socket, answers every request and loops a node's
own frame back at it, and a node that runs the driver against it --
transmit, receive, a damaged FCS, a ranging frame. It is the shortest
worked example of the protocol above, and the smallest one that runs:
`sh tests/check-emulation.sh` builds and runs it, needing nothing
installed and no server started.
