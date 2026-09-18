# SYS_STATE comparison: bench readings vs. DW1000 User Manual and APS022

## 1. Document revisions found

- `/home/sdalu/Repos/decawave-drivers/docs/DW1000 User Manual.pdf` — title page reads
  `© Decawave Ltd 2017   DW1000 User Manual                   Version 2.18`.
  **Revision found: 2.18** (matches the expected revision).
- `/home/sdalu/Repos/decawave-drivers/docs/APS022_Debugging_DW1000_based_products_systems.pdf` —
  title page reads `Version 1.4` (`© 2024 Qorvo, US, Inc. – All Rights Reserved`).
  **Revision found: 1.4** (matches the expected revision).

Section used in the User Manual: **7.2.27 "Register file: 0x19 – DW1000 State Information"**
(pages 98–100), found directly — it is present in this 2.18 copy, so no register-name fallback
search was needed.

Sections used in APS022: **4.3 "The System States (SYS_STATE) register"** (p. 11), **4.4
"Overall system states"** / Table 2 (p. 11), **4.5 "Receiver states"** / Table 3 (pp. 11–12), and
**4.6 "Transmitter states"** / Table 4 (p. 12) — these match the requested 4.3–4.6 exactly.

## 2. Reading-by-reading comparison

Decode basis: register file 0x19 is a 32-bit word laid out (per UM 2.18 §7.2.27, p. 99) as
TX_STATE in bits 3–0, RX_STATE in bits 11–8, PMSC_STATE in bits 23–16, remaining bits reserved.
Byte layout of the 8-hex-digit readings below: byte3=bits31-24, byte2=bits23-16 (PMSC_STATE),
byte1=bits15-8 (byte containing RX_STATE), byte0=bits7-0 (byte containing TX_STATE).

| # | Reading | Driver's interpretation | PMSC_STATE name (manual) | TX_STATE name (manual) | RX_STATE name (manual) | Verdict |
|---|---|---|---|---|---|---|
| 1 | `0x00010000` | idle | `0x1` = **IDLE** ("DW1000 is in IDLE") | `0x0` = **IDLE** ("Transmitter is IDLE") | `0x00` = **IDLE** ("Receiver is in idle") | Agree |
| 2 | `0x00020000` | delayed send waiting for its DX_TIME | `0x2` = **TX_WAIT** ("DW1000 is waiting to start transmitting") | `0x0` = IDLE | `0x00` = IDLE | Agree |
| 3 | PMSC value `3` (no full word given) | first microseconds after the RXENAB command | `0x3` = **RX_WAIT** ("DW1000 is waiting to enter receive mode") | not provided in this reading — bench value given as PMSC field only | not provided in this reading — bench value given as PMSC field only | Agree (PMSC field only; TX_STATE/RX_STATE not assessable from the given data) |
| 4 | `0x00040001` | frame going out (driver reads TX_STATE=1, preamble) | `0x4` = **TX** ("DW1000 is transmitting") | `0x1` = **PREAMBLE** ("Transmitting preamble,") | `0x00` = IDLE | Agree |
| 5 | `0x4x050500` (x = 0..8), driver reads RX_STATE=5 | receiver enabled | `0x5` = **RX** ("DW1000 is in receive mode") | `0x0` = IDLE | `0x05` = **PREAMBLE_FND** ("Receiver is waiting to detect preamble") | Agree — see note on bits 31–24 below |

Note on reading 5: the varying nibble `x` sits in bits 27–24 of `0x4x050500`, i.e. inside the
byte the manual marks "Reserved" (bits 31–24, alongside PMSC_STATE's bits 23–16). Neither UM
2.18 §7.2.27 nor APS022 Table 2 documents any meaning for bits 31–24. **Manual's name for those
bits: not listed.**

Independent corroboration in APS022 §4.3 (p. 11): "if a RX enable command has been issued, the
states register will report 0x050500 (RX) and then once a frame has been received the register
will report 0x010000 (IDLE)." This matches readings 5 (low 24 bits `0x050500`) and 1
(`0x010000`, i.e. `0x00010000`) exactly.

## 3. Verbatim quotes

### DW1000 User Manual 2.18, §7.2.27 "Register file: 0x19 – DW1000 State Information"

Bit-map (p. 99):
```
REG:19:00 – SYS_STATE – System Information (octets 0 to 3)
31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0
       Reserverd          - - - - PMSC_STATE - - -          RX_STATE   Reserved TX_STATE
```

TX_STATE field definition (p. 99), verbatim:
> "TX_STATE Current Transmit State Machine value: Description
> reg:19:00 0x0 - IDLE Transmitter is IDLE
> bit: 3-0
> 0x1 - PREAMBLE Transmitting preamble,
> 0x2 - SFD Transmitting SFD
> 0x3 - PHR Transmitting PHY Header data
> 0x4 - SDE Tranmitting PHR parity SECDED bits
> 0x5 - DATA Transmitting data block (330 symbols)
>
> TX_STATE Reserved
> reg:19:00
> bit: 7-4"

RX_STATE field definition (p. 100), verbatim:
> "RX_STATE Current Receive State Machine value Description
> reg:19:00 0x00 - IDLE Receiver is in idle
> bit: 11-8
> 0x01 - START_ANALOG. Start analog receiver blocks
> 0x04 - RX_RDY Receiver ready
> 0x05 - PREAMBLE_FND Receiver is waiting to detect preamble
> 0x06 - PRMBL_TIMEOUT Preamble timeout
> 0x07 - SFD_FND SFD found
> 0x08 - CNFG_PHR_RX Configure for PHR reception
> 0x09 - PHR_RX_STRT PHR reception started
> 0x0A - DATA_RATE_RDY Ready for data reception
> 0x0C - DATA_RX_SEQ Data reception
> 0x0D - CNFG_DATA_RX Configure for data
> 0x0E - PHR_NOT_OK PHR error
> 0x0F - LAST_SYMBOL Received last symbol
> 0x10 - WAIT_RSD_DONE Wait for Reed Solomon decoder to finish
> 0x11 - RSD_OK Reed Solomon correct
> 0x12 - RSD_NOT_OK Reed Solomon error
> 0x13 - RECONFIG_110 Reconfigure for 110 kbps data
> 0x14 - WAIT_110_PHR Wait for 110 kbps PHR
> RX_STATE Reserved
> reg:19:00
> bit: 15-12"

PMSC_STATE field definition (p. 100), verbatim:
> "PMSC_STATE Current PMSC State Machine value Description
> reg:19:00 0x0 - INIT DW1000 is in INIT
> bit: 23:16
> 0x1 - IDLE DW1000 is in IDLE
> 0x2 - TX_WAIT DW1000 is waiting to start transmitting
> 0x3 - RX_WAIT DW1000 is waiting to enter receive mode
> 0x4 - TX DW1000 is transmitting
> 0x5 - RX DW1000 is in receive mode
> reg:19:00 Reserved
> bit: 31-24"

### APS022 v1.4, §4.3–4.6

§4.3 introductory text (p. 11), verbatim:
> "This register contains information on the current DW1000 state.
>
> The states are divided into 3 main types: TX, RX or IDLE and are shown in Table 2. Receiver
> states are shown in Table 3 while transmitter states are shown in Table 4 below.
>
> Using the system status and states register the user can check what state the DW1000 is in,
> e.g. if a RX enable command has been issued, the states register will report 0x050500 (RX) and
> then once a frame has been received the register will report 0x010000 (IDLE). The SYS_STATE
> register bits are described below:"

§4.4 Table 2 (p. 11), verbatim:
> "Table 2: Bits 23:16 PMSC_STATE Current PMSC State Machine value
>
> Bit Value  State     Description
> 0x0        INIT      DW1000 is in INIT
> 0x1        IDLE      DW1000 is in IDLE
> 0x2        TX WAIT   DW1000 is waiting to start transmitting
> 0x3        RX WAIT   DW1000 is waiting to enter receive mode
> 0x4        TX        DW1000 is transmitting
> 0x5        RX        DW1000 is in receive mode"

§4.5 Table 3 (pp. 11–12), verbatim:
> "Table 3: Bits 15:8 RX_STATE Current Receive State Machine value
>
> Bit Value  State              Description
> 0x00       IDLE               Receiver is in idle
> 0x01       START ANALOG       Start analog receiver blocks
> 0x04       RX READY           Receiver ready
> 0x05       PREAMBLE FIND      Receiver is waiting to detect preamble
> 0x06       PREAMBLE TO        Preamble timeout
> 0x07       SFD FOUND          SFD found
> 0x08       CONFIGURE PHR RX   Configure for PHR reception
> 0x09       PHR RX START       PHR reception started
> 0x0A       DATA RATE READY    Ready for data reception
> 0x0C       DATA RX SEQ        Data reception
> 0x0D       CONFIG DATA        Configure for data
> 0x0E       PHR NOT OK         PHR error
> 0x0F       LAST SYMBOL        Received last symbol
> 0x10       WAIT RSD DONE      Wait for Reed Solomon decoder to finish
> 0x11       RSD OK             Reed Solomon correct
> 0x12       RSD NOT OK         Reed Solomon error
> 0x13       RECONFIG 110       Reconfigure for 110 kbps data
> 0x14       WAIT 110 PHR       Wait for 110 kbps PHR"

§4.6 Table 4 (p. 12), verbatim:
> "Table 4: Bits 7:0 TX_STATE Bits 7:4 - Reserved, 3:0 Current Transmit State Machine value
>
> Bit Value  State      Description
> 0x0        IDLE       Transmitter is in idle
> 0x1        PREAMBLE   Transmitting preamble
> 0x2        SFD        Transmitting SFD
> 0x3        PHR        Transmitting PHR
> 0x4        SDE        Transmitting PHR parity SECDED bits
> 0x5        DATA       Transmitting data
> 0x6        RSP DATA   Transmitting Reed Solomon parity block"

## 4. Bit-position disagreements

- **Driver's `(value >> 16) & 0x1F` (PMSC_STATE as bits 20–16, a 5-bit field) vs. the manual.**
  UM 2.18 §7.2.27 (p. 100) states the PMSC_STATE field as `bit: 23:16` — an 8-bit field, three
  bits wider than the 5-bit mask (`0x1F`) the driver applies. APS022 Table 2 (p. 11) agrees with
  the User Manual: "Bits 23:16 PMSC_STATE". So both documents define PMSC_STATE as occupying
  bits 23–16, not bits 20–16 as the driver's mask implies. In practice this makes no numeric
  difference for any of the five bench readings, because bits 23–21 are `0` in all of them — but
  formally the driver's bit width disagrees with both documents.

- **RX_STATE bit width disagrees between the two documents themselves** (neither one against the
  driver, since the brief gives no explicit driver formula for RX_STATE bits): UM 2.18 §7.2.27
  (p. 99–100) states RX_STATE as `bit: 11-8` (a 4-bit nibble, with bits 15–12 marked
  "Reserved"), while APS022 Table 3 (p. 11) titles the field "Bits 15:8 RX_STATE" (the full
  byte). This is a manual-vs-application-note disagreement, not a driver disagreement. It does
  not affect any of the five readings above, since bits 15–12 are `0` in all of them, so both
  bit-width interpretations extract the same value (`0x5` for reading 5, `0x0` elsewhere).

- TX_STATE bit position agrees across both documents and the driver (bits 3–0).

## 5. Manual's name for PMSC value 3

**RX_WAIT** — UM 2.18 §7.2.27 (p. 100): `0x3 - RX_WAIT` "DW1000 is waiting to enter receive
mode". APS022 Table 2 (p. 11) agrees, calling it `RX WAIT` (no underscore) with the identical
description. It is listed, not "not listed".
