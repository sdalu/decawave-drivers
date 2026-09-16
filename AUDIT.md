# Audit against the DW1000 User Manual and Errata

Every SPI register access in the driver core was checked against the
DW1000 User Manual, and the operations checked for coherence with one
another. Recorded here so the next reader does not repeat the work.

- **Tree audited**: commit `117a784` (v1.2.0-1), `hw/drivers/dw1000/`
  only: `src/dw1000.c`, `src/dw1000_send.c`, the inline API in
  `include/dw1000/dw1000.h`, all of `dw1000_reg.h`, and the OTP
  addresses in `dw1000_otp.h`.
- **References**: User Manual 2.12 (every section and page number
  below is 2.12 unless stated), cross-checked with 2.15 and, in a second
  round, with **2.18** (249 pages, the current revision); Errata 1.4
  (April 2021); application note **APS022 v1.4** (Qorvo, 2024). The PDFs
  sit in `docs/`, which git ignores. For the code-to-code comparison,
  Decawave's own uwb-dw1000 at `9503f2a` (30 September 2020, the last
  upstream commit), checked out at `~/Repos/uwb-dw1000`.
- **Dates**: 2026-09-14, against 2.12 and 2.15; same day, a second round
  against 2.18 and APS022 once those PDFs arrived.
- **Not audited**, and still not: the OSAL ports (seven of them now,
  `port/emulation` having landed since), the Zephyr and MyNewt glue, any
  consumer of the driver, and whether an errata past 1.4 exists.

## Status

Every finding the first round raised has been fixed (`3a6a44f`) except
the proprietary SFD at 6.8 Mbps. The 2.18/APS022 round added five more,
of which four are fixed and one (the channel 5 analogue values, which
is a decision rather than a defect) is under **Open** below, with the
rest of what remains. Fixed findings are kept as a one-line ledger
under **Fixed**; their reasoning now lives in the code, at the sites
named there.

The first round's patch was verified against the manual, the errata and
the vendor driver independently of the person who wrote it; `make lib
OSAL=null`, `make check-options` and `make check` (including the
`port/emulation` smoke test) pass, and `clang --analyze` reports nothing.
The option matrix was 256 combinations when the audit ran and is 128
now, `DW1000_WITH_DWM1000_EVK_COMPATIBILITY` having been dropped in
`c58f953`.

## Open

What this audit did not cover in the first place is the **Not audited**
bullet above; it is still uncovered.

### Errata RX-1 is neither enforced nor documented

A TX buffer write past offset 127, issued before the second RX buffer is
read out, corrupts byte 128 of that buffer. Nothing in the driver
mentions it. It bites a double-buffered node answering with a frame
longer than 127 bytes. Either reject or split such a write in
`dw1000_tx_write_frame_data()`, or state the constraint in the header.
The only erratum item still uncovered; see the Errata 1.4 table.

### Channel 5 RF_TXCTRL and TC_PGDELAY are the pre-2.16 values

- **Location**: `src/dw1000.c`, `channel_tunning[]`, the channel 5 row.
- **Bug**: the manual changed both of that row's analogue values after
  2.12, and the driver still carries the old ones.

  | Register | driver | 2.12 | current | changed in |
  | :------- | :----- | :--- | :------ | :--------- |
  | RF_TXCTRL (0x28:0C) | `0x001E3FE0` | `0x001E3FE0` | `0x001E3FE3` (Table 38) | 2.16 |
  | TC_PGDELAY (0x2A:0B) | `0xC0` | `0xC0` | `0xB5` (Table 40) | 2.18 |

  2.18's change log gives the reason for the second: "TC_PGDELAY setting
  for channel 5 updated to 0xB5, maximising power in CH B/W". The
  RF_TXCTRL delta sets bits 1:0, which fall in the field 2.18 describes
  as "reserved … program only as directed in Table 38". Both values were
  read off the rendered page; note the table's channel 5 cell is
  literally typeset `0x00 1E3FE3`, a stray space in Decawave's own
  document, and 2.18's bit diagram just below it still draws the old
  `…E0`.
- **Impact**: the transmit spectrum on channel 5 only. Nothing else in
  the driver depends on either value, and channels 1, 2, 3, 4 and 7 are
  unchanged in both tables.
- **Fix**: *not applied; your call.* uwb-dw1000 (September 2020, well
  after both manual revisions) still has `RF_TXCTRL_CH5 0x001E3FE0` and
  `TC_PGDELAY_CH5 0xC0`, so adopting the new values puts this driver
  alone among DW1000 drivers, and TC_PGDELAY is the one value the manual
  warns "may need to be tuned for spectral regulation compliance
  depending on external circuitry". Following the current manual argues
  for changing both; matching every deployed driver argues for neither.

### Proprietary SFD at 6.8 Mbps uses the Decawave RXPACC adjustment

- **Location**: `src/dw1000.c`, `_dw1000_radio_tuning()`, the
  `rxpacc_adj` selection.
- **Bug**: the DWSFD field text (§7.2.32, p. 112) ends "For 6.8 Mbps the
  standard 8-symbol SFD", and no table defines DWSFD=1 at 6.8 Mbps. The
  driver applies the Decawave-8 adjustment (-10) where the standard-8
  value is -5 (Table 18, p. 96). The manual is ambiguous on what the
  chip does. The cost is a five-symbol error in the RXPACC used by the
  power estimate when no saturation occurred, about 0.3 dB at N = 128;
  interoperation between nodes running this driver is unaffected.
- **Fix**: *not applied.* Two facts cut the other way. Table 18 does
  carry a Decawave 8-symbol row (-6+1-5 = -10), untagged by bitrate, so
  "no table defines DWSFD=1 at 6.8 Mbps" above is too strong; and both
  Decawave drivers program a Decawave SFD of length 8 at 6.8 Mbps
  (uwb-dw1000 `DW_NS_SFD_LEN_6M8` = 8), reading the parenthetical as
  "the Decawave SFD is 8 symbols long there" rather than "DWSFD is
  ignored". That reading is also the only one consistent with setting
  TNSSFD/RNSSFD alongside DWSFD: were DWSFD ignored at 6.8 Mbps, those
  two would select the user-configured SFD of Table 22, whose sequence
  bytes this driver never programs. Until the measurement below, the
  driver keeps -10 and says why in the code.

### The vendor policy differences are not documented anywhere but here

The six entries under *Policy differences, to document rather than
change* exist only in this file. `README.md` says nothing about
DIS_STXP, the FCS included in the reported frame length, the
IDLE-before-TXSTRT precondition, or who re-enables the receiver after a
frame or an error.

### Settled: the delayed-send floor is airtime, not power-up

Measured on rpi-a (2026-09-15), the smallest lead the chip accepts
against the airtime the driver computes, three runs per configuration:

| preamble | Ton computed | min lead measured | residual |
| :------- | -----------: | ----------------: | -------: |
| 128  | 138.4 µs  | 173.4 µs  | 35.0 µs |
| 256  | 268.7 µs  | 303.4 µs  | 34.7 µs |
| 1024 | 1050.2 µs | 1085.1 µs | 34.9 µs |
| 4096 | 4176.3 µs | 4211.3 µs | 34.9 µs |

The floor tracks the preamble over a 30x range while the residual stays
flat, so it is the preamble and SFD airtime and not the transmit
power-up, which would not scale. It also checks the table 60 arithmetic
behind `dw1000_tx_get_preamble_airtime()`: computed Ton predicts the
hardware boundary to a constant within 0.3 µs in four configurations.
The residual holds the host's register writes and the power-up together;
this cannot separate them, but bounds both at about 35 µs on that host.

### Two measurements would settle two claims

- **The TX-1 band.** The ruby-dw1000 binding exposes no raw register
  read, so HPDWARN and TXPUTE cannot be told apart in a refusal;
  whether the fix removes the silent band or merely moves it into the
  later, reported region is open. The erratum's wording argues for
  removal, and either way the driver no longer claims success for an
  unsent frame.
- **The SFD at 6.8 Mbps.** RXPACC against RXPACC_NOSAT, with and
  without DWSFD, decides between -10 and -5.

### Latent, not defects

- The sleep pieces are inert only because no sleep entry point exists:
  `dw->sleep_mode` is accumulated and never written to AON_WCFG,
  AON_CFG1 is cleared but never uploaded with UPL_CFG, and TX_ANTD is
  not preserved (§7.2.26). A trap for whoever adds one.
- Vendor features we lack, as backlog: the carrier-integrator clock
  offset (DRX_CAR_INT), frame duration helpers, sleep and wake, the
  event counters (0x2F).
- SYS_STATE (0x19) is defined in the register map and never read. 2.18
  documents its fields and APS022 §4.3-4.6 tabulates them, so the IDLE
  preconditions the driver can currently only assert in prose
  (`dw1000_rx_set_timeout()`, TXSTRT) could be checked: PMSC_STATE
  reads 0x1 for IDLE.
- The `wait4resp` clear in the RXFCG branch makes the §5.4 workaround in
  the TXFRS branch dead code, now a second time over through the AUTOACK
  gate. Deliberately left undocumented in the code, as moot.

## Fixed

Severity as it was rated: consequence and reachability, not how
interesting the defect was.

| Sev | Defect | Where it was fixed |
| :-- | :----- | :----------------- |
| High | Errata TX-1 not worked around: a delayed transmit could silently never happen | `_dw1000_tx_clock_force()`, `dw1000_tx_start()`, `dw1000_process_events()` |
| Med | Double-buffered frame info read LDE_THRESH, which is not in the swinging set | `dw1000_process_events()`, `dw1000_rx_get_info()` |
| Med | Release-build frame-length clamp ignored the PHR mode | `dw1000_tx_fctrl()` |
| Med | Embed-timestamp without the delayed-start flag sent now with a future timestamp | `dw1000_tx_extended_vsendv()` |
| Med | A stale AAT tore down the receiver WAIT4RESP had just armed | `dw1000_process_events()`, TXFRS branch |
| Med | Frame filtering had no PAN id or short address to filter on | `dw1000_set_pan_id()`, `dw1000_set_short_address()` |
| Med | An overrun during the rx_ok callback was never seen, and never reset | `dw1000_process_events()`, RXFCG branch |
| Med | Antenna-calibration reference power wrong for channels 4 and 7 | `channel_prf_calibration[]` |
| Low | RXFCE cleared with interrupts unmasked in the error branch | `dw1000_process_events()`, RX error branch |
| Low | Clock-drift helper divided by RX_TTCKI with no zero guard | `dw1000_rx_get_clock_drift()` |
| Low | SOFTRESET issued immediately after the AON SAVE | `_dw1000_softreset()` |
| Low | Status bits a caller can unmask but the event loop never clears | documented on `dw1000_interrupt()` |
| Low | A preprocessor directive inside a macro argument | `dw1000_tx_fctrl()` |
| Low | Invalid embed-timestamp modes embedded nothing when asserts are off | `_dw1000_tx_prepare_delayed_embed_timestamp()` |
| Low | The proprietary SFD set DWSFD without TNSSFD and RNSSFD | `_dw1000_radio_tuning()` |
| Low | Late delayed receive cleared the transmit status group | `dw1000_rx_start()`, HPDWARN branch |
| Low | Documentation contradictions: README against the header, three missing preconditions, four wrong section numbers | `README.md`, `dw1000.h`, `dw1000.c` |

Then, from the 2.18 and APS022 round:

| Sev | Defect | Where it was fixed |
| :-- | :----- | :----------------- |
| Med | Manual TX power encoded with a 3 dB coarse step where 2.18 §7.2.31.1 says 2.5 dB: clamp 67 → 61, coarse step 6 → 5 half-dB units | `_dw1000_radio_tuning()`, and the `DW1000_TX_POWER*` docs |
| Low | The preamble timeout documented as a hard deadline, which 2.18 §7.2.40.9 says it is not | `dw1000_rx_set_timeout_preamble()` |
| Low | The LDECLK "reserved in the UM, and unnamed" comment covered seven documented bits | `dw1000_reg.h` |
| Med | The delayed-send lead ignored the preamble and SFD airtime (Ton) that must precede the RMARKER | `_dw1000_radio_tuning()` caches `dw->tx_ton` and `dw1000_tx_get_preamble_airtime()` exports it; the default lead is taken on top of it, an explicit one is used as given, and either is refused with -1 if it cannot be met |

**The TX-1 measurement**, the only one taken on hardware. ruby-dw1000
`errata-tx1/` counts delayed sends the driver accepts (`tx_start()`
returns 0) that never raise TXFRS; one node, no receiver, since the
symptom is sender side. On rpi-b, sweeping each build's own boundary at
0.1 µs, 200 attempts per lead, trials shuffled: 85 silent losses in 3347
eligible attempts before (`117a784`) against 0 in 3279 after
(`3a6a44f`). The band is about 0.4 µs wide, immediately above the lead
at which the chip stops refusing the send as too late, with the loss
rate running 85.7%, 52.7%, 18.3%, 4.9%, 0 at 159.2 to 159.6 µs. The
fix costs about 8.5 µs of extra lead (159.3 to 167.8 µs on rpi-b), of
which only ~1.6 µs is SPI clocking at 20 MHz and the rest spidev ioctl
overhead, so that delta belongs to the port; against the 400 µs default
lead, ~2%. The boundary itself is not a host figure: at the 128-symbol
preamble the sweep used, about 138 µs of it is the preamble and SFD
airtime APS022 calls Ton. Since that was measured, Ton is computed from
the radio configuration and exported, the default lead is taken on top
of it so that one constant serves every preamble length, and a lead that
cannot be met is refused with -1 before the frame is written, rather
than left to the chip to report as HPDWARN afterwards. The silent band
itself is gone: TX-1 is worked around for every delayed start, which is
what the 0 above measures.
The clock-rate excess measured while a send is pending
(ruby-dw1000, `delayed-send/README.md`) is unchanged with TXCLKS forced
on (2.57-3.01 ppm against 2.58-3.35 ppm), so the two effects are
unrelated.

## Refuted candidates

Kept here so they are not raised again.

| Candidate | Why it is not a defect |
| :-------- | :--------------------- |
| 850 kbps proprietary SFD sets DWSFD but not TNSSFD/RNSSFD; 2.12 Table 22 calls 1/0/0 the 8-symbol SFD | UM 2.15 §7.2.32 adds "DWSFD takes precedence over TNSSFD & RNSSFD. If DWSFD is set, the settings of TNSSFD & RNSSFD are ignored." DWSFD with SFD_LENGTH 16 is the 16-symbol SFD; the 2.12 table row is the self-contradiction. |
| GPIO_MODE written without GPCE/GPRN set (§7.2.39 note) | Vendor `dwt_setleds()` does the same; the LED clock is GPDCE, which is set. |
| Temperature/voltage read without ADCCE (§7.2.50.1) | Table 14 (§6.4), the manual's own procedure, omits it; the vendor omits it; SAR readings in ruby-dw1000 `temperature-effects/` track ambient in 1.14 °C steps. |
| 12 µs between clearing and setting SOFTRESET | The manual states no timing (§7.2.50.1, p. 196); the comment's "PLL lock" rationale is wrong but harmless. |
| RXENAB issued in double-buffered mode when RXAUTR already re-enabled the receiver | The manual is silent on RXENAB while already enabled. Suspected only. |
| No HSRBP == ICRBP test in the good-frame path (Figure 14) | Figure 14's arrow polarity contradicts §4.3.2; the driver relies on RXOVRR one frame later. Manual ambiguous. |
| CRC error in double-buffered mode causes a full TRXOFF and receiver reset | Matches the vendor ISR; the effect is visible through `rx_error`. |
| TXPUTE aborts a frame §7.2.17 says would still be on time | Settled the other way, in our favour: APS022 §5.4 requires it. "The TXPUTE event should always be checked, and if it is set, the transmission must be aborted (SYS_CTRL_TRXOFF issued) otherwise the DW1000 may never issue a TXFRS event." |

## Coherence between operations

- **SYS_CFG shadow.** All four writers (`dw1000_initialise()`,
  `dw1000_configure()`, `dw1000_rx_set_timeout()`,
  `dw1000_rx_set_frame_filtering()`) keep `dw->reg.sys_cfg` in step with
  the chip, so configuration preserves filtering and timeout bits and
  initialisation rebuilds the shadow after the reset.
- **Bring-up sequence.** Soft reset follows §7.2.50.1 exactly; the LDE
  load, OTP reads, LDO kick and XTAL trim all run under the forced XTI
  clock in the order Table 4 and §6.3 require, and the return to
  sequencing produces exactly 0x0200. `dw1000_configure()` relies on the
  shadow and on DIS_STXP set by `dw1000_initialise()`;
  `dw1000_tx_fctrl()` relies on the TX_FCTRL base cached by
  `dw1000_configure()`. Only the README states this order.
- **Transceiver off and event handling.** The mask, TRXOFF, clear,
  unmask order of Figure 15 holds everywhere, once the redundant RXFCE
  write was dropped. The double-buffer pointer sync is gated on dropping
  the good-frame bits, so a reported but unread frame is never handed
  back to the chip.
- **Delayed send.** DX_TIME is the raw RMARKER time with its low nine
  bits zeroed, the embedded value is DX_TIME plus TX_ANTD, and that
  equals what TX_STAMP reports (§3.3, §7.2.25). The retry path rewrites
  only DX_TIME and the timestamp, leaving buffer and frame control
  intact.

## Register map

Every `#define` in `dw1000_reg.h` (432 in total) was checked against the
manual: register file ids, sub-register offsets, bit positions, widths
and masks. No mismatch. Three notes:

- `DW1000_LEN_DRX_CONF` (46) and `DW1000_LEN_OTP_IF` (19) disagree with
  Table 2's 44 and 18, but Table 2 would exclude RXPACC_NOSAT and
  OTP_SF, which the driver uses. The driver is self-consistent and the
  constants are unused.
- `DW1000_FLG_PMSC_CTRL0_LDECLK` (bit 8) is reserved in §7.2.50.1 and
  corroborated only by Table 4's 0x0301/0x0200 values.
- The comment "Reserved in the UM, and unnamed" above LDECLK also covers
  seven documented bits (ADCCE to KHZCLKEN). Fixed: the comment now
  names LDECLK alone.

Every tuning table was compared entry by entry on the rendered pages: 30
channel values (FS_PLLCFG, FS_PLLTUNE, RF_TXCTRL, RF_RXCTRLH,
TC_PGDELAY), 12 manual TX power words, 24 LDE_REPC values, the PRF, PAC,
bitrate and preamble tables, and the five RXPACC adjustments. Against
2.12 all match, apart from the channel 4 and 7 calibration power, since
corrected. Against **2.18** two more differ, both in the channel 5 row —
RF_TXCTRL and TC_PGDELAY, and are under **Open**; no other table moved
between 2.12 and 2.18.

## Errata 1.4 coverage

| Item | Status in this driver |
| :--- | :-------------------- |
| TX-1 delayed TX may not complete | Worked around: the TX clock is forced on for a delayed send. Measured on hardware, 85 silent losses before against 0 after; see **Fixed**. |
| RX-1 byte 128 of the second RX buffer corrupted by a TX write past offset 127 before readout | **Not enforced or documented**; see **Open**. |
| TX-2 TX buffer index reset at TXSTRT | Not applicable; the driver offers no fast-turnaround write during transmission. |
| IRQ-1 IRQ glitch in double-buffered mode | Mitigated by the masked clears; `dw1000_pending_interrupt()` supports the poll-the-line workaround. |
| PMSC-1 wake-up event longer than 500 µs | Not applicable; no sleep entry point. `dw1000_hardreset()` holds WAKEUP high. |
| SYSSTAT-1 PLL lock bits unreliable | Not applicable; the driver never reads CPLOCK or CLKPLL_LL, and never calibrates the PLL. |

## Comparison with the Decawave driver

The core was compared function by function with uwb-dw1000
(`dw1000_dev.c`, `dw1000_phy.c`, `dw1000_mac.c`, `dw1000_otp.c`,
`dw1000_regs.h`), in six areas (register map and tables, bring-up,
radio configuration, transmit, receive and events, formulas), each
citing file and line on both sides. The register map, every field, bit
and mask, and every tuning table are byte-identical. Where the two
diverge, ours nearly always follows the User Manual or the older
deca_device.c, and uwb-dw1000 is the one that drifted. The comparison
added four findings above (PANADR, the overrun re-check, TNSSFD/RNSSFD,
the late delayed receive).

**Vendor differences that are the vendor's defects.** Not to be adopted:

| Item | uwb-dw1000 | Ours |
| :--- | :--------- | :--- |
| AGC_TUNE3 | never written (constant 0x0055, unused) | 0x0035, Table 26 |
| DRX_TUNE1b | 0x0010 for any rate with a 64-symbol preamble | Table 32: 0x0010 only at 6.8 Mbps |
| DRX_SFDTOC | fixed 129 from its config default | 1 + preamble + SFD − PAC (§7.2.40.7) |
| TC_PGDELAY | the channel 5 value for every channel, from its default config | per-channel table |
| XTAL trim | OTP value applied only when the config says 0xFF; its default is 0x10, so never | OTP first, config override, 0x10 fallback |
| AON_CFG1 at init | `reg \|= ~SMXX` sets the bits its comment says to clear | 0x0000 |
| LDECLK after the LDE load | left forced (PMSC_CTRL0 byte 1 stays 0x03) | 0x0200, Table 4 step L-3 |
| RXOVRR | written as 1 to clear | never written; §7.2.17 makes it read-only, HRBPT clears it |
| Clock drift sign | `-RXTOFS / interval` | `RXTOFS / interval`, the §7.2.22 worked example |
| Frame filtering across reconfigure | reset to the config default | preserved |

Two rows the vendor and this driver share, and the current manual does
not: `RF_TXCTRL_CH5` (`0x001E3FE0`) and `TC_PGDELAY_CH5` (`0xC0`) are
uwb-dw1000's values too, though it postdates both manual revisions that
changed them. Neither driver tracked the change; see **Open**.

**Where ours follows deca_device.c rather than uwb-dw1000**: the SAR
temperature read (RF_CONF 0x80, 0x0A, 0x0F then TC_SARC; uwb-dw1000 only
reads the wakeup sample), the AAT-plus-wait4resp workaround after TXFRS
(absent in uwb-dw1000; ours gates it on AUTOACK), the
per-frame TR bit, and the OTP read with no delay before OTP_RDAT.

**Policy differences, to document rather than change**:

- The vendor re-enables the receiver inside its ISR after a good frame
  in single-buffer mode and after every RX error; ours leaves that to
  the callback or to RXAUTR, and leaves the receiver off after an error
  or timeout.
- The vendor writes TRXOFF before every TXSTRT; ours requires IDLE as a
  documented precondition. Ours sets DIS_STXP at init; the vendor leaves
  smart TX power on. The frame length ours reports includes the FCS; the
  vendor's excludes it. Both say so.
- LDEERR is an RX error in ours (as in deca_regs.h) but only cleared in
  uwb-dw1000, which puts RXOVRR in that group instead. Our default
  SYS_MASK therefore never unmasks RXOVRR; an overrun always coincides
  with a pending RXFCG, so the IRQ line is asserted anyway.
- The vendor reads the frame, the timestamps and the diagnostics into
  its own buffers inside the ISR, then toggles HRBPT, then calls the
  user; ours calls the user first and expects the callback to read the
  frame, then toggles. Both are internally consistent.

**Only the vendor has**: the carrier-integrator clock offset
(DRX_CAR_INT), frame duration helpers, sleep and wake, the event
counters (0x2F). **Only ours has**: the delayed-send retry, SFCST, the
RXPACC SFD adjustment, the antenna-calibration table, the RX power
correction curve, the configuration validation in every build.

## Manual versions

2.12 was the reference for the first round. 2.15 (244 pages) lists in
its change log the edits after 2.12: LDE_CFG1 and LDE_CFG2 NLOS notes,
OTP Table 10 (part id in word 6), PLL2_SEQ_EN at PMSC_CTRL0 bit 24
(unused here), RXRFTO/LDEERR wording, the transmitter configuration
procedure, and the DWSFD precedence note.

**2.18** (249 pages, dated July 2019, the current revision) closes the
gap. Its change log gives the edits after 2.15, and every one was chased
down:

| Rev | Edit | Bearing on the driver |
| :-- | :--- | :-------------------- |
| 2.16 | Table 38, channel 5 default | **RF_TXCTRL `0x001E3FE0` → `0x001E3FE3`**; see **Open** |
| 2.16 | PRES_SLEEP defined (AON_WCFG) | None: the driver defines only the two ONW_* bits, and has no sleep path |
| 2.16 | PART/CHIP/LOT ID scheme, Appendix 4 | None; OTP decoding the driver does not do |
| 2.17 | HIRQ_POL "as part of 0x04" | A correction to §2.2.2, which had said 0x0D. The register map was right all along, and so is the driver |
| 2.18 | Fig 10, PHR 21 bits → 19 | None; the driver's "7-bit length" statement is about TFLEN, untouched |
| 2.18 | §7.2.31.1 coarse gain 3 dB → 2.5 dB, range 33.5 → 30.5 dB | **The manual TX power encoding**; see **Open** |
| 2.18 | 0x19 SYS_STATE described | An opportunity, not a defect: the driver defines the register, never reads it, and documents IDLE preconditions it cannot check. APS022 §4.3-4.6 tabulates the values |
| 2.18 | §7.2.40.9 preamble timeout comment | **The PTO is suspendable**; see **Open** |
| 2.18 | Table 40, TC_PGDELAY channel 5 → `0xB5` | **The other half of the channel 5 row**; see **Open** |
| 2.18 | NLOS note removed from LDE_CFG1/CFG2 | None: `lde_cfg2` comes from the PRF table and is never the 0x0003 the note named |

APS022 v1.4 (Qorvo, 2024, 23 pages) was read in the same round. It
settles the TXPUTE candidate in the refuted table, supplies the Ton
constraint on delayed sends, and tabulates SYS_STATE. Nothing in it
contradicts the driver.

The errata history ends at 1.4 (15 April 2021); whether a newer revision
exists is still unchecked.

## Tooling

- **Locally**: `make lib OSAL=null`, `make check-options` (256 of 256
  option combinations, `-Wall -Wextra`, zero warnings), `clang
  --analyze` on both sources (no reports).
- **On rpi-a** (Debian, LLVM 19.1.7, Cppcheck 2.17.1, GCC 12), over the
  core plus `port/null`: `gcc -O2 -Wall -Wextra -fanalyzer` reports
  nothing. `clang-tidy` (`clang-analyzer-*`, `bugprone-*`, `cert-*`,
  `misc-*`) gave the two `switch` warnings since fixed, plus two
  narrowing conversions checked to be range-safe (the temperature result
  fits `int16_t` for any 8-bit delta; the RX_TTCKO cast is the intended
  sign extension); the rest is `_dw1000_*` reserved-identifier naming
  and `memset`/`memcpy` advisories. `cppcheck --force
  --check-level=exhaustive` (`warning,style,performance,portability`)
  aborted on the directive inside the assert, since rewritten; with that
  assert rewritten on a scratch copy the whole tree was clean across all
  option configurations, bar the identity byte-swap macros on
  little-endian and two pointers that could be `const`.
- **Would widen the hunt**: `port/emulation` now carries a register
  model and `make check` runs its smoke test, which is most of the
  register-write recorder this audit asked for. What is left is turning
  findings into regression tests against it; `tests/` otherwise checks
  only the manifest and the option matrix.

## Method

Seven review pieces ran in parallel: the register map in two halves,
and five function groups (initialisation and clocks, radio
configuration, transmit, receive, event processing). Each cited the
code line and quoted the manual for every claim. Every High and Medium
was re-derived from the manual text, and for tables from the rendered
PDF page, before entering this file. A candidate the vendor driver
shares, that the manual contradicts elsewhere, or that measurement
refutes was moved to the refuted table rather than downgraded silently.

The second round, once the 2.18 and APS022 PDFs arrived, was narrower:
walk 2.18's change log entry by entry from 2.16 forward, read each cited
page, and check the driver against it; then read APS022 whole. Table
values were confirmed on the rendered page, not on extracted text; the
channel 5 RF_TXCTRL cell needed it, since Decawave typeset it with a
stray space. Every claim in that round is checked against the current
tree, not against the audited commit.
