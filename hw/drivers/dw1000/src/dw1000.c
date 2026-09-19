/*
 * Copyright (c) 2018-2024,2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/*
 * References to the decawave documentation is provided through
 * the code using:
 *  DS  = Datasheet
 *  UM  = User Manual (2.08)
 *  APS = Application Note
 *  MN  = MyNewt Decawave driver
 * 
 * Some references are also made to the official decawave driver:
 *  deca_device.c
 */



/*
 * TODO: Range bias
 *
 * For example see:
 *   https://github.com/bitcraze/libdw1000/blob/master/src/libdw1000.c
 */

// APS011: Sources of error in DW1000 based TWR schemes

// UM §8.3.1:
// For enhanced ranging accuracy the ranging software can adjust the
// antenna delay to compensate for changes in temperature. Typically
// the reported range will vary by 2.15 mm / 0°C and by 5.35 cm / Vbatt.

// UM §2.3.2 : For delayed TX/RX the receiver stays in IDLE mode
//             until transmission/reception time has been reached


/**
 * @file    dw1000.c
 * @brief   DW1000 low level driver source.
 *
 * @addtogroup DW1000
 * @{
 */


/*
 * UM §9.3  : Data rate, preamble length, PRF
 * UM §4.1.1: Preamble detection
 *
 * PLEN (Preamble length) / PAC (Preamble Acquisition Chunk)
 *   tx_plen: 64 | 128 | 256 | 512 | 1024 | 1536 | 2048 | 4096
 *   rx_pac : 8  | 8   | 16  | 16  | 32   | 64   | 64   | 64
 *
 * Bitrate / preamble length
 *  6800 kbps :   64 or  128 or  256
 *   850 kbps :  256 or  512 or 1024
 *   110 kbps : 2048 or 4096
 *
 * "UWB microsecond" unit is = 512/499Mhz = 1.026... µs 
 */




#include <math.h>
#include <string.h>
#include <inttypes.h>
#include "dw1000/osal.h"
#include "dw1000/dw1000.h"


/*===========================================================================*/
/* Local definitions                                                         */
/*===========================================================================*/

/*
 * Maximum header size for SPI transaction
 */
#define DW1000_SPI_HEADER_MAX_LENGTH 3

/*
 * Identification for DW1000 device
 */
#define DW1000_ID_DEVICE   0xDECA0130

/*
 * Clock configuration
 */
#define DW1000_CLOCK_SEQUENCING            0
#define DW1000_CLOCK_SYS_XTI               1
#define DW1000_CLOCK_SYS_PLL               2
#define DW1000_CLOCK_TX_CONTINOUSFRAME     3
#define DW1000_CLOCK_LDE_LOAD              4
#define DW1000_CLOCK_ACC_READ              5
#define DW1000_CLOCK_ACC_DONE              6



/*===========================================================================*/
/* Local variables and types                                                 */
/*===========================================================================*/

// Information tables
//----------------------------------------------------------------------

// Channel mapping to table index
static const int8_t channel_table_mapping[] = {
    -1, 0, 1, 2, 3, 4, -1, 5
};

#if 0
// Parameters according to channel
// UM §10.5: DW1000 has a maximum receive bandwith of 900 MHz
//           Channel 4 has a bandwith of 1331 MHz
//           Channel 7 has a bandwith of 1081 MHz
//           Other channels have a bandwith of 499 MHz
struct channel_table {
    uint16_t frequency;       // In 0.1MHz step
    uint16_t bandwidth;       // In 0.1MHz step
    uint8_t  pcode_16mhz[2];  // Recommended preambles for 16 MHz PRF
    uint8_t  pcode_64mhz[4];  // Recommended preambles for 64 MHz PRF
};
static const struct channel_table channel_table[] = {
    { 34944,  4992, {1,2}, { 9,10,11,12} }, // 1
    { 39936,  4992, {3,4}, { 9,10,11,12} }, // 2
    { 44928,  4992, {5,6}, { 9,10,11,12} }, // 3
    { 39936, 13312, {7,8}, {17,18,19,20} }, // 4
    { 64896,  4992, {3,4}, { 9,10,11,12} }, // 5
    { 64896, 10816, {7,8}, {17,18,19,20} }, // 7
};

// Allowed preamble code for "dynamic preamble select" for 64 MHz PRF
// UM §10.5: UWB channels and preamble codes
static const uint8_t pcode_64mhz_dps[] = {
    13, 14, 15, 16, 21, 22, 23, 24
};
#endif


// Preamble symbol duration, in device clock ticks
//----------------------------------------------------------------------

// UM table 60: the preamble symbol lasts 993.59ns at 16MHz PRF and
// 1017.63ns at 64MHz. Held in picoseconds so the conversion to ticks of
// DW1000_TIME_CLOCK_HZ stays integer; it truncates, which is the safe
// direction for a lead time the caller adds a margin to anyway.
#define _DW1000_PSYM_TICKS(ps)						\
    ((uint32_t)(((uint64_t)(ps) * DW1000_TIME_CLOCK_HZ) / 1000000000000ull))
#define DW1000_TICKS_PER_PSYM_16MHZ _DW1000_PSYM_TICKS( 993590)
#define DW1000_TICKS_PER_PSYM_64MHZ _DW1000_PSYM_TICKS(1017630)


// Internal calibration tables
//----------------------------------------------------------------------

// UM §8.3.1: Calibration method
//  -> Power at -41.3dBm and 0dBi antenna
struct _channel_prf_calibration { // [channel][DW1000_PRF_{4,16,64}MHZ]
    uint8_t  power;       // Power at receiver input (dBm/MHz) 
    uint16_t separation;  // Antenna separation in centimeters
};
static const struct _channel_prf_calibration channel_prf_calibration[6][3] = {
    // Note: 4MHz PRF is unsupported by DW1000
    //  4MHz  ,   16MHz      ,   64MHz
    { { 0, 0 }, { 108, 1475 }, { 104,  930 } }, // 1
    { { 0, 0 }, { 108, 1290 }, { 104,  814 } }, // 2
    { { 0, 0 }, { 108, 1147 }, { 104,  724 } }, // 3
    { { 0, 0 }, { 104,  868 }, { 104,  868 } }, // 4
    { { 0, 0 }, { 108,  794 }, { 104,  501 } }, // 5
    { { 0, 0 }, { 104,  534 }, { 104,  534 } }, // 7
};

// Internal tunning tables
//----------------------------------------------------------------------

// UM §7.2.31.4: Transmit Power Control Reference Values
struct _tx_power {
    uint16_t prf_16mhz;
    uint16_t prf_64mhz;
};
static const struct _tx_power manual_tx_power[] = {
    { 0x7575, 0x6767 }, // Channel 1
    { 0x7575, 0x6767 }, // Channel 2
    { 0x6F6F, 0x8B8B }, // Channel 3
    { 0x5F5F, 0x9A9A }, // Channel 4
    { 0x4848, 0x8585 }, // Channel 5
    { 0x9292, 0xD1D1 }, // Channel 7
};

// Tunning according to channel
// UM §7.2.44.2: Frequency synthesiser - PLL configuration
// UM §7.2.44.3: Frequency synthesiser - PLL tuning
// UM §7.2.41.3: Value for RF_RXCTRLH
// UM §7.2.41.4: Value for RF_TXCTRL
// UM §7.2.43.6: Pulse Generator Delay
struct _channel_tunning {
    uint32_t fs_pll_cfg;      // Frequency synthesiser - PLL configuration
    uint8_t  fs_pll_tune;     // Frequency synthesiser – PLL Tuning
    uint32_t rf_txctrl;       // RF configuration for TX
    uint8_t  rf_rxctrlh;      // RF configuration for RX
    uint8_t  tc_pgdelay;      // Pulse Generator Delay
};
/* Channel 5 carries the values of the CURRENT manual, which are not the
 * ones every other DW1000 driver ships. Both were changed after UM 2.12
 * and both changes are deliberate here:
 *
 *   RF_TXCTRL  0x001E3FE0 -> 0x001E3FE3   UM 2.16, table 38
 *   TC_PGDELAY       0xC0 -> 0xB5         UM 2.18, table 40
 *
 * The first is a real defect in the old value, not a preference.
 * Decawave's own issue tracker carries it (uwb-dw1000 issue 2): the old
 * setting produces about 2 dB of spurs in the channel 5 transmit
 * spectrum, which "can cause regulatory issues when using channel 5,
 * meaning a lower TX power has to be used to meet regulation". That
 * issue is still open and its pull request unmerged, so uwb-dw1000
 * carrying 0x001E3FE0 is neglect rather than a considered choice, and
 * matching it would be matching a bug. Note 2.18 still draws the old
 * ...E0 in the bit diagram below table 38, and typesets the table cell
 * as "0x00 1E3FE3" with a stray space; the table is the normative one.
 *
 * The second comes with 2.18's change log entry, "TC_PGDELAY setting for
 * channel 5 updated to 0xB5, maximising power in CH B/W". TC_PGDELAY
 * sets the pulse width and so the occupied bandwidth.
 *
 * Measured on the bench when they were adopted, rpi-c to rpi-d,
 * channel 5, matched runs:
 *
 *   received signal power   -80.98 dBm -> -81.59 dBm   (-0.61 dB)
 *   SDS-TWR distance         73.2 cm   ->  68.6 cm     (-4.5 cm)
 *
 * The distance is a BIAS shift, not an accuracy gain: there is no ground
 * truth in that measurement. Changing the pulse width changes the
 * effective antenna delay, so cfg->tx_antenna_delay and
 * cfg->rx_antenna_delay calibrated against 0xC0 no longer hold, and a
 * node keeping its old calibration reads about 4 cm short on channel 5.
 * Re-calibrate after taking this driver, and do not compare distances
 * measured across the change.
 */
static const struct _channel_tunning channel_tunning[] = {
    { 0x09000407, 0x1E, 0x00005C40, 0xD8, 0xC9 }, // Channel 1
    { 0x08400508, 0x26, 0x00045CA0, 0xD8, 0xC2 }, // Channel 2
    { 0x08401009, 0x56, 0x00086CC0, 0xD8, 0xC5 }, // Channel 3
    { 0x08400508, 0x26, 0x00045C80, 0xBC, 0x95 }, // Channel 4
    { 0x0800041D, 0xBE, 0x001E3FE3, 0xD8, 0xB5 }, // Channel 5 (UM 2.16/2.18)
    { 0x0800041D, 0xBE, 0x001E7DE0, 0xBC, 0x93 }, // Channel 7
};

// Tunning according to PRF
// UM §7.2.47.6: LDE_CFG2
// UM §7.2.36.3: AGC_TUNE1
// UM §7.2.40.3: DRX_TUNE1a
// UM §7.2.40.5: DRX_TUNE2
struct _prf_tunning {
    uint16_t lde_cfg2;     // LDE config
    uint16_t agc_tune1;    // AGC tunning
    uint16_t drx_tune1a;   // DRX tunning
    uint32_t drx_tune2[4]; // DRX tunning depending of PAC
};
static const struct _prf_tunning prf_tunning[] = {
    //  4MHz (unsupported by DW1000) [DW1000_PRF_4MHZ ]
    {      0,      0,      0, {          0,         0,         0,         0 } },
    // 16MHz                         [DW1000_PRF_16MHZ]
    { 0x1607, 0x8870, 0x0087, { 0x311A002D,0x331A0052,0x351A009A,0x371A011D } },
    // 64MHz                         [DW1000_PRF_64MHZ]
    { 0x0607, 0x889B, 0x008D, { 0x313B006B,0x333B00BE,0x353B015E,0x373B0296 } }
};

// Tunning according to bitrate
// UM §7.2.40.2: DRX_TUNE0b
// UM §7.2.34  : User defined SFD sequence
struct _bitrate_tunning {
    uint16_t drx_tune0b;
    struct {
	uint16_t drx_tune0b;
	uint8_t  usr_sfd_len;
    } proprietary_sfd;
};
static const struct _bitrate_tunning bitrate_tunning[] = {
    { 0x000A, { 0x0016, 64 } }, //  110Kb/s
    { 0x0001, { 0x0006, 16 } }, //  850Kb/s
    { 0x0001, { 0x0002,  8 } }  // 6800Kb/s
};

// UM §7.2.47.7: LDE REPC
static const uint16_t lde_repc_tunning[] = {
    0, // No preamble code 0
    0x5998, 0x5998, 0x51EA, 0x428E, 0x451E, 0x2E14,
    0x8000, 0x51EA, 0x28F4, 0x3332, 0x3AE0, 0x3D70,
    0x3AE0, 0x35C2, 0x2B84, 0x35C2, 0x3332, 0x35C2,
    0x35C2, 0x47AE, 0x3AE0, 0x3850, 0x30A2, 0x3850
};

/* The computed SFD timeout is the only user of the PAC table below, and
 * of the check that the enum values still index it the way it is laid
 * out. With DW1000_WITH_SFD_TIMEOUT_DEFAULT there is nothing to compute,
 * so leave it out rather than carry an unused table, and a
 * -Wunused-const-variable warning, into every image built that way.
 * The preamble table that follows it is needed either way, the transmit
 * airtime being computed from it unconditionally. */
#if !DW1000_WITH_SFD_TIMEOUT_DEFAULT

// PAC symbol size
#if (DW1000_PAC8  != 0) || (DW1000_PAC16 != 1) ||	\
    (DW1000_PAC32 != 2) || (DW1000_PAC64 != 3)
#error "unexpected DW1000_PACx value for pac_size table"
#endif
static const uint8_t pac_symbol_size[] = {
    8, 16, 32, 64
};

#endif // !DW1000_WITH_SFD_TIMEOUT_DEFAULT

// PLEN symbol size
#if                               (DW1000_PLEN_64     != 0x1)  ||	\
                                  (DW1000_PLEN_1024   != 0x2)  ||	\
                                  (DW1000_PLEN_4096   != 0x3)  ||	\
    (defined(DW1000_PLEN_128 ) && (DW1000_PLEN_128    != 0x5)) ||	\
    (defined(DW1000_PLEN_1536) && (DW1000_PLEN_1536   != 0x6)) ||	\
    (defined(DW1000_PLEN_256 ) && (DW1000_PLEN_256    != 0x9)) ||	\
    (defined(DW1000_PLEN_2048) && (DW1000_PLEN_2048   != 0xA)) ||	\
    (defined(DW1000_PLEN_512 ) && (DW1000_PLEN_512    != 0xD))
#error "unexpected DW1000_PLEN_xxx value for plen_size table"
#endif
static const uint16_t plen_symbol_size[] = {
       0, // 0x0
      64, // 0x1
    1024, // 0x2
    4096, // 0x3
#if DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH
       0, // 0x4
     128, // 0x5
    1536, // 0x6
       0, // 0x7
       0, // 0x8
     256, // 0x9
    2048, // 0xA
       0, // 0xB
       0, // 0xC
     512, // 0xD
#endif
};



/*===========================================================================*/
/* Local functions                                                           */
/*===========================================================================*/

/**
 * @internal
 * @brief Compute SPI header for DW1000 register access
 *
 * @note hdr *must* have a storage size of DW1000_SPI_HEADER_MAX_LENGTH
 *
 * @param[in]  reg      register [0..63]
 * @param[in]  offset   offset to access inside the register
 * @param[in]  write    is it in write mode?
 * @param[out] hdr      buffer where to write the header
 * @param[out] hlen     size of the constructed header
 */
static inline
void _dw1000_spi_header(uint8_t reg,  size_t offset, bool write,
			uint8_t *hdr, size_t *hlen) {
    DW1000_ASSERT(reg    <= 0x3F,    "invalid register number");
    DW1000_ASSERT(offset <= 0x7FFFu, "out of range offset");

    // Start by assuming register with offset 0
    //  and compute additionnal header bytes due to offset
    *hlen  = 1;
    hdr[0] = reg & 0x3F;

    if (offset != 0) {
	hdr[0] |= 0x40;
	
	hdr[1]   = offset & 0x7F;
	*hlen    = 2;
	offset >>= 7;
	
	if (offset != 0) {
	    hdr[1] |= 0x80;
	    hdr[2]  = offset & 0xFF;
	    *hlen   = 3;
	}
    }

    // Toggle write flag
    if (write) {
	hdr[0] |= 0x80;
    }
}


/**
 * @internal
 * @brief Clear bits for clearing register
 *
 * @param[in]  dw       driver context
 */
static inline
void _dw1000_reg_clear32(dw1000_t *dw,
			uint8_t reg, size_t offset, uint32_t value) {
    uint32_t val = _dw1000_reg_read32(dw, reg, offset);
    _dw1000_reg_write32(dw, reg, offset, val & ~value);
}


/**
 * @internal
 * @brief  Set clocks for appropriate mode
 *
 * @param[in]  dw       driver context
 * @param[in]  mode     mode for which to set the clocks
 *                       - DW1000_CLOCK_SEQUENCING
 *                       - DW1000_CLOCK_SYS_XTI
 *                       - DW1000_CLOCK_SYS_PLL
 *                       - DW1000_CLOCK_TX_CONTINOUSFRAME
 *                       - DW1000_CLOCK_LDE_LOAD
 */
static
void _dw1000_clocks(dw1000_t *dw, int mode) {
    /* PMSC CTRL0 is a 4-byte length field.
     * Byte 0 holds the sys/tx/rx clock selections, and byte 1 holds the
     * LDECLK bit that UM §2.5.5.10 (table 4) requires set while the LDE
     * microcode is copied from ROM to RAM. Both are read-modify-written,
     * so bits no mode touches keep their value, except the high byte
     * during DW1000_CLOCK_LDE_LOAD, which table 4 pins to an exact value.
     */

    // Read current value
    uint8_t pmsc_ctrl0[2];
    _dw1000_reg_read(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0, pmsc_ctrl0, 2);

    // Change value according to mode
    switch(mode) {
    case DW1000_CLOCK_SEQUENCING:
	// UM §2.5.5.10 (table 4, step L-3): 0x0200 once the LDE load is done
	pmsc_ctrl0[0] &= ~DW1000_MSK_PMSC_CTRL0_SYSCLKS;
	pmsc_ctrl0[1] &= ~(DW1000_FLG_PMSC_CTRL0_LDECLK >> 8);
	break;

    case DW1000_CLOCK_SYS_XTI:
	pmsc_ctrl0[0] &= ~DW1000_MSK_PMSC_CTRL0_SYSCLKS;
	pmsc_ctrl0[0] |=  DW1000_VAL_PMSC_CTRL0_SYSCLKS_19M;
        break;

    case DW1000_CLOCK_SYS_PLL:
	pmsc_ctrl0[0] &= ~DW1000_MSK_PMSC_CTRL0_SYSCLKS;
	pmsc_ctrl0[0] |=  DW1000_VAL_PMSC_CTRL0_SYSCLKS_125M;
        break;

    case DW1000_CLOCK_TX_CONTINOUSFRAME:
	pmsc_ctrl0[0] = 0x22 | (pmsc_ctrl0[0] & 0xCC);	
        break;

    case DW1000_CLOCK_LDE_LOAD:
	// UM §2.5.5.10 (table 4, step L-1): PMSC_CTRL0[15:0] = 0x0301, ie:
	// system clock forced to the 19.2MHz XTI *and* LDECLK enabled.
	// LDECLK (bit 8) is marked reserved in UM §7.2.50.1 and has no
	// mnemonic there, but table 4 requires it set while the microcode is
	// copied from ROM to RAM; deca_device.c sets it too, in
	// _dwt_enableclocks(FORCE_LDE). Without it the microcode is not
	// copied, the LDE runs on an empty RAM (LDERUNE defaults to 1), and
	// RX_STAMP never gets its leading edge correction.
	// The high byte is assigned, not or-ed: the table prescribes the
	// exact value, and or-ing would make the sequence depend on bit 9
	// still holding its reset value.
	pmsc_ctrl0[0] &= ~DW1000_MSK_PMSC_CTRL0_SYSCLKS;
	pmsc_ctrl0[0] |=  DW1000_VAL_PMSC_CTRL0_SYSCLKS_19M;
	pmsc_ctrl0[1]  =  0x03;
        break;

    case DW1000_CLOCK_ACC_READ:
	// UM §7.2.38 leaves the accumulator readable only while its memory
	// is clocked, which the sequencer does not arrange on its own. The
	// receiver clock is forced to the 125MHz PLL, and FACE and AMCE
	// (§7.2.50.1) put the analog and the accumulator memory clocks on.
	// deca_device.c does the same three in _dwt_enableclocks(READ_ACC_ON).
	pmsc_ctrl0[0] &= ~DW1000_MSK_PMSC_CTRL0_RXCLKS;
	pmsc_ctrl0[0] |=  DW1000_VAL_PMSC_CTRL0_RXCLKS_125M
	                      << DW1000_OFF_PMSC_CTRL0_RXCLKS;
	pmsc_ctrl0[0] |=  DW1000_FLG_PMSC_CTRL0_FACE;
	pmsc_ctrl0[1] |=  DW1000_FLG_PMSC_CTRL0_AMCE >> 8;
	break;

    case DW1000_CLOCK_ACC_DONE:
	// The three back off again. RXCLKS_AUTO being 0, clearing the mask
	// is what returns the receiver clock to the sequencer.
	pmsc_ctrl0[0] &= ~DW1000_MSK_PMSC_CTRL0_RXCLKS;
	pmsc_ctrl0[0] &= ~DW1000_FLG_PMSC_CTRL0_FACE;
	pmsc_ctrl0[1] &= ~(DW1000_FLG_PMSC_CTRL0_AMCE >> 8);
	break;

    default:
        break;
    }

    // Force sending lower byte (ie: sys/tx/rx clocks) first
    _dw1000_reg_write(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0,
		     &pmsc_ctrl0[0], 1);
    _dw1000_reg_write(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0 + 1,
		     &pmsc_ctrl0[1], 1);
}


/**
 * @internal
 * @brief  Force the transmitter clock on, or return it to sequencing
 *
 * @details Errata 1.4 §3.1 (TX-1): for a delayed transmit whose send
 *          time falls between the TXPUTE window and "time OK", the
 *          frame is not sent and *neither* HPDWARN nor TXPUTE is
 *          raised, so dw1000_tx_start() cannot tell the failure from a
 *          success and no TX done event ever follows. The workaround
 *          the erratum gives is to force the TX clock on before the
 *          delayed TX command is issued ("PMSC_CTRL0 bits 5,4 set to
 *          1,0"); returning it to automatic sequencing once the frame
 *          is out is this driver's own doing, so that the forced clock
 *          costs power only while a delayed send is pending.
 *
 * @note    PMSC_CTRL0 byte 0 holds SYSCLKS, RXCLKS and TXCLKS; it is
 *          read-modify-written so the other selections keep their value.
 *
 * @param[in]  dw       driver context
 * @param[in]  force    true to force the clock on, false to release it
 */
static
void _dw1000_tx_clock_force(dw1000_t *dw, bool force) {
    uint8_t pmsc_ctrl0 =
	_dw1000_reg_read8(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0);

    pmsc_ctrl0 &= ~DW1000_MSK_PMSC_CTRL0_TXCLKS;
    pmsc_ctrl0 |= (force ? DW1000_VAL_PMSC_CTRL0_TXCLKS_125M
		         : DW1000_VAL_PMSC_CTRL0_TXCLKS_AUTO)
	          << DW1000_OFF_PMSC_CTRL0_TXCLKS;

    _dw1000_reg_write8(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0,
		       pmsc_ctrl0);

    dw->tx_clk_forced = force ? 1 : 0;
}


/**
 * @internal
 * @brief  Return the transmitter clock to sequencing if this driver
 *         forced it on for a delayed send (Errata 1.4 §3.1, TX-1)
 *
 * @param[in]  dw       driver context
 */
static inline
void _dw1000_tx_clock_release(dw1000_t *dw) {
    if (dw->tx_clk_forced)
	_dw1000_tx_clock_force(dw, false);
}


/**
 * @internal
 * @brief Perform software reset of the DW1000
 *
 * @pre The SPI interface must have been initialized to call this
 *      function.
 *
 * @param[in]  dw       driver context
 */
static
void _dw1000_softreset(dw1000_t *dw) {
    // Switch to XTAL
    _dw1000_clocks(dw, DW1000_CLOCK_SYS_XTI);

    // Disable PKTSEQ
    //  (default value for the other bits of the 16-bit word are 0 anyway)
    _dw1000_reg_write16(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL1, 0x0000); 

    // Clear AON auto download (as reset will trigger AON download)
    _dw1000_reg_write16(dw, DW1000_REG_AON, DW1000_OFF_AON_WCFG, 0x0000);
    // Clear wake-up configuration
    _dw1000_reg_write8 (dw, DW1000_REG_AON, DW1000_OFF_AON_CFG0, 0x00);
    // Upload new configuration
    _dw1000_reg_write8 (dw, DW1000_REG_AON, DW1000_OFF_AON_CTRL,
			DW1000_FLG_AON_CTRL_SAVE);
    // UM §2.4.1.2: the AON array copy takes about 7µs, and SPI access
    // must be avoided while it runs. At 3MHz the next transaction nearly
    // covers it on its own, but nothing in the driver caps the SPI clock,
    // so wait rather than rely on the port being slow.
    _dw1000_delay_usec(10); // Be large, using 10µs instead of 7µs

    // Reset All (HIF, TX, RX, PMSC) (put flags to 0)
    _dw1000_reg_write8 (dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0_SOFTRESET,
			0x00);
    // DW1000 takes 10µs to lock clock PLL after reset (automatic after reset)
    _dw1000_delay_usec(12); // Be large, using 12µs instead of 10µs
    // Clear reset (put flags to 1)
    _dw1000_reg_write8 (dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0_SOFTRESET,
			0xF0);
    
    // Reset internal flags
    dw->state         = DW1000_STATE_IDLE;
    dw->tx_clk_forced = 0;
    dw->rx_reset_due  = 0;
    dw->rx_want       = DW1000_RX_WANT_NONE;
    dw->tx_suspect    = 0;
#if DW1000_WITH_DEBUG
    dw->tx_late_flags = 0;
    dw->dbg_lde_cuts  = 0;
#endif
}


/**
 * @internal
 * @brief Tune the DW1000 radio
 *
 * @note  Magic is in the air!
 *
 * @param[in]  dw        driver context
 */
static inline
void _dw1000_radio_tuning(dw1000_t *dw) {
    /* The driver's own copy, already validated by _dw1000_radio_is_valid()
     * and stored by dw1000_configure() before this is called.
     */
    const struct dw1000_radio *radio = &dw->radio;


    /* Retrieve channel/PRF/Bitrate table helpers
     */
    const struct _channel_tunning *ci = &channel_tunning
	                                [channel_table_mapping[radio->channel]];
    const struct _prf_tunning     *pi = &prf_tunning[radio->prf];   
    const struct _bitrate_tunning *bi = &bitrate_tunning[radio->bitrate];
  
    
    /* Configure LDE
     */
    // UM §7.2.47.2: A value of 12 or 13 for NTM, and a value of 3 for PMULT
    //               has been found to work well
    const uint8_t  lde_cfg1 = (3 << 5) | (13);   // PMULT = 3 / NTM = 13 

    // UM §7.2.47.6: LDE_CFG2
    //  (Using prf_tunning)
    const uint16_t lde_cfg2 = pi->lde_cfg2;

    // UM §7.2.47.7: LDE_REPC must be divided by 8 for 110Kb/s bitrate
    //  (Using lde_repc_tunning)
    uint16_t lde_repc = lde_repc_tunning[radio->rx_pcode];
    if (radio->bitrate == DW1000_BITRATE_110KBPS)
	lde_repc >>= 3;


    /* Configure FS_CTRL
     *  (Using channel_tunning)
     */
    const uint32_t fs_pll_cfg  = ci->fs_pll_cfg;
    const uint8_t  fs_pll_tune = ci->fs_pll_tune;

    
    /* Configure RF_CONF
     *  (Using channel_tunning)
     */
    const uint8_t  rf_rxctrlh  = ci->rf_rxctrlh;
    const uint32_t rf_txctrl   = ci->rf_txctrl;

    
    /* Configure TC_PGDELAY
     *  (Using channel_tunning)
     */
    const uint8_t  tc_pgdelay  = ci->tc_pgdelay;


    /* Configure TX_POWER
     *  (Using manual_tx_power)
     * XXX: this assume Smart Power is *disabled* (DIS_STXP),
     *      this is the case as this library is in Smart Power Disable
     *      by default, and doesn't support changing it for now
     */
    if (radio->tx_power & DW1000_TX_POWER_FLG_MANUAL) {
	// UM 2.18 §7.2.31.1: "The gain control range is 30.5 dB consisting
	// of 32 fine (mixer gain) control steps of 0.5 dB and 7 coarse (DA
	// gain) steps of 2.5 dB", ie 61 half-dB steps: 15 dB coarse
	// + 15.5 dB fine.
	//  NOTE: 2.12 read 33.5 dB and 3 dB coarse steps, which is what this
	//        computed until UM 2.18 corrected it. A node calibrated
	//        against the old mapping is off by 0.5 dB per coarse step.
	uint8_t power_05db = radio->tx_power & DW1000_TX_POWER_MSK_MANUAL;
	if (power_05db > 61) power_05db = 61;
	// UM §7.2.31.4: power = coarse (DA, 2.5dB = 5 x 0.5dB steps)
	//                     + fine (mixer, 0.5dB steps, 0..31)
	// Coarse field is (6 - coarse) and is 3-bit encoded (110..000)
	//  Coarse is taken first, as UM §7.2.31.1 asks for the best
	//  spectral shape, with the remainder left to the fine steps.
	uint8_t coarse = power_05db / 5;
	if (coarse > 6) coarse = 6;
	uint8_t fine   = power_05db - coarse * 5;
	uint8_t power  = ((6 - coarse) << 5) | (fine);
	dw->tx_power = (power << 16) | (power << 8);
    } else {
	const struct _tx_power *tp =
	    &manual_tx_power[channel_table_mapping[radio->channel]];
	dw->tx_power = ((radio->prf == DW1000_PRF_64MHZ)
			  ? tp->prf_64mhz   // NOTE: PRF 4MHz is unsupported
			  : tp->prf_16mhz   //       by the DW1000
			) << 8;
    }
    
    /* Configure DRX Tune
     */
    // UM §7.2.40.2: DRX_TUNE0b
    //  (Using bitrate_tunning)
    uint16_t drx_tune0b = bi->drx_tune0b;
#if DW1000_WITH_PROPRIETARY_SFD
    if (radio->proprietary.sfd)
	drx_tune0b = bi->proprietary_sfd.drx_tune0b;
#endif
    
    // UM §7.2.40.3: DRX_TUNE1a
    //  (Using prf_tunning)
    const uint16_t drx_tune1a = pi->drx_tune1a;
	
    // UM §7.2.40.4: DRX_TUNE1b
    // NOTE: Table 32 leaves one combination uncovered: a 64 symbol
    //  preamble at 850kbps. It scopes 0x0010 to 6.8Mbps and 0x0020 to
    //  preamble lengths 128..1024, so neither row claims that case.
    //  Ours takes 0x0020, reading the bitrate as the deciding term;
    //  deca_device.c and uwb-dw1000 both take 0x0010, their PLEN_64
    //  test not being gated on the bitrate. Unreachable in practice,
    //  a 64 symbol preamble being a 6.8Mbps instrument. See AUDIT.md.
    uint16_t drx_tune1b = 0x0020;
    if       (radio->bitrate == DW1000_BITRATE_110KBPS)
	drx_tune1b = 0x0064;
    else if ((radio->bitrate == DW1000_BITRATE_6800KBPS) && 
	     (radio->tx_plen == DW1000_PLEN_64))
	drx_tune1b = 0x0010;

    // UM §7.2.40.5: DRX_TUNE2
    //  (Using prf_tunning)
    const uint32_t drx_tune2 = pi->drx_tune2[radio->rx_pac];

    // Preamble and SFD lengths, in symbols. Used by the SFD timeout just
    // below, and by the transmit airtime cached at the end of this
    // function; computed unconditionally so that both have them whatever
    // DW1000_WITH_SFD_TIMEOUT* select.
    //
    // UM §4.1.3: SFD detection
    //  In the standard, the SFD is 64 symbols long for 110Kb/s,
    //  and 8 symbols for other bitrate (8500Kb/s, 6.8Mb/s)
    const uint16_t plen_symbols = plen_symbol_size[radio->tx_plen];
    const uint16_t sfd_symbols  =
#if DW1000_WITH_PROPRIETARY_SFD
	radio->proprietary.sfd
	? bitrate_tunning[radio->bitrate].proprietary_sfd.usr_sfd_len
	:
#endif
	  ((radio->bitrate == DW1000_BITRATE_110KBPS) ? 64 : 8);

    // UM §7.2.40.7: DRX_SFDTOC
    // Timeout value of 0 is forbidden, so guess the optimal timeout
    // SFD timeout is in symbol unit.
    // It seems to be possible to compute optimal timeout value
    //    1
    //  + Preamble length
    //  + SFD length (SFD = Start of Frame Delimiter)
    //  - PAC size   (PAC = Preamble Acquisition Chunk)
    const uint16_t drx_sfdtoc =
#if DW1000_WITH_SFD_TIMEOUT
	radio->sfd_timeout ? radio->sfd_timeout
	                   :
#endif
#if DW1000_WITH_SFD_TIMEOUT_DEFAULT
        DW1000_SFD_TIMEOUT_DEFAULT
#else
	(1 + plen_symbols + sfd_symbols - pac_symbol_size[radio->rx_pac])
#endif
	;
    
    // UM §7.2.40.10: DRX_TUNE4H
    const uint16_t drx_tune4h = radio->tx_plen == DW1000_PLEN_64
	                      ? 0x0010  // For preample length == 64
	                      : 0x0028; // For preample length >= 128


    /* Configure AGC Tune (UM §7.2.36)
     */
    // UM §7.2.36.3: AGC_TUNE1
    //  (Using prf_tunning)
    const uint16_t agc_tune1 = pi->agc_tune1;

    // UM §7.2.36.5: AGC_TUNE2
    const uint32_t agc_tune2 = 0x2502A907;

    // UM §7.2.36.7: AGC_TUNE3
    const uint16_t agc_tune3 = 0x0035;
    
#if DW1000_WITH_PROPRIETARY_SFD
    /* Configure USR_SFD
     */
    // UM §7.2.34: User defined SFD sequence
    // In our case we are only dealing with decawave configuration
    //   which impact SFD_LENGTH (ie: when DWSFD of CHAN_CTRL is set)
    uint8_t usr_sfd_len = bi->proprietary_sfd.usr_sfd_len;
#endif


    /* Save RXPACC adjustement
     */
    // We are only dealing with Standard or Decawave SFD (no user defined)
    // UM §7.2.18: [Table 18]: RXPACC Adjustement by SFD code
    //  Norm         | SFD length | Adjustement | Bitrate
    //               |            | to RXPACC   | (recommanded for SFD)
    //  -------------+------------+-------------+----------------------
    //  Standard     |  8         |  -5         | 6800k or 850k
    //               | 64         | -64         |  110k 
    //  -------------+--------------------------+----------------------
    //  Decawave     |  8         | -10         | 6800k
    //   proprietary | 16         | -18         |  850k
    //               | 64         | -82         |  110k
#if DW1000_WITH_PROPRIETARY_SFD
    /* SETTLED at 6.8 Mbps, by measurement. The DWSFD field text
     * (UM §7.2.32) ends "(For 6.8 Mbps the standard 8-symbol SFD)",
     * which read one way would make the standard-8 adjustment (-5) the
     * right one there, and the Decawave-8 row of table 18 (-10) wrong.
     *
     * It is the other reading: the Decawave SFD is 8 symbols long at
     * 6.8 Mbps, and DWSFD still selects it. Measured on the bench
     * (rpi-c to rpi-d, 6.8 Mbps, 40 frames per run, the
     * matched runs interleaved with the crossed ones so a dead link
     * could not be mistaken for a result):
     *
     *   transmitter  receiver   frames heard
     *   DWSFD set    set        40, 40
     *   DWSFD clear  clear      40, 40
     *   DWSFD set    clear       0
     *   DWSFD clear  set         0, 0
     *
     * Setting DWSFD changes the sequence on the air at 6.8 Mbps: a node
     * with it set cannot hear a node without it, either way round. And
     * UM 2.15 §7.2.32 says DWSFD takes precedence, TNSSFD and RNSSFD
     * being ignored while it is set, so the sequence it selects cannot
     * be the user-defined SFD of table 22 (whose bytes this driver never
     * programs anyway). What is left is the Decawave SFD, so the
     * Decawave-8 adjustment is the correct one and the driver keeps it.
     *
     * This does not measure RXPACC itself, which would have been the
     * direct check; the binding exposes no raw register read. It
     * measures which sequence goes on the air, which is the fact the
     * adjustment depends on.
     */
    if (radio->proprietary.sfd) {
	switch(usr_sfd_len) {
	case  8: dw->rxpacc_adj = -10; break;
	case 16: dw->rxpacc_adj = -18; break;
	case 64: dw->rxpacc_adj = -82; break;
	default: DW1000_ASSERT(0, "unexpected proprietary SFD length");
	}
    } else {
#endif
	// UM §4.1.3: SFD detection
	//  SFD is 64 symbols for 110k bitrate, otherwise it is 8 symbols
	dw->rxpacc_adj = (radio->bitrate == DW1000_BITRATE_110KBPS) ? -64 : -5;
#if DW1000_WITH_PROPRIETARY_SFD
    }
#endif

    /* Transmit airtime of the preamble and SFD (Ton)
     *
     * APS022 §5.4 and its figure 5: for a delayed send the programmed
     * time, which is the RMARKER, has to be later than the moment the
     * TXSTRT command is issued *plus* Ton, because the RMARKER marks the
     * end of the SFD and the chip must already be transmitting preamble
     * by then. Cached here in device clock ticks so that
     * dw1000_tx_extended_vsendv() can size its default delay and refuse
     * a lead that cannot be met.
     *
     * At 4096+64 symbols and 65024 ticks a symbol this is about 2.7e8
     * ticks, comfortably inside a uint32_t (~4.3e9, ie ~67 ms).
     */
    dw->tx_ton = (uint32_t)(plen_symbols + sfd_symbols)
	       * ((radio->prf == DW1000_PRF_64MHZ)
		  ? DW1000_TICKS_PER_PSYM_64MHZ
		  : DW1000_TICKS_PER_PSYM_16MHZ);

    
    /* Apply configurations
     */
    // Apply LDE_IF
    _dw1000_reg_write16(dw, DW1000_REG_LDE_IF,   DW1000_OFF_LDE_REPC,
		       lde_repc);
    _dw1000_reg_write8 (dw, DW1000_REG_LDE_IF,   DW1000_OFF_LDE_CFG1,
		       lde_cfg1);
    _dw1000_reg_write16(dw, DW1000_REG_LDE_IF,   DW1000_OFF_LDE_CFG2,
		       lde_cfg2);
    
    // Apply FS_CTRL
    _dw1000_reg_write32(dw, DW1000_REG_FS_CTRL,  DW1000_OFF_FS_PLLCFG,
		       fs_pll_cfg);
    _dw1000_reg_write8 (dw, DW1000_REG_FS_CTRL,  DW1000_OFF_FS_PLLTUNE,
		       fs_pll_tune);

    // Apply RF_CONF
    _dw1000_reg_write8 (dw, DW1000_REG_RF_CONF,  DW1000_OFF_RF_RXCTRLH,
		       rf_rxctrlh);
    _dw1000_reg_write32(dw, DW1000_REG_RF_CONF,  DW1000_OFF_RF_TXCTRL,
		       rf_txctrl);

    // Apply TC_PGDELAY
    _dw1000_reg_write8 (dw, DW1000_REG_TX_CAL,   DW1000_OFF_TC_PGDELAY,
		       tc_pgdelay);

    // Apply TX_POWER
    _dw1000_reg_write32(dw, DW1000_REG_TX_POWER, DW1000_OFF_NONE,
		       dw->tx_power);

    // Apply AGC_CTRL
    _dw1000_reg_write16(dw, DW1000_REG_AGC_CTRL, DW1000_OFF_AGC_TUNE1,
		       agc_tune1);
    _dw1000_reg_write32(dw, DW1000_REG_AGC_CTRL, DW1000_OFF_AGC_TUNE2,
		       agc_tune2);
    _dw1000_reg_write16(dw, DW1000_REG_AGC_CTRL, DW1000_OFF_AGC_TUNE3,
		       agc_tune3);

    // Apply DRX_CONF
    _dw1000_reg_write16(dw, DW1000_REG_DRX_CONF, DW1000_OFF_DRX_TUNE0B,
		       drx_tune0b);
    _dw1000_reg_write16(dw, DW1000_REG_DRX_CONF, DW1000_OFF_DRX_TUNE1A,
		       drx_tune1a);
    _dw1000_reg_write16(dw, DW1000_REG_DRX_CONF, DW1000_OFF_DRX_TUNE1B,
		       drx_tune1b);
    _dw1000_reg_write32(dw, DW1000_REG_DRX_CONF, DW1000_OFF_DRX_TUNE2,
		       drx_tune2);
    _dw1000_reg_write16(dw, DW1000_REG_DRX_CONF, DW1000_OFF_DRX_TUNE4H,
		       drx_tune4h);
    _dw1000_reg_write16(dw, DW1000_REG_DRX_CONF, DW1000_OFF_DRX_SFDTOC,
		       drx_sfdtoc);

    // Apply USR_SFD
#if DW1000_WITH_PROPRIETARY_SFD
    if (radio->proprietary.sfd) {
	_dw1000_reg_write8 (dw, DW1000_REG_USR_SFD,  DW1000_OFF_USR_SFD_LENGTH,
			   usr_sfd_len);
    }
#endif   
    
    // HOTFIX: From the official deca_device.c:
    // "The SFD transmit pattern is initialised by the DW1000 upon a
    //  user TX request, but (due to an IC issue) it is not done for an
    //  auto-ACK TX. The SYS_CTRL write below works around this issue,
    //  by simultaneously initiating and aborting a transmission, which
    //  correctly initialises the SFD after its configuration or
    //  reconfiguration.  This issue is not documented at the time of
    //  writing this code. It should be in next release of DW1000 User
    //  Manual (v2.09, from July 2016)."
    // It is documented since UM 2.12, §5.3.1.2 (p. 52).
    // => Request "TX start" and "TRX off" at the same time
    _dw1000_reg_write8 (dw, DW1000_REG_SYS_CTRL, DW1000_OFF_SYS_CTRL,
		       DW1000_FLG_SYS_CTRL_TXSTRT | DW1000_FLG_SYS_CTRL_TRXOFF);
}


/**
 * @internal
 * @brief Reset the DW1000 receiver
 * 
 * @note  Used to deal with a bug in DW1000, see UM §4.1.6.
 *
 * @param[in]  dw       driver context
 */
static inline
void _dw1000_rx_reset(dw1000_t *dw) {
    // Trigger reset for RX by creating a 0 pulse
    _dw1000_reg_write8(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0_SOFTRESET,
		      0xE0);
    _dw1000_reg_write8(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0_SOFTRESET,
		      0xF0);
}


/**
 * @internal
 * @brief Get the preamble accumulation count
 *
 * @param[in]  dw       driver context
 *
 * @return the preamble acculumation count
 */
static inline
uint16_t _dw1000_rx_get_pacc_count(dw1000_t *dw) {
    // Get Preamble accumulation count... and adjust it
    // UM §7.2.18: RX Frame Information Register (RXPACC field)
    uint16_t rxpacc       =
	(_dw1000_reg_read32(dw, DW1000_REG_RX_FINFO, DW1000_OFF_NONE) &
	 DW1000_MSK_RX_FINFO_RXPACC) >> DW1000_SFT_RX_FINFO_RXPACC;
    // UM table 7 lists RX_FINFO (and so RXPACC) among the double
    // buffered registers, but not DRX_RXPACC_NOSAT. In double buffered
    // mode the live register may already describe the *next* frame, so
    // use the value dw1000_process_events() sampled for this one.
    uint16_t rxpacc_nosat = dw->config->dblbuff
	? dw->rxpacc_nosat
	: _dw1000_reg_read16(dw, DW1000_REG_DRX_CONF,
			     DW1000_OFF_DRX_RXPACC_NOSAT);
    if (rxpacc == rxpacc_nosat) {
	// The SFD adjustment is negative; clamp at 0 instead of
	// wrapping the unsigned count when the accumulation was
	// shorter than the adjustment
	int32_t adjusted = (int32_t)rxpacc + dw->rxpacc_adj;
	rxpacc = (adjusted > 0) ? adjusted : 0;
    }

    return rxpacc;
}


/**
 * @internal
 * @brief Ensure RX buffers pointers are the same.
 *
 * @param dw        driver context
 */
static
void _dw1000_rx_sync_dblbuff(dw1000_t *dw) {
    // Never while the host still holds a reported frame. Between the
    // RXENAB of the double buffered RXFCG branch and the HRBPT that
    // follows the rx_ok callback the two pointers are misaligned on
    // purpose: ICRBP is on the buffer the chip is filling, HSRBP on the
    // one being read out. Aligning them there issues HRBPT and hands
    // that buffer back to the chip, so the rest of the read-out returns
    // the other buffer (wrong payload, wrong timestamp, no error), and
    // the driver's own toggle afterwards leaves the pointers inverted.
    // A host reaches this from inside its callback by doing what the
    // send functions ask of it, dw1000_txrx_off() before a transmit, so
    // the guard belongs here rather than in each caller.
    if (dw->rx_held)
	return;

    // UM §7.2.17: System Event Status Register
    //  => Status is a 5 bytes register (DW1000_REG_SYS_STATUS), the
    //     first 4 of which carry ICRBP (31) and HSRBP (30), and RXFCG
    //     (14) and RXDFR (13) below. The whole word rather than the one
    //     byte at offset 3 that the pointers need: three more bytes on
    //     a read already made, one SPI transaction either way.
    const uint32_t sys_stat =
	_dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
    const bool ic   = sys_stat & DW1000_FLG_SYS_STATUS_ICRBP;
    const bool host = sys_stat & DW1000_FLG_SYS_STATUS_HSRBP;
    if (ic == host)
	return;

    // Nor while the chip has reported a frame the host has not been told
    // about. With the pointers apart the swinging bits are the host
    // buffer's latch rather than the chip's own flags (DW1000.md, "With
    // the buffer pointers aligned, the swinging status bits are the
    // chip's flags, not the buffer's"), so an RXFCG or RXDFR read here
    // is a frame completed into that buffer and not yet processed:
    // ICRBP != HSRBP is what the driver itself calls the normal state
    // for one, not a misalignment to repair, and the toggle would hand
    // the frame back to the chip unread, its RXFCG swinging out with it.
    if (sys_stat & (DW1000_FLG_SYS_STATUS_RXFCG |
		    DW1000_FLG_SYS_STATUS_RXDFR))
	return;

    // UM §7.2.15: System Control Register
    //  => Only accessing last byte of SYS_CTRL (where is HRBPT flag)
    //     Trigger buffer toggle by writting 1 to HRBPT
    _dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, 3 ,
		       (1 << (DW1000_SFT_SYS_CTRL_HRBPT - 24)));
}


/**
 * @internal
 * @brief Clear event status bits with the interrupts masked off
 *
 * @note  UM §4.3.3 (figure 14) and §4.3.4 (figure 15): the status bits
 *        of the double buffered swinging set (RXDFR, RXFCG, RXFCE,
 *        LDEDONE) glitch when they are cleared, so the interrupts must
 *        be masked while it happens or the glitch is delivered as a
 *        spurious interrupt. Only needed in double buffered mode; in
 *        single buffer mode those bits don't swing and a plain write
 *        does, saving two SPI transactions per frame.
 *
 * @param dw        driver context
 * @param clear     event status bits to clear
 */
static inline
void _dw1000_rx_clear_status_dblbuff(dw1000_t *dw, uint32_t clear) {
    // Save and clear interrupt mask
    uint32_t sys_mask =
	_dw1000_reg_read32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE);
    _dw1000_reg_write32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE, 0);

    // Clear the requested events bits (done by writting 1 to them)
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE, clear);

    // Restore interrupt mask
    _dw1000_reg_write32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE, sys_mask);
}



/*===========================================================================*/
/* Exported functions                                                        */
/*===========================================================================*/

// Registers (read/write)
//----------------------------------------------------------------------

void _dw1000_reg_read(dw1000_t *dw,
    uint8_t reg, size_t offset, void* data, size_t length) {
    DW1000_ASSERT(reg    <= 0x3F,               "invalid register number");
    DW1000_ASSERT(offset <= 0x7FFFu,            "out of range offset");
    DW1000_ASSERT(length <= (0x8000u - offset), "out of range length");

    // Build SPI header
    uint8_t hdr[DW1000_SPI_HEADER_MAX_LENGTH];
    size_t hdrlen;
    _dw1000_spi_header(reg, offset, false, hdr, &hdrlen);

    _dw1000_spi_recv(dw->config->spi, hdr, hdrlen, data, length);
}


void _dw1000_reg_write(dw1000_t *dw,
    uint8_t reg, size_t offset, void* data, size_t length) {
    DW1000_ASSERT(reg    <= 0x3F,               "invalid register number");
    DW1000_ASSERT(offset <= 0x7FFFu,            "out of range offset");
    DW1000_ASSERT(length <= (0x8000u - offset), "out of range length");

    // Build SPI header
    uint8_t hdr[DW1000_SPI_HEADER_MAX_LENGTH];
    size_t hdrlen;
    _dw1000_spi_header(reg, offset, true, hdr, &hdrlen);

    _dw1000_spi_send(dw->config->spi, hdr, hdrlen, data, length);
}


// OTP (read)
//----------------------------------------------------------------------

void dw1000_otp_read(dw1000_t *dw,
		     uint16_t address, uint32_t *data, size_t length) {
    DW1000_ASSERT(address <= 0x07FFu, "address is 11-bit encoded");
    DW1000_ASSERT(length  <= (0x0800u - address), "out of range");

    // Assuming we have exclusive use of the OTP_CTRL,
    // so we don't care about previously assigned value

    // UM §6.3.2 (table 13): each word is read by writing the address,
    // then OTPREAD|OTPRDEN, then clearing OTP_CTRL, and only then
    // reading OTP_RDAT. OTPREAD is self clearing, OTPRDEN is not, so
    // leaving it asserted across the read departs from the documented
    // sequence (and from deca_device.c's _dwt_otpread()).
    const uint16_t otp_ctrl = DW1000_FLG_OTP_CTRL_OTPREAD |
	                      DW1000_FLG_OTP_CTRL_OTPRDEN;
    for ( ; length-- > 0 ; address++, data++) {
	// Write the address to read
	_dw1000_reg_write16(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_ADDR,
			   address);
	// Perform reading by asserting OTP Read (self clearing)
	// and having OTP Read Enable set
	_dw1000_reg_write16(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_CTRL,
			   otp_ctrl);
	// Clear OTP_CTRL (OTPRDEN is not self clearing)
	_dw1000_reg_write16(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_CTRL,
			   0x0000);
	// Read Value
	*data = _dw1000_reg_read32(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_RDAT);
    }
}


// Setup helpers
//----------------------------------------------------------------------

bool
dw1000_get_calibration(uint8_t channel, uint8_t prf,
		       uint8_t *power, uint16_t *separation) {
    if ((channel < 1) || (channel > 7) || (channel == 6)) {
	return false;
    }

    switch(prf) {
    case DW1000_PRF_16MHZ:
    case DW1000_PRF_64MHZ:
	break;
    case DW1000_PRF_4MHZ:
    default:
	return false;
    }
    
    // Retrieve calibration information
    const struct _channel_prf_calibration *calib =
	&channel_prf_calibration[ channel_table_mapping[channel] ][ prf ];

    // Save calibration information
    if (power) {
	*power      = calib->power;
    }
    if (separation) {
	*separation = calib->separation;
    }

    return true;
}


// Initialisation
//----------------------------------------------------------------------

void dw1000_init(dw1000_t *dw, const dw1000_config_t *cfg) {
    memset(dw, 0, sizeof(*dw));
    dw->config = cfg;
}

void dw1000_hardreset(dw1000_t *dw) {
    if (dw->config->reset == DW1000_IOLINE_NONE)
	return;

    // Perform hard reset
    // DS §1.2: Reset pin must be de-asserter at least 10 ns.
    _dw1000_ioline_clear(dw->config->reset);
    _dw1000_delay_usec(1); // Be large, using 1µs instead of 10ns
    _dw1000_ioline_set(dw->config->reset);

    // Ensure wake up after reset
    if (dw->config->wakeup != DW1000_IOLINE_NONE)
	_dw1000_ioline_set(dw->config->wakeup);

    // Seems that 5ms should be enough to have the chip running
    // but not quite sure about it see UM §2.4
    _dw1000_delay_msec(8);  // Be large, using 8ms instead of 5ms
}


int dw1000_initialise(dw1000_t *dw) {
    // We won't bother to read default register value. We assume
    // values are at their defaults due to reset performed inside

    const dw1000_config_t *cfg = dw->config;

    // Double buffering with nowhere to report an overrun is a receiver
    // that stops for good on the first one. dw1000_process_events()
    // recovers the chip (transceiver off, receiver reset, pointers
    // re-aligned) but leaves the re-arming to the host, as it does for
    // every other receive error, and the rx_error callback is how the
    // host is asked to do it. With none registered the recovery runs to
    // completion and nothing ever enables the receiver again.
    //
    // Refused rather than papered over. A driver that re-armed by itself
    // here would hide the frames an overrun means were lost, which is
    // exactly what a host measuring distances cannot afford not to know.
    // And refused at initialisation rather than at the overrun, because
    // it is a property of the configuration: this is the cheapest moment
    // to say so, and it is before any SPI traffic.
    //
    // It is also what lets the MRXOVRR line below be conditioned on
    // cfg->dblbuff alone and still match the rest of that block, every
    // other line of which is conditioned on the callback that consumes
    // the event.
    if (cfg->dblbuff && (cfg->cb.rx_error == NULL))
	return -1;

    // Start SPI at low speed
    _dw1000_spi_low_speed(cfg->spi);
    
    // Read and validate device ID
    dw->id.device = _dw1000_reg_read32(dw, DW1000_REG_DEV_ID, 0);
    if (dw->id.device != DW1000_ID_DEVICE) {
        return -1;
    }

    // Ensure reset state
    _dw1000_softreset(dw);
    
    // Clock need to be running at XTAL value for safe reading of OTP
    // or loading microcode (see MN: _dw1000_phy_load_microcode)
    _dw1000_clocks(dw, DW1000_CLOCK_SYS_XTI);

    // Retrieve Chip and Lot identification
    // UM 6.3.1: OTP memory map
    dw->id.chip = dw1000_otp_get(dw, DW1000_OTP_CHIP_ID) & DW1000_MSK_CHIP_ID;
    dw->id.lot  = dw1000_otp_get(dw, DW1000_OTP_LOT_ID ) & DW1000_MSK_LOT_ID;
    
    // Clock PLL lock detect tune.
    //  (Default value for the WAIT register is 0)
    // UM §7.2.37.1: Ensure reliable operation of the clock PLL lock
    //               detect flags.
    _dw1000_reg_write32(dw, DW1000_REG_EXT_SYNC, DW1000_OFF_EC_CTRL,
		       DW1000_FLG_EC_CTRL_PLLLDT);

    // Read OTP reference volatage / temperature
    uint32_t vbat = dw1000_otp_get(dw, DW1000_OTP_VBAT);
    uint32_t temp = dw1000_otp_get(dw, DW1000_OTP_TEMP);
    dw->ref_vbat_33  = vbat & 0xFF;
    dw->ref_vbat_37  = (vbat >> 8) & 0xFF;
    dw->ref_temp_23  = temp & 0xFF;
    dw->ref_temp_ant = (temp >> 8) & 0xFF;
    
    // Read OTP revision number, and XTAL trim value
    // UM §6.3.1: OTP memory map
    uint32_t rev_trim = dw1000_otp_get(dw, DW1000_OTP_REV_XTRIM);
    dw->otp_rev = (rev_trim >> 8) & 0xFF;
    dw->xtrim   = (rev_trim >> 0) & 0x1F;

    // Replace OTP XTRAL trim value if there is a user defined
    if (cfg->xtrim) {
	dw->xtrim = cfg->xtrim;
    }
    
    // If no calibration value, set to mid-range
    if (!dw->xtrim) {
        dw->xtrim = DW1000_XTRIM_MIDRANGE; 
    }

    // Configure XTAL trim (5 bits)
    // UM §7.2.44.5: bits 7/6/5 must be kept at 0/1/1
    _dw1000_reg_write8(dw, DW1000_REG_FS_CTRL, DW1000_OFF_FS_XTALT,
		       (3 << 5) | (dw->xtrim & 0x1F));

    // Automatically load LDO tune from OTP and kick it
    // UM §2.4.1.3: Only first byte of OTP_LDOTUNE need to be checked 
    uint32_t ldo_tune = dw1000_otp_get(dw, DW1000_OTP_LDO_TUNE);
    if (ldo_tune & 0xFF) {
	// Kick LDO
	_dw1000_reg_write8(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_SF,
			   DW1000_FLG_OTP_SF_LDO_KICK); 
	// Remain us, that sleep mode must kick LDO tune at wake-up
	dw->sleep_mode |= DW1000_FLG_AON_WCFG_ONW_LLDO;
    }

    // Dealing with LDE (leading edge detect) code
    // UM §7.2.46.3: Load code or clear run bit
    if (cfg->lde_loading) { //-> Loading of LDE code
	// UM §2.5.5.10 (table 4, step L-1): the load only happens with
	// PMSC_CTRL0[15:0] at 0x0301; step L-3 (back to 0x0200) is done by
	// the DW1000_CLOCK_SEQUENCING switch below.
	_dw1000_clocks(dw, DW1000_CLOCK_LDE_LOAD);

	// Start the LDE load (table 4, step L-2)
	_dw1000_reg_write16(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_CTRL,
			    DW1000_FLG_OTP_CTRL_LDELOAD);

	// Official deca_device.c says that loading code can take up to 120 µs
	_dw1000_delay_usec(150); // Be large, using 150µs instead of 120µs
    
	// Remain us, that sleep mode must load the LDE code at wake-up
        dw->sleep_mode |= DW1000_FLG_AON_WCFG_ONW_LLDE;
    } else {                       //-> Disable LDE running (as no code loaded)
	_dw1000_reg_clear32(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL1,
			    DW1000_FLG_PMSC_CTRL1_LDERUNE);
    }

    // Return clocks to default behaviour
    _dw1000_clocks(dw, DW1000_CLOCK_SEQUENCING); 

    // According to official deca_device.c:
    //   The 3 bits in AON CFG1 register must be cleared
    //   to ensure proper operation of the DW1000 in DEEPSLEEP mode.
    // UM §7.2.45.8: Other bits defaults to 0
    _dw1000_reg_write16(dw, DW1000_REG_AON, DW1000_OFF_AON_CFG1, 0x0000);
    
    // Read system register / store local copy.
    //   Configuring: double buffer / smart power / irq polarity
    //
    // UM §7.2.6: the reserved bits should always be set to 0
    //  
    // WARN: Disabling Smart Power by default, as Smart Power can impact
    //       ranging calculation when applying correction bias
    uint32_t sys_cfg = _dw1000_reg_read32(dw, DW1000_REG_SYS_CFG, 0) &
	               DW1000_MSK_SYS_CFG;
    if (cfg->dblbuff) { sys_cfg &= ~DW1000_FLG_SYS_CFG_DIS_DRXB; }
    else              { sys_cfg |=  DW1000_FLG_SYS_CFG_DIS_DRXB; }
    if (cfg->rxauto)  { sys_cfg |=  DW1000_FLG_SYS_CFG_RXAUTR;   }
    sys_cfg |=  DW1000_FLG_SYS_CFG_HIRQ_POL;
    sys_cfg |=  DW1000_FLG_SYS_CFG_DIS_STXP;   // Disable Smart Power
    _dw1000_reg_write32(dw, DW1000_REG_SYS_CFG, DW1000_OFF_NONE, sys_cfg);
    dw->reg.sys_cfg = sys_cfg;
    
    // Switch SPI to high speed (if supported)
    _dw1000_spi_high_speed(cfg->spi);

    // GPIO for LEDs
    if (cfg->leds) {
	// Ensure kHZ clock is running and enable de-bouncing clock.
	// XXX: seems to be mandatory?!
	uint32_t pmsc_ctrl0 =
	    _dw1000_reg_read32(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0);
	pmsc_ctrl0 |= DW1000_FLG_PMSC_CTRL0_GPDCE    |
	              DW1000_FLG_PMSC_CTRL0_KHZCLKEN ;
	_dw1000_reg_write32(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0,
			   pmsc_ctrl0);

	// Configure GPIO for LED mode
        uint32_t gpio_mode =
	    _dw1000_reg_read32(dw, DW1000_REG_GPIO_CTRL, DW1000_OFF_GPIO_MODE);
        gpio_mode &= ~(DW1000_MSK_GPIO_MSGP0 | DW1000_MSK_GPIO_MSGP1 |
		       DW1000_MSK_GPIO_MSGP2 | DW1000_MSK_GPIO_MSGP3);
	if (cfg->leds & DW1000_LED_RXOK)
	    gpio_mode |= DW1000_VAL_GPIO_0_RXOKLED << DW1000_SFT_GPIO_MSGP0;
	if (cfg->leds & DW1000_LED_SFD )
	    gpio_mode |= DW1000_VAL_GPIO_1_SFDLED  << DW1000_SFT_GPIO_MSGP1;
	if (cfg->leds & DW1000_LED_RX  )
	    gpio_mode |= DW1000_VAL_GPIO_2_RXLED   << DW1000_SFT_GPIO_MSGP2;
	if (cfg->leds & DW1000_LED_TX  )
	    gpio_mode |= DW1000_VAL_GPIO_3_TXLED   << DW1000_SFT_GPIO_MSGP3;
        _dw1000_reg_write32(dw, DW1000_REG_GPIO_CTRL, DW1000_OFF_GPIO_MODE,
			   gpio_mode);

        // Enable LEDs to blink and set default blink time.
        uint32_t pmsc_ledc = DW1000_FLG_PMSC_LEDC_BLNKEN | cfg->leds_blink_time;
        _dw1000_reg_write32(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_LEDC,
			   pmsc_ledc);
    }

    // GPIO (IRQ)
    uint32_t gpio_mode =
	_dw1000_reg_read32(dw, DW1000_REG_GPIO_CTRL, DW1000_OFF_GPIO_MODE);
    gpio_mode &= ~(DW1000_MSK_GPIO_MSGP8);
    gpio_mode |= ((cfg->irq == DW1000_IOLINE_NONE)
		  ? DW1000_VAL_GPIO_8_GPIO
		  : DW1000_VAL_GPIO_8_IRQ) << DW1000_SFT_GPIO_MSGP8;
    _dw1000_reg_write32(dw, DW1000_REG_GPIO_CTRL, DW1000_OFF_GPIO_MODE,
		       gpio_mode);

    // Antenna delay
    _dw1000_reg_write16(dw, DW1000_REG_LDE_IF,  DW1000_OFF_LDE_RXANTD,
		       cfg->rx_antenna_delay);
    _dw1000_reg_write16(dw, DW1000_REG_TX_ANTD, DW1000_OFF_NONE,
		       cfg->tx_antenna_delay);

    // By default enable interrupt corresponding to the registered callbacks
    uint32_t sys_mask = 0;
    if (cfg->cb.tx_done   ) { sys_mask |= DW1000_FLG_SYS_MASK_MTXFRS;     }
    if (cfg->cb.rx_timeout) { sys_mask |= DW1000_MSK_SYS_MASK_ALL_RX_TO;  }
    if (cfg->cb.rx_error  ) { sys_mask |= DW1000_MSK_SYS_MASK_ALL_RX_ERR; }
    if (cfg->cb.rx_ok     ) { sys_mask |= DW1000_FLG_SYS_MASK_MRXFCG;     }
    // Double buffering holds one frame while the next lands in the other
    // buffer; a third arriving before the host has freed one is an
    // overrun. dw1000_process_events() recovers from it, but only gets
    // the chance if the overrun raises an interrupt: RXOVRR is absent
    // from ALL_RX_ERR, so a double buffered host that does not unmask it
    // here stays in the errored state of UM §4.3.5 instead of recovering.
    if (cfg->dblbuff      ) { sys_mask |= DW1000_FLG_SYS_MASK_MRXOVRR;    }
    dw1000_interrupt(dw, sys_mask, true);

    return 0;
}


/**
 * @internal
 * @brief Check a radio configuration against what the DW1000 accepts
 *
 * Every field checked here indexes a tuning table, so a bad value is an
 * out of range read and not merely a wrong setting: a channel of 0 or 6
 * makes channel_table_mapping[] yield -1, and channel_tunning[-1] and
 * manual_tx_power[-1] are then read out of bounds.
 *
 * This used to be a wall of DW1000_ASSERT inside dw1000_configure().
 * That maps to assert() / __ASSERT / osalDbgAssert on four of the five
 * ports, so it vanished under NDEBUG, without CONFIG_ASSERT, or without
 * CH_DBG_ENABLE_ASSERTS: exactly the builds that ship.
 *
 * @param[in]  radio    radio configuration
 *
 * @return true when every field is usable
 */
static
bool _dw1000_radio_is_valid(dw1000_radio_t radio) {
    if (radio == NULL)
	return false;

    /* Values must be in range, each one is a table index
     */
    switch (radio->bitrate) {
    case DW1000_BITRATE_110KBPS:
    case DW1000_BITRATE_850KBPS:
    case DW1000_BITRATE_6800KBPS:
	break;
    default:
	return false;
    }

    switch (radio->channel) {
    case 1: case 2: case 3: case 4: case 5: case 7:
	break;
    default:                    // 0, 6, and anything above 7
	return false;
    }

    switch (radio->rx_pac) {
    case DW1000_PAC8:  case DW1000_PAC16:
    case DW1000_PAC32: case DW1000_PAC64:
	break;
    default:
	return false;
    }

    switch (radio->tx_plen) {
    case DW1000_PLEN_64:
    case DW1000_PLEN_1024:
    case DW1000_PLEN_4096:
#if DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH
    case DW1000_PLEN_128:
    case DW1000_PLEN_256:
    case DW1000_PLEN_512:
    case DW1000_PLEN_1536:
    case DW1000_PLEN_2048:
#endif
	break;
    default:
	return false;
    }

    if ((radio->tx_pcode < 1) || (radio->tx_pcode > 24) ||
	(radio->rx_pcode < 1) || (radio->rx_pcode > 24))
	return false;

    /* Values must be supported by the DW1000
     *  (4MHz PRF is accepted by the API but not by the receiver)
     */
    // UM §10.5: preamble codes 1..8 go with 16MHz PRF, 9..24 with 64MHz
    switch (radio->prf) {
    case DW1000_PRF_16MHZ:
	if ((radio->tx_pcode > 8) || (radio->rx_pcode > 8))
	    return false;
	break;
    case DW1000_PRF_64MHZ:
	if ((radio->tx_pcode < 9) || (radio->rx_pcode < 9))
	    return false;
	break;
    case DW1000_PRF_4MHZ:
	// FALLTHROUGH
    default:
	return false;
    }

    // radio->proprietary.sfd is a one bit field, it cannot be out of range

    return true;
}


int dw1000_configure(dw1000_t *dw, dw1000_radio_t radio) {
    /* Guard against out of range and unsupported values.
     * Checked in every build: on failure nothing is written to the chip
     * and dw->radio is left alone, so the driver keeps whatever
     * configuration it already had.
     */
    if (! _dw1000_radio_is_valid(radio))
	return -1;

    /* Configure SYS_CFG
     */
    // Use driver reference value
    uint32_t sys_cfg = dw->reg.sys_cfg;

    // Using Long Frames mode (ie: proprietary PHR mode)
    sys_cfg &= ~DW1000_MSK_SYS_CFG_PHR_MODE;
#if DW1000_WITH_PROPRIETARY_LONG_FRAME
    if (radio->proprietary.long_frames) {
	sys_cfg |=
	    DW1000_VAL_SYS_CFG_PHR_MODE_EXT << DW1000_SFT_SYS_CFG_PHR_MODE;
    }
#endif
    
    // UM §4.1.3: SFD detection
    // Bitrate at 110Kb/s need RXM110K flag (this will set the SFD length)
    // In the standard, the SFD is 64 symbols long for 110Kb/s,
    // and 8 symbols for other bitrate (8500Kb/s, 6.8Mb/s)
    sys_cfg &= ~DW1000_FLG_SYS_CFG_RXM110K;
    if (radio->bitrate == DW1000_BITRATE_110KBPS)
        sys_cfg |=  DW1000_FLG_SYS_CFG_RXM110K;


    /* Configure CHAN CTRL (UM §7.2.32)
     */
    // Same channel is used for TX/RX
    // Deal with proprietary decawave SFD
    //  The DWSFD will trigger reading of USR_SFD#SFD_LENGTH
    const uint32_t chan_ctrl =
	// Channel
	((uint32_t)radio->channel  << DW1000_SFT_CHAN_CTRL_TX_CHAN ) |
	((uint32_t)radio->channel  << DW1000_SFT_CHAN_CTRL_RX_CHAN ) |
	// PRF
	((uint32_t)radio->prf      << DW1000_SFT_CHAN_CTRL_RXPRF   ) |
#if DW1000_WITH_PROPRIETARY_SFD
	// SFD
	//   UM 2.15 §7.2.32 states that DWSFD takes precedence and that
	//   TNSSFD/RNSSFD are then ignored, so setting DWSFD alone is
	//   enough. Both Decawave drivers (deca_device.c dwt_configure(),
	//   uwb-dw1000 dw1000_mac_config()) set the three together; do the
	//   same, so that a reader comparing the drivers has one less
	//   difference to account for.
	((uint32_t)radio->proprietary.sfd << DW1000_SFT_CHAN_CTRL_DWSFD ) |
	((uint32_t)radio->proprietary.sfd << DW1000_SFT_CHAN_CTRL_TNSSFD) |
	((uint32_t)radio->proprietary.sfd << DW1000_SFT_CHAN_CTRL_RNSSFD) |
#endif
	// Preamble code (TX/RX)
	((uint32_t)radio->tx_pcode << DW1000_SFT_CHAN_CTRL_TX_PCODE) |
	((uint32_t)radio->rx_pcode << DW1000_SFT_CHAN_CTRL_RX_PCODE) ;
    

    /* Configure TX FCTRL (UM §7.2.10)
     */
    // UM §7.2.10
    // Below is shorted to 4 bytes out of 5 (byte 5 is delay)
    // Set up TX Preamble Size, PRF and Bit Rate
    // NOTE: tx_plen value is encoding TXPSR and PE
    const uint32_t tx_fctrl =
	(radio->tx_plen << DW1000_SFT_TX_FCTRL_PLEN ) |
	(radio->prf     << DW1000_SFT_TX_FCTRL_TXPRF) |
	(radio->bitrate << DW1000_SFT_TX_FCTRL_TXBR ) ;


    /* Save radio and some register settings to driver memory
     */
    dw->radio        = *radio;   
    dw->reg.tx_fctrl = tx_fctrl;
    dw->reg.sys_cfg  = sys_cfg;


    /* Apply
     */
    _dw1000_reg_write32(dw, DW1000_REG_SYS_CFG,  DW1000_OFF_NONE, sys_cfg  );
    _dw1000_reg_write32(dw, DW1000_REG_CHAN_CTRL,DW1000_OFF_NONE, chan_ctrl);
    _dw1000_reg_write32(dw, DW1000_REG_TX_FCTRL, DW1000_OFF_NONE, tx_fctrl );

    
    /* Perform radio tuning...
     */
    _dw1000_radio_tuning(dw);

    return 0;
}


// System
//----------------------------------------------------------------------

void dw1000_read_temp_vbat(dw1000_t *dw, int16_t *temp, uint16_t *vbat) {
    // From official deca_device.c (undocummented, part of RF_RES2)
    //   These writes should be single writes and in sequence
    // Enable TLD Bias
    _dw1000_reg_write8(dw, DW1000_REG_RF_CONF, 0x11, 0x80);
    // Enable TLD Bias and ADC Bias
    _dw1000_reg_write8(dw, DW1000_REG_RF_CONF, 0x12, 0x0A);
    // Enable Outputs (only after Biases are up and running)
    _dw1000_reg_write8(dw, DW1000_REG_RF_CONF, 0x12, 0x0F);

    // Mark as read
    _dw1000_reg_write16(dw, DW1000_REG_TX_CAL, DW1000_OFF_TC_SARC, 0);
    // Enable reading of new value
    _dw1000_reg_write16(dw, DW1000_REG_TX_CAL, DW1000_OFF_TC_SARC,
			DW1000_FLG_TC_SARC_SAR_CTRL);

    // UM 7.2.43.1: TC_SARC
    //   The enable should set for a minimum of 2.5 μs to allow the SAR
    //   time to complete its reading.
    _dw1000_delay_usec(4); // Be large using 4µs instead of 2.5µs

    // Reading voltage and temperature at once
    uint8_t tempvbat[2];
    _dw1000_reg_read(dw, DW1000_REG_TX_CAL, DW1000_OFF_TC_SARL,
		     &tempvbat, sizeof(tempvbat));

    // Mark as read, terminate SAR
    _dw1000_reg_write16(dw, DW1000_REG_TX_CAL, DW1000_OFF_TC_SARC, 0);

    // UM §7.243.2: TC_SARL
    if (temp) *temp = 2300 + ((tempvbat[1] - dw->ref_temp_23) * 114);
    if (vbat) *vbat = 3300 + ((tempvbat[0] - dw->ref_vbat_33) * 1000) / 173;
}


void dw1000_leds_blink(dw1000_t *dw, uint8_t leds) {
    const uint32_t pmsc_ledc =
	_dw1000_reg_read32(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_LEDC);
    const uint32_t mask = DW1000_MSK_PMSC_LEDC_BLNKNOW &
	(leds << DW1000_SFT_PMSC_LEDC_BLNKNOW);

    _dw1000_reg_write32(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_LEDC,
		       pmsc_ledc |  mask);
    _dw1000_reg_write32(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_LEDC,
		       pmsc_ledc & ~mask);
}


// Interruption handling
//----------------------------------------------------------------------

void dw1000_interrupt(dw1000_t *dw, uint32_t bitmask, bool enable) {
    uint32_t sys_mask = _dw1000_reg_read32(dw, DW1000_REG_SYS_MASK, 0);
    
    if (enable) { sys_mask |=  bitmask; } // Set
    else        { sys_mask &= ~bitmask; } // Clear

    sys_mask &= DW1000_MSK_SYS_MASK;
    _dw1000_reg_write32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE, sys_mask);
}


/**
 * @internal
 * @brief Drop what the receiver raised, leaving the transceiver alone
 *
 * @details The status side of _dw1000_txrx_off(): the receive-side bits
 *          in @p clear are cleared, through the masked write the
 *          swinging set needs in double buffered receive, and the
 *          buffer pointers are realigned when frame bits are among
 *          them. No TRXOFF: this is for an event found while a
 *          transmission is in flight, which TRXOFF would abort with no
 *          TXFRS ever raised for it (UM §7.2.15).
 *
 * @param[in]  dw       driver context
 * @param[in]  clear    event status bits to clear
 */
static void _dw1000_rx_drop_status(dw1000_t *dw, uint32_t clear) {
    if (dw->config->dblbuff) {
	_dw1000_rx_clear_status_dblbuff(dw, clear);
    } else {
	_dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
			    clear);
    }
    if (clear & DW1000_MSK_SYS_STATUS_ALL_RX_GOOD)
	_dw1000_rx_sync_dblbuff(dw);
}


/**
 * @internal
 * @brief  Recover the receiver from a double buffered overrun
 *
 * @details UM §4.3.3/§4.3.5: a frame arrived while both buffers were
 *          still held by the host, so the buffered data can no longer be
 *          trusted. Drop everything the receiver holds, reset it, and
 *          report it as a receive error so that the host re-arms the
 *          receiver as it does for any other error.
 *
 * @param[in]  dw       driver context
 * @param[in]  status   status word to hand to the rx_error callback
 */
static
void _dw1000_rx_overrun_recover(dw1000_t *dw, uint32_t status) {
    const dw1000_config_t *cfg = dw->config;

    // Both buffers are being dropped, so nothing is held any more and
    // the re-align below must not be suppressed. Cleared first, before
    // the first thing that syncs.
    dw->rx_held = 0;

    // RXOVRR is deliberately absent from the bits cleared here: UM
    // §7.2.17 makes it READ ONLY, so writing 1 to it does nothing.
    //
    // With a transmission in flight the TRXOFF would abort it, and the
    // chip raises no TXFRS for a frame it never finished (UM §7.2.15):
    // drop the receiver's status without it, and put the receiver
    // reset off until dw1000_rx_start(). What belongs to the pending
    // send (tx_pending, wait4resp) is left to its completion.
    //
    // The state is the whole question here: a completion shown in the
    // status word dw1000_process_events() is working on has been booked
    // into it before this runs (DESIGN.md, "One state, one table"), so a
    // send still in TX or TX_W4R is a send still on the air. The state
    // and not the word @p status carries, deliberately: on the call that
    // follows the rx_ok callback that word is read afresh, and a
    // completion that landed while the callback ran is then reported by
    // the next pass rather than swallowed by a TRXOFF here that clears
    // no TX bit and leaves the transmitter looking free.
    if (_dw1000_tx_pending(dw)) {
	_dw1000_rx_drop_status(dw, DW1000_MSK_SYS_STATUS_ALL_RX_GOOD |
				   DW1000_MSK_SYS_STATUS_ALL_RX_ERR  |
				   DW1000_MSK_SYS_STATUS_ALL_RX_TO);
	dw->rx_reset_due = 1;
    } else {
	_dw1000_txrx_off(dw, DW1000_MSK_SYS_STATUS_ALL_RX_GOOD |
			     DW1000_MSK_SYS_STATUS_ALL_RX_ERR  |
			     DW1000_MSK_SYS_STATUS_ALL_RX_TO);
	_dw1000_rx_reset(dw);
    }

    // UM §4.3.5: "The overrun condition and the RXOVRR status bit will
    // be cleared as soon as the host issues the HRBPT command". That is
    // the only thing that clears it, and the re-align inside
    // _dw1000_txrx_off() will not do it: on overrun the IC has wrapped
    // back onto the buffer the host still holds (§4.3.5), so ICRBP ==
    // HSRBP and the conditional toggle issues nothing. Both buffers are
    // being discarded here, so issue HRBPT unconditionally, then
    // re-align, leaving the pointers matched whichever way the chip
    // moved ICRBP. Without this the flag stays set and every later call
    // re-enters this path, tearing the receiver down again each time
    // the rx_error callback re-arms it.
    // UM §7.2.15: only the last byte of SYS_CTRL, where HRBPT lives.
    _dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, 3,
		       (1 << (DW1000_SFT_SYS_CTRL_HRBPT - 24)));
    _dw1000_rx_sync_dblbuff(dw);

    if (cfg->cb.rx_error) {
	cfg->cb.rx_error(dw, status);
    }
}


/* PMSC_STATE, SYS_STATE bits 16..20. User Manual 2.12 documents none
 * of this: register file 0x19 is "reserved" there (7.2.27), and no
 * table of its fields exists in the manual, the errata or the API
 * guide. The values are ruby-dw1000's bench: 1 idle, 2 while a delayed
 * send waits for DX_TIME, 3 for the first microseconds after an enable
 * (the receiver's 16 us start-up, UM 7.2.15), 4 while a frame goes out
 * (TX_STATE 1, preamble), 5 with the receiver enabled, traffic or none
 * (RX_STATE 5). A delayed send is asked of DX_TIME as well, the two
 * agreeing. */
#define _PMSC_STATE(state)      (((state) >> 16) & 0x1F)
#define _PMSC_STATE_TX_WAIT     2
#define _PMSC_STATE_TX          4

/* One millisecond of device ticks, the margin past a frame's airtime
 * before an absent send is called dropped. The transceiver is seen IDLE
 * for a moment after TXSTRT, before the preamble; the margin covers it
 * many times over. */
#define _TX_DROPPED_MARGIN      ((uint32_t)(DW1000_TIME_CLOCK_HZ / 1000))

static int _dw1000_rx_start(dw1000_t *dw, int8_t rx_mode, bool host);

void _dw1000_tx_release(dw1000_t *dw) {
    // The send is over and nobody will be told: its flags stop being
    // this send's here, so they go with it. UM 7.2.17 has TXFRB, TXPRS,
    // TXPHS and TXFRS "automatically cleared at the next transmitter
    // enable", and a TXSTRT the chip drops -- into a listening receiver,
    // or under Errata TX-1 -- is not one, so the next send would inherit
    // them and both tests for "is this send real" read them: the TXFRS
    // branch of dw1000_process_events() would book a completion for a
    // frame that never left, and _dw1000_tx_dropped() would take a
    // standing TXFRB for proof the send is on the air. AAT goes with
    // them, as it does in that branch.
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
			DW1000_MSK_SYS_STATUS_ALL_TX);
    _dw1000_tx_done_state(dw);          // IDLE, or the receiver WAIT4RESP put up
    dw->tx_suspect = 0;
    _dw1000_tx_clock_release(dw);
}

bool _dw1000_tx_dropped(dw1000_t *dw, uint32_t status) {
    if (! _dw1000_tx_pending(dw))
	return false;
    if (status & (DW1000_FLG_SYS_STATUS_TXFRB | DW1000_FLG_SYS_STATUS_TXPRS |
		  DW1000_FLG_SYS_STATUS_TXPHS | DW1000_FLG_SYS_STATUS_TXFRS)) {
	dw->tx_suspect = 0;
	return false;
    }
    const uint32_t state = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATE,
					      DW1000_OFF_NONE);
    const unsigned pmsc  = _PMSC_STATE(state);
    if ((pmsc == _PMSC_STATE_TX) || (pmsc == _PMSC_STATE_TX_WAIT)) {
	dw->tx_suspect = 0;
	return false;
    }
    const uint64_t now = dw1000_get_system_time(dw);
    if (dw->tx_delayed) {
	// A delayed send waits for DX_TIME reading TX_WAIT (measured), a
	// state the manual does not document; DX_TIME itself is asked as
	// well: still ahead, less than half a period away, the send is
	// waiting, not dropped
	uint64_t dx = 0;
	_dw1000_reg_read(dw, DW1000_REG_DX_TIME, DW1000_OFF_NONE,
			 (uint8_t *)&dx, 5);
	dx = dw1000_le64_to_cpu(dx);
	if (((dx - now) & ((1ull << DW1000_TIME_CLOCK_BITS) - 1)) <
	    (1ull << (DW1000_TIME_CLOCK_BITS - 1))) {
	    dw->tx_suspect = 0;
	    return false;
	}
    }
    if (dw->tx_suspect == 0) {
	dw->tx_suspect = now | 1;           // never 0, which means none
	return false;
    }
    // Modulo the 40-bit clock, which wraps every 17.2 s: a pair of
    // sightings straddling a wrap reads short and answers "not yet" once,
    // the suspicion standing, and the next sighting answers right
    const uint64_t elapsed =
	(now - dw->tx_suspect) & ((1ull << DW1000_TIME_CLOCK_BITS) - 1);
    if (elapsed < (uint64_t)dw->tx_airtime + _TX_DROPPED_MARGIN)
	return false;

    // Dropped: the transmitter is free, and nothing is on the air
    dw->state      = DW1000_STATE_IDLE;
    dw->tx_suspect = 0;
    _dw1000_tx_clock_release(dw);
    if (dw->config->cb.tx_dropped)
	dw->config->cb.tx_dropped(dw);
    return true;
}


/**
 * @internal
 * @brief Whether @p status shows a frame whose LDE run was cut
 *
 * @details RXFCG without LDEDONE, and at least one of RXPRD, RXSFDD and
 *          RXPHD: a reception has happened since the last clear, and
 *          its leading edge run never finished. A TRXOFF terminated the
 *          reception after the payload and its CRC were in and before
 *          the run, and the chip posts RXFCG for it all the same
 *          (DW1000.md, "A TRXOFF between RXFCG and LDEDONE leaves the
 *          frame without its timestamp, and the IC pointer where it
 *          was"; 5 of 4482 and 11 of 4415 deliveries over two
 *          duplex soaks).
 *
 *          The detect bits are what tell such a frame from the chip's
 *          record of an earlier reception read with the two buffer
 *          pointers on one buffer (DW1000.md, "With the buffer pointers
 *          aligned"), which carries RXFCG without LDEDONE too and is no
 *          frame at all.
 *
 * @param[in]  status   a SYS_STATUS word
 */
static inline bool _dw1000_rx_lde_pending(uint32_t status) {
    return (status & DW1000_FLG_SYS_STATUS_RXFCG) &&
	   ! (status & DW1000_FLG_SYS_STATUS_LDEDONE) &&
	   (status & (DW1000_FLG_SYS_STATUS_RXPRD  |
		      DW1000_FLG_SYS_STATUS_RXSFDD |
		      DW1000_FLG_SYS_STATUS_RXPHD));
}


/**
 * @internal
 * @brief Put the receiver back where the policy and the host want it
 *
 * @details The one place dw1000_process_events() enables the receiver on
 *          the policy's behalf (DESIGN.md, "The receiver policy, and the
 *          send the chip never began"): the state says whether anything
 *          has it up, and dw->rx_want what the host has asked for:
 *          KEEP, the standing ask of cfg->rx_keep_on, which no enable
 *          spends, or ONCE, a dw1000_rx_start() that came over a send
 *          on the air and is spent here. Nothing of the double buffer is
 *          decided here: the RXENAB the good-frame branch writes is the
 *          buffer's, and the re-enable the TXFRS branch owes a WAIT4RESP
 *          receiver is that receiver's.
 *
 *          Called at the end of the pass, and in the timeout and error
 *          branches once the UM 4.1.6 reset is applied and before the
 *          callback, where the receiver being down costs preambles. One
 *          difference follows from that second use: a start recorded
 *          over a send (ONCE) with the policy off is honoured
 *          in those two branches now, rather than at the end of the
 *          pass, in the one case that can reach them with the
 *          transmitter free, namely that send's completion sitting in
 *          the same status word. The receiver ends the pass in the same
 *          place either way; the enable is written before the callback
 *          instead of after it.
 *
 * @param[in]  dw       driver context
 */
static void _dw1000_rx_reconcile(dw1000_t *dw) {
    if ((dw->rx_want == DW1000_RX_WANT_ONCE) && _dw1000_rx_up(dw))
	dw->rx_want = DW1000_RX_WANT_NONE;  // up already, WAIT4RESP's doing
    if ((dw->state == DW1000_STATE_IDLE) &&
	(dw->rx_want != DW1000_RX_WANT_NONE)) {
	if (dw->rx_want == DW1000_RX_WANT_ONCE)
	    dw->rx_want = DW1000_RX_WANT_NONE;  // spent; KEEP stands
	_dw1000_rx_start(dw, DW1000_RX_IMMEDIATE, false);
    }
}


bool dw1000_process_events(dw1000_t *dw) {
    const dw1000_config_t *cfg = dw->config;

    // Set for events that are handled and reported but do not survive in
    // the status word returned below: the overrun, whose bits are
    // stripped from it once handled, and the frame whose LDE run a
    // TRXOFF cut, whose RXFCG is stripped the same way.
    bool processed = false;
    
    // UM §7.2.17: System Event Status Register
    // It's a 5 bytes register, the last byte contain low-status information
    // ( TXPUTE | RXPREJ | RXRSCS ) we won't read it.
    uint32_t status =
	_dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE); 
#if DW1000_WITH_DEBUG
    dw->dbg_pass[0] = status;
    dw->dbg_pass[1] = dw->dbg_pass[2] = 0;
#endif

    // A send in progress that the chip does not show, past its airtime,
    // never began: cleared and reported here, so that the receive side
    // below stops treating it as on the air
    if (_dw1000_tx_pending(dw) && _dw1000_tx_dropped(dw, status))
	processed = true;

    // The completion this word carries, booked into the state before
    // anything else reads it (DESIGN.md, "One state, one table"). The
    // chip has left TX the moment TXFRS is up, so every branch below is
    // entitled to see the transmitter free; asking "is a send on the
    // air" then means asking the state, and nothing has to carry the
    // status word around to correct it. What the completion owes the
    // chip and the host (the TX group cleared, the forced TX clock
    // released, the hotfixes, the owed receiver reset, tx_done) is
    // reported by the TXFRS branch below, in the order it always had.
    const bool tx_completed = _dw1000_tx_pending(dw) &&
	                      (status & DW1000_FLG_SYS_STATUS_TXFRS);
    if (tx_completed) {
	_dw1000_tx_done_state(dw);      // IDLE, or RX_W4R: the chip's own
	dw->tx_suspect = 0;
    }

    // Handle RX overrun (double buffered receive only)
    //   UM §4.3.3: a frame arrived while both buffers were still held
    //   by the host, so the buffered data can no longer be trusted.
    //   Recover the receiver (transceiver off, receiver reset, buffer
    //   pointers re-aligned) and report it as a receive error, so that
    //   the host re-arms the receiver as it does for any other error.
    if (cfg->dblbuff && (status & DW1000_FLG_SYS_STATUS_RXOVRR)) {
	_dw1000_rx_overrun_recover(dw, status);
	processed = true;

	// Nothing of the receive side is left to handle below
	status &= ~(DW1000_MSK_SYS_STATUS_ALL_RX_GOOD |
		    DW1000_MSK_SYS_STATUS_ALL_RX_ERR  |
		    DW1000_MSK_SYS_STATUS_ALL_RX_TO   |
		    DW1000_FLG_SYS_STATUS_RXOVRR);
    }

    // The frame whose LDE run was cut
    //   UM §7.2.17: LDEDONE says the LDE is done, and the LDE is what
    //   writes RX_TIME; UM table 7 swings it with RXDFR, RXFCG and
    //   RXFCE. A word carrying RXFCG with LDEDONE clear and a detect
    //   bit is a frame whose reception a TRXOFF terminated after the
    //   payload and its CRC were in and before the leading edge run:
    //   the chip posts RXFCG for it all the same, and the bit it will
    //   never post is LDEDONE (DW1000.md, "A TRXOFF between RXFCG and
    //   LDEDONE leaves the frame without its timestamp, and the IC
    //   pointer where it was"; 5 of 4482 and 11 of 4415 deliveries
    //   over two duplex soaks, every one beside this node's own
    //   transmit bits).
    //
    //   There is nothing to wait for: two duplex soaks spent
    //   46 waits of up to 1 ms of the chip's own clock on such words
    //   and not one of them ever saw the bit come up. What
    //   the frame has not got is a timestamp of its own, and RX_TIME
    //   still holds the previous frame's, so it is reported through
    //   rx_error and no payload is offered for it: a survey of every
    //   consumer found none that tests LDEDONE in its
    //   rx_ok, and in SPANK the status word never reaches the place the
    //   timestamp is read at all (PROPAGATE.md, "The frame a node's own
    //   send cut is now a receive error"), so a frame handed over as a
    //   good one is a wrong distance or clock offset in every ranging
    //   host, silently. A node that only listens never sees one: the
    //   cut is this node's own TRXOFF (DESIGN.md, "The frame whose LDE
    //   run was cut").
    const bool lde_cut = _dw1000_rx_lde_pending(status);

    // The word the host is handed for it, kept before the strip below:
    // RXFCG set with LDEDONE clear and a detect bit is what tells this
    // error from every other, and no other word carries it.
    const uint32_t lde_cut_status = status;

    if (lde_cut) {
#if DW1000_WITH_DEBUG
	// How many frames the pass reported as a cut: the bench counts
	// them against the deliveries of the same soak.
	dw->dbg_lde_cuts++;
#endif
	// Out of the good-frame branch, and into the error section at
	// the end of the pass: the bits are taken out of the word this
	// pass works on, so neither the stale strip below nor the RXFCG
	// branch runs for it, and rx_drop carries ALL_RX_GOOD with the
	// drop that section makes (RXFCG was not seen as a frame in
	// this pass), which is what clears them on the chip. No HRBPT
	// is written for it anywhere: UM §4.3.2 moves ICRBP for "a new
	// frame with good CRC" and the cut run never got there, so the
	// host pointer is already on the buffer the chip still owns,
	// and a toggle would move it off (DW1000.md, "A buffer toggle
	// that moves the host off the chip's buffer under a live
	// receiver"). The event does not survive in the word returned
	// below, so the pass counts it here.
	status &= ~(DW1000_FLG_SYS_STATUS_RXFCG  |
		    DW1000_FLG_SYS_STATUS_RXDFR  |
		    DW1000_FLG_SYS_STATUS_LDEDONE);
	processed = true;
    }

    // Double buffered, a good frame shown with the two buffer pointers
    // on the same buffer and none of the detect bits set is not a frame.
    // Measured on rpi-c and rpi-d, 24 of 24 duplicates over 96 runs
    // (INVESTIGATE.md 1b): with the pointers aligned the swinging
    // bits read as the chip's own flags of its last reception, which no
    // status write clears (a masked clear written there reads back set)
    // and which the next receiver enable resets, while the receive
    // registers still select the host's buffer. A frame read out with
    // the receiver off, over a send on the air or beside its
    // completion, is followed by exactly that: the toggle lands the host
    // on the buffer the chip parked on after that frame, the previous
    // frame is still there, and the pass that follows would report it a
    // second time with this frame's flags. RXPRD, RXSFDD and RXPHD are
    // single instances, cleared with every frame reported and set by
    // any reception since, so their absence beside RXFCG is the mark.
    // Nothing to clear and nothing to toggle: the enable that ends this
    // pass (rx_keep_on, or a start recorded over the send) resets the
    // flags, and a host that re-arms from its callbacks has the
    // completion in this same pass to do it from, as it always had.
    if (cfg->dblbuff && (status & DW1000_FLG_SYS_STATUS_RXFCG) &&
	(((status & DW1000_FLG_SYS_STATUS_HSRBP) != 0) ==
	 ((status & DW1000_FLG_SYS_STATUS_ICRBP) != 0)) &&
	! (status & (DW1000_FLG_SYS_STATUS_RXPRD  |
		     DW1000_FLG_SYS_STATUS_RXSFDD |
		     DW1000_FLG_SYS_STATUS_RXPHD))) {
	status &= ~(DW1000_FLG_SYS_STATUS_RXFCG | DW1000_FLG_SYS_STATUS_RXDFR |
		    DW1000_FLG_SYS_STATUS_LDEDONE);
    }

    // Handle RX good frame event
    // We just care about RXFCG, which means everything is ok
    //   RXPRD   : Receiver Preamble Detected status
    //   RXSFDD  : Receiver SFD Detected.
    //   LDEDONE : LDE processing done
    //   RXPHD   : Receiver PHY Header Detect
    //   RXDFR   : Receiver Data Frame Ready
    //   RXFCG   : Receiver FCS Good
    if (status & DW1000_FLG_SYS_STATUS_RXFCG) {
	// Clear all receive status bits
	uint32_t clear = DW1000_MSK_SYS_STATUS_ALL_RX_GOOD;

	// UM §4.3.3: double buffered receive
	//   The frame sits in the host side buffer, and the receive
	//   registers (RX_FINFO, RX_BUFFER, RX_FQUAL, RX_TTCKI, RX_TTCKO,
	//   RX_TIME) are read from it, so the receiver can be re-enabled
	//   right away: the next frame lands in the other buffer while
	//   this one is read out, instead of being lost during that time.
	//   The buffer pointers must not be synced here (the host side one
	//   still designates the buffer being read); the host side pointer
	//   is toggled once the callback has consumed the frame, below.
	//   The rx_ok callback must therefore NOT re-enable the receiver.
	if (cfg->dblbuff) {
	    //   Two registers the read-out needs are *not* in that set:
	    //   DRX_RXPACC_NOSAT (0x27:2C) and LDE_THRESH (0x2E:0000) are
	    //   both absent from UM table 7 (whose swinging set is four
	    //   status bits and register files 0x10 to 0x15), so there is
	    //   a single live instance of each and the next frame's LDE
	    //   run overwrites them. Sample them here, while they still
	    //   belong to the frame being reported, and before the
	    //   receiver is re-enabled below.
	    dw->rxpacc_nosat =
		_dw1000_reg_read16(dw, DW1000_REG_DRX_CONF,
				   DW1000_OFF_DRX_RXPACC_NOSAT);
	    dw->lde_thresh =
		_dw1000_reg_read16(dw, DW1000_REG_LDE_IF,
				   DW1000_OFF_LDE_THRESH);

	    // Not over a transmission, though: this frame arrived before
	    // the send, and the chip is in TX. The receiver is not enabled
	    // on top of that; the completion's handler re-arms it, as it
	    // would after any send. A completion in this same status word
	    // means the send is over, and the receiver goes back on here
	    // as it always did: a host that sees RXFCG beside its
	    // completion and leaves the re-arm to rx_ok counts on it.
#if DW1000_WITH_DEBUG
	    const bool enable_late = ! cfg->rx_enable_early;
#else
	    const bool enable_late = true;
#endif
	    if (tx_completed && enable_late) {
		// Beside the send's own completion: the frame is taken for
		// the response the send may have expected (the stale frame
		// and the response cannot be told apart here), and the
		// receiver is not enabled now but at the end of this pass,
		// once the completion is booked and its flags cleared, through
		// _dw1000_rx_start() and the buffer pointer sync it runs
		// first. That sync is the point: an enable written here and
		// a toggle after it leave the host pointer off the chip's
		// buffer under a live receiver, and the pass that does that
		// loses the next frame unread and the one after to an
		// overrun. Before the stale frame was told apart above, the
		// stale pass toggled exactly so, and rpi-d lost about one
		// frame in fifty with the enable written here against none
		// at the end of the pass; the chip's enable timing was not
		// involved, and since that fix the two placements lose
		// alike. rx_enable_early is kept as the bench knob that
		// measured it. The read-out below runs with the receiver off,
		// this once.
		//
		// IDLE, and the WAIT4RESP of a TX_W4R send consumed with
		// it: the frame reported here is taken for the response
		// that send expected, so nothing is waited for any more,
		// and the receiver the end of the pass puts back is the
		// driver's. This is where the completion booked at the top
		// of the pass ends up as well, the state being written
		// after it rather than before.
		dw->state = DW1000_STATE_IDLE;
		if (dw->rx_want != DW1000_RX_WANT_KEEP)
		    dw->rx_want = DW1000_RX_WANT_ONCE;
	    } else if (! _dw1000_tx_pending(dw)) {
		_dw1000_reg_write16(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_NONE,
				    DW1000_FLG_SYS_CTRL_RXENAB);
		dw->state = DW1000_STATE_RX;
	    }

	    // From here to the toggle below the host side buffer belongs
	    // to this frame and to whoever reads it out, the rx_ok
	    // callback included. _dw1000_rx_sync_dblbuff() honours this.
	    dw->rx_held = 1;
	}

        // Read frame info
	//   and deduce length and ranging
	size_t length;
	bool   ranging;
	dw1000_rx_get_frame_info(dw, &length, &ranging);
	    
        // HOTFIX: From the official deca_device.c:
        //   "Because of a previous frame not being received properly,
        //    AAT bit can be set upon the proper reception of a frame not
        //    requesting for acknowledgement (ACK frame is not actually
        //    sent though). If the AAT bit is set, check ACK request bit
        //    in frame control to confirm"
	//
	// The first two octets are read as an IEEE 802.15.4 frame control
	// field, and nothing here checks that they are one. They are:
	// §7.2.17 sets AAT only "when frame filtering is enabled and a
	// data frame (or MAC command frame) is received (correctly
	// addressed and with a good CRC)", so a frame that got this far
	// with AAT standing is one the chip itself parsed and accepted as
	// 802.15.4. Nor can the bit be left over from a run when filtering
	// was on: the same section has AAT "automatically cleared by the
	// next receiver enable", and the receiver had to be enabled to
	// receive this frame. So the read is sound whenever it happens,
	// and the whole block costs nothing when AAT is clear, which is
	// why no option guards it.
        if ((status & DW1000_FLG_SYS_STATUS_AAT) &&
	    (length >= (2 + DW1000_CRC_LENGTH))) {
	    // Assuming IEEE802.15.4-2011 compliant frames
	    // Get report frame control
	    //  (First 2 bytes of the received frame)
	    uint8_t fctrl[2];
	    dw1000_rx_read_frame_data(dw, fctrl, sizeof(fctrl), 0);

	    if ((fctrl[0] & 0x20) == 0) { 
		// Clear AAT status
		clear  |=  DW1000_FLG_SYS_STATUS_AAT;
		status &= ~DW1000_FLG_SYS_STATUS_AAT;
		// No wait for response. Not the RX_W4R this pass's own
		// completion booked at the top: that send's WAIT4RESP is
		// the receiver the chip has only just put up, which the
		// TXFRS branch below still owes a reset (UM §4.1.6), and
		// the AAT this clears belongs to the frame reported here,
		// not to it.
		if (! tx_completed && (dw->state == DW1000_STATE_RX_W4R))
		    dw->state = DW1000_STATE_RX;
	    }
        }

	// Clear wait4resp internal flag: the receiver this frame came
	// through was the one WAIT4RESP had armed. Not over a send on the
	// air, though: that frame predates the send, whose WAIT4RESP is
	// still armed on the chip. A frame beside the completion in one
	// status word may be either, the stale one or the response; the
	// response is assumed, as it always was.
	//
	// Not the RX_W4R this pass's own completion booked at the top:
	// that send's WAIT4RESP receiver went up after this frame, so
	// this frame did not come through it, and the state has to
	// reach the TXFRS branch below as the RX_W4R that branch
	// expects (dw1000_tx_is_expecting_response(), and the owed
	// receiver reset of UM §4.1.6). HEAD read TX_W4R here and
	// left it alone for the same reason.
	if (! tx_completed && (dw->state == DW1000_STATE_RX_W4R))
	    dw->state = DW1000_STATE_RX;

	// Effectively clearing status
	//   In double buffered mode the bits being cleared are part of
	//   the swinging set and glitch as they go, so the interrupts
	//   are masked around the write (UM §4.3.3, figure 14)
	if (cfg->dblbuff) {
	    _dw1000_rx_clear_status_dblbuff(dw, clear);
#if DW1000_WITH_DEBUG
	    dw->dbg_pass[1] = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS,
						 DW1000_OFF_NONE);
#endif
	} else {
	    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS,
				DW1000_OFF_NONE, clear);
	}

	// Single buffered, the chip is idle once the frame is in, and the
	// callback may well enable the receiver again: said before it runs,
	// so that a start it makes is what stands afterwards. Not over a
	// send still on the air; beside its completion the chip is idle too.
	//
	// A send that expects a response is the exception: beside its own
	// completion the chip is not idle at all, having put its WAIT4RESP
	// receiver up the moment the frame ended. That is the RX_W4R this
	// pass's completion booked at the top, and it is left alone here,
	// so that dw1000_tx_is_expecting_response() answers true in the
	// TXFRS branch of this same pass and the receiver reset owed under
	// UM §4.1.6 is applied there; written over with IDLE it was
	// neither, and the next send went into a listening receiver, where
	// the chip drops it. An RX_W4R no completion of this pass produced
	// is not the exception: the line above has already turned it into
	// RX. (The double buffered branch settles the same case its own
	// way.)
	if (! cfg->dblbuff && ! _dw1000_tx_pending(dw) &&
	    ! (tx_completed && (dw->state == DW1000_STATE_RX_W4R)))
	    dw->state = DW1000_STATE_IDLE;

	// Call the corresponding callback if present
        if (cfg->cb.rx_ok) {
            cfg->cb.rx_ok(dw, status, length, ranging);
        }

        // Toggle the Host side Receive Buffer Pointer
        if (cfg->dblbuff) {
	    // The read-out is over: the buffer may be released, and the
	    // pointers may be aligned again by whoever needs to.
	    dw->rx_held = 0;

	    // UM §4.3.3 (figure 14): RXOVRR is tested *after* the frame has
	    // been read out and before the toggle, not only on entry. The
	    // receiver was re-enabled above, so while the callback ran a
	    // second frame could land in the other buffer and a third
	    // overrun, and HRBPT is the very thing that clears RXOVRR
	    // (§4.3.5, §7.2.17), so toggling unconditionally would erase
	    // the evidence and no later call would ever see it. Take the
	    // overrun path instead of toggling: the receiver is left in
	    // the "errored state which may persist" (§4.3.5) otherwise.
	    const uint32_t ovrr =
		_dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS,
				   DW1000_OFF_NONE);
	    if (ovrr & DW1000_FLG_SYS_STATUS_RXOVRR) {
		_dw1000_rx_overrun_recover(dw, ovrr);
		processed = true;
	    } else {
		// UM §7.2.15: System Control Register
		//  => Only accessing last byte of SYS_CTRL (where is HRBPT
		//     flag) Trigger buffer toggle by writting 1 to HRBPT
		_dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, 3 ,
				  (1 << (DW1000_SFT_SYS_CTRL_HRBPT - 24)));
#if DW1000_WITH_DEBUG
		dw->dbg_pass[2] = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS,
						     DW1000_OFF_NONE);
#endif
	    }
        }
    }

    
    // Handle TX confirmation event.
    // We just care about TXFRS, which means everything is done
    //   AAT   : Automatic Acknowledge Trigger
    //   TXFRB : Transmit Frame Begins
    //   TXPRS : Transmit Preamble Sent
    //   TXPHS : Transmit PHY Header Sent
    //   TXFRS : Transmit Frame Sent
    if (status & DW1000_FLG_SYS_STATUS_TXFRS) {
	// Clear TX events (AAT | TXFRB | TXPRS | TXPHS | TXFRS)
	//   Only using 4 bytes out of 5 (See UM §7.2.17)
        _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
			   DW1000_MSK_SYS_STATUS_ALL_TX);

	// The frame is out: the transmitter was freed at the top of the
	// pass, where the completion was booked into the state, and if a
	// delayed send forced the TX clock on (Errata 1.4 §3.1) it
	// returns to automatic sequencing here
	_dw1000_tx_clock_release(dw);

	// HOTFIX: UM §5.4: Transmit and automatically wait for response
	//   "If the response that is received is a frame requesting an
	//    acknowledgement frame, the DW1000 will transmit the ACK if
	//    automatic acknowledge is enabled, but the receiver will
	//    re-enable following the transmission of the ACK. Depending
	//    on host response times this may allow the
	//    acknowledge-requesting frame to be overwritten, or other
	//    behaviour such as receiver timeouts resulting from the
	//    device being in the RX state rather than in IDLE."
	//
	//  => Force returning to IDLE state (RX off),
	//     if "Automatic Acknowledge Trigger" (AAT) and
	//        "Wait for Response" (wait4resp)
	//
	//  UM §7.2.17 adds that AAT "should be ignored" when automatic
	//  acknowledgement is not enabled: with frame filtering on, a
	//  received frame carrying the ACK request bit latches AAT even
	//  though no ACK is ever sent, and the hotfix in the RXFCG branch
	//  above clears it only when that bit is clear. Without the
	//  AUTOACK test below, a response sent with
	//  DW1000_TX_RESPONSE_EXPECTED would then land here and the
	//  TRXOFF plus receiver reset would tear down the very receiver
	//  WAIT4RESP had just armed, losing the expected response.
        if((dw->reg.sys_cfg & DW1000_FLG_SYS_CFG_AUTOACK) &&
	   (status & DW1000_FLG_SYS_STATUS_AAT) &&
	   (dw->state == DW1000_STATE_RX_W4R)) {
	    // Turn off receiver, returning to IDLE state
	    _dw1000_txrx_off(dw, DW1000_MSK_SYS_STATUS_ALL_TX     |
				 DW1000_MSK_SYS_STATUS_ALL_RX_ERR |
				 DW1000_MSK_SYS_STATUS_ALL_RX_TO  |
				 DW1000_MSK_SYS_STATUS_ALL_RX_GOOD);
	    // Reset in case a frame was already being received
            _dw1000_rx_reset(dw);
        }

	// A receiver reset owed from an error handled while this send was
	// on the air is applied by dw1000_rx_start(), which WAIT4RESP
	// bypasses: the chip enabled the receiver by itself the moment the
	// frame ended, unreset. Take it back down, reset and re-enable it
	// here, before a response can arrive; wait4resp stays armed, the
	// host's tx_done reading it as usual.
	if (dw->rx_reset_due && (dw->state == DW1000_STATE_RX_W4R)) {
	    _dw1000_txrx_off(dw, 0);
	    _dw1000_rx_start(dw, DW1000_RX_IMMEDIATE, false);
	    dw->state = DW1000_STATE_RX_W4R;    // still the chip's own receive
	}

        // Call the corresponding callback if present
        if (cfg->cb.tx_done) {
	    cfg->cb.tx_done(dw, status);
        }
    }

    
    // What the timeout and error branches below drop, on top of their own
    // bits: the frame-ready flags a failed frame leaves behind, unless a
    // good frame was reported in this same pass. That branch has cleared
    // them already, re-enabled the receiver and toggled HRBPT, so the
    // swinging bits now read here are the *next* buffer's, and a frame
    // that landed there while rx_ok ran carries an RXFCG no branch has
    // seen: written over, it is gone, and the sync inside the off would
    // then find nothing latched and hand the buffer back to the chip
    // unread. Reproduced in port/emulation (tests/emulation/dblbuff.c,
    // step_error_beside_a_good_frame): RXAUTR set, RXPHE
    // beside RXFCG in one snapshot, the frame delivered during rx_ok
    // lost with no callback for it.
    //
    // The frame whose LDE run was cut is on the other side of that
    // "unless": its RXFCG was stripped above and no branch reported it
    // as a frame, so ALL_RX_GOOD is in the drop and the bits the cut
    // frame left behind go with it.
    const uint32_t rx_drop = DW1000_MSK_SYS_STATUS_ALL_RX_ERR |
			     DW1000_MSK_SYS_STATUS_ALL_RX_TO  |
			     ((status & DW1000_FLG_SYS_STATUS_RXFCG)
			      ? 0 : DW1000_MSK_SYS_STATUS_ALL_RX_GOOD);

    // Handle frame reception/preamble detect timeout events
    if (status & DW1000_MSK_SYS_STATUS_ALL_RX_TO) {
	// No plain status write here: _dw1000_txrx_off() below clears a
	// superset of these bits from inside the masked window UM §4.3.4
	// (figure 15) requires. Clearing them twice only costs a spurious
	// interrupt (Errata IRQ-1) and a wasted status read per event.

	// Turn off receiver (return to IDLE state), dropping what the
	// receiver raised, the frame-ready flags the failed frame
	// leaves behind included, or the receiver stays confused, but
	// not what the transmitter raised: dw1000_txrx_off() would also
	// clear a completion set since the status snapshot taken above,
	// and nothing would ever report it
	//
	// Unless a transmission is in flight: what the receiver raised
	// before the send is dropped without the TRXOFF, which would abort
	// the frame on the air with no TXFRS ever raised for it (UM
	// §7.2.15), and the reset below is owed to dw1000_rx_start()
	if (_dw1000_tx_pending(dw)) {
	    _dw1000_rx_drop_status(dw, rx_drop);
	    dw->rx_reset_due = 1;
	} else {
	    _dw1000_txrx_off(dw, rx_drop);
	}

	// HOTFIX: UM §4.1.6: RX Message timestamp
	//   "Due to an issue in the re-initialisation of the receiver,
	//    it is necessary to apply a receiver reset after an
	//    error or timeout event.
	//    (It is not necessary to do this for RXPTO and RXSFDTO)"
        if (! _dw1000_tx_pending(dw))
            _dw1000_rx_reset(dw);

	// The receiver back on here, once the reset is applied and before
	// the callback, when the policy or a recorded start wants it: the
	// end of the pass would do the same a few register accesses later,
	// and every microsecond the receiver is down after an error is a
	// preamble missed. Not over a send in flight, which leaves the state
	// in TX or TX_W4R and the reconcile with nothing to do; the sync
	// inside the start keeps a frame completed into the other buffer
	// while the good-frame branch of this pass ran.
	_dw1000_rx_reconcile(dw);

        // Call the corresponding callback if present
        if (cfg->cb.rx_timeout) {
            cfg->cb.rx_timeout(dw, status);
        }
    }

    
    // Handle RX errors events, and the frame whose LDE run a TRXOFF cut
    //   The cut frame is one of them: it has no timestamp of its own and
    //   the driver offers no payload for it, so it takes the same route
    //   an RXPHE or an RXFCE takes, and a host re-arms from rx_error as
    //   it already does for those (DESIGN.md, "The frame whose LDE run
    //   was cut"). Its own bits are dropped with rx_drop above.
    if ((status & DW1000_MSK_SYS_STATUS_ALL_RX_ERR) || lde_cut) {
	// No plain status write here: _dw1000_txrx_off() below clears a
	// superset of these bits from inside the masked window UM §4.3.4
	// (figure 15) requires. RXFCE is part of the double buffered
	// swinging set, so clearing it unmasked glitches the interrupt
	// line (Errata IRQ-1): one spurious interrupt and one wasted
	// status read per CRC error.

	// Turn off receiver (return to IDLE state), dropping what the
	// receiver raised, the frame-ready flags the failed frame
	// leaves behind included, or the receiver stays confused, but
	// not what the transmitter raised: dw1000_txrx_off() would also
	// clear a completion set since the status snapshot taken above,
	// and nothing would ever report it
	//
	// Unless a transmission is in flight: what the receiver raised
	// before the send is dropped without the TRXOFF, which would abort
	// the frame on the air with no TXFRS ever raised for it (UM
	// §7.2.15), and the reset below is owed to dw1000_rx_start()
	if (_dw1000_tx_pending(dw)) {
	    _dw1000_rx_drop_status(dw, rx_drop);
	    dw->rx_reset_due = 1;
	} else {
	    _dw1000_txrx_off(dw, rx_drop);
	}

	// HOTFIX: UM §4.1.6: RX Message timestamp
	//   "Due to an issue in the re-initialisation of the receiver,
	//    it is necessary to apply a receiver reset after an
	//    error or timeout event.
	//    (It is not necessary to do this for RXPTO and RXSFDTO)"
        if (! _dw1000_tx_pending(dw))
            _dw1000_rx_reset(dw);

	// The receiver back on here, once the reset is applied and before
	// the callback, when the policy or a recorded start wants it: the
	// end of the pass would do the same a few register accesses later,
	// and every microsecond the receiver is down after an error is a
	// preamble missed. Not over a send in flight, which leaves the state
	// in TX or TX_W4R and the reconcile with nothing to do; the sync
	// inside the start keeps a frame completed into the other buffer
	// while the good-frame branch of this pass ran.
	_dw1000_rx_reconcile(dw);

        // Call the corresponding callback if present
	//   The cut frame is handed the word the pass was entered with,
	//   RXFCG set and LDEDONE clear, and not the stripped one: those
	//   two bits beside a detect bit are how a host that wants to
	//   count these tells them from a CRC error.
        if (cfg->cb.rx_error) {
            cfg->cb.rx_error(dw, lde_cut ? lde_cut_status : status);
        }
    }

    // rx_keep_on: the receiver back on, once, after every callback has
    // run, if the host wants it and nothing has it up: not over a send
    // on the air, whose completion pass will get here too. The same
    // reconcile the timeout and error branches ran is what closes the
    // pass, so a branch never writes RXENAB on the policy's behalf and
    // the order of the branches does not matter to it.
    _dw1000_rx_reconcile(dw);

    return processed || (status & (DW1000_FLG_SYS_STATUS_RXFCG     |
				   DW1000_FLG_SYS_STATUS_TXFRS     |
				   DW1000_MSK_SYS_STATUS_ALL_RX_TO |
				   DW1000_MSK_SYS_STATUS_ALL_RX_ERR));
}


// Operations common to TX/RX
//----------------------------------------------------------------------

inline
void dw1000_txrx_set_time(dw1000_t *dw, uint64_t time) {
    // UM §3.3: the low 9 bits of the given delay are ignored
    time = dw1000_cpu_to_le64(time);
    _dw1000_reg_write(dw, DW1000_REG_DX_TIME, DW1000_OFF_NONE, &time, 5);
}


void _dw1000_txrx_off(dw1000_t *dw, uint32_t clear) {
    // Save interrupt mask
    uint32_t sys_mask =
	_dw1000_reg_read32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE);

    // Clear interrupt mask
    _dw1000_reg_write32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE, 0);

    // Disable the radio
    _dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_NONE,
		       DW1000_FLG_SYS_CTRL_TRXOFF);

    
    // UM §7.2.17: System Event Status Register
    // Clear the requested events bits (done by writting 1 to them)
    if (clear != 0)
	_dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE, clear);

    // Re-align the double buffer pointers, but only when the received
    // frames are being dropped along with their status: in double
    // buffered mode a frame reported and not yet read out sits in the
    // host side buffer, and syncing the pointers would hand that buffer
    // back to the chip (dw1000_txrx_idle() is used before a transmit
    // for exactly that case).
    if (clear & DW1000_MSK_SYS_STATUS_ALL_RX_GOOD)
	_dw1000_rx_sync_dblbuff(dw);

    // Reset internal flags
    //   The transceiver is off, so whatever was being transmitted is
    //   over one way or the other and the next send may go ahead.
    dw->state      = DW1000_STATE_IDLE;
    dw->tx_suspect = 0;

    // A delayed send may have been armed and is being cancelled here, so
    // the TX clock it forced on (Errata 1.4 §3.1) has to be released too
    _dw1000_tx_clock_release(dw);

    // Restore interrupt mask
    _dw1000_reg_write32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE, sys_mask); 
}


// Transmission (TX)
//----------------------------------------------------------------------

void dw1000_tx_set_rx_activation_delay(dw1000_t *dw, uint32_t delay) {
    DW1000_ASSERT(delay <= DW1000_MAX_TX_RX_ACTIVATION_DELAY,
		  "out of range delay");

    uint32_t val =
	_dw1000_reg_read32(dw, DW1000_REG_ACK_RESP_T, DW1000_OFF_NONE);

    val &= ~DW1000_MSK_ACK_RESP_T_W4R_TIM;
    val |= delay & DW1000_MSK_ACK_RESP_T_W4R_TIM;

    _dw1000_reg_write32(dw, DW1000_REG_ACK_RESP_T, DW1000_OFF_NONE, val);
}


uint32_t dw1000_tx_get_power(dw1000_t *dw) {
    return _dw1000_reg_read32(dw, DW1000_REG_TX_POWER, DW1000_OFF_NONE);
}

uint8_t dw1000_tx_power_to_05db(uint32_t txpower) {
    // dw1000_configure() writes the same byte into bits 23..16 and 15..8;
    // either answers, so take the first. The coarse field holds
    // (6 - coarse) in 3 bits and the fine field 5 bits, which is what the
    // encoder in _dw1000_radio_tuning() builds; see its UM 7.2.31.1 note.
    uint8_t byte   = (txpower >> 16) & 0xFF;
    uint8_t coarse = 6 - ((byte >> 5) & 0x07);
    uint8_t fine   = byte & 0x1F;
    uint8_t power  = coarse * 5 + fine;

    // The encoder clamps at 61; a word from elsewhere need not have been
    // built by it, and 61 is the top of the range either way.
    return power > 61 ? 61 : power;
}


void dw1000_tx_fctrl(dw1000_t *dw, size_t length, size_t offset,
		     int tx_mode) {
    /* The standard PHR carries a 7-bit length, so a frame longer than
     * 127 bytes needs the proprietary long frame mode (PHR_MODE = 11,
     * UM §7.2.10 p. 75 and §3.4 p. 27). TFLEN+TFLE is 10 bits wide
     * either way: without long frames the extra three bits are written
     * but cannot be carried on air, so the limit is a property of the
     * PHR mode, not of the field width.
     *
     * Computed into a local rather than spelled inside the
     * DW1000_ASSERT() argument list: C11 §6.10.3 ¶11 makes a
     * preprocessor directive inside a macro argument list undefined
     * behaviour, and cppcheck reacts to one by refusing to expand the
     * macro and abandoning the rest of the file.
     */
    const size_t max_length = dw1000_tx_get_frame_maxsize(dw);
    DW1000_ASSERT(length <= max_length, "bad frame length");
    dw->tx_length = (uint16_t)length;

    // TXBOFFS is a 10-bit field; a larger offset would corrupt the
    // neighbouring TX_FCTRL bits
    const size_t max_offset =
	DW1000_MSK_TX_FCTRL_TXBOFFS    >> DW1000_SFT_TX_FCTRL_TXBOFFS;
    DW1000_ASSERT(offset <= max_offset, "bad buffer offset");

    // The asserts above are compiled out on four of the five ports, so
    // both values are clamped as well before they are shifted into
    // place. TFLEN+TFLE occupy bits 0-9 and TXBOFFS bits 22-31: an
    // out of range length shifts into TXBR (bits 13-14) and silently
    // changes the on-air bitrate, which is a far worse failure than the
    // truncated frame clamping gives, and the same silent-clamp
    // behaviour dw1000_tx_write_frame_data() already applies to the
    // buffer write this length describes.
    if (length > max_length) length = max_length;
    if (offset > max_offset) offset = max_offset;

    uint32_t tx_fctrl = dw->reg.tx_fctrl;
    if (tx_mode & DW1000_TX_RANGING)
	tx_fctrl |= DW1000_FLG_TX_FCTRL_TR;

    tx_fctrl |=
	(length << DW1000_SFT_TX_FCTRL_TFLEN)   |
	(offset << DW1000_SFT_TX_FCTRL_TXBOFFS) ;

    _dw1000_reg_write32(dw, DW1000_REG_TX_FCTRL, 0, tx_fctrl);
}


void dw1000_tx_write_frame_data(dw1000_t *dw,
			  uint8_t *data, size_t length, size_t offset) {
    // Protect device from buffer overflow
    //  (Computed without overflow to avoid bypassing the clamp)
    if (offset >= 1024)
	return;
    if (length > 1024 - offset)
	length = 1024 - offset;

    // Write data
    _dw1000_reg_write(dw, DW1000_REG_TX_BUFFER, offset, data, length);
}


/* Airtime of the send in progress, in device ticks: preamble and SFD
 * (tx_ton, APS022 5.4), the PHR, and the payload with its Reed-Solomon
 * parity (48 bits per 330-bit block, UM 10.2). The PHR goes at 850 kbps
 * except at 110 kbps, where it goes at 110. A bound, not a timestamp:
 * dropped-send detection adds a millisecond to it. */
static uint32_t _dw1000_tx_airtime(const dw1000_t *dw) {
    static const uint32_t kbps[3] = { 110, 850, 6800 };
    const unsigned br   = (dw->radio.bitrate < 3) ? dw->radio.bitrate : 2;
    const uint64_t phr  = 21ull * DW1000_TIME_CLOCK_HZ
			/ ((br == DW1000_BITRATE_110KBPS) ? 110000ull
							  : 850000ull);
    const uint64_t bits = (uint64_t)dw->tx_length * 8ull * 378ull / 330ull;
    const uint64_t data = bits * DW1000_TIME_CLOCK_HZ / ((uint64_t)kbps[br] * 1000ull);
    return dw->tx_ton + (uint32_t)phr + (uint32_t)data;
}

int dw1000_tx_start(dw1000_t *dw, int tx_mode) {
    uint8_t sys_ctrl  = DW1000_FLG_SYS_CTRL_TXSTRT;

    // Errata 1.4 §3.3 (TX-2): "When preparing the transmission of the
    // next frame, by writing to a part of the TX buffer which is not
    // used for the current transmission, the new data written is written
    // erroneously at offset 0, thus corrupting the data currently being
    // transmitted." The send functions refuse before they write, which
    // is what actually prevents that; this catches the caller driving
    // the transmitter by hand, too late to save the frame in flight but
    // in time to say so rather than start a second one on top of it.
    //
    // dw->tx_pending is the driver's own record, so a free transmitter
    // costs no SPI. A pending one is asked of the chip once: a completion
    // standing there is over and consumed by this send, a send with no
    // flag may have been dropped (_dw1000_tx_dropped()), and only a frame
    // on the air is refused.
    if (_dw1000_tx_pending(dw)) {
	const uint32_t status =
	    _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
	if (status & DW1000_FLG_SYS_STATUS_TXFRS)
	    _dw1000_tx_release(dw);
	else if (! _dw1000_tx_dropped(dw, status))
	    return DW1000_TX_ERR_BUSY;
    }

    // Set wait for response flag
    if (tx_mode & DW1000_TX_RESPONSE_EXPECTED) {
	sys_ctrl |= DW1000_FLG_SYS_CTRL_WAIT4RESP;
    }

    // Set delayed start flag
    if (tx_mode & DW1000_TX_DELAYED_START)
        sys_ctrl |= DW1000_FLG_SYS_CTRL_TXDLYS;

    // Set suppression of auto-FCS transmission
    if (tx_mode & DW1000_TX_NO_AUTO_CRC) {
	sys_ctrl |= DW1000_FLG_SYS_CTRL_SFCST;
    }

    // Errata 1.4 §3.1 (TX-1): a delayed send whose time falls in the
    // band just after the TXPUTE window is silently dropped, with
    // neither HPDWARN nor TXPUTE raised and no TX done event to follow.
    // Forcing the TX clock on before TXDLYS|TXSTRT is the workaround the
    // erratum gives; releasing it again is ours, on TXFRS, on the late
    // path below, or in _dw1000_txrx_off(), whichever comes first.
    if (tx_mode & DW1000_TX_DELAYED_START)
	_dw1000_tx_clock_force(dw, true);

    // What the transmitter raised for an earlier frame must be gone
    // before this one is pending, so that a completion the chip shows
    // with a send pending is this send's: dw1000_process_events() and
    // dw1000_rx_start() read TXFRS to tell a send on the air from one
    // that is over, and dw1000_txrx_idle() leaves a standing one in
    // place by design. UM 7.2.17 has TXFRB, TXPRS, TXPHS and TXFRS
    // "automatically cleared at the next transmitter enable", which an
    // immediate TXSTRT is, so nothing is written for one. A delayed
    // send is enabled at DX_TIME, and the clearing waits for that
    // moment (measured: a standing TXFRS stays set through the whole
    // wait), so for it the four are cleared here, one write, rather
    // than have a stale TXFRS read as this send's completion while it
    // waits.
    if (tx_mode & DW1000_TX_DELAYED_START)
	_dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
			    DW1000_FLG_SYS_STATUS_TXFRB | DW1000_FLG_SYS_STATUS_TXPRS |
			    DW1000_FLG_SYS_STATUS_TXPHS | DW1000_FLG_SYS_STATUS_TXFRS);

    // Write to SYS_CTRL register, which will trigger transmit
    _dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_SYS_CTRL, sys_ctrl);
    dw->state      = (tx_mode & DW1000_TX_RESPONSE_EXPECTED)
		   ? DW1000_STATE_TX_W4R : DW1000_STATE_TX;
    dw->tx_suspect = 0;
    dw->tx_delayed = (tx_mode & DW1000_TX_DELAYED_START) ? 1 : 0;
    dw->tx_airtime = _dw1000_tx_airtime(dw);

    // Perform extra check for delayed transmit
    if (tx_mode & DW1000_TX_DELAYED_START) {
	// UM §7.2.17: System Event Status Register
	//  => Status is a 5 bytes register (DW1000_REG_SYS_STATUS),
	//     we will read the last 2 bytes (ie: offset 3)
	//     which contains the TXPUTE (34) and HPDWARN (27) flags
	const uint16_t msk =
	    (1 << (DW1000_SFT_SYS_STATUS_HPDWARN - 24)) |
	    (1 << (DW1000_SFT_SYS_STATUS_TXPUTE  - 24)) ;
	const size_t   off = 3;
	    
	// Check status
	uint16_t tx_ok = 0;
        tx_ok = _dw1000_reg_read16(dw, DW1000_REG_SYS_STATUS, off);
        if ((tx_ok & msk) == 0)
            return 0;
#if DW1000_WITH_DEBUG
	dw->tx_late_flags = tx_ok & msk;
#endif

	// From official deca_device.c:
	// Transmit Delayed Send set over Half a Period away or Power Up error
	// (there is enough time to send but not to power up individual blocks)
	// ==> Cancel delayed send

	// As we are turning off the transceiver (TRXOFF), we can blow
	// as well other flags
	_dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_SYS_CTRL,
			   DW1000_FLG_SYS_CTRL_TRXOFF);
	dw->state      = DW1000_STATE_IDLE;

	// Nothing is pending anymore: let the TX clock be sequenced again
	// (Errata 1.4 §3.1)
	_dw1000_tx_clock_release(dw);

	return DW1000_TX_ERR_TOO_LATE;
    }

    return 0;
}


// Reception (RX)
//----------------------------------------------------------------------

void dw1000_rx_set_timeout(dw1000_t *dw, uint16_t timeout) {
    // UM §7.2.14: Receive Frame Wait Timeout Period
    if (timeout > 0) {
        _dw1000_reg_write16(dw, DW1000_REG_RX_FWTO, DW1000_OFF_NONE, timeout);
    }
    
    // UM §7.2.6 : System Configuration
    if (timeout > 0) { dw->reg.sys_cfg |=  DW1000_FLG_SYS_CFG_RXWTOE; }
    else             { dw->reg.sys_cfg &= ~DW1000_FLG_SYS_CFG_RXWTOE; }
    _dw1000_reg_write32(dw, DW1000_REG_SYS_CFG, DW1000_OFF_NONE,
			dw->reg.sys_cfg);
}


void dw1000_rx_set_frame_filtering(dw1000_t *dw, uint16_t bitmask) {
    // Read System Configuration register (and hide reserved bits)
    uint32_t sys_cfg =
	_dw1000_reg_read32(dw, DW1000_REG_SYS_CFG, 0) & DW1000_MSK_SYS_CFG;

    if (bitmask) {
	// Sanity check bitmask
	//   (bitmaks is a mapping on a subset of SYS CFG register)
	bitmask &=  DW1000_MSK_SYS_CFG_FF_ALL;
        // Apply bitmask
        sys_cfg &= ~DW1000_MSK_SYS_CFG_FF_ALL; 
        sys_cfg |=  bitmask;
	// Enable filtering
	sys_cfg |=  DW1000_FLG_SYS_CFG_FFEN;
    } else {
	// Disable filtering (as bitmask is empty)
        sys_cfg &= ~DW1000_FLG_SYS_CFG_FFEN;
    }

    _dw1000_reg_write32(dw, DW1000_REG_SYS_CFG, DW1000_OFF_NONE, sys_cfg);
    // Save it for internal usage
    dw->reg.sys_cfg = sys_cfg;
}


/* The start itself. A host's call (host true) speaks for the host: it
 * clears a start recorded over a send, and under rx_keep_on it says
 * the host wants to listen, on the paths that leave the receiver up. The
 * driver's own starts, at the end of a pass and at a WAIT4RESP
 * completion, say nothing of the kind. */
static int _dw1000_rx_start(dw1000_t *dw, int8_t rx_mode, bool host) {
    const bool wanted = host && dw->config->rx_keep_on;

    // Not over a transmission on the air: recorded, and honoured at the
    // end of the completion's dw1000_process_events(). A pending send
    // that is over, or that never began, is cleared here first.
    if (_dw1000_tx_pending(dw)) {
	const uint32_t status =
	    _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
	if (! (status & DW1000_FLG_SYS_STATUS_TXFRS) &&
	    ! _dw1000_tx_dropped(dw, status)) {
	    if (host)
		dw->rx_want = wanted ? DW1000_RX_WANT_KEEP
				     : DW1000_RX_WANT_ONCE;
	    return 0;
	}
    }
    // A start the host makes now supersedes one it recorded over a send.
    // The policy's standing ask is not a recorded start and survives it,
    // unless the policy itself is off, in which case nothing reads it.
    if (host)
	dw->rx_want = (wanted && (dw->rx_want == DW1000_RX_WANT_KEEP))
	    ? DW1000_RX_WANT_KEEP : DW1000_RX_WANT_NONE;

    // A receiver reset owed from an error or timeout handled while a
    // transmission was in flight (UM §4.1.6): applied now that the
    // receiver is being brought up
    if (dw->rx_reset_due) {
	dw->rx_reset_due = 0;
	_dw1000_rx_reset(dw);
    }

    // Sync double buffer unless explicitely disabled
    if (! (rx_mode & DW1000_RX_NO_DBLBUFF_SYNC)) {
        _dw1000_rx_sync_dblbuff(dw);
    }

    // Trigger receiving by writting to SYS_CTRL
    // UM §7.2.15: System Control Register
    // We will just access the 2 lower bytes to
    //  enable radio, and delayed received if requested
    uint16_t sys_ctrl = DW1000_FLG_SYS_CTRL_RXENAB;
    if (rx_mode & DW1000_RX_DELAYED_START) {
        sys_ctrl |= DW1000_FLG_SYS_CTRL_RXDLYE ;
    }
    _dw1000_reg_write16(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_NONE, sys_ctrl);
    dw->state = DW1000_STATE_RX;

    // Check for errors if delayed start was requested
    if (rx_mode & DW1000_RX_DELAYED_START) {
	// UM §7.2.17: System Event Status Register
	//  => HPDWARN is in the 4th byte
	uint8_t sys_status = _dw1000_reg_read8(dw, DW1000_REG_SYS_STATUS, 3);
        if ((sys_status & (DW1000_FLG_SYS_STATUS_HPDWARN >> 24)) != 0)  {
	    // Too late: back to IDLE, dropping what the receiver raised
	    _dw1000_txrx_off(dw, DW1000_MSK_SYS_STATUS_ALL_RX_GOOD |
			         DW1000_MSK_SYS_STATUS_ALL_RX_ERR  |
			         DW1000_MSK_SYS_STATUS_ALL_RX_TO);
	    // Stay idle if asked: the host did not want the receiver
	    // on at any other time, so it is not wanted either
            if (rx_mode & DW1000_RX_IDLE_ON_DELAY_ERROR)
		return DW1000_RX_ERR_TOO_LATE;
	    // Fallback to immediate start
	    _dw1000_reg_write16(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_NONE,
				DW1000_FLG_SYS_CTRL_RXENAB);
	    dw->state = DW1000_STATE_RX;
	    if (wanted)
		dw->rx_want = DW1000_RX_WANT_KEEP;
	    return 1;
        }
    }

    if (wanted)
	dw->rx_want = DW1000_RX_WANT_KEEP;
    return 0;
}

int dw1000_rx_start(dw1000_t *dw, int8_t rx_mode) {
    return _dw1000_rx_start(dw, rx_mode, true);
}


inline
void dw1000_rx_read_frame_data(dw1000_t *dw,
			       uint8_t *data, size_t length, size_t offset) {
    // Protect device from overreading the buffer
    //  (Computed without overflow to avoid bypassing the clamp)
    if (offset >= 1024)
	return;
    if (length > 1024 - offset)
	length = 1024 - offset;

    // Read data
    _dw1000_reg_read(dw, DW1000_REG_RX_BUFFER, offset, data, length);
}


void dw1000_rx_get_info(dw1000_t *dw, dw1000_rxinfo_t *rxinfo) {
    // First path index (UM §7.2.23)
    rxinfo->first_path =
	_dw1000_reg_read16(dw, DW1000_REG_RX_TIME, DW1000_OFF_RX_TIME_FP_INDEX);

    // Standard deviation of noise (UM §7.2.20)
    rxinfo->std_noise =
	_dw1000_reg_read16(dw, DW1000_REG_RX_FQUAL, DW1000_OFF_RX_FQUAL_STD_NOISE);

    // LDE threshold (UM §7.2.47.1)
    // Not part of the double buffered swinging set of UM table 7: in
    // double buffered mode the receiver is re-enabled before this
    // callback runs, so the live register may already hold the next
    // frame's LDE result. Use the value sampled for this frame, as
    // _dw1000_rx_get_pacc_count() does for DRX_RXPACC_NOSAT.
    rxinfo->max_noise = dw->config->dblbuff
	? dw->lde_thresh
	: _dw1000_reg_read16(dw, DW1000_REG_LDE_IF, DW1000_OFF_LDE_THRESH);
}


void dw1000_rx_get_time_tracking(dw1000_t *dw,
				 int32_t *offset, uint32_t *interval) {
    // UM §7.2.21: Receiver Time Tracking Interval
    if (interval) {
	*interval =
	    _dw1000_reg_read32(dw, DW1000_REG_RX_TTCKI, DW1000_OFF_NONE);
    }

    // UM §7.2.22: Receiver Time Tracking Offset
    if (offset) {
	// Sign extending see:
	//   http://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend
	//
	//   unsigned b; // number of bits representing the number in x
	//   int x;      // sign extend this b-bit number to r
	//   int r;      // resulting sign-extended number
	//   int const m = 1U << (b - 1); 
	//
	//   x = x & ((1U << b) - 1); 
	//   r = (x ^ m) - m;
	//
	int32_t x =
	    _dw1000_reg_read32(dw, DW1000_REG_RX_TTCKO, DW1000_OFF_NONE) &
	       DW1000_MSK_RX_TTCKO_RXTOFS;
	const int32_t m = 1U << (DW1000_LEN_RX_TTCKO_RXTOFS-1);
	*offset = (x ^ m) - m;
    }
}


void dw1000_rx_get_power_estimate(dw1000_t *dw,
				  double *signal, double *firstpath) {
    // PRF of 4MHZ is unsupported by DW1000
    DW1000_ASSERT((dw->radio.prf == DW1000_PRF_16MHZ) ||
		  (dw->radio.prf == DW1000_PRF_64MHZ),
		  "unsupported PRF value");

    // UM §4.7.1/§4.7.2: "A = is the constant 113.77 for a PRF of 16 MHz,
    // or, the constant 121.74 for a PRF of 64 MHz" (unchanged in UM 2.05,
    // 2.09 and 2.15).
    //  PRF     4   16        64
    //  A       -   113.77    121.74
    double N  = (double) _dw1000_rx_get_pacc_count(dw);
    double A  = dw->radio.prf == DW1000_PRF_16MHZ ? 113.77 : 121.74;

    // N divides both estimates below. The SFD adjustment in
    // _dw1000_rx_get_pacc_count() can legitimately leave it at 0, and so
    // can a failed SPI read (the OSAL contract zeroes the buffer). 0 here
    // would yield +inf / NaN, which is worse than an obvious sentinel:
    // every comparison against NaN is false, so dw1000_rx_power_correction()
    // would hand it straight back and the caller would see it as a reading.
    if (N < 1.0) {
	if (signal)    *signal    = -INFINITY;
	if (firstpath) *firstpath = -INFINITY;
	return;
    }

    // Firstpath power
    if (firstpath) {
        // UM §7.2.23: Receive Time Stamp
	// UM §7.2.20: RX Frame Quality Information
	double F1 = (double) _dw1000_reg_read16(dw,
			DW1000_REG_RX_TIME,  DW1000_OFF_RX_TIME_FP_AMPL1);
	double F2 = (double) _dw1000_reg_read16(dw,
			DW1000_REG_RX_FQUAL, DW1000_OFF_RX_FQUAL_FP_AMPL2);
	double F3 = (double) _dw1000_reg_read16(dw,
			DW1000_REG_RX_FQUAL, DW1000_OFF_RX_FQUAL_FP_AMPL3);
	*firstpath= 10.0 * log10((F1*F1 + F2*F2 + F3*F3) / (N*N)) - A;
    }

    // Signal power
    if (signal) {
	double C  = (double) _dw1000_reg_read16(dw,DW1000_REG_RX_FQUAL,
						   DW1000_OFF_RX_FQUAL_CIR_PWR);
	*signal   = 10.0 * log10((C * 131072.0) / (N * N)) - A;
    }
}


double dw1000_rx_power_correction(dw1000_t *dw, double p) {
    // UM §4.7: [Figure 22]: Estimated RX level versus actual RX level
    switch (dw->radio.prf) {
    case DW1000_PRF_16MHZ:
	// Approximated by segment:
	// Estimated: -105 / -88 / -81
	// Real     : -105 / -88 / -65
	if (p > -88) p += (p + 88) * 2.2857;
	break;
	
    case DW1000_PRF_64MHZ:
	// Approximated by segment:
	// Estimated: -105 / -88 / -81 / -79
	// Real     : -105 / -88 / -78 / -66.5
	if (p > -88) p += (p + 88) * 0.42857;
	if (p > -78) p += (p + 78) * 3.0;
	break;

    case DW1000_PRF_4MHZ:
	// PRF of 4MHZ is unsupported by DW1000
	// FALLTHROUGH
    default:
	DW1000_ASSERT(0, "unsupported PRF value");
	break;
    }

    // XXX: is it better or worst to trim it?
    if (p > -60)
	p = -60;

    return p;
}


#if DW1000_WITH_EVENT_COUNTERS
/*===========================================================================*/
/* Event counters                                                            */
/*===========================================================================*/

void dw1000_event_counters_start(dw1000_t *dw) {
    // UM §7.2.48.1: the bits are self-clearing and the register takes a
    // two-byte minimum write, "if a one-byte write is made to this
    // register, the bits will not clear as expected".
    _dw1000_reg_write16(dw, DW1000_REG_DIG_DIAG, DW1000_OFF_EVC_CTRL,
			DW1000_FLG_EVC_CTRL_EVC_EN);
}


void dw1000_event_counters_clear(dw1000_t *dw) {
    // UM §7.2.48.1 prescribes the order: EVC_CLR does nothing while
    // EVC_EN stands, so 0x02 stops the counters and zeroes them, then
    // 0x01 starts them again. There is no third state; counting cannot
    // be turned off once it has been turned on, only cleared.
    _dw1000_reg_write16(dw, DW1000_REG_DIG_DIAG, DW1000_OFF_EVC_CTRL,
			DW1000_FLG_EVC_CTRL_EVC_CLR);
    _dw1000_reg_write16(dw, DW1000_REG_DIG_DIAG, DW1000_OFF_EVC_CTRL,
			DW1000_FLG_EVC_CTRL_EVC_EN);
}


void dw1000_event_counters_read(dw1000_t *dw, dw1000_event_counters_t *evc) {
    DW1000_ASSERT(evc, "no event counter structure");

    // The twelve counters run contiguously from EVC_PHE to EVC_TPW, so
    // one transaction takes the lot and all twelve agree on when they
    // were sampled. Each is a 12-bit value in a little endian 16-bit
    // field (UM §7.2.48), read-only, and not cleared by being read.
    uint8_t buf[DW1000_OFF_EVC_TPW + 2 - DW1000_OFF_EVC_PHE];
    _dw1000_reg_read(dw, DW1000_REG_DIG_DIAG, DW1000_OFF_EVC_PHE,
		     buf, sizeof(buf));

    uint16_t *out[] = {
	&evc->phe,  &evc->rse,  &evc->fcg, &evc->fce,
	&evc->ffr,  &evc->ovr,  &evc->sto, &evc->pto,
	&evc->fwto, &evc->txfs, &evc->hpw, &evc->tpw,
    };

    for (size_t i = 0 ; i < (sizeof(out) / sizeof(out[0])) ; i++) {
	uint16_t v = (uint16_t)buf[2*i] | ((uint16_t)buf[2*i + 1] << 8);
	*out[i] = v & DW1000_MSK_EVC_COUNT;
    }
}
#endif



#if DW1000_WITH_ACCUMULATOR
/*===========================================================================*/
/* Accumulator (CIR)                                                         */
/*===========================================================================*/

size_t dw1000_rx_read_accumulator(dw1000_t *dw, uint16_t index,
				  uint8_t *data, size_t size) {
    DW1000_ASSERT(data, "no accumulator buffer");

    // One octet of every transaction is spent on the dummy below, so a
    // one byte buffer carries no sample at all.
    if ((size < 2) || (index >= DW1000_LEN_ACC_MEM))
	return 0;

    // Never read past the end of the accumulator, and never write past
    // the end of the caller's buffer: the shorter of the two wins.
    size_t avail = (size_t)DW1000_LEN_ACC_MEM - index;
    if ((size - 1) > avail)
	size = avail + 1;

    // UM §7.2.38 leaves the accumulator readable only while its memory
    // is clocked, which the sequencer does not arrange on its own.
    _dw1000_clocks(dw, DW1000_CLOCK_ACC_READ);
    _dw1000_reg_read(dw, DW1000_REG_ACC_MEM, index, data, size);
    _dw1000_clocks(dw, DW1000_CLOCK_ACC_DONE);

    // UM §7.2.38: "Because of an internal memory access delay when
    // reading the accumulator the first octet output is a dummy octet
    // that should be discarded. This is true no matter what sub-index
    // the read begins at." Dropped here rather than left to the caller,
    // so that data[0] is the octet at `index` and the buffer is indexed
    // the way the accumulator is. deca_device.c leaves it in place and
    // documents the off-by-one into its callers instead.
    memmove(data, data + 1, size - 1);

    return size - 1;
}
#endif



#if DW1000_WITH_TEMP_COMPENSATION
/*===========================================================================*/
/* Temperature corrections                                                   */
/*===========================================================================*/

/* Transmit power droops as the die warms, near enough linearly. APS023
 * Part 2 §5.3 gives the slope, from testing over a number of parts:
 * 0.035 dB/°C on channel 2 and 0.065 dB/°C on channel 5. No figure is
 * published for any other channel, and none is invented here.
 *
 * Held as half-dB steps per hundredth of a degree, scaled by 10000, so
 * that the correction is integer throughout: 0.035 dB/°C is 0.07 half-dB
 * per °C is 7/10000 half-dB per 1/100 °C. The unit is that of
 * dw1000_read_temp_vbat(), so the difference of two of its readings is
 * this function's argument unconverted.
 */
#define DW1000_TEMP_COMP_05DB_CH2                 7
#define DW1000_TEMP_COMP_05DB_CH5                13
#define DW1000_TEMP_COMP_SCALE                10000

/* UM §7.2.31.1: the gain control range is 30.5 dB, which is 61 half-dB
 * steps, of 6 coarse (DA) steps of 2.5 dB and 31 fine (mixer) steps of
 * 0.5 dB. */
#define DW1000_TX_POWER_05DB_MAX                 61


/**
 * @internal
 * @brief One transmit power octet, as a level in half-dB steps
 *
 * @details UM §7.2.31.1 (figure 26): bits 7:5 carry the coarse gain as
 *          (6 - coarse) and bits 4:0 the fine gain. The same encoding
 *          _dw1000_radio_tuning() builds and dw1000_tx_power_to_05db()
 *          reads, spelt once more here because this works octet by
 *          octet where that takes the register's word.
 */
static inline
uint8_t _dw1000_tx_power_octet_to_05db(uint8_t octet) {
    uint8_t coarse = 6 - ((octet >> 5) & 0x07);
    uint8_t fine   = octet & 0x1F;
    uint8_t level  = coarse * 5 + fine;

    return level > DW1000_TX_POWER_05DB_MAX
	       ? DW1000_TX_POWER_05DB_MAX : level;
}


/**
 * @internal
 * @brief A level in half-dB steps, as a transmit power octet
 *
 * @details The coarse steps are taken before the remainder goes to the
 *          fine ones: UM §7.2.31.1 asks for the coarse gain to be
 *          adjusted first, for the best spectral shape.
 */
static inline
uint8_t _dw1000_tx_power_05db_to_octet(uint8_t level) {
    if (level > DW1000_TX_POWER_05DB_MAX)
	level = DW1000_TX_POWER_05DB_MAX;

    uint8_t coarse = level / 5;
    if (coarse > 6)
	coarse = 6;
    uint8_t fine   = level - coarse * 5;

    return (uint8_t)(((6 - coarse) << 5) | fine);
}


uint32_t dw1000_tx_power_temp_correction(dw1000_t *dw, uint32_t txpower,
					 int16_t delta_temp) {
    int32_t slope;

    switch (dw->radio.channel) {
    case 2:  slope = DW1000_TEMP_COMP_05DB_CH2; break;
    case 5:  slope = DW1000_TEMP_COMP_05DB_CH5; break;
    // No published slope for the others: hand back the reference rather
    // than guess at one.
    default: return txpower;
    }

    /* APS023 Part 2 §5.3 step 2: the temperature difference against the
     * channel's slope gives the power difference. Rounded to nearest and
     * away from zero, so that warming and cooling by the same amount
     * undo one another rather than both losing half a step.
     */
    int32_t num = (int32_t)delta_temp * slope;
    int32_t adj = (num >= 0)
	        ? (num + DW1000_TEMP_COMP_SCALE / 2) / DW1000_TEMP_COMP_SCALE
	        : (num - DW1000_TEMP_COMP_SCALE / 2) / DW1000_TEMP_COMP_SCALE;

    if (adj == 0)
	return txpower;

    /* Step 3: the difference applied to the reference setting. UM §7.2.31
     * has the register as "four octets each of which specifies a separate
     * transmit power setting", so each is moved on its own. An octet left
     * at zero is one nothing set: dw1000_configure() writes only the two
     * the manual mode reads, and giving the other two a power here would
     * turn settings that are off into settings that are not.
     */
    uint32_t out = 0;
    for (int i = 0 ; i < 4 ; i++) {
	uint8_t octet = (txpower >> (i * 8)) & 0xFF;

	if (octet != 0) {
	    int32_t level =
		(int32_t)_dw1000_tx_power_octet_to_05db(octet) + adj;

	    // Clamped, not wrapped: the range is the part's, and a level
	    // outside it is a request for the nearest one inside.
	    if (level < 0)
		level = 0;
	    if (level > DW1000_TX_POWER_05DB_MAX)
		level = DW1000_TX_POWER_05DB_MAX;

	    octet = _dw1000_tx_power_05db_to_octet((uint8_t)level);
	}

	out |= ((uint32_t)octet) << (i * 8);
    }

    return out;
}




void dw1000_tx_set_power(dw1000_t *dw, uint32_t txpower) {
    dw->tx_power = txpower;
    _dw1000_reg_write32(dw, DW1000_REG_TX_POWER, DW1000_OFF_NONE, txpower);
}


/**
 * @internal
 * @brief Take the pulse generator out of the sequencer's hands
 *
 * @details The calibration below runs with the packet sequencer off and
 *          the analog blocks it would have driven held on by hand. Both
 *          calibration entry points save what they change and put it
 *          back, so the transceiver is as they found it; both therefore
 *          require it to have been idle to begin with.
 */
static
void _dw1000_pg_cal_enter(dw1000_t *dw, uint8_t *pmsc0, uint16_t *pmsc1,
			  uint32_t *rf_conf) {
    *pmsc0   = _dw1000_reg_read8 (dw, DW1000_REG_PMSC,
				  DW1000_OFF_PMSC_CTRL0);
    *pmsc1   = _dw1000_reg_read16(dw, DW1000_REG_PMSC,
				  DW1000_OFF_PMSC_CTRL1);
    *rf_conf = _dw1000_reg_read32(dw, DW1000_REG_RF_CONF,
				  DW1000_OFF_RF_CONF);

    // APS023 Part 2 §4.2 gives these four writes exactly: 0x01 to
    // PMSC_CTRL0, 0x0000 to PMSC_CTRL1, 0x001FA700 to RF_CONF, 0x22 to
    // PMSC_CTRL0. Crystal first, so the sequencer stops from a known
    // clock.
    _dw1000_clocks(dw, DW1000_CLOCK_SYS_XTI);

    // UM §7.2.50.2: writing 0 to PKTSEQ hands the analog RF subsystems
    // back from the PMSC. Restored from the saved word, not from the
    // 0xE7 the manual gives for enabling, so that nothing else in the
    // register is disturbed.
    _dw1000_reg_write16(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL1, 0x0000);

    // With the sequencer off, the LDOs and the pulse generator have to
    // be held on explicitly (UM §7.2.41.1).
    _dw1000_reg_write32(dw, DW1000_REG_RF_CONF, DW1000_OFF_RF_CONF,
			DW1000_MSK_RF_CONF_TXPOW |
			DW1000_MSK_RF_CONF_PGMIXBIASEN);

    // System and transmit clocks on the 125MHz PLL, which is what the
    // counter below is clocked from.
    _dw1000_clocks(dw, DW1000_CLOCK_TX_CONTINOUSFRAME);
}


/**
 * @internal
 * @brief Give it back, exactly as it was
 */
static
void _dw1000_pg_cal_leave(dw1000_t *dw, uint8_t pmsc0, uint16_t pmsc1,
			  uint32_t rf_conf) {
    _dw1000_reg_write8 (dw, DW1000_REG_PMSC,    DW1000_OFF_PMSC_CTRL0, pmsc0);
    _dw1000_reg_write16(dw, DW1000_REG_PMSC,    DW1000_OFF_PMSC_CTRL1, pmsc1);
    _dw1000_reg_write32(dw, DW1000_REG_RF_CONF, DW1000_OFF_RF_CONF,    rf_conf);
}


/**
 * @internal
 * @brief One pulse generator calibration, returning its count
 */
static
uint16_t _dw1000_pg_measure(dw1000_t *dw, uint8_t pg_delay) {
    const uint8_t ctrl = DW1000_FLG_TC_PG_CTRL_DIR_CONV |
	                 DW1000_MSK_TC_PG_CTRL_TMEAS;

    _dw1000_reg_write8(dw, DW1000_REG_TX_CAL, DW1000_OFF_TC_PGDELAY, pg_delay);
    _dw1000_reg_write8(dw, DW1000_REG_TX_CAL, DW1000_OFF_TC_PG_CTRL, ctrl);
    _dw1000_reg_write8(dw, DW1000_REG_TX_CAL, DW1000_OFF_TC_PG_CTRL,
		       ctrl | DW1000_FLG_TC_PG_CTRL_CALSTART);

    // The two writes are the app note's 0xBC then 0xBD (§4.2). CALSTART
    // clears itself when the measurement is done; §4.3 waits "~10us"
    // and deca_device.c a whole millisecond, so a hundred microseconds
    // sits an order of magnitude above the one and well under the other.
    _dw1000_delay_usec(100);

    return _dw1000_reg_read16(dw, DW1000_REG_TX_CAL, DW1000_OFF_TC_PG_STATUS)
	& DW1000_MSK_TC_PG_STATUS_DELAY;
}


uint16_t dw1000_tx_get_pg_count(dw1000_t *dw, uint8_t pg_delay) {
    uint8_t  pmsc0;
    uint16_t pmsc1;
    uint32_t rf_conf;

    _dw1000_pg_cal_enter(dw, &pmsc0, &pmsc1, &rf_conf);

    // Averaged over ten, as deca_device.c does, the count being noisy
    // enough that a single reading makes a poor reference.
    uint32_t sum = 0;
    for (int i = 0 ; i < 10 ; i++)
	sum += _dw1000_pg_measure(dw, pg_delay);

    _dw1000_pg_cal_leave(dw, pmsc0, pmsc1, rf_conf);

    return (uint16_t)(sum / 10);
}


uint8_t dw1000_tx_calibrate_pg_delay(dw1000_t *dw, uint16_t target_count) {
    uint8_t  pmsc0;
    uint16_t pmsc1;
    uint32_t rf_conf;

    _dw1000_pg_cal_enter(dw, &pmsc0, &pmsc1, &rf_conf);

    // APS023 Part 2 §4.3: start PG_DELAY at 0x80 and walk down the bits,
    // keeping a bit set where the count read exceeds the reference and
    // clearing it where it falls short, while recording the delay whose
    // count came closest. Closest rather than last because the search can
    // step past the reference on its final move. The starting window of
    // 300 is deca_device.c's, not the app note's, and doubles as a
    // refusal: a part that never comes within it leaves best at zero.
    uint8_t  best      = 0;
    uint8_t  current   = 0x80;
    uint8_t  bit       = 0x80;
    int32_t  closest   = 300;

    for (int i = 0 ; i < 7 ; i++) {
	bit    >>= 1;
	current |= bit;

	uint16_t count = _dw1000_pg_measure(dw, current);

	int32_t delta = (int32_t)count - (int32_t)target_count;
	if (delta < 0)
	    delta = -delta;
	if (delta < closest) {
	    closest = delta;
	    best    = current;
	}

	// A count above the target means the bandwidth was low, which a
	// longer pulse generator delay raises.
	if (count > target_count) current |=  bit;
	else                      current &= ~bit;
    }

    _dw1000_pg_cal_leave(dw, pmsc0, pmsc1, rf_conf);

    return best;
}
#endif


/** @} */
