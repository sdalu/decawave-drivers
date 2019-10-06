/*
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
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

// UM §4.7.1 : Estimating the signal power in the first path
// UM §4.7.2 : Estimating the receive signal power
// UM §7.2.18: RX Frame Information Register


/**
 * @file    dw1000.c
 * @brief   DW1000 low level driver source.
 *
 * @addtogroup DW1000
 * @{
 */


/*
 * For double buffering enabled, this code need to be check
 * for correctness
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

#include "dw1000/osal.h"
#include "dw1000/dw1000.h"


/*===========================================================================*/
/* Driver local definitions.                                                 */
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


/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/



/*===========================================================================*/
/* Driver local variables and types.                                         */
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
    { { 0, 0 }, { 102,  868 }, { 102,  868 } }, // 4
    { { 0, 0 }, { 108,  794 }, { 104,  501 } }, // 5
    { { 0, 0 }, { 102,  534 }, { 102,  534 } }, // 7
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
// UM §7.2.43.4: Pulse Generator Delay
struct _channel_tunning {
    uint32_t fs_pll_cfg;      // Frequency synthesiser - PLL configuration
    uint8_t  fs_pll_tune;     // Frequency synthesiser – PLL Tuning
    uint32_t rf_txctrl;       // RF configuration for TX
    uint8_t  rf_rxctrlh;      // RF configuration for RX
    uint8_t  tc_pgdelay;      // Pulse Generator Delay
};
static const struct _channel_tunning channel_tunning[] = {
    { 0x09000407, 0x1E, 0x00005C40, 0xD8, 0xC9 }, // Channel 1
    { 0x08400508, 0x26, 0x00045CA0, 0xD8, 0xC2 }, // Channel 2
    { 0x08401009, 0x56, 0x00086CC0, 0xD8, 0xC5 }, // Channel 3
    { 0x08400508, 0x26, 0x00045C80, 0xBC, 0x95 }, // Channel 4
    { 0x0800041D, 0xBE, 0x001E3FE0, 0xD8, 0xC0 }, // Channel 5
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

// PAC symbol size
#if (DW1000_PAC8  != 0) || (DW1000_PAC16 != 1) ||	\
    (DW1000_PAC32 != 2) || (DW1000_PAC64 != 3)
#error "unexpected DW1000_PACx value for pac_size table"
#endif
static const uint8_t pac_symbol_size[] = {
    8, 16, 32, 64
};

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
/* Driver local functions.                                                   */
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
    // Sanity check
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
 * @brief  Set clocks for appropriate mode
 *
 * @param[in]  dw       driver context
 * @param[in]  mode     mode for which to set the clocks
 *                       - DW1000_CLOCK_SEQUENCING
 *                       - DW1000_CLOCK_SYS_XTI
 *                       - DW1000_CLOCK_SYS_PLL
 *                       - DW1000_CLOCK_TX_CONTINOUSFRAME
 */
static
void _dw1000_clocks(dw1000_t *dw, int mode) {
    /* PMSC CTRL0 is a 4-byte length field
     * Here we are only interested in the first byte (holding SYSCLKS)
     */

    // Read current value
    uint8_t pmsc_ctrl0[1];
    _dw1000_reg_read(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0, pmsc_ctrl0, 1);

    // Change value according to mode
    switch(mode) {
    case DW1000_CLOCK_SEQUENCING:
	pmsc_ctrl0[0] &= ~DW1000_MSK_PMSC_CTRL0_SYSCLKS;
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

    default:
        break;
    }

    // Force sending lower byte (ie: sys/tx/rx clocks) first
    _dw1000_reg_write(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0,
		     &pmsc_ctrl0[0], 1);
}


/**
 * @private
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

    // Reset All (HIF, TX, RX, PMSC) (put flags to 0)
    _dw1000_reg_write8 (dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0_SOFTRESET,
			0x00);
    // DW1000 takes 10µs to lock clock PLL after reset (automatic after reset)
    _dw1000_delay_usec(12); // Be large, using 12µs instead of 10µs
    // Clear reset (put flags to 1)
    _dw1000_reg_write8 (dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0_SOFTRESET,
			0xF0);
    
    // Reset internal flags
    dw->wait4resp = 0;
}


/**
 * @private
 * @brief Tune the DW1000 radio
 *
 * @note  Magic is in the air!
 *
 * @param[in]  dw        driver context
 */
static inline
void _dw1000_radio_tuning(dw1000_t *dw) {
    const dw1000_radio_t radio = dw->radio;

    /* Sanity check
     */
    DW1000_ASSERT(radio != NULL, "radio parameter not defined");


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
    const struct _tx_power *tp =
	&manual_tx_power[channel_table_mapping[radio->channel]];
    uint32_t tx_power = ((radio->prf == DW1000_PRF_64MHZ)
			 ? tp->prf_64mhz   // NOTE: PRF 4MHz is unsupported
			 : tp->prf_16mhz   //       by the DW1000
			) << 8;

#if DW1000_WITH_DWM1000_EVK_COMPATIBILITY
    // Increasing TX power by 3dbm
    // Excerpt from ?????:
    //   To achieve best results when using the DWM1000 with Decawave’s
    //   DecaRanging software, you will need to adjust the default transmit
    //   power value programmed into the DWM1000 by the software. This is
    //   because DecaRanging software is targeted at Decawave ’s EVB1000
    //   evaluation board which has a different RF path compared to the
    //   DWM1000. You should increase the transmit power by approximately 3 dB.
    uint8_t txpowsd  = (tx_power >> 16) & 0xFF;
    uint8_t txpowphr = (tx_power >>  8) & 0xFF;
    if      ((txpowsd  >> 5  ) >  0) { txpowsd -= 1 << 5;  }  // Coarse
    else if ((txpowsd  & 0x1F) < 25) { txpowsd += 6;       }  // Fine
    else                             { txpowsd  = 0x1F;    }  // (max power)
    if      ((txpowphr >> 5  ) >  0) { txpowphr -= 1 << 5; }  // Coarse
    else if ((txpowphr & 0x1F) < 25) { txpowphr += 6;      }  // Fine
    else                             { txpowphr  = 0x1F;   }  // (max power)
    tx_power = (txpowsd << 16) | (txpowphr << 8);
#endif
    
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
    // NOTE: All cases don't seem to be covered but match official deca_device.c
    uint16_t drx_tune1b = 0x0020;
    if       (radio->bitrate == DW1000_BITRATE_110KBPS)
	drx_tune1b = 0x0064;
    else if ((radio->bitrate == DW1000_BITRATE_6800KBPS) && 
	     (radio->tx_plen == DW1000_PLEN_64))
	drx_tune1b = 0x0010;

    // UM §7.2.40.5: DRX_TUNE2
    //  (Using prf_tunning)
    const uint32_t drx_tune2 = pi->drx_tune2[radio->rx_pac];

    // UM §7.2.40.7: DRX_SFDTOC
    // Timeout value of 0 is forbidden, so guess the optimal timeout
    // SFD timeout is in symbol unit.
    // It seems to be possible to compute optimal timeout value
    //    1
    //  + Preamble length
    //  + SFD length (SFD = Start of Frame Delimiter)
    //  - PAC size   (PAC = Preamble Acquisition Chunk)
    //
    // UM §4.1.13: SFD detection
    //  In the standard, the SFD is 64 symbols long for 110Kb/s,
    //  and 8 symbols for other bitrate (8500Kb/s, 6.8Mb/s)
    const uint16_t drx_sfdtoc =
#if DW1000_WITH_SFD_TIMEOUT
	radio->sfd_timeout ? radio->sfd_timeout
	                   :
#endif
#if DW1000_WITH_SFD_TIMEOUT_DEFAULT
        DW1000_SFD_TIMEOUT_DEFAULT
#else
	(1 + plen_symbol_size[radio->tx_plen]
	   + (
#if DW1000_WITH_PROPRIETARY_SFD
	      radio->proprietary.sfd
	      ? bitrate_tunning[radio->bitrate].proprietary_sfd.usr_sfd_len
	      :
#endif
	        ((radio->bitrate == DW1000_BITRATE_110KBPS) ? 64 : 8))
  	   - pac_symbol_size[radio->rx_pac])
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
    if (radio->proprietary.sfd) {
	switch(usr_sfd_len) {
	case  8: dw->rxpacc_adj = -10; break;
	case 16: dw->rxpacc_adj = -18; break;
	case 64: dw->rxpacc_adj = -82; break;
	default: DW1000_ASSERT(0, "invalid register number");
	}
    } else {
#endif
	// UM §4.1.3: SFD detection
	//  SFD is 64 symbols for 110k bitrate, otherwise it is 8 symbols
	dw->rxpacc_adj = (radio->bitrate == DW1000_BITRATE_110KBPS) ? -64 : -5;
#if DW1000_WITH_PROPRIETARY_SFD
    }
#endif

    
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
		       tx_power);

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
    // => Request "TX start" and "TRX off" at the same time
    _dw1000_reg_write8 (dw, DW1000_REG_SYS_CTRL, DW1000_OFF_SYS_CTRL,
		       DW1000_FLG_SYS_CTRL_TXSTRT | DW1000_FLG_SYS_CTRL_TRXOFF);
}


/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/



/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @internal
 * @brief Write date to the DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read [0x00..0x3F]
 * @param[in]  offset   write data at the offset [0x000..0x7FFF]
 * @param[in]  data     data to be written
 * @param[in]  length   length of data to write
 *
 * @notapi
 */
void _dw1000_reg_write(dw1000_t *dw,
    uint8_t reg, size_t offset, void* data, size_t length) {
    // Sanity check
    DW1000_ASSERT(reg    <= 0x3F,               "invalid register number");
    DW1000_ASSERT(offset <= 0x7FFFu,            "out of range offset");
    DW1000_ASSERT(length <= (0x8000u - offset), "out of range length");

    // Build SPI header
    uint8_t hdr[DW1000_SPI_HEADER_MAX_LENGTH];
    size_t hdrlen;
    _dw1000_spi_header(reg, offset, true, hdr, &hdrlen);

    // Perform write
    _dw1000_spi_send(dw->config->spi,  // SPI handler
		     hdr,  hdrlen,     // Send register request 
		     data, length);    // Write data
}


/**
 * @internal
 * @brief Read data from the DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read [0x00..0x3F]
 * @param[in]  offset   read data from the offset [0x000..0x7FFF]
 * @param[out] data     will hold read data
 * @param[in]  length   length of data to read
 */
void _dw1000_reg_read(dw1000_t *dw,
    uint8_t reg, size_t offset, void* data, size_t length) {
    // Sanity check
    DW1000_ASSERT(reg    <= 0x3F,               "invalid register number");
    DW1000_ASSERT(offset <= 0x7FFFu,            "out of range offset");
    DW1000_ASSERT(length <= (0x8000u - offset), "out of range length");

    // Build SPI header
    uint8_t hdr[DW1000_SPI_HEADER_MAX_LENGTH];
    size_t hdrlen;
    _dw1000_spi_header(reg, offset, false, hdr, &hdrlen);

    // Perform read
    _dw1000_spi_recv(dw->config->spi,  // SPI handler
		     hdr,  hdrlen,     // Send register request 
		     data, length);    // Read data
}


/**
 * @brief Read 32bit words from OTP memory
 *
 * @pre   The system clock need to be set to XTI
 *
 * @note  Assuming we have exclusive use of the OTP_CTRL.
 *
 * @param[in]  dw       driver context
 * @param[in]  address  address to read (11-bit) [0x0000..0x07FF]
 * @param[out] data     array of 32bit word   
 * @param[in]  length   length of data to read
 */
void dw1000_otp_read(dw1000_t *dw,
		     uint16_t address, uint32_t *data, size_t length) {
    // Sanity check
    DW1000_ASSERT(address <= 0x07FFu, "address is 11-bit encoded");
    DW1000_ASSERT(length  <= (0x0800u - address), "out of range");

    // Assuming we have exclusive use of the OTP_CTRL,
    // so we don't care about previously assigned value

    uint16_t otp_ctrl = 0x0003; // OTPREAD | OTPRDEN
    for ( ; length-- > 0 ; address++, data++) {
	// Write the address to read
	_dw1000_reg_write16(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_ADDR,
			   address);
	// Perform reading by asserting OTP Read (self clearing)
	// and having OTP Read Enable set
	_dw1000_reg_write16(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_CTRL,
			   otp_ctrl);
	// Read Value
	*data = _dw1000_reg_read32(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_RDAT);
    }

    // Clear OTPRDEN
    otp_ctrl = 0x0000;
    _dw1000_reg_write16(dw, DW1000_REG_OTP_IF, DW1000_OFF_OTP_CTRL, otp_ctrl);
}


/**
 * @brief Perform hard reset (if supported) of the DW1000
 *
 * @note Hard reset need to be supported by the hardware and 
 *       configured in the software
 *
 * @note After the hardreset a new initialisation of the DW1000 
 *       need to be performed by calling @p dw1000_initialise
 *
 * @details Perform a hard reset of the DW1000, 
 *          if not supported this is a no-op
 *
 * @param[in]  dw       driver context
 */
void dw1000_hardreset(dw1000_t *dw) {
    // Sanity check
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


/**
 * @brief Perform initialisation/reset of the DW1000
 *
 * @note  The SPI bus frequency will be momentary set to
 *        the low speed.
 *
 * @param[in]  dw       driver context
 * 
 * @retval  0           DW1000 successfully initialized
 * @retval -1           Chip not identified as DW1000
 */
int dw1000_initialise(dw1000_t *dw) {
    // We won't bother to read default register value. We assume
    // values are at their defaults due to reset performed inside

    const dw1000_config_t *cfg = dw->config;

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
    //  (Default value for the while register is 0)
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
	// Start the LDE load
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
    dw1000_interrupt(dw, sys_mask, true);

    // Yeah!
    return 0;
}


/**
 * @brief Blink a set of LEDs.
 *
 * @note  LEDs need to have been configured @p dw1000_config_t object.
 *
 * @param[in]  dw       driver context
 * @param[in]  leds     leds to blink using a led mask
 */
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


/**
 * @internal
 * @brief Set bits for settings register
 *
 * @param[in]  dw       driver context
 */
void _dw1000_reg_set32(dw1000_t *dw,
		      uint8_t reg, size_t offset, uint32_t value) {
    uint32_t val = _dw1000_reg_read32(dw, reg, offset);
    _dw1000_reg_write32(dw, reg, offset, val | value);
}


/**
 * @internal
 * @brief Clear bits for clearing register
 *
 * @param[in]  dw       driver context
 */
void _dw1000_reg_clear32(dw1000_t *dw,
			uint8_t reg, size_t offset, uint32_t value) {
    uint32_t val = _dw1000_reg_read32(dw, reg, offset);
    _dw1000_reg_write32(dw, reg, offset, val & ~value);
}


/**
 * @brief Initialize the DW1000 driver
 *
 * @param dw        driver context
 * @param cfg       driver configuration
 */
void dw1000_init(dw1000_t *dw, const dw1000_config_t *cfg) {
    memset(dw, 0, sizeof(*dw));
    dw->config = cfg;
}


/**
 * @brief Configure the DW1000 driver
 *
 * @param dw        driver context
 */
void dw1000_configure(dw1000_t *dw, dw1000_radio_t radio) {
    /* Guard against out of range value
     */
#if DW1000_WITH_PROPRIETARY_SFD
    DW1000_ASSERT((radio->proprietary.sfd == 0) ||
		  (radio->proprietary.sfd == 1),
		  "invalid sfd flag");
#endif
    
    DW1000_ASSERT((radio->bitrate == DW1000_BITRATE_110KBPS ) ||
		  (radio->bitrate == DW1000_BITRATE_850KBPS ) ||
		  (radio->bitrate == DW1000_BITRATE_6800KBPS),
		  "invalid bit rate");

    DW1000_ASSERT((radio->channel >= 1) &&
		  (radio->channel <= 7) &&
		  (radio->channel != 6),
		  "invalid channel");

    DW1000_ASSERT((radio->prf == DW1000_PRF_4MHZ ) ||
		  (radio->prf == DW1000_PRF_16MHZ) ||
		  (radio->prf == DW1000_PRF_64MHZ),
		  "invalid PRF value");

    DW1000_ASSERT((radio->tx_pcode >= 1) && (radio->tx_pcode <= 24),
		  "invalid TX pcode value");

    DW1000_ASSERT((radio->rx_pcode >= 1) && (radio->rx_pcode <= 24),
		  "invalid RX pcode value");
	
    DW1000_ASSERT((radio->tx_plen == DW1000_PLEN_64  ) ||
		  (radio->tx_plen == DW1000_PLEN_1024) ||
		  (radio->tx_plen == DW1000_PLEN_4096) ||
#if DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH
		  (radio->tx_plen == DW1000_PLEN_128 ) ||
		  (radio->tx_plen == DW1000_PLEN_256 ) ||
		  (radio->tx_plen == DW1000_PLEN_512 ) ||
		  (radio->tx_plen == DW1000_PLEN_1536) ||
		  (radio->tx_plen == DW1000_PLEN_2048) ||
#endif
		  0,
		  "invalid preambule length");

    DW1000_ASSERT((radio->rx_pac == DW1000_PAC8 ) ||
		  (radio->rx_pac == DW1000_PAC16) ||
		  (radio->rx_pac == DW1000_PAC32) ||
		  (radio->rx_pac == DW1000_PAC64),
		  "invalid PAC value");
    
    /* Guard against value unsupported by DW1000
     */
    DW1000_ASSERT(radio->prf != DW1000_PRF_4MHZ,
		  "PRF at 4MHz is unsupported by DW1000 receiver");

    DW1000_ASSERT(((radio->prf == DW1000_PRF_64MHZ) &&
		   (radio->tx_pcode >= 9) && (radio->tx_pcode <= 24)) ||
		  ((radio->prf == DW1000_PRF_16MHZ) &&
		   (radio->tx_pcode >= 1) && (radio->tx_pcode <=  8)),
		  "incoherency between preamble code and prf");
    

    
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
    
    // UM §4.1.13: SFD detection
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
	(radio->channel         << DW1000_SFT_CHAN_CTRL_TX_CHAN ) | 
	(radio->channel         << DW1000_SFT_CHAN_CTRL_RX_CHAN ) | 
	// PRF
	(radio->prf             << DW1000_SFT_CHAN_CTRL_RXPRF   ) | 
#if DW1000_WITH_PROPRIETARY_SFD
	// SFD
	(radio->proprietary.sfd << DW1000_SFT_CHAN_CTRL_DWSFD   ) |
#endif
	// Preamble code (TX/RX)
	(radio->tx_pcode        << DW1000_SFT_CHAN_CTRL_TX_PCODE) | 
	(radio->rx_pcode        << DW1000_SFT_CHAN_CTRL_RX_PCODE) ; 
    

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
    dw->radio        = radio;    
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
}


/**
 * @brief Read temperature and battery voltage
 *
 * @param dw         driver context
 * @param[out] temp  temperature (in 1/100 °C)
 * @param[out] vbat  battery voltage in mV
 */
void dw1000_read_temp_vbat(dw1000_t *dw, uint16_t *temp, uint16_t *vbat) {
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


/**
 * @brief Set time for delayed send or received time
 *
 * @note  The device time unit is 1 / (499.2 * 128) second
 * @note  The device assignable time unit is 512
 *
 * @param dw        driver context
 * @param time      time for delayed send or received time
 */
inline
void dw1000_txrx_set_time(dw1000_t *dw, uint64_t time) {
    time = dw1000_cpu_to_le64(time);
    _dw1000_reg_write(dw, DW1000_REG_DX_TIME, DW1000_OFF_NONE, &time, 5);
}


/**
 * @brief Set context for sending frame
 *
 * @details The length, is the total length of the frame (including
 *          the 2-byte CRC)
 *
 * @note In standard mode length can be up to 127 bytes, 
 *       in proprietary long-frame-mode length can be up to 1023 bytes
 *
 * @param dw        driver context
 * @param length    frame length
 * @param offset    frame offset in DW TX buffer
 * @param tx_mode   use DW1000_TX_RANGING flag, to indicate a ranging frame
 */
void dw1000_tx_fctrl(dw1000_t *dw, size_t length, size_t offset,
		     uint8_t tx_mode) {
    DW1000_ASSERT(
#if DW1000_WITH_PROPRIETARY_LONG_FRAME
		  (dw->radio->proprietary.long_frames && (length <= 1023)) ||
#endif
		  (length <= 127), "bad frame length");
    
    uint32_t tx_fctrl = dw->reg.tx_fctrl;
    if (tx_mode & DW1000_TX_RANGING)
	tx_fctrl |= DW1000_FLG_TX_FCTRL_TR;

    tx_fctrl |=
	(length << DW1000_SFT_TX_FCTRL_TFLEN)   |
	(offset << DW1000_SFT_TX_FCTRL_TXBOFFS) ;

    _dw1000_reg_write32(dw, DW1000_REG_TX_FCTRL, 0, tx_fctrl);
}


/**
 * @brief Write data to the DW TX buffer
 *
 * @note  DW TX buffer is 1024 bytes (UM §7.2.11). 
 * @note  Data outside buffer will be silently discarded
 *
 * @param dw        driver context
 * @param data      data to write
 * @param length    length of the data being written to buffer
 * @param offset    offset to write data to
 */
void dw1000_tx_write_frame_data(dw1000_t *dw,
			  uint8_t *data, size_t length, size_t offset) {
    // Protect device from buffer overflow
    if (offset > 1024)
	return;
    if ((offset + length) > 1024)
	length = 1024 - offset;

    // Write data
    _dw1000_reg_write(dw, DW1000_REG_TX_BUFFER, offset, data, length);
}


/**
 * @brief Start transmitting a frame
 *
 * @note   Data and frame context should have already been set by 
 *         @p dw1000_tx_data and @p dw1000_tx_fctrl
 *
 * @note   If using @p DW1000_TX_DELAYED_START, the transmission time
 *         should have been previously set using @p dw1000_trx_time
 *
 * @param dw         driver context
 * @param tx_mode    a set of the following flags are supported:
 *                   DW1000_TX_DELAYED_START, DW1000_TX_RESPONSE_EXPECTED,
 *                   DW1000_TX_NO_AUTO_CRC
 *
 * @retval  0        Transmission started
 * @retval -1        It was not possible to start transmission.
 *                   (Can happen when @p DW1000_TX_DELAYED_START is set)
 */
int dw1000_tx_start(dw1000_t *dw, uint8_t tx_mode) {
    uint8_t sys_ctrl  = DW1000_FLG_SYS_CTRL_TXSTRT;

    // Set wait for response flag
    if (tx_mode & DW1000_TX_RESPONSE_EXPECTED) {
	sys_ctrl |= DW1000_FLG_SYS_CTRL_WAIT4RESP;
        dw->wait4resp = 1;
    }

    // Set delayed start flag
    if (tx_mode & DW1000_TX_DELAYED_START)
        sys_ctrl |= DW1000_FLG_SYS_CTRL_TXDLYS;

    // Set suppression of auto-FCS transmission
    if (tx_mode & DW1000_TX_NO_AUTO_CRC) {
	sys_ctrl |= DW1000_FLG_SYS_CTRL_SFCST;
    }

    // Write to SYS_CTRL register, which will trigger transmit
    _dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_SYS_CTRL, sys_ctrl);

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
	uint16_t tx_ok = 0 ;
        tx_ok = _dw1000_reg_read16(dw, DW1000_REG_SYS_STATUS, off);
        if ((tx_ok & msk) == 0)
            return 0;

	// From official deca_device.c:
	// Transmit Delayed Send set over Half a Period away or Power Up error
	// (there is enough time to send but not to power up individual blocks)
	// ==> Cancel delayed send

	// As we are turning off the transceiver (TRXOFF), we can blow
	// as well other flags
	_dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_SYS_CTRL,
			   DW1000_FLG_SYS_CTRL_TRXOFF);
	dw->wait4resp = 0;

	return -1;
    }

    return 0;
}


/**
 * @brief Start sending a frame
 *
 * @note   According to the @p DW1000_TX_NO_AUTO_CRC flag, if unset
 *         transmitted frame will have the CRC automatically computed
 *         and appended to the frame so transmitted frame will be length+2; if
 *         set, transmitted frame length will be of the specified length
 *         but a CRC-16-CCITT must be explicitely embedded in the frame data
 *
 * @note   If using @p DW1000_TX_DELAYED_START, the transmission time
 *         should have been previously set using @p dw1000_txrx_set_time
 *
 * @param dw        driver context
 * @param data      data to send
 * @param length    length of the data
 * @param tx_mode   a set of the following flags are supported:
 *                  DW1000_TX_DELAYED_START, DW1000_TX_RESPONSE_EXPECTED,  
 *                  DW1000_TX_RANGING, DW1000_TX_NO_AUTO_CRC
 *
 * @retval  0        Transmission started
 * @retval -1        It was not possible to start transmission.
 *                   (Can happen when @p DW1000_TX_DELAYED_START is set)
 */
int dw1000_tx_send(dw1000_t *dw,
		   uint8_t *data, size_t length, uint8_t tx_mode) {

    // Write data to DW TX buffer
    dw1000_tx_write_frame_data(dw, data, length, 0);
    // Adjust data length if CRC is automatically appended
    if (! (tx_mode & DW1000_TX_NO_AUTO_CRC))
	length += DW1000_CRC_LENGTH;
    // Set transmission control parameters
    dw1000_tx_fctrl(dw, length, 0, tx_mode);
    // Start sending
    return dw1000_tx_start(dw, tx_mode);
}


/**
 * @brief Start sending a frame
 *
 * @note   According to the @p DW1000_TX_NO_AUTO_CRC flag, if unset
 *         transmitted frame will have the CRC automatically computed
 *         and appended to the frame so transmitted frame will be length+2; if
 *         set, transmitted frame length will be of the specified length
 *         but a CRC-16-CCITT must be explicitely embedded in the frame data
 *
 * @note   If using @p DW1000_TX_DELAYED_START, the transmission time
 *         should have been previously set using @p dw1000_txrx_set_time
 *
 * @param dw        driver context
 * @param iovec     io vector
 * @param iovcnt    number of elements in vector
 * @param tx_mode   a set of the following flags are supported:
 *                  DW1000_TX_DELAYED_START, DW1000_TX_RESPONSE_EXPECTED,  
 *                  DW1000_TX_RANGING, DW1000_TX_NO_AUTO_CRC
 *
 * @retval  0        Transmission started
 * @retval -1        It was not possible to start transmission.
 *                   (Can happen when @p DW1000_TX_DELAYED_START is set)
 */
int dw1000_tx_sendv(dw1000_t *dw,
		    struct iovec *iovec, int iovcnt, uint8_t tx_mode) {
    size_t length = 0;

    // Write data to DW TX buffer and compute offset/length
    for ( ; iovcnt > 0 ; iovec++, iovcnt--) {
	dw1000_tx_write_frame_data(dw, iovec->iov_base, iovec->iov_len, length);
	length += iovec->iov_len;
    }
    // Adjust data length if CRC is automatically appended
    if (! (tx_mode & DW1000_TX_NO_AUTO_CRC))
	length += DW1000_CRC_LENGTH;
    // Set transmission control parameters
    dw1000_tx_fctrl(dw, length, 0, tx_mode);
    // Start sending
    return dw1000_tx_start(dw, tx_mode);
}


/**
 * @brief Reset the DW1000 receiver
 * 
 * @note  Used to deal with a bug in DW1000, see UM §4.1.6.
 *
 * @param[in]  dw       driver context
 */
static inline void
dw1000_rx_reset(dw1000_t *dw) {
    // Trigger reset for RX by creating a 0 pulse
    _dw1000_reg_write8(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0_SOFTRESET,
		      0xE0);
    _dw1000_reg_write8(dw, DW1000_REG_PMSC, DW1000_OFF_PMSC_CTRL0_SOFTRESET,
		      0xF0);
}


/**
 * @brief Read data from the DW RX buffer
 *
 * @note  DW RX buffer is 1024 bytes (UM §7.2.19).
 * @note  Trying to read data outside the buffer will be silently ignored
 *
 * @param dw        driver context
 * @param data      where to write data
 * @param length    length of the data being read from buffer
 * @param offset    offset to read data from
 */
inline
void dw1000_rx_read_frame_data(dw1000_t *dw,
			       uint8_t *data, size_t length, size_t offset) {
    // Protect device from overreading the buffer
    if (offset > 1024)
	return;
    if ((offset + length) > 1024)
	length = 1024 - offset;
    
    // Read data
    _dw1000_reg_read(dw, DW1000_REG_RX_BUFFER, offset, data, length);
}


/**
 * @brief Ensure RX buffers pointers are the same.
 *
 * @param dw        driver context
 */
void dw1000_rx_sync_dblbuf(dw1000_t *dw) {
    // UM §7.2.17: System Event Status Register
    //  => Status is a 5 bytes register (DW1000_REG_SYS_STATUS),
    //     we will read the 1 byte at offset 3
    //     which contains the ICRBP (31) and HSRBP (30) flags    
    uint8_t sys_stat = _dw1000_reg_read8(dw, DW1000_REG_SYS_STATUS, 3); 
    const bool ic   = sys_stat & (1 << (DW1000_SFT_SYS_STATUS_ICRBP - 24));
    const bool host = sys_stat & (1 << (DW1000_SFT_SYS_STATUS_HSRBP - 24));
    if (ic != host) {
	// UM §7.2.15: System Control Register
	//  => Only accessing last byte of SYS_CTRL (where is HRBPT flag)
	//     Trigger buffer toggle by writting 1 to HRBPT
        _dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, 3 ,
			  (1 << (DW1000_SFT_SYS_CTRL_HRBPT - 24)));
    }
}

/**
 * @brief Set interrupt mask.
 *
 * @param dw        driver context
 * @param bitmask   interrupt bitmask
 * @param enable    type of operation to perform
 */
void dw1000_interrupt(dw1000_t *dw, uint32_t bitmask, bool enable) {
    uint32_t sys_mask = _dw1000_reg_read32(dw, DW1000_REG_SYS_MASK, 0);
    
    if (enable) { sys_mask |=  bitmask; } // Set
    else        { sys_mask &= ~bitmask; } // Clear

    sys_mask &= DW1000_MSK_SYS_MASK;
    _dw1000_reg_write32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE, sys_mask);
}


/**
 * @brief Turn off transceiver
 *
 * @param dw        driver context
 */
void dw1000_rx_off(dw1000_t *dw) {   
    // Save interrupt mask
    uint32_t sys_mask =
	_dw1000_reg_read32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE);

    // Clear interrupt mask
    _dw1000_reg_write32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE, 0); 

    // Disable the radio
    _dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_NONE,
		       DW1000_FLG_SYS_CTRL_TRXOFF); 

    
    // UM §7.2.17: System Event Status Register
    // Clear events bits (done by writting 1 to seem)
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
		       (DW1000_MSK_SYS_STATUS_ALL_TX     |
			DW1000_MSK_SYS_STATUS_ALL_RX_ERR |
			DW1000_MSK_SYS_STATUS_ALL_RX_TO  |
			DW1000_MSK_SYS_STATUS_ALL_RX_GOOD));

    // Reset double buffer
    dw1000_rx_sync_dblbuf(dw);
    
    // Reset internal flags
    dw->wait4resp = 0;

    // Restore interrupt mask
    _dw1000_reg_write32(dw, DW1000_REG_SYS_MASK, DW1000_OFF_NONE, sys_mask); 
}


/**
 * @brief Start receiving
 *
 * @param dw        driver context
 * @param rx_mode   Receiving mode 
 *                   - @p DW1000_RX_IDLE_ON_DELAY_ERROR
 *                   - @p DW1000_RX_DELAYED_START
 * @retval  0        Reception started
 * @retval  1        Reception started, but delayed start was not
 *                   respected.
 * @retval -1        It was not possible to start receiving.
 *                   (Can happen when @p DW1000_RX_DELAYED_START
 *                   and @p DW1000_RX_IDLE_ON_DELAY_ERROR are set)
 */
int dw1000_rx_start(dw1000_t *dw, int8_t rx_mode) {
    // Sync double buffer unless explicitely disabled
    if (! (rx_mode & DW1000_RX_NO_DBLBUF_SYNC)) {
        dw1000_rx_sync_dblbuf(dw);
    }

    // Trigger receiving by writting to SYS_CTRL
    // UM §7.2.15: System Control Register
    // We will just access the 2 lower bytes to 
    //  enable radio, and delayed rceived if requested
    uint16_t sys_ctrl = DW1000_FLG_SYS_CTRL_RXENAB;
    if (rx_mode & DW1000_RX_DELAYED_START) {
        sys_ctrl |= DW1000_FLG_SYS_CTRL_RXDLYE ;
    }
    _dw1000_reg_write16(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_NONE, sys_ctrl);

    // Perform extra check for delayed receive
    if (rx_mode & DW1000_RX_DELAYED_START) {
	// Read 1 byte at offset 3 to get the 4th byte out of 5
	uint8_t sys_status = _dw1000_reg_read8(dw, DW1000_REG_SYS_STATUS, 3); 
	// If delay has passed start RX immediately
	// unless DW1000_RX_IDLE_ON_DELAY_ERROR is set in rx_mode
        if ((sys_status & (DW1000_FLG_SYS_STATUS_HPDWARN >> 24)) != 0)  {
	    // Return to an off (idle) state
            dw1000_rx_off(dw); 
	    // Keep it off on error if requested
            if (rx_mode & DW1000_RX_IDLE_ON_DELAY_ERROR)
		return -1;
	    // Fallback to immediate start
	    _dw1000_reg_write16(dw, DW1000_REG_SYS_CTRL, DW1000_OFF_NONE,
				DW1000_FLG_SYS_CTRL_RXENAB);
	    return 1;
        }
    }

    return 0;
}


/**
 * @brief To be used for interrupt processing
 *
 * @note  This *can't* be used in interrupt handler, due to SPI request
 */
bool dw1000_process_events(dw1000_t *dw) {
    const dw1000_config_t *cfg = dw->config;
    
    // UM §7.2.17: System Event Status Register
    // It's a 5 bytes register, the last byte contain low-status information
    // ( TXPUTE | RXPREJ | RXRSCS ) we won't read it.
    uint32_t status =
	_dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE); 
    
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

        // Read frame info
	//   and deduce length and ranging
	size_t length;
	bool   ranging;
	dw1000_rx_get_frame_info(dw, &length, &ranging);
	    
#if DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011
        // HOTFIX: From the official deca_device.c:
        //   "Because of a previous frame not being received properly,
        //    AAT bit can be set upon the proper reception of a frame not
        //    requesting for acknowledgement (ACK frame is not actually
        //    sent though). If the AAT bit is set, check ACK request bit
        //    in frame control to confirm"
	// WARN: Only for IEEE802.15.4-2011 compliant frames
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
		// No wait for response
		dw->wait4resp = 0;
	    }
        }
#endif

	// Effectively clearing status
        _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE, clear);

	// Call the corresponding callback if present
        if (cfg->cb.rx_ok) {
            cfg->cb.rx_ok(dw, status, length, ranging);
        }

	// Toggle the Host side Receive Buffer Pointer
        if (cfg->dblbuff) {
	    // UM §7.2.15: System Control Register
	    //  => Only accessing last byte of SYS_CTRL (where is HRBPT flag)
	    //     Trigger buffer toggle by writting 1 to HRBPT
	    _dw1000_reg_write8(dw, DW1000_REG_SYS_CTRL, 3 ,
			      (1 << (DW1000_SFT_SYS_CTRL_HRBPT - 24)));
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
        if((status & DW1000_FLG_SYS_STATUS_AAT) && dw->wait4resp) {
	    // Turn off receiver, returning to IDLE state
	    dw1000_rx_off(dw);
        }

        // Call the corresponding callback if present
        if (cfg->cb.tx_done) {
	    cfg->cb.tx_done(dw, status);
        }
    }

    
    // Handle frame reception/preamble detect timeout events
    if (status & DW1000_MSK_SYS_STATUS_ALL_RX_TO) {
	// Clear RX timeout events
	//   Only using 4 bytes out of 5 (See UM §7.2.17)
        _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
			   DW1000_MSK_SYS_STATUS_ALL_RX_TO); 

	// Turn off receiver (return to IDLE state)
	dw1000_rx_off(dw);

	// HOTFIX: UM §4.1.6: RX Message timestamp
	//   "Due to an issue in the re-initialisation of the receiver,
	//    it is necessary to apply a receiver reset after an
	//    error or timeout event"
        dw1000_rx_reset(dw);

        // Call the corresponding callback if present
        if (cfg->cb.rx_timeout) {
            cfg->cb.rx_timeout(dw, status);
        }
    }

    
    // Handle RX errors events
    if (status & DW1000_MSK_SYS_STATUS_ALL_RX_ERR) {
	// Clear RX error events
	//   Only using 4 bytes out of 5 (See UM §7.2.17)
        _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
			   DW1000_MSK_SYS_STATUS_ALL_RX_ERR);

	// Turn off receiver (return to IDLE state)
	dw1000_rx_off(dw);

	// HOTFIX: UM §4.1.6: RX Message timestamp
	//   "Due to an issue in the re-initialisation of the receiver,
	//    it is necessary to apply a receiver reset after an
	//    error or timeout event"
        dw1000_rx_reset(dw);

        // Call the corresponding callback if present
        if (cfg->cb.rx_error) {
            cfg->cb.rx_error(dw, status);
        }
    }

    return status & (DW1000_FLG_SYS_STATUS_RXFCG     |
		     DW1000_FLG_SYS_STATUS_TXFRS     |
		     DW1000_MSK_SYS_STATUS_ALL_RX_TO |
		     DW1000_MSK_SYS_STATUS_ALL_RX_ERR);
}


/**
 * @brief Set the reception timeout for the full frame
 *
 * @details The timeout value need to take in consideration the
 *          delay before transmission and the transmission time of
 *          the whole frame.
 *
 * @param [in]  dw      driver context
 * @param [in]  timeout timeout in "UWB microsencond" units (between 0..65535),
 *                       a value of 0 disable the timeout
 */
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


/**
 * @brief Enable/Disable frame filtering
 *
 * @param dw        driver context
 * @param[in] bitmask   enabling filtering: DW1000_FF_DISABLED
 *                      or a combination of
 *      DW1000_FF_COORDINATOR    frames with no destination address
 *      DW1000_FF_BEACON         beacon frames
 *      DW1000_FF_DATA           data frames
 *      DW1000_FF_ACK            ack frames
 *      DW1000_FF_MAC            mac control frames
 *      DW1000_FF_RESERVED       reserved frame types
 *      DW1000_FF_TYPE_4         type-4 frames
 *      DW1000_FF_TYPE_5         type-5 frames
 */
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

    // Apply configuration
    _dw1000_reg_write32(dw, DW1000_REG_SYS_CFG, DW1000_OFF_NONE, sys_cfg);
    // And save it for internal usage
    dw->reg.sys_cfg = sys_cfg;
}


/**
 * @brief Read reception information
 *
 * @details Retrieve information about signal quality 
 *          (first path, standard noise, ...)
 *
 * @param [in]  dw      driver context
 * @param [out] rxinfo  information about frame reception
 */
void dw1000_rx_get_info(dw1000_t *dw, dw1000_rxinfo_t *rxinfo) {
    // First path index (UM §7.2.23)
    rxinfo->first_path =
	_dw1000_reg_read16(dw, DW1000_REG_RX_TIME, DW1000_OFF_RX_TIME_FP_INDEX);

    // Standard deviation of noise (UM §4.3)
    rxinfo->std_noise =
	_dw1000_reg_read16(dw, DW1000_REG_RX_FQUAL, DW1000_OFF_RX_FQUAL_STD_NOISE);

    // LDE threshold (UM §7.2.47.1)
    rxinfo->max_noise =
	_dw1000_reg_read16(dw, DW1000_REG_LDE_IF, DW1000_OFF_LDE_THRESH);
}


/**
 * @brief Set the delay to automatically active reception after a transmission.
 *
 * @param [in]  dw      driver context
 * @param [in]  delay   delay in "UWB microsecond" units
 *                       (between 0 .. @p DW1000_MAX_TX_RX_ACTIVATION_DELAY)
 */
void dw1000_tx_set_rx_activation_delay(dw1000_t *dw, uint32_t delay) {
    DW1000_ASSERT(delay <= DW1000_MAX_TX_RX_ACTIVATION_DELAY,
		  "out of range delay");

    uint32_t val =
	_dw1000_reg_read32(dw, DW1000_REG_ACK_RESP_T, DW1000_OFF_NONE);

    val &= ~DW1000_MSK_ACK_RESP_T_W4R_TIM;
    val |= delay & DW1000_MSK_ACK_RESP_T_W4R_TIM;

    _dw1000_reg_write32(dw, DW1000_REG_ACK_RESP_T, DW1000_OFF_NONE, val);
}


/**
 * @brief Get (an estimation of) transmitter clock drift
 *
 * @details The transmitter clock drift is calculated with
 *          <code>drift = offset/interval</code>.
 *          If positive the transmitter clock is running faster, 
 *          if negative the transmitter clock is running slower.
 *
 * @note    Interval value is dependant of the radio configuration (PRF value),
 *          so it is not necessary to retrieve it everytime.
 *
 * @param[in]  dw       driver context
 * @param[out] offset   clock offset calculated during the interval
 * @param[out] interval time interval used to calculate the offset
 */
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


/**
 * @brief Get the preamble accumulation count
 *
 * @param[in]  dw       driver context
 *
 * @return the preamble acculumation count
 */
uint16_t dw1000_rx_get_pacc_count(dw1000_t *dw) {
    // Get Preamble accumulation count... and adjust it
    // UM §7.2.18: RX Frame Information Register (RXPACC field)
    uint16_t rxpacc       =
	(_dw1000_reg_read32(dw, DW1000_REG_RX_FINFO, DW1000_OFF_NONE) &
	 DW1000_MSK_RX_FINFO_RXPACC) >> DW1000_SFT_RX_FINFO_RXPACC;
    uint16_t rxpacc_nosat =
	_dw1000_reg_read16(dw, DW1000_REG_DRX_CONF,DW1000_OFF_DRX_RXPACC_NOSAT);
    if (rxpacc == rxpacc_nosat)
	rxpacc += dw->rxpacc_adj;

    return rxpacc;
}




/**
 * @brief Correct received power reading (estimated vs actual)
 *
 * @note  See UM §4.7, fig 22: Estimated RX level versus actual RX level
 *
 * @param[in]  dw       driver context
 * @param[in]  p        estimated power
 *
 * @return "actual" power
 */
static
double _dw1000_rx_power_correction(dw1000_t *dw, double p) {
    // UM §4.7: [Figure 22]: Estimated RX level versus actual RX level
    switch (dw->radio->prf) {
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

    // Sanity check
    // XXX: is it better or worst to trim it?
    if (p > -60)
	p = -60;

    // Return corrected power
    return p;
}


/**
 * @brief Compute the received signal and/or firstpath power in dBm
 *
 * @param[in]  dw         driver context
 * @param[out] signal     received signal power in dBm
 * @param[out] firstpath  received firstpath power in dBm
 */
void dw1000_rx_get_power_estimate(dw1000_t *dw,
				  double *signal, double *firstpath) {
    // PRF of 4MHZ is unsupported by DW1000
    DW1000_ASSERT((dw->radio->prf == DW1000_PRF_16MHZ) ||
		  (dw->radio->prf == DW1000_PRF_64MHZ),
		  "unsupported PRF value");

    // UM §4.7: Assessing the quality of reception and the RX timestamp
    //  PRF     4   16        64
    //  A       -   113.77    127.74
    double N  = (double) dw1000_rx_get_pacc_count(dw);
    double A  = dw->radio->prf == DW1000_PRF_16MHZ ? 113.77 : 127.74;

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
	double p  = 10.0 * log10((F1*F1 + F2*F2 + F3*F3) / (N*N)) - A;

	*firstpath = _dw1000_rx_power_correction(dw, p);
    }

    // Signal power
    if (signal) {
	double C  = (double) _dw1000_reg_read16(dw,DW1000_REG_RX_FQUAL,
						   DW1000_OFF_RX_FQUAL_CIR_PWR);
	double p  = 10.0 * log10((C * 131072.0) / (N * N)) - A;

	*signal = _dw1000_rx_power_correction(dw, p);
    }
}


/**
 * @brief Get information to help calibration process.
 *
 * @param[in]  channel    Channel (1, 2, 3, 4, 5, or 7)
 * @param[in]  prf        PRF (@p DW1000_PRF_16MHZ or @p DW1000_PRF_64MHZ)
 * @param[out] power      Power at receiver input (dBm/MHz) 
 * @param[out] separation Antenna separation in centimeters
 */
bool
dw1000_get_calibration(uint8_t channel, uint8_t prf,
		       uint8_t *power, uint16_t *separation) {
    // Sanity check on channel
    if ((channel < 1) || (channel > 7) || (channel == 6)) {
	return false;
    }

    // Sanity check on PRF
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

    // Job's done
    return true;
}



/** @} */
