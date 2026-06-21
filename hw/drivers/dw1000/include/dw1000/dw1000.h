/*
 * Copyright (c) 2018-2024
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_H__
#define __DW1000_H__

/**
 * @file    dw1000.c
 * @brief   DW1000 low level driver source.
 *
 * @addtogroup DW1000
 * @{
 */


#include "dw1000/osal.h"
#include "dw1000/dw1000_bswap.h"
#include "dw1000/dw1000_otp.h"
#include "dw1000/dw1000_reg.h"



/*===========================================================================*/
/* Compile time options                                                      */
/*===========================================================================*/

/**
 * \defgroup Config Compile time configuration
 */
/** @{ */

/**
 * @brief Increase transmit power by about 3db, for compatibility with
 *        DecaRanging software when using DWM1000 module
 */
#if !defined(DW1000_WITH_DWM1000_EVK_COMPATIBILITY) || defined(__DOXYGEN__)
#define DW1000_WITH_DWM1000_EVK_COMPATIBILITY   0
#endif

/**
 * @brief Add support for proprieray preamble length
 */
#if !defined(DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH) || defined(__DOXYGEN__)
#define DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH 1
#endif

/**
 * @brief Add support for proprietary SFD
 */
#if !defined(DW1000_WITH_PROPRIETARY_SFD) || defined(__DOXYGEN__)
#define DW1000_WITH_PROPRIETARY_SFD 1
#endif

/**
 * @brief Add support for proprietary long frame
 */
#if !defined(DW1000_WITH_PROPRIETARY_LONG_FRAME) || defined(__DOXYGEN__)
#define DW1000_WITH_PROPRIETARY_LONG_FRAME 0
#endif

/**
 * @brief Add support for user defined SFD timeout
 */
#if !defined(DW1000_WITH_SFD_TIMEOUT) || defined(__DOXYGEN__)
#define DW1000_WITH_SFD_TIMEOUT 0
#endif

/**
 * @brief Use default SFD timeout value instead of computed one
 */
#if !defined(DW1000_WITH_SFD_TIMEOUT_DEFAULT) || defined(__DOXYGEN__)
#define DW1000_WITH_SFD_TIMEOUT_DEFAULT 0
#endif

/**
 * @brief Compile hotfix for AAT and IEEE802.15.4-2011 compliant frames
 *
 * @details Because of a previous frame not being received properly,
 *          AAT bit can be set upon the proper reception of a frame not
 *          requesting for acknowledgement (ACK frame is not actually
 *          sent though). If the AAT bit is set, check ACK request bit
 *          in frame control to confirm (this implementation works only
 *          for IEEE802.15.4-2011 compliant frames).
 */
#if !defined(DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011) || defined(__DOXYGEN__)
#define DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011 1
#endif

/**
 * @brief Value for default SFD timeout
 *
 * @details Value can be between 1 and 65535, but useful values
 *          are usually between 120 and 4161.
 */
#if !defined(DW1000_SFD_TIMEOUT_DEFAULT) || defined(__DOXYGEN__)
#define DW1000_SFD_TIMEOUT_DEFAULT DW1000_SFD_TIMEOUT_MAX
#endif

/** @} */



/*===========================================================================*/
/* Constants                                                                 */
/*===========================================================================*/

/**
 * @brief Frequency of the clock used for timestamping (Hz)
 */
#define DW1000_TIME_CLOCK_HZ (499200000ull * 128)

/**
 * @brief Number of bits for the clock used for timestamping
 */
#define DW1000_TIME_CLOCK_BITS 40

/**
 * @brief Frequency of the clock used for timestamping (MHz)
 */
#define DW1000_TIME_CLOCK_MHZ 63897.6

/**
 * @brief Minimum clock time to be used in delayed transmit/receive
 * @see DW1000_CLOCK_ROUNDUP
 */
#define DW1000_CLOCK_MIN 512

/**
 * @brief Round up the clock value to be used in delayed transmit/receive
 * @see DW1000_CLOCK_MIN
 */
#define DW1000_CLOCK_ROUNDUP(x) (((x) + 511) & (~0x1FF))

/**
 * @brief Convert µsec to DW1000 clock unit (rounded)
 */
#define DW1000_USEC_TO_CLOCK(x)					\
    ((((x) * DW1000_TIME_CLOCK_HZ) + (1000000-1)) / 1000000)

/**
 * @brief Convert DW1000 clock unit to µsec
 */
#define DW1000_CLOCK_TO_USEC(x)					\
    (x / DW1000_TIME_CLOCK_MHZ)

/**
 * @brief Convert msec to DW1000 clock unit (rounded)
 */
#define DW1000_MSEC_TO_CLOCK(x)					\
    ((((x) * DW1000_TIME_CLOCK_HZ) + (1000-1)) / 1000)

/**
 * @brief Convert DW1000 clock unit to msec
 */
#define DW1000_CLOCK_TO_MSEC(x)					\
    (DW1000_CLOCK_TO_USEC(x)/1000)

/**
 * @brief Length of CRC field
 */
#define DW1000_CRC_LENGTH 2

/**
 * @brief Frame maximum size
 */
#if DW1000_WITH_PROPRIETARY_LONG_FRAME
#define DW1000_FRAME_MAXSIZE 1023
#else
#define DW1000_FRAME_MAXSIZE 127
#endif



#define DW1000_TX_POWER_FLG_MANUAL  0x80 /**< @internal */
#define DW1000_TX_POWER_MSK_MANUAL  0x3f /**< @internal */

/**
 * @brief Transmit power
 */
#define DW1000_TX_POWER_AUTO	0

/**
 * @brief Define the transmit power up to 30.5dB in 0.5db unit.
 */
#define DW1000_TX_POWER_05DB(v)					\
    ((v) | DW1000_TX_POWER_FLG_MANUAL)
/**
 * @brief Define the transmit power up to 30.5dB in 0.5db step.
 */
#define DW1000_TX_POWER(v)					\
    DW1000_TX_POWER_05DB((uint8_t)(2 * (v)))



/*===========================================================================*/
/* Driver variables and types                                                */
/*===========================================================================*/

/**
 * @brief DW1000 RX info
 */
typedef struct dw1000_rxinfo {
    /**
     * @brief First path index
     */
    uint16_t first_path;
    /**
     * @brief Standard deviation of noise
     */
    uint16_t std_noise;
    /**
     * @brief LDE threshold
     */
    uint16_t max_noise;
} dw1000_rxinfo_t;


/**
 * @brief DW1000 Radio Configuration
 */
typedef struct dw1000_radio {
    /**
     * @brief Channel number 
     * @note  Possible channel values are: 1, 2, 3, 4, 5, 7
     */
    uint8_t    channel; 
    /**
     * @brief Pulse Repetition Frequency 
     * @note  @p DW1000_PRF_16MHZ or @p DW1000_PRF_64MHZ
     */
    uint8_t    prf; 
    /**
     * @brief Acquisition Chunk Size (Relates to RX preamble length)
     */
    uint8_t    rx_pac;
    /**
     * @brief DW1000_PLEN_64..DW1000_PLEN_4096
     */
    uint8_t    tx_plen;
    /**
     * @brief TX preamble code
     */
    uint8_t    tx_pcode;
    /**
     * @brief RX preamble code
     */
    uint8_t    rx_pcode;
    /**
     * @brief Bit rate @p DW1000_BITRATE_110KBPS, @p DW1000_BITRATE_850KBPS 
     *        or @p DW1000_BITRATE_6800KBPS
     */
    uint8_t    bitrate;
    /**
     * @brief Transmit power (max 30.5dB in 0.5db step).
     *        If not set, will default to maximal allowed regulation value
     *        according to channel and prf setting.
     */
    uint8_t    tx_power;
    
#if DW1000_WITH_SFD_TIMEOUT
    /**
     * @brief SFD timeout value (in symbols).
     * If 0 fallback to DW1000_SFD_TIMEOUT_MAX
     */
    uint16_t   sfd_timeout;
#endif
    
#if DW1000_WITH_PROPRIETARY_SFD || DW1000_WITH_PROPRIETARY_LONG_FRAME
    struct {
#if DW1000_WITH_PROPRIETARY_LONG_FRAME
	/**
	 * @brief Support long frames, up to 1023 bytes
	 */
	uint8_t long_frames:1;
#endif
#if DW1000_WITH_PROPRIETARY_SFD
	/**
	 * @brief Use non standard SFD (improved performance)
	 */
	uint8_t sfd:1;
#endif
    } proprietary;
#endif
} *dw1000_radio_t;





/**
 * @brief DW1000 driver context
 */
typedef struct dw1000 dw1000_t;

/**
 * @brief DW1000 Configuration
 */
typedef struct dw1000_config {
    /**
     * @brief Use of double buffer
     */
    uint8_t    dblbuff:1;
    /**
     * @brief Loading of Leading Edge detection code
     */
    uint8_t    lde_loading:1;
    /**
     * @brief Automatically re-eanbling of receiver (except for timeout)
     */
    uint8_t    rxauto:1;
    /**
     * @brief Define led blink time in 14ms unit
     */
    uint8_t    leds_blink_time;
    /**
     * @brief SPI driver
     * @note Config for low speed is <3MHz, for high speed < 20MHz
     */
    dw1000_spi_driver_t *spi;
    /**
     * @brief IRQ line
     */
    dw1000_ioline_t   irq;
    /**
     * @brief Reset line
     */
    dw1000_ioline_t   reset;
    /**
     * @brief WakeUp line
     */
    dw1000_ioline_t   wakeup;
    /**
     * @brief Cristal trimming (optional)
     */
    uint8_t    xtrim;
    /**
     * @brief Set of LED wired to DW1000
     */
    uint8_t    leds;
    /**
     * @brief Delay to take into account for antenna reception
     */
    uint16_t   tx_antenna_delay;
    /**
     * @brief Delay to take into account for antenna transmission
     */
    uint16_t   rx_antenna_delay;
    /**
     * @brief callbacks
     */
    struct {
	/**
	 * @brief Callback for TX done.
	 *
	 * The callback will be triggered for the following events
	 * (unless the default interuption mask is changed):
	 *     - Transmit frame sent (TXFRS)
	 *
	 * Default interruption mask: DW1000_FLG_SYS_MASK_MTXFRS
	 */
	void (*tx_done   )(dw1000_t *dw, uint32_t status);

	/**
	 * @brief Callback for RX timeout.
	 *
	 * The callback will be triggered for the following events
	 * (unless the default interuption mask is changed):
	 *     - Receive frame wait timeout (RXRFTO)
	 *     - Preamble detection timeout (RXPTO)
	 *
	 * Default interruption mask: DW1000_MSK_SYS_MASK_ALL_RX_TO
	 */
	void (*rx_timeout)(dw1000_t *dw, uint32_t status);

	/**
	 * @brief Callback for RX error.
	 *
	 * The callback will be triggered for the following events
	 * (unless the default interuption mask is changed):
	 *     - Receiver PHY header error (RXPHE)
	 *     - Receiver FCS Error (RXFCE)
	 *     - Receiver Reed Solomon frame sync (RXRFSL)
	 *     - Leading edge detection processing error (LDEERR)
	 *     - Receive SFD timeout (RXSFDTO)
	 *     - Automatic frame filtering rejection (AFFREJ)
	 *
	 * Default interruption mask: DW1000_MSK_SYS_MASK_ALL_RX_ERR
	 */
	void (*rx_error  )(dw1000_t *dw, uint32_t status);

	/**
	 * @brief Callback for RX ok
	 *
	 * The callback will be triggered for the following events
	 * (unless the default interuption mask is changed):
	 *     - Receiver FCS Good (RXFCG)
	 *
	 * Default interruption mask: DW1000_FLG_SYS_MASK_MRXFCG
	 */
	void (*rx_ok     )(dw1000_t *dw, uint32_t status,
			   size_t length, bool ranging);
    } cb;
} dw1000_config_t;



/**
 * @brief DW1000 driver context
 */
struct dw1000 {
    const dw1000_config_t *config;

    dw1000_radio_t radio;

    struct {
	uint32_t device;
	uint32_t chip;
	uint32_t lot;
    } id;
    
    uint8_t  xtrim;
    uint8_t  otp_rev;
    uint8_t  ref_vbat_33;
    uint8_t  ref_vbat_37;
    uint8_t  ref_temp_23;
    uint8_t  ref_temp_ant;
    
    /* */
    uint32_t wait4resp;
    uint32_t sleep_mode;

    int8_t   rxpacc_adj;
    
    struct {
	uint32_t sys_cfg;
	uint32_t tx_fctrl;
    } reg;

    uint32_t tx_power;
};



/**
 * @brief Maximum allowed delay for automatic activation of reception
 *        after transmission.
 */
#define DW1000_MAX_TX_RX_ACTIVATION_DELAY ((1<<20)-1)



#define DW1000_BITRATE_110KBPS  0
#define DW1000_BITRATE_850KBPS  1
#define DW1000_BITRATE_6800KBPS 2

#define DW1000_PRF_4MHZ      0  // Unsupported by DW1000 receiver
#define DW1000_PRF_16MHZ     1
#define DW1000_PRF_64MHZ     2


#define DW1000_XTRIM_MIDRANGE 0x10

// UM §7.2.10 (table 16) (careful with the table bit order: 19,18 21,20)
// Preamble Lenght (PLEN) value is encoded to map on (TXPSR | PE) of TX_FCTRL
#define DW1000_PLEN_64     0x1    //!   64 symbols preamble length
#define DW1000_PLEN_1024   0x2    //! 1024 symbols preamble length
#define DW1000_PLEN_4096   0x3    //! 4096 symbols preamble length
#if DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH
#define DW1000_PLEN_128    0x5    //!  128 symbols preamble length (proprietary)
#define DW1000_PLEN_256    0x9    //!  256 symbols preamble length (proprietary)
#define DW1000_PLEN_512    0xD    //!  512 symbols preamble length (proprietary)
#define DW1000_PLEN_1536   0x6    //! 1536 symbols preamble length (proprietary)
#define DW1000_PLEN_2048   0xA    //! 2048 symbols preamble length (proprietary)
#endif


/* Preamble Acquisition Chunk (PAC) size in symbols
 */
#define DW1000_PAC8        0   // PAC  8
#define DW1000_PAC16       1   // PAC 16
#define DW1000_PAC32       2   // PAC 32
#define DW1000_PAC64       3   // PAC 64



#define DW1000_SFD_TIMEOUT_MAX  (4096 + 64 + 1)


#define DW1000_LED_RXOK    (1 << 0) //<! Mask for RXOK led
#define DW1000_LED_SFD     (1 << 1) //<! Mask for SFD led
#define DW1000_LED_RX      (1 << 2) //<! Mask for RX led
#define DW1000_LED_TX      (1 << 3) //<! Mask for TX led

#define DW1000_LED_TXRX    (DW1000_LED_TX   | DW1000_LED_RX    )
#define DW1000_LED_STATUS  (DW1000_LED_SFD  | DW1000_LED_RXOK  )
#define DW1000_LED_ALL     (DW1000_LED_TXRX | DW1000_LED_STATUS)

#define DW1000_TX_IMMEDIATE           0x00
#define DW1000_TX_DELAYED_START       0x01
#define DW1000_TX_RESPONSE_EXPECTED   0x02
#define DW1000_TX_RANGING             0x04
#define DW1000_TX_NO_AUTO_CRC         0x08

#define DW1000_RX_IMMEDIATE           0x00
#define DW1000_RX_DELAYED_START       0x01
#define DW1000_RX_IDLE_ON_DELAY_ERROR 0x02
#define DW1000_RX_NO_DBLBUF_SYNC      0x04

// Frame filtering
#define DW1000_FF_DISABLED         0
#define DW1000_FF_COORDINATOR      DW1000_FLG_SYS_CFG_FFBC
#define DW1000_FF_BEACON           DW1000_FLG_SYS_CFG_FFAB
#define DW1000_FF_DATA             DW1000_FLG_SYS_CFG_FFAD
#define DW1000_FF_ACK              DW1000_FLG_SYS_CFG_FFAA
#define DW1000_FF_MAC              DW1000_FLG_SYS_CFG_FFAM
#define DW1000_FF_RESERVED         DW1000_FLG_SYS_CFG_FFAR
#define DW1000_FF_TYPE_4           DW1000_FLG_SYS_CFG_FFA4
#define DW1000_FF_TYPE_5           DW1000_FLG_SYS_CFG_FFA5



/*===========================================================================*/
/* Registers (read/write)                                                    */
/*===========================================================================*/

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
	uint8_t reg, size_t offset, void* data, size_t length);


/**
 * @internal
 * @brief Write date to the DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read [0x00..0x3F]
 * @param[in]  offset   write data at the offset [0x000..0x7FFF]
 * @param[in]  data     data to be written
 * @param[in]  length   length of data to write
 */
void _dw1000_reg_write(dw1000_t *dw,
	uint8_t reg, size_t offset, void* data, size_t length);


/**
 * @internal
 * @brief Write a byte to DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to write
 * @param[in]  offset   offset in the register
 * @param[in]  data     byte to write
 */
static inline void
_dw1000_reg_write8(dw1000_t *dw, uint8_t reg, size_t offset, uint8_t data)
{
    _dw1000_reg_write(dw, reg, offset, &data, sizeof(data));
}


/**
 * @internal
 * @brief Write a 16-bit word to DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to write
 * @param[in]  offset   offset in the register
 * @param[in]  data     16-bit word to write
 */
static inline void
_dw1000_reg_write16(dw1000_t *dw, uint8_t reg, size_t offset, uint16_t data)
{
    data = dw1000_cpu_to_le16(data);
    _dw1000_reg_write(dw, reg, offset, &data, sizeof(data));
}


/**
 * @internal
 * @brief Write a 32-bit word to DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to write
 * @param[in]  offset   offset in the register
 * @param[in]  data     32-bit word to write
 */
static inline void
_dw1000_reg_write32(dw1000_t *dw, uint8_t reg, size_t offset, uint32_t data)
{
    data = dw1000_cpu_to_le32(data);
    _dw1000_reg_write(dw, reg, offset, &data, sizeof(data));
}


/**
 * @internal
 * @brief Write a 64-bit word to DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to write
 * @param[in]  offset   offset in the register
 * @param[in]  data     64-bit word to write
 */
static inline void
_dw1000_reg_write64(dw1000_t *dw, uint8_t reg, size_t offset, uint64_t data)
{
    data = dw1000_cpu_to_le64(data);
    _dw1000_reg_write(dw, reg, offset, &data, sizeof(data));
}


/**
 * @internal
 * @brief Read a byte from DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read
 * @param[in]  offset   offset in the register
 *
 * @return byte
 */
static inline uint8_t
_dw1000_reg_read8(dw1000_t *dw, uint8_t reg, size_t offset)
{
    uint8_t data;
    _dw1000_reg_read(dw, reg, offset, &data, sizeof(data));
    return data;
}


/**
 * @internal
 * @brief Read a 16-bit word from DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read
 * @param[in]  offset   offset in the register
 *
 * @return 16-bit word
 */
static inline uint16_t
_dw1000_reg_read16(dw1000_t *dw, uint8_t reg, size_t offset)
{
    uint16_t data;
    _dw1000_reg_read(dw, reg, offset, &data, sizeof(data));
    return dw1000_le16_to_cpu(data);
}


/**
 * @internal
 * @brief Read a 32-bit word from DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read
 * @param[in]  offset   offset in the register
 *
 * @return 32-bit word
 */
static inline uint32_t
_dw1000_reg_read32(dw1000_t *dw, uint8_t reg, size_t offset)
{
    uint32_t data;
    _dw1000_reg_read(dw, reg, offset, &data, sizeof(data));
    return dw1000_le32_to_cpu(data);
}


/**
 * @internal
 * @brief Read a 64-bit word from DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read
 * @param[in]  offset   offset in the register
 *
 * @return 64-bit word
 */
static inline uint64_t
_dw1000_reg_read64(dw1000_t *dw, uint8_t reg, size_t offset)
{
    uint64_t data;
    _dw1000_reg_read(dw, reg, offset, &data, sizeof(data));
    return dw1000_le64_to_cpu(data);
}



/*===========================================================================*/
/* OTP (read)                                                                */
/*===========================================================================*/

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
		     uint16_t address, uint32_t *data, size_t length);


/**
 * @brief Get 32bit words from OTP memory
 *
 * @pre   The system clock need to be set to XTI
 *
 * @note  Assuming we have exclusive use of the OTP_CTRL,
 *
 * @param[in]  dw       driver context
 * @param[in]  address  address to read (11-bit)
 *
 * @return 32bit word from OTP memory
 */
static inline uint32_t
dw1000_otp_get(dw1000_t *dw, uint16_t address)
{
    uint32_t data;
    // dw1000_otp_read() already returns host-order words, so no further
    // endian conversion must be applied here.
    dw1000_otp_read(dw, address, &data, 1);
    return data;
}



/*===========================================================================*/
/* Setup helpers                                                             */
/*===========================================================================*/

/**
 * @brief Get information to help calibration process.
 *
 * @param[in]  channel    Channel (1, 2, 3, 4, 5, or 7)
 * @param[in]  prf        PRF (@p DW1000_PRF_16MHZ or @p DW1000_PRF_64MHZ)
 * @param[out] power      Power at receiver input (dBm/MHz) 
 * @param[out] separation Antenna separation in centimeters
 */
bool dw1000_get_calibration(uint8_t channel, uint8_t prf,
			    uint8_t *power, uint16_t *separation);



/*===========================================================================*/
/* Initialisation                                                            */
/*===========================================================================*/

/**
 * @brief Initialize the DW1000 driver
 *
 * @param dw        driver context
 * @param cfg       driver configuration
 */
void dw1000_init(dw1000_t *dw, const dw1000_config_t *cfg);


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
void dw1000_hardreset(dw1000_t *dw);


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
int dw1000_initialise(dw1000_t *dw);


/**
 * @brief Configure the DW1000 driver
 *
 * @param dw        driver context
 */
void dw1000_configure(dw1000_t *dw, dw1000_radio_t radio);



/*===========================================================================*/
/* System                                                                    */
/*===========================================================================*/

/**
 * @brief Get system time
 *
 * @param[in]  dw        driver context
 *
 * @return system time (40-bit clock)
 */
static inline uint64_t
dw1000_get_system_time(dw1000_t *dw)
{
    uint64_t sys_time = 0;
    _dw1000_reg_read(dw, DW1000_REG_SYS_TIME, DW1000_OFF_NONE,
		    ((uint8_t *)(&sys_time)) + 0, 5);

    return dw1000_le64_to_cpu(sys_time);
}


/**
 * @brief Read temperature and battery voltage
 *
 * @param dw         driver context
 * @param[out] temp  temperature (in 1/100 °C)
 * @param[out] vbat  battery voltage in mV
 */
void dw1000_read_temp_vbat(dw1000_t *dw, uint16_t *temp, uint16_t *vbat);


/**
 * @brief Blink a set of LEDs.
 *
 * @note  LEDs need to have been configured @p dw1000_config_t object.
 *
 * @param[in]  dw       driver context
 * @param[in]  leds     leds to blink using a led mask
 */
void dw1000_leds_blink(dw1000_t *dw, uint8_t leds);


/**
 * @brief Set the used EUI64 address.
 *
 * @note  This will be used by the Receive Frame Filtering function.
 *
 * @param[in]  dw       driver context
 * @param[in]  eui64    EUI64 address
 */
static inline
void dw1000_set_eui(dw1000_t *dw, uint64_t eui64)
{
    _dw1000_reg_write64(dw, DW1000_REG_EUI, DW1000_OFF_NONE, eui64);
}


/**
 * @brief Get the used EUI64 address.
 *
 * @note  During DW1000 initialisation or upon waking up from sleep mode,
 *        the value is loaded from OTP memory area, the value will need to
 *        be overwritten later using dw1000_set_eui() if necessary.
 *
 * @param[in]  dw       driver context
 *
 * @return EUI64 address
 */
static inline
uint64_t dw1000_get_eui(dw1000_t *dw)
{
    return _dw1000_reg_read64(dw, DW1000_REG_EUI, DW1000_OFF_NONE);
}




/*===========================================================================*/
/* Interruption handling                                                     */
/*===========================================================================*/

/**
 * @brief Check if an interrupt is pending
 *
 * @param[in]  dw       driver context
 *
 * @return true if an interrupt is pending, false otherwise.
 */
static inline bool
dw1000_pending_interrupt(dw1000_t *dw)
{
    // UM §7.2.17: System Event Status Register
    //  => Status is a 5 bytes register (DW1000_REG_SYS_STATUS),
    //     we will use only the first 4 bytes to access
    return _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE) &
	     DW1000_FLG_SYS_STATUS_IRQS;
}


/**
 * @brief Set interrupt mask.
 *
 * @param dw        driver context
 * @param bitmask   interrupt bitmask
 *                  (from: @p DW1000_FLG_SYS_MASK_*, @p DW1000_MSK_SYS_MASK_*)
 * @param enable    type of operation to perform
 */
void dw1000_interrupt(dw1000_t *dw, uint32_t bitmask, bool enable);


/**
 * @brief To be used for interrupt processing
 *
 * Will perform basic low-level processing of the events and will
 * call the registered event handler (rx_ok, tx_done, rx_timeout, rx_error).
 *
 * In the case of rx_timeout and rx_error, the dw1000 is forcefully returned
 * to the IDLE state. For rx_ok and tx_done, it is left in the programmed
 * next step.
 *
 * @note  This *can't* be used in interrupt handler, due to SPI request
 *
 * @param dw        driver context
 *
 * @retval true     at least one event has been processed
 * @retval false    there was no event to process
 */
bool dw1000_process_events(dw1000_t *dw);



/*===========================================================================*/
/* Operations common to TX/RX                                                */
/*===========================================================================*/

/**
 * @brief Set time for delayed send or received time
 *
 * @note  The device time unit is 1 / (499.2e6 * 128) second
 * @note  The device assignable time unit is 512 (about 8ns),
 *        which means that the 9 lower bytes of the given time are ignored.
 *
 * @param dw        driver context
 * @param time      time for delayed send or received time
 */
void dw1000_txrx_set_time(dw1000_t *dw, uint64_t time);


/**
 * @brief Turn off transceiver
 *
 * @param dw        driver context
 */
void dw1000_txrx_off(dw1000_t *dw);



/*===========================================================================*/
/* Transmission (TX)                                                         */
/*===========================================================================*/

/**
 * @brief Set the delay to automatically active reception after a transmission.
 *
 * @param [in]  dw      driver context
 * @param [in]  delay   delay in "UWB microsecond" units
 *                       (between 0 .. @p DW1000_MAX_TX_RX_ACTIVATION_DELAY)
 */
void dw1000_tx_set_rx_activation_delay(dw1000_t *dw, uint32_t delay);

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
void dw1000_tx_fctrl(dw1000_t *dw, size_t length, size_t offset, int tx_mode);

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
				uint8_t *data, size_t length, size_t offset);


/**
 * @brief Start transmitting a frame
 *
 * @note   Data and frame context should have already been set by 
 *         @p dw1000_tx_data and @p dw1000_tx_fctrl
 *
 * @note   If using @p DW1000_TX_DELAYED_START, the transmission time
 *         should have been previously set using @p dw1000_txrx_set_time
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
int dw1000_tx_start(dw1000_t *dw, int tx_mode);


/**
 * @brief Check if currently expecting response.
 *
 * Following a transmitted frame with the @p DW1000_TX_RESPONSE_EXPECTED
 * flag set, the receiver will automatically turn on, so it won't be
 * necessary to explicitely turn it on.
 *
 * @param[in]  dw       driver context
 *
 * @retval true		Frame was sent with @p DW1000_TX_RESPONSE_EXPECTED
 *                      and no response received so far.
 * @retval false	Otherwise
 */
static inline bool
dw1000_tx_is_expecting_response(dw1000_t *dw) {
    return dw->wait4resp;
}


/**
 * @brief Check the status of TX done (ie: TXFRS flag)
 *
 * @details Everything that specified that the transmission is ended.
 *
 * @param[in]  dw       driver context
 */
static inline uint32_t dw1000_tx_is_status_done(dw1000_t *dw)
{
    // UM §7.2.17: System Event Status Register
    //  => Status is a 5 bytes register (DW1000_REG_SYS_STATUS),
    //     we will use only the first 4 bytes to access
    //     DW1000_FLG_SYS_STATUS_TXFRS (7)
    return _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE) &
	DW1000_FLG_SYS_STATUS_TXFRS;
}


/**
 * @brief Clear/Acknowledge the TX done event (ie: TXFRS flag)
 *
 * @param dw        driver context
 */
static inline void
dw1000_tx_clear_status_done(dw1000_t *dw)
{
    // Trigger clearing of TX frame sent event by setting it 1
    // UM §7.2.17: System Event Status Register
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
		       DW1000_FLG_SYS_STATUS_TXFRS);
}


/**
 * @brief Get frame transmission time
 *
 * @param[in]  dw        driver context
 *
 * @return RMARKER transmission time (40-bit clock)
 */
static inline uint64_t dw1000_tx_get_rmarker_time(dw1000_t *dw)
{
    uint64_t tx_time = 0;
    _dw1000_reg_read(dw, DW1000_REG_TX_TIME, DW1000_OFF_TX_TIME_TX_STAMP,
		    ((uint8_t *)(&tx_time)), 5);
    return dw1000_le64_to_cpu(tx_time);
}



/*===========================================================================*/
/* Reception (RX)                                                            */
/*===========================================================================*/

/**
 * @brief Set timeout value for preamble detection
 *
 * @param[in] dw        driver context
 * @param[in] timeout   timeout (0..65535) is expressed in PAC-size unit,
 *                      a value of 0 disable the timeout.
 */
static inline void
dw1000_rx_set_timeout_preamble(dw1000_t *dw, uint16_t timeout)
{
    _dw1000_reg_write16(dw, DW1000_REG_DRX_CONF, DW1000_OFF_DRX_PRETOC, timeout);
}


/**
 * @brief Set the reception timeout for the full frame
 *
 * @details The timeout value need to take in consideration the
 *          delay before transmission and the transmission time of
 *          the whole frame.
 *
 * @param [in] dw       driver context
 * @param [in] timeout  timeout in "UWB microsencond" units (between 0..65535),
 *                       a value of 0 disable the timeout
 */
void dw1000_rx_set_timeout(dw1000_t *dw, uint16_t timeout);


/**
 * @brief Enable/Disable frame filtering
 *
 * @param[in] dw        driver context
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
void dw1000_rx_set_frame_filtering(dw1000_t *dw, uint16_t bitmask);


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
int dw1000_rx_start(dw1000_t *dw, int8_t rx_mode);


/**
 * @brief Get frame length and ranging flag
 *
 * @param[in]  dw        driver context
 * @param[out] length    frame size (including CRC)
 * @param[out] ranging   indicate a ranging frame
 */
static inline void
dw1000_rx_get_frame_info(dw1000_t *dw, size_t *length, bool *ranging)
{
    const uint32_t rx_finfo =
	_dw1000_reg_read32(dw, DW1000_REG_RX_FINFO, DW1000_OFF_NONE);

    if (length) {
#if DW1000_WITH_PROPRIETARY_LONG_FRAME
	const uint32_t msk = dw->radio->proprietary.long_frames
	                   ? DW1000_MSK_RX_FINFO_RXFLE_RXFLEN
	                   : DW1000_MSK_RX_FINFO_RXFLEN;
#else
	const uint32_t msk = DW1000_MSK_RX_FINFO_RXFLEN;
#endif
	*length  = (rx_finfo & msk) >> DW1000_SFT_RX_FINFO_RXFLEN;
    }

    if (ranging) {
	*ranging = rx_finfo & DW1000_FLG_RX_FINFO_RNG;
    }
}


/**
 * @brief Get frame length
 *
 * @param[in]  dw        driver context
 *
 * @return frame size (including CRC)
 */
static inline size_t
dw1000_rx_get_frame_length(dw1000_t *dw)
{
    size_t length;
    dw1000_rx_get_frame_info(dw, &length, NULL);
    return length;
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
void dw1000_rx_read_frame_data(dw1000_t *dw,
			       uint8_t *data, size_t length, size_t offset);


/**
 * @brief Read reception information
 *
 * @details Retrieve information about signal quality 
 *          (first path, standard noise, ...)
 *
 * @param [in]  dw      driver context
 * @param [out] rxinfo  information about frame reception
 */
void dw1000_rx_get_info(dw1000_t *dw, dw1000_rxinfo_t *rxinfo);


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
				 int32_t *offset, uint32_t *interval);


/**
 * @brief Compute the estimated received signal and/or firstpath power in dBm
 *
 * @param[in]  dw         driver context
 * @param[out] signal     received signal power in dBm
 * @param[out] firstpath  received firstpath power in dBm
 */
void dw1000_rx_get_power_estimate(dw1000_t *dw,
				   double *signal, double *firstpath);


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
double dw1000_rx_power_correction(dw1000_t *dw, double p);


/**
 * @brief Check the status of RX done 
 *
 * @details Everything that specified that the reception is ended:
 *           good (RXFCG), received with errors, or timeout.
 *
 * @param[in]  dw       driver context
 */
static inline uint32_t
dw1000_rx_is_status_done(dw1000_t *dw)
{
    // UM §7.2.17: System Event Status Register
    //  => Status is a 5 bytes register (DW1000_REG_SYS_STATUS),
    //     we will use only the first 4 bytes to access
    // We don't use the DW1000_MSK_SYS_STATUS_ALL_RX_GOOD,
    //  as it includes flags indicating the *good* intermediate states
    //  we will directly used the final good state: DW1000_FLG_SYS_STATUS_RXFCG
    return _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE) &
	(DW1000_FLG_SYS_STATUS_RXFCG     |
	 DW1000_MSK_SYS_STATUS_ALL_RX_TO |
	 DW1000_MSK_SYS_STATUS_ALL_RX_ERR);
}


/**
 * @brief Clear/Acknowledge the RX done events (good, errors, timeout)
 *
 * @param[in]  dw       driver context
 */
static inline
void dw1000_rx_clear_status_done(dw1000_t *dw) {
    // Trigger clearing of RX frame received event by setting them to 1
    // UM §7.2.17: System Event Status Register
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
		       DW1000_MSK_SYS_STATUS_ALL_RX);
}


/**
 * @brief Get frame reception time
 *
 * @param[in]  dw        driver context
 *
 * @return RMARKER reception time (40-bit clock)
 */
static inline
uint64_t dw1000_rx_get_rmarker_time(dw1000_t *dw) {
    uint64_t rx_time = 0;
    _dw1000_reg_read(dw, DW1000_REG_RX_TIME, DW1000_OFF_RX_TIME_RX_STAMP,
		    ((uint8_t *)(&rx_time)), 5);
    return dw1000_le64_to_cpu(rx_time);
}


/**
 * @brief Get (an estimation of) transmitter clock drift
 *
 * @details If positive the transmitter clock is running faster, 
 *          if negative the transmitter clock is running slower.
 *
 * @param[in]  dw       driver context
 */
static inline
double dw1000_rx_get_clock_drift(dw1000_t *dw) {
    int32_t  offset;
    uint32_t interval;
    dw1000_rx_get_time_tracking(dw, &offset, &interval);
    return (double)offset / (double)interval;
}


/** @} */

#endif
