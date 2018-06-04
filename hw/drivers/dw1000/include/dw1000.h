/*
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
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


#define DW1000_WITH_DWM1000_EVK_COMPATIBILITY   TRUE

#define DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH TRUE
#define DW1000_WITH_PROPRIETARY_SFD 		TRUE
#define DW1000_WITH_SFD_TIMEOUT 		0
#define DW1000_WITH_SFD_TIMEOUT_DEFAULT 	0
#define DW1000_WITH_PROPRIETARY_LONG_FRAME 	0




#include "bswap.h"
#include "dw1000_otp.h"
#include "dw1000_reg.h"



/**
 * @brief Frequency of the clock for timestamping
 */
#define DW1000_TIMESTAMP_FREQ (499200000ull * 128)

/**
 * @brief Round up the clock value to be used in delayed transmit/receive
 */
#define DW1000_CLOCK_ROUNDUP(x) (((x) + 511) & (~0x1FF))



#define DW1000_US(x) ((((x) * DW1000_TIMESTAMP_FREQ) + (1000000-1)) / 1000000)
#define DW1000_MS(x) ((((x) * DW1000_TIMESTAMP_FREQ) + (1000-1)) / 1000)


#define DW1000_TIME_CLOCK_MHZ 63897.6

/**
 * @brief Length of CRC field
 */
#define DW1000_CRC_LENGTH 2



/**
 */
#define DW1000_TU_TO_US(x) (x / DW1000_TIME_CLOCK_MHZ)
#define DW1000_TU_TO_MS(x) (DW1000_TU_TO_US(x)/1000)

/**
 */
#define DW1000_TU_MIN 512




/*===========================================================================*/
/* Compile time options                                                      */
/*===========================================================================*/

/**
 * @brief Increase transmit power by about 3db, for compatibility with
 *        DecaRanging software when using DWM1000 module
 */
#if !defined(DW1000_WITH_DWM1000_EVK_COMPATIBILITY) || defined(__DOXYGEN__)
#define DW1000_WITH_DWM1000_EVK_COMPATIBILITY   FALSE
#endif

/**
 * @brief Add support for proprieray preamble length
 */
#if !defined(DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH) || defined(__DOXYGEN__)
#define DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH TRUE
#endif

/**
 * @brief Add support for proprietary SFD
 */
#if !defined(DW1000_WITH_PROPRIETARY_SFD) || defined(__DOXYGEN__)
#define DW1000_WITH_PROPRIETARY_SFD TRUE
#endif

/**
 * @brief Add support for proprietary long frame
 */
#if !defined(DW1000_WITH_PROPRIETARY_LONG_FRAME) || defined(__DOXYGEN__)
#define DW1000_WITH_PROPRIETARY_LONG_FRAME TRUE
#endif

/**
 * @brief Add support for user defined SFD timeout
 */
#if !defined(DW1000_WITH_SFD_TIMEOUT) || defined(__DOXYGEN__)
#define DW1000_WITH_SFD_TIMEOUT TRUE
#endif

/**
 * @brief Use default SFD timeout value instead of computed one
 */
#if !defined(DW1000_WITH_SFD_TIMEOUT_DEFAULT) || defined(__DOXYGEN__)
#define DW1000_WITH_SFD_TIMEOUT_DEFAULT TRUE
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
#define DW1000_WITH_HOTFIX_AAT_IEEE802_15_4_2011 TRUE
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
     * @note  {DW1000_PRF_16MHZ} or {DW1000_PRF_64MHZ}
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
     * @brief Bit rate {DW1000_BITRATE_110KBPS, DW1000_BITRATE_850KBPS 
     *        or DW1000_BITRATE_6800KBPS}
     */
    uint8_t    bitrate;
    
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
	 * @brief USe non standard SFD (improved performance)
	 */
	uint8_t sfd:1;
#endif
    } proprietary;
#endif
} *dw1000_radio_t;





/**
 * @brief DW1000 driver context
 */
typedef struct dw1000 *dw1000_t;

/**
 * @brief DW1000 Configuration
 */
typedef struct {
    uint8_t    lde_loading:1;
    /**
     * @brief Define led blink time in 14ms unit
     */
    uint8_t    leds_blink_time;
    /**
     * @brief SPI driver
     */
    SPIDriver *spi;
    /**
     * @brief SPI config for low speed (<3MHz)
     */
    const SPIConfig *spi_low_cfg;
    /**
     * @brief SPI config for high speed (<20MHz)
     * @note  Can be set to NULL if only using low speed
     */
    const SPIConfig *spi_high_cfg;
    /**
     * @brief IRQ line
     */
    ioline_t   irq;
    /**
     * @brief Reset line
     */
    ioline_t   reset;
    /**
     * @brief WakeUp line
     */
    ioline_t   wakeup;
    /**
     * @brief Cristal trimming (optional)
     */
    uint8_t    xtrim;
    /**
     * @brief Set of LED wired to DW1000
     */
    uint8_t    leds;
    /**
     * @brief Use of double buffer
     */
    bool       dblbuff; 
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
	void (*tx_done   )(dw1000_t dw, uint32_t status);
	void (*rx_timeout)(dw1000_t dw, uint32_t status);
	void (*rx_error  )(dw1000_t dw, uint32_t status);
	void (*rx_ok     )(dw1000_t dw, size_t length, bool ranging,
			                                     uint32_t status);
    } cb;
} DW1000Config;



/**
 * @brief DW1000 driver context
 */
struct dw1000 {
    const DW1000Config *config;

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

    int8_t rxpacc_adj;
    
    struct {
	uint32_t sys_cfg;
	uint32_t tx_fctrl;
    } reg;
};






/**
 * @brief Maximum allowed delay for automatic activation of reception
 *        after transmission.
 */
#define DW1000_MAX_TX_RX_ACTIVATION_DELAY ((1<<20)-1)




/*
 * Clock configuration
 */

#define DW1000_CLOCK_SEQUENCING            0
#define DW1000_CLOCK_SYS_XTI               1
#define DW1000_CLOCK_SYS_PLL               2
#define DW1000_CLOCK_TX_CONTINOUSFRAME     3




#define DW1000_BITRATE_110KBPS  0
#define DW1000_BITRATE_850KBPS  1
#define DW1000_BITRATE_6800KBPS 2

#define DW1000_PRF_4MHZ      0  //!< Unsupported by DW100 receiver!
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
#define DW1000_PAC8        0   //!< PAC  8
#define DW1000_PAC16       1   //!< PAC 16
#define DW1000_PAC32       2   //!< PAC 32
#define DW1000_PAC64       3   //!< PAC 64



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




void dw1000_init(dw1000_t dw, const DW1000Config *cfg);

msg_t dw1000_initialise(dw1000_t dw);

void dw1000_hardreset(dw1000_t dw);



void _dw1000_reg_read(dw1000_t dw,
   uint8_t reg, size_t offset, void* data, size_t length);

void _dw1000_reg_write(dw1000_t dw,
   uint8_t reg, size_t offset, void* data, size_t length);

void _dw1000_reg_set32(dw1000_t dw,
		      uint8_t reg, size_t offset, uint32_t value);

void _dw1000_reg_clear32(dw1000_t dw,
			uint8_t reg, size_t offset, uint32_t value);


void dw1000_leds_blink(dw1000_t dw, uint8_t leds);


void dw1000_configure(dw1000_t dw, dw1000_radio_t radio);


void dw1000_interrupt(dw1000_t dw, uint32_t bitmask, bool enable);

bool dw1000_process_events(dw1000_t dw);


msg_t dw1000StartSend(dw1000_t dw,
		      uint8_t *data, size_t length, uint8_t tx_mode);


void dw1000_otp_read(dw1000_t dw,
		     uint16_t address, uint32_t *data, size_t length);




void dw1000_read_temp_vbat(dw1000_t dw, uint16_t *temp, uint16_t *vbat);



void dw1000_txrx_set_time(dw1000_t dw, uint64_t time);

void dw1000_tx_set_rx_activation_delay(dw1000_t dw, uint32_t delay);

void dw1000_tx_write_frame_data(dw1000_t dw, uint8_t *data, size_t length, size_t offset);

msg_t dw1000_tx_send(dw1000_t dw,
		     uint8_t *data, size_t length, uint8_t tx_mode);




void dw1000_rx_reset(dw1000_t dw);
void dw1000_rx_set_timeout(dw1000_t dw, uint16_t timeout);

void dw1000_rx_set_frame_filtering(dw1000_t dw, uint16_t bitmask);

void dw1000_rx_get_info(dw1000_t dw, dw1000_rxinfo_t *rxinfo);


void dw1000_rx_read_frame_data(dw1000_t dw, uint8_t *data, size_t length, size_t offset);

msg_t dw1000_rx_start(dw1000_t dw, int8_t rx_mode);

void dw1000_rx_get_time_tracking(dw1000_t dw,
				 int32_t *offset, uint32_t *interval);


void dw1000_rx_get_power_estimate(dw1000_t dw,
				  double *signal, double *firstpath);



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
static inline
uint32_t dw1000_otp_get(dw1000_t dw, uint16_t address) {
    uint32_t data;
    dw1000_otp_read(dw, address, &data, 1);
    return le32_to_cpu(data);
}

/**
 * @private
 * @brief Write a byte to DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to write
 * @param[in]  offset   offset in the register
 * @param[in]  data     byte to write
 */
static inline
void _dw1000_reg_write8(dw1000_t dw,
    uint8_t reg, size_t offset, uint8_t data) {
    _dw1000_reg_write(dw, reg, offset, &data, sizeof(data));
}

/**
 * @private
 * @brief Write a 16-bit word to DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to write
 * @param[in]  offset   offset in the register
 * @param[in]  data     16-bit word to write
 */
static inline
void _dw1000_reg_write16(dw1000_t dw,
    uint8_t reg, size_t offset, uint16_t data) {
    data = cpu_to_le16(data);
    _dw1000_reg_write(dw, reg, offset, &data, sizeof(data));
}

/**
 * @private
 * @brief Write a 32-bit word to DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to write
 * @param[in]  offset   offset in the register
 * @param[in]  data     32-bit word to write
 */
static inline
void _dw1000_reg_write32(dw1000_t dw,
    uint8_t reg, size_t offset, uint32_t data) {
    data = cpu_to_le32(data);
    _dw1000_reg_write(dw, reg, offset, &data, sizeof(data));
}

/**
 * @private
 * @brief Read a byte from DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read
 * @param[in]  offset   offset in the register
 *
 * @return byte
 */
static inline
uint8_t _dw1000_reg_read8(dw1000_t dw,
    uint8_t reg, size_t offset) {
    uint8_t data;
    _dw1000_reg_read(dw, reg, offset, &data, sizeof(data));
    return data;
}

/**
 * @private
 * @brief Read a 16-bit word from DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read
 * @param[in]  offset   offset in the register
 *
 * @return 16-bit word
 */
static inline
uint16_t _dw1000_reg_read16(dw1000_t dw,
    uint8_t reg, size_t offset) {
    uint16_t data;
    _dw1000_reg_read(dw, reg, offset, &data, sizeof(data));
    return le16_to_cpu(data);
}

/**
 * @private
 * @brief Read a 32-bit word from DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to read
 * @param[in]  offset   offset in the register
 *
 * @return 32-bit word
 */
static inline
uint32_t _dw1000_reg_read32(dw1000_t dw,
    uint8_t reg, size_t offset) {
    uint32_t data;
    _dw1000_reg_read(dw, reg, offset, &data, sizeof(data));
    return le32_to_cpu(data);
}


/**
 * @brief Check the status of TX done (ie: TXFRS flag)
 *
 * @details Everything that specified that the transmission is ended.
 *
 * @param[in]  dw       driver context
 */
static inline
uint32_t dw1000_tx_is_status_done(dw1000_t dw) {
    // UM §7.2.17: System Event Status Register
    //  => Status is a 5 bytes register (DW1000_REG_SYS_STATUS),
    //     we will use only the first 4 bytes to access
    //     DW1000_FLG_SYS_STATUS_TXFRS (7)
    return _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE) &
	DW1000_FLG_SYS_STATUS_TXFRS;
}

/**
 * @brief Check the status of RX done 
 *
 * @details Everything that specified that the reception is ended:
 *           good (RXFCG), received with errors, or timeout.
 *
 * @param[in]  dw       driver context
 */
static inline
uint32_t dw1000_rx_is_status_done(dw1000_t dw) {
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
 * @brief Clear/Acknowledge the TX done event (ie: TXFRS flag)
 *
 * @param dw        driver context
 */
static inline
void dw1000_tx_clear_status_done(dw1000_t dw) {
    // Trigger clearing of TX frame sent event by setting it 1
    // UM §7.2.17: System Event Status Register
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
		       DW1000_FLG_SYS_STATUS_TXFRS);
}

/**
 * @brief Clear/Acknowledge the RX done events (good, errors, timeout)
 *
 * @param[in]  dw       driver context
 */
static inline
void dw1000_rx_clear_status_done(dw1000_t dw) {
    // Trigger clearing of RX frame received event by setting them to 1
    // UM §7.2.17: System Event Status Register
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
		       DW1000_MSK_SYS_STATUS_ALL_RX);
}


/**
 * @brief Set timeout value for preamble detection
 *
 * @param[in] dw        driver context
 * @param[in] timeout   timeout (0..65535) is expressed in PAC-size unit,
 *                      a value of 0 disable the timeout.
 */
static inline
void dw1000_rx_set_timeout_preamble(dw1000_t dw, uint16_t timeout) {
    _dw1000_reg_write16(dw, DW1000_REG_DRX_CONF, DW1000_OFF_DRX_PRETOC,
		       timeout);
}


/**
 * @brief Get frame length and ranging flag
 *
 * @param[in]  dw        driver context
 * @param[out] length    frame size (including CRC)
 * @param[out] ranging   indicate a ranging frame
 */
static inline
void dw1000_rx_get_frame_info(dw1000_t dw, size_t *length, bool *ranging) {
    const uint32_t rx_finfo =
	_dw1000_reg_read32(dw, DW1000_REG_RX_FINFO, DW1000_OFF_NONE);

    if (length) {
#if DW1000_WITH_PROPRIETARY_LONG_FRAME
	const uint32_t msk = dw->radio->proprietary.long_frames
	                   ? DW1000_MSK_RX_FINFO_RXFLEN_EXT
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
static inline
size_t dw1000_rx_get_frame_length(dw1000_t dw) {
    size_t length;
    dw1000_rx_get_frame_info(dw, &length, NULL);
    return length;
}


/**
 * @brief Get system time
 *
 * @param[in]  dw        driver context
 *
 * @return system time (40-bit clock)
 */
static inline
uint64_t dw1000_get_system_time(dw1000_t dw) {
    uint64_t sys_time = 0;
    _dw1000_reg_read(dw, DW1000_REG_SYS_TIME, DW1000_OFF_NONE,
		    ((uint8_t *)(&sys_time)) + 0, 5);

    return le64_to_cpu(sys_time);
}

/**
 * @brief Get frame reception time
 *
 * @param[in]  dw        driver context
 *
 * @return RMARKER reception time (40-bit clock)
 */
static inline
uint64_t dw1000_rx_get_rmarker_time(dw1000_t dw) {
    uint64_t rx_time = 0;
    _dw1000_reg_read(dw, DW1000_REG_RX_TIME, DW1000_OFF_RX_TIME_RX_STAMP,
		    ((uint8_t *)(&rx_time)), 5);
    return le64_to_cpu(rx_time);
}


/**
 * @brief Get frame transmission time
 *
 * @param[in]  dw        driver context
 *
 * @return RMARKER transmission time (40-bit clock)
 */
static inline
uint64_t dw1000_tx_get_rmarker_time(dw1000_t dw) {
    uint64_t tx_time = 0;
    _dw1000_reg_read(dw, DW1000_REG_TX_TIME, DW1000_OFF_TX_TIME_TX_STAMP,
		    ((uint8_t *)(&tx_time)), 5);
    return le64_to_cpu(tx_time);
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
double dw1000_rx_get_clock_drift(dw1000_t dw) {
    int32_t  offset;
    uint32_t interval;
    dw1000_rx_get_time_tracking(dw, &offset, &interval);
    return (double)offset / (double)interval;
}


/** @} */

#endif

