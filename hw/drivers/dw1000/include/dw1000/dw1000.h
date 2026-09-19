/*
 * Copyright (c) 2018-2024,2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_H__
#define __DW1000_H__

/**
 * @file    dw1000.h
 * @brief   DW1000 low level driver header.
 *
 * @addtogroup DW1000
 * @{
 */


#include "dw1000/osal.h"
#include "dw1000/dw1000_bswap.h"
#include "dw1000/dw1000_otp.h"
#include "dw1000/dw1000_reg.h"
#include "dw1000/dw1000_version.h"



/*===========================================================================*/
/* Compile time options                                                      */
/*===========================================================================*/

/**
 * \defgroup Config Compile time configuration
 */
/** @{ */

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
 * @brief Keep the bench diagnostics
 *
 * @details Three things a measurement wants and a radio does not:
 *          the receiver enable placed back inside the event pass
 *          (dw1000_config_t::rx_enable_early), the flags of a refused
 *          delayed send (dw1000_t::tx_late_flags) and the status word
 *          at three points of a pass (dw1000_t::dbg_pass). The last
 *          costs two SYS_STATUS reads per double buffered good frame.
 *          Off unless something is being measured; INVESTIGATE.md
 *          says what each was for.
 */
#if !defined(DW1000_WITH_DEBUG) || defined(__DOXYGEN__)
#define DW1000_WITH_DEBUG 0
#endif

/**
 * @brief Build with the event counters
 *
 * @details The diagnostic bank at register 0x2F (UM §7.2.48): PHY
 *          header errors, Reed-Solomon errors, good and bad CRCs,
 *          frames the filter rejected, overruns, the three timeouts,
 *          frames sent, and the two warnings. The chip counts them
 *          whether or not they are read, so the cost of building this
 *          in is the code alone, and the cost of enabling them at run
 *          time is a small amount of power (§7.2.48.1 says so).
 *          Counting cannot be turned back off once started, only
 *          cleared.
 */
#if !defined(DW1000_WITH_EVENT_COUNTERS) || defined(__DOXYGEN__)
#define DW1000_WITH_EVENT_COUNTERS 0
#endif

/**
 * @brief Build with the die temperature corrections
 *
 * @details Transmit power and channel bandwidth both drift with die
 *          temperature, and the driver otherwise reads the temperature
 *          without ever acting on it. Two independent halves, both
 *          needing a reference captured when the board was calibrated:
 *          @p dw1000_tx_power_temp_correction() is arithmetic on a
 *          power register, cheap and safe at any time, while
 *          @p dw1000_tx_calibrate_pg_delay() and
 *          @p dw1000_tx_get_pg_count() drive a calibration on the chip
 *          and demand an idle transceiver.
 */
#if !defined(DW1000_WITH_TEMP_COMPENSATION) || defined(__DOXYGEN__)
#define DW1000_WITH_TEMP_COMPENSATION 0
#endif

/**
 * @brief Build with the accumulator (CIR) read
 *
 * @details The channel impulse response the leading edge estimate was
 *          made from, at register 0x25 (UM §7.2.38). What separates a
 *          first path from a reflection, and so the one diagnostic that
 *          speaks to ranging accuracy rather than to link health. The
 *          whole accumulator is @p DW1000_LEN_ACC_MEM bytes, which is
 *          larger than most of the hosts this driver runs on care to
 *          spare, so the read is partial by design and the buffer is
 *          the caller's.
 */
#if !defined(DW1000_WITH_ACCUMULATOR) || defined(__DOXYGEN__)
#define DW1000_WITH_ACCUMULATOR 0
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
    ((x) / DW1000_TIME_CLOCK_MHZ)

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
 * @brief Speed of light, in metres per second
 *
 * As a double, because the two conversions below need it to be: see
 * @p DW1000_METER_TO_CLOCK().
 *
 * Definable from outside, and left alone if it already is, so that a
 * caller working to a propagation velocity other than this one (a
 * calibrated figure, or a medium that is not air) can set it once for
 * the whole build rather than avoid the conversions. Define it before
 * this header is reached, on the command line or in a configuration
 * header, and every use here follows.
 */
#if !defined(DW1000_SPEED_OF_LIGHT_MPS)
#define DW1000_SPEED_OF_LIGHT_MPS 299792458.0
#endif

/**
 * @brief Convert metres of flight to DW1000 clock unit
 *
 * One metre is about 213.1 ticks and one tick about 4.7 mm, which is why
 * this is floating point where @p DW1000_USEC_TO_CLOCK() is not: metres
 * as an integer would quantise to the nearest 213 ticks, and for the
 * antenna delays in use around here (154.2 m, 154.6 m of round trip)
 * writing 154 instead of 154.6 is 128 ticks out: 0.6 m of range.
 *
 * Meant for a compile-time constant, where it folds and no floating
 * point reaches the image:
 *
 * @code
 *     .tx_antenna_delay = DW1000_METER_TO_CLOCK(154.6) / 2,
 *     .rx_antenna_delay = DW1000_METER_TO_CLOCK(154.6) / 2,
 * @endcode
 *
 * Deliberately not rounded, unlike the µsec and msec conversions: every
 * caller so far halves the result, which would undo it.
 *
 * @note An antenna delay is a register field of 16 bits. Use
 *       @p dw1000_validate_antenna_delay() to convert one that came from
 *       a user, which range-checks it and says what is wrong.
 */
#define DW1000_METER_TO_CLOCK(x)				\
    (((x) * (double)DW1000_TIME_CLOCK_HZ) / DW1000_SPEED_OF_LIGHT_MPS)

/**
 * @brief Convert DW1000 clock unit to metres of flight
 *
 * The other direction, for turning a measured interval into a distance.
 */
#define DW1000_CLOCK_TO_METER(x)				\
    (((x) * DW1000_SPEED_OF_LIGHT_MPS) / (double)DW1000_TIME_CLOCK_HZ)

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

/**
 * @defgroup TxError Why a transmission was refused
 *
 * Every one is negative, so a caller testing the sign or comparing
 * against 0 is unaffected; only a caller that needs to tell the causes
 * apart has to look. @p DW1000_TX_ERR_TOO_LATE keeps the value -1 it
 * always had, that being the cause the headers documented when -1 was
 * the only answer.
 *
 * The distinction worth making is what the caller should do next.
 * @p DW1000_TX_ERR_TOO_LATE and @p DW1000_TX_ERR_BUSY are transient: the
 * same call may succeed later, with more lead or once the transmitter is
 * free. @p DW1000_TX_ERR_LEAD is a configuration the host has to fix
 * (see @p dw1000_tx_set_default_delay()). The rest will fail identically
 * however often they are retried.
 * @{
 */
/** The chip refused the programmed transmission time */
#define DW1000_TX_ERR_TOO_LATE      (-1)
/** A transmission is still in flight; its completion is not consumed */
#define DW1000_TX_ERR_BUSY          (-2)
/** The frame is longer than @p dw1000_tx_get_frame_maxsize() allows */
#define DW1000_TX_ERR_FRAME_SIZE    (-3)
/** No lead is configured, or the one given is at or below the airtime */
#define DW1000_TX_ERR_LEAD          (-4)
/** @p tx_mode names no usable timestamp encoding */
#define DW1000_TX_ERR_MODE          (-5)
/** The embedded timestamp does not fit inside the frame */
#define DW1000_TX_ERR_TIMESTAMP     (-6)
/** Errata 1.4 §3.2 (RX-1): too long while a received frame is held */
#define DW1000_TX_ERR_BUFFER_HELD   (-7)
/** @} */

/**
 * @defgroup RxError Why a reception was refused
 * @{
 */
/** The programmed receive time had already passed, and
 *  @p DW1000_RX_IDLE_ON_DELAY_ERROR asked to stay idle rather than
 *  fall back to an immediate start */
#define DW1000_RX_ERR_TOO_LATE      (-1)
/** No longer returned: a start over a transmission on the air is
 *  recorded and honoured at the completion (dw1000_rx_start()). Kept
 *  for a host that tests for it. */
#define DW1000_RX_ERR_BUSY          (-2)
/** @} */



#define DW1000_TX_POWER_FLG_MANUAL  0x80 /**< @internal */
/* 7 bits, so that a request above the 30.5dB ceiling lands outside the
 * usable range and is clamped by _dw1000_radio_tuning(), instead of
 * losing bit 6 to the gap between mask and flag and silently asking for
 * the *minimum* output power. */
#define DW1000_TX_POWER_MSK_MANUAL  0x7f /**< @internal */

/**
 * @brief Transmit power
 */
#define DW1000_TX_POWER_AUTO	0

/**
 * @brief Define the transmit power up to 30.5dB in 0.5db unit.
 * @note  UM 2.18 §7.2.31.1: the gain control range is 30.5dB (7 coarse
 *        steps of 2.5dB plus 32 fine steps of 0.5dB), ie 61 half-dB
 *        steps. Values above 61 are clamped to 61 (30.5dB); the argument
 *        must stay below 128 or the uint8_t encoding wraps.
 */
#define DW1000_TX_POWER_05DB(v)					\
    ((v) | DW1000_TX_POWER_FLG_MANUAL)
/**
 * @brief Define the transmit power up to 30.5dB in 0.5db step.
 * @note  Values above 30.5 are clamped to 30.5dB; the argument must stay
 *        below 64 or the uint8_t encoding wraps.
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
     * @brief Use of double buffer (UM §4.3.3)
     *
     * When set, dw1000_process_events() re-enables the receiver as soon
     * as a good frame is reported, before calling the rx_ok callback,
     * so that the next frame lands in the other buffer while the
     * callback reads this one out; it toggles the host side buffer
     * pointer once the callback returns. Two obligations follow for the
     * rx_ok callback: it must NOT re-enable the receiver itself, and it
     * must read everything it needs from the frame (RX_BUFFER, RX_TIME,
     * RX_FQUAL, RX_TTCKI, RX_TTCKO) BEFORE returning. Once the pointer
     * has been toggled those registers show the other buffer, and a
     * host that defers the read-out to a later step gets the previous
     * frame's data with the current frame's length. A receiver overrun
     * (RXOVRR: both buffers held while a third frame arrived) is
     * recovered (transceiver off, receiver reset, pointers re-aligned)
     * and reported through the rx_error callback with RXOVRR set in the
     * status, which is expected to re-enable the receiver as for any
     * error. dw1000_initialise() unmasks the RXOVRR interrupt
     * (DW1000_FLG_SYS_MASK_MRXOVRR) along with the receive ones when
     * this flag is set; the recovery would otherwise run only when some
     * other event happened to bring the driver in. That callback is
     * required: the recovery re-arms nothing by itself, so
     * dw1000_initialise() refuses this flag without an rx_error
     * callback rather than leave the receiver to stop for good on the
     * first overrun.
     *
     * Errata 1.4 §3.2 (RX-1) applies to this mode and to no other: a
     * transmit whose payload passes index 127, issued while a frame is
     * still unread in the second receive buffer, corrupts that frame's
     * 129th octet. The send functions refuse such a transmit while a
     * frame is held, and it cannot arise at all without proprietary long
     * frames. See @p dw1000_tx_write_frame_data().
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
     * @brief The driver keeps the receiver on
     *
     * Once dw1000_rx_start() has been called, dw1000_process_events()
     * ends every pass by enabling the receiver again if it should be on
     * and is not: after an error, a timeout, an overrun, a single
     * buffered good frame, and a completion whose send did not ask for
     * a response. It stays off only while a send is on the air, and
     * after dw1000_txrx_off(). The callbacks then re-arm nothing, and a
     * dw1000_rx_start() issued over a send is recorded and honoured at
     * its completion rather than refused.
     */
    uint8_t    rx_keep_on:1;
#if DW1000_WITH_DEBUG || defined(__DOXYGEN__)
    /**
     * @brief Bench knob: enable the receiver inside the event pass
     *
     * Double buffered, a good frame reported beside the host's own
     * completion has the receiver enabled at the end of the pass, once
     * the completion is booked. Set, it is enabled where the frame is
     * handled, before the read-out, as the driver did before 2084ca2.
     * A bench knob (DW1000.md, "A buffer toggle
     * that moves the host off the chip's buffer under a live
     * receiver"); leave clear.
     */
    uint8_t    rx_enable_early:1;
#endif
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
     * @brief Delay to take into account for antenna transmission
     */
    uint16_t   tx_antenna_delay;
    /**
     * @brief Delay to take into account for antenna reception
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
	/** A send that never began has been found out and cleared: the
	 *  transmitter is free again, and the host waiting for that send's
	 *  completion can stop. Diagnosed on dw1000_tx_send(),
	 *  dw1000_tx_start(), dw1000_rx_start() and dw1000_process_events(),
	 *  from the chip's state read past the frame's airtime; optional.
	 *  It runs inside the call that found the send out, which then goes
	 *  on with its own send: do not send from it, note the loss and
	 *  retry after that call returns, or the retry's buffer write lands
	 *  under a frame on the air (Errata TX-2). */
	void (*tx_dropped)(dw1000_t *dw);
    } cb;
} dw1000_config_t;



/**
 * @brief DW1000 driver context
 */
struct dw1000 {
    const dw1000_config_t *config; // Borrowed: must outlive the driver
    struct dw1000_radio radio;     // Copied: caller keeps nothing alive
                                   //  prf == 0 until one has succeeded

    struct {
	uint32_t device;    // DEV_ID, checked against DW1000_ID_DEVICE
	uint32_t chip;      // OTP 0x006, assigned at production test
	uint32_t lot;       // OTP 0x007, foundry lot
    } id;

    uint8_t  xtrim;         // XTAL trim: OTP, config override, else 0x10
    uint8_t  otp_rev;       // OTP revision (OTP 0x01E, high byte)
    uint8_t  ref_vbat_33;   // SAR reading at 3.3V        (OTP 0x008)
    uint8_t  ref_vbat_37;   // SAR reading at 3.7V        (OTP 0x008)
    uint8_t  ref_temp_23;   // SAR reading at 23C         (OTP 0x009)
    uint8_t  ref_temp_ant;  // SAR at antenna calibration (OTP 0x009)

    uint32_t sleep_mode;    // Accumulated, never written: no sleep path
    uint8_t  tx_clk_forced; // TX clock forced on (errata 1.4 3.1, TX-1)
    uint8_t  state;         // The transceiver as the driver knows it, one
                            //  of enum dw1000_state below: TX and TX_W4R
                            //  are a send in progress (Errata 1.4 3.3,
                            //  TX-2, refuses another), RX_W4R a receiver
                            //  the chip put up itself after a send that
                            //  expects a response
    uint32_t tx_ton;        // Preamble+SFD airtime, ticks (APS022 5.4)
    uint32_t tx_delay;      // Whole delayed-send lead, tx_ton included,
    uint32_t tx_retry_delay; // and that of its retry; 0 until the host
                            //  sets dw1000_tx_set_default_delay()

    int8_t   rxpacc_adj;    // RXPACC SFD correction (UM table 18)
    uint16_t rxpacc_nosat;  // Sampled for the frame being reported:
    uint16_t lde_thresh;    // neither is in the swinging set, UM table 7
    uint8_t  rx_held;       // Double buffered: a reported frame is still
                            //  in the host side buffer, being read out
                            //  by the rx_ok callback. Releasing it now
                            //  would hand it back to the chip mid-read.
    uint8_t  rx_reset_due;  // A receive error or timeout was handled
                            //  while a transmission was in flight, so
                            //  the receiver reset UM 4.1.6 asks for
                            //  was put off: dw1000_rx_start() applies it
    uint8_t  rx_want;       // What the host has asked of the receiver,
                            //  one of enum dw1000_rx_want below: NONE,
                            //  ONCE (a dw1000_rx_start() that came over
                            //  a send on the air, honoured at the
                            //  completion's pass) or KEEP (rx_keep_on,
                            //  the host having asked to listen and not
                            //  to stop: dw1000_txrx_off)
    uint16_t tx_length;     // Frame length of the send in progress, CRC
                            //  included, for its airtime
    uint32_t tx_airtime;    // Airtime of the send in progress, ticks
    uint8_t  tx_delayed;    // The send in progress waits for DX_TIME
#if DW1000_WITH_DEBUG
    uint32_t dbg_pass[3];   // Bench trace (DW1000_DUPLEX_TRACE): SYS_STATUS
                            //  as dw1000_process_events() read it on
                            //  entry, after the double buffered clear,
                            //  after the HRBPT toggle (0: not reached)
    uint16_t tx_late_flags; // Bytes 3..4 of SYS_STATUS at the last
                            //  too-late refusal (HPDWARN bit 3, TXPUTE
                            //  bit 10): kept for a probe, since the
                            //  TRXOFF of the refusal clears them
    uint32_t dbg_lde_cuts;  // How many frames dw1000_process_events()
                            //  reported through rx_error for having had
                            //  their LDE run cut by a TRXOFF (RXFCG set,
                            //  LDEDONE clear): the bench counts them
                            //  against the soak's deliveries
                            //  (doc/bench/2026-09-19-lde)
#endif
    uint64_t tx_suspect;    // System time at which a send in progress was
                            //  first seen absent from the chip (no TX
                            //  flag, transceiver not in TX); 0 otherwise.
                            //  Past the airtime it is a dropped send.

    struct {
	uint32_t sys_cfg;   // Shadow of SYS_CFG,  UM 7.2.6
	uint32_t tx_fctrl;  // Shadow of TX_FCTRL, UM 7.2.10
    } reg;

    uint32_t tx_power;      // Encoded TX_POWER word (UM 7.2.31)
};

/**
 * @internal
 * @brief The transceiver, as far as the driver knows
 *
 * One state in place of the flags it used to keep, so that every
 * operation and every event is a cell of one table (DESIGN.md, "One
 * state, one table"). IDLE is the chip doing nothing; RX the receiver
 * up at the host's request or the driver's; RX_W4R the receiver the
 * chip put up itself at the end of a send that expects a response; TX
 * a send on the air; TX_W4R one that will leave the receiver up.
 */
enum dw1000_state {
    DW1000_STATE_IDLE = 0,
    DW1000_STATE_RX,
    DW1000_STATE_RX_W4R,
    DW1000_STATE_TX,
    DW1000_STATE_TX_W4R,
};

/**
 * @internal
 * @brief What the host has asked of the receiver
 *
 * One field in place of the two the driver kept until 1.5 (`rx_wanted`
 * and `rx_deferred`), the receive intent being one thing with three
 * settings rather than two flags with a dead combination (DESIGN.md,
 * "The receiver policy, and the send the chip never began"). NONE is
 * nothing asked for; ONCE a single start, recorded over a send on the
 * air and spent when the receiver goes back up; KEEP the rx_keep_on
 * policy's standing ask, which no enable consumes and which only
 * dw1000_txrx_off() (or a soft reset) ends.
 */
enum dw1000_rx_want {
    DW1000_RX_WANT_NONE = 0,
    DW1000_RX_WANT_ONCE,
    DW1000_RX_WANT_KEEP,
};

/** @internal A send is in progress: another is refused (Errata TX-2) */
static inline bool _dw1000_tx_pending(const dw1000_t *dw) {
    return (dw->state == DW1000_STATE_TX) || (dw->state == DW1000_STATE_TX_W4R);
}
/** @internal The receiver is up, by the host or by WAIT4RESP */
static inline bool _dw1000_rx_up(const dw1000_t *dw) {
    return (dw->state == DW1000_STATE_RX) || (dw->state == DW1000_STATE_RX_W4R);
}
/** @internal A response is expected: by a send on the air, or by the
 *  receiver it left up */
static inline bool _dw1000_w4r(const dw1000_t *dw) {
    return (dw->state == DW1000_STATE_TX_W4R) || (dw->state == DW1000_STATE_RX_W4R);
}
/** @internal The send is over: IDLE, or the receiver WAIT4RESP put up.
 *  A state the good-frame branch of the same pass has already moved on
 *  (RX or RX_W4R, the receiver being up by then) is left where it is. */
static inline void _dw1000_tx_done_state(dw1000_t *dw) {
    if (dw->state == DW1000_STATE_TX)
	dw->state = DW1000_STATE_IDLE;
    else if (dw->state == DW1000_STATE_TX_W4R)
	dw->state = DW1000_STATE_RX_W4R;
}



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
#define DW1000_RX_NO_DBLBUFF_SYNC     0x04

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
 * @brief Write data to the DW1000 register
 *
 * @param[in]  dw       driver context
 * @param[in]  reg      register to write [0x00..0x3F]
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
 * @note  @p cfg is held by pointer, not copied, and is read for as long
 *        as the driver is used: the callbacks on every event, and
 *        @p dblbuff on every received frame. It must outlive @p dw.
 *        Contrast @p dw1000_configure(), which copies.
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
 * @retval -1           Chip not identified as DW1000, or @p dblbuff is
 *                      set with no rx_error callback to report a
 *                      receiver overrun to, which would leave the
 *                      receiver off for good on the first one
 */
int dw1000_initialise(dw1000_t *dw);


/**
 * @brief Configure the DW1000 driver
 *
 * @note  The configuration is validated in every build, not only where
 *        assertions are enabled. Each field indexes a tuning table, so an
 *        out of range value would be an out of range read.
 *
 * @warning On failure the chip is left untouched and @p dw keeps the
 *          configuration it already had (which on a first call is
 *          none at all). Transmitting or receiving after a failed
 *          configure is a programming error.
 *
 * @note  The configuration is copied into @p dw. Calling code keeps no
 *        version of it alive: build one on the stack, pass it, let it
 *        go; it needs neither @p static nor a lifetime beyond the
 *        call. That is why it is copied rather than borrowed, the
 *        driver reading it long afterwards
 *        (@p dw1000_rx_get_power_estimate(),
 *        @p dw1000_rx_power_correction(), and with long frames
 *        @p dw1000_rx_get_frame_info() on every received frame), which
 *        would make the natural spelling (a local in a setup function)
 *        a use after return.
 *
 * @pre   @p dw1000_initialise() has been called: this function builds on
 *        the SYS_CFG shadow and on DIS_STXP that initialisation set up,
 *        and it caches the TX_FCTRL base that @p dw1000_tx_fctrl()
 *        needs.
 *
 * @param dw        driver context
 * @param radio     radio configuration
 *
 * @retval  0       DW1000 configured
 * @retval -1       Invalid or unsupported radio configuration:
 *                  channel not one of 1, 2, 3, 4, 5, 7; unknown bitrate,
 *                  PAC or preamble length; preamble code outside 1..24
 *                  or not matching the PRF (1..8 for 16MHz, 9..24 for
 *                  64MHz); PRF of 4MHz, which the receiver does not
 *                  support; or @p radio being NULL.
 */
int dw1000_configure(dw1000_t *dw, dw1000_radio_t radio);



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
 * @param[out] temp  temperature (in 1/100 °C, signed: can be below 0 °C)
 * @param[out] vbat  battery voltage in mV
 */
void dw1000_read_temp_vbat(dw1000_t *dw, int16_t *temp, uint16_t *vbat);


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
 * @brief Set the used PAN identifier.
 *
 * @note  This will be used by the Receive Frame Filtering function
 *        (UM §7.2.5). PANADR is left at 0 by the chip reset, so with
 *        frame filtering enabled and no PAN id set the DW1000 accepts
 *        only frames addressed to PAN id 0.
 *
 * @param[in]  dw       driver context
 * @param[in]  pan_id   PAN identifier
 */
static inline
void dw1000_set_pan_id(dw1000_t *dw, uint16_t pan_id)
{
    _dw1000_reg_write16(dw, DW1000_REG_PANADR,
			DW1000_OFF_PANADR_PAN_ID, pan_id);
}


/**
 * @brief Get the used PAN identifier.
 *
 * @param[in]  dw       driver context
 *
 * @return PAN identifier
 */
static inline
uint16_t dw1000_get_pan_id(dw1000_t *dw)
{
    return _dw1000_reg_read16(dw, DW1000_REG_PANADR,
			      DW1000_OFF_PANADR_PAN_ID);
}


/**
 * @brief Set the used short (16-bit) address.
 *
 * @note  This will be used by the Receive Frame Filtering function
 *        (UM §7.2.5). SHORT_ADDR is left at 0 by the chip reset, so with
 *        frame filtering enabled and no short address set, unicast data
 *        frames addressed to this node by its short address are rejected
 *        (AFFREJ); only broadcast and EUI-addressed frames get through.
 *
 * @param[in]  dw       driver context
 * @param[in]  addr     short address
 */
static inline
void dw1000_set_short_address(dw1000_t *dw, uint16_t addr)
{
    _dw1000_reg_write16(dw, DW1000_REG_PANADR,
			DW1000_OFF_PANADR_SHORT_ADDR, addr);
}


/**
 * @brief Get the used short (16-bit) address.
 *
 * @param[in]  dw       driver context
 *
 * @return short address
 */
static inline
uint16_t dw1000_get_short_address(dw1000_t *dw)
{
    return _dw1000_reg_read16(dw, DW1000_REG_PANADR,
			      DW1000_OFF_PANADR_SHORT_ADDR);
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
 * @warning @p dw1000_process_events() only clears the status bits it
 *          reports on: RXFCG and the receive groups, TXFRS and the
 *          transmit group. Unmasking a bit it does not handle (CPLOCK,
 *          GPIOIRQ, TXBERR, RFPLL_LL, CLKPLL_LL, or an intermediate TX
 *          or RX bit without its terminal bit) leaves that bit set
 *          once it is raised, and so leaves the IRQ line asserted for
 *          good (UM §7.2.16). The mask @p dw1000_initialise() installs
 *          never does this; a caller widening it is on its own.
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
 *        which means that the 9 lower bits of the given time are ignored.
 *
 * @note  For a delayed *send* this is the RMARKER, the end of the SFD,
 *        not the moment transmission starts. The chip has to be sending
 *        preamble @p dw1000_tx_get_preamble_airtime() before it (about
 *        138 us at a 128 symbol preamble, 4.2 ms at 4096) so a time
 *        less than that ahead of the present cannot be met however
 *        quickly the host follows with @p dw1000_tx_start(). Leave room
 *        for the host's own latency on top: the register writes between
 *        this call and TXSTRT, and the chip's transmit power-up.
 *        @p dw1000_tx_extended_sendv() sizes and checks all of that on
 *        the caller's behalf; a caller driving TXSTRT itself owns it.
 * @note  A delayed *receive* has no such floor (nothing is on the air
 *        before the receiver turns on) only the host latency up to
 *        @p dw1000_rx_start().
 *
 * @param dw        driver context
 * @param time      time for delayed send or received time
 */
void dw1000_txrx_set_time(dw1000_t *dw, uint64_t time);


/**
 * @internal
 * @brief Turn off transceiver, clearing the given event status bits
 *
 * Backs dw1000_txrx_off() and dw1000_txrx_idle(); the event processing
 * calls it directly to drop what the receiver raised without touching
 * what the transmitter did.
 *
 * @param dw        driver context
 * @param clear     event status bits to clear (none when 0)
 */
void _dw1000_txrx_off(dw1000_t *dw, uint32_t clear);

/**
 * @internal
 * @brief Whether the send in progress never began, clearing it if so
 *
 * @details A TXSTRT is dropped by the chip when written into a receiver
 *          locking onto a preamble (measured), and a delayed send can
 *          be dropped by Errata TX-1. Either leaves tx_pending set with
 *          no transmit flag ever raised. This looks at the chip: a TX
 *          flag, or PMSC in TX or TX_WAIT, means the send is real. Seen
 *          absent, the send is only suspected, the time noted; seen
 *          absent again past its airtime and a margin, it is dropped:
 *          tx_pending is cleared, the tx_dropped callback runs, and
 *          true is returned. No SPI on a send that is where it should be
 *          beyond the status word the caller already has.
 *
 * @param[in]  dw       driver context
 * @param[in]  status   SYS_STATUS, as just read
 * @return true when a dropped send was cleared
 */
bool _dw1000_tx_dropped(dw1000_t *dw, uint32_t status);

/**
 * @internal
 * @brief Book a completion the chip shows, without reporting it
 *
 * @details What the TXFRS branch of dw1000_process_events() records
 *          about a finished send, minus the callback: the transmitter
 *          is free, the forced TX clock released, and a WAIT4RESP send
 *          has left the receiver up. For a send that is over and
 *          unreported when the host sends again, which consumes it.
 *
 * @param[in]  dw       driver context
 */
void _dw1000_tx_release(dw1000_t *dw);


/**
 * @brief Turn off transceiver, dropping the pending events
 *
 * The transceiver returns to IDLE and every event the transmitter or the
 * receiver had raised is cleared, whether it has been reported or not.
 * That is what makes it the right call for shutting the radio down or
 * abandoning an operation; when the pending events still matter, use
 * dw1000_txrx_idle().
 *
 * @param dw        driver context
 */
/** dw1000_txrx_stop(): leave the pending event status in place */
#define DW1000_TXRX_KEEP_EVENTS         (1 << 0)

/**
 * @brief Stop the transceiver
 *
 * The transceiver returns to IDLE, a send or a reception in progress
 * abandoned. Every event the transmitter or the receiver had raised goes
 * with it, reported or not, unless @p DW1000_TXRX_KEEP_EVENTS is among
 * @p flags. dw1000_txrx_off() and dw1000_txrx_idle() are the two
 * settings of that flag by name.
 *
 * @param dw        driver context
 * @param flags     0, or @p DW1000_TXRX_KEEP_EVENTS
 */
static inline void
dw1000_txrx_stop(dw1000_t *dw, int flags)
{
    if (! (flags & DW1000_TXRX_KEEP_EVENTS)) {
	// rx_keep_on: the host stops listening, and a start it recorded
	// over a send goes with it
	dw->rx_want = DW1000_RX_WANT_NONE;
    }
    _dw1000_txrx_off(dw, (flags & DW1000_TXRX_KEEP_EVENTS) ? 0 :
			 (DW1000_MSK_SYS_STATUS_ALL_TX     |
			  DW1000_MSK_SYS_STATUS_ALL_RX_ERR |
			  DW1000_MSK_SYS_STATUS_ALL_RX_TO  |
			  DW1000_MSK_SYS_STATUS_ALL_RX_GOOD));
}

static inline void
dw1000_txrx_off(dw1000_t *dw)
{
    dw1000_txrx_stop(dw, 0);
}


/**
 * @brief Turn off transceiver, keeping the pending events
 *
 * The transceiver returns to IDLE, leaving the event status untouched:
 * a frame the receiver has already reported, or a transmission that has
 * just completed, is still there to be processed.
 *
 * @param dw        driver context
 */
static inline void
dw1000_txrx_idle(dw1000_t *dw)
{
    dw1000_txrx_stop(dw, DW1000_TXRX_KEEP_EVENTS);
}



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
 * @brief Read the TX_POWER register back from the chip
 *
 * The applied transmit power: what @p DW1000_TX_POWER_AUTO resolved to
 * out of the calibration table, or what a manual setting was clamped and
 * encoded to. @p dw1000_tx_power_to_05db() decodes it.
 *
 * It is read from the chip rather than from @p dw->tx_power, the word the
 * driver last wrote, but do not read more into that than it carries.
 * TX_POWER is a plain read/write register with no status field, so it
 * returns what was written; and dw1000_configure() clamps, encodes and
 * resolves the automatic setting before caching, so the cache is already
 * the applied value and not the request. The chip is not a second opinion
 * here, and an earlier version of this comment said it was.
 *
 * What the read adds over the cache is one thing only: it is what would
 * notice the chip having reset since it was configured. The identity
 * check in dw1000_initialise() would not, a reset chip reporting the same
 * DEV_ID with every configuration register back at its default. Anything
 * else it could catch, a write that never landed or a chip that never
 * left reset, that identity check catches already and for every register.
 *
 * And it is a one field proxy even for that. A chip that reset has also
 * lost the channel, the PRF and the antenna delays, each of which
 * invalidates a measurement at least as thoroughly. A caller that needs
 * to know the chip still holds its configuration wants all of them
 * checked, not this one.
 *
 * @param dw   driver context
 * @return     the TX_POWER word (UM 2.18 7.2.31)
 */
uint32_t dw1000_tx_get_power(dw1000_t *dw);

/**
 * @brief Decode a TX_POWER word to half-dB steps
 *
 * The exact inverse of the encoding _dw1000_radio_tuning() applies: a
 * 3-bit coarse (DA gain) field holding (6 - coarse) and a 5-bit fine
 * (mixer gain) field, for 61 half-dB steps over 30.5 dB.
 *
 * Being the exact inverse, it cannot detect an error in that encoding:
 * a wrong encoder round-trips cleanly through this. It exists so that a
 * consumer needs no second copy of the field layout.
 *
 * Pure: it touches no chip, so it decodes a word from anywhere.
 *
 * @param txpower  a TX_POWER word
 * @return         applied power in half-dB steps, 0 .. 61
 */
uint8_t dw1000_tx_power_to_05db(uint32_t txpower);


/**
 * @brief Largest frame the chip can carry, CRC included
 *
 * @details 127 in standard mode, and 1023 only when the build has
 *          proprietary long frames AND the radio is configured for them
 *          (UM §7.2.10, §3.4). Both conditions, which is why this is a
 *          run time value and not @p DW1000_FRAME_MAXSIZE.
 *
 * @note  This is the whole frame. With the CRC appended automatically
 *        (that is, without @p DW1000_TX_NO_AUTO_CRC) the largest payload
 *        a caller may pass to the send functions is this less
 *        @p DW1000_CRC_LENGTH.
 *
 * @param[in]  dw       driver context
 *
 * @return the maximum frame length, in bytes
 */
static inline size_t
dw1000_tx_get_frame_maxsize(dw1000_t *dw)
{
#if DW1000_WITH_PROPRIETARY_LONG_FRAME
    if (dw->radio.proprietary.long_frames)
	return DW1000_FRAME_MAXSIZE;
#else
    (void)dw;
#endif
    return 127;
}


/**
 * @brief Set context for sending frame
 *
 * @details The length, is the total length of the frame (including
 *          the 2-byte CRC)
 *
 * @note In standard mode length can be up to 127 bytes,
 *       in proprietary long-frame-mode length can be up to 1023 bytes.
 *       An out of range length is clamped (and asserted on, where the
 *       port keeps asserts): the standard PHR cannot carry more than
 *       127 (UM §7.2.10, §3.4).
 *
 * @pre   @p dw1000_configure() has been called: the preamble, PRF and
 *        bitrate it cached in the TX_FCTRL base are written out again
 *        by this function.
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
 * @warning Errata 1.4 §3.2 (RX-1): writing here past index 127 and then
 *        issuing a send, while a double buffered frame sits unread in
 *        the second receive buffer, corrupts that frame's 129th octet.
 *        This function does NOT enforce it, having no way to refuse: it
 *        returns void, and it cannot know a send will follow. The send
 *        functions of <dw1000/dw1000_send.h> do refuse it. A caller
 *        driving the transmitter by hand, through this and
 *        @p dw1000_tx_fctrl() and @p dw1000_tx_start(), owns the
 *        constraint itself. It cannot arise without proprietary long
 *        frames, @p dw1000_tx_get_frame_maxsize() being 127 otherwise.
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
 * @note   This does not check that the programmed time is far enough ahead:
 *         doing so would cost a SYS_TIME read on the critical path.
 *         @p dw1000_txrx_set_time() says what "far enough" means; a time
 *         that is not comes back as HPDWARN, and returns -1.
 *
 * @param dw         driver context
 * @param tx_mode    a set of the following flags are supported:
 *                   DW1000_TX_DELAYED_START, DW1000_TX_RESPONSE_EXPECTED,
 *                   DW1000_TX_NO_AUTO_CRC
 *
 * @retval  0        Transmission started
 * @retval <0        Refused; one of @ref TxError.
 *                   @p DW1000_TX_ERR_BUSY when a previous transmission
 *                   is still on the air (one that is over is consumed
 *                   here, one that never began is cleared first, its
 *                   tx_dropped callback run), and, with
 *                   @p DW1000_TX_DELAYED_START,
 *                   @p DW1000_TX_ERR_TOO_LATE when the chip found the
 *                   programmed time already past.
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
    return _dw1000_w4r(dw);
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
    // Acknowledging the completion is what releases the transmitter for
    // the next frame, for a host that polls instead of calling
    // dw1000_process_events(). Without this a polling host would see
    // every send after its first refused.
    _dw1000_tx_done_state(dw);

    // Trigger clearing of the TX events by setting them to 1
    // UM §7.2.17: System Event Status Register
    //
    // All of them, not TXFRS alone: TXFRB, TXPRS and TXPHS are
    // "automatically cleared at the next transmitter enable", and a
    // TXSTRT the chip drops (into a listening receiver, or under Errata
    // TX-1) is not one. Left standing, any one of them is read by
    // _dw1000_tx_dropped() as proof the next send is on the air, so the
    // drop is never found out and every send after it is refused
    // DW1000_TX_ERR_BUSY. AAT goes with them, as it does in the TXFRS
    // branch of dw1000_process_events().
    _dw1000_reg_write32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE,
		       DW1000_MSK_SYS_STATUS_ALL_TX);
}


/**
 * @brief Get the airtime preceding the RMARKER (Ton)
 *
 * @details The preamble and SFD of a frame are transmitted before the
 *          RMARKER, which is what a delayed send programs through
 *          @p dw1000_txrx_set_time(). The chip must therefore already be
 *          transmitting this much earlier than the programmed time
 *          (APS022 §5.4), so a delayed send commanded with a shorter
 *          lead than this cannot be honoured, whatever the host does.
 *
 * @note   Depends only on the preamble length, the SFD length and the
 *         PRF, so it changes with @p dw1000_configure() and not from
 *         frame to frame. Reads 0 before the first configuration.
 * @note   About 138 us at a 128 symbol preamble, 1.05 ms at 1024 and
 *         4.2 ms at 4096: a lead chosen without it is only safe for
 *         short preambles.
 *
 * @param[in]  dw        driver context
 *
 * @return airtime of the preamble and SFD, in @p DW1000_TIME_CLOCK_HZ
 *         ticks, as a lead time to compare or add to a delay
 */
static inline uint32_t dw1000_tx_get_preamble_airtime(dw1000_t *dw)
{
    return dw->tx_ton;
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
 * @note  This is not a hard deadline. UM 2.18 §7.2.40.9: "during the PTO
 *        timeout period occasionally in certain circumstances preamble
 *        may be detected and not confirmed. In this case the PTO
 *        countdown will be suspended (delaying the timeout) for a
 *        minimum of 1 PAC + 32 symbol times, but perhaps longer if
 *        preamble detections continue." The manual therefore advises
 *        backing it with the SFD timeout (@p dw1000_radio_t::sfd_timeout,
 *        set at configuration) and the frame wait timeout
 *        (@p dw1000_rx_set_timeout()), both of which do bound the wait.
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
 * @pre    The DW1000 is in IDLE state: UM §7.2.14 requires RX_FWTO to
 *         be written only while the receiver is off.
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
 * @pre   With @p DW1000_RX_DELAYED_START, the reception time has been
 *        programmed with @p dw1000_txrx_set_time() (DX_TIME, UM §3.3):
 *        this function only arms RXDLYE, it does not set the time.
 *
 * @param dw        driver context
 * @param rx_mode   Receiving mode
 *                   - @p DW1000_RX_IDLE_ON_DELAY_ERROR
 *                   - @p DW1000_RX_DELAYED_START
 * @retval  0        Reception started
 * @retval  1        Reception started, but delayed start was not
 *                   respected.
 * @retval  0        Reception started; or, with a transmission on the
 *                   air, recorded: the receiver comes up at the
 *                   completion's dw1000_process_events(), nothing
 *                   written now, whatever the policy.
 * @retval <0        Refused, one of @ref RxError:
 *                   @p DW1000_RX_ERR_TOO_LATE when
 *                   @p DW1000_RX_DELAYED_START and
 *                   @p DW1000_RX_IDLE_ON_DELAY_ERROR are both set and
 *                   the programmed time had passed.
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
	const uint32_t msk = dw->radio.proprietary.long_frames
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
 * @note   In double buffered mode @p max_noise is the LDE_THRESH value
 *         sampled by @p dw1000_process_events() for the frame being
 *         reported, not a live read: LDE_THRESH is not part of the
 *         double buffered swinging set, and the receiver has been
 *         re-enabled by the time the rx_ok callback runs.
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
 * @note  Both are set to -INFINITY when the preamble accumulation count
 *        is 0, which leaves no usable estimate (a frame shorter than the
 *        SFD adjustment, or a failed SPI read).
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
    // RX_TTCKI reads 0 before the first frame has been demodulated, and
    // a failed SPI read zeroes the buffer too (the OSAL contract). Both
    // would give +/-inf or NaN, which the caller cannot tell from a
    // reading; report no drift instead, as the power estimate does for
    // the analogous case.
    if (interval == 0)
	return 0.0;
    return (double)offset / (double)interval;
}


/*===========================================================================*/
/* Diagnostics and temperature corrections                                   */
/*===========================================================================*/

#if DW1000_WITH_EVENT_COUNTERS || defined(__DOXYGEN__)

/**
 * @brief The chip's own tally of what the receiver and transmitter saw
 *
 * @details Every field is a 12-bit count (UM §7.2.48). A counter wraps
 *          at 4096 rather than saturating, nothing being carried above
 *          bit 11, so a reading is unambiguous only while fewer than
 *          4096 of its event have happened since the last clear. The
 *          manual does not say so; DW1000.md does.
 */
typedef struct {
    uint16_t phe;	/**< PHY header errors                          */
    uint16_t rse;	/**< Reed-Solomon errors (frame sync loss)      */
    uint16_t fcg;	/**< frames received with a good FCS            */
    uint16_t fce;	/**< frames received with a bad FCS             */
    uint16_t ffr;	/**< frames the frame filter rejected           */
    uint16_t ovr;	/**< receiver overruns                          */
    uint16_t sto;	/**< SFD timeouts                               */
    uint16_t pto;	/**< preamble detection timeouts                */
    uint16_t fwto;	/**< frame wait timeouts                        */
    uint16_t txfs;	/**< frames sent                                */
    uint16_t hpw;	/**< half period warnings                       */
    uint16_t tpw;	/**< transmitter power-up warnings              */
} dw1000_event_counters_t;

/**
 * @brief Start counting events
 *
 * @note  There is no matching stop. UM §7.2.48.1 gives the control
 *        register no disable, only an enable and a clear, so once
 *        started the counters run until the chip is reset.
 *        @p dw1000_event_counters_clear() is the only way back to zero.
 *
 * @param[in]  dw       driver context
 */
void dw1000_event_counters_start(dw1000_t *dw);

/**
 * @brief Zero the counters, and leave them counting
 *
 * @param[in]  dw       driver context
 */
void dw1000_event_counters_clear(dw1000_t *dw);

/**
 * @brief Read all twelve counters
 *
 * @note  One SPI transaction, so the twelve values agree on when they
 *        were sampled. Reading does not clear them.
 *
 * @param[in]  dw       driver context
 * @param[out] evc      where to put the counts
 */
void dw1000_event_counters_read(dw1000_t *dw, dw1000_event_counters_t *evc);

#endif


#if DW1000_WITH_ACCUMULATOR || defined(__DOXYGEN__)

/**
 * @brief Read part of the accumulator (the channel impulse response)
 *
 * @details The CIR the leading edge estimate was made from: complex
 *          samples, a 16-bit real then a 16-bit imaginary part for each
 *          tap, four octets to a tap, one tap to a nanosecond. The span
 *          is one symbol, 992 taps at 16MHz PRF and 1016 at 64MHz
 *          (UM §7.2.38), so the whole accumulator is far larger than
 *          most hosts want to hold and this reads a window of it.
 *
 * @note    The chip emits a dummy octet at the head of every
 *          accumulator read, whatever sub-index it starts at. That
 *          octet is dropped here, so @p data[0] is the octet at
 *          @p index; it does cost one byte of @p size, which is why the
 *          return value is one less than the size asked for.
 *
 * @warning Only meaningful between a received frame and the next
 *          receiver enable: the accumulator is overwritten by the next
 *          reception.
 *
 * @param[in]  dw       driver context
 * @param[in]  index    first accumulator octet wanted,
 *                      0 to @p DW1000_LEN_ACC_MEM - 1
 * @param[out] data     buffer of @p size octets
 * @param[in]  size     size of @p data, at least 2
 *
 * @return  octets of accumulator placed in @p data, or 0 if @p index is
 *          past the end of the accumulator or @p size is below 2
 */
size_t dw1000_rx_read_accumulator(dw1000_t *dw, uint16_t index,
				  uint8_t *data, size_t size);

#endif


#if DW1000_WITH_TEMP_COMPENSATION || defined(__DOXYGEN__)

/**
 * @brief Correct a transmit power setting for a change in die temperature
 *
 * @details Transmit power droops as the die warms, near enough linearly:
 *          0.035 dB/°C on channel 2 and 0.065 dB/°C on channel 5
 *          (APS023 part 2 §5.3). This is arithmetic on a register
 *          value: it touches neither the chip nor @p dw, and is safe to
 *          call at any time. @p dw1000_tx_set_power() applies the
 *          result.
 *
 * @note    @p delta_temp is in hundredths of a degree, the unit
 *          @p dw1000_read_temp_vbat() reports, so the difference of two
 *          of its readings goes in unconverted.
 *
 * @note    Each of the register's four settings is moved on its own, and
 *          one left at zero is left alone. The result is clamped to the
 *          part's range rather than allowed to wrap.
 *
 * @note    On channels other than 2 and 5 the reference is returned
 *          unchanged, no slope being published for them.
 *
 * @param[in]  dw          driver context, read for the configured channel
 * @param[in]  txpower     TX_POWER register value as measured at the
 *                         reference temperature, from
 *                         @p dw1000_tx_get_power()
 * @param[in]  delta_temp  current temperature minus the temperature the
 *                         reference was taken at, in 1/100 °C
 *
 * @return the corrected TX_POWER register value
 */
uint32_t dw1000_tx_power_temp_correction(dw1000_t *dw, uint32_t txpower,
					 int16_t delta_temp);

/**
 * @brief Set the transmit power
 *
 * @details Writes TX_POWER and records it, so that
 *          @p dw1000_tx_get_power() keeps agreeing with the chip.
 *
 * @warning The value is not validated: it is a register value, not a
 *          level in dB, and the only supported way to arrive at one is
 *          @p dw1000_tx_get_power() possibly through
 *          @p dw1000_tx_power_temp_correction(). A reconfigure
 *          overwrites it.
 *
 * @param[in]  dw       driver context
 * @param[in]  txpower  TX_POWER register value
 */
void dw1000_tx_set_power(dw1000_t *dw, uint32_t txpower);

/**
 * @brief Measure the pulse generator count for a given delay
 *
 * @details The reference half of the bandwidth correction. Taken once,
 *          when the board is calibrated and its temperature known, and
 *          kept; @p dw1000_tx_calibrate_pg_delay() later searches for
 *          the delay that reproduces this count at another temperature.
 *          Averaged over ten measurements, the count being noisy.
 *
 * @pre     The transceiver must be idle. The measurement stops the
 *          packet sequencer and drives the analog blocks by hand,
 *          putting everything back afterwards.
 *
 * @param[in]  dw        driver context
 * @param[in]  pg_delay  the TC_PGDELAY value to measure, normally the
 *                       one the configured channel uses
 *
 * @return the averaged pulse generator count
 */
uint16_t dw1000_tx_get_pg_count(dw1000_t *dw, uint8_t pg_delay);

/**
 * @brief Search for the pulse generator delay that restores the bandwidth
 *
 * @details Channel bandwidth drifts with die temperature; this finds the
 *          TC_PGDELAY that brings the pulse generator count back to the
 *          reference @p dw1000_tx_get_pg_count() recorded. A binary
 *          search over seven bits, so seven measurements.
 *
 * @pre     The transceiver must be idle, as for
 *          @p dw1000_tx_get_pg_count().
 *
 * @note    The result is not applied. Write it with
 *          @p _dw1000_reg_write8() to TC_PGDELAY, or hold it until the
 *          next configure.
 *
 * @param[in]  dw            driver context
 * @param[in]  target_count  the reference count to search towards
 *
 * @return the best TC_PGDELAY found, or 0 if no setting came within 300
 *         counts of @p target_count
 */
uint8_t dw1000_tx_calibrate_pg_delay(dw1000_t *dw, uint16_t target_count);

#endif

/** @} */

#endif
