/*
 * Copyright (c) 2024-2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The DW1000 register model.
 *
 * A SPI transfer from the driver is decoded here into a register access,
 * and the registers that mean something (SYS_CTRL, SYS_CFG,
 * SYS_STATUS, SYS_MASK, the TX and RX buffers and their timestamps)
 * are given the behaviour the chip has: writing TXSTRT sends a frame to
 * the medium server, an incoming frame fills RX_BUFFER and sets the RX
 * status bits, and any status bit that the mask lets through raises the
 * IRQ line.
 *
 * The model also keeps time, which is what the rest of it is built on:
 * SYS_TIME free-runs off the host clock, a delayed send or receive waits
 * for the moment programmed into DX_TIME, and the receive timeouts of
 * RX_FWTO and DRX_PRETOC expire on their own. That needs a thread, which
 * is the one thing here a bare register array would not have had.
 *
 * Three threads reach this file, and they are not interchangeable:
 *
 *  - the host's, through the SPI and IO line calls;
 *  - the rsvc reader's, delivering frames from the medium server;
 *  - the model's own deadline thread.
 *
 * Every one of them takes e->mutex for the registers, and none of them
 * holds it across the line callback: that call lands in code this port
 * does not own, and a node is entitled to reach for the driver from it.
 *
 * What the model does not do is in port/emulation/README.md, which is also the
 * contract with the medium server.
 *
 * Originally written as a spank OSAL port; it has no dependency on spank.
 */


#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include <pthread.h>
#include <time.h>
#include <sys/queue.h>

#include "dw1000/osal.h"
#include "dw1000/dw1000.h"
#include "dw1000/dw1000_reg.h"
#include "dw1000/dw1000_bswap.h"
#include "dw1000/emulation.h"
#include "rsvc.h"
#include "emu_log.h"
#include "emulation.h"


/* What a deadline, once reached, makes the model do. One slot each, so
 * that arming a second receive timeout replaces the first rather than
 * queueing behind it, which is what the chip does, the timeouts being
 * counters and not a list.
 */
#define E_DEADLINE_TX		0	/* a delayed send comes due     */
#define E_DEADLINE_RX		1	/* a delayed receive turns on   */
#define E_DEADLINE_RXTO		2	/* a receive timeout expires    */
#define E_DEADLINE_COUNT	3

struct e_deadline {
    bool	armed;
    uint64_t	at;		/* device time, 64 bits (see
				 * e_clock_forward): which lap of the
				 * 40-bit counter is meant, not just
				 * where in it */
    uint64_t	arg;		/* E_DEADLINE_RXTO: the status flag */
    uint64_t	seq;		/* bumped on every arm; see below   */
};

struct dw1000_emulation {
    bool drop_next_start;               /* dw1000_emulation_drop_next_start() */
    pthread_mutex_t	mutex;
    rsvc_t 		*rsvc;
    void               (*line_cb)(int line, void *args);
    void                *line_args;
    int 		state;
    bool                irq;

    /* The double receive buffer of UM 4.3. `dblbuff` is DIS_DRXB
     * inverted, tracked as the host writes SYS_CFG; `rbp_host` moves on
     * the HRBPT command and `rbp_ic` on each frame received with a good
     * CRC, and both are mirrored into the HSRBP and ICRBP status bits
     * where a host reads them. `pending` counts frames the IC has put
     * in a buffer that the host has not released yet: at two, both
     * buffers are full and the next frame is an overrun.
     */
    bool		dblbuff;
    int			rbp_host;
    int			rbp_ic;
    unsigned		pending;

    /* Set the first time the medium server cannot be reached. A running
     * simulation ends by taking the server down while its nodes are
     * still going, so this is the ordinary end of a run: it is said once
     * and then the model goes quiet, rather than a line per frame for
     * however long the node takes to notice it is over.
     */
    bool		medium_gone;

    /* The deadline thread. The chip meets a programmed time by counting
     * its own clock; the model has no tick to count, so it sleeps until
     * the host clock says the moment has come. One thread serves every
     * deadline: at most one send and one receive are ever outstanding.
     */
    pthread_t		timer;
    pthread_cond_t	timer_cond;
    bool		timer_running;
    bool		timer_stop;
    struct e_deadline	deadline[E_DEADLINE_COUNT];
    uint64_t		deadline_seq;

    /* A delayed send, held between the TXDLYS command and the moment it
     * goes out. The frame is taken from TX_BUFFER when the command is
     * issued rather than when it is sent, which is what the host is told
     * to expect: it may not touch the buffer while a send is pending.
     */
    struct dw1000_driver_iopkt	tx_pkt;
    size_t			tx_pktlen;
    bool			tx_delayed;
    uint64_t			tx_rawst;	/* RMARKER, from DX_TIME  */
    uint64_t			tx_start;	/* ... less Ton, 64 bits  */
    
    struct e_register   reg[DW1000_COUNT_REGISTERS];

    E_DEFINE_REGISTER_SINGLE(DEV_ID    );
    E_DEFINE_REGISTER_SINGLE(EUI       );
    E_DEFINE_REGISTER_SINGLE(PANADR    );
    E_DEFINE_REGISTER_SINGLE(SYS_CFG   );
    E_DEFINE_REGISTER_SINGLE(SYS_TIME  );
    E_DEFINE_REGISTER_SINGLE(TX_FCTRL  );
    E_DEFINE_REGISTER_SINGLE(TX_BUFFER );
    E_DEFINE_REGISTER_SINGLE(DX_TIME   );
    E_DEFINE_REGISTER_SINGLE(RX_FWTO   );
    E_DEFINE_REGISTER_SINGLE(SYS_CTRL  );
    E_DEFINE_REGISTER_SINGLE(SYS_MASK  );
    E_DEFINE_REGISTER_DOUBLE(SYS_STATUS);
    E_DEFINE_REGISTER_DOUBLE(RX_FINFO  );
    E_DEFINE_REGISTER_DOUBLE(RX_BUFFER );
    E_DEFINE_REGISTER_DOUBLE(RX_FQUAL  );
    E_DEFINE_REGISTER_DOUBLE(RX_TTCKI  );
    E_DEFINE_REGISTER_DOUBLE(RX_TTCKO  );
    E_DEFINE_REGISTER_DOUBLE(RX_TIME   );
    E_DEFINE_REGISTER_SINGLE(TX_TIME   );
    E_DEFINE_REGISTER_SINGLE(TX_ANTD   );
    E_DEFINE_REGISTER_SINGLE(SYS_STATE );
    E_DEFINE_REGISTER_SINGLE(ACK_RESP_T);
    E_DEFINE_REGISTER_SINGLE(RX_SNIFF  );
    E_DEFINE_REGISTER_SINGLE(TX_POWER  );
    E_DEFINE_REGISTER_SINGLE(CHAN_CTRL );
    E_DEFINE_REGISTER_SINGLE(USR_SFD   );
    E_DEFINE_REGISTER_SINGLE(AGC_CTRL  );
    E_DEFINE_REGISTER_SINGLE(EXT_SYNC  );
    E_DEFINE_REGISTER_SINGLE(ACC_MEM   );
    E_DEFINE_REGISTER_SINGLE(GPIO_CTRL );
    E_DEFINE_REGISTER_SINGLE(DRX_CONF  );
    E_DEFINE_REGISTER_SINGLE(RF_CONF   );
    E_DEFINE_REGISTER_SINGLE(TX_CAL    );
    E_DEFINE_REGISTER_SINGLE(FS_CTRL   );
    E_DEFINE_REGISTER_SINGLE(AON       );
    E_DEFINE_REGISTER_SINGLE(OTP_IF    );
    E_DEFINE_REGISTER_SINGLE(LDE_IF    );
    E_DEFINE_REGISTER_SINGLE(DIG_DIAG  );
    E_DEFINE_REGISTER_SINGLE(PMSC      );
};



#define TRACE_ENTER(fmt, ...)						\
    EMU_DEBUG("--> %s" fmt, __FUNCTION__, __VA_ARGS__);

#define TRACE_LEAVE() EMU_DEBUG("<-- %s", __func__);


/* The interrupt line, in two halves. e_irq_update() recomputes IRQS from
 * SYS_STATUS and SYS_MASK and reports whether the line just went from low
 * to high; it touches registers, so it runs with the model's mutex held.
 * e_irq_fire() is the call out to whoever owns the line, and runs with the
 * mutex *released*: it lands in code this port does not own, and holding a
 * non-recursive lock across a callback into a driver is how the caller
 * ends up deadlocked against its own SPI transfer.
 */
static bool e_irq_update(struct dw1000_emulation *e);
static void e_irq_fire(struct dw1000_emulation *e);


E_DEFINE_PASSTHROUGH(SYS_STATUS, 0xff, 0x1b, 0xff, 0xff, 0xff);

/* SYS_STATUS is write-one-to-clear, except where it is not. UM 7.2.17
 * calls five of its bits READ ONLY, each maintained by the chip and each
 * cleared by the chip on its own terms:
 *
 *   IRQS    (0)  "cannot be cleared or overwritten"; it is the OR of the
 *                masked status bits and nothing else.
 *   HPDWARN (27) "will clear when the delayed TX/RX is cancelled or when
 *                the delay remaining is no longer greater than half a
 *                period of the system clock".
 *   HSRBP   (30) reports which set the host is on; moved by the HRBPT
 *   ICRBP   (31) command and by the receiver, not by writing the bit.
 *   TXPUTE  (34) "will clear as soon as the DW1000 begins to send
 *                preamble, (or if the DW1000 is returned to idle)".
 *
 * Byte 0 bit 0, byte 3 bits 3, 6 and 7, byte 4 bit 2.
 */
E_DEFINE_READONLY(SYS_STATUS, 0x01, 0x00, 0x00, 0xc8, 0x04);



E_DEFINE_ACCESS(DEV_ID,     E_ACCESS(   4,  RO  ));
E_DEFINE_ACCESS(EUI,        E_ACCESS(   8,  RW  ));
E_DEFINE_ACCESS(PANADR,     E_ACCESS(   4,  RW  ));
E_DEFINE_ACCESS(SYS_CFG,    E_ACCESS(   4,  RW  ));
E_DEFINE_ACCESS(SYS_TIME,   E_ACCESS(   5,  RW  ));
E_DEFINE_ACCESS(TX_FCTRL,   E_ACCESS(   5,  RW  ));
E_DEFINE_ACCESS(TX_BUFFER,  E_ACCESS(1024,  WO  ));
E_DEFINE_ACCESS(DX_TIME,    E_ACCESS(   5,  RW  ));
E_DEFINE_ACCESS(RX_FWTO,    E_ACCESS(   2,  RW  ));
E_DEFINE_ACCESS(SYS_CTRL,   E_ACCESS(   4, SRW  ));
E_DEFINE_ACCESS(SYS_MASK,   E_ACCESS(   4,  RW  ));
E_DEFINE_ACCESS(SYS_STATUS, E_ACCESS(   5, RWCLR));
E_DEFINE_ACCESS(RX_FINFO,   E_ACCESS(   4,  ROD ));
E_DEFINE_ACCESS(RX_BUFFER,  E_ACCESS(1024,  ROD ));
E_DEFINE_ACCESS(RX_FQUAL,   E_ACCESS(   8,  ROD ));
E_DEFINE_ACCESS(RX_TTCKI,   E_ACCESS(   4,  ROD ));
E_DEFINE_ACCESS(RX_TTCKO,   E_ACCESS(   5,  ROD ));
E_DEFINE_ACCESS(RX_TIME,    E_ACCESS(  14,  ROD ));
E_DEFINE_ACCESS(TX_TIME,    E_ACCESS(  10,  RO  ));
E_DEFINE_ACCESS(TX_ANTD,    E_ACCESS(   2,  RW  ));
E_DEFINE_ACCESS(SYS_STATE,  E_ACCESS(   5,  RO  ));
E_DEFINE_ACCESS(ACK_RESP_T, E_ACCESS(   4,  RW  ));
E_DEFINE_ACCESS(RX_SNIFF,   E_ACCESS(   4,  RW  ));
E_DEFINE_ACCESS(TX_POWER,   E_ACCESS(   4,  RW  ));
E_DEFINE_ACCESS(CHAN_CTRL,  E_ACCESS(   4,  RW  ));
E_DEFINE_ACCESS(USR_SFD,    E_ACCESS(  41,  RW  ));
E_DEFINE_ACCESS(AGC_CTRL,   E_ACCESS(   2, NONE ),  // 0x00 -
		            E_ACCESS(   2,  RW  ),  // 0x02 AGC_CTRL1
		            E_ACCESS(   2,  RW  ),  // 0x04 AGC_TUNE1
		            E_ACCESS(   6, NONE ),  // 0x06 -
                            E_ACCESS(   4,  RW  ),  // 0x0C AGC_TUNE2
		            E_ACCESS(   2, NONE ),  // 0x10 -
		            E_ACCESS(   2,  RW  ),  // 0x12 AGC_TUNE3
		            E_ACCESS(  10, NONE ),  // 0x14 -
		            E_ACCESS(   3,  RO  )); // 0x1E AGC_STAT1
E_DEFINE_ACCESS(EXT_SYNC,   E_ACCESS(   4,  RW  ),  // 0x00 EC_CTRL
		            E_ACCESS(   4,  RO  ),  // 0x04 EC_RXTC
		            E_ACCESS(   4,  RO  )); // 0x08 EC_GOLP
E_DEFINE_ACCESS(ACC_MEM,    E_ACCESS(4064,  RO  ));
E_DEFINE_ACCESS(GPIO_CTRL,  E_ACCESS(   4,  RW  ),  // 0x00 GPIO_MODE
		            E_ACCESS(   4, NONE ),  // 0x04 - (Reserved)
		            E_ACCESS(   4,  RW  ),  // 0x08 GPIO_DIR
		            E_ACCESS(   4,  RW  ),  // 0x0C GPIO_DOUT
		            E_ACCESS(   4,  RW  ),  // 0x10 GPIO_IRQE
		            E_ACCESS(   4,  RW  ),  // 0x14 GPIO_ISEN
		            E_ACCESS(   4,  RW  ),  // 0x18 GPIO_IMODE
		            E_ACCESS(   4,  RW  ),  // 0x1C GPIO_IBES
		            E_ACCESS(   4,  RW  ),  // 0x20 GPIO_ICLR
		            E_ACCESS(   4,  RW  ),  // 0x24 GPIO_IDBE
		            E_ACCESS(   4,  RW  )); // 0x28 GPIO_RAW
E_DEFINE_ACCESS(DRX_CONF,   E_ACCESS(   2, NONE ),  // 0x00 -
		            E_ACCESS(   2,  RW  ),  // 0x02 DRX_TUNE0b
		            E_ACCESS(   2,  RW  ),  // 0x04 DRX_TUNE1a
		            E_ACCESS(   2,  RW  ),  // 0x06 DRX_TUNE1b
		            E_ACCESS(   4,  RW  ),  // 0x08 DRX_TUNE2
		            E_ACCESS(  20, NONE ),  // 0x0C - (Reserved)
		            E_ACCESS(   2,  RW  ),  // 0x20 DRX_SFDTOC
		            E_ACCESS(   2, NONE ),  // 0x22 Reserved
		            E_ACCESS(   2,  RW  ),  // 0x24 DRX_PRETOC
		            E_ACCESS(   2,  RW  ),  // 0x26 DRX_TUNE4H
		            E_ACCESS(   2,  RO  ),  // 0x28 DRX_CAR_INT
		            E_ACCESS(   2, NONE ),  // 0x2A - (Undocumented)
		            E_ACCESS(   2,  RO  )); // 0x2C RXPACC_NOSAT
E_DEFINE_ACCESS(RF_CONF,    E_ACCESS(   4,  RW  ),  // 0x00 RF_CONF
		            E_ACCESS(   7, NONE ),  // 0x04 RF_RES1
		            E_ACCESS(   1,  RW  ),  // 0x0B RF_RXCTRLH
		            E_ACCESS(   3,  RW  ),  // 0x0C RF_TXCTRL
		            E_ACCESS(   1, UNDEF),  // 0x0F - (Undocumented)
		            E_ACCESS(  28,  RW  ),  // 0x10 RF_RES2
		            E_ACCESS(   4,  RO  ),  // 0x2C RF_STATUS
		            E_ACCESS(   5,  RW  )); // 0x30 LDOTUNE
E_DEFINE_ACCESS(TX_CAL,     E_ACCESS(   2,  RW  ),  // 0x00 TC_SARC
		            E_ACCESS(   1, NONE ),  // 0x02 - (Undocumented)
		            E_ACCESS(   3,  RO  ),  // 0x03 TC_SARL
		            E_ACCESS(   2,  RO  ),  // 0x06 TC_SARW
		            E_ACCESS(   1,  RW  ),  // 0x08 TC_PG_CTRL
		            E_ACCESS(   2,  RO  ),  // 0x09 TC_PG_STATUS
		            E_ACCESS(   1,  RW  ),  // 0x0B TC_PGDELAY
		            E_ACCESS(   1,  RW  )); // 0x0C TC_PGTEST
E_DEFINE_ACCESS(FS_CTRL,    E_ACCESS(   7, NONE ),  // 0x00 FS_RES1
		            E_ACCESS(   4,  RW  ),  // 0x07 FS_PLLCFG
		            E_ACCESS(   1,  RW  ),  // 0x0B FS_PPLTUNE
		            E_ACCESS(   2, NONE ),  // 0x0C FS_RES2
		            E_ACCESS(   1,  RW  ),  // 0x0E FS_XTALT
			    E_ACCESS(   6, NONE )); // 0x0F FS_RES3
E_DEFINE_ACCESS(AON,        E_ACCESS(   2,  RW  ),  // 0x00 AON_WCFG
		            E_ACCESS(   1,  RW  ),  // 0x02 AON_CTRL
		            E_ACCESS(   1,  RW  ),  // 0x03 AON_RDAT
		            E_ACCESS(   1,  RW  ),  // 0x04 AON_ADDR
		            E_ACCESS(   1, NONE ),  // 0x05 - (Undocumented)
		            E_ACCESS(   4,  RW  ),  // 0x06 AON_CFG0
			    E_ACCESS(   2,  RW  )); // 0x0A AON_CFG1
E_DEFINE_ACCESS(OTP_IF,     E_ACCESS(   4,  RW  ),  // 0x00 OTP_WDAT
		            E_ACCESS(   2,  RW  ),  // 0x04 OTP_ADDR
		            E_ACCESS(   2,  RW  ),  // 0x06 OTP_CTRL
		            E_ACCESS(   2,  RW  ),  // 0x08 OTP_STAT
		            E_ACCESS(   4,  RO  ),  // 0x0A OTP_RDAT
		            E_ACCESS(   4,  RW  ),  // 0x0E OTP_SRDAT
		            E_ACCESS(   1,  RW  )); // 0x12 OTP_SF
E_DEFINE_ACCESS(LDE_IF,     E_ACCESS(   2,  RO  ),  // 0x0000 LDE_THRESH
		            E_ACCESS(2052, NONE ),  // 0x0002 - (Undocumented)
		            E_ACCESS(   1,  RW  ),  // 0x0806 LDE_CFG1
		            E_ACCESS(2041, NONE ),  // 0x0807 - (Undocumented)
		            E_ACCESS(   2,  RO  ),  // 0x1000 LDE_PPINDX
		            E_ACCESS(   2,  RO  ),  // 0x1002 LDE_PPAMPL
		            E_ACCESS(2048, NONE ),  // 0x1004 - (Undocumented)
		            E_ACCESS(   2,  RW  ),  // 0x1804 LDE_RXANTD
		            E_ACCESS(   2,  RW  ),  // 0x1806 LDE_CFG2
		            E_ACCESS(4092, NONE ),  // 0x1808 - (Undocumented)
		            E_ACCESS(   2,  RW  )); // 0x2804 LDE_REPC
E_DEFINE_ACCESS(DIAG,       E_ACCESS(   4, SRW  ),  // 0x00 EVC_CTRL
		            E_ACCESS(   2,  RO  ),  // 0x04 EVC_PHE
		            E_ACCESS(   2,  RO  ),  // 0x06 EVC_RSE
		            E_ACCESS(   2,  RO  ),  // 0x08 EVC_FCG
		            E_ACCESS(   2,  RO  ),  // 0x0A EVC_FCE
		            E_ACCESS(   2,  RO  ),  // 0x0C EVC_FFR
		            E_ACCESS(   2,  RO  ),  // 0x0E EVC_OVR
		            E_ACCESS(   2,  RO  ),  // 0x10 EVC_STO
		            E_ACCESS(   2,  RO  ),  // 0x12 EVC_PTO
		            E_ACCESS(   2,  RO  ),  // 0x14 EVC_FWTO
		            E_ACCESS(   2,  RO  ),  // 0x16 EVC_TXFS
		            E_ACCESS(   2,  RO  ),  // 0x18 EVC_HPW
		            E_ACCESS(   2,  RO  ),  // 0x1A EVC_TPW
		            E_ACCESS(   8, NONE ),  // 0x1C EVC_RES1
		            E_ACCESS(   2,  RW  )); // 0x24 DIAG_TMC
E_DEFINE_ACCESS(PMSC,       E_ACCESS(   4,  RW  ),  // 0x00 PMSC_CTRL0
		            E_ACCESS(   4,  RW  ),  // 0x04 PMSC_CTRL1
		            E_ACCESS(   4, NONE ),  // 0x08 PMSC_RES1
		            E_ACCESS(   1,  RW  ),  // 0x0C PMSC_SNOZT
		            E_ACCESS(   3, NONE ),  // - (Undocumented)
		            E_ACCESS(  22, NONE ),  // 0x10 PMSC_RES2
		            E_ACCESS(   2,  RW  ),  // 0x26 PMSC_TXSEQ
		            E_ACCESS(   4,  RW  )); // 0X28 PMSC_LEDC


    
void rsvc_uwb_handler(rsvc_t *rsvc, uint16_t type,
		      void *data, size_t length, void *args);


/*-- Frame check sequence ----------------------------------------------*/

/* The 802.15.4 FCS, which the chip appends on transmission and checks on
 * reception, and the driver has none of its own.
 * From zephyr lib/crc/crc16_sw.c
 */
static uint16_t
e_crc16_ccitt(const uint8_t *src, size_t len)
{
    uint16_t seed = 0x0000;
    for (; len > 0; len--) {
	uint8_t e, f;

	e = seed ^ *src++;
	f = e ^ (e << 4);
	seed = (seed >> 8) ^ ((uint16_t)f << 8) ^ ((uint16_t)f << 3) ^ ((uint16_t)f >> 4);
    }
    return seed;
}


/*-- The clock ---------------------------------------------------------*/

/* Device time from the host clock.
 *
 * DW1000_TIME_CLOCK_HZ is 63 897 600 000, so the seconds term overflows
 * 64 bits long before the epoch reaches 2026. That is deliberate and
 * harmless: unsigned multiplication wraps modulo 2^64, 2^40 divides
 * 2^64, so masking the wrapped product to 40 bits gives exactly the same
 * answer as reducing the true product. The Ruby server computes
 * `(Time.now.to_r * DW1000_HZ).to_i & MASK` in arbitrary precision and
 * lands on the same number.
 *
 * The sub-second term is folded differently only to keep it in range:
 * DW1000_TIME_CLOCK_HZ / 10^9 is 638976/10^4 exactly, and nanoseconds
 * times 638976 cannot overflow. Truncating there matches the server's
 * floor, since the seconds term is a whole number of ticks.
 */
static uint64_t e_clock_full(void) {
    struct timespec ts;

    if (clock_gettime(CLOCK_REALTIME, &ts) != 0)
	EMU_FATAL("clock_gettime(CLOCK_REALTIME) failed");

    return ((uint64_t)ts.tv_sec * DW1000_TIME_CLOCK_HZ)
	 + ((uint64_t)ts.tv_nsec * 638976ull) / 10000ull;
}

uint64_t dw1000_emulation_clock(void) {
    return e_clock_full() & E_CLOCK_MASK;
}

/* The next moment at which the 40-bit counter will read @p at, as a
 * 64-bit tick count.
 *
 * A programmed time is 40 bits, so on its own it says nothing about
 * which of the counter's 17.2-second laps is meant, and the chip
 * answers that the same way every time: the next one. UM 3.3 makes the
 * consequence explicit, that a host which programs a time just gone by
 * "has to complete almost a whole clock count period before the start
 * time is reached", and that this is what HPDWARN exists to warn about.
 *
 * Deadlines are therefore carried at 64 bits, where that lap is not in
 * doubt. The wider counter has itself wrapped since the epoch, which
 * costs nothing: differences are exact modulo 2^64, so any interval
 * shorter than about three years subtracts correctly.
 */
static uint64_t e_clock_forward(uint64_t now_full, uint64_t at) {
    return now_full + (((at & E_CLOCK_MASK) - (now_full & E_CLOCK_MASK))
		       & E_CLOCK_MASK);
}


/*-- The double receive buffer -----------------------------------------*/

/* Publish the two buffer pointers into SYS_STATUS.
 *
 * HSRBP and ICRBP are bits 30 and 31, which the passthrough mask puts
 * outside the swinging part of SYS_STATUS, so one write reaches both
 * sets and a host reads the same answer whichever set it is on. The
 * mutex must be held.
 */
static void e_dblbuff_publish(struct dw1000_emulation *e) {
    uint32_t sys_status = E_REG_IC_READ32_KEY(e, SYS_STATUS);

    if (e->rbp_host) DW1000_SET_FLG(sys_status, SYS_STATUS_HSRBP);
    else             DW1000_CLR_FLG(sys_status, SYS_STATUS_HSRBP);
    if (e->rbp_ic)   DW1000_SET_FLG(sys_status, SYS_STATUS_ICRBP);
    else             DW1000_CLR_FLG(sys_status, SYS_STATUS_ICRBP);

    E_REG_IC_WRITE32_KEY(e, sys_status, SYS_STATUS);
}

/* The HRBPT command: the host is done with the buffer it holds.
 *
 * UM 4.3.2: "Every time the HRBPT command is issued the HSRBP status bit
 * will toggle", and UM 4.3.5: "The overrun condition and the RXOVRR
 * status bit will be cleared as soon as the host issues the HRBPT
 * command." The mutex must be held.
 */
static void e_dblbuff_toggle_host(struct dw1000_emulation *e) {
    e->rbp_host = !e->rbp_host;
    if (e->pending > 0)
	e->pending--;

    uint32_t sys_status = E_REG_IC_READ32_KEY(e, SYS_STATUS);
    DW1000_CLR_FLG(sys_status, SYS_STATUS_RXOVRR);
    E_REG_IC_WRITE32_KEY(e, sys_status, SYS_STATUS);

    e_dblbuff_publish(e);
    EMU_DEBUG("dblbuff: HRBPT, host now on set %d (ic %d, %u pending)",
	      e->rbp_host, e->rbp_ic, e->pending);
}


/* Does the receiver come back on by itself?
 *
 * UM 5.3.2, the RXAUTR configuration bit. It is what makes the double
 * buffer worth having: without it the host has to re-enable between
 * frames, so a second frame can never arrive while the first is still
 * unread and an overrun cannot happen.
 *
 * Nothing is sent to the medium server when the receiver comes back
 * this way. RX_CONFIG is a request, answered synchronously, and the
 * only place this is decided is inside the rsvc reader's own callback
 * which is the thread that would have to read the reply. The
 * reference server applies no behaviour to RX_CONFIG anyway
 * (port/emulation/README.md), so the model's state is the whole of it.
 */
static bool e_rx_auto_reenable(struct dw1000_emulation *e, bool good) {
    uint32_t sys_cfg = E_REG_IC_READ32_KEY(e, SYS_CFG);

    if (!DW1000_GET_FLG(sys_cfg, SYS_CFG_RXAUTR))
	return false;

    /* UM 7.2.6, RXAUTR, whose two cases are not the same:
     *
     *   (a) Double-buffered mode: After a frame reception event or
     *       failure (except a frame wait timeout), the receiver will
     *       re-enable to receive another frame.
     *   (b) Single-buffered mode: After a frame reception failure
     *       (except a frame wait timeout), the receiver will re-enable
     *       to re-attempt reception.
     *
     * So a *good* frame re-enables the receiver only when there is a
     * second buffer for the next one to go into. Single buffered, the
     * chip stops and waits for the host to read the one buffer out,
     * which is the whole reason double buffering exists.
     *
     * The frame wait timeout is excluded in both, and needs no code
     * here: a timeout returns to idle without asking.
     */
    return e->dblbuff ? true : !good;
}

/* A frame has been taken: the IC moves to the other set.
 *
 * UM 4.3.2: "Reception of a new frame with good CRC will cause the ICRBP
 * bit to increment (or toggle). In the case that a received frame is
 * rejected by frame filtering or bad CRC the ICRBP will not move on and
 * the buffer will be reused for the next incoming frame." So this is
 * called for a good frame and nothing else.
 *
 * Called after the interrupt has been worked out, not before: everything
 * about the frame, its status bits included, belongs to the set the
 * IC was on while writing it.
 */
static void e_dblbuff_advance_ic(struct dw1000_emulation *e) {
    if (!e->dblbuff)
	return;

    e->rbp_ic = !e->rbp_ic;
    e->pending++;
    e_dblbuff_publish(e);
    EMU_DEBUG("dblbuff: frame taken, ic now on set %d (host %d, %u pending)",
	      e->rbp_ic, e->rbp_host, e->pending);
}


/*-- Deadlines ---------------------------------------------------------*/

/* Arm, disarm, and wait. Every one of these needs the model's mutex held
 * by the caller; the thread itself drops it only to run an action.
 */
static void e_deadline_arm(struct dw1000_emulation *e, int which,
			   uint64_t at, uint64_t arg);
static void e_deadline_disarm(struct dw1000_emulation *e, int which);
static void e_deadline_disarm_all(struct dw1000_emulation *e);

#define E_STATE_OFF		0
#define E_STATE_WAKEUP		1
#define E_STATE_INIT		2
#define E_STATE_IDLE		3
#define E_STATE_SLEEP		4
#define E_STATE_DEEPSLEEP	5
#define E_STATE_TX		6
#define E_STATE_RX		7
#define E_STATE_SNOOZE		8
#define E_STATE_TX_WAIT		9	/* TXDLYS armed, DX_TIME not reached */
#define E_STATE_RX_WAIT		10	/* RXDLYE armed, DX_TIME not reached */

static char *e_state[] = {
    [E_STATE_OFF      ] = "OFF",
    [E_STATE_WAKEUP   ] = "WAKEUP",
    [E_STATE_INIT     ] = "INIT",
    [E_STATE_IDLE     ] = "IDLE",
    [E_STATE_SLEEP    ] = "SLEEP",
    [E_STATE_DEEPSLEEP] = "DEEPSLEEP",
    [E_STATE_TX       ] = "TX",
    [E_STATE_RX	      ] = "RX",
    [E_STATE_SNOOZE   ] = "SNOOZE",
    [E_STATE_TX_WAIT  ] = "TX_WAIT",
    [E_STATE_RX_WAIT  ] = "RX_WAIT"
};
    

#define E_GET_STATE_STR(e)						\
    (e_state[E_GET_STATE(e)])

#define E_IS_STATE(e, _state)						\
    ((e)->state == E_STATE_##_state)

#define E_GET_STATE(e)							\
    ((e)->state)

#define E_SET_STATE(e, _state)						\
    do {								\
	EMU_DEBUG("changing state from %s to " #_state " (%s)",	\
		    E_GET_STATE_STR(e), __func__);			\
	(e)->state = E_STATE_##_state;					\
    } while(0)


/*-- Airtime and timing constants --------------------------------------*/

/* One preamble symbol, in device clock ticks.
 *
 * UM table 60: a preamble symbol is 496 chips at 16 MHz PRF and 508 at
 * 64 MHz, the chip rate being 499.2 MHz. The device clock runs at 128
 * times the chip rate, so a symbol is a whole number of ticks and the
 * 993.59 ns and 1017.63 ns of the table are that number rounded for
 * print.
 */
#define E_TICKS_PER_PSYM_16MHZ	(496 * 128)
#define E_TICKS_PER_PSYM_64MHZ	(508 * 128)

/* One RX_FWTO unit, in device clock ticks.
 *
 * UM 7.2.14: "the exact unit is 512 counts of the fundamental 499.2 MHz
 * UWB clock, or 1.026 us". 512 chips times 128 ticks per chip.
 */
#define E_TICKS_PER_FWTO	(512 * 128)

/* The transmitter power-up time.
 *
 * UM 3.3 puts it at "a few microseconds" and gives no number; it is what
 * separates a delayed send that goes out cleanly from one that raises
 * TXPUTE, its preamble truncated while the transmitter comes up. Five
 * microseconds is this model's choice, not the manual's.
 */
#define E_TX_POWERUP_TICKS	((uint64_t)DW1000_USEC_TO_CLOCK(5))


/*-- HPDWARN and TXPUTE ------------------------------------------------*/

/* Both are conditions, not events, and the model must not latch them.
 *
 * UM 7.2.17 on HPDWARN: "READ ONLY. It will clear when the delayed TX/RX
 * is cancelled or when the delay remaining is no longer greater than
 * half a period of the system clock." And on TXPUTE: "READ ONLY. It will
 * clear as soon as the DW1000 begins to send preamble, (or if the DW1000
 * is returned to idle)."
 *
 * So neither is set once and left; each is recomputed from how far the
 * armed operation still is from starting, which is what makes a
 * cancelled operation clear HPDWARN by itself (TRXOFF disarms, nothing
 * is pending, the bit reads zero), and what makes a marginal delay stop
 * warning once the counter has caught up with it.
 *
 * That the deadline is carried at 64 bits is what makes this expressible:
 * "more than half a period away" is a real distance here, where at 40
 * bits it would be indistinguishable from "just behind us".
 *
 * Called before anything reads SYS_STATUS. The mutex must be held.
 */
static void e_status_derived(struct dw1000_emulation *e) {
    uint64_t now      = e_clock_full();
    uint64_t sys_stat = E_REG_IC_READ40_KEY(e, SYS_STATUS);
    bool     hpdwarn  = false;
    bool     txpute   = false;
    int64_t  togo     = 0;
    bool     pending  = false;

    if (E_IS_STATE(e, TX_WAIT) && e->deadline[E_DEADLINE_TX].armed) {
	togo    = (int64_t)(e->tx_start - now);
	pending = true;
    } else if (E_IS_STATE(e, RX_WAIT) && e->deadline[E_DEADLINE_RX].armed) {
	togo    = (int64_t)(e->deadline[E_DEADLINE_RX].at - now);
	pending = true;
    }

    if (pending) {
	// "more than half a period of the system clock away"
	hpdwarn = togo > (int64_t)(1ull << (DW1000_TIME_CLOCK_BITS - 1));

	/* The transmitter is powering up: past the command, not yet at
	 * the start of preamble, and with less than the power-up time to
	 * go. The manual notes a host is unlikely ever to catch this,
	 * the window being a few symbol times; it is just as narrow here.
	 */
	txpute = E_IS_STATE(e, TX_WAIT) && !hpdwarn &&
	         (togo > 0) && ((uint64_t)togo < E_TX_POWERUP_TICKS);
    }

    if (hpdwarn) DW1000_SET_FLG(sys_stat, SYS_STATUS_HPDWARN);
    else         DW1000_CLR_FLG(sys_stat, SYS_STATUS_HPDWARN);
    if (txpute)  DW1000_SET_FLG(sys_stat, SYS_STATUS_TXPUTE);
    else         DW1000_CLR_FLG(sys_stat, SYS_STATUS_TXPUTE);

    E_REG_IC_WRITE40_KEY(e, sys_stat, SYS_STATUS);
}






/*-- The deadline thread -----------------------------------------------*/

/* What a deadline does when it comes due. Called with the model's mutex
 * released, because two of the three end up in a blocking socket call.
 */
static void e_deadline_fire_tx  (struct dw1000_emulation *e, uint64_t seq);
static void e_deadline_fire_rx  (struct dw1000_emulation *e, uint64_t seq);
static void e_deadline_fire_rxto(struct dw1000_emulation *e, uint64_t seq,
				 uint64_t flag);

/* Is the deadline this action was dequeued from still the one that is
 * current? False if the slot has been re-armed since, which means the
 * host cancelled and started again while the action was on its way. The
 * mutex must be held.
 */
static bool e_deadline_current(struct dw1000_emulation *e, int which,
			       uint64_t seq) {
    return !e->deadline[which].armed && (e->deadline[which].seq == seq);
}

/* Arm a deadline. The mutex must be held. Re-arming a slot replaces what
 * was there: the chip's timeouts are counters, not a queue.
 */
static void e_deadline_arm(struct dw1000_emulation *e, int which,
			   uint64_t at, uint64_t arg) {
    DW1000_ASSERT(which >= 0 && which < E_DEADLINE_COUNT,
		  "deadline index in range");
    e->deadline[which].armed = true;
    e->deadline[which].at    = at;
    e->deadline[which].arg   = arg;
    e->deadline[which].seq   = ++e->deadline_seq;
    pthread_cond_signal(&e->timer_cond);
}

static void e_deadline_disarm(struct dw1000_emulation *e, int which) {
    DW1000_ASSERT(which >= 0 && which < E_DEADLINE_COUNT,
		  "deadline index in range");
    e->deadline[which].armed = false;
}

static void e_deadline_disarm_all(struct dw1000_emulation *e) {
    for (int i = 0 ; i < E_DEADLINE_COUNT ; i++)
	e->deadline[i].armed = false;
}

/* The medium server answered a call. Clears the unreachable latch, so
 * that a server which stalls once and recovers is reported once per
 * spell rather than once ever.
 */
static void e_medium_ok(struct dw1000_emulation *e) {
    if (!e->medium_gone)
	return;                 /* the common case, and no lock needed */

    pthread_mutex_lock(&e->mutex);
    e->medium_gone = false;
    pthread_mutex_unlock(&e->mutex);
}

/* The medium server is unreachable.
 *
 * Returns true the first time, so the caller can say so once. Takes the
 * mutex itself: the callers are the paths that talk to the server, and
 * those run with it released.
 */
static bool e_medium_lost(struct dw1000_emulation *e) {
    bool first;

    pthread_mutex_lock(&e->mutex);
    first = !e->medium_gone;
    e->medium_gone = true;


    /* Nothing is on the air any more, and nothing is going to complete.
     * Say so in the model's own state rather than leaving it in TX or RX
     * waiting for a report that cannot arrive, and drop the deadlines
     * with it.
     */
    e_deadline_disarm_all(e);
    E_SET_STATE(e, IDLE);
    pthread_mutex_unlock(&e->mutex);

    return first;
}


/* The earliest armed deadline, or -1. "Earliest" is on the 40-bit clock,
 * so it is the smallest non-negative distance from now and not the
 * smallest number.
 */
static int e_deadline_next(struct dw1000_emulation *e, uint64_t now_full) {
    int     best  = -1;
    int64_t bestd = 0;

    for (int i = 0 ; i < E_DEADLINE_COUNT ; i++) {
	if (!e->deadline[i].armed)
	    continue;
	int64_t d = (int64_t)(e->deadline[i].at - now_full);
	if (best < 0 || d < bestd) {
	    best  = i;
	    bestd = d;
	}
    }
    return best;
}

/* The thread. It owns nothing: it waits for the earliest armed deadline,
 * disarms it, and runs its action with the mutex dropped.
 */
static void *e_timer_thread(void *args) {
    struct dw1000_emulation *e = args;

    pthread_mutex_lock(&e->mutex);
    while (!e->timer_stop) {
	uint64_t now  = e_clock_full();
	int      next = e_deadline_next(e, now);

	if (next < 0) {
	    // Nothing armed: sleep until something is, or until shutdown.
	    pthread_cond_wait(&e->timer_cond, &e->mutex);
	    continue;
	}

	int64_t ticks = (int64_t)(e->deadline[next].at - now);
	if (ticks > 0) {
	    /* Not yet. Sleep on the same clock the deadline is expressed
	     * in (CLOCK_REALTIME, which is what dw1000_emulation_clock()
	     * samples), so that a step of the host clock moves both.
	     */
	    struct timespec until;
	    clock_gettime(CLOCK_REALTIME, &until);
	    uint64_t ns = (uint64_t)((ticks * 10000ull) / 638976ull);
	    until.tv_sec  += (time_t)(ns / 1000000000ull);
	    until.tv_nsec += (long)  (ns % 1000000000ull);
	    if (until.tv_nsec >= 1000000000L) {
		until.tv_nsec -= 1000000000L;
		until.tv_sec  += 1;
	    }
	    pthread_cond_timedwait(&e->timer_cond, &e->mutex, &until);
	    // Re-derive everything: the slot may have been disarmed.
	    continue;
	}

	/* Due. Take it out of the way, and carry its stamp to the action.
	 *
	 * The action has to drop the mutex (two of the three end up in
	 * a blocking socket call), and the host can do anything in that
	 * gap, including a TRXOFF that cancels this operation and a fresh
	 * command that starts another one. Checking the model's state is
	 * not enough to tell those apart: a receive cancelled and
	 * immediately re-enabled is in state RX either way, and the stale
	 * timeout would fire into the new session. The stamp is what
	 * distinguishes them, the slot being restamped on every arm.
	 */
	uint64_t arg = e->deadline[next].arg;
	uint64_t seq = e->deadline[next].seq;
	e_deadline_disarm(e, next);

	pthread_mutex_unlock(&e->mutex);
	switch (next) {
	case E_DEADLINE_TX:   e_deadline_fire_tx  (e, seq);      break;
	case E_DEADLINE_RX:   e_deadline_fire_rx  (e, seq);      break;
	case E_DEADLINE_RXTO: e_deadline_fire_rxto(e, seq, arg); break;
	default:              EMU_FATAL("unknown deadline %d", next);
	}
	pthread_mutex_lock(&e->mutex);
    }
    pthread_mutex_unlock(&e->mutex);

    return NULL;
}


void
e_reg_sanitize(struct dw1000_emulation *e,
	       int idx, size_t length, size_t offset) {
    if (idx >= DW1000_COUNT_REGISTERS) {
	EMU_FATAL("writing beyond register address space");
    }
    struct e_register *reg = &e->reg[idx];
    if (reg->data[0] == NULL || reg->data[1] == NULL) {
	EMU_FATAL("register 0x%02x not allocated", idx);
    }
    if (offset + length > reg->size) {
	EMU_FATAL("register 0x%02x overflow"
		    " (off=%zu + size=%zu > reg-size=%u)",
		    idx, offset, length, reg->size);
    }
}

void e_reg_write(struct dw1000_emulation *e, int idx, size_t offset,
		 uint8_t *data, size_t length, bool system) {
    e_reg_sanitize(e, idx, length, offset);

    struct e_register *reg        = E_REG(e, idx);
    int                side       = E_SWINGSET(e, system);
    uint8_t           *reg_data_c = reg->data[side];
    uint8_t           *reg_data_o = reg->data[(side + 1) % 2];
    size_t             reg_start  = 0;
    size_t             reg_end    = offset + length;
    int                seg_idx    = 0;

    // Find first segment containing the offset
    for (seg_idx = 0 ; reg->access[seg_idx].length ; seg_idx++) {
	if (offset < reg_start + reg->access[seg_idx].length)
	    break;
	reg_start += reg->access[seg_idx].length;
    }

    // Sanity check (should already be catched by e_reg_sanitize())
    if (reg->access[seg_idx].length == 0)
	EMU_FATAL("offset outside register size"
		    " (register 0x%02x)", idx);

    // Check and copy for various segments
    do {
	DW1000_ASSERT(reg->access[seg_idx].length > 0,
		      "register segment has a length");

	// Restrict offset/length to the current segment
	size_t s_offset = EMU_MAX(offset, reg_start);
	size_t s_length = EMU_MIN(reg->access[seg_idx].length,
				  reg_end - s_offset);

	// Access validation
	if (!system && !reg->access[seg_idx].mode.write) {
	    EMU_FATAL("trying to write to read-only part"
			" (register=0x%02x, offset=%zu, length=%zu)",
			idx, s_offset, s_length);
	}

	// COPY or CLEAR
	if (!system && reg->access[seg_idx].mode.clear) {
	    for (size_t i = 0 ; i < s_length ; i++) {
		/* Write-one-to-clear, minus the bits the chip owns: a
		 * host write leaves those exactly as they were.
		 */
		uint8_t bits = data[i];
		if (reg->readonly)
		    bits &= ~reg->readonly[s_offset + i];
		reg_data_c[s_offset + i] &= ~bits;
	    }
	} else {
	    memcpy(&reg_data_c[s_offset], data, s_length);
	}
	// Passthrough as some register have only few bits part of a swing-set
	if (reg->passthrough) {
	    for (size_t i = s_offset ; i < s_offset + s_length ; i++) {
		reg_data_o[i] = (reg_data_o[i] & ~reg->passthrough[i]) |
		                (reg_data_c[i] &  reg->passthrough[i]) ;
	    }
	}

	// Move segment start
	reg_start += reg->access[seg_idx].length;

	// Move to next segment
	seg_idx += 1;
	
	// Reducing data source
	data   += s_length;
	length -= s_length;

	//
	/* The cast is what lets the author's check survive -Wextra: on a
	 * size_t, `>= 0` is always true and gcc says so. The point is
	 * that the running count did not wrap below zero.
	 */
	DW1000_ASSERT((ssize_t)length >= 0,
		      "remaining length did not wrap");
    } while(reg_start < reg_end);
}




void e_reg_read(struct dw1000_emulation *e, int idx,  size_t offset,
		uint8_t *data, size_t length,  bool system) {
    e_reg_sanitize(e, idx, length, offset);


    struct e_register *reg       = E_REG(e, idx);
    uint8_t           *reg_data  = reg->data[E_SWINGSET(e, system)];
    size_t             reg_start = 0;
    size_t             reg_end   = offset + length;
    int                seg_idx   = 0;

    // Find first segment containing the offset
    for (seg_idx = 0 ; reg->access[seg_idx].length ; seg_idx++) {
	if (offset < reg_start + reg->access[seg_idx].length)
	    break;
	reg_start += reg->access[seg_idx].length;
    }

    // Sanity check (should already be catched by e_reg_sanitize())
    if (reg->access[seg_idx].length == 0)
	EMU_FATAL("offset outside register size"
		    " (register 0x%02x)", idx);

    // Check and copy for various segments
    do {
	DW1000_ASSERT(reg->access[seg_idx].length > 0,
		      "register segment has a length");

	// Restrict offset/length to the current segment
	size_t s_offset = EMU_MAX(offset, reg_start);
	size_t s_length = EMU_MIN(reg->access[seg_idx].length,
				  reg_end - s_offset);

	if (system || reg->access[seg_idx].mode.read) {
	    memcpy(data, &reg_data[s_offset], s_length);
	} else {
	    EMU_FATAL("trying to read to write-only part"
			" (register=0x%02x, offset=%zu, length=%zu)",
			idx, s_offset, s_length);
	}

	// Move segment start
	reg_start += reg->access[seg_idx].length;

	// Move to next segment
	seg_idx += 1;
	
	// Reducing data source
	data   += s_length;
	length -= s_length;

	//
	/* The cast is what lets the author's check survive -Wextra: on a
	 * size_t, `>= 0` is always true and gcc says so. The point is
	 * that the running count did not wrap below zero.
	 */
	DW1000_ASSERT((ssize_t)length >= 0,
		      "remaining length did not wrap");
    } while(reg_start < reg_end);
}



struct dw1000_emulation *
dw1000_emulation_create(rsvc_t *rsvc,
			void (*line_cb)(int line, void *args), void *line_args) {
    struct dw1000_emulation *e = calloc(1, sizeof(struct dw1000_emulation));
    if (e == NULL)
	EMU_FATAL("out of memory allocating the emulation");

    pthread_mutex_init(&e->mutex, NULL);
    pthread_cond_init(&e->timer_cond, NULL);

    e->rsvc      = rsvc;
    e->line_cb   = line_cb;
    e->line_args = line_args;
   
    E_SET_STATE(e, OFF);
    
    E_REG_ATTACH(e, SYS_TIME,    SINGLE); // refreshed on every read
    E_REG_ATTACH(e, DX_TIME,     SINGLE); // delayed send and receive
    E_REG_ATTACH(e, RX_FWTO,     SINGLE); // frame wait timeout
    E_REG_ATTACH(e, DEV_ID,      SINGLE);
    E_REG_ATTACH(e, SYS_CFG,     SINGLE); // DIS_DRXB, RXWTOE, RXAUTR
    E_REG_ATTACH(e, TX_FCTRL,    SINGLE); // todo
    E_REG_ATTACH(e, TX_BUFFER,   SINGLE);
    E_REG_ATTACH(e, SYS_CTRL,    SINGLE);
    E_REG_ATTACH(e, SYS_MASK,    SINGLE); 
    E_REG_ATTACH(e, SYS_STATUS,  DOUBLE);
    E_REG_ATTACH(e, RX_FINFO,    DOUBLE);
    E_REG_ATTACH(e, RX_BUFFER,   DOUBLE);
    E_REG_ATTACH(e, RX_FQUAL,    DOUBLE);
    E_REG_ATTACH(e, RX_TTCKI,    DOUBLE);
    E_REG_ATTACH(e, RX_TTCKO,    DOUBLE);
    E_REG_ATTACH(e, RX_TIME,     DOUBLE);
    E_REG_ATTACH(e, TX_TIME,     SINGLE);
    E_REG_ATTACH(e, TX_ANTD,     SINGLE); // todo
    E_REG_ATTACH(e, TX_POWER,    SINGLE); // todo
    E_REG_ATTACH(e, CHAN_CTRL,   SINGLE); // ignored
    E_REG_ATTACH(e, USR_SFD,     SINGLE); // ignored
    E_REG_ATTACH(e, AGC_CTRL,    SINGLE); // ignored
    E_REG_ATTACH(e, EXT_SYNC,    SINGLE); // ignored
    E_REG_ATTACH(e, GPIO_CTRL,   SINGLE);
    E_REG_ATTACH(e, DRX_CONF,    SINGLE); // RXPACC_NOSAT, DRX_PRETOC, DRX_TUNE2
    E_REG_ATTACH(e, RF_CONF,     SINGLE); // ignored
    E_REG_ATTACH(e, TX_CAL,      SINGLE); // #define DW1000_OFF_TC_SARC : SAR_LTEMP / SAR_LVBAT
    E_REG_ATTACH(e, FS_CTRL,     SINGLE); // ignored
    E_REG_ATTACH(e, AON,         SINGLE); // ignored
    E_REG_ATTACH(e, OTP_IF,      SINGLE); // ignored
    E_REG_ATTACH(e, LDE_IF,      SINGLE);
    E_REG_ATTACH(e, PMSC,        SINGLE); // ignored

    E_REG_ATTACH(e, EUI,         TODO);
    E_REG_ATTACH(e, PANADR,      TODO);
    E_REG_ATTACH(e, SYS_STATE,   SINGLE);
    E_REG_ATTACH(e, RX_SNIFF,    TODO);
    E_REG_ATTACH(e, ACK_RESP_T,  TODO);
    E_REG_ATTACH(e, ACC_MEM,     TODO);
    E_REG_ATTACH(e, DIAG,        TODO);
    
    
    e->reg[DW1000_REG_SYS_STATUS].passthrough = reg_p_SYS_STATUS;
    e->reg[DW1000_REG_SYS_STATUS].readonly    = reg_ro_SYS_STATUS;
    
    dw1000_emulation_reset(e);

    if (!rsvc_register(e->rsvc, RSVC_UWB_IO, rsvc_uwb_handler, e)) {
	EMU_FATAL("failed to register RSVC callback");
    }

    /* Last, so that nothing the thread can reach is still being built.
     * It idles on the condition variable until a deadline is armed.
     */
    if (pthread_create(&e->timer, NULL, e_timer_thread, e) != 0)
	EMU_FATAL("failed to start the deadline thread");
    e->timer_running = true;

    return e;
}

void dw1000_emulation_stop(struct dw1000_emulation *e) {
    if ((e == NULL) || !e->timer_running)
	return;

    pthread_mutex_lock(&e->mutex);
    e_deadline_disarm_all(e);
    e->timer_stop = true;
    pthread_cond_signal(&e->timer_cond);
    pthread_mutex_unlock(&e->mutex);

    pthread_join(e->timer, NULL);
    e->timer_running = false;
}

void dw1000_emulation_destroy(struct dw1000_emulation *e) {
    if (e == NULL)
	return;

    /* Step 1, if the caller has not done it. Step 2 is the caller's and
     * cannot be done here: the connection is theirs, and closing it is
     * what joins the thread that would otherwise deliver a frame into
     * the memory freed below.
     *
     * rsvc_unregister() is deliberately not called. It only marks the
     * handler unused, and does not wait for one already dispatched, so
     * it buys nothing that rsvc_close() has not already bought, and
     * calling it after rsvc_close() would be locking a mutex that
     * rsvc_close() has destroyed.
     */
    dw1000_emulation_stop(e);

    pthread_cond_destroy(&e->timer_cond);
    pthread_mutex_destroy(&e->mutex);
    free(e);
}

void dw1000_emulation_drop_next_start(struct dw1000_emulation *e) {
    pthread_mutex_lock(&e->mutex);
    e->drop_next_start = true;
    pthread_mutex_unlock(&e->mutex);
}

void dw1000_emulation_reset(struct dw1000_emulation *e) {
    /* Under the mutex, like every other writer of this state. A host can
     * drive the reset line at any moment, including while the rsvc
     * reader is copying a frame into RX_BUFFER or the deadline thread is
     * part way through an action, and memset-ing every register from
     * underneath either of those is a plain data race. Safe to take
     * during dw1000_emulation_create() too: nothing else exists yet to
     * hold it.
     */
    pthread_mutex_lock(&e->mutex);

    E_SET_STATE(e, IDLE);

    /* Whatever was programmed is gone with the rest. Without this a
     * delayed send armed before the reset would still fire afterwards,
     * into a model that has forgotten it.
     */
    e_deadline_disarm_all(e);
    e->tx_delayed = false;
    e->tx_rawst   = 0;
    e->tx_start   = 0;
    e->tx_pktlen  = 0;

    /* The line is deasserted by the reset, and the model's memory of its
     * level has to go with it: a stale `true` here would swallow the
     * first rising edge after the reset, which is the one that says the
     * chip came back.
     */
    e->irq = false;

    /* Both pointers to set 0 and nothing outstanding. DIS_DRXB is part
     * of the reset SYS_CFG written below, so the model comes up single
     * buffered, which is what the chip does (UM 4.3.1).
     */
    e->dblbuff   = false;
    e->rbp_host = 0;
    e->rbp_ic   = 0;
    e->pending  = 0;

    // Clear everything
    for (int i = 0 ; i < DW1000_COUNT_REGISTERS ; i++) {
	struct e_register *r = &e->reg[i];
	if ((r->data[0] != NULL))
	    memset(r->data[0], 0, r->size);
	if ((r->data[1] != NULL) &&
	    (r->data[1] != r->data[0]))
	    memset(r->data[1], 0, r->size);
    }

    E_REG_IC_WRITE32_KEY(e, 0xDECA0130, DEV_ID);
    
    uint32_t sys_cfg =
	DW1000_FLG_SYS_CFG_DIS_DRXB |
	DW1000_FLG_SYS_CFG_HIRQ_POL ;
    E_REG_IC_WRITE32_KEY(e, sys_cfg, SYS_CFG);

    uint32_t tx_fctrl =
	(127                    << DW1000_SFT_TX_FCTRL_TFLEN  ) |
	(DW1000_BITRATE_850KBPS << DW1000_SFT_TX_FCTRL_TXBR   ) |
	(DW1000_PRF_64MHZ       << DW1000_SFT_TX_FCTRL_TXPRF  ) |
	(DW1000_PLEN_1024       << DW1000_SFT_TX_FCTRL_TXPSR  ) |
	(0                      << DW1000_SFT_TX_FCTRL_TXBOFFS) ;
    E_REG_IC_WRITE32_KEY(e, tx_fctrl, TX_FCTRL);

    uint32_t chan_ctrl =
	(5 << DW1000_SFT_CHAN_CTRL_TX_CHAN) |
	(5 << DW1000_SFT_CHAN_CTRL_RX_CHAN) ;
    E_REG_IC_WRITE32_KEY(e, chan_ctrl, CHAN_CTRL);

    uint32_t gpio_ctrl =
	(DW1000_VAL_GPIO_0_GPIO << DW1000_SFT_GPIO_MSGP0) |
	(DW1000_VAL_GPIO_1_GPIO << DW1000_SFT_GPIO_MSGP1) |
	(DW1000_VAL_GPIO_2_GPIO << DW1000_SFT_GPIO_MSGP2) |
	(DW1000_VAL_GPIO_3_GPIO << DW1000_SFT_GPIO_MSGP3) |
	(DW1000_VAL_GPIO_4_GPIO << DW1000_SFT_GPIO_MSGP4) |
	(DW1000_VAL_GPIO_5_GPIO << DW1000_SFT_GPIO_MSGP5) |
	(DW1000_VAL_GPIO_6_GPIO << DW1000_SFT_GPIO_MSGP6) |
	(DW1000_VAL_GPIO_7_SYNC << DW1000_SFT_GPIO_MSGP7) |
	(DW1000_VAL_GPIO_8_IRQ  << DW1000_SFT_GPIO_MSGP8) ;
    E_REG_IC_WRITE32_KEY(e, gpio_ctrl, GPIO_CTRL);


    uint32_t sys_status = 0;
    DW1000_SET_FLG(sys_status, SYS_STATUS_CPLOCK);

    E_REG_IC_WRITE32_KEY(e, sys_status, SYS_STATUS);

    pthread_mutex_unlock(&e->mutex);
}


/*-- Receive timing ----------------------------------------------------*/

/* PAC size in preamble symbols, recovered from DRX_TUNE2.
 *
 * UM 7.2.40.9 programs DRX_PRETOC in units of PAC size, and UM 7.2.40.5
 * is where the PAC size is actually set, as one of eight opaque tuning
 * words, four per PRF. The model has to go the other way, so it matches
 * the word it was given against the same eight values; a host that wrote
 * something else gets the 8-symbol default and a warning, which is all
 * that can honestly be said about an undocumented value.
 */
static unsigned e_pac_symbols(uint32_t drx_tune2) {
    static const struct { uint32_t tune2; unsigned pac; } known[] = {
	{ 0x311A002D,  8 }, { 0x331A0052, 16 },	  // 16 MHz PRF
	{ 0x351A009A, 32 }, { 0x371A011D, 64 },
	{ 0x313B006B,  8 }, { 0x333B00BE, 16 },	  // 64 MHz PRF
	{ 0x353B015E, 32 }, { 0x373B0296, 64 },
    };
    for (size_t i = 0 ; i < sizeof(known)/sizeof(known[0]) ; i++)
	if (known[i].tune2 == drx_tune2)
	    return known[i].pac;

    EMU_WARNING("DRX_TUNE2 = 0x%08" PRIx32 " is not one of the eight"
		" documented tuning words; assuming a PAC of 8 symbols",
		drx_tune2);
    return 8;
}

/* Ticks per preamble symbol for the configured PRF (CHAN_CTRL.RXPRF).
 */
static uint64_t e_ticks_per_psym(struct dw1000_emulation *e) {
    uint32_t chan_ctrl = E_REG_IC_READ32_KEY(e, CHAN_CTRL);
    return (DW1000_GET_VAL(chan_ctrl, CHAN_CTRL_RXPRF) == DW1000_PRF_64MHZ)
	 ? E_TICKS_PER_PSYM_64MHZ : E_TICKS_PER_PSYM_16MHZ;
}

/* Arm the receive timeout that will expire first, counting from @p from.
 *
 * Two of the chip's three receive timeouts can fire in this model:
 *
 *  - RXPTO, the preamble detection timeout (DRX_PRETOC), which UM
 *    7.2.40.9 starts "as soon as the receiver is enabled to hunt for
 *    preamble" and whose period is (DRX_PRETOC + 1) PACs;
 *  - RXRFTO, the frame wait timeout (RX_FWTO), which UM 7.2.14 starts at
 *    the same moment when RXWTOE is set.
 *
 * RXSFDTO cannot: UM 7.2.40.7 starts it at preamble detection, and this
 * model has no preamble: a frame either arrives whole from the medium
 * server or does not arrive. Not arming it is the honest reading; see
 * port/emulation/README.md.
 *
 * Whichever of the two is nearer is the one that will be reached, and
 * both stop the reception, so a single slot holds them. The mutex must
 * be held.
 */
static void e_rx_arm_timeouts(struct dw1000_emulation *e, uint64_t from_full) {
    uint32_t sys_cfg  = E_REG_IC_READ32_KEY(e, SYS_CFG);
    uint16_t pretoc   = E_REG_IC_READ16_KEY(e, DRX_CONF, DRX_PRETOC);
    uint16_t fwto     = E_REG_IC_READ16_KEY(e, RX_FWTO);

    bool     have     = false;
    uint64_t at       = 0;
    uint64_t flag     = 0;

    // Preamble detection timeout: a programmed zero disables it.
    if (pretoc != 0) {
	uint32_t tune2 = E_REG_IC_READ32_KEY(e, DRX_CONF, DRX_TUNE2);
	uint64_t span  = (uint64_t)(pretoc + 1u)
	               * e_pac_symbols(tune2)
	               * e_ticks_per_psym(e);
	at    = from_full + span;
	flag  = DW1000_FLG_SYS_STATUS_RXPTO;
	have  = true;
    }

    // Frame wait timeout: only when RXWTOE says so.
    if (DW1000_GET_FLG(sys_cfg, SYS_CFG_RXWTOE) && (fwto != 0)) {
	uint64_t span = (uint64_t)fwto * E_TICKS_PER_FWTO;
	uint64_t cand = from_full + span;
	if (!have || (int64_t)(cand - at) < 0) {
	    at   = cand;
	    flag = DW1000_FLG_SYS_STATUS_RXRFTO;
	}
	have = true;
    }

    if (have) {
	EMU_DEBUG("rx: arming %s in %" PRIu64 " ticks",
		  (flag == DW1000_FLG_SYS_STATUS_RXPTO) ? "RXPTO" : "RXRFTO",
		  at - from_full);
	e_deadline_arm(e, E_DEADLINE_RXTO, at, flag);
    } else {
	e_deadline_disarm(e, E_DEADLINE_RXTO);
    }
}


/*-- Receiver enable ---------------------------------------------------*/

/* Put the receiver on the air and tell the medium server. The mutex must
 * NOT be held; the caller has already moved the state.
 */
static int e_rx_engage(struct dw1000_emulation *e) {
    EMU_DEBUG("rx_start: requesting rx start (sending RX_CONFIG packet)");
    struct dw1000_driver_iopkt iopkt = {
	 .drvid           = (uintptr_t) e,
	 .type            = DW1000_RSVC_RX_CONFIG,
	 .rx_config.flags = 0,
    };
    size_t pktlen = DW1000_DRIVER_PKTLEN_RX_CONFIG();

    if (rsvc_i(e->rsvc, RSVC_UWB_IO, &iopkt, pktlen) < 0) {
	/* The server has gone. Not fatal: a node outliving its medium is
	 * how every simulation ends, and aborting there turns an orderly
	 * shutdown into what looks like a crash. The receiver simply
	 * never hears anything again, which is the truthful model of a
	 * radio with nothing on the other end.
	 */
	if (e_medium_lost(e))
	    EMU_WARNING("the medium server is not answering; this node"
			" hears and says nothing until it does");
	return -1;
    }
    e_medium_ok(e);
    EMU_DEBUG("rx_start: done");
    return 0;
}

int dw1000_emulation_recv(struct dw1000_emulation *e) {
    EMU_DEBUG("rx_start: enter (state=%s)", E_GET_STATE_STR(e));
    pthread_mutex_lock(&e->mutex);

    // Auto-clear RX enable flag as command taken into account
    uint32_t sys_ctrl  = E_REG_IC_READ32_KEY(e, SYS_CTRL);
    bool     delayed   = DW1000_GET_FLG(sys_ctrl, SYS_CTRL_RXDLYE);
    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_RXENAB);
    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_RXDLYE);
    E_REG_IC_WRITE32_KEY(e, sys_ctrl, SYS_CTRL);

    /* Sanity check.
     *
     * Not "the model is idle": with RXAUTR the receiver puts itself back
     * on after every frame (UM 5.3.2), so a host that enables it again
     * (which dw1000_rx_start() does on each call) finds it already
     * on, and the chip takes that in its stride. What is not allowed is
     * enabling the receiver on top of a transmission.
     */
    DW1000_ASSERT(! E_IS_STATE(e, TX) && ! E_IS_STATE(e, TX_WAIT),
		  "receiver not enabled during a transmission");
    DW1000_ASSERT(! DW1000_GET_FLG(sys_ctrl, SYS_CTRL_TXSTRT),
		  "receiver enabled with no transmit pending");


    if (delayed) {
	/* UM 4.2: the chip stays idle until SYS_TIME reaches DX_TIME and
	 * only then turns the receiver on, which is also the moment the
	 * receive timeouts start counting (UM 7.2.40.9).
	 */
	uint64_t now = e_clock_full();
	uint64_t dx  = E_REG_IC_READ40_KEY(e, DX_TIME) & ~0x1FFull;
	uint64_t at  = e_clock_forward(now, dx);

	/* UM 4.2 and 7.2.17: a turn-on time already gone by is not an
	 * error the chip acts on. It waits for the counter to come round
	 * to it, "almost a whole clock count period", and HPDWARN is
	 * how the host is told, so that it can "take recovery measures"
	 * if it wants to. Taking them is the host's move, not the
	 * model's, and this driver's move is TRXOFF.
	 *
	 * So nothing is refused here and nothing is latched; the receive
	 * is armed on whichever lap the counter reaches DX_TIME, and
	 * e_status_derived() answers HPDWARN from the distance to it.
	 */
	E_SET_STATE(e, RX_WAIT);
	e_deadline_arm(e, E_DEADLINE_RX, at, 0);
	EMU_DEBUG("rx_start: delayed to 0x%010" PRIx64 " (in %" PRIu64
		  " ticks)", dx, at - now);
	pthread_mutex_unlock(&e->mutex);
	return 0;
    }

    // Mark as RX state
    E_SET_STATE(e, RX);
    e_rx_arm_timeouts(e, e_clock_full());

    pthread_mutex_unlock(&e->mutex);

    return e_rx_engage(e);
}

/* A delayed receive has come due. */
static void e_deadline_fire_rx(struct dw1000_emulation *e, uint64_t seq) {
    pthread_mutex_lock(&e->mutex);
    if (!E_IS_STATE(e, RX_WAIT) || !e_deadline_current(e, E_DEADLINE_RX, seq)) {
	// Cancelled between the deadline expiring and this running.
	EMU_DEBUG("rx: delayed receive no longer current, dropped");
	pthread_mutex_unlock(&e->mutex);
	return;
    }
    E_SET_STATE(e, RX);
    e_rx_arm_timeouts(e, e_clock_full());
    pthread_mutex_unlock(&e->mutex);

    e_rx_engage(e);
}

/* A receive timeout has expired: raise its bit and stop receiving.
 *
 * UM 7.2.14: the timeout "disables the receiver", and the receiver will
 * not re-enable by itself afterwards whatever the double buffer and
 * auto-re-enable settings say.
 */
static void e_deadline_fire_rxto(struct dw1000_emulation *e, uint64_t seq,
				 uint64_t flag) {
    bool edge;

    pthread_mutex_lock(&e->mutex);
    /* The state check alone is not enough here. A receiver that was
     * turned off and straight back on is in state RX either way, and
     * this timeout belongs to the session that was cancelled: firing
     * it would end the new one early, and leave the new session's own
     * timeout to expire later into nothing.
     */
    if (!E_IS_STATE(e, RX) || !e_deadline_current(e, E_DEADLINE_RXTO, seq)) {
	EMU_DEBUG("rx: timeout no longer current, dropped");
	pthread_mutex_unlock(&e->mutex);
	return;
    }

    uint32_t sys_status = E_REG_IC_READ32_KEY(e, SYS_STATUS);
    sys_status |= (uint32_t)flag;
    E_REG_IC_WRITE32_KEY(e, sys_status, SYS_STATUS);

    EMU_DEBUG("rx: timeout reached (%s)",
	      (flag == DW1000_FLG_SYS_STATUS_RXPTO) ? "RXPTO" : "RXRFTO");

    E_SET_STATE(e, IDLE);
    edge = e_irq_update(e);
    pthread_mutex_unlock(&e->mutex);

    if (edge) e_irq_fire(e);
}


/*-- Transmit ----------------------------------------------------------*/

/* Airtime of the preamble and the SFD, in device clock ticks: what the
 * chip has to already be transmitting before the RMARKER.
 *
 * APS022 5.4 calls it Ton, and UM 3.3 says the chip derives its internal
 * transmitter start time for a delayed send by subtracting exactly this
 * from the programmed time. Read out of TX_FCTRL rather than taken from
 * the driver, which is the point of a register model.
 *
 * The SFD is the standard one, 64 symbols at 110 kbps and 8 otherwise
 * (UM 4.1.3). A host that has selected the proprietary SFD through
 * CHAN_CTRL.DWSFD gets a Ton short by up to 56 symbols here, which moves
 * the TXPUTE boundary and nothing else.
 */
static uint64_t e_tx_ton(struct dw1000_emulation *e) {
    static const uint16_t plen_symbols[16] = {
	   0,   64, 1024, 4096,    0,  128, 1536,    0,
	   0,  256, 2048,    0,    0,  512,    0,    0,
    };

    uint32_t tx_fctrl = E_REG_IC_READ32_KEY(e, TX_FCTRL);
    unsigned plen     = (tx_fctrl >> DW1000_SFT_TX_FCTRL_TXPSR) & 0xF;
    unsigned bitrate  = (tx_fctrl >> DW1000_SFT_TX_FCTRL_TXBR ) & 0x3;
    unsigned prf      = (tx_fctrl >> DW1000_SFT_TX_FCTRL_TXPRF) & 0x3;

    uint64_t symbols  = plen_symbols[plen]
	              + ((bitrate == DW1000_BITRATE_110KBPS) ? 64 : 8);

    return symbols * ((prf == DW1000_PRF_64MHZ) ? E_TICKS_PER_PSYM_64MHZ
		                                : E_TICKS_PER_PSYM_16MHZ);
}

/* Hand the held frame to the medium server. The mutex must NOT be held;
 * the caller has already moved the state and taken its own copy of the
 * packet, which is what keeps the decision to send and the send itself
 * from being separable.
 */
static int e_tx_deliver(struct dw1000_emulation *e,
			struct dw1000_driver_iopkt *pkt, size_t len) {
    /* Not const: rsvc_i() takes void*, and adding a cast here to keep a
     * const that the call cannot honour would say less than this does.
     */
    if (rsvc_i(e->rsvc, RSVC_UWB_IO, pkt, len) < 0) {
	/* As for the receiver. The state matters more here: the caller
	 * has already moved the model to TX, and without this it would
	 * sit there waiting for a TX_DONE that cannot come, refusing
	 * every later command on the "transmit in progress" check.
	 */
	if (e_medium_lost(e))
	    EMU_WARNING("the medium server is not answering; this node"
			" hears and says nothing until it does");
	return -1;
    }
    e_medium_ok(e);
    return 0;
}

/* Move to TX and take a copy of the frame, under one hold. */
static int e_tx_engage(struct dw1000_emulation *e) {
    pthread_mutex_lock(&e->mutex);
    E_SET_STATE(e, TX);
    struct dw1000_driver_iopkt pkt = e->tx_pkt;
    size_t                     len = e->tx_pktlen;
    pthread_mutex_unlock(&e->mutex);

    return e_tx_deliver(e, &pkt, len);
}

/* A delayed send has come due.
 *
 * The check and the commitment are one hold. Splitting them (test the
 * state, drop the mutex, retake it and move to TX) left a window in
 * which a TRXOFF could find nothing armed (the slot having already been
 * dequeued), set the model idle, and then have the frame go out anyway
 * behind the host's back.
 */
static void e_deadline_fire_tx(struct dw1000_emulation *e, uint64_t seq) {
    struct dw1000_driver_iopkt pkt;
    size_t                     len;

    pthread_mutex_lock(&e->mutex);
    if (!E_IS_STATE(e, TX_WAIT) || !e_deadline_current(e, E_DEADLINE_TX, seq)) {
	// TRXOFF got here first, or this send has already been replaced.
	EMU_DEBUG("tx: delayed send no longer current, dropped");
	pthread_mutex_unlock(&e->mutex);
	return;
    }
    E_SET_STATE(e, TX);
    pkt = e->tx_pkt;
    len = e->tx_pktlen;
    pthread_mutex_unlock(&e->mutex);

    e_tx_deliver(e, &pkt, len);
}


int dw1000_emulation_send(struct dw1000_emulation *e) {
    pthread_mutex_lock(&e->mutex);

    uint32_t tx_fctrl   = E_REG_IC_READ32_KEY(e, TX_FCTRL);
    uint32_t sys_ctrl   = E_REG_IC_READ32_KEY(e, SYS_CTRL);
    /* 40 bits, not 32: TXPUTE is bit 34, in the fifth byte, and the
     * 32-bit accessors would drop it on the way back out.
     */
    uint64_t sys_status = E_REG_IC_READ40_KEY(e, SYS_STATUS);
    int      len        = DW1000_GET_VAL(tx_fctrl, TX_FCTRL_TFLE_TFLEN);
    int      offset     = DW1000_GET_VAL(tx_fctrl, TX_FCTRL_TXBOFFS   );
    int      ranging    = DW1000_GET_FLG(tx_fctrl, TX_FCTRL_TR        );
    bool     delayed    = DW1000_GET_FLG(sys_ctrl, SYS_CTRL_TXDLYS    );
    /* WAIT4RESP is not read here: it is acted upon when the server
     * reports the frame sent (DW1000_RSVC_TX_DONE).
     */

    /* Asked for by dw1000_emulation_drop_next_start(): the start is
     * taken and nothing happens, as under Errata TX-1.
     */
    if (e->drop_next_start) {
	e->drop_next_start = false;
	EMU_WARNING("transmit start dropped on request");
	DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TXSTRT);
	DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TXDLYS);
	DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_SFCST);
	E_REG_IC_WRITE32_KEY(e, sys_ctrl, SYS_CTRL);
	pthread_mutex_unlock(&e->mutex);
	return 0;
    }

    /* A TXSTRT written while the receiver is on is dropped, no flag
     * raised and nothing sent: measured on ruby-dw1000's bench, 200
     * raw starts into a listening receiver with no traffic, 200
     * dropped (SYS_STATE 0x4x050500 before and after, the chip still
     * receiving), which is also how one send in six thousand was lost
     * with RXAUTR set. The driver's contract is IDLE before a send, and
     * a host that breaks it now loses the frame the way the chip loses
     * it, rather than the run.
     */
    if (E_IS_STATE(e, RX) || E_IS_STATE(e, RX_WAIT)) {
	EMU_WARNING("transmit start dropped (state=%s)", E_GET_STATE_STR(e));
	DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TXSTRT);
	DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TXDLYS);
	DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_SFCST);
	E_REG_IC_WRITE32_KEY(e, sys_ctrl, SYS_CTRL);
	pthread_mutex_unlock(&e->mutex);
	return 0;
    }

    // Sanity check
    DW1000_ASSERT(E_IS_STATE(e, IDLE),
		  "transmit started from idle");
    DW1000_ASSERT(! DW1000_GET_FLG(sys_ctrl, SYS_CTRL_RXENAB),
		  "transmit started with the receiver off");


    uint8_t flags = 0;
    if (ranging) flags |= DW1000_RSVC_FLG_RANGING;


    // Cleared at transmitter enable:
    //  TXFRB : Transmit Frame Begin
    //  TXPRS : Transmit Preamble Sent
    //  TXPHS : Transmit PHY Header Sent
    //  TXFRS : Transmit Frame Sent
    DW1000_SET_FLG(sys_status, SYS_STATUS_TXFRB);
    DW1000_CLR_FLG(sys_status, SYS_STATUS_TXPRS);
    DW1000_CLR_FLG(sys_status, SYS_STATUS_TXPHS);
    DW1000_CLR_FLG(sys_status, SYS_STATUS_TXFRS);

    // Sys Ctrl
    // - Suppress auto-FCS transmission
    bool sfcst = DW1000_GET_FLG(sys_ctrl, SYS_CTRL_SFCST);
    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_SFCST);
    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TXSTRT);
    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TXDLYS);
    E_REG_IC_WRITE32_KEY(e, sys_ctrl, SYS_CTRL);

    /* Build the frame now, not when it goes out. The chip reads TX_BUFFER
     * as it transmits, but a host is told not to touch the buffer while a
     * send is pending, so taking a copy here differs from the chip only
     * for a host that has already broken that rule.
     */
    struct dw1000_driver_iopkt *iopkt = &e->tx_pkt;
    memset(iopkt, 0, sizeof(*iopkt));
    iopkt->drvid            = (uintptr_t) e;
    iopkt->type             = DW1000_RSVC_TX;
    iopkt->tx.flags         = flags;
    iopkt->tx.antenna_delay = E_REG_IC_READ16_KEY(e, TX_ANTD);

    // Deal with CRC
    DW1000_ASSERT(len >= 2, "frame long enough to hold the FCS");
    uint16_t crc16;
    if (sfcst) {
	// CRC has been provided, copy
	memcpy(iopkt->tx.frame, &E_REG_DATA_KEY(e, TX_BUFFER)[offset], len);
	// memcpy, not a cast: see the receive path for why
	uint16_t fcs;
	memcpy(&fcs, &iopkt->tx.frame[len - 2], sizeof(fcs));
	crc16 = dw1000_le16_to_cpu(fcs);
    } else {
	// Compute CRC16 CCITT
	crc16 = e_crc16_ccitt(&E_REG_DATA_KEY(e, TX_BUFFER)[offset], len - 2);

	// Copy data and CRC
	uint16_t crc16_le = dw1000_cpu_to_le16(crc16);
        memcpy(iopkt->tx.frame, &E_REG_DATA_KEY(e, TX_BUFFER)[offset], len - 2);
	memcpy(&iopkt->tx.frame[len-2], (uint8_t *)&crc16_le, 2);
    }

    e->tx_pktlen  = DW1000_DRIVER_PKTLEN_TX(len);
    e->tx_delayed = delayed;

    EMU_DEBUG("tx_start: %s packet"
	      " (offset=%d, size=%d, auto-crc=%c"
	      " crc16=0x%04" PRIx16 ")",
	      delayed ? "holding" : "sending",
	      offset, len, sfcst ? 'N': 'Y', crc16);

    if (!delayed) {
	e->tx_rawst = 0;
	E_REG_IC_WRITE40_KEY(e, sys_status, SYS_STATUS);
	/* The flags the enable cleared may have held the line up: settle
	 * IRQS now, as the chip would, so that this send's completion is a
	 * rising edge of its own. */
	(void)e_irq_update(e);
	pthread_mutex_unlock(&e->mutex);
	return e_tx_engage(e);
    }

    /* Delayed send. UM 3.3: the programmed time is the RMARKER, "the raw
     * TX time, TX_RAWST ... before the antenna delay is added", its low
     * nine bits ignored. The chip works back from it by Ton to an
     * internal start time, waits for the counter to reach that, and
     * begins preamble.
     */
    uint64_t now   = e_clock_full();
    uint64_t dx    = E_REG_IC_READ40_KEY(e, DX_TIME) & ~0x1FFull;
    uint64_t ton   = e_tx_ton(e);
    uint64_t start = (dx - ton) & E_CLOCK_MASK;

    /* UM 3.3: "it is the internal start time mentioned above that is used
     * when deciding whether to set the HPDWARN event", and UM 7.2.17
     * makes HPDWARN a condition rather than an event, so nothing is
     * set here. e_status_derived() answers it from how far the start
     * time still is, every time SYS_STATUS is read, and TRXOFF clears it
     * by disarming.
     *
     * Neither flag cancels anything. UM 3.3 is explicit that a long delay
     * may be intended, that HPDWARN "can be ignored and the transmission
     * will begin at the allotted time", and that stopping it is the
     * host's move, by TRXOFF. So the send is armed either way and the
     * driver's own policy decides.
     *
     * The lap is chosen on the start time and not on the RMARKER. For a
     * start time just behind us the chip waits almost a whole period for
     * the counter to come round to it, and the RMARKER follows Ton after
     * that, which is a different lap from the one the RMARKER alone
     * would have picked.
     */
    e->tx_start = e_clock_forward(now, start);
    e->tx_rawst = dx;
    E_REG_IC_WRITE40_KEY(e, sys_status, SYS_STATUS);

    E_SET_STATE(e, TX_WAIT);
    e_deadline_arm(e, E_DEADLINE_TX, e->tx_start + ton, 0);

    EMU_DEBUG("tx_start: delayed, preamble in %" PRId64 " ticks,"
	      " RMARKER at 0x%010" PRIx64,
	      (int64_t)(e->tx_start - now), dx);

    pthread_mutex_unlock(&e->mutex);
    return 0;
}


void
_dw1000_spi_header_decode(uint8_t *hdr, size_t hlen,
			  uint8_t *reg, size_t *offset, bool *write)
{
    *reg    = hdr[0] & 0x3F;
    *write  = hdr[0] & 0x80;
    *offset = 0;

    if (hdr[0] & 0x40) {
	DW1000_ASSERT(hlen > 1, "SPI header carries its second byte");
        *offset = hdr[1] & 0x7F;
	
	if (hdr[1] & 0x80) {
	    DW1000_ASSERT(hlen > 2, "SPI header carries its third byte");
	    *offset |= (hdr[2] & 0xFF) << 7;
	}
    }
}


void
_dw1000_ioline_set(dw1000_ioline_t line) {
    switch(line->line) {
    case DW1000_IOLINE_RESET:
	dw1000_emulation_reset(line->emulation);
	
        break;
    case DW1000_IOLINE_WAKEUP:
	EMU_FATAL("unimplemented: wakeup line not implemented");
	DW1000_ASSERT(0, "wakeup line not implemented");
	break;
    }
}
void
_dw1000_ioline_clear(dw1000_ioline_t line) {
    switch(line->line) {
    case DW1000_IOLINE_RESET:
	break;
    case DW1000_IOLINE_WAKEUP:
	break;
    }
}


void _dw1000_spi_send(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {

    uint8_t reg;
    size_t  offset;
    bool    write;
    
    struct dw1000_emulation *e = spi->emulation;
    
    _dw1000_spi_header_decode(hdr, hdrlen, &reg, &offset, &write);

    bool send = false;
    bool recv = false;
    pthread_mutex_lock(&e->mutex);
    
    E_REG_HOST_WRITE_IDX(e, reg, offset, data, datalen);

    if (reg == DW1000_REG_SYS_CFG) {
	/* UM 4.3.1: double buffering is DIS_DRXB cleared. Tracked here
	 * rather than read per access, since which set an access uses is
	 * decided before the access happens.
	 */
	uint32_t sys_cfg = E_REG_IC_READ32_KEY(e, SYS_CFG);
	bool     want    = !DW1000_GET_FLG(sys_cfg, SYS_CFG_DIS_DRXB);
	if (want != e->dblbuff) {
	    e->dblbuff = want;
	    EMU_DEBUG("dblbuff: %s", want ? "enabled" : "disabled");
	    e_dblbuff_publish(e);
	}
    }

    if (reg == DW1000_REG_SYS_CTRL) {
	uint32_t sys_ctrl = E_REG_IC_READ32_KEY(e, SYS_CTRL);

	/* HRBPT is a command bit, self-clearing, and independent of the
	 * rest of SYS_CTRL: the driver writes it on its own in the last
	 * byte of the register.
	 */
	if (DW1000_GET_FLG(sys_ctrl, SYS_CTRL_HRBPT)) {
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_HRBPT);
	    E_REG_IC_WRITE32_KEY(e, sys_ctrl, SYS_CTRL);
	    e_dblbuff_toggle_host(e);
	}

	if (DW1000_GET_FLG(sys_ctrl, SYS_CTRL_TRXOFF)) {
	    EMU_DEBUG("trxoff");
	    /* Whatever was programmed is off the books: a delayed send or
	     * receive still counting down, and the receive timeout that
	     * would have ended the reception being cancelled here.
	     */
	    e_deadline_disarm_all(e);
	    // Return to IDLE state
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TXSTRT);
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_SFCST);
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TXDLYS);
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_CANSFCS);
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TRXOFF);
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_WAIT4RESP);
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_RXENAB);
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_RXDLYE);

	    E_SET_STATE(e, IDLE);
	} else if (DW1000_GET_FLG(sys_ctrl, SYS_CTRL_TXSTRT)) {
	    send = true;
	} else if (DW1000_GET_FLG(sys_ctrl, SYS_CTRL_RXENAB)) {
	    recv = true;

		
	}
	E_REG_IC_WRITE32_KEY(e, sys_ctrl, SYS_CTRL);
    }
    pthread_mutex_unlock(&e->mutex);

    if (send) dw1000_emulation_send(e);
    if (recv) dw1000_emulation_recv(e);

    /* IRQS is derived from SYS_STATUS and SYS_MASK, so recomputing it is a
     * read-modify-write of a register and belongs under the mutex, which
     * it was not, and a frame arriving on the rsvc reader thread between
     * the two reads here left IRQS describing neither status.
     */
    pthread_mutex_lock(&e->mutex);
    bool edge = e_irq_update(e);
    pthread_mutex_unlock(&e->mutex);

    if (edge) e_irq_fire(e);
}

void _dw1000_spi_recv(dw1000_spi_driver_t *spi,
		      uint8_t *hdr,  size_t hdrlen,
		      uint8_t *data, size_t datalen) {
    uint8_t reg;
    size_t  offset;
    bool    write;
    struct dw1000_emulation *e = spi->emulation;

    _dw1000_spi_header_decode(hdr, hdrlen, &reg, &offset, &write);

    pthread_mutex_lock(&e->mutex);

    /* SYS_TIME free-runs on the chip, so it is not storage the host wrote
     * and the model happens to keep: it is whatever the clock says at the
     * instant of the read. Sample it here rather than on a tick, which
     * the model has no way to generate.
     */
    if (reg == DW1000_REG_SYS_TIME)
	E_REG_IC_WRITE40_KEY(e, dw1000_emulation_clock(), SYS_TIME);

    /* Same reasoning for HPDWARN and TXPUTE: they are conditions the
     * chip evaluates, not events it remembers, so they are worked out at
     * the moment the host looks.
     */
    if (reg == DW1000_REG_SYS_STATUS)
	e_status_derived(e);

    /* SYS_STATE is the model's own state, spelled the way the chip was
     * seen to spell it. User Manual 2.12 documents none of it (register
     * file 0x19 is reserved there): the values are ruby-dw1000's bench,
     * 0x00040001 while a frame goes out, 0x4x050500 with the receiver
     * enabled (traffic or none), 0x00020000 while a delayed send waits
     * for DX_TIME, 0x00010000 idle, and PMSC 3 for the first
     * microseconds after an enable; so PMSC_STATE in bits 16..20 is 4
     * in TX, 5 in RX, 2 in TX_WAIT, 1 idle, TX_STATE in bits 0..3 is 1
     * in TX, RX_STATE in bits 8..12 is 5 in RX. RX_WAIT 3 and INIT 0
     * are this model's own, by analogy.
     */
    if (reg == DW1000_REG_SYS_STATE) {
	uint32_t pmsc = 0, txs = 0, rxs = 0;
	if      (E_IS_STATE(e, IDLE))    pmsc = 1;
	else if (E_IS_STATE(e, TX_WAIT)) pmsc = 2;
	else if (E_IS_STATE(e, RX_WAIT)) pmsc = 3;
	else if (E_IS_STATE(e, TX))    { pmsc = 4; txs = 1; }
	else if (E_IS_STATE(e, RX))    { pmsc = 5; rxs = 5; }
	E_REG_IC_WRITE40_KEY(e, (uint64_t)((pmsc << 16) | (rxs << 8) | txs),
			     SYS_STATE);
    }

    E_REG_HOST_READ_IDX(e, reg, offset, data, datalen );
    pthread_mutex_unlock(&e->mutex);
}

void _dw1000_spi_low_speed(dw1000_spi_driver_t *spi) {
    (void)spi;
    // No-op
}

void _dw1000_spi_high_speed(dw1000_spi_driver_t *spi) {
    (void)spi;
    // No-op
}


void rsvc_uwb_handler(rsvc_t *rsvc, uint16_t type, void *data, size_t length, void *args) {
    struct dw1000_driver_iopkt *iopkt = data;
    struct dw1000_emulation    *e     = args;
    bool                        edge  = false;
    bool                        taken = false;

    /* The service type is what got us here, and the connection is reached
     * through the emulation; neither is needed again.
     */
    (void)rsvc; (void)type;

    
    EMU_DEBUG("Got RSVC interrupt (type: 0x%0x)", iopkt->type);

    pthread_mutex_lock(&e->mutex);

    // Sanity check 
    size_t iopktlen = length;
    if (sizeof(*iopkt) < length)
	iopktlen = sizeof(*iopkt);

    // Dispatch according to type
    switch (iopkt->type) {
    case DW1000_RSVC_RX: {
	// Frame length
        int framelen = iopktlen - offsetof(struct dw1000_driver_iopkt,rx.frame);
        DW1000_ASSERT((framelen >= 2) && (framelen <= DW1000_LEN_RX_BUFFER),
		      "received frame fits the receive buffer");
	
	// Ranging
	bool ranging = iopkt->rx.flags & DW1000_RSVC_FLG_RANGING;

	/* Compute the FCS and compare it with the two bytes the frame
	 * carries. Read through memcpy: `frame` is a member of a packed
	 * struct at an offset that is not a multiple of two, so casting
	 * into it is an unaligned access, undefined and a fault on the
	 * architectures that trap it.
	 */
	uint16_t crc = e_crc16_ccitt(iopkt->rx.frame, framelen - 2);
	uint16_t fcs;
	memcpy(&fcs, &iopkt->rx.frame[framelen - 2], sizeof(fcs));
	bool crc_ok = crc == dw1000_le16_to_cpu(fcs);
	
        // Debug info
        EMU_DEBUG("RSVC INT <RX_DONE>"
		  " (frame-size=%d, crc=%s, state=%s)",
		  framelen, crc_ok ? "OK" : "KO", E_GET_STATE_STR(e));

	// State check
        if (! E_IS_STATE(e, RX)) {
	    EMU_WARNING("packet lost (receiver not enabled: %s)",
			E_GET_STATE_STR(e));
	    goto done;
	}

	/* UM 7.2.14(b): a frame arriving stops the frame wait timeout
	 * counter, so RXRFTO will not be set. Same for the preamble
	 * timeout, which UM 7.2.40.9 ends at preamble detection.
	 */
	e_deadline_disarm(e, E_DEADLINE_RXTO);

	/* Overrun. UM 4.3.5: the IC has filled one buffer, moved to the
	 * other and filled that too, and come back to a buffer the host
	 * has still not released with HRBPT. The frame in progress is
	 * abandoned ("the frame reception in progress will be aborted"),
	 * ICRBP does not move, and RXOVRR stands until the host
	 * issues HRBPT.
	 */
	if (e->dblbuff && (e->pending >= 2)) {
	    uint32_t ovrr = E_REG_IC_READ32_KEY(e, SYS_STATUS);
	    DW1000_SET_FLG(ovrr, SYS_STATUS_RXOVRR);
	    E_REG_IC_WRITE32_KEY(e, ovrr, SYS_STATUS);

	    EMU_WARNING("receiver overrun: both buffers hold a frame the"
			" host has not read out");

	    /* UM 4.3.5: "assuming RX auto-re-enable is enabled (by
	     * RXAUTR) the receiver will begin looking for preamble
	     * again".
	     */
	    if (e_rx_auto_reenable(e, false)) {
		e_rx_arm_timeouts(e, e_clock_full());
	    } else {
		E_SET_STATE(e, IDLE);
	    }

	    edge = e_irq_update(e);
	    goto done;
	}
	
	// Not handled: AFFREJ AAT RXOVRR RXRSCS RXPREJ
	uint32_t sys_status = E_REG_IC_READ32_KEY(e, SYS_STATUS);
	DW1000_SET_FLG(sys_status, SYS_STATUS_RXPRD);
	DW1000_CLR_FLG(sys_status, SYS_STATUS_RXPTO);   // Not emulated
	DW1000_SET_FLG(sys_status, SYS_STATUS_RXSFDD);
	DW1000_CLR_FLG(sys_status, SYS_STATUS_RXSFDTO); // Not emulated
	DW1000_SET_FLG(sys_status, SYS_STATUS_LDEDONE);
	DW1000_CLR_FLG(sys_status, SYS_STATUS_LDEERR);
	DW1000_SET_FLG(sys_status, SYS_STATUS_RXPHD);
	DW1000_CLR_FLG(sys_status, SYS_STATUS_RXPHE);
	DW1000_CLR_FLG(sys_status, SYS_STATUS_RXRFSL);
	DW1000_SET_FLG(sys_status, SYS_STATUS_RXDFR);
	DW1000_CLR_FLG(sys_status, SYS_STATUS_RXRFTO);  // Not emulated

	// RX_TTCKI / RX_TTCKO
	uint32_t rx_ttcki = 0x1000; // Fake value, need improvement
	uint64_t rx_ttcko = 0;      // Fake value, need improvement
	E_REG_IC_WRITE32_KEY(e, rx_ttcki, RX_TTCKI);
	E_REG_IC_WRITE40_KEY(e, rx_ttcko, RX_TTCKO);
	
        // Antenna delay
	uint16_t antenna_delay = E_REG_IC_READ16_KEY(e, LDE_IF, LDE_RXANTD);

	// Reception time
	// (adjusted to take into account antenna delay)
	uint64_t rx_time = dw1000_le64_to_cpu(iopkt->rx.timestamp);
	E_REG_IC_WRITE40_KEY(e, rx_time, RX_TIME, RX_TIME_RX_STAMP);
	
	// Raw transmission time
	// (which is on a 512-tick boudary)
	// => building a fake one 
	rx_time += antenna_delay;
	rx_time  = DW1000_CLOCK_ROUNDUP(rx_time);
	E_REG_IC_WRITE40_KEY(e, rx_time, RX_TIME, RX_TIME_RX_RAWST);

	// Update RX_FINFO
	// Not supported: RXPACC, RXPSR, RXBR, RXNSPL
	uint32_t rx_finfo = 0;
        DW1000_SET_VAL(rx_finfo, RX_FINFO_RXFLE_RXFLEN, framelen);
	if (ranging) { DW1000_SET_FLG(rx_finfo, RX_FINFO_RNG); }
	else         { DW1000_CLR_FLG(rx_finfo, RX_FINFO_RNG); }
	E_REG_IC_WRITE32_KEY(e, rx_finfo, RX_FINFO);
	
	// Update RX_BUFFER
	memcpy(E_REG_DATA_KEY(e, RX_BUFFER), iopkt->rx.frame, framelen);

	// Check CRC
        if (crc_ok) {
	    DW1000_SET_FLG(sys_status, SYS_STATUS_RXFCG);
	    DW1000_CLR_FLG(sys_status, SYS_STATUS_RXFCE);
	} else {
	    DW1000_CLR_FLG(sys_status, SYS_STATUS_RXFCG);
	    DW1000_SET_FLG(sys_status, SYS_STATUS_RXFCE);
	}
	    
	// Write status
	E_REG_IC_WRITE32_KEY(e, sys_status, SYS_STATUS);

        EMU_DEBUG("RSVC INT <RX_DONE> commited to registers");

	/* Only a good frame moves the IC on to the other buffer, and it
	 * does so after the interrupt has been worked out below.
	 */
	taken = crc_ok;

	/* UM 5.3.2: with RXAUTR the receiver turns itself back on for
	 * the next frame, and UM 7.2.14 restarts the frame wait
	 * countdown with it. Otherwise the chip goes idle and waits for
	 * the host.
	 */
	if (e_rx_auto_reenable(e, crc_ok)) {
	    e_rx_arm_timeouts(e, e_clock_full());
	} else {
	    E_SET_STATE(e, IDLE);
	}

	break;
    }
	
    case DW1000_RSVC_TX_DONE: {
	EMU_DEBUG("RSVC INT <TX_DONE> (ts=0x%010lx)", iopkt->tx_done.timestamp);

	/* Only if a transmission is actually in progress. A TRXOFF while
	 * the frame was on its way to the server aborts it, and the chip
	 * then raises no TXFRS and calls nothing back; without this check
	 * the late reply would report a frame the host has cancelled, and
	 * would drag whatever state the host has since reached (a fresh
	 * receive, say) back to IDLE behind its back.
	 */
	if (! E_IS_STATE(e, TX)) {
	    EMU_WARNING("transmit completion ignored (state=%s)",
			E_GET_STATE_STR(e));
	    goto done;
	}

	uint64_t antd = E_REG_IC_READ16_KEY(e, TX_ANTD);
	uint64_t raw;

	if (e->tx_delayed) {
	    /* A delayed send does not need to be told when it happened:
	     * UM 3.3 says the chip works its transmitter start time
	     * backwards from the programmed time precisely so that the
	     * RMARKER lands on it, so TX_RAWST *is* DX_TIME with its low
	     * nine bits cleared, and TX_STAMP is that plus the antenna
	     * delay. The server's own stamp is discarded here; it carries
	     * the scheduling jitter of the moment this node got round to
	     * sending the request, which the chip would not have.
	     *
	     * That jitter is not gone, only moved: it is still in the
	     * arrival times the server computes for every *other* node,
	     * which have no such register to be corrected from. Closing
	     * that needs a field on the wire, and the protocol has none;
	     * see port/emulation/README.md.
	     */
	    raw = e->tx_rawst;
	} else {
	    uint64_t tx_time = dw1000_le64_to_cpu(iopkt->tx_done.timestamp);
	    raw = (tx_time - antd) & E_CLOCK_MASK;

	    // Assert that the transmission time as been generated
	    // taking into account clock transmission at a 512-tick boundary
	    DW1000_ASSERT(raw == DW1000_CLOCK_ROUNDUP(raw),
			  "transmit timestamp on a 512-tick boundary");
	}

	E_REG_IC_WRITE40_KEY(e, (raw + antd) & E_CLOCK_MASK,
			     TX_TIME, TX_TIME_TX_STAMP);
	E_REG_IC_WRITE40_KEY(e, raw, TX_TIME, TX_TIME_TX_RAWST);
	e->tx_delayed = false;
	
	// Mark every part of the packet as transmitted
        uint32_t sys_status = E_REG_IC_READ32_KEY(e, SYS_STATUS);
	DW1000_SET_FLG(sys_status, SYS_STATUS_TXPRS);
	DW1000_SET_FLG(sys_status, SYS_STATUS_TXPHS);
	DW1000_SET_FLG(sys_status, SYS_STATUS_TXFRS);
	E_REG_IC_WRITE32_KEY(e, sys_status, SYS_STATUS);

	// Check if we need to move to RX or IDLE 
	uint32_t sys_ctrl = E_REG_IC_READ32_KEY(e, SYS_CTRL);
	bool rx_enable = DW1000_GET_FLG(sys_ctrl, SYS_CTRL_WAIT4RESP);

	if (rx_enable) {
	    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_WAIT4RESP);
	    E_REG_IC_WRITE32_KEY(e, sys_ctrl, SYS_CTRL);
	    E_SET_STATE(e, RX);

	    /* This is a receiver turn-on like any other, so the timeouts
	     * start counting here too: UM 7.2.14 says the frame wait
	     * timeout starts "each time the receiver is enabled", and
	     * without this a host that armed RX_FWTO and sent with
	     * WAIT4RESP would wait for a reply that never times out.
	     *
	     * W4R_TIM, the programmable turnaround delay of ACK_RESP_T,
	     * is not modelled: the receiver comes on immediately.
	     */
	    e_rx_arm_timeouts(e, e_clock_full());
	} else {
	    E_SET_STATE(e, IDLE);
	}
	
	break;
    }
	
    default:
	EMU_FATAL("RSVC INT <unknown> (type=0x%x, size=%zu)",
		    iopkt->type, iopktlen);
    }


    edge = e_irq_update(e);
    if (taken)
	e_dblbuff_advance_ic(e);

 done:
    pthread_mutex_unlock(&e->mutex);

    if (edge) e_irq_fire(e);
}


/* Recompute IRQS and report the rising edge. The model's mutex must be
 * held; nothing here leaves the model.
 */
static bool e_irq_update(struct dw1000_emulation *e) {
    // HPDWARN and TXPUTE are conditions; settle them before reading.
    e_status_derived(e);

    /* Re-compute IRQS.
     *
     * From the HOST's set, not the IC's. UM 7.2.17 defines IRQS over the
     * status bits as the host sees them ("whenever a status bit ... is
     * activated and the corresponding bit in [SYS_MASK] is enabled"),
     * and four of those bits are per-buffer. Taking them from the IC's
     * set means that with both buffers full, the second frame's RXFCG
     * sits in the set the host is on while the IC has already swung back
     * to the other, so IRQS reads zero and the edge never comes: the
     * frame is stranded until a third one arrives. That is also why the
     * whole mask/clear/unmask/HRBPT dance of UM 4.3.3 exists: the line
     * follows the bits the host can see.
     */
    uint32_t sys_status = E_REG_HOST_READ32_KEY(e, SYS_STATUS);
    uint32_t sys_mask   = E_REG_IC_READ32_KEY(e, SYS_MASK);
    bool     irqs       = (sys_status & sys_mask) != 0;

    /* Written back as byte 0 alone. The whole word came from the host's
     * set and this is IC-side write, so writing it all back would push
     * that set's per-buffer bits into the other one. Byte 0 is entirely
     * passthrough (it holds no per-buffer bit), so one write reaches
     * both sets and disturbs nothing.
     */
    uint8_t byte0 = (uint8_t)sys_status;
    if (irqs) { byte0 |=  (uint8_t)DW1000_FLG_SYS_STATUS_IRQS; }
    else      { byte0 &= ~(uint8_t)DW1000_FLG_SYS_STATUS_IRQS; }
    E_REG_IC_WRITE_KEY(e, &byte0, 1, SYS_STATUS);

    // The line is level-sensitive on the chip; what a host sees is the
    // edge, so only the low-to-high transition is reported.
    bool edge = !e->irq && irqs;
    e->irq = irqs;
    return edge;
}

/* Drive the line. The model's mutex must NOT be held.
 */
static void e_irq_fire(struct dw1000_emulation *e) {
    EMU_DEBUG("posting interrupt");
    if (e->line_cb)
	e->line_cb(DW1000_IOLINE_IRQ, e->line_args);
}


