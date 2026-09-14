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
 * and the few registers that mean something -- SYS_CTRL, SYS_STATUS,
 * SYS_MASK, the TX and RX buffers and their timestamps -- are given the
 * behaviour the chip has: writing TXSTRT sends a frame to the medium
 * server, an incoming frame fills RX_BUFFER and sets the RX status bits,
 * and any status bit that the mask lets through raises the IRQ line.
 *
 * Originally written as a spank OSAL port; it has no dependency on spank.
 */

/* TODO:
 *  - correctly implement swing-set for double buffer use.
 */


#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include <pthread.h>
#include <sys/queue.h>

#include "dw1000/osal.h"
#include "dw1000/dw1000.h"
#include "dw1000/dw1000_reg.h"
#include "dw1000/dw1000_bswap.h"
#include "dw1000/emulation.h"
#include "rsvc.h"
#include "emu_log.h"
#include "emulation.h"


struct dw1000_emulation {
    pthread_mutex_t	mutex;
    rsvc_t 		*rsvc;
    void               (*line_cb)(int line, void *args);
    void                *line_args;
    int 		state;
    int 		swing_set;
    bool                irq;
    
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


void e_raise_interrupt(struct dw1000_emulation *e);


E_DEFINE_PASSTHROUGH(SYS_STATUS, 0xff, 0x1b, 0xff, 0xff, 0xff);



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


#define E_STATE_OFF		0
#define E_STATE_WAKEUP		1
#define E_STATE_INIT		2
#define E_STATE_IDLE		3
#define E_STATE_SLEEP		4
#define E_STATE_DEEPSLEEP	5
#define E_STATE_TX		6
#define E_STATE_RX		7
#define E_STATE_SNOOZE		8

static char *e_state[] = {
    [E_STATE_OFF      ] = "OFF",
    [E_STATE_WAKEUP   ] = "WAKEUP",
    [E_STATE_INIT     ] = "INIT",
    [E_STATE_IDLE     ] = "IDLE",
    [E_STATE_SLEEP    ] = "SLEEP",
    [E_STATE_DEEPSLEEP] = "DEEPSLEEP",
    [E_STATE_TX       ] = "TX",
    [E_STATE_RX	      ] = "RX",
    [E_STATE_SNOOZE   ] = "SNOOZE"
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
    uint8_t           *reg_data_c = reg->data[e->swing_set];
    uint8_t           *reg_data_o = reg->data[(e->swing_set + 1) % 2];
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
	    for (size_t i = 0 ; i < s_length ; i++)
		reg_data_c[s_offset + i] &= ~data[i];
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
    uint8_t           *reg_data  = reg->data[e->swing_set];
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
    
    e->rsvc      = rsvc;
    e->line_cb   = line_cb;
    e->line_args = line_args;
   
    e->swing_set = 0;

    E_SET_STATE(e, OFF);
    
    E_REG_ATTACH(e, DEV_ID,      SINGLE);
    E_REG_ATTACH(e, SYS_CFG,     SINGLE); 
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
    E_REG_ATTACH(e, DRX_CONF,    SINGLE); // ignored except DW1000_OFF_DRX_RXPACC_NOSAT
    E_REG_ATTACH(e, RF_CONF,     SINGLE); // ignored
    E_REG_ATTACH(e, TX_CAL,      SINGLE); // #define DW1000_OFF_TC_SARC : SAR_LTEMP / SAR_LVBAT
    E_REG_ATTACH(e, FS_CTRL,     SINGLE); // ignored
    E_REG_ATTACH(e, AON,         SINGLE); // ignored
    E_REG_ATTACH(e, OTP_IF,      SINGLE); // ignored except...
    E_REG_ATTACH(e, LDE_IF,      SINGLE);
    E_REG_ATTACH(e, PMSC,        SINGLE); // ignored

    E_REG_ATTACH(e, EUI,         TODO);
    E_REG_ATTACH(e, PANADR,      TODO);
    E_REG_ATTACH(e, SYS_TIME,    TODO);
    E_REG_ATTACH(e, SYS_STATE,   TODO);
    E_REG_ATTACH(e, DX_TIME,     TODO);
    E_REG_ATTACH(e, RX_FWTO,     TODO);
    E_REG_ATTACH(e, RX_SNIFF,    TODO);
    E_REG_ATTACH(e, ACK_RESP_T,  TODO);
    E_REG_ATTACH(e, ACC_MEM,     TODO);
    E_REG_ATTACH(e, DIAG,        TODO);
    
    
    e->reg[DW1000_REG_SYS_STATUS].passthrough = reg_p_SYS_STATUS;
    
    dw1000_emulation_reset(e);

    if (!rsvc_register(e->rsvc, RSVC_UWB_IO, rsvc_uwb_handler, e)) {
	EMU_FATAL("failed to register RSVC callback");
    }

    return e;
}

void dw1000_emulation_reset(struct dw1000_emulation *e) {
    E_SET_STATE(e, IDLE);
    e->swing_set = 0;

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
}


int dw1000_emulation_recv(struct dw1000_emulation *e) {
    EMU_DEBUG("rx_start: enter (state=%s)", E_GET_STATE_STR(e));
    pthread_mutex_lock(&e->mutex);

    // Auto-clear RX enable flag as command taken into account
    uint32_t sys_ctrl  = E_REG_IC_READ32_KEY(e, SYS_CTRL);
    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_RXENAB);
    E_REG_IC_WRITE32_KEY(e, sys_ctrl, SYS_CTRL);

    // Sanity check
    DW1000_ASSERT(E_IS_STATE(e, IDLE),
		  "receiver enabled from idle");
    DW1000_ASSERT(! DW1000_GET_FLG(sys_ctrl, SYS_CTRL_TXSTRT),
		  "receiver enabled with no transmit pending");

    // Mark as RX state
    E_SET_STATE(e, RX);

    EMU_DEBUG("rx_start: requesting rx start (sending RX_CONFIG packet)");
    struct dw1000_driver_iopkt iopkt = {
	 .drvid           = (uintptr_t) e,
	 .type            = DW1000_RSVC_RX_CONFIG,
	 .rx_config.flags = 0,
    };
    size_t pktlen = DW1000_DRIVER_PKTLEN_RX_CONFIG();
    
    pthread_mutex_unlock(&e->mutex);

    if (rsvc_i(e->rsvc, RSVC_UWB_IO, &iopkt, pktlen) < 0) {
	EMU_FATAL("rx_start: sending RX_CONFIG failed");
    }
    EMU_DEBUG("rx_start: done");

    return 0;
}


int dw1000_emulation_send(struct dw1000_emulation *e) {
    pthread_mutex_lock(&e->mutex);

    uint32_t tx_fctrl   = E_REG_IC_READ32_KEY(e, TX_FCTRL);
    uint32_t sys_ctrl   = E_REG_IC_READ32_KEY(e, SYS_CTRL);
    uint32_t sys_status = E_REG_IC_READ32_KEY(e, SYS_STATUS);
    int      len        = DW1000_GET_VAL(tx_fctrl, TX_FCTRL_TFLE_TFLEN);
    int      offset     = DW1000_GET_VAL(tx_fctrl, TX_FCTRL_TXBOFFS   );
    int      ranging    = DW1000_GET_FLG(tx_fctrl, TX_FCTRL_TR        );
    /* WAIT4RESP is not read here: it is acted upon when the server
     * reports the frame sent (DW1000_RSVC_TX_DONE).
     */

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
    E_REG_IC_WRITE32_KEY(e, sys_status, SYS_STATUS);

    // Sys Ctrl     
    // - Suppress auto-FCS transmission
    bool sfcst = DW1000_GET_FLG(sys_ctrl, SYS_CTRL_SFCST);
    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_SFCST);
    DW1000_CLR_FLG(sys_ctrl, SYS_CTRL_TXSTRT);
    E_REG_IC_WRITE32_KEY(e, sys_ctrl, SYS_CTRL);
    
    struct dw1000_driver_iopkt iopkt = {
	 .drvid            = (uintptr_t) e,
	 .type             = DW1000_RSVC_TX,
	 .tx.flags         = flags,
	 .tx.antenna_delay = E_REG_IC_READ16_KEY(e, TX_ANTD)
    };

    // Deal with CRC
    DW1000_ASSERT(len >= 2, "frame long enough to hold the FCS");
    uint16_t crc16;
    if (sfcst) {
	// CRC has been provided, copy 
	memcpy(iopkt.tx.frame, &E_REG_DATA_KEY(e, TX_BUFFER)[offset], len);
	crc16 = dw1000_le16_to_cpu(*(uint16_t *)&iopkt.tx.frame[len - 2]);
    } else {
	// CRC need to be computed
	DW1000_ASSERT(len >= 2, "frame long enough to hold the FCS");

	// Compute CRC16 CCITT
	crc16 = e_crc16_ccitt(&E_REG_DATA_KEY(e, TX_BUFFER)[offset], len - 2);

	// Copy data and CRC
	uint16_t crc16_le = dw1000_cpu_to_le16(crc16);
        memcpy(iopkt.tx.frame, &E_REG_DATA_KEY(e, TX_BUFFER)[offset], len - 2);
	memcpy(&iopkt.tx.frame[len-2], (uint8_t *)&crc16_le, 2);
    }
    
    size_t pktlen = DW1000_DRIVER_PKTLEN_TX(len);

    // Move to TW state
    E_SET_STATE(e, TX);

    // Prepare and send io packet
    EMU_DEBUG("tx_start: sending packet"
	      " (offset=%d, size=%d, auto-crc=%c"
	      " crc16=0x%04" PRIx16 ")",
	      offset, len, sfcst ? 'N': 'Y', crc16);
    
    pthread_mutex_unlock(&e->mutex);
    
    if (rsvc_i(e->rsvc, RSVC_UWB_IO, &iopkt, pktlen) < 0) {
	EMU_DEBUG("tx_start: sending packet failed");

        return -1;
    }

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

    if (reg == DW1000_REG_SYS_CTRL) {
	uint32_t sys_ctrl = E_REG_IC_READ32_KEY(e, SYS_CTRL);
	if (DW1000_GET_FLG(sys_ctrl, SYS_CTRL_TRXOFF)) {
	    EMU_DEBUG("trxoff");
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

    e_raise_interrupt(e);
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

	// Compute CRC
	uint16_t crc    = e_crc16_ccitt(iopkt->rx.frame, framelen - 2);
	bool     crc_ok = dw1000_cpu_to_le16(crc) ==
	                  *(uint16_t *)&iopkt->rx.frame[framelen - 2];
	
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
	uint64_t rx_time = dw1000_cpu_to_le64(iopkt->rx.timestamp);
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
	
	// Return to IDLE state
	E_SET_STATE(e, IDLE);

	break;
    }
	
    case DW1000_RSVC_TX_DONE: {
	EMU_DEBUG("RSVC INT <TX_DONE> (ts=0x%010lx)", iopkt->tx_done.timestamp);

        // Transmission time
	// (adjusted to take into account antenna delay)
	uint64_t tx_time = dw1000_cpu_to_le64(iopkt->tx_done.timestamp);
	E_REG_IC_WRITE40_KEY(e, tx_time, TX_TIME, TX_TIME_TX_STAMP);

	// Raw transmission time
	// (which is on a 512-tick boudary)
        tx_time -= E_REG_IC_READ16_KEY(e, TX_ANTD);
	E_REG_IC_WRITE40_KEY(e, tx_time, TX_TIME, TX_TIME_TX_RAWST);

	// Assert that the transmission time as been generated
	// taking into account clock transmission at a 512-tick boundary
	DW1000_ASSERT(tx_time == DW1000_CLOCK_ROUNDUP(tx_time),
		      "transmit timestamp on a 512-tick boundary");
	
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
	} else {
	    E_SET_STATE(e, IDLE);
	}
	
	break;
    }
	
    default:
	EMU_FATAL("RSVC INT <unknown> (type=0x%x, size=%zu)",
		    iopkt->type, iopktlen);
    }


    e_raise_interrupt(e);

 done:
    pthread_mutex_unlock(&e->mutex);
}


void e_raise_interrupt(struct dw1000_emulation *e) {
    // Re-compute interruption flag IRQS
    uint32_t sys_status = E_REG_IC_READ32_KEY(e, SYS_STATUS);
    uint32_t sys_mask   = E_REG_IC_READ32_KEY(e, SYS_MASK);
    bool     irqs       = (sys_status & sys_mask) != 0;

    // Update sys_status
    if (irqs) { DW1000_SET_FLG(sys_status, SYS_STATUS_IRQS); }
    else      { DW1000_CLR_FLG(sys_status, SYS_STATUS_IRQS); }
    E_REG_IC_WRITE32_KEY(e, sys_status, SYS_STATUS);

    // Raise interrupts (only on state change)
    if (!e->irq && irqs) {
	EMU_DEBUG("posting interrupt");	
	if (e->line_cb)
	    e->line_cb(DW1000_IOLINE_IRQ, e->line_args);
    }
    e->irq = irqs;
}


