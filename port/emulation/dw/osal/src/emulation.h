/*
 * Copyright (c) 2024-2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Private macros of the register model: how a register is declared, how
 * it is attached to the model, and how the two sides reach it: IC-side
 * (the model itself, which may write a read-only register) and host-side
 * (the driver over SPI, which may not).
 */

#ifndef _DW1000_OSAL_EMULATION_
#define _DW1000_OSAL_EMULATION_


/*----------------------------------------------------------------------*/
/* The device clock                                                     */
/*----------------------------------------------------------------------*/

/* A device timestamp is 40 bits, and every register field that holds one
 * is 40 bits wide. Arithmetic on device time is done in uint64_t and
 * brought back through this mask, so that a delay crossing the wrap is
 * the same subtraction as one that does not.
 */
#define E_CLOCK_MASK		((uint64_t)((1ull << DW1000_TIME_CLOCK_BITS) - 1))

/* Signed distance from @p a to @p b on the 40-bit clock, in ticks:
 * positive if b is ahead of a. The chip compares a programmed time with
 * the running counter, and that counter wraps every ~17.2 seconds, so
 * "is this deadline in the past" is a question about the shorter arc
 * between the two points and not about which number is larger. Half the
 * range each way, which is the best any such comparison can do and is
 * what the chip's own half-period warning is named after.
 */
static inline int64_t e_clock_delta(uint64_t a, uint64_t b) {
    uint64_t d = (b - a) & E_CLOCK_MASK;
    return (d & (1ull << (DW1000_TIME_CLOCK_BITS - 1)))
	? (int64_t)d - (int64_t)(1ull << DW1000_TIME_CLOCK_BITS)
	: (int64_t)d;
}


/*----------------------------------------------------------------------*/
/* Min/Max                                                              */
/*----------------------------------------------------------------------*/

#define EMU_MIN(a,b)				     \
    ({						     \
	__typeof__ (a) _emu_a = (a);		     \
	__typeof__ (b) _emu_b = (b);		     \
	_emu_a < _emu_b ? _emu_a : _emu_b;	     \
    })

#define EMU_MAX(a,b)				     \
    ({						     \
	__typeof__ (a) _emu_a = (a);		     \
	__typeof__ (b) _emu_b = (b);		     \
	_emu_a > _emu_b ? _emu_a : _emu_b;	     \
    })


#define E_MODE(type)		E_MODE_##type
#define E_MODE_NONE		{ 0 }
#define E_MODE_UNDEF		{ .read = 1, .write = 1             }
#define E_MODE_RO		{ .read = 1                         }
#define E_MODE_ROD		{ .read = 1                         }
#define E_MODE_SRW		{ .read = 1, .write = 1             }
#define E_MODE_RW		{ .read = 1, .write = 1             }
#define E_MODE_WO		{            .write = 1             }
#define E_MODE_RWCLR		{ .read = 1, .write = 1, .clear = 1 }

#define E_ACCESS(len, type)	{ .length = len, .mode = E_MODE(type) }
#define E_END			{ 0 }

#define E_DEFINE_PASSTHROUGH(type, ...)					\
    static uint8_t reg_p_##type[] = { __VA_ARGS__ }

/* Bits the host may not write, byte by byte, 1 meaning read only.
 *
 * A register's access mode is per segment, which is enough for every
 * register but SYS_STATUS: that one is write-one-to-clear as a whole and
 * yet holds a handful of bits the manual calls READ ONLY, which the chip
 * maintains and a host write must not disturb. UM 7.2.17 names them.
 */
#define E_DEFINE_READONLY(type, ...)					\
    static uint8_t reg_ro_##type[] = { __VA_ARGS__ }


#define E_DEFINE_REGISTER_SINGLE(type)			\
    uint8_t reg_##type[DW1000_LEN_##type]

#define E_DEFINE_REGISTER_DOUBLE(type)			\
    struct {						\
	uint8_t reg_##type##_A[DW1000_LEN_##type];	\
	uint8_t reg_##type##_B[DW1000_LEN_##type];	\
    }

#define E_DEFINE_ACCESS(type, ...)				\
    static struct e_access a_##type [] = { __VA_ARGS__, E_END }

void e_reg_read(struct dw1000_emulation *e, int idx,  size_t offset,
		uint8_t *data, size_t length,  bool system);
void e_reg_write(struct dw1000_emulation *e, int idx, size_t offset,
		 uint8_t *data, size_t length, bool system);

struct e_access {
    unsigned int length;
    struct {
	unsigned int read  : 1;
	unsigned int write : 1;
	unsigned int clear : 1;
    } mode;
};

struct e_register {
    unsigned int     size;
    struct e_access *access;
    uint8_t         *data[2];
    uint8_t         *passthrough;
    uint8_t         *readonly;
};

#define _E_GET_OVERRIDE_3(_1, _2, _3, NAME, ...) NAME
#define _E_GET_OVERRIDE_4(_1, _2, _3, _4, NAME, ...) NAME
#define _E_GET_OVERRIDE_5(_1, _2, _3, _4, _5, NAME, ...) NAME



/* Which of the two register sets a side is looking at.
 *
 * UM table 7 duplicates seven registers and four SYS_STATUS bits; the
 * host reaches the set named by HSRBP and the IC the one named by
 * ICRBP. Read from the model's own fields rather than from SYS_STATUS,
 * which is where the chip keeps them: selecting a set is what every
 * register access starts with, so asking a register for the answer
 * would be asking it of itself. The two are kept in step: the fields
 * are what decides, the status bits are what the host reads.
 *
 * With DIS_DRXB set, which is the reset value, there is one buffer and
 * both sides are on set 0.
 */
#define E_SWINGSET_IC(e)	((e)->dblbuff ? (e)->rbp_ic   : 0)
#define E_SWINGSET_HOST(e)	((e)->dblbuff ? (e)->rbp_host : 0)

/* ... and which one a given access uses. `system` is the IC. */
#define E_SWINGSET(e, system)						\
    ((system) ? E_SWINGSET_IC(e) : E_SWINGSET_HOST(e))



#define E_REG(e, idx)		(&(e)->reg[idx])
#define E_REG_KEY(e, idx)         E_REG(e, DW1000_REG_##idx)


/* The IC's view of a register's bytes: what the model itself reads and
 * writes. A host-side equivalent is not needed; the host only ever
 * arrives through e_reg_read()/e_reg_write().
 */
#define E_REG_DATA(e, idx)	(E_REG(e,idx)->data[E_SWINGSET_IC(e)])
#define E_REG_DATA_KEY(e, idx)	E_REG_DATA(e, DW1000_REG_##idx)


#define E_REG_ATTACH(_e, _key, _mode)					\
    _E_REG_ATTACH_##_mode(_e, _key)

#define _E_REG_ATTACH_SINGLE(_e, _idx)					\
    do {								\
	(_e)->reg[DW1000_REG_##_idx] = (struct e_register) {		\
	    .size   = sizeof((_e)->reg_##_idx),				\
	    .access = a_##_idx,						\
	    .data   =  { [0] = (_e)->reg_##_idx,			\
			 [1] = (_e)->reg_##_idx },			\
	};								\
    } while(0)

#define _E_REG_ATTACH_DOUBLE(_e, _idx)					\
    do {								\
	_Static_assert(sizeof((_e)->reg_##_idx##_A) ==			\
		       sizeof((_e)->reg_##_idx##_B),			\
		       "swing set register size differ");		\
	(_e)->reg[DW1000_REG_##_idx] = (struct e_register) {		\
	    .size = sizeof((_e)->reg_##_idx##_A),			\
	    .access = a_##_idx,						\
	    .data = { [0] = (_e)->reg_##_idx##_A,			\
		      [1] = (_e)->reg_##_idx##_B },			\
	};								\
    } while(0)

#define _E_REG_ATTACH_TODO(_e, _idx)					\
    do {								\
	(void) a_##_idx;						\
    } while(0)



/*
 * Host side reading/writing of registers
 */

#define E_REG_IC_WRITE_IDX(_e, _idx, _off, _data, _len)			\
    e_reg_write(_e, _idx, _off, _data, _len, true)
#define E_REG_IC_READ_IDX(_e, _data, _len, _idx, _off)			\
    e_reg_read(_e, _idx, _off, _data, _len, true)


#define E_REG_IC_READ_KEY(_e, _data, _len, _idx, _off)			\
    e_reg_read(_e, DW1000_REG_##_idx, DW1000_OFF_##_off, _data, _len, true)
#define E_REG_IC_WRITE_KEY(...)						\
    _E_GET_OVERRIDE_5(__VA_ARGS__, _E_REG_IC_WRITE_KEY_OFF,		\
                                   _E_REG_IC_WRITE_KEY_0)(__VA_ARGS__)
#define _E_REG_IC_WRITE_KEY_OFF(_e,  _data, _len, _idx, _off)		\
    e_reg_write(_e, DW1000_REG_##_idx, DW1000_OFF_##_off, _data, _len, true)
#define _E_REG_IC_WRITE_KEY_0(_e,  _data, _len, _idx)			\
    e_reg_write(_e, DW1000_REG_##_idx, 0, _data, _len, true)


#define E_REG_IC_WRITE16_KEY(...)					\
    _E_GET_OVERRIDE_4(__VA_ARGS__, _E_REG_IC_WRITE16_KEY_OFF,		\
                                   _E_REG_IC_WRITE16_KEY_0)(__VA_ARGS__)
#define E_REG_IC_WRITE32_KEY(...)					\
    _E_GET_OVERRIDE_4(__VA_ARGS__, _E_REG_IC_WRITE32_KEY_OFF,		\
                                   _E_REG_IC_WRITE32_KEY_0)(__VA_ARGS__)
#define E_REG_IC_WRITE40_KEY(...)					\
    _E_GET_OVERRIDE_4(__VA_ARGS__, _E_REG_IC_WRITE40_KEY_OFF,		\
                                   _E_REG_IC_WRITE40_KEY_0)(__VA_ARGS__)
#define E_REG_IC_READ16_KEY(...)					\
    _E_GET_OVERRIDE_3(__VA_ARGS__, _E_REG_IC_READ16_KEY_OFF,		\
                                   _E_REG_IC_READ16_KEY_0)(__VA_ARGS__)
#define E_REG_IC_READ32_KEY(...)					\
    _E_GET_OVERRIDE_3(__VA_ARGS__, _E_REG_IC_READ32_KEY_OFF,		\
                                   _E_REG_IC_READ32_KEY_0)(__VA_ARGS__)
#define E_REG_IC_READ40_KEY(...)					\
    _E_GET_OVERRIDE_3(__VA_ARGS__, _E_REG_IC_READ40_KEY_OFF,		\
                                   _E_REG_IC_READ40_KEY_0)(__VA_ARGS__)

#define _E_REG_WRITE_BITS_OFF(mode, e, val, idx, off, system)		\
    e_reg_write_##mode(e, val, DW1000_REG_##idx, DW1000_OFF_##off, system)
#define _E_REG_WRITE_BITS_0(mode, e, val, idx, system)			\
    e_reg_write_##mode(e, val, DW1000_REG_##idx, DW1000_OFF_NONE, system)
#define _E_REG_READ_BITS_OFF(mode, e, idx, off, system)			\
    e_reg_read_##mode(e, DW1000_REG_##idx, DW1000_OFF_##off, system)
#define _E_REG_READ_BITS_0(mode, e, idx, system)			\
    e_reg_read_##mode(e, DW1000_REG_##idx, DW1000_OFF_NONE, system)

#define _E_REG_IC_WRITE16_KEY_OFF(_e, v, idx, off)			\
    _E_REG_WRITE_BITS_OFF(16, _e, v, idx, off, true)
#define _E_REG_IC_WRITE16_KEY_0(_e, v, idx)				\
    _E_REG_WRITE_BITS_0(16, _e, v, idx, true)
#define _E_REG_IC_WRITE32_KEY_OFF(_e, v, idx, off)			\
    _E_REG_WRITE_BITS_OFF(32, _e, v, idx, off, true)
#define _E_REG_IC_WRITE32_KEY_0(_e, v, idx)				\
    _E_REG_WRITE_BITS_0(32, _e, v, idx, true)
#define _E_REG_IC_WRITE40_KEY_OFF(_e, v, idx, off)			\
    _E_REG_WRITE_BITS_OFF(40, _e, v, idx, off, true)
#define _E_REG_IC_WRITE40_KEY_0(_e, v, idx)				\
    _E_REG_WRITE_BITS_0(40, _e, v, idx, true)

#define _E_REG_IC_READ16_KEY_OFF(_e, idx, off)				\
    _E_REG_READ_BITS_OFF(16, _e, idx, off, true)
#define _E_REG_IC_READ16_KEY_0(_e, idx)					\
    _E_REG_READ_BITS_0(16, _e, idx, true)
#define _E_REG_IC_READ32_KEY_OFF(_e, idx, off)				\
    _E_REG_READ_BITS_OFF(32, _e, idx, off, true)
#define _E_REG_IC_READ32_KEY_0(_e, idx)					\
    _E_REG_READ_BITS_0(32, _e, idx, true)
#define _E_REG_IC_READ40_KEY_OFF(_e, idx, off)				\
    _E_REG_READ_BITS_OFF(40, _e, idx, off, true)
#define _E_REG_IC_READ40_KEY_0(_e, idx)					\
    _E_REG_READ_BITS_0(40, _e, idx, true)


/*
 * Host side reading/writing of registers
 */

#define E_REG_HOST_WRITE_IDX(_e, _idx, _off, _data, _len)	\
    e_reg_write(_e, _idx, _off, _data, _len, false)
#define E_REG_HOST_READ_IDX(_e, _idx, _off, _data, _len)			\
    e_reg_read(_e, _idx, _off, _data, _len, false)


#define E_REG_HOST_READ16_KEY(...)					\
    _E_GET_OVERRIDE_3(__VA_ARGS__, _E_REG_HOST_READ16_KEY_OFF,		\
                                   _E_REG_HOST_READ16_KEY_0)(__VA_ARGS__)
#define E_REG_HOST_READ32_KEY(...)					\
    _E_GET_OVERRIDE_3(__VA_ARGS__, _E_REG_HOST_READ32_KEY_OFF,		\
                                   _E_REG_HOST_READ32_KEY_0)(__VA_ARGS__)
#define E_REG_HOST_READ40_KEY(...)					\
    _E_GET_OVERRIDE_3(__VA_ARGS__, _E_REG_HOST_READ40_KEY_OFF,		\
                                   _E_REG_HOST_READ40_KEY_0)(__VA_ARGS__)

#define _E_REG_HOST_READ16_KEY_OFF(_e, idx, off)			\
    _E_REG_READ_BITS_OFF(16, _e, idx, off, false)
#define _E_REG_HOST_READ16_KEY_0(_e, idx)				\
    _E_REG_READ_BITS_0(16, _e, idx, false)
#define _E_REG_HOST_READ32_KEY_OFF(_e, idx, off)			\
    _E_REG_READ_BITS_OFF(32, _e, idx, off, false)
#define _E_REG_HOST_READ32_KEY_0(_e, idx)				\
    _E_REG_READ_BITS_0(32, _e, idx, false)
#define _E_REG_HOST_READ40_KEY_OFF(_e, idx, off)			\
    _E_REG_READ_BITS_OFF(40, _e, idx, off, false)
#define _E_REG_HOST_READ40_KEY_0(_e, idx)				\
    _E_REG_READ_BITS_0(40, _e, idx, false)








static inline
void e_reg_write_16(struct dw1000_emulation *e, uint16_t u16,
		    int idx, size_t offset, bool system) {
    uint16_t data = dw1000_cpu_to_le16(u16);
    e_reg_write(e, idx, offset, (uint8_t*) &data, sizeof(data), system);
}

static inline
void e_reg_write_32(struct dw1000_emulation *e, uint32_t u32,
		    int idx, size_t offset, bool system) {
    uint32_t data = dw1000_cpu_to_le32(u32);
    e_reg_write(e, idx, offset, (uint8_t*) &data, sizeof(data), system);
}

static inline
void e_reg_write_40(struct dw1000_emulation *e, uint64_t u40,
		    int idx, size_t offset, bool system) {
    uint64_t data = dw1000_cpu_to_le64(u40);
    e_reg_write(e, idx, offset, (uint8_t*) &data, 5, system);
}



static inline
uint16_t e_reg_read_16(struct dw1000_emulation *e, int idx,
		       size_t offset, bool system) {
    uint16_t data;
    e_reg_read(e, idx, offset, (uint8_t *)&data, sizeof(data), system);
    return dw1000_le16_to_cpu(data);
}

static inline
uint32_t e_reg_read_32(struct dw1000_emulation *e, int idx,
		       size_t offset, bool system) {
    uint32_t data;
    e_reg_read(e, idx, offset, (uint8_t *)&data, sizeof(data), system);
    return dw1000_le32_to_cpu(data);
}

static inline
uint64_t e_reg_read_40(struct dw1000_emulation *e, int idx,
		       size_t offset, bool system) {
    uint64_t data = 0;
    e_reg_read(e, idx, offset, (uint8_t *)&data, 5, system);
    return dw1000_le64_to_cpu(data);
}


#endif
