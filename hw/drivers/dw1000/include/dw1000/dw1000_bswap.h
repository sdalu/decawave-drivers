/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DW1000_BSWAP_H
#define DW1000_BSWAP_H

#if defined(__cplusplus)
extern "C" {
#endif

#if !defined(__BYTE_ORDER__)
#error __BYTE_ORDER is not defined
#endif
#if ! ((__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) ||	\
       (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__   ))
#error __BYTE_ORDER__ of unknown value
#endif

    
#define DW1000_BSWAP_16(x)						\
    (uint16_t)((((x) & 0xFF00) >> 8) |					\
	       (((x) & 0x00FF) << 8))
#define DW1000_BSWAP_32(x)						\
    (uint32_t)((((x) & 0xFF000000UL) >> 24UL) |				\
	       (((x) & 0x00FF0000UL) >>  8UL) |				\
	       (((x) & 0x0000FF00UL) <<  8UL) |				\
	       (((x) & 0x000000FFUL) << 24UL))
#define DW1000_BSWAP_64(x)						\
    (uint64_t)((((x) & 0xFF00000000000000UL) >> 56UL) |			\
	       (((x) & 0x00FF000000000000UL) >> 40UL) |			\
	       (((x) & 0x0000FF0000000000UL) >> 24UL) |			\
	       (((x) & 0x000000FF00000000UL) >>  8UL) |			\
	       (((x) & 0x00000000FF000000UL) <<  8UL) |			\
	       (((x) & 0x0000000000FF0000UL) << 24UL) |			\
	       (((x) & 0x000000000000FF00UL) << 40UL) |			\
	       (((x) & 0x00000000000000FFUL) << 56UL))

    
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define dw1000_le16_to_cpu(x)           dw1000_bswap_16(x)
#define dw1000_le32_to_cpu(x)           dw1000_bswap_32(x)
#define dw1000_le64_to_cpu(x)           dw1000_bswap_64(x)
#define dw1000_be16_to_cpu(x)           (x)
#define dw1000_be32_to_cpu(x)           (x)
#define dw1000_be64_to_cpu(x)           (x)
#define dw1000_cpu_to_le16(x)           dw1000_bswap_16(x)
#define dw1000_cpu_to_le32(x)           dw1000_bswap_32(x)
#define dw1000_cpu_to_le64(x)           dw1000_bswap_64(x)
#define dw1000_cpu_to_be16(x)           (x)
#define dw1000_cpu_to_be32(x)           (x)
#define dw1000_cpu_to_be64(x)           (x)
#define DW1000_LE16_TO_CPU(x)           DW1000_BSWAP_16(x)
#define DW1000_LE32_TO_CPU(x)           DW1000_BSWAP_32(x)
#define DW1000_LE64_TO_CPU(x)           DW1000_BSWAP_64(x)
#define DW1000_BE16_TO_CPU(x)           (x)
#define DW1000_BE32_TO_CPU(x)           (x)
#define DW1000_BE64_TO_CPU(x)           (x)
#define DW1000_CPU_TO_LE16(x)           DW1000_BSWAP_16(x)
#define DW1000_CPU_TO_LE32(x)           DW1000_BSWAP_32(x)
#define DW1000_CPU_TO_LE64(x)           DW1000_BSWAP_64(x)
#define DW1000_CPU_TO_BE16(x)           (x)
#define DW1000_CPU_TO_BE32(x)           (x)
#define DW1000_CPU_TO_BE64(x)           (x)
#endif

   
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define dw1000_le16_to_cpu(x)           (x)
#define dw1000_le32_to_cpu(x)           (x)
#define dw1000_le64_to_cpu(x)           (x)
#define dw1000_be16_to_cpu(x)           dw1000_bswap_16(x)
#define dw1000_be32_to_cpu(x)           dw1000_bswap_32(x)
#define dw1000_be64_to_cpu(x)           dw1000_bswap_64(x)
#define dw1000_cpu_to_le16(x)           (x)
#define dw1000_cpu_to_le32(x)           (x)
#define dw1000_cpu_to_le64(x)           (x)
#define dw1000_cpu_to_be16(x)           dw1000_bswap_16(x)
#define dw1000_cpu_to_be32(x)           dw1000_bswap_32(x)
#define dw1000_cpu_to_be64(x)           dw1000_bswap_64(x)
#define DW1000_LE16_TO_CPU(x)           (x)
#define DW1000_LE32_TO_CPU(x)           (x)
#define DW1000_LE64_TO_CPU(x)           (x)
#define DW1000_BE16_TO_CPU(x)           DW1000_BSWAP_16(x)
#define DW1000_BE32_TO_CPU(x)           DW1000_BSWAP_32(x)
#define DW1000_BE64_TO_CPU(x)           DW1000_BSWAP_64(x)
#define DW1000_CPU_TO_LE16(x)           (x)
#define DW1000_CPU_TO_LE32(x)           (x)
#define DW1000_CPU_TO_LE64(x)           (x)
#define DW1000_CPU_TO_BE16(x)           DW1000_BSWAP_16(x)
#define DW1000_CPU_TO_BE32(x)           DW1000_BSWAP_32(x)
#define DW1000_CPU_TO_BE64(x)           DW1000_BSWAP_64(x)
#endif

    
static inline uint16_t dw1000_bswap_16(const uint16_t x)
    __attribute__ ((warn_unused_result))
    __attribute__ ((const))
    __attribute__ ((always_inline));

static inline uint16_t dw1000_bswap_16(const uint16_t x) {
    if (__builtin_constant_p(x))
	return DW1000_BSWAP_16(x);

    uint8_t                             tmp;
    union { uint16_t x; uint8_t b[2]; } data;
    
    data.x    = x;
    tmp       = data.b[0];
    data.b[0] = data.b[1];
    data.b[1] = tmp;
    
    return data.x;
}

static inline uint32_t dw1000_bswap_32(const uint32_t x)
    __attribute__ ((warn_unused_result))
    __attribute__ ((const))
    __attribute__ ((always_inline));


static inline uint32_t dw1000_bswap_32(const uint32_t x) {
    if (__builtin_constant_p(x))
	return DW1000_BSWAP_32(x);
    
    uint8_t                             tmp;
    union { uint32_t x; uint8_t b[4]; } data;
    
    data.x    = x;    
    tmp       = data.b[0];
    data.b[0] = data.b[3];
    data.b[3] = tmp;
    tmp       = data.b[1];
    data.b[1] = data.b[2];
    data.b[2] = tmp;
    
    return data.x;
}
    
static inline uint64_t dw1000_bswap_64(const uint64_t x)
    __attribute__ ((warn_unused_result))
    __attribute__ ((const))
    __attribute__ ((always_inline));


static inline uint64_t dw1000_bswap_64(const uint64_t x) {
    if (__builtin_constant_p(x))
	return DW1000_BSWAP_64(x);
    
    uint8_t                             tmp;
    union { uint64_t x; uint8_t b[8]; } data;

    data.x    = x;    
    tmp       = data.b[0];
    data.b[0] = data.b[7];
    data.b[7] = tmp;
    tmp       = data.b[1];
    data.b[1] = data.b[6];
    data.b[6] = tmp;
    tmp       = data.b[2];
    data.b[2] = data.b[5];
    data.b[5] = tmp;
    tmp       = data.b[3];
    data.b[3] = data.b[4];
    data.b[4] = tmp;
    
    return data.x;
}
    
static inline void dw1000_bswap_n(void* const data, uint8_t len)
    __attribute__ ((nonnull (1)));

static inline void dw1000_bswap_n(void* const data, uint8_t len) {
    uint8_t* ptr = (uint8_t*)data;

    for ( ; len > 1 ; ptr++, len -= 2 ) {
	uint8_t tmp      = *ptr;
	*ptr             = *(ptr + len - 1);
	*(ptr + len - 1) = tmp;
    }
}
    
#if defined(__cplusplus)
}
#endif

#endif
