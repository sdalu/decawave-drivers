/*
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 */

#ifndef __DW1000_OTP_H__
#define __DW1000_OTP_H__


/*
 * OTP
 */
#define DW1000_OTP_EUID           0x000

#define DW1000_OTP_LDO_TUNE       0x004

#define DW1000_OTP_CHIP_ID        0x006
#define DW1000_MSK_CHIP_ID        0x000FFFFFu

#define DW1000_OTP_LOT_ID         0x007
#define DW1000_MSK_LOT_ID         0x0FFFFFFFu

#define DW1000_OTP_VBAT           0x008

#define DW1000_OTP_TEMP           0x009

#define DW1000_OTP_TX_POWER       0x010


#define DW1000_OTP_REV_XTRIM      0x01E

#endif
