/*
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 */

#ifndef __DW1000_REG_H__
#define __DW1000_REG_H__


#define DW1000_OFF_NONE              0

/*
 * registers
 */
#define DW1000_REG_DEV_ID         0x00
#define DW1000_REG_DEV_ID_LEN        4


#define DW1000_REG_EUI            0x01


#define DW1000_REG_PANADR         0x03


#define DW1000_REG_SYS_CFG        0x04
#define DW1000_FLG_SYS_CFG_FFEN       (1 <<  0)
#define DW1000_FLG_SYS_CFG_FFBC       (1 <<  1)
#define DW1000_FLG_SYS_CFG_FFAB       (1 <<  2)
#define DW1000_FLG_SYS_CFG_FFAD       (1 <<  3)
#define DW1000_FLG_SYS_CFG_FFAA       (1 <<  4)
#define DW1000_FLG_SYS_CFG_FFAM       (1 <<  5)
#define DW1000_FLG_SYS_CFG_FFAR       (1 <<  6)
#define DW1000_FLG_SYS_CFG_FFA4       (1 <<  7)
#define DW1000_FLG_SYS_CFG_FFA5       (1 <<  8)
#define DW1000_FLG_SYS_CFG_HIRQ_POL   (1 <<  9)
#define DW1000_FLG_SYS_CFG_SPI_EDGE   (1 << 10)
#define DW1000_FLG_SYS_CFG_DIS_FCE    (1 << 11)
#define DW1000_FLG_SYS_CFG_DIS_DRXB   (1 << 12)
#define DW1000_FLG_SYS_CFG_DIS_PHE    (1 << 13)
#define DW1000_FLG_SYS_CFG_DIS_RSDE   (1 << 14)
#define DW1000_FLG_SYS_CFG_FCS_INIT2F (1 << 15)
#define DW1000_SFT_SYS_CFG_PHR_MODE   16
#define DW1000_LEN_SYS_CFG_PHR_MODE   2
#define DW1000_MSK_SYS_CFG_PHR_MODE   (3 << 16)
#define DW1000_VAL_SYS_CFG_PHR_MODE_STD 0x0
#define DW1000_VAL_SYS_CFG_PHR_MODE_EXT 0x3
#define DW1000_FLG_SYS_CFG_DIS_STXP   (1 << 18)
#define DW1000_FLG_SYS_CFG_RXM110K    (1 << 22)
#define DW1000_FLG_SYS_CFG_RXWTOE     (1 << 28)
#define DW1000_FLG_SYS_CFG_RXAUTR     (1 << 29)
#define DW1000_FLG_SYS_CFG_AUTOACK    (1 << 30)
#define DW1000_FLG_SYS_CFG_AACKPEND   (1 << 31)
#define DW1000_MSK_SYS_CFG        (			\
    DW1000_FLG_SYS_CFG_FFEN       |			\
    DW1000_FLG_SYS_CFG_FFBC       |			\
    DW1000_FLG_SYS_CFG_FFAB       |			\
    DW1000_FLG_SYS_CFG_FFAD       |			\
    DW1000_FLG_SYS_CFG_FFAA       |			\
    DW1000_FLG_SYS_CFG_FFAM       |			\
    DW1000_FLG_SYS_CFG_FFAR       |			\
    DW1000_FLG_SYS_CFG_FFA4       |			\
    DW1000_FLG_SYS_CFG_FFA5       |			\
    DW1000_FLG_SYS_CFG_HIRQ_POL   |			\
    DW1000_FLG_SYS_CFG_SPI_EDGE   |			\
    DW1000_FLG_SYS_CFG_DIS_FCE    |			\
    DW1000_FLG_SYS_CFG_DIS_DRXB   |			\
    DW1000_FLG_SYS_CFG_DIS_PHE    |			\
    DW1000_FLG_SYS_CFG_DIS_RSDE   |			\
    DW1000_FLG_SYS_CFG_FCS_INIT2F |			\
    DW1000_MSK_SYS_CFG_PHR_MODE   |			\
    DW1000_FLG_SYS_CFG_DIS_STXP   |			\
    DW1000_FLG_SYS_CFG_RXM110K    |			\
    DW1000_FLG_SYS_CFG_RXWTOE     |			\
    DW1000_FLG_SYS_CFG_RXAUTR     |			\
    DW1000_FLG_SYS_CFG_AUTOACK    |			\
    DW1000_FLG_SYS_CFG_AACKPEND   )
#define DW1000_MSK_SYS_CFG_FF_ALL (			\
    DW1000_FLG_SYS_CFG_FFBC       |			\
    DW1000_FLG_SYS_CFG_FFAB       |			\
    DW1000_FLG_SYS_CFG_FFAD       |			\
    DW1000_FLG_SYS_CFG_FFAA       |			\
    DW1000_FLG_SYS_CFG_FFAM       |			\
    DW1000_FLG_SYS_CFG_FFAR       |			\
    DW1000_FLG_SYS_CFG_FFA4       |			\
    DW1000_FLG_SYS_CFG_FFA5       )
    

#define DW1000_REG_SYS_TIME       0x06
#define DW1000_LEN_SYS_TIME       5

#define DW1000_REG_TX_FCTRL       0x08
#define DW1000_SFT_TX_FCTRL_TFLEN  0
#define DW1000_SFT_TX_FCTRL_TXBR  13
#define DW1000_SFT_TX_FCTRL_TR    15
#define DW1000_SFT_TX_FCTRL_TXPRF 16
#define DW1000_SFT_TX_FCTRL_TXPSR 18
#define DW1000_SFT_TX_FCTRL_PE    20
#define DW1000_SFT_TX_FCTRL_TXBOFFS 22
#define DW1000_SFT_TX_FCTRL_PLEN  DW1000_SFT_TX_FCTRL_TXPSR
#define DW1000_FLG_TX_FCTRL_TR    (1 << 15)

#define DW1000_REG_TX_BUFFER      0x09

#define DW1000_REG_DX_TIME        0x0A

#define DW1000_REG_RX_FWTO        0x0C

#define DW1000_REG_SYS_CTRL       0x0D
#define DW1000_OFF_SYS_CTRL       0x00
#define DW1000_SFT_SYS_CTRL_HRBPT     24
#define DW1000_FLG_SYS_CTRL_SFCST     (1 <<  0)
#define DW1000_FLG_SYS_CTRL_TXSTRT    (1 <<  1)
#define DW1000_FLG_SYS_CTRL_TXDLYS    (1 <<  2)
#define DW1000_FLG_SYS_CTRL_CANSFCS   (1 <<  3)
#define DW1000_FLG_SYS_CTRL_TRXOFF    (1 <<  6)
#define DW1000_FLG_SYS_CTRL_WAIT4RESP (1 <<  7)
#define DW1000_FLG_SYS_CTRL_RXENAB    (1 <<  8)
#define DW1000_FLG_SYS_CTRL_RXDLYE    (1 <<  9)
#define DW1000_FLG_SYS_CTRL_HRBPT     (1 << 24)

#define DW1000_REG_SYS_MASK       0x0E
#define DW1000_FLG_SYS_MASK_MCPLOCK   (1 <<  1)
#define DW1000_FLG_SYS_MASK_MESYNCR   (1 <<  2)
#define DW1000_FLG_SYS_MASK_MAAT      (1 <<  3)
#define DW1000_FLG_SYS_MASK_MTXFRB    (1 <<  4)
#define DW1000_FLG_SYS_MASK_MTXPRS    (1 <<  5)
#define DW1000_FLG_SYS_MASK_MTXPHS    (1 <<  6)
#define DW1000_FLG_SYS_MASK_MTXFRS    (1 <<  7)
#define DW1000_FLG_SYS_MASK_MRXPRD    (1 <<  8)
#define DW1000_FLG_SYS_MASK_MRXSFDD   (1 <<  9)
#define DW1000_FLG_SYS_MASK_MLDEDONE  (1 << 10)
#define DW1000_FLG_SYS_MASK_MRXPHD    (1 << 11)
#define DW1000_FLG_SYS_MASK_MRXPHE    (1 << 12)
#define DW1000_FLG_SYS_MASK_MRXDFR    (1 << 13)
#define DW1000_FLG_SYS_MASK_MRXFCG    (1 << 14)
#define DW1000_FLG_SYS_MASK_MRXFCE    (1 << 15)
#define DW1000_FLG_SYS_MASK_MRXRFSL   (1 << 16)
#define DW1000_FLG_SYS_MASK_MRXRFTO   (1 << 17)
#define DW1000_FLG_SYS_MASK_MLDEERR   (1 << 18)
#define DW1000_FLG_SYS_MASK_MRXOVRR   (1 << 20)
#define DW1000_FLG_SYS_MASK_MRXPTO    (1 << 21)
#define DW1000_FLG_SYS_MASK_MGPIOIRQ  (1 << 22)
#define DW1000_FLG_SYS_MASK_MSLP2INIT (1 << 23)
#define DW1000_FLG_SYS_MASK_MRFPLLLL  (1 << 24)
#define DW1000_FLG_SYS_MASK_MCPLLLL   (1 << 25)
#define DW1000_FLG_SYS_MASK_MRXSFDTO  (1 << 26)
#define DW1000_FLG_SYS_MASK_MHPDWARN  (1 << 27)
#define DW1000_FLG_SYS_MASK_MTXBERR   (1 << 28)
#define DW1000_FLG_SYS_MASK_MAFFREJ   (1 << 29)
#define DW1000_MSK_SYS_MASK (		\
     DW1000_FLG_SYS_MASK_MCPLOCK   |	\
     DW1000_FLG_SYS_MASK_MESYNCR   |	\
     DW1000_FLG_SYS_MASK_MAAT      |	\
     DW1000_FLG_SYS_MASK_MTXFRB    |	\
     DW1000_FLG_SYS_MASK_MTXPRS    |	\
     DW1000_FLG_SYS_MASK_MTXPHS    |	\
     DW1000_FLG_SYS_MASK_MTXFRS    |	\
     DW1000_FLG_SYS_MASK_MRXPRD    |	\
     DW1000_FLG_SYS_MASK_MRXSFDD   |	\
     DW1000_FLG_SYS_MASK_MLDEDONE  |	\
     DW1000_FLG_SYS_MASK_MRXPHD    |	\
     DW1000_FLG_SYS_MASK_MRXPHE    |	\
     DW1000_FLG_SYS_MASK_MRXDFR    |	\
     DW1000_FLG_SYS_MASK_MRXFCG    |	\
     DW1000_FLG_SYS_MASK_MRXFCE    |	\
     DW1000_FLG_SYS_MASK_MRXRFSL   |	\
     DW1000_FLG_SYS_MASK_MRXRFTO   |	\
     DW1000_FLG_SYS_MASK_MLDEERR   |	\
     DW1000_FLG_SYS_MASK_MRXOVRR   |	\
     DW1000_FLG_SYS_MASK_MRXPTO    |	\
     DW1000_FLG_SYS_MASK_MGPIOIRQ  |	\
     DW1000_FLG_SYS_MASK_MSLP2INIT |	\
     DW1000_FLG_SYS_MASK_MRFPLLLL  |	\
     DW1000_FLG_SYS_MASK_MCPLLLL   |	\
     DW1000_FLG_SYS_MASK_MRXSFDTO  |	\
     DW1000_FLG_SYS_MASK_MHPDWARN  |	\
     DW1000_FLG_SYS_MASK_MTXBERR   |	\
     DW1000_FLG_SYS_MASK_MAFFREJ   )


#define DW1000_REG_SYS_STATUS     0x0F
#define DW1000_SFT_SYS_STATUS_HPDWARN 27
#define DW1000_SFT_SYS_STATUS_HSRBP   30
#define DW1000_SFT_SYS_STATUS_ICRBP   31
#define DW1000_SFT_SYS_STATUS_TXPUTE  34
#define DW1000_FLG_SYS_STATUS_IRQS    (1 <<  0)
#define DW1000_FLG_SYS_STATUS_CPLOCK  (1 <<  1)
#define DW1000_FLG_SYS_STATUS_ESYNCR  (1 <<  2)
#define DW1000_FLG_SYS_STATUS_AAT     (1 <<  3)
#define DW1000_FLG_SYS_STATUS_TXFRB   (1 <<  4)
#define DW1000_FLG_SYS_STATUS_TXPRS   (1 <<  5)
#define DW1000_FLG_SYS_STATUS_TXPHS   (1 <<  6)
#define DW1000_FLG_SYS_STATUS_TXFRS   (1 <<  7)
#define DW1000_FLG_SYS_STATUS_RXPRD   (1 <<  8)
#define DW1000_FLG_SYS_STATUS_RXSFDD  (1 <<  9)
#define DW1000_FLG_SYS_STATUS_LDEDONE (1 << 10)
#define DW1000_FLG_SYS_STATUS_RXPHD   (1 << 11)
#define DW1000_FLG_SYS_STATUS_RXPHE   (1 << 12)
#define DW1000_FLG_SYS_STATUS_RXDFR   (1 << 13)
#define DW1000_FLG_SYS_STATUS_RXFCG   (1 << 14)
#define DW1000_FLG_SYS_STATUS_RXFCE   (1 << 15)
#define DW1000_FLG_SYS_STATUS_RXRFSL  (1 << 16)
#define DW1000_FLG_SYS_STATUS_RXRFTO  (1 << 17)
#define DW1000_FLG_SYS_STATUS_LDEERR  (1 << 18)
#define DW1000_FLG_SYS_STATUS_RXOVRR  (1 << 20)
#define DW1000_FLG_SYS_STATUS_RXPTO   (1 << 21)
#define DW1000_FLG_SYS_STATUS_GPIOIRQ (1 << 22)
#define DW1000_FLG_SYS_STATUS_SLP2INIT (1 << 23)
#define DW1000_FLG_SYS_STATUS_RFPL_LL (1 << 24)
#define DW1000_FLG_SYS_STATUS_CLKPLL_LL (1 << 25)
#define DW1000_FLG_SYS_STATUS_RXSFDTO (1 << 26)
#define DW1000_FLG_SYS_STATUS_HPDWARN (1 << 27)
#define DW1000_FLG_SYS_STATUS_TXBERR  (1 << 28)
#define DW1000_FLG_SYS_STATUS_AFFREJ  (1 << 29)
#define DW1000_FLG_SYS_STATUS_HSRBP   (1 << 30)
#define DW1000_FLG_SYS_STATUS_ICRBP   (1 << 31)
#define DW1000_FLG_SYS_STATUS_RXRSCS  (1 << 32)
#define DW1000_FLG_SYS_STATUS_RXPREJ  (1 << 33)
#define DW1000_FLG_SYS_STATUS_TXPUTE  (1 << 34)
#define DW1000_MSK_SYS_STATUS_ALL_DBLBUFF	\
    (DW1000_FLG_SYS_STATUS_RXDFR   |		\
     DW1000_FLG_SYS_STATUS_RXFCG)
#define DW1000_MSK_SYS_STATUS_ALL_RX_GOOD	\
    (DW1000_FLG_SYS_STATUS_RXDFR   |		\
     DW1000_FLG_SYS_STATUS_RXFCG   |		\
     DW1000_FLG_SYS_STATUS_RXPRD   |		\
     DW1000_FLG_SYS_STATUS_RXSFDD  |		\
     DW1000_FLG_SYS_STATUS_RXPHD   |		\
     DW1000_FLG_SYS_STATUS_LDEDONE)
#define DW1000_MSK_SYS_STATUS_ALL_RX_ERR	\
    (DW1000_FLG_SYS_STATUS_RXPHE   |		\
     DW1000_FLG_SYS_STATUS_RXFCE   |		\
     DW1000_FLG_SYS_STATUS_RXRFSL  |		\
     DW1000_FLG_SYS_STATUS_RXSFDTO |		\
     DW1000_FLG_SYS_STATUS_AFFREJ  |		\
     DW1000_FLG_SYS_STATUS_LDEERR)
#define DW1000_MSK_SYS_STATUS_ALL_RX_TO		\
    (DW1000_FLG_SYS_STATUS_RXRFTO  |		\
     DW1000_FLG_SYS_STATUS_RXPTO)
#define DW1000_MSK_SYS_STATUS_ALL_RX		\
    (DW1000_MSK_SYS_STATUS_ALL_RX_GOOD |	\
     DW1000_MSK_SYS_STATUS_ALL_RX_TO   |	\
     DW1000_MSK_SYS_STATUS_ALL_RX_ERR)
#define DW1000_MSK_SYS_STATUS_ALL_RX_DONE	\
    (DW1000_FLG_SYS_STATUS_RXFCG       |	\
     DW1000_MSK_SYS_STATUS_ALL_RX_TO   |	\
     DW1000_MSK_SYS_STATUS_ALL_RX_ERR)
#define DW1000_MSK_SYS_STATUS_ALL_TX		\
    (DW1000_FLG_SYS_STATUS_AAT     |		\
     DW1000_FLG_SYS_STATUS_TXFRB   |		\
     DW1000_FLG_SYS_STATUS_TXPRS   |		\
     DW1000_FLG_SYS_STATUS_TXPHS   |		\
     DW1000_FLG_SYS_STATUS_TXFRS)

#define DW1000_REG_RX_FINFO       0x10
#define DW1000_SFT_RX_FINFO_RXFLEN  		 0
#define DW1000_SFT_RX_FINFO_RXNSPL 		11
#define DW1000_SFT_RX_FINFO_RXBR   		13
#define DW1000_SFT_RX_FINFO_RNG    		15
#define DW1000_SFT_RX_FINFO_RXPRF  		16
#define DW1000_SFT_RX_FINFO_RXPSR 		18
#define DW1000_SFT_RX_FINFO_RXPACC    		20

#define DW1000_MSK_RX_FINFO_RXFLEN 0x7F
#define DW1000_MSK_RX_FINFO_RXFLEN_EXT 0x03FF
#define DW1000_FLG_RX_FINFO_RNG    (1 << 15)
#define DW1000_MSK_RX_FINFO_RXPACC (0xFFF << DW1000_SFT_RX_FINFO_RXPACC)


#define DW1000_REG_RX_BUFFER      0x11

#define DW1000_REG_RX_FQUAL       0x12
#define DW1000_OFF_RX_FQUAL_STD_NOISE 0x00
#define DW1000_OFF_RX_FQUAL_FP_AMPL2  0x02
#define DW1000_OFF_RX_FQUAL_FP_AMPL3  0x04
#define DW1000_OFF_RX_FQUAL_CIR_PWR   0x06

#define DW1000_REG_RX_TTCKI       0x13
#define DW1000_LEN_RX_TTCKI       4

#define DW1000_REG_RX_TTCKO       0x14
#define DW1000_LEN_RX_TTCKO       5
#define DW1000_SFT_RX_TTCKO_RXTOFS  0
#define DW1000_LEN_RX_TTCKO_RXTOFS  19
#define DW1000_MSK_RX_TTCKO_RXTOFS  (0x7FFFF << DW1000_SFT_RX_TTCKO_RXTOFS)


#define DW1000_REG_RX_TIME        0x15
#define DW1000_OFF_RX_TIME_RX_STAMP 0
#define DW1000_OFF_RX_TIME_FP_INDEX 5
#define DW1000_OFF_RX_TIME_FP_AMPL1 7
#define DW1000_OFF_RX_TIME_RX_RAWST 9

#define DW1000_REG_TX_TIME        0x17
#define DW1000_OFF_TX_TIME_TX_STAMP     0
#define DW1000_OFF_TX_TIME_TX_RAWST     5

#define DW1000_REG_TX_ANTD        0x18

#define DW1000_REG_SYS_STATE      0x19

#define DW1000_REG_ACK_RESP_T     0x1A

#define DW1000_SFT_ACK_RESP_T_W4R_TIM 0
#define DW1000_MSK_ACK_RESP_T_W4R_TIM (0xFF << 0)
#define DW1000_SFT_ACK_RESP_T_ACK_TIM 24
#define DW1000_MSK_ACK_RESP_T_ACK_TIM (0xFFFFF << 24)


#define DW1000_REG_RX_SNIFF       0x1D

#define DW1000_REG_TX_POWER       0x1E

#define DW1000_REG_CHAN_CTRL      0x1F
#define DW1000_SFT_CHAN_CTRL_TX_CHAN   0
#define DW1000_SFT_CHAN_CTRL_RX_CHAN   4
#define DW1000_SFT_CHAN_CTRL_DWSFD    17
#define DW1000_SFT_CHAN_CTRL_RXPRF    18
#define DW1000_SFT_CHAN_CTRL_TNSSFD   20
#define DW1000_SFT_CHAN_CTRL_RNSSFD   21
#define DW1000_SFT_CHAN_CTRL_TX_PCODE 22
#define DW1000_SFT_CHAN_CTRL_RX_PCODE 27
#define DW1000_MSK_CHAN_CTRL_TX_CHAN  (0xF  <<  0)
#define DW1000_MSK_CHAN_CTRL_RX_CHAN  (0xF  <<  4)
#define DW1000_FLG_CHAN_CTRL_DWSFD    (   1 << 17)
#define DW1000_MSK_CHAN_CTRL_RXPRF    (0x3  << 18)
#define DW1000_FLG_CHAN_CTRL_TNSSFD   (   1 << 20)
#define DW1000_FLG_CHAN_CTRL_RNSSFD   (   1 << 21)
#define DW1000_MSK_CHAN_CTRL_TX_PCODE (0x1F << 22)
#define DW1000_MSK_CHAN_CTRL_RX_PCODE (0x1F << 27)

#define DW1000_REG_USR_SFD        0x21
#define DW1000_OFF_USR_SFD_LENGTH 0

#define DW1000_REG_AGC_CTRL       0x23
#define DW1000_OFF_AGC_CTRL1      0x02
#define DW1000_OFF_AGC_TUNE1      0x04
#define DW1000_OFF_AGC_TUNE2      0x0C
#define DW1000_OFF_AGC_TUNE3      0x12
#define DW1000_OFF_AGC_STAT1      0x1E

#define DW1000_REG_EXT_SYNC       0x24
#define DW1000_OFF_EC_CTRL        0x00
#define DW1000_FLG_EC_CTRL_PLLLDT 0x00000004
#define DW1000_OFF_EC_RXTC        0x04
#define DW1000_OFF_EC_GOLP        0x08

#define DW1000_REG_ACC_MEM        0x25

#define DW1000_REG_GPIO_CTRL      0x26
#define DW1000_OFF_GPIO_MODE      0x00
#define DW1000_OFF_GPIO_DIR       0x08
#define DW1000_OFF_GPIO_DOUT      0x0C
#define DW1000_OFF_GPIO_IRQE      0x10
#define DW1000_OFF_GPIO_ISEN      0x14
#define DW1000_OFF_GPIO_IMODE     0x18
#define DW1000_OFF_GPIO_IBES      0x1C
#define DW1000_OFF_GPIO_ICLR      0x20
#define DW1000_OFF_GPIO_IDBE      0x24
#define DW1000_OFF_GPIO_RAW       0x28
#define DW1000_MSK_GPIO_MSGP0     (0x3 <<  6)
#define DW1000_MSK_GPIO_MSGP1     (0x3 <<  8)
#define DW1000_MSK_GPIO_MSGP2     (0x3 << 10)
#define DW1000_MSK_GPIO_MSGP3     (0x3 << 12)
#define DW1000_MSK_GPIO_MSGP4     (0x3 << 14)
#define DW1000_MSK_GPIO_MSGP5     (0x3 << 16)
#define DW1000_MSK_GPIO_MSGP6     (0x3 << 18)
#define DW1000_MSK_GPIO_MSGP7     (0x3 << 20)
#define DW1000_MSK_GPIO_MSGP8     (0x3 << 22)
#define DW1000_SFT_GPIO_MSGP0      6
#define DW1000_SFT_GPIO_MSGP1      8
#define DW1000_SFT_GPIO_MSGP2     10
#define DW1000_SFT_GPIO_MSGP3     12
#define DW1000_SFT_GPIO_MSGP4     14
#define DW1000_SFT_GPIO_MSGP5     16
#define DW1000_SFT_GPIO_MSGP6     18
#define DW1000_SFT_GPIO_MSGP7     20
#define DW1000_SFT_GPIO_MSGP8     22
#define DW1000_VAL_GPIO_0_GPIO    0
#define DW1000_VAL_GPIO_0_RXOKLED 1
#define DW1000_VAL_GPIO_1_GPIO    0
#define DW1000_VAL_GPIO_1_SFDLED  1
#define DW1000_VAL_GPIO_2_GPIO    0
#define DW1000_VAL_GPIO_2_RXLED   1
#define DW1000_VAL_GPIO_3_GPIO    0
#define DW1000_VAL_GPIO_3_TXLED   1
#define DW1000_VAL_GPIO_4_GPIO    0
#define DW1000_VAL_GPIO_4_EXTPA   1
#define DW1000_VAL_GPIO_5_GPIO    0
#define DW1000_VAL_GPIO_5_EXTTXE  1
#define DW1000_VAL_GPIO_6_GPIO    0
#define DW1000_VAL_GPIO_6_EXTRXE  1
#define DW1000_VAL_GPIO_7_SYNC    0
#define DW1000_VAL_GPIO_7_GPIO    1
#define DW1000_VAL_GPIO_8_IRQ     0
#define DW1000_VAL_GPIO_8_GPIO    1


#define DW1000_REG_DRX_CONF       0x27
#define DW1000_OFF_DRX_TUNE0B     0x02
#define DW1000_OFF_DRX_TUNE1A     0x04
#define DW1000_OFF_DRX_TUNE1B     0x06
#define DW1000_OFF_DRX_TUNE2      0x08
#define DW1000_OFF_DRX_SFDTOC     0x20
#define DW1000_OFF_DRX_PRETOC     0x24
#define DW1000_OFF_DRX_TUNE4H     0x26
#define DW1000_OFF_DRX_RXPACC_NOSAT 0x2C

#define DW1000_REG_RF_CONF        0x28
#define DW1000_OFF_RF_CONF        0x00
#define DW1000_OFF_RF_RXCTRLH     0x0B
#define DW1000_OFF_RF_TXCTRL      0x0C
#define DW1000_OFF_RF_STATUS      0x2C
#define DW1000_OFF_RF_LDOTUNE     0x30

#define DW1000_REG_TX_CAL         0x2A
#define DW1000_OFF_TC_SARC	  0x00
#define DW1000_FLG_TC_SARC_SAR_CTRL (1 << 0)
#define DW1000_OFF_TC_SARL        0x03
#define DW1000_OFF_TC_SARW        0x06
#define DW1000_OFF_TC_PGDELAY     0x0B
#define DW1000_OFF_TC_PGTEST      0x0C

#define DW1000_REG_FS_CTRL        0x2B
#define DW1000_OFF_FS_PLLCFG      0x07
#define DW1000_OFF_FS_PLLTUNE     0x0B
#define DW1000_OFF_FS_XTALT       0x0E

#define DW1000_REG_AON            0x2C
#define DW1000_OFF_AON_WCFG       0x00
#define DW1000_FLG_AON_WCFG_ONW_LLDE  0x0800
#define DW1000_FLG_AON_WCFG_ONW_LLDO  0x1000
#define DW1000_OFF_AON_CTRL       0x02
#define DW1000_OFF_AON_RDAT       0x03
#define DW1000_OFF_AON_ADDR       0x04
#define DW1000_OFF_AON_CFG0       0x06
#define DW1000_OFF_AON_CFG1       0x0A

#define DW1000_REG_OTP_IF         0x2D
#define DW1000_OFF_OTP_WDAT       0x00
#define DW1000_OFF_OTP_ADDR       0x04
#define DW1000_OFF_OTP_CTRL       0x06
#define DW1000_FLG_OTP_CTRL_LDELOAD 0x8000
#define DW1000_OFF_OTP_STAT       0x08
#define DW1000_OFF_OTP_RDAT       0x0A
#define DW1000_OFF_OTP_SRDAT      0x0E
#define DW1000_OFF_OTP_SF         0x12
#define DW1000_FLG_OTP_SF_OPS_KICK 0x01
#define DW1000_FLG_OTP_SF_LDO_KICK 0x02

#define DW1000_REG_LDE_IF         0x2E
#define DW1000_OFF_LDE_THRESH     0x0000
#define DW1000_OFF_LDE_CFG1       0x0806
#define DW1000_OFF_LDE_PPINDX     0x1000
#define DW1000_OFF_LDE_PPAMPL     0x1002
#define DW1000_OFF_LDE_RXANTD     0x1804
#define DW1000_OFF_LDE_CFG2       0x1806
#define DW1000_OFF_LDE_REPC       0x2804
#define DW1000_REG_DIG_DIAG       0x2F

#define DW1000_REG_PMSC           0x36
#define DW1000_OFF_PMSC_CTRL0     0x00 
#define DW1000_OFF_PMSC_CTRL0_SOFTRESET     	0x03

#define DW1000_FLG_PMSC_CTRL0_FACE 	(1 <<  6)
#define DW1000_FLG_PMSC_CTRL0_ADCCE 	(1 << 10)
#define DW1000_FLG_PMSC_CTRL0_AMCE 	(1 << 15)
#define DW1000_FLG_PMSC_CTRL0_GPCE 	(1 << 16)
#define DW1000_FLG_PMSC_CTRL0_GPRN 	(1 << 17)
#define DW1000_FLG_PMSC_CTRL0_GPDCE 	(1 << 18)
#define DW1000_FLG_PMSC_CTRL0_GPDRN 	(1 << 19)
#define DW1000_FLG_PMSC_CTRL0_KHZCLKEN 	(1 << 23)
#define DW1000_OFF_PMSC_CTRL1     		0x04 
#define DW1000_FLG_PMSC_CTRL1_LDERUNE 	(1 << 17) 
#define DW1000_OFF_PMSC_RES1      		0x08 
#define DW1000_OFF_PMSC_SNOZT     		0x0C 
#define DW1000_OFF_PMSC_RES2      		0x10 
#define DW1000_OFF_PMSC_TXSEQ     		0x26
#define DW1000_OFF_PMSC_LEDC      		0x28
#define DW1000_MSK_PMSC_LEDC_BLINK_TIM (0xFF << 0)
#define DW1000_FLG_PMSC_LEDC_BLNKEN    (1 << 8)
#define DW1000_MSK_PMSC_LEDC_BLNKNOW   (0xF << 16)
#define DW1000_SFT_PMSC_LEDC_BLNKNOW   16

#endif
