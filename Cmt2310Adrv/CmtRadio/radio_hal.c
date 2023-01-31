/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-29     Tobby       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_spi.h"
#include <string.h>
#include "pin_config.h"
#include "base_types.h"
#include "cmt2310a_Reg.h"
#include "cmt2310a_def.h"


#define DBG_TAG "radio_hal"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/******************************
**Name:  vRadioGpioInit
**Func:  Radio SPI-4 and GPIO config
**Input: None
*Output: None
********************************/
void vRadioGpioInit(void)
{
	vSpiMasterInit();			//init spi-4 gpio
}


/******************************
**Name: bRadioReadReg
**Func: radio read one register
**Input: None
*Output: read out data
********************************/
unsigned char bRadioReadReg(unsigned char addr)
{
	return(bSpiReadByte(addr));
}

/******************************
**Name: bRadioWriteReg
**Func: radio write one register
**Input: None
*Output: old data
********************************/
unsigned char bRadioWriteReg(unsigned char addr, unsigned char reg_dat)
{
	return(bSpiWriteByte(addr, reg_dat));
}

/******************************
**Name: bRadioSetReg
**Func: radio set some bits of register
**Input: None
*Output: old data
********************************/
unsigned char bRadioSetReg(unsigned char addr, unsigned char set_bits, unsigned char mask_bits)
{
	unsigned char tmp_dat;

	tmp_dat  = bSpiReadByte(addr);
	tmp_dat &= (~mask_bits);
	tmp_dat |= (set_bits & mask_bits);
	return(bSpiWriteByte(addr, tmp_dat));
}

/******************************
**Name: vRadioLoadRegs
**Func: radio read some registers
**Input: None
*Output: None
********************************/
void vRadioLoadRegs(unsigned char sta_adr, unsigned char *ptr_buf, unsigned char length)
{
	unsigned char i;
	for(i=0; i<length; i++)
		ptr_buf[i] = bSpiReadByte(sta_adr++);
}

/******************************
**Name: vRadioStoreRegs
**Func: radio write some registers
**Input: None
*Output: None
********************************/
void vRadioStoreRegs(unsigned char sta_adr, unsigned char *ptr_buf, unsigned char length)
{
	unsigned char i;
	for(i=0; i<length; i++)
		bSpiWriteByte(sta_adr++, ptr_buf[i]);
}

/******************************
**Name: vRadioBurstReadRegs
**Func: radio read some registers, just for
**      Page0, start from 0x28, Packet Config Area & System Config Area, bank 5~15
**      Page1, start from 0x00
**Input: None
*Output: None
********************************/
void vRadioBurstReadRegs(unsigned char *ptr_buf, unsigned char length)
{
	vSpiBurstRead(CMT2310A_CRW_PORT, ptr_buf, length);
}

/******************************
**Name: vRadioBurstWriteRegs
**Func: radio write some registers, just for
**      Page0, start from 0x28, Packet Config Area & System Config Area, bank 5~15
**      Page1, start from 0x00
**Input: None
*Output: None
********************************/
void vRadioBurstWriteRegs(unsigned char *ptr_buf, unsigned char length)
{
	vSpiBurstWrite(CMT2310A_CRW_PORT, ptr_buf, length);
}

/******************************
**Name: vRadioReadFifo
**Func: radio read FIFO
**Input: None
*Output: None
********************************/
void vRadioReadFifo(unsigned char *ptr_fifo, unsigned char length)
{
	unsigned char tmp;
	tmp = bRadioReadReg(CMT2310A_CTL_REG_19);
	if(tmp&M_FIFO_MERGE_EN)								//when fifo merge
		{
		tmp &= (~M_FIFO_TX_RX_SEL);
		bRadioWriteReg(CMT2310A_CTL_REG_19, tmp);
		}
	vSpiBurstRead(CMT2310A_FIFO_RW_PORT, ptr_fifo, length);
}


/******************************
**Name: vRadioWriteFifo
**Func: radio write FIFO
**Input: None
*Output: None
********************************/
void vRadioWriteFifo(unsigned char *ptr_fifo, unsigned char length)
{
	unsigned char tmp;
	tmp = bRadioReadReg(CMT2310A_CTL_REG_19);
	if(tmp&M_FIFO_MERGE_EN)								//when fifo merge
		{
			tmp &= (~M_FIFO_TX_RX_SEL);
			bRadioWriteReg(CMT2310A_CTL_REG_19, tmp);		//TX FIFO
		}
	vSpiBurstWrite(CMT2310A_FIFO_RW_PORT, ptr_fifo, length);
}

/******************************
**Name: vRadioReadTxFifo
**Func: radio read FIFO
**Input: None
*Output: None
********************************/
void vRadioReadTxFifo(unsigned char *ptr_fifo, unsigned char length)
{
	unsigned char tmp;

	tmp = bRadioReadReg(CMT2310A_CTL_REG_19);
	if(tmp&M_FIFO_MERGE_EN)								//when fifo merge
		tmp |= M_FIFO_TX_RX_SEL;
	tmp |= M_FIFO_TX_TEST_EN;
	bRadioWriteReg(CMT2310A_CTL_REG_19, tmp);

	vSpiBurstRead(CMT2310A_FIFO_RW_PORT, ptr_fifo, length);

	tmp &= (~M_FIFO_TX_TEST_EN);
	bRadioWriteReg(CMT2310A_CTL_REG_19, tmp);
}

//######################################################################
//                       GPIO
//######################################################################
/******************************
**Name:  vRadioSpiModeSel
**Func:  Radio SPI-4 or SPI-3 Select
**Input: FALSE: select SPI-4
*        TRUE: select SPI-3
*Output: None
**note:  should be point to page0
********************************/
void vRadioSpiModeSel(boolean_t spi_mod)
{
	if(spi_mod)
		bRadioSetReg(CMT2310A_CTL_REG_07, M_SPI_3W_EN, VAL_BIT3);
	else
		bRadioSetReg(CMT2310A_CTL_REG_07, 0, VAL_BIT3);
}

/******************************
**Name:  vRadioSetTxDin
**Func:  Radio tx_din enable or disable
**Input: cfg_din,  FALSE: disable
*        	       TRUE:  enable
*        pin_sel,  CMT2310A_TX_DIN_GPIO3: GPIO3 as Tx Din
* 				   CMT2310A_TX_DIN_GPIO4: GPIO4 as Tx Din
*                  CMT2310A_TX_DIN_nIRQ:  NIRQ as Tx Din
*Output: None
********************************/
void vRadioSetTxDin(boolean_t cfg_din, unsigned char pin_sel)
{
	bRadioSetReg(CMT2310A_CTL_REG_05, pin_sel, CMT2310A_TX_DIN_SEL);

	if(cfg_din)
		{
		bRadioSetReg(CMT2310A_CTL_REG_04, CMT2310A_TX_DIN_EN, CMT2310A_TX_DIN_EN);
		vRadioRegPageSel(1);
		bRadioSetReg(CMT2310A_TX_DR_REG_02, (0<<7), (1<<7));
		vRadioRegPageSel(0);
		}
	else
		{
		bRadioSetReg(CMT2310A_CTL_REG_04, 0, CMT2310A_TX_DIN_EN);
		vRadioRegPageSel(1);
		bRadioSetReg(CMT2310A_TX_DR_REG_02, (1<<7), (1<<7));
		vRadioRegPageSel(0);
		}
}

/******************************
**Name:  vRadioSetDclk
**Func:  Radio digital clkout enable or disable
**Input: FALSE: disable
*        TRUE:  enable
*Output: None
* note:  only active on GPIO4, and priority high over than other function on GPIO4
********************************/
void vRadioSetDigClkOut(boolean_t cfg_out)
{
	if(cfg_out)
		bRadioSetReg(CMT2310A_CTL_REG_06, CMT2310A_DIG_CLKOUT_EN, CMT2310A_DIG_CLKOUT_EN);
	else
		bRadioSetReg(CMT2310A_CTL_REG_06, 0, CMT2310A_DIG_CLKOUT_EN);
}

/******************************
**Name:  vRadioSetLfxoPad
**Func:  Radio enable or disable exteral 32768Hz xo connect
**       when enable this functiong, GPIO2 & GPIO3 as lfxo connect pin
**Input: FALSE: disable
*        TRUE:  enable
*Output: None
* note:  only active on GPIO2 & GPIO3, and priority high over than other function on GPIO2 & GPIO3
********************************/
void vRadioSetLfxoPad(boolean_t cfg_lfxo)
{
	if(cfg_lfxo)
		bRadioSetReg(CMT2310A_CTL_REG_07, CMT2310A_LFXO_PAD_EN, CMT2310A_LFXO_PAD_EN);
	else
		bRadioSetReg(CMT2310A_CTL_REG_07, 0, CMT2310A_LFXO_PAD_EN);
}

/******************************
**Name:  vRadioSetGpio0
**Func:  Radio config GPIO0
**Input: CMT2310A_GPIO0_DOUT
*        CMT2310A_GPIO0_INT1
* 		 CMT2310A_GPIO0_INT2
* 		 CMT2310A_GPIO0_DCLK
*		 CMT2310A_GPIO0_INT3
*Output: None
********************************/
void vRadioSetGpio0(unsigned char gpio0_sel)
{
	bRadioSetReg(CMT2310A_CTL_REG_04, gpio0_sel, CMT2310A_GPIO0_SEL);
}

/******************************
**Name:  vRadioSetGpio1
**Func:  Radio config GPIO1
**Input: CMT2310A_GPIO1_DCLK
*        CMT2310A_GPIO1_INT1
* 		 CMT2310A_GPIO1_INT2
* 		 CMT2310A_GPIO1_DOUT
*Output: None
********************************/
void vRadioSetGpio1(unsigned char gpio1_sel)
{
	bRadioSetReg(CMT2310A_CTL_REG_04, gpio1_sel, CMT2310A_GPIO1_SEL);
}

/******************************
**Name:  vRadioSetGpio2
**Func:  Radio config GPIO2
**Input: CMT2310A_GPIO2_INT1
*        CMT2310A_GPIO2_INT2
* 		 CMT2310A_GPIO2_DCLK
* 		 CMT2310A_GPIO2_DOUT
*		 CMT2310A_GPIO2_INT3
*Output: None
********************************/
void vRadioSetGpio2(unsigned char gpio2_sel)
{
	bRadioSetReg(CMT2310A_CTL_REG_05, gpio2_sel, CMT2310A_GPIO2_SEL);
}

/******************************
**Name:  vRadioSetGpio3
**Func:  Radio config GPIO3
**Input: CMT2310A_GPIO3_INT2
*        CMT2310A_GPIO3_INT1
* 		 CMT2310A_GPIO3_DCLK
* 		 CMT2310A_GPIO3_DOUT
* 		 CMT2310A_GPIO3_DIN
*Output: None
********************************/
void vRadioSetGpio3(unsigned char gpio3_sel)
{
	bRadioSetReg(CMT2310A_CTL_REG_05, gpio3_sel, CMT2310A_GPIO3_SEL);
}

/******************************
**Name:  vRadioSetGpio4
**Func:  Radio config GPIO4
**Input: CMT2310A_GPIO4_DOUT
*        CMT2310A_GPIO4_INT1
* 		 CMT2310A_GPIO4_INT2
* 		 CMT2310A_GPIO4_DCLK
* 		 CMT2310A_GPIO4_DIN
*Output: None
********************************/
void vRadioSetGpio4(unsigned char gpio4_sel)
{
	bRadioSetReg(CMT2310A_CTL_REG_06, gpio4_sel, CMT2310A_GPIO4_SEL);
}

/******************************
**Name:  vRadioSetGpio5
**Func:  Radio config GPIO5
**Input: CMT2310A_GPIO5_nRST
*        CMT2310A_GPIO5_INT1
* 		 CMT2310A_GPIO5_INT2
* 		 CMT2310A_GPIO5_DOUT
* 		 CMT2310A_GPIO5_DCLK
*Output: None
********************************/
void vRadioSetGpio5(unsigned char gpio5_sel)
{
	bRadioSetReg(CMT2310A_CTL_REG_06, gpio5_sel, CMT2310A_GPIO5_SEL);
}

/******************************
**Name:  vRadioSetNirq
**Func:  Radio config NIRQ pin
**Input: CMT2310A_nIRQ_INT1
*        CMT2310A_nIRQ_INT2
* 		 CMT2310A_nIRQ_DCLK
* 		 CMT2310A_nIRQ_DOUT
* 		 CMT2310A_nIRQ_DIN
*Output: None
********************************/
void vRadioSetNirq(unsigned char nirq_sel)
{
	bRadioSetReg(CMT2310A_CTL_REG_07, nirq_sel, CMT2310A_nIRQ_SEL);
}

/******************************
**Name:  vRadioTcxoDrvSel
**Func:  Radio set TCXO drive power, nIRQ pin
**Input:  0=strengh
		  3=week
*Output: None
********************************/
void vRadioTcxoDrvSel(unsigned char drv_sel)
{
	vRadioRegPageSel(1);
	switch(drv_sel&0x03)
		{
		case 0:  bRadioSetReg(CMT2310A_CMT_REG_05, (0<<6), (3<<6)); break;
		case 1:  bRadioSetReg(CMT2310A_CMT_REG_05, (1<<6), (3<<6)); break;
		case 2:  bRadioSetReg(CMT2310A_CMT_REG_05, (2<<6), (3<<6)); break;
		default: bRadioSetReg(CMT2310A_CMT_REG_05, (3<<6), (3<<6)); break;
		}
	vRadioRegPageSel(0);
}


//######################################################################
//                      Auxrl
//######################################################################

/******************************
**Name:  vRadioRegPageSel
**Func:  Radio register page select(page0, page1, page2)
**Input:   1: select page 1
*          0: select page 0
*          2: select page 2
*Output: None
********************************/
void vRadioRegPageSel(unsigned char page_sel)
{
	page_sel &= 0x03;
	switch(page_sel)
		{
		case 2:  bRadioSetReg(CMT2310A_PAGE_CTL_REG, CMT2310A_PAGE_2, CMT2310A_PAGE_SEL_MASK); break;
		case 1:  bRadioSetReg(CMT2310A_PAGE_CTL_REG, CMT2310A_PAGE_1, CMT2310A_PAGE_SEL_MASK); break;
		case 0:
		default: bRadioSetReg(CMT2310A_PAGE_CTL_REG, CMT2310A_PAGE_0, CMT2310A_PAGE_SEL_MASK); break;
		}
}

/******************************
**Name:  vRadioPowerUpBoot
**Func:  Radio power up boot start
**Input: None
*Output: None
********************************/
void vRadioPowerUpBoot(void)
{
	bSpiWriteByte(CMT2310A_CTL_REG_00,  CMT2310A_REBOOT);
}

/******************************
**Name:  vRadioSoftReset
**Func:  Radio soft reset
**Input: None
*Output: None
********************************/
void vRadioSoftReset(void)
{
	unsigned char i;
	bRadioWriteReg(CMT2310A_SOFT_RST, 0xFF);
	for(i=0; i<20; i++)
	    rt_thread_mdelay(2);
}

/******************************
**Name:  vRadioSetPaOutputMode
**Func:  Radio config PA output mode
**Input: cfg_en
*               FALSE: single ended PA output mode
*               TRUE:  differential PA output mode
*Output: None
********************************/
void vRadioSetPaOutputMode(boolean_t cfg_en)
{
	if(cfg_en)
		bRadioSetReg(CMT2310A_CTL_REG_22, CMT2310A_PA_DIFF_SEL, CMT2310A_PA_DIFF_SEL);
	else
		bRadioSetReg(CMT2310A_CTL_REG_22, 0, CMT2310A_PA_DIFF_SEL);
}

/******************************
**Name:  vRadioSetTxDataInverse
**Func:  Radio config Tx Data inverse
**Input: cfg_en
*               FALSE: normal
*               TRUE:  inverse
*Output: None
********************************/
void vRadioSetTxDataInverse(boolean_t cfg_en)
{
	if(cfg_en)
		bRadioSetReg(CMT2310A_CTL_REG_22, CMT2310A_TX_DATA_INV, CMT2310A_TX_DATA_INV);
	else
		bRadioSetReg(CMT2310A_CTL_REG_22, 0, CMT2310A_TX_DATA_INV);
}

/******************************
**Name:  vRadioSetAntSwitch
**Func:  Radio config GPIO0 & GPIO1 as antenna switch control
**Input: cfg_en,     FALSE: disable antenna switch function
*                    TRUE:  enable antenna switch function
*
*                                      GPIO0     GPIO1
*        cfg_polar,  FALSE: RX_STATE     1         0
*                           TX_STATE     0         1
*
*                    TRUE:  RX_STATE     0         1
*                           TX_STATE     1         0
*Output: None
* note:  priority high over than other function on GPIO0 & GPIO1
********************************/
void vRadioSetAntSwitch(boolean_t cfg_en, boolean_t cfg_polar)
{
	unsigned char cfg_tmp = 0;

	if(cfg_en)
		cfg_tmp |= CMT2310A_TRX_SWT_EN;
	if(cfg_polar)
		cfg_tmp |= CMT2310A_TRX_SWT_INV;

	bRadioSetReg(CMT2310A_CTL_REG_22, cfg_en, (CMT2310A_TRX_SWT_EN|CMT2310A_TRX_SWT_INV));
}

/******************************
**Name:  vRadioDcdcCfg
**Func:  Radio DC-DC config
**Input: cfg_en
*               FALSE: normal
*               TRUE:  inverse ȡ��
*Output: None
********************************/
void vRadioDcdcCfg(boolean_t on_off)
{
	vRadioRegPageSel(1);
	if(on_off)							//Buck_sel = 1;
		bRadioSetReg(CMT2310A_CMT_REG_01, 0x10, 0x10);
	else
		bRadioSetReg(CMT2310A_CMT_REG_01, 0x00, 0x10);
	vRadioRegPageSel(0);
}

/******************************
**Name:  vRadioCapLoad
**Func:  Radio Set Cap load value
**Input: cap value, range 0-31
*Output: None
********************************/
void vRadioCapLoad(unsigned char cap_value)
{
	cap_value &= 0x1F;
 	vRadioRegPageSel(1);
	bRadioSetReg(CMT2310A_CMT_REG_06, cap_value, 0x1F);
	vRadioRegPageSel(0);
}

/******************************
**Name:  vRadioLfoscCfg
**Func:  Radio Set LFOSC
**Input: TRUE:  enable
**       FALSE: disable
*Output: None
********************************/
void vRadioLfoscCfg(boolean_t on_off)
{
 	vRadioRegPageSel(1);
	if(on_off)
		bRadioSetReg(CMT2310A_CMT_REG_13, 0x38, 0x38);
	else
		bRadioSetReg(CMT2310A_CMT_REG_13, 0x00, 0x38);
	vRadioRegPageSel(0);
}
