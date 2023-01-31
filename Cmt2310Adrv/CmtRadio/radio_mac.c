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

#define DBG_TAG "radio_mac"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


extern struct ax5043 rf_433;

/******************************
**Name:  bRadioGetCurrentChannl
**Func:  Radio get current active channl number
**Input: None
*Output: channl number
********************************/
unsigned char bRadioGetCurrentChannl(void)
{
	return(bRadioReadReg(CMT2310A_FREQ_CHANL_ACT_REG));
}

/******************************
**Name:  vRadioSetTxSeqNumber
**Func:  Radio set transmit sequence number
**Input: transmit init sequence number
*Output: None
********************************/
void vRadioSetTxSeqNumberInitValue(FRAME_CFG *frm_cfg)
{
	bRadioWriteReg(CMT2310A_SEQNUM_TX_IN_L_REG, (unsigned char)((*frm_cfg).SEQNUM_TX_IN));
	bRadioWriteReg(CMT2310A_SEQNUM_TX_IN_H_REG, (unsigned char)(((*frm_cfg).SEQNUM_TX_IN)>>8));	
}

/******************************
**Name:  wRadioGetTxSeqNumber
**Func:  Radio get current transmit sequence number
**Input: None
*Output: current transmit sequence number
********************************/
unsigned short wRadioGetTxSeqNumberCurrent(FRAME_CFG *frm_cfg)
{
	unsigned short seq_num = 0;
	
	seq_num   = bRadioReadReg(CMT2310A_SEQNUM_TX_OUT_H_REG);
	seq_num <<= 8;
	seq_num  |= bRadioReadReg(CMT2310A_SEQNUM_TX_OUT_L_REG);
	(*frm_cfg).SEQNUM_TX_CURRENT_OUT = seq_num;
	return((*frm_cfg).SEQNUM_TX_CURRENT_OUT = seq_num);
}


/******************************
**Name:  vRadioSetTxFCS2
**Func:  Radio set transmit packet FCS2 value
**Input: transmit FCS2
*Output: None
********************************/
void vRadioSetTxFCS2(FRAME_CFG *frm_cfg)
{
	bRadioWriteReg(CMT2310A_FCS2_TX_IN_REG, (*frm_cfg).FCS2_TX_IN);
}

/******************************
**Name:  vRadioGetRxFCS2
**Func:  Radio get receive packet FCS2 value
**Input: None
*Output: receive FCS2
********************************/
unsigned char bRadioGetRxFCS2(FRAME_CFG *frm_cfg)
{
	(*frm_cfg).FCS2_RX_OUT = bRadioReadReg(CMT2310A_FCS2_RX_OUT_REG);
	return((*frm_cfg).FCS2_RX_OUT);
}

/******************************
**Name:  vRadioSetPayloadLength
**Func:  Radio config payload length
**Input: length
*Output: None
********************************/
void vRadioSetPayloadLength(FRAME_CFG *frm_cfg)
{
	unsigned short len;
	
	if((*frm_cfg).PAYLOAD_LENGTH!=0)
		{
		len = (*frm_cfg).PAYLOAD_LENGTH - 1;
			bRadioWriteReg(CMT2310A_PAYLOAD_LENGTH_L_REG, (byte)len);
			bRadioWriteReg(CMT2310A_PAYLOAD_LENGTH_H_REG, (byte)(len>>8));
		}
}

/******************************
**Name:  wRadioGetPayloadLength
**Func:  Radio get payload length
**Input: None
*Output: payload length
********************************/
unsigned short vRadioGetPayloadLength(FRAME_CFG *frm_cfg)
{
	unsigned short length = 0;
	
	length   = bRadioReadReg(CMT2310A_PAYLOAD_LENGTH_H_REG);
	length <<= 8;
	length  |= bRadioReadReg(CMT2310A_PAYLOAD_LENGTH_L_REG);
	(*frm_cfg).PAYLOAD_LENGTH = length+1;
	return(length);
}

//######################################################################
//                       Packet config 
//######################################################################
/******************************
**Name:  vRadioCfgPreamble
**Func:  Radio config preamble 
**Input: preamble struct
*Output: None
********************************/
void vRadioCfgPreamble(PREAMBLE_CFG *prm_ptr)
{
	unsigned char cfg_tmp;
	
	cfg_tmp = bRadioReadReg(CMT2310A_CTL_REG_40);
	if((*prm_ptr).PREAM_LENG_UNIT==0)
		cfg_tmp &= (~CMT2310A_PREAM_LENG_UNIIT);
	else
		cfg_tmp |= CMT2310A_PREAM_LENG_UNIIT;
	cfg_tmp &= (~CMT2310A_RX_PREAM_SIZE_MASK);		
	cfg_tmp |= ((((*prm_ptr).RX_PREAM_SIZE)<<3)&CMT2310A_RX_PREAM_SIZE_MASK);
	bRadioWriteReg(CMT2310A_CTL_REG_40, cfg_tmp);
	bRadioWriteReg(CMT2310A_CTL_REG_41, (unsigned char)((*prm_ptr).TX_PREAM_SIZE));
	bRadioWriteReg(CMT2310A_CTL_REG_42, (unsigned char)((*prm_ptr).TX_PREAM_SIZE>>8));
	bRadioWriteReg(CMT2310A_CTL_REG_43, (*prm_ptr).PREAM_VALUE);
}

/******************************
**Name:  vRadioCfgSyncWord
**Func:  Radio config sync word 
**Input: sync word struct
*Output: None
********************************/
void vRadioCfgSyncWord(SYNC_CFG *sync_ptr)
{
	unsigned char i;
	unsigned char adr;
	
	bRadioWriteReg(CMT2310A_CTL_REG_44, (*sync_ptr).SYN_CFG_u.SYNC_CFG_REG);
	for(i=0, adr=CMT2310A_CTL_REG_52; i<8; i++, adr--)
		{
		bRadioWriteReg(adr, (*sync_ptr).SYNC_VALUE[i]);
		bRadioWriteReg((adr+8), (*sync_ptr).SYNC_FEC_VALUE[i]);
		}
	
	if((*sync_ptr).SYNC_VALUE_SEL==0)
		bRadioSetReg(CMT2310A_CTL_REG_64, 0, CMT2310A_SYNC_VALUE_SEL);
	else
		bRadioSetReg(CMT2310A_CTL_REG_64, CMT2310A_SYNC_VALUE_SEL, CMT2310A_SYNC_VALUE_SEL);	
}

/******************************
**Name:  vRadioCfgNodeAddr
**Func:  Radio config node address
**Input: node address struct
*Output: None
********************************/
void vRadioCfgNodeAddr(ADDR_CFG *node_addr_ptr)
{
	unsigned char cfg_tmp;
	
	cfg_tmp  = bRadioReadReg(CMT2310A_CTL_REG_64);
	cfg_tmp &= 0x80;
	cfg_tmp |= ((*node_addr_ptr).ADDR_CFG_u.ADDR_CFG_REG&0x7F);
	bRadioWriteReg(CMT2310A_CTL_REG_64, cfg_tmp);
	
	bRadioWriteReg(CMT2310A_SRC_ADDR_L_REG, (*node_addr_ptr).SRC_ADDR[0]);
	bRadioWriteReg(CMT2310A_SRC_ADDR_H_REG, (*node_addr_ptr).SRC_ADDR[1]);
	
	bRadioWriteReg(CMT2310A_DEST_ADDR_L_REG, (*node_addr_ptr).DEST_ADDR[0]);
	bRadioWriteReg(CMT2310A_DEST_ADDR_H_REG, (*node_addr_ptr).DEST_ADDR[1]);	
	
	bRadioWriteReg(CMT2310A_SRC_BITMASK_L_REG, (*node_addr_ptr).SRC_BITMASK[0]);
	bRadioWriteReg(CMT2310A_SRC_BITMASK_H_REG, (*node_addr_ptr).SRC_BITMASK[1]);

	bRadioWriteReg(CMT2310A_DEST_BITMASK_L_REG, (*node_addr_ptr).DEST_BITMASK[0]);
	bRadioWriteReg(CMT2310A_DEST_BITMASK_H_REG, (*node_addr_ptr).DEST_BITMASK[1]);
}

/******************************
**Name:  vRadioCfgCrc
**Func:  Radio config crc
**Input: crc struct
*Output: None
********************************/
void vRadioCfgCrc(CRC_CFG *crc_ptr)
{
	unsigned char i, adr;

	bRadioWriteReg(CMT2310A_CTL_REG_73, (unsigned char)((*crc_ptr).CRC_CFG_u.CRC_CFG_REG));

	for(i=0, adr=CMT2310A_CTL_REG_74; i<4; i++, adr++)
		{
		bRadioWriteReg(adr, (*crc_ptr).CRC_SEED_u.u8_SEED[i]);
		bRadioWriteReg((adr+4), (*crc_ptr).CRC_POLY_u.u8_POLY[i]);
		}

	if((*crc_ptr).CRC_CFG_u._BITS.CRC_REFOUT)
		bRadioSetReg(CMT2310A_CTL_REG_82, CMT2310A_CRC_REFOUT, CMT2310A_CRC_REFOUT);
	else
		bRadioSetReg(CMT2310A_CTL_REG_82, 0, CMT2310A_CRC_REFOUT);

	if((*crc_ptr).CRC_CFG_u._BITS.CRCERR_CLR_FIFO_EN)
		bRadioSetReg(CMT2310A_CTL_REG_84, CMT2310A_CRCERR_CLR_FIFO_EN, CMT2310A_CRCERR_CLR_FIFO_EN);
	else
		bRadioSetReg(CMT2310A_CTL_REG_84, 0, CMT2310A_CRCERR_CLR_FIFO_EN);
}

/******************************
**Name:  vRadioCfgCodeFormat
**Func:  Radio config code format
**Input: code format struct
*Output: None
********************************/
void vRadioCfgCodeFormat(CODING_FORMAT_CFG *code_format_ptr)
{
	unsigned char cfg_tmp;
	
	cfg_tmp  = bRadioReadReg(CMT2310A_CTL_REG_82);
	cfg_tmp &= 0x80;
	cfg_tmp |= ((*code_format_ptr).CODING_FORMAT_CFG_u.CODING_CFG_REG&0x3F);
	if((*code_format_ptr).WHITEN_SEED&0x0100)
		cfg_tmp |= CMT2310A_WHITEN_SEED_B8;
	bRadioWriteReg(CMT2310A_CTL_REG_82, cfg_tmp);
	
	bRadioWriteReg(CMT2310A_CTL_REG_83, (unsigned char)((*code_format_ptr).WHITEN_SEED));

	cfg_tmp  = ((*code_format_ptr).CODING_FORMAT_CFG_u.CODING_CFG_REG>>8);
	cfg_tmp &= (~CMT2310A_FEC_PAD_CODE_H_MASK);
	cfg_tmp |= ((unsigned char)(((*code_format_ptr).FEC_PAD_CODE)>>6)&CMT2310A_FEC_PAD_CODE_H_MASK);
	bRadioWriteReg(CMT2310A_CTL_REG_93, cfg_tmp);
	
	bRadioWriteReg(CMT2310A_CTL_REG_94, (unsigned char)((*code_format_ptr).FEC_PAD_CODE));
}

/******************************
**Name:  vRadioCfgFrameFormat
**Func:  Radio config frame format
**Input: frame format struct
*Output: None
********************************/
void vRadioCfgFrameFormat(FRAME_CFG *frame_format_ptr)
{
	unsigned char cfg_tmp;

	bRadioSetReg(CMT2310A_CTL_REG_40, (*frame_format_ptr).DATA_MODE, CMT2310A_DATA_MODE_MASK);

	bRadioWriteReg(CMT2310A_CTL_REG_63, (*frame_format_ptr).FRAME_CFG1_u.FRAME_CFG1_REG);	
	
	cfg_tmp  = bRadioReadReg(CMT2310A_CTL_REG_84);	
	cfg_tmp &= 0x80;	
	cfg_tmp |= ((*frame_format_ptr).FRAME_CFG2_u.FRAME_CFG2_REG&0x7F);
	bRadioWriteReg(CMT2310A_CTL_REG_84, cfg_tmp);		
	
	bRadioWriteReg(CMT2310A_TX_PKT_NUM_L_REG,  (unsigned char)(*frame_format_ptr).TX_PKT_NUM);	
	bRadioWriteReg(CMT2310A_TX_PKT_NUM_H_REG,  (unsigned char)((*frame_format_ptr).TX_PKT_NUM>>8));	

	bRadioWriteReg(CMT2310A_SEQNUM_TX_IN_L_REG,  (unsigned char)(*frame_format_ptr).SEQNUM_TX_IN);	
	bRadioWriteReg(CMT2310A_SEQNUM_TX_IN_H_REG,  (unsigned char)((*frame_format_ptr).SEQNUM_TX_IN>>8));	
		
	bRadioWriteReg(CMT2310A_TX_PKT_GAP_REG,  (*frame_format_ptr).TX_PKT_GAP);		
	
	bRadioWriteReg(CMT2310A_FCS2_TX_IN_REG,  (*frame_format_ptr).FCS2_TX_IN);		
	bRadioWriteReg(CMT2310A_FCS2_RX_OUT_REG, (*frame_format_ptr).FCS2_RX_OUT);
}

/******************************
**Name:  vRadioCfgFrameFormat
**Func:  Radio config DATA_MODE
**Input: frame format struct
*Output: None
********************************/
void vRadioCfgDatamodeFormat(FRAME_CFG *frame_format_ptr)
{
	unsigned char cfg_tmp;
	bRadioSetReg(CMT2310A_CTL_REG_40, (*frame_format_ptr).DATA_MODE, CMT2310A_DATA_MODE_MASK);
}


/******************************
**Name:  vRadioCdrTracingModeCfg
**Func:  Radio config cdr tracing config
**Input: cdr tracing 
*Output: None
********************************/
void vRadioCdrTracingModeCfg(CDR_TRACING_CFG *cdr_ptr)
{
	vRadioRegPageSel(1);	
	
	bRadioWriteReg(CMT2310A_RX_CDR_REG_00, (*cdr_ptr).CDR_CFG0_u.CDR_CFG0_REG);	
	
    (*cdr_ptr).CDR_CFG1_u.CDR_CFG1_REG &= 0xF8;
	(*cdr_ptr).CDR_CFG1_u.CDR_CFG1_REG |= ((unsigned char)((*cdr_ptr).CDR_BR_TH>>16)&0x07);
	bRadioSetReg(CMT2310A_RX_CDR_REG_03, (*cdr_ptr).CDR_CFG1_u.CDR_CFG1_REG, 0x1F);
	
	bRadioWriteReg(CMT2310A_RX_CDR_REG_01, (unsigned char)(*cdr_ptr).CDR_BR_TH);	
	bRadioWriteReg(CMT2310A_RX_CDR_REG_02, (unsigned char)((*cdr_ptr).CDR_BR_TH>>8));	

	vRadioRegPageSel(0);	
}


//######################################################################
//                       System control
//######################################################################
void vRadioCfgWorkMode(WORK_MODE_CFG *run_mode_ptr)
{
	bRadioWriteReg(CMT2310A_CTL_REG_03, (*run_mode_ptr).FREQ_CHANL_NANU);
	
	bRadioWriteReg(CMT2310A_CTL_REG_11, (*run_mode_ptr).FREQ_DONE_TIMES);
	bRadioWriteReg(CMT2310A_CTL_REG_12, (*run_mode_ptr).FREQ_SPACE);
	bRadioWriteReg(CMT2310A_CTL_REG_13, (*run_mode_ptr).FREQ_TIMES);
	
	bRadioWriteReg(CMT2310A_CTL_REG_96,  (*run_mode_ptr).WORK_MODE_CFG1_u.WORK_MODE_CFG1_REG);
	bRadioWriteReg(CMT2310A_CTL_REG_97,  (*run_mode_ptr).WORK_MODE_CFG2_u.WORK_MODE_CFG2_REG);
	bRadioWriteReg(CMT2310A_CTL_REG_98,  (*run_mode_ptr).WORK_MODE_CFG3_u.WORK_MODE_CFG3_REG);
	bRadioWriteReg(CMT2310A_CTL_REG_105, (*run_mode_ptr).WORK_MODE_CFG4_u.WORK_MODE_CFG4_REG);
	bRadioWriteReg(CMT2310A_CTL_REG_106, (*run_mode_ptr).WORK_MODE_CFG5_u.WORK_MODE_CFG5_REG);	
	bRadioSetReg(CMT2310A_CTL_REG_22, (*run_mode_ptr).WORK_MODE_CFG6_u.WORK_MODE_CFG6_REG, 0xC0);		//����Ϊ�ֶ���Ƶ����һֱ��

	bRadioWriteReg(CMT2310A_CTL_REG_99, (unsigned char)((*run_mode_ptr).SLEEP_TIMER_M));
	bRadioWriteReg(CMT2310A_CTL_REG_100, (((unsigned char)((*run_mode_ptr).SLEEP_TIMER_M>>3))&0xE0)|((*run_mode_ptr).SLEEP_TIMER_R&0x1F));

	bRadioWriteReg(CMT2310A_CTL_REG_101, (unsigned char)((*run_mode_ptr).RX_TIMER_T1_M));
	bRadioWriteReg(CMT2310A_CTL_REG_102, (((unsigned char)((*run_mode_ptr).RX_TIMER_T1_M>>3))&0xE0)|((*run_mode_ptr).RX_TIMER_T1_R&0x1F));
	
	bRadioWriteReg(CMT2310A_CTL_REG_103, (unsigned char)((*run_mode_ptr).RX_TIMER_T2_M));
	bRadioWriteReg(CMT2310A_CTL_REG_104, (((unsigned char)((*run_mode_ptr).RX_TIMER_T2_M>>3))&0xE0)|((*run_mode_ptr).RX_TIMER_T2_R&0x1F));
	
	bRadioWriteReg(CMT2310A_CTL_REG_107, (unsigned char)((*run_mode_ptr).RX_TIMER_CSMA_M));
	bRadioWriteReg(CMT2310A_CTL_REG_108, (((unsigned char)((*run_mode_ptr).RX_TIMER_CSMA_M>>3))&0xE0)|((*run_mode_ptr).RX_TIMER_CSMA_R&0x1F));
	bRadioWriteReg(CMT2310A_CTL_REG_110, (*run_mode_ptr).TX_DC_TIMES);
	bRadioWriteReg(CMT2310A_CTL_REG_113, (*run_mode_ptr).TX_RS_TIMES);
	bRadioWriteReg(CMT2310A_CTL_REG_115, (*run_mode_ptr).CSMA_TIMES);
	bRadioWriteReg(CMT2310A_CTL_REG_118, (unsigned char)((*run_mode_ptr).SLEEP_TIMER_CSMA_M));
	bRadioWriteReg(CMT2310A_CTL_REG_119, (((unsigned char)((*run_mode_ptr).SLEEP_TIMER_CSMA_M>>3))&0xE0)|((*run_mode_ptr).SLEEP_TIMER_CSMA_R&0x1F));
}

void vRadioReadRunModeCfg(void)
{
	bRadioReadReg(CMT2310A_CTL_REG_11);
	bRadioReadReg(CMT2310A_CTL_REG_12);
	bRadioReadReg(CMT2310A_CTL_REG_13);

	bRadioReadReg(CMT2310A_CTL_REG_96);
	bRadioReadReg(CMT2310A_CTL_REG_97);	
	bRadioReadReg(CMT2310A_CTL_REG_98);	
	bRadioReadReg(CMT2310A_CTL_REG_105);
	bRadioReadReg(CMT2310A_CTL_REG_106);
	
	bRadioReadReg(CMT2310A_CTL_REG_99);
	bRadioReadReg(CMT2310A_CTL_REG_100);
	
	bRadioReadReg(CMT2310A_CTL_REG_101);
	bRadioReadReg(CMT2310A_CTL_REG_102);

	bRadioReadReg(CMT2310A_CTL_REG_103);
	bRadioReadReg(CMT2310A_CTL_REG_104);
	
	bRadioReadReg(CMT2310A_CTL_REG_107);
	bRadioReadReg(CMT2310A_CTL_REG_108);
	bRadioReadReg(CMT2310A_CTL_REG_110);
	bRadioReadReg(CMT2310A_CTL_REG_113);
	bRadioReadReg(CMT2310A_CTL_REG_115);
	bRadioReadReg(CMT2310A_CTL_REG_118);
	bRadioReadReg(CMT2310A_CTL_REG_119);
}	
	
unsigned char bRadioGetTxDutyCycleDoneTimes(WORK_MODE_CFG *run_mode_ptr)
{
	(*run_mode_ptr).TX_DC_DONE_TIMES = bRadioReadReg(CMT2310A_CTL_REG_112);
	return((*run_mode_ptr).TX_DC_DONE_TIMES);
}	

unsigned char bRadioGetTxResendDoneTimes(WORK_MODE_CFG *run_mode_ptr)
{
	(*run_mode_ptr).TX_RS_DONE_TIMES = bRadioReadReg(CMT2310A_CTL_REG_114);
	return((*run_mode_ptr).TX_RS_DONE_TIMES);
}

unsigned char bRadioGetCMSADoneTimes(WORK_MODE_CFG *run_mode_ptr)
{
	(*run_mode_ptr).CSMA_DONE_TIMES = bRadioReadReg(CMT2310A_CTL_REG_116);
	return((*run_mode_ptr).CSMA_DONE_TIMES);
}

void vRadioSendWithAck(boolean_t w_ack, FRAME_CFG *frame_format_ptr)
{
	unsigned char tmp;
	
	tmp = bRadioReadReg(CMT2310A_FCS2_TX_IN_REG);
	
	if(w_ack)
		(*frame_format_ptr).FCS2_TX_IN = tmp|0x80;
	else
		((*frame_format_ptr)).FCS2_TX_IN = tmp&0x7F;
	
	bRadioWriteReg(CMT2310A_FCS2_TX_IN_REG, ((*frame_format_ptr)).FCS2_TX_IN);
}



void vRadioEnableTxAck(boolean_t en_flg, WORK_MODE_CFG *run_mode_ptr)
{
	if(en_flg)
		{
		(*run_mode_ptr).WORK_MODE_CFG1_u._BITS.TX_ACK_EN   = 1;			//enable TX_ACK		
		(*run_mode_ptr).WORK_MODE_CFG2_u._BITS.RX_TIMER_EN = 1;	
		}
	else
		{
		(*run_mode_ptr).WORK_MODE_CFG1_u._BITS.TX_ACK_EN   = 0;			//disable TX_ACK
		(*run_mode_ptr).WORK_MODE_CFG2_u._BITS.RX_TIMER_EN = 0;	
		}
	
	vRadioCfgWorkMode(run_mode_ptr);
}


void vRadioEnableRxAck(boolean_t en_flg, WORK_MODE_CFG *run_mode_ptr)
{
	if(en_flg)
		(*run_mode_ptr).WORK_MODE_CFG2_u._BITS.RX_ACK_EN = 1;			//enable RX_ACK		
	else
		(*run_mode_ptr).WORK_MODE_CFG2_u._BITS.RX_ACK_EN = 0;			//disable RX_ACK

	vRadioCfgWorkMode(run_mode_ptr);
}


unsigned char bRadioGetFreqChanl(void)
{
	return(bRadioReadReg(CMT2310A_FREQ_CHANL_ACT_REG));
}

void vRadioCsmaEnable(boolean_t on_off, WORK_MODE_CFG *run_mode_ptr)
{
	if(on_off)
		{
		(*run_mode_ptr).WORK_MODE_CFG2_u._BITS.RX_TIMER_EN = 1;		
		(*run_mode_ptr).WORK_MODE_CFG2_u._BITS.CSMA_EN = 1;	
		vRadioCfgWorkMode(run_mode_ptr);
		}
	else
		{
		(*run_mode_ptr).WORK_MODE_CFG2_u._BITS.CSMA_EN = 0;
		bRadioSetReg(CMT2310A_CTL_REG_97, 0x00, 0x80);
		}	
}

void vRadioSetRssiAbsThValue(signed char rssi)
{
	vRadioRegPageSel(1);	
	bRadioWriteReg(CMT2310A_RSSI_ABS_TH_REG, (unsigned char)rssi);		
	vRadioRegPageSel(0);
}

void vRadioSetPjdDetWin(unsigned char pjd_win)				//0:4-jump,  1:6-jump,   2:8-jump,   3:10-jump
{
	pjd_win  &= 0x03;
	pjd_win <<= 4;

	vRadioRegPageSel(1);									//CMT2310A_RX_2FSK_REG_01 is in bank1
	bRadioSetReg(CMT2310A_RX_2FSK_REG_01, pjd_win, 0x30);
	vRadioRegPageSel(0);
}
