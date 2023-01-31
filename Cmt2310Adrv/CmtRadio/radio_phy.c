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


#define DBG_TAG "radio_phy"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

extern struct ax5043 rf_433;

/******************************
 **Name:  vRadioSetInterruptSel
 **Func:  Radio config INT1 & INT2 pin seleciton
 **Input: int1_sel, int2_sel,
 **
 **      CMT2310A_INT_MIX				(0<<0)					// all interrupt mixed together
 **      CMT2310A_INT_ANT_LOCK			(1<<0)					// antenna locked
 **      CMT2310A_INT_RSSI_PJD_VALID		(2<<0)					// rssi and/or pjd valid
 **      CMT2310A_INT_PREAM_PASS			(3<<0)					// preamble detected
 **      CMT2310A_INT_SYNC_PASS			(4<<0)					// sync word detected
 **      CMT2310A_INT_ADDR_PASS			(5<<0)					// node address detected
 **      CMT2310A_INT_CRC_PASS			(6<<0)					// crc ok detected
 **      CMT2310A_INT_PKT_OK				(7<<0)					// packet received detected
 **      CMT2310A_INT_PKT_DONE			(8<<0)					// packet received detected, even wrong packet or collision
 **      CMT2310A_INT_SLEEP_TMO			(9<<0)					// sleep timer time-out
 **      CMT2310A_INT_RX_TMO				(10<<0)					// rx timer time-out
 **      CMT2310A_INT_RX_FIFO_NMTY		(11<<0)					// rx fifo non-empty
 **      CMT2310A_INT_RX_FIFO_TH			(12<<0)					// rx fifo threshold
 **      CMT2310A_INT_RX_FIFO_FULL		(13<<0)					// rx fifo full
 **      CMT2310A_INT_RX_FIFO_WBYTE		(14<<0)					// rx fifo write byte trigger
 **      CMT2310A_INT_RX_FIFO_OVF		(15<<0)					// rx fifo overflow
 **      CMT2310A_INT_TX_DONE			(16<<0)					// tx done
 **      CMT2310A_INT_TX_FIFO_NMTY		(17<<0)					// tx fifo non-empty
 **      CMT2310A_INT_TX_FIFO_TH			(18<<0)					// tx fifo threshold
 **      CMT2310A_INT_TX_FIFO_FULL		(19<<0)					// tx fifo full
 **      CMT2310A_INT_STATE_IS_READY		(20<<0)					// state is ready
 **      CMT2310A_INT_STATE_IS_FS		(21<<0)					// state is FS
 **      CMT2310A_INT_STATE_IS_RX		(22<<0)					// state is rx
 **      CMT2310A_INT_STATE_IS_TX		(23<<0)					// state is tx
 **      CMT2310A_INT_LBD_STATUS			(24<<0)					// LBD status
 **      CMT2310A_INT_API_CMD_FAILED		(25<<0)					// API Command failed
 **      CMT2310A_INT_API_DONE			(26<<0)					// API execute finish
 **      CMT2310A_INT_TX_DC_DONE			(27<<0)					// ??
 **      CMT2310A_INT_ACK_RECV_FAILED	(28<<0)					// ack recieve failed
 **      CMT2310A_INT_TX_RESEND_DONE		(29<<0)					// tx re-send done
 **      CMT2310A_INT_NACK_RECV			(30<<0)					// non-ack received
 **      CMT2310A_INT_SEQ_MATCH			(31<<0)					// sequence number match
 **      CMT2310A_INT_CSMA_DONE			(32<<0)					// CSMA done
 **      CMT2310A_INT_CCA_STATUS			(33<<0)					// CCA status match
 **
 *Output: None
 ********************************/
void vRadioSetInt1Sel(unsigned char int1_sel)
{
    bRadioSetReg(CMT2310A_INT1_SEL_REG, int1_sel, CMT2310A_INT1_SEL_MASK);
}

void vRadioSetInt2Sel(unsigned char int2_sel)
{
    bRadioSetReg(CMT2310A_INT2_SEL_REG, int2_sel, CMT2310A_INT2_SEL_MASK);
}

/******************************
 **Name:  vRadioSetInterruptPolar
 **Func:  Radio config INT pin out polar mode
 **Input: int1_polar, FALSE: ative-high, normal-low
 *                    TRUE:  ative-low,  normal-high
 *        int2_polar, FALSE: ative-high, normal-low
 *                    TRUE:  ative-low,  normal-high
 *        int3_polar, FALSE: ative-high, normal-low
 *                    TRUE:  ative-low,  normal-high
 *Output: None
 ********************************/
void vRadioSetInt1Polar(boolean_t int1_polar)
{
    if (int1_polar)
        bRadioSetReg(CMT2310A_CTL_REG_17, CMT2310A_INT1_POLAR, CMT2310A_INT1_POLAR);
    else
        bRadioSetReg(CMT2310A_CTL_REG_17, 0, CMT2310A_INT1_POLAR);
}

void vRadioSetInt2Polar(boolean_t int2_polar)
{
    if (int2_polar)
        bRadioSetReg(CMT2310A_CTL_REG_17, CMT2310A_INT2_POLAR, CMT2310A_INT2_POLAR);
    else
        bRadioSetReg(CMT2310A_CTL_REG_17, 0, CMT2310A_INT2_POLAR);
}

void vRadioSetInt3Polar(boolean_t int3_polar)
{
    if (int3_polar)
        bRadioSetReg(CMT2310A_CTL_REG_19, CMT2310A_INT3_POLAR, CMT2310A_INT3_POLAR);
    else
        bRadioSetReg(CMT2310A_CTL_REG_19, 0, CMT2310A_INT3_POLAR);
}

//######################################################################
//                       RSSI
//######################################################################

/******************************
 **Name:  vRadioRssiUpdateSel
 **Func:  Radio rssi update select
 **Input: CMT2310A_RSSI_UPDATE_ALWAYS   ---- always
 **       CMT2310A_RSSI_UPDATE_PREAM_OK ---- when preamble ok
 **       CMT2310A_RSSI_UPDATE_SYNC_OK  ---- when sync ok
 **       CMT2310A_RSSI_UPDATE_PKT_DONE ---- when pkt done
 *Output: None
 ********************************/
void vRadioRssiUpdateSel(unsigned char sel)
{
    vRadioRegPageSel(1);				//CMT2310A_RX_RSSI_REG_00 is in page1
    bRadioSetReg(CMT2310A_RX_RSSI_REG_00, sel, CMT2310A_RSSI_UPDATE_SEL_MASK);
    vRadioRegPageSel(0);
}

/******************************
 **Name:  bRadioGetRssi
 **Func:  Radio get rssi value
 **Input: None
 *Output: rssi with sign
 ********************************/
unsigned char bRadioGetRssi(void)
{
    return (bRadioReadReg(CMT2310A_CTL_REG_34));
}

/******************************
 **Name:  vRadioRssiConfig
 **Func:  Radio config Rssi
 **Input: None
 *Output: None
 ********************************/
void vRadioRssiConfig(RSSI_CFG rssi_cfg)
{
    vRadioRegPageSel(1);
    bRadioSetReg(CMT2310A_RX_RSSI_REG_00, rssi_cfg.FRAME_CFG1_u.RSSI_CFG_REG,
            (CMT2310A_COLL_STEP_SEL_MASK | CMT2310A_RSSI_UPDATE_SEL_MASK | CMT2310A_COLL_DET_EN));
    bRadioWriteReg(CMT2310A_RSSI_ABS_TH_REG, rssi_cfg.RSSI_ABS_TH);
    vRadioRegPageSel(0);
}

/******************************
 **Name:  vRadioRssiCalOffset
 **Func:  Radio config Rssi calibrate offset
 **Input: None
 *Output: None
 ********************************/
void vRadioRssiCalOffset(unsigned char cal_offset)
{
    vRadioRegPageSel(0);
    bRadioWriteReg(CMT2310A_RSSI_CAL_OFFSET_REG, cal_offset);
}

//######################################################################
//                       Misc
//######################################################################
/******************************
 **Name:  bRadioGetLbdValue
 **Func:  Radio get LBD value
 **Input: None
 *Output: LBD value
 ********************************/
unsigned char bRadioGetLbdValue(void)
{
    return (bRadioReadReg(CMT2310A_LBD_REG));
}

/******************************
 **Name:  vRadioSetLbdTH
 **Func:  Radio get LBD threshold value
 **Input: LBD threshold value
 *Output: None
 ********************************/
void vRadioSetLbdTH(unsigned char lbd_th)
{
    bRadioWriteReg(CMT2310A_LBD_TH_REG, lbd_th);
}

/******************************
 **Name:  bRadioGetTemperature
 **Func:  Radio get Temperature value
 **Input: None
 *Output:
 ********************************/
unsigned char bRadioGetTemperature(void)
{
    return (bRadioReadReg(CMT2310A_TEMP_REG));
}

/******************************
 **Name:  bRadioApiCommand
 **Input: api_cmd, API command code
 *Output:
 * note:  only support 0x01 command code
 ********************************/
boolean_t bRadioApiCommand(unsigned char api_cmd)
{
    unsigned short int i;

    bRadioWriteReg(CMT2310A_API_CMD_REG, api_cmd);

    api_cmd |= CMT2310A_API_CMD_FLAG;				//0000 0001|1000 0000 =1000 0001

    for (i = 0; i < 500; i++)
    {
        rt_thread_mdelay(2);
        if (bRadioReadReg(CMT2310A_CTL_REG_09) == api_cmd)
        {
//            LOG_D("bRadioReadReg = ture\r\n");
            return (TRUE);

        }

    }
//    LOG_D("bRadioReadReg = %d\r\n",bRadioReadReg(CMT2310A_CTL_REG_09));
    return (FALSE);
}

/******************************
 **Name:  vRadioCdrModeCfg
 **Func:  Radio set CDR mode
 **Input: enum CDR_MODE
 *Output:
 ********************************/
void vRadioCdrModeCfg(enum CDR_MODE cdr_mode)
{
    vRadioRegPageSel(1);
    bRadioSetReg(CMT2310A_RX_CDR_REG_00, cdr_mode, 0x03);
    vRadioRegPageSel(0);
}

/******************************
 **Name:  vRadioTxRampCfg
 **Func:  Radio set tx ramp
 **Input:
 **       tx_ramp_en: TRUE = enable; FALSE = disable
 **       tx_ramp_step: ramp value, about 1.9us/step
 *Output:
 ********************************/
void vRadioTxRampCfg(boolean_t tx_ramp_en, unsigned short tx_ramp_step)
{
    vRadioRegPageSel(1);
    if (tx_ramp_en)
    {
        bRadioSetReg(CMT2310A_TX_MODE_REG_00, (1 << 2), (1 << 2));
        bRadioWriteReg(CMT2310A_TX_PWR_REG_03, (unsigned char) tx_ramp_step);
        bRadioSetReg(CMT2310A_TX_PWR_REG_05, (unsigned char) ((tx_ramp_step >> 4) & 0xF0), 0xF0);
    }
    else
        bRadioSetReg(CMT2310A_TX_MODE_REG_00, (0 << 2), (1 << 2));
    vRadioRegPageSel(0);
}

/******************************
 **Name:  vRadioTxGaussianCfg
 **Func:  Radio set tx with Gaussian
 **Input:
 **       tx_gaus_en: TRUE = enable; FALSE = disable
 **       tx_gaus_bt: 0=0.3, 1=0.5, 2=0.8, 3=1.0
 *Output:
 ********************************/
void vRadioTxGaussianCfg(boolean_t tx_gaus_en, unsigned char tx_gaus_bt)
{
    unsigned char tmp = 0;
    tmp = (tx_gaus_bt & 0x03);
    tmp <<= 5;
    if (tx_gaus_en)
        tmp |= 0x10;
    vRadioRegPageSel(1);
    bRadioSetReg(CMT2310A_TX_MODE_REG_00, tmp, 0x70);
    vRadioRegPageSel(0);
}

//######################################################################
//                       Work State
//######################################################################
/******************************
 **Name:  bRadioGetState
 **Func:  Radio state
 **Input: None
 *Output: Radio State
 *        CMT2310A_STATE_IS_IDLE | CMT2310A_STATE_IS_SLEEP | CMT2310A_STATE_IS_READY | CMT2310A_STATE_IS_RFS | CMT2310A_STATE_IS_TFS | CMT2310A_STATE_IS_RX | CMT2310A_STATE_IS_TX
 * note:  page0
 ********************************/
unsigned char bRadioGetState(void)
{
    return (bRadioReadReg(CMT2310A_CHIP_MODE_STA_REG));
}

unsigned char bRadioSwitchWaiting(unsigned char targ_state)
{
    unsigned char i = 0;
    do
    {
        rt_thread_mdelay(1);						//Լ26us������SPI����Լ8us���ϼ�33us��ѯһ��
        if (bRadioGetState() == targ_state)
            break;
        i++;
    } while (i < 60);
    return (i);
}

unsigned char bRadioGoSleep(void)
{
    bRadioWriteReg(CMT2310A_CTL_REG_01, CMT2310A_GO_SLEEP);
    return (bRadioSwitchWaiting(CMT2310A_STATE_IS_SLEEP));
}

unsigned char bRadioGoStandby(void)
{
    bRadioWriteReg(CMT2310A_CTL_REG_01, CMT2310A_GO_READY);
    rt_thread_mdelay(1);
    bRadioWriteReg(CMT2310A_CTL_REG_01, CMT2310A_GO_READY);
    return (bRadioSwitchWaiting(CMT2310A_STATE_IS_READY));
}

unsigned char bRadioGoTx(void)
{
    bRadioWriteReg(CMT2310A_CTL_REG_01, CMT2310A_GO_TX);
    return (bRadioSwitchWaiting(CMT2310A_STATE_IS_TX));
}

unsigned char bRadioGoRx(void)
{
    bRadioWriteReg(CMT2310A_CTL_REG_01, CMT2310A_GO_RX);
    return (bRadioSwitchWaiting(CMT2310A_STATE_IS_RX));
}

unsigned char bRadioGoTxFS(void)
{
    bRadioWriteReg(CMT2310A_CTL_REG_01, CMT2310A_GO_TFS);
    return (bRadioSwitchWaiting(CMT2310A_STATE_IS_TFS));
}

unsigned char bRadioGoRxFS(void)
{
    bRadioWriteReg(CMT2310A_CTL_REG_01, CMT2310A_GO_RFS);
    return (bRadioSwitchWaiting(CMT2310A_STATE_IS_RFS));
}

//######################################################################
//                       FIFO Control
//######################################################################
/******************************
 **Name:  vRadioSetFifoTH
 **Func:  Radio set fifo threshold value
 **Input: fifo_th
 *Output: None
 ********************************/
void vRadioSetFifoTH(unsigned short int fifo_th)
{
    if (fifo_th >= 256)
        bRadioSetReg(CMT2310A_CTL_REG_19, CMT2310A_FIFO_TH_BIT8, CMT2310A_FIFO_TH_BIT8);
    else
        bRadioSetReg(CMT2310A_CTL_REG_19, 0, CMT2310A_FIFO_TH_BIT8);

    bRadioWriteReg(CMT2310A_CTL_REG_20, (unsigned char) fifo_th);
}

/******************************
 **Name:  vRadioFifoRetent
 **Func:  Radio set fifo retent
 **Input: cfg_en, FALSE: disable fifo retention
 *                TRUE:  enable fifo retention
 *Output: None
 * note:  fifo retention function active in sleep
 ********************************/
void vRadioFifoRetent(boolean_t cfg_en)
{
    if (cfg_en)
        bRadioSetReg(CMT2310A_CTL_REG_19, 0, CMT2310A_PD_FIFO);
    else
        bRadioSetReg(CMT2310A_CTL_REG_19, CMT2310A_PD_FIFO, CMT2310A_PD_FIFO);
}

/******************************
 **Name:  vRadioFifoAutoClearBeforeRx
 **Func:  Radio set fifo auto clear when entry to Rx
 **Input: cfg_en, FALSE: disable fifo auto clear
 *                TRUE:  enable fifo auto clear
 *Output: None
 ********************************/
void vRadioFifoAutoClearGoRx(boolean_t cfg_en)
{
    if (cfg_en)
        bRadioSetReg(CMT2310A_CTL_REG_19, CMT2310A_FIFO_AUTO_CLR_RX_EN, CMT2310A_FIFO_AUTO_CLR_RX_EN);
    else
        bRadioSetReg(CMT2310A_CTL_REG_19, 0, CMT2310A_FIFO_AUTO_CLR_RX_EN);
}

/******************************
 **Name:  vRadioFifoAutoRestoreWhenTxDone
 **Func:  Radio set fifo auto re-store when after Tx
 **Input: cfg_en, FALSE: disable fifo auto re-store
 *                TRUE:  enable fifo auto re-store
 *Output: None
 * none:  if need repeat send packet, this function should be enable
 ********************************/
void vRadioFifoAutoRestoreWhenTxDone(boolean_t cfg_en)
{
    if (cfg_en)
        bRadioSetReg(CMT2310A_CTL_REG_19, CMT2310A_FIFO_AUTO_RES_TX_EN, CMT2310A_FIFO_AUTO_RES_TX_EN);
    else
        bRadioSetReg(CMT2310A_CTL_REG_19, 0, CMT2310A_FIFO_AUTO_RES_TX_EN);
}

/******************************
 **Name:  vRadioFifoMerge
 **Func:  Radio set fifo merge together
 **Input: cfg_en, FALSE: disable fifo merge, 128B for Tx, and 128B for Rx
 *                TRUE:  enable fifo merge, 256B for Tx or Rx
 *Output: None
 ********************************/
void vRadioFifoMerge(boolean_t cfg_en)
{
    if (cfg_en)
        bRadioSetReg(CMT2310A_CTL_REG_19, CMT2310A_FIFO_MERGE_EN, CMT2310A_FIFO_MERGE_EN);
    else
        bRadioSetReg(CMT2310A_CTL_REG_19, 0, CMT2310A_FIFO_MERGE_EN);
}

/******************************
 **Name:  vRadioFifoTRxUsageSel
 **Func:  Radio set fifo used for Tx or Rx
 **Input: cfg_tx, FALSE: used for Rx
 *                TRUE:  used for Tx
 *Output: None
 * note:  when FIFO merge together active
 ********************************/
void vRadioFifoTRxUsageSel(boolean_t cfg_tx)
{
    if (cfg_tx)
        bRadioSetReg(CMT2310A_CTL_REG_19, CMT2310A_FIFO_TX_RX_SEL, CMT2310A_FIFO_TX_RX_SEL);	//for rx
    else
        bRadioSetReg(CMT2310A_CTL_REG_19, 0, CMT2310A_FIFO_TX_RX_SEL);							//for tx
}

/******************************
 **Name:  vRadioFifoGetStatus
 **Func:  Radio get fifo status
 **Input: fifo_status
 *Output: None
 ********************************/
void vRadioFifoGetStatus(FIFO_STATUS_FLG *fifo_status)
{
    (*fifo_status).FIFO_FLG_REG = bRadioReadReg(CMT2310A_CTL_REG_28);
}

/******************************
 **Name:  vRadioClearTxFifo
 **Func:  Radio clear tx fifo
 **Input: None
 *Output: None
 ********************************/
void vRadioClearTxFifo(void)
{
    bRadioSetReg(CMT2310A_CTL_REG_27, CMT2310A_TX_FIFO_CLR, CMT2310A_TX_FIFO_CLR);
}

/******************************
 **Name:  vRadioClearRxFifo
 **Func:  Radio clear rx fifo
 **Input: None
 *Output: None
 ********************************/
void vRadioClearRxFifo(void)
{
    bRadioSetReg(CMT2310A_CTL_REG_27, CMT2310A_RX_FIFO_CLR, CMT2310A_RX_FIFO_CLR);
}

/******************************
 **Name:  vRadioManualResetTxFifoPointer
 **Func:  Radio manual store tx fifo, tx fifo pointer reset
 *        can be resend the same message
 **Input: None
 *Output: None
 ********************************/
void vRadioManualResetTxFifoPointer(void)
{
    bRadioSetReg(CMT2310A_CTL_REG_27, CMT2310A_TX_FIFO_RESTORE, CMT2310A_TX_FIFO_RESTORE);
}

//######################################################################
//                       Interrupt Control
//######################################################################
/******************************
 **Name:  vRadioInterruptSoucreCfg
 **Func:  Radio set interrupt source config
 **Input: int_src_ctrl
 *Output: None
 ********************************/
void vRadioInterruptSoucreCfg(INT_SRC_CFG *int_src_ctrl)
{
    bRadioWriteReg(CMT2310A_CTL_REG_18, (*int_src_ctrl)._BYTE.INT_CTL1_REG);
    bRadioWriteReg(CMT2310A_CTL_REG_21, (*int_src_ctrl)._BYTE.INT_CTL2_REG);
    bRadioWriteReg(CMT2310A_CTL_REG_23, (*int_src_ctrl)._BYTE.INT_CTL3_REG);
    bRadioWriteReg(CMT2310A_CTL_REG_14, (*int_src_ctrl)._BYTE.INT_CTL4_REG);

}

/******************************
 **Name:  vRadioInterruptSoucreFlag
 **Func:  Radio get interrupt source flag
 **Input: int_src_flag
 *Output: None
 ********************************/
void vRadioInterruptSoucreFlag(INT_SRC_FLG *int_src_flag)
{
    (*int_src_flag)._BYTE.INT_FLAG1_REG = bRadioReadReg(CMT2310A_CTL_REG_24);
    (*int_src_flag)._BYTE.INT_FLAG2_REG = bRadioReadReg(CMT2310A_CTL_REG_26);
    (*int_src_flag)._BYTE.INT_FLAG3_REG = bRadioReadReg(CMT2310A_CTL_REG_30);
    (*int_src_flag)._BYTE.INT_FLAG4_REG = bRadioReadReg(CMT2310A_CTL_REG_32);
}

/******************************
 **Name:  vRadioInterruptSoucreClear
 **Func:  Radio clear interrupt source
 **Input: int_src_clr
 *Output: None
 ********************************/
void vRadioInterruptSoucreClear(INT_SRC_CLR *int_src_clr)
{
    bRadioWriteReg(CMT2310A_CTL_REG_24, (*int_src_clr)._BYTE.INT_CLR1_REG);
    bRadioWriteReg(CMT2310A_CTL_REG_25, (*int_src_clr)._BYTE.INT_CLR2_REG);
    bRadioWriteReg(CMT2310A_CTL_REG_29, (*int_src_clr)._BYTE.INT_CLR3_REG);
    bRadioWriteReg(CMT2310A_CTL_REG_31, (*int_src_clr)._BYTE.INT_CLR4_REG);
}

//######################################################################
//                       Config
//######################################################################

/******************************
 **Name:  vRadioConfigPageReg
 **Func:  Radio config page 0/1 regsisters
 *        page0, start address from 0x28
 *        page1, start address from 0x00
 **Input: page_sel: Page0, Page1
 *        reg_ptr:
 *        reg_len:
 *Output: None
 * note:  Page2 is not support burst mode
 ********************************/
void vRadioConfigPageReg(byte page_sel, unsigned char const reg_ptr[], unsigned char reg_len)
{
    vRadioRegPageSel(page_sel);

    vRadioBurstWriteRegs((byte *) reg_ptr, reg_len);

    vRadioRegPageSel(0);
}

void vRadioReadPageReg(byte page_sel, unsigned char reg_ptr[], unsigned char reg_len)
{
    vRadioRegPageSel(page_sel);

    vRadioBurstReadRegs(reg_ptr, reg_len);

    vRadioRegPageSel(0);
}

boolean_t bRadioIsExist(void)
{
    unsigned char back, dat;

    back = bRadioReadReg(CMT2310A_CTL_REG_12);
//    LOG_D("back = %02X", back);
    bRadioWriteReg(CMT2310A_CTL_REG_12, 0xAA);

    dat = bRadioReadReg(CMT2310A_CTL_REG_12);
//    LOG_D("dat = %02X", dat);
    bRadioWriteReg(CMT2310A_CTL_REG_12, back);

    if (0xAA == dat)
        return (TRUE);
    else
        return (FALSE);
}

uint32_t lRadioChipVersion(void)							//�ں˹̼�
{
    uint32_t chip_ver;

    vRadioRegPageSel(2);

    chip_ver = bRadioReadReg(CMT2310A_CHIP_VERSION_00);
    chip_ver <<= 8;

    chip_ver |= bRadioReadReg(CMT2310A_CHIP_VERSION_01);
    chip_ver <<= 8;

    chip_ver = bRadioReadReg(CMT2310A_CHIP_VERSION_02);

    vRadioRegPageSel(0);

    return (chip_ver);
}

