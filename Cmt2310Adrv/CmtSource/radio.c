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
#include "base_types.h"
#include "cmt2310a_def.h"
#include "radio.h"

#define DBG_TAG "radio"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#define     LIMIT_TIMER_TX_CNT  100
CMT2310A_CFG g_radio;					//
uint16_t systimer;

unsigned char g_reg_read_buf[128];

extern const unsigned char g_cmt2310a_rx_page0[CMT2310A_PAGE0_SIZE];
extern const unsigned char g_cmt2310a_rx_page1[CMT2310A_PAGE1_SIZE];
extern const unsigned char g_cmt2310a_rx_page2[CMT2310A_PAGE2_SIZE];

extern const unsigned char g_cmt2310a_tx_10dbm_page0[CMT2310A_PAGE0_SIZE];
extern const unsigned char g_cmt2310a_tx_10dbm_page1[CMT2310A_PAGE1_SIZE];
extern const unsigned char g_cmt2310a_tx_10dbm_page2[CMT2310A_PAGE2_SIZE];

extern const unsigned char g_cmt2310a_tx_20dbm_page0[CMT2310A_PAGE0_SIZE];
extern const unsigned char g_cmt2310a_tx_20dbm_page1[CMT2310A_PAGE1_SIZE];
extern const unsigned char g_cmt2310a_tx_20dbm_page2[CMT2310A_PAGE2_SIZE];
//#define RF_DEBUGTEST

/******************************
 **Name:  vRadioInit
 **Func:  Radio config spi & reset
 **Input: None
 *Output: None
 ********************************/
void vRadioTxInit(void)
{
    vRadioSoftReset();

    vRadioConfigPageReg(0, g_cmt2310a_tx_10dbm_page0, CMT2310A_PAGE0_SIZE);      //config page 0
    vRadioConfigPageReg(1, g_cmt2310a_tx_10dbm_page1, CMT2310A_PAGE1_SIZE);      //config page 1
    vRadioSetNirq(CMT2310A_nIRQ_TCXO);  //1.for TCXO need cofig as nIRQ pin at first
    vRadioTcxoDrvSel(0);                //2.drive power

    vRadioPowerUpBoot();                //
    rt_thread_mdelay(10);

    bRadioGoStandby();                //2ua
    rt_thread_mdelay(2);
    bRadioApiCommand(0x02);     //镜像抑制校准 提升抗干扰
    rt_thread_mdelay(10);
    bRadioApiCommand(0x01);     //镜像抑制校准 IR Calibration, need some times

    vRadioCapLoad(2);                       //4.这个RFPDK已经设置，通过调制他可以校准中心频点
    while (bRadioIsExist() == FALSE)
    {
        LOG_D("not found CMT2310A");
        rt_thread_mdelay(1000);
    }
    //GPIOn default setting
    vRadioSetGpio0(CMT2310A_GPIO0_INT3);
    vRadioSetGpio1(CMT2310A_GPIO1_INT2);
    vRadioSetGpio2(CMT2310A_GPIO2_DCLK);
    vRadioSetGpio3(CMT2310A_GPIO3_DOUT);
    vRadioSetGpio4(CMT2310A_GPIO4_INT1);
    vRadioSetGpio5(CMT2310A_GPIO5_nRST);

    //INT1 = RX_FIFO_WBYTE,   INT2 = PKT_DONE
    vRadioSetInt1Sel(INT_SRC_RX_FIFO_WBYTE);
    vRadioSetInt2Sel(INT_SRC_PKT_DONE);
    vRadioSetInt1Polar(FALSE);
    vRadioSetInt2Polar(FALSE);
    vRadioSetInt3Polar(FALSE);

    //interrupt source enable config
    g_radio.int_src_en._BITS.PKT_DONE_EN = 1;
    g_radio.int_src_en._BITS.CRC_PASS_EN = 1;
    g_radio.int_src_en._BITS.ADDR_PASS_EN = 0;
    g_radio.int_src_en._BITS.SYNC_PASS_EN = 1;
    g_radio.int_src_en._BITS.PREAM_PASS_EN = 1;
    g_radio.int_src_en._BITS.TX_DONE_EN = 1;
    g_radio.int_src_en._BITS.RX_TOUT_EN = 1;            //曼码时需要使能
    g_radio.int_src_en._BITS.LD_STOP_EN = 0;
    g_radio.int_src_en._BITS.LBD_STOP_EN = 0;
    g_radio.int_src_en._BITS.LBD_STAT_EN = 0;
    g_radio.int_src_en._BITS.PKT_ERR_EN = 0;
    g_radio.int_src_en._BITS.RSSI_COLL_EN = 0;
    g_radio.int_src_en._BITS.OP_CMD_FAILED_EN = 0;
    g_radio.int_src_en._BITS.RSSI_PJD_EN = 0;
    g_radio.int_src_en._BITS.SEQ_MATCH_EN = 0;
    g_radio.int_src_en._BITS.NACK_RECV_EN = 0;
    g_radio.int_src_en._BITS.TX_RESEND_DONE_EN = 0;
    g_radio.int_src_en._BITS.ACK_RECV_FAILED_EN = 0;
    g_radio.int_src_en._BITS.TX_DC_DONE_EN = 0;
    g_radio.int_src_en._BITS.CSMA_DONE_EN = 0;
    g_radio.int_src_en._BITS.CCA_STAT_EN = 0;
    g_radio.int_src_en._BITS.API_DONE_EN = 0;
    g_radio.int_src_en._BITS.TX_FIFO_TH_EN = 1;
    g_radio.int_src_en._BITS.TX_FIFO_NMTY_EN = 1;
    g_radio.int_src_en._BITS.TX_FIFO_FULL_EN = 1;
    g_radio.int_src_en._BITS.RX_FIFO_OVF_EN = 1;
    g_radio.int_src_en._BITS.RX_FIFO_TH_EN = 1;
    g_radio.int_src_en._BITS.RX_FIFO_NMTY_EN = 1;
    g_radio.int_src_en._BITS.RX_FIFO_FULL_EN = 1;
    vRadioInterruptSoucreCfg(&g_radio.int_src_en);

    //FIFO Init
    vRadioFifoMerge(FALSE);                                              //7.不合并fifo，txrx每个128BYTE，合并之后256BYTE
    vRadioSetFifoTH(30);                                                        //8.设置EIEO 阔值中断
    vRadioClearRxFifo();                                                        //9.reset & clear fifo
    vRadioClearTxFifo();

    vRadioFifoAutoClearGoRx(TRUE);                           //10.when crc error, need to auto clear fifo, should enable

    vRadioRssiUpdateSel(CMT2310A_RSSI_UPDATE_ALWAYS);                                  //11. rssi 实时监测always

    vRadioSetAntSwitch(FALSE, FALSE);                                //12 . 关闭电子开关 GPIO

    vRadioDcdcCfg(TRUE);                                                         //13 . dc-dc on
}

void vRadioRxInit(void)
{
    vRadioSoftReset();

    vRadioConfigPageReg(0, g_cmt2310a_rx_page0, CMT2310A_PAGE0_SIZE);      //config page 0
    vRadioConfigPageReg(1, g_cmt2310a_rx_page1, CMT2310A_PAGE1_SIZE);      //config page 1

    vRadioSetNirq(CMT2310A_nIRQ_TCXO);  //1.for TCXO need cofig as nIRQ pin at first
    vRadioTcxoDrvSel(0);                //2.drive power

    vRadioPowerUpBoot();                //
    rt_thread_mdelay(10);

    bRadioGoStandby();                //2ua
    rt_thread_mdelay(2);
    bRadioApiCommand(0x02);     //镜像抑制校准 提升抗干扰
    rt_thread_mdelay(10);
    bRadioApiCommand(0x01);     //镜像抑制校准 IR Calibration, need some times

    vRadioCapLoad(2);                       //4.这个RFPDK已经设置，通过调制他可以校准中心频点
    while (bRadioIsExist() == FALSE)
    {
        LOG_D("not found CMT2310A");
        rt_thread_mdelay(1000);
    }
    //GPIOn default setting
    vRadioSetGpio0(CMT2310A_GPIO0_INT3);
    vRadioSetGpio1(CMT2310A_GPIO1_INT2);
    vRadioSetGpio2(CMT2310A_GPIO2_DCLK);
    vRadioSetGpio3(CMT2310A_GPIO3_DOUT);
    vRadioSetGpio4(CMT2310A_GPIO4_INT1);
    vRadioSetGpio5(CMT2310A_GPIO5_nRST);

    //INT1 = RX_FIFO_WBYTE,   INT2 = PKT_DONE
    vRadioSetInt1Sel(INT_SRC_RX_FIFO_WBYTE);
    vRadioSetInt2Sel(INT_SRC_PKT_DONE);
    vRadioSetInt1Polar(FALSE);
    vRadioSetInt2Polar(FALSE);
    vRadioSetInt3Polar(FALSE);

    //interrupt source enable config
    g_radio.int_src_en._BITS.PKT_DONE_EN = 1;
    g_radio.int_src_en._BITS.CRC_PASS_EN = 1;
    g_radio.int_src_en._BITS.ADDR_PASS_EN = 0;
    g_radio.int_src_en._BITS.SYNC_PASS_EN = 1;
    g_radio.int_src_en._BITS.PREAM_PASS_EN = 1;
    g_radio.int_src_en._BITS.TX_DONE_EN = 1;
    g_radio.int_src_en._BITS.RX_TOUT_EN = 1;            //曼码时需要使能
    g_radio.int_src_en._BITS.LD_STOP_EN = 0;
    g_radio.int_src_en._BITS.LBD_STOP_EN = 0;
    g_radio.int_src_en._BITS.LBD_STAT_EN = 0;
    g_radio.int_src_en._BITS.PKT_ERR_EN = 0;
    g_radio.int_src_en._BITS.RSSI_COLL_EN = 0;
    g_radio.int_src_en._BITS.OP_CMD_FAILED_EN = 0;
    g_radio.int_src_en._BITS.RSSI_PJD_EN = 0;
    g_radio.int_src_en._BITS.SEQ_MATCH_EN = 0;
    g_radio.int_src_en._BITS.NACK_RECV_EN = 0;
    g_radio.int_src_en._BITS.TX_RESEND_DONE_EN = 0;
    g_radio.int_src_en._BITS.ACK_RECV_FAILED_EN = 0;
    g_radio.int_src_en._BITS.TX_DC_DONE_EN = 0;
    g_radio.int_src_en._BITS.CSMA_DONE_EN = 0;
    g_radio.int_src_en._BITS.CCA_STAT_EN = 0;
    g_radio.int_src_en._BITS.API_DONE_EN = 0;
    g_radio.int_src_en._BITS.TX_FIFO_TH_EN = 1;
    g_radio.int_src_en._BITS.TX_FIFO_NMTY_EN = 1;
    g_radio.int_src_en._BITS.TX_FIFO_FULL_EN = 1;
    g_radio.int_src_en._BITS.RX_FIFO_OVF_EN = 1;
    g_radio.int_src_en._BITS.RX_FIFO_TH_EN = 1;
    g_radio.int_src_en._BITS.RX_FIFO_NMTY_EN = 1;
    g_radio.int_src_en._BITS.RX_FIFO_FULL_EN = 1;
    vRadioInterruptSoucreCfg(&g_radio.int_src_en);

    //FIFO Init
    vRadioFifoMerge(FALSE);                                              //7.不合并fifo，txrx每个128BYTE，合并之后256BYTE
    vRadioSetFifoTH(30);                                                        //8.设置EIEO 阔值中断
    vRadioClearRxFifo();                                                        //9.reset & clear fifo
    vRadioClearTxFifo();

    vRadioFifoAutoClearGoRx(TRUE);                           //10.when crc error, need to auto clear fifo, should enable

    vRadioRssiUpdateSel(CMT2310A_RSSI_UPDATE_ALWAYS);                                  //11.rssi 实时监测always

    vRadioSetAntSwitch(FALSE, FALSE);                                //12 .关闭电子开关 GPIO

    vRadioDcdcCfg(TRUE);                                                         //13 . dc-dc on

    g_radio.frame_cfg.PAYLOAD_LENGTH = 47;
    vRadioSetPayloadLength(&g_radio.frame_cfg);
    vRadioSetInt1Sel(CMT2310A_INT_PKT_DONE);
    vRadioSetInt2Sel(CMT2310A_INT_RX_FIFO_WBYTE);
    vRadioRssiUpdateSel(CMT2310A_RSSI_UPDATE_SYNC_OK);      //11.  rssi 实时监测 always
    bRadioGoRx();
    IRQ1_Bounding();

}

void vRadioClearInterrupt(void)
{
    vRadioInterruptSoucreFlag(&g_radio.int_src_flag);

    g_radio.int_src_clear._BITS.SLEEP_TMO_CLR = g_radio.int_src_flag._BITS.SLEEP_TMO_FLG;
    g_radio.int_src_clear._BITS.RX_TMO_CLR = g_radio.int_src_flag._BITS.RX_TMO_FLG;
    g_radio.int_src_clear._BITS.TX_DONE_CLR = g_radio.int_src_flag._BITS.TX_DONE_FLG;

    g_radio.int_src_clear._BITS.PKT_DONE_CLR = g_radio.int_src_flag._BITS.PKT_DONE_FLG;
    g_radio.int_src_clear._BITS.CRC_PASS_CLR = g_radio.int_src_flag._BITS.CRC_PASS_FLG;
    g_radio.int_src_clear._BITS.ADDR_PASS_CLR = g_radio.int_src_flag._BITS.ADDR_PASS_FLG;
    g_radio.int_src_clear._BITS.SYNC_PASS_CLR = g_radio.int_src_flag._BITS.SYNC_PASS_FLG
            | g_radio.int_src_flag._BITS.SYNC1_PASS_FLG;
    g_radio.int_src_clear._BITS.PREAM_PASS_CLR = g_radio.int_src_flag._BITS.PREAM_PASS_FLG;

    g_radio.int_src_clear._BITS.LBD_STAT_CLR = g_radio.int_src_flag._BITS.LBD_STATUS_FLG;
    g_radio.int_src_clear._BITS.PKT_ERR_CLR = g_radio.int_src_flag._BITS.PKT_ERR_FLG;
    g_radio.int_src_clear._BITS.RSSI_COLL_CLR = g_radio.int_src_flag._BITS.RSSI_COLL_FLG;
    g_radio.int_src_clear._BITS.OP_CMD_FAILED_CLR = g_radio.int_src_flag._BITS.OP_CMD_FAILED_FLG;
    g_radio.int_src_clear._BITS.ANT_LOCK_CLR = g_radio.int_src_flag._BITS.ANT_LOCK_FLG;

    g_radio.int_src_clear._BITS.SEQ_MATCH_CLR = g_radio.int_src_flag._BITS.SEQ_MATCH_FLG;
    g_radio.int_src_clear._BITS.NACK_RECV_CLR = g_radio.int_src_flag._BITS.NACK_RECV_FLG;
    g_radio.int_src_clear._BITS.TX_RESEND_DONE_CLR = g_radio.int_src_flag._BITS.TX_RESEND_DONE_FLG;
    g_radio.int_src_clear._BITS.ACK_RECV_FAILED_CLR = g_radio.int_src_flag._BITS.ACK_RECV_FAILED_FLG;
    g_radio.int_src_clear._BITS.TX_DC_DONE_CLR = g_radio.int_src_flag._BITS.TX_DC_DONE_FLG;
    g_radio.int_src_clear._BITS.CSMA_DONE_CLR = g_radio.int_src_flag._BITS.CSMA_DONE_FLG;
    g_radio.int_src_clear._BITS.CCA_STATUS_CLR = g_radio.int_src_flag._BITS.CCA_STATUS_FLG;
    g_radio.int_src_clear._BITS.API_DONE_CLR = g_radio.int_src_flag._BITS.API_DONE_FLG;

    vRadioInterruptSoucreClear(&g_radio.int_src_clear);
}

void vRadioReadAllStatus(void)
{
    bRadioGetState();                                   //读取工作状态
    vRadioFifoGetStatus(&g_radio.fifo_status_flag);     //读取EIEO状态
    vRadioInterruptSoucreFlag(&g_radio.int_src_flag);   //读取各个中断状态

    bRadioReadReg(CMT2310A_CTL_REG_04);                 //读取GPIO0/1的设置
    bRadioReadReg(CMT2310A_CTL_REG_05);                 //读取GPIO2/3的设置
    bRadioReadReg(CMT2310A_CTL_REG_06);                 //读取GPIO4/5的设置

    bRadioReadReg(CMT2310A_CTL_REG_16);                 //读取INT1设置
    bRadioReadReg(CMT2310A_CTL_REG_17);                 //读取INT2设置
}

void vRadioCmpReg(byte const wr_ptr[], byte rd_ptr[], byte cmp_ptr[], byte length)
{
    byte i;

    for (i = 0; i < length; i++)
    {
        if (wr_ptr[i] != rd_ptr[i])
            cmp_ptr[i] = 0xFF;
        else
            cmp_ptr[i] = 0x00;
    }
}

void vRadioGoTxInit(void)
{
    IRQ1_DisIrq();

    vRadioConfigPageReg(0, g_cmt2310a_tx_10dbm_page0, CMT2310A_PAGE0_SIZE); //config page 0
    vRadioConfigPageReg(1, g_cmt2310a_tx_10dbm_page1, CMT2310A_PAGE1_SIZE); //config page 1

    vRadioSetInt1Sel(CMT2310A_INT_TX_DONE);
    vRadioSetInt2Sel(CMT2310A_INT_TX_FIFO_NMTY);
}

void vRadioGoRxInit(void)
{
    vRadioConfigPageReg(0, g_cmt2310a_rx_page0, CMT2310A_PAGE0_SIZE);      //config page 0
    vRadioConfigPageReg(1, g_cmt2310a_rx_page1, CMT2310A_PAGE1_SIZE);      //config page 1
    g_radio.frame_cfg.PAYLOAD_LENGTH = 47;
    vRadioSetPayloadLength(&g_radio.frame_cfg);
    vRadioSetInt1Sel(CMT2310A_INT_PKT_DONE);
    vRadioSetInt2Sel(CMT2310A_INT_RX_FIFO_WBYTE);
    vRadioRssiUpdateSel(CMT2310A_RSSI_UPDATE_SYNC_OK);      //11.rssi 实时监测 always
    bRadioGoRx();
    IRQ1_Bounding();
}

void Normal_send(uint8_t *Buf, uint8_t Length)
{
    vRadioGoTxInit();
    g_radio.frame_cfg.PAYLOAD_LENGTH = Length;
    vRadioSetPayloadLength(&g_radio.frame_cfg);
    vRadioWriteFifo(Buf, Length);
    bRadioGoTx();
    for (systimer = 0; systimer < LIMIT_TIMER_TX_CNT; systimer++)
    {
        if (RF_GPIO4_H())
        {
            bRadioGoStandby();
            vRadioClearTxFifo();
            vRadioClearInterrupt();
            bRadioGoSleep();
            vRadioGoRxInit();
            break;
        }
        rt_thread_mdelay(1);
    }
    if (systimer >= LIMIT_TIMER_TX_CNT)
    {
        bRadioGoStandby();
        bRadioGoSleep();
        vRadioGoRxInit();
    }
}

void RF_GPIO4_Init(void)
{
    rt_pin_mode(Radio_IRQ, PIN_MODE_INPUT);
}

void RF_GPIO4_H(void)
{
    return rt_pin_read(Radio_IRQ);
}

void IRQ1_EnIrq(void)
{
    rt_pin_irq_enable(Radio_IRQ, PIN_IRQ_ENABLE);
}
void IRQ1_DisIrq(void)
{
    rt_pin_irq_enable(Radio_IRQ, PIN_IRQ_DISABLE);
}

