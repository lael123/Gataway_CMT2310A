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
#include "cmt2310a_Reg.h"
#include "base_types.h"
#include "cmt2310a_def.h"
#include "radio_drv.h"

#define DBG_TAG "CMTWORK"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

rt_sem_t IRQ1_Sem = RT_NULL;
rt_thread_t rf_433_task = RT_NULL;
rt_timer_t rf_433_receiver_timer = RT_NULL;

struct cmt2310 rf_433;

void rf_433_sem_init(void)
{
    IRQ1_Sem = rt_sem_create("IRQ1_Sem", 0, RT_IPC_FLAG_FIFO);
}

void IRQ1_ISR(void *parameter)
{
    rt_sem_release(IRQ1_Sem);
}

void IRQ1_Bounding(void)
{
    rt_pin_mode(Radio_IRQ, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(Radio_IRQ, PIN_IRQ_MODE_RISING, IRQ1_ISR, RT_NULL);
    rt_pin_irq_enable(Radio_IRQ, PIN_IRQ_ENABLE);
}

void rf_433_receiver_timer_start(void)
{
    rt_timer_start(rf_433_receiver_timer);
}

void rf_433_receiver_timer_callback(void *parameter)
{
    bRadioGoStandby();
    bRadioGoSleep();
    vRadioGoRxInit();
    LOG_W("rf_433 receiver timeout\r\n");
}

int32_t CMTTX_REC_Test(void)
{
    rf_433_start();
}

void rf_433_task_callback(void *parameter)
{
    static rt_err_t result;
    uint8_t g_radio_rx_buf[47];            //高频接收缓存
    uint16_t calc_crc = 0;
    uint16_t src_crc = 0;

    uint8_t Dis_payload[50] = { 0 };
    int8_t Rssi_dbm;

    vRadioRxInit();
    while (1)
    {
        result = rt_sem_take(IRQ1_Sem, RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            rf_433_receiver_timer_start();
            vRadioReadFifo(g_radio_rx_buf, 47);

            if (g_radio_rx_buf[0] == 0x21) //
            {
                memcpy(Dis_payload, g_radio_rx_buf, 33);
                calc_crc = crc16_test(33, Dis_payload);
                src_crc = two_char_to_int(g_radio_rx_buf[33], g_radio_rx_buf[34]);
                Rssi_dbm = bRadioReadReg(CMT2310A_RSSI_REG); //-128;   //  CMT2310A_RSSI_REG
                if (calc_crc == src_crc)
                {
                    rf433_rx_callback(Rssi_dbm, Dis_payload, 32);
                }
                else
                {
                    LOG_D("EURev ERRER= %s\r\n", Dis_payload);
                }
            }
            else if (g_radio_rx_buf[0] == 0x2D)
            {
                memcpy(Dis_payload, g_radio_rx_buf, 45);
                calc_crc = crc16_test(45, Dis_payload);
                src_crc = two_char_to_int(g_radio_rx_buf[45], g_radio_rx_buf[46]);
                Rssi_dbm = bRadioReadReg(CMT2310A_RSSI_REG); //-128;   //  CMT2310A_RSSI_REG
                if (calc_crc == src_crc)
                {
                    rf433_rx_callback(Rssi_dbm, Dis_payload, 44);
                }
                else
                {
                    LOG_D("MURev ERRER= %s\r\n", Dis_payload);
                }
            }
            else if (g_radio_rx_buf[0] == 0x2C)
            {
                memcpy(Dis_payload, g_radio_rx_buf, 44);
                calc_crc = crc16_test(44, Dis_payload);
                src_crc = two_char_to_int(g_radio_rx_buf[44], g_radio_rx_buf[45]);
                Rssi_dbm = bRadioReadReg(CMT2310A_RSSI_REG); //-128;   //  CMT2310A_RSSI_REG
                if (calc_crc == src_crc)
                {
                    rf433_rx_callback(Rssi_dbm, Dis_payload, 43);
                }
                else
                {
                    LOG_D("MURevA ERRER= %s\r\n", Dis_payload);
                }
            }
            else if (g_radio_rx_buf[0] == 0x2A)
            {
                memcpy(Dis_payload, g_radio_rx_buf, 42);
                calc_crc = crc16_test(42, Dis_payload);
                src_crc = two_char_to_int(g_radio_rx_buf[42], g_radio_rx_buf[43]);
                Rssi_dbm = bRadioReadReg(CMT2310A_RSSI_REG); //-128;   //  CMT2310A_RSSI_REG
                if (calc_crc == src_crc)
                {
                    LOG_D("GURev  = %s,RSSI = %d\r\n", Dis_payload, Rssi_dbm);
                }
                else
                {
                    LOG_D("GURev ERRER= %s\r\n", Dis_payload);
                }
            }
            memset(g_radio_rx_buf, 0, 47);
            memset(Dis_payload, 0, 50);
            vRadioClearRxFifo();
            vRadioClearInterrupt();
            rt_timer_stop(rf_433_receiver_timer);
            bRadioGoRx();
        }
    }
}

void rf_433_start(void)
{
    vSpiMasterInit();
    rf_433_receiver_timer = rt_timer_create("rf_433_receiver timeout", rf_433_receiver_timer_callback, RT_NULL, 1000,
    RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    rf_433_task = rt_thread_create("rf_433_task", rf_433_task_callback, RT_NULL, 2048, 5, 10);
    rt_thread_startup(rf_433_task);
    rf_433_sem_init();
    rf_led(1);
    beep_power(1);
}

