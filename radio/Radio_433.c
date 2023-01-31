/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-01     Rick       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_spi.h"
#include <string.h>
#include "AX5043.h"
#include "Radio_Common.h"
#include "pin_config.h"
#include "Radio_Decoder.h"
#include "Radio_Encoder.h"
#include "Radio_Drv.h"
#include "Config_433.h"

#define DBG_TAG "radio_433"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

struct ax5043 rf_433;

rt_sem_t IRQ1_Sem=RT_NULL;
rt_thread_t rf_433_task=RT_NULL;
rt_timer_t rf_433_send_timer = RT_NULL;

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
struct ax5043_config *rf_433_config_init(void)
{
    static struct ax5043_config *config;
    config = rt_malloc(sizeof(struct ax5043_config));
    config->axradio_phy_vcocalib = 0;
    config->axradio_phy_preamble_len = 32;
    config->axradio_phy_preamble_byte = 0x55;
    config->axradio_phy_preamble_flags = 0x38;
    config->axradio_phy_preamble_appendbits = 0;
    config->axradio_phy_preamble_appendpattern = 0x00;
    config->axradio_phy_chanfreq[0] = 0x10b81f83;
    config->axradio_phy_chanvcoiinit[0] = 0x99;
    config->axradio_phy_chanpllrnginit[0] = 0x0a;
    config->axradio_phy_maxfreqoffset = 1683;
    config->axradio_phy_rssireference = 0x3A;
    config->axradio_phy_rssioffset = 64;
    config->axradio_framing_synclen = 32;
    config->axradio_framing_syncflags = 0x38;
    config->axradio_framing_syncword[0] = 0xcc;
    config->axradio_framing_syncword[1] = 0xaa;
    config->axradio_framing_syncword[2] = 0xcc;
    config->axradio_framing_syncword[3] = 0xaa;
    return config;
}
void rf_433_sem_init(void)
{
    IRQ1_Sem = rt_sem_create("IRQ1_Sem", 0, RT_IPC_FLAG_FIFO);
}
void rf_433_driver_init(void)
{
    rf_433.config = rf_433_config_init();
    rf_433.socket = rf_433_radio_spi_init();
    strcpy(rf_433.name,"rf_433");
    memcpy(rf_433.RegValue,set_registers_433,sizeof(set_registers_433));
    memcpy(rf_433.TXRegValue,set_registers_tx_433,sizeof(set_registers_tx_433));
    memcpy(rf_433.RXRegValue,set_registers_rx_433,sizeof(set_registers_rx_433));
    IRQ1_Bounding();
    rf_startup(&rf_433);
    vcoi_rng_get(&rf_433);
    Ax5043SetRegisters_RX(&rf_433);
    AX5043ReceiverON(&rf_433);
}
void rf_433_send_timer_callback(void *parameter)
{
    if(rf_433.ubRFState != trxstate_rx)
    {
        LOG_W("rf_433 Send timeout\r\n");
        SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_RADIOEVENTMASK0, 0x00);
        rf_restart(&rf_433);
    }
}
void rf_433_send_timer_start(void)
{
    rt_timer_start(rf_433_send_timer);
}
void rf_433_task_callback(void *parameter)
{
    while(1)
    {
        static rt_err_t result;
        result = rt_sem_take(IRQ1_Sem, RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            switch (rf_433.ubRFState)
            {
            case trxstate_rx: //0x01
                ReceiveData(&rf_433);
                Ax5043SetRegisters_RX(&rf_433);
                AX5043Receiver_Continuous(&rf_433);
                if (rf_433.RxLen != 0)
                {
                    rf433_rx_callback(rf_433.ubRssi,rf_433.RXBuff,rf_433.RxLen-1);
                }
                break;
            case trxstate_wait_xtal:     //3
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_IRQMASK1, 0x00);//AX5043_IRQMASK1 = 0x00 otherwise crystal ready will fire all over again
                rf_433.ubRFState = trxstate_xtal_ready;
                break;
            case trxstate_pll_ranging:     //5
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_IRQMASK1, 0x00);//AX5043_IRQMASK1 = 0x00 otherwise autoranging done will fire all over again
                rf_433.ubRFState = trxstate_pll_ranging_done;
                break;
            case trxstate_pll_settling:     //7
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_RADIOEVENTMASK0, 0x00);// AX5043_RADIOEVENTMASK0 = 0x00;
                rf_433.ubRFState = trxstate_pll_settled;
                break;
            case trxstate_tx_xtalwait:    //9
                SpiReadSingleAddressRegister(&rf_433,REG_AX5043_RADIOEVENTREQ0); //make sure REVRDONE bit is cleared, so it is a reliable indicator that the packet is out
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_FIFOSTAT, 0x03);
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_IRQMASK1, 0x00);
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_IRQMASK0, 0x08);// enable fifo free threshold
                rf_433.ubRFState = trxstate_tx_longpreamble;
                if ((SpiReadSingleAddressRegister(&rf_433,REG_AX5043_MODULATION) & 0x0F) == 9) { // 4-FSK
                    SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_FIFODATA, (AX5043_FIFOCMD_DATA | (7 << 5)));
                    SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_FIFODATA, 2);  // length (including flags)
                    SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_FIFODATA, 0x01);  // flag PKTSTART -> dibit sync
                    SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_FIFODATA, 0x11); // dummy byte for forcing dibit sync
                }
                TransmitData(&rf_433);
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_PWRMODE, AX5043_PWRSTATE_FULL_TX); //AX5043_PWRMODE = AX5043_PWRSTATE_FULL_TX;
                break;
            case trxstate_tx_longpreamble:
            case trxstate_tx_shortpreamble:
            case trxstate_tx_packet:
                TransmitData(&rf_433);
                break;
            case trxstate_tx_waitdone:                 //D
                rt_timer_stop(rf_433_send_timer);
                SpiReadSingleAddressRegister(&rf_433,REG_AX5043_RADIOEVENTREQ0);        //clear Interrupt flag
                if (SpiReadSingleAddressRegister(&rf_433,REG_AX5043_RADIOSTATE) != 0)
                {
                    break;
                }
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_RADIOEVENTMASK0, 0x00);
                rf_restart(&rf_433);
                break;
            default:
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_IRQMASK1, 0x00);
                SpiWriteSingleAddressRegister(&rf_433,REG_AX5043_IRQMASK0, 0x00);
                break;
            }
        }
    }
}
void rf_433_start(void)
{
    rf_433_init();
    rf_led(1);
    beep_power(1);
}
void rf_test(void)
{
    rf_restart(&rf_433);
}
void rf_433_init(void)
{
    rf_433_sem_init();
    rf_433_send_timer = rt_timer_create("rf_433_send timeout", rf_433_send_timer_callback, RT_NULL, 1000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    rf_433_task = rt_thread_create("rf_433_task", rf_433_task_callback, RT_NULL, 2048, 8, 10);
    rt_thread_startup(rf_433_task);
    rf_433_driver_init();
}
