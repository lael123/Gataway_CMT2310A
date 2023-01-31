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
#include "radio_drv.h"

#include <stdio.h>

#define DBG_TAG "radio_spi"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

extern struct cmt2310 rf_433;

#define RADIO_SPI2_NSS_PORT GPIOB
#define RADIO_SPI2_NSS_PIN GPIO_PIN_12

#define     SPI_DUMMY_VALUE     0x00

/******************************
 **Name: vSpiMasterInit
 **Func: SPI Master
 **Input: None
 *Output: None
 ********************************/
void vSpiMasterInit(void)
{
    rf_433.socket = rf_433_radio_spi_init();
}

struct rt_spi_device *rf_433_radio_spi_init(void)
{
    rt_err_t res;
    static struct rt_spi_device *radio_spi_device;
    rt_hw_spi_device_attach("spi2", "rf_433", RADIO_SPI2_NSS_PORT, RADIO_SPI2_NSS_PIN);
    radio_spi_device = (struct rt_spi_device *)rt_device_find("rf_433");
    if (!radio_spi_device)
    {
        LOG_D("spi_device_attach failed! cant't find rf_433 device!\n");
        return RT_NULL;
    }
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0. */
    cfg.max_hz = 5000000;             /* 大于 2M 才能响应 */

    res = rt_spi_configure(radio_spi_device, &cfg);

    if (res != RT_EOK)
    {
        LOG_D("rt_spi_configure failed!\r\n");
    }
    res = rt_spi_take_bus(radio_spi_device);
    if (res != RT_EOK)
    {
        LOG_D("rt_spi_take_bus failed!\r\n");
    }

    res = rt_spi_release_bus(radio_spi_device);

    if(res != RT_EOK)
    {
        LOG_D("rt_spi_release_bus failed!\r\n");
    }

    return radio_spi_device;
}

byte  bSpiWriteByte(byte spi_adr, byte spi_dat)
{
    spi_adr &= 0x7F;
    rt_spi_send_then_send(rf_433.socket,&spi_adr,1,&spi_dat,1);
    return 1;
}

uint8_t bSpiReadByte(uint8_t spi_adr)
{
    spi_adr |= 0x80;
    uint8_t RcvAddr ;
    rt_spi_send_then_recv(rf_433.socket,&spi_adr,1,&RcvAddr,1);
    return RcvAddr ;
}

void vSpiBurstWrite(uint8_t spi_adr,uint8_t data[],uint8_t Length)
{
    spi_adr &= 0x7F;
    rt_spi_send_then_send(rf_433.socket,&spi_adr,1,data,Length);
}

void vSpiBurstRead(uint8_t spi_adr,uint8_t data[],uint8_t Length)
{
    spi_adr |= 0x80;
    rt_spi_send_then_recv(rf_433.socket,&spi_adr ,1,data,Length);
}


