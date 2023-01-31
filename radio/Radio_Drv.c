/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-06-16     Rick       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_spi.h"
#include <string.h>
#include "AX5043.h"
#include "pin_config.h"
#include "Radio_Drv.h"

#define DBG_TAG "radio_drv"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define RADIO_SPI2_NSS_PORT GPIOB
#define RADIO_SPI2_NSS_PIN GPIO_PIN_12

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
    cfg.max_hz = 5000000;             /* max 200k */

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
void Ax5043_Spi_Reset(struct ax5043 *dev)
{
    uint8_t msg[2] = {0};
    msg[0] = REG_AX5043_PWRMODE|0x80 ;
    msg[1] = 0x80 ;
    rt_spi_send(dev->socket,msg,2);
    msg[0] = REG_AX5043_PWRMODE|0x80 ;
    msg[1] = 0x00 ;
    rt_spi_send(dev->socket,msg,2);
}
void SpiWriteSingleAddressRegister(struct ax5043 *dev,uint8_t Addr, uint8_t Data)
{
    uint8_t ubAddr = Addr|0x80;
    rt_spi_send_then_send(dev->socket,&ubAddr,1,&Data,1);
}
void SpiWriteLongAddressRegister(struct ax5043 *dev,uint16_t Addr, uint8_t Data)
{
    uint16_t ubAddr;
    ubAddr = Addr|0xF000;
    uint8_t msg[2] = {0};
    msg[0] = ( ubAddr & 0xFF00 ) >> 8;
    msg[1] = ubAddr & 0x00FF;
    rt_spi_send_then_send(dev->socket,msg,2,&Data,1);
}
void SpiWriteData(struct ax5043 *dev,uint8_t *pBuf,uint8_t Length)
{
    uint8_t  data;
    data=REG_AX5043_FIFODATA| 0x80;
    rt_spi_send_then_send(dev->socket,&data,1,pBuf,Length);
}
uint8_t SpiReadSingleAddressRegister(struct ax5043 *dev,uint8_t Addr)
{
    uint8_t ubAddr ;
    uint8_t RcvAddr ;
    ubAddr = Addr&0x7F ;//read common bit7=0
    rt_spi_send_then_recv(dev->socket,&ubAddr,1,&RcvAddr,1);
    return RcvAddr ;
}
uint8_t SpiReadLongAddressRegister(struct ax5043 *dev,uint16_t Addr)
{
    uint16_t ubAddr ;
    uint8_t RcvAddr ;
    ubAddr = Addr|0x7000 ;//read common bit7=0
    uint8_t msg[2] = {0};
    msg[0] = ( ubAddr & 0xFF00 ) >> 8;
    msg[1] = ubAddr & 0x00FF;
    rt_spi_send_then_recv(dev->socket,msg,2,&RcvAddr,1);
    return RcvAddr ;
}
void SpiReadData(struct ax5043 *dev,uint8_t *pBuf,uint8_t Length)
{
    uint8_t SendAddr ;
    SendAddr=REG_AX5043_FIFODATA & 0x7F;
    rt_spi_send_then_recv(dev->socket,&SendAddr,1,pBuf,Length);
}
