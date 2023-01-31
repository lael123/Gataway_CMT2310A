/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-06-16     Rick       the first version
 */
#ifndef RADIO_RADIO_DRV_H_
#define RADIO_RADIO_DRV_H_
#include <stdint.h>

struct ax5043
{
    struct rt_spi_device *socket;
    struct ax5043_config *config;
    char name[10];
    int ubRssi;
    uint8_t ubRFState;
    uint8_t TXBuff[64];
    uint8_t TxLen;
    uint8_t RXBuff[64];
    uint8_t RxLen;
    uint8_t axradio_phy_chanpllrng[1];
    uint8_t axradio_phy_chanvcoi[1];
    uint8_t axradio_power_now;
    uint16_t RegValue[126][2];
    uint16_t TXRegValue[6][2];
    uint16_t RXRegValue[6][2];
    uint32_t axradio_txbuffer_cnt;
};
struct ax5043_config
{
    uint8_t axradio_phy_vcocalib;
    uint8_t axradio_phy_preamble_len;
    uint8_t axradio_framing_synclen;
    uint8_t axradio_phy_preamble_flags;
    uint8_t axradio_phy_preamble_byte;
    uint8_t axradio_phy_preamble_appendbits;
    uint8_t axradio_phy_preamble_appendpattern;
    uint32_t axradio_phy_chanfreq[1];
    uint8_t axradio_phy_chanvcoiinit[1];
    uint8_t axradio_phy_chanpllrnginit[1];
    int  axradio_phy_maxfreqoffset;
    int8_t axradio_phy_rssioffset;
    int8_t axradio_phy_rssireference;
    uint8_t axradio_framing_syncflags;
    uint8_t axradio_framing_syncword[4];
};

void IRQ1_Bounding(void);
void IRQ2_Bounding(void);
void Ax5043_Spi_Init(void);
void Ax5043_Spi_Reset(struct ax5043 *dev);
void SpiWriteSingleAddressRegister(struct ax5043 *dev,uint8_t Addr, uint8_t Data);
void SpiWriteLongAddressRegister(struct ax5043 *dev,uint16_t Addr, uint8_t Data);
void SpiLongWriteLongAddressRegister(struct ax5043 *dev,uint16_t Addr, uint16_t Data);
void SpiWriteData(struct ax5043 *dev,uint8_t *pBuf,uint8_t Length);
void SpiReadData(struct ax5043 *dev,uint8_t *pBuf,uint8_t Length);
uint8_t SpiReadSingleAddressRegister(struct ax5043 *dev,uint8_t Addr);
uint8_t SpiReadLongAddressRegister(struct ax5043 *dev,uint16_t Addr);
struct rt_spi_device *rf_433_radio_spi_init(void);
struct rt_spi_device *rf_4068_radio_spi_init(void);

#endif /* RADIO_RADIO_DRV_H_ */
