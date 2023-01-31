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

#define DBG_TAG "radio_common"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

uint8_t axradio_get_pllvcoi(struct ax5043 *dev)
{
    if (dev->config->axradio_phy_vcocalib)
    {
        uint8_t x = dev->axradio_phy_chanvcoi[0];
        if (x & 0x80)
            return x;
    }
    {
        uint8_t x = dev->config->axradio_phy_chanvcoiinit[0];
        if (x & 0x80)
        {
            if (!(dev->config->axradio_phy_chanpllrnginit[0] & 0xF0)) {
                x += (dev->axradio_phy_chanpllrng[0] & 0x0F) - (dev->config->axradio_phy_chanpllrnginit[0] & 0x0F);
                x &= 0x3f;
                x |= 0x80;
            }
            return x;
        }
    }
    return SpiReadLongAddressRegister(dev,REG_AX5043_PLLVCOI);
}
void vcoi_rng_get(struct ax5043 *dev)
{
    LOG_I("%s VCO is %x\r\n",dev->name,axradio_get_pllvcoi(dev));
    LOG_I("%s RNG is %d\r\n",dev->name,dev->axradio_phy_chanpllrng[0]);
}
void InitAx5043REG(struct ax5043 *dev)
{
    static uint8_t i;

    for (i = 0; i<125 ; i++)
    {
        SpiWriteLongAddressRegister(dev,dev->RegValue[i][0],dev->RegValue[i][1] );
    }
    SpiWriteLongAddressRegister(dev,REG_AX5043_PKTLENOFFSET, (SpiReadLongAddressRegister(dev,REG_AX5043_PKTLENOFFSET)));
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PINFUNCIRQ, 0x03);    // AX5043_PINFUNCIRQ = 0x03;  use as IRQ pin
    SpiWriteLongAddressRegister(dev,REG_AX5043_PKTSTOREFLAGS, (0 ? 0x13 : 0x15)); // store RF offset, RSSI and delimiter timing
}
uint8_t ax5043_init_registers_common(struct ax5043 *dev)
{
    uint8_t rng = dev->axradio_phy_chanpllrng[0];
    if (rng & 0x20)
        return AXRADIO_ERR_RANGING;
    if (SpiReadSingleAddressRegister(dev,REG_AX5043_PLLLOOP) & 0x80)
        SpiWriteSingleAddressRegister(dev,REG_AX5043_PLLRANGINGB, (rng & 0x0F));
    else
        SpiWriteSingleAddressRegister(dev,REG_AX5043_PLLRANGINGA, (rng & 0x0F));
    rng = axradio_get_pllvcoi(dev);
    if (rng & 0x80)
        SpiWriteLongAddressRegister(dev,REG_AX5043_PLLVCOI, rng);
    return AXRADIO_ERR_NOERROR;
}
uint8_t Ax5043SetRegisters_TX(struct ax5043 *dev)
{
    static uint8_t i;

    for (i = 0 ; i < 6 ; i++)
    {
        SpiWriteLongAddressRegister(dev,dev->TXRegValue[i][0],dev->TXRegValue[i][1] );
    }
    return ax5043_init_registers_common(dev);
}
uint8_t Ax5043SetRegisters_RX(struct ax5043 *dev)
{
    static uint8_t i;

    for (i = 0 ; i < 6 ; i++)
    {
        SpiWriteLongAddressRegister(dev,dev->RXRegValue[i][0],dev->RXRegValue[i][1] );
    }
    return ax5043_init_registers_common(dev);
}
void Ax5043_Reset(struct ax5043 *dev)
{
    static uint8_t ubAddres;
    Ax5043_Spi_Reset(dev);

    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, 0x00);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_SCRATCH, 0x55);
    do
    {
        ubAddres = SpiReadSingleAddressRegister(dev,REG_AX5043_SCRATCH);
    }
    while (ubAddres != 0x55);

    SpiWriteSingleAddressRegister(dev,REG_AX5043_PINFUNCIRQ, 0x01);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PINFUNCIRQ, 0x00);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PINFUNCIRQ, 0x03);
}
void RadioXtalON(struct ax5043 *dev)
{
    static uint8_t ubTemp;
    dev->ubRFState = trxstate_wait_xtal;
    ubTemp = SpiReadSingleAddressRegister(dev,REG_AX5043_IRQMASK1);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK1, ubTemp|0x01);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON);
    do
    {
        rt_thread_mdelay(10);
    }
    while (dev->ubRFState == trxstate_wait_xtal);
}
void AX5043ReceiverON(struct ax5043 *dev)
{
    SpiWriteLongAddressRegister(dev,REG_AX5043_RSSIREFERENCE, dev->config->axradio_phy_rssireference);
    SpiWriteLongAddressRegister(dev,REG_AX5043_TMGRXPREAMBLE1, 0x00);
    SpiWriteLongAddressRegister(dev,REG_AX5043_PKTSTOREFLAGS , 0x50 );
    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFOSTAT,0x03);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE,AX5043_PWRSTATE_FULL_RX);
    dev->ubRFState = trxstate_rx;
    SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK0,0x01);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK1,0x00);
}
void AX5043Receiver_Continuous(struct ax5043 *dev)
{
    uint32_t ubTemp=0;

    SpiWriteSingleAddressRegister(dev,REG_AX5043_RADIOEVENTMASK0, 0x04);
    SpiWriteLongAddressRegister(dev,REG_AX5043_RSSIREFERENCE, dev->config->axradio_phy_rssireference);

    SpiWriteLongAddressRegister(dev,REG_AX5043_TMGRXAGC, 0);
    SpiWriteLongAddressRegister(dev,REG_AX5043_TMGRXPREAMBLE1, 0);
    SpiWriteLongAddressRegister(dev,REG_AX5043_PKTMISCFLAGS , 0 );

    ubTemp = SpiReadLongAddressRegister(dev,REG_AX5043_PKTSTOREFLAGS);
    ubTemp &= ~0x40;
    SpiWriteLongAddressRegister(dev,REG_AX5043_PKTSTOREFLAGS, ubTemp);

    SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK0,0x01);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFOSTAT,0x03);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE,AX5043_PWRSTATE_FULL_RX);
    dev->ubRFState = trxstate_rx;
    SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK1,0x00);

}
void ReceiveData(struct ax5043 *dev)
{
    uint8_t ubDataLen,ubTepm;
    uint8_t ubTemCom;
    uint8_t ubOffSet;
    uint32_t uwFreqOffSet;

    dev->ubRssi = 0;
    dev->RxLen = 0;
    memset(dev->RXBuff,0,sizeof(dev->RXBuff));

    ubDataLen = SpiReadSingleAddressRegister(dev,REG_AX5043_RADIOEVENTREQ0);

    while (SpiReadSingleAddressRegister(dev,REG_AX5043_IRQREQUEST0) & 0x01)
    {
        ubTemCom = SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
        ubDataLen = (ubTemCom & 0xE0) >>5 ;
        if (ubDataLen == 0x07)
        {
            ubDataLen = SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
        }
        ubTemCom &=0x1F;
        switch (ubTemCom)
        {
            case AX5043_FIFOCMD_DATA:
                if (!ubDataLen)
                    break;
                SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                --ubDataLen;
                SpiReadData(dev,dev->RXBuff,ubDataLen);
                dev->RxLen = ubDataLen;
                ubTepm = SpiReadSingleAddressRegister(dev,REG_AX5043_IRQMASK0);
                SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK0, ubTepm & (uint8_t)~0x01);
                break;

            case AX5043_FIFOCMD_RFFREQOFFS:
                if (ubDataLen != 3)
                    goto dropchunk;
                ubOffSet = SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                ubOffSet &= 0x0F;
                ubOffSet |= 1+(~(ubOffSet&0x08));
                uwFreqOffSet=((signed char)ubOffSet)>>8 ;
                uwFreqOffSet <<=8;
                uwFreqOffSet |= ubOffSet;
                uwFreqOffSet <<=8;
                ubOffSet = SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                uwFreqOffSet |= ubOffSet;
                uwFreqOffSet <<=8;
                ubOffSet =SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                uwFreqOffSet |= ubOffSet;
                break;

            case AX5043_FIFOCMD_RSSI:
                if (ubDataLen != 1)
                    goto dropchunk;
                {
                    int8_t r = SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                    dev->ubRssi = r - (int16_t)dev->config->axradio_phy_rssioffset;
                }
                break;
            case AX5043_FIFOCMD_TIMER:
                if (ubDataLen != 3)
                    goto dropchunk;
                SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                break;
            case AX5043_FIFOCMD_ANTRSSI:
                if (!ubDataLen)
                    break;
                {
                    SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                }
                --ubDataLen;
                goto dropchunk;
            default:
            dropchunk:
                if (!ubDataLen)
                    break;
                ubTepm = ubDataLen;
                do
                {
                    SpiReadSingleAddressRegister(dev,REG_AX5043_FIFODATA);
                }
                while (--ubTepm);
                break;
        }
    }
}
void Ax5043_OFF(struct ax5043 *dev)
{
    SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK0, 0x00);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK1, 0x00);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON);
    SpiWriteLongAddressRegister(dev,REG_AX5043_LPOSCCONFIG, 0x00);
    dev->ubRFState = trxstate_off;
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);
}
void TransmitData(struct ax5043 *dev)
{
    uint8_t ubFreeCnt;
    while(1)
    {
        ubFreeCnt = SpiReadSingleAddressRegister(dev,REG_AX5043_FIFOFREE0); //uint8_t cnt = AX5043_FIFOFREE0;
        if (SpiReadSingleAddressRegister(dev,REG_AX5043_FIFOFREE1))ubFreeCnt = 0xff;
        switch (dev->ubRFState)
        {
            case trxstate_tx_longpreamble: //A
                if (dev->axradio_txbuffer_cnt==0)
                {
                    dev->ubRFState = trxstate_tx_shortpreamble;
                    dev->axradio_txbuffer_cnt = dev->config->axradio_phy_preamble_len;
                    goto shortpreamble;
                }
                if (ubFreeCnt < 4)
                    goto fifocommit;
                ubFreeCnt = 7;
                if (dev->axradio_txbuffer_cnt < 7)
                {
                    ubFreeCnt = dev->axradio_txbuffer_cnt;
                }
                dev->axradio_txbuffer_cnt -= ubFreeCnt;
                ubFreeCnt <<= 5;
                SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, 0x02 | (3 << 5));     //AX5043_FIFODATA = AX5043_FIFOCMD_REPEATDATA | (3 << 5); //0x62  bit[7:5]= 011 Three byte payload;#define AX5043_FIFOCMD_REPEATDATA 0x02
                SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, 0x38);     //AX5043_FIFODATA = axradio_phy_preamble_flags;  //axradio_phy_preamble_flags = 0x38; 0b0011 1000  bit5=UNENC bit4=RAW  bit3=NOCRC
                SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, ubFreeCnt);//AX5043_FIFODATA = cnt;
                SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, 0x55);      //AX5043_FIFODATA = axradio_phy_preamble_byte; //axradio_phy_preamble_byte = 0x55;
                break;
            case trxstate_tx_shortpreamble:   //B
            shortpreamble:
                if (dev->axradio_txbuffer_cnt == 0)
                {
                    if (ubFreeCnt < 15)
                        goto fifocommit;
                    if (dev->config->axradio_phy_preamble_appendbits) {
                        uint8_t byte;
                        SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, (AX5043_FIFOCMD_DATA | (2 << 5)));
                        SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, 0x1C);
                        byte = dev->config->axradio_phy_preamble_appendpattern;
                        if (SpiReadLongAddressRegister(dev,REG_AX5043_PKTADDRCFG) & 0x80) {
                            // msb first -> stop bit below
                            byte &= 0xFF << (8-dev->config->axradio_phy_preamble_appendbits);
                            byte |= 0x80 >> dev->config->axradio_phy_preamble_appendbits;
                        } else {
                            // lsb first -> stop bit above
                            byte &= 0xFF >> (8-dev->config->axradio_phy_preamble_appendbits);
                            byte |= 0x01 << dev->config->axradio_phy_preamble_appendbits;
                        }
                        SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, byte);
                    }
                    if ((SpiReadSingleAddressRegister(dev,REG_AX5043_FRAMING) & 0x0E) == 0x06 && dev->config->axradio_framing_synclen)                                  //axradio_framing_synclen=32
                    {
                        uint8_t len_byte = dev->config->axradio_framing_synclen;
                        uint8_t i = (len_byte & 0x07) ? 0x04 : 0;
                        len_byte += 7;
                        len_byte >>= 3;
                        SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA,0x01 | ((len_byte + 1) << 5));//AX5043_FIFODATA =   0xA1; //0x01 | ((len_byte + 1) << 5);  //0xA1
                        SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, dev->config->axradio_framing_syncflags | i);
                        for (i = 0; i < len_byte; ++i) {
                            // better put a brace, it might prevent SDCC from optimizing away the assignement...
                            SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, dev->config->axradio_framing_syncword[i]);
                        }
                        dev->ubRFState = trxstate_tx_packet;
                        break;
                    }
                }
                if (ubFreeCnt < 4)goto fifocommit;
                ubFreeCnt = 255;
                if (dev->axradio_txbuffer_cnt < (255*8))
                    ubFreeCnt = dev->axradio_txbuffer_cnt >> 3;
                if (ubFreeCnt)
                {
                    dev->axradio_txbuffer_cnt -= ((uint16_t)ubFreeCnt) << 3;
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA , AX5043_FIFOCMD_REPEATDATA | (3 << 5));
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA ,dev->config->axradio_phy_preamble_flags);
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA , ubFreeCnt);
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA ,dev->config->axradio_phy_preamble_byte);
                    break;
                }
                {
                    uint8_t byte = dev->config->axradio_phy_preamble_byte;
                    ubFreeCnt = dev->axradio_txbuffer_cnt;
                    dev->axradio_txbuffer_cnt = 0;
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA ,AX5043_FIFOCMD_DATA | (2 << 5));
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, 0x1C);
                    if (SpiReadLongAddressRegister(dev,REG_AX5043_PKTADDRCFG) & 0x80)
                    {
                        // msb first -> stop bit below
                        byte &= 0xFF << (8-ubFreeCnt);
                        byte |= 0x80 >> ubFreeCnt;
                    }
                    else
                    {
                        // lsb first -> stop bit above
                        byte &= 0xFF >> (8-ubFreeCnt);
                        byte |= 0x01 << ubFreeCnt;
                    }
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, byte);
                }
                break;
            case trxstate_tx_packet:  //C
                if (dev->TxLen < 11)goto fifocommit;
                SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA, AX5043_FIFOCMD_DATA | (7 << 5));//AX5043_FIFODATA = AX5043_FIFOCMD_DATA | (7 << 5);
                SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA,1 + dev->TxLen);//write FIFO chunk length byte (data length+MAC_len + Flag byte=64+3+1)
                SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFODATA,3);//AX5043_FIFODATA = flags=0x03;
                SpiWriteData(dev,dev->TXBuff,dev->TxLen);          //ax5043_writefifo(&axradio_txbuffer[axradio_txbuffer_cnt], cnt);
                goto pktend;
            default:
                return;
        }
    }
    fifocommit:
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFOSTAT, 4); // commit
    pktend:
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFOSTAT, 4); // commit
        dev->ubRFState = trxstate_tx_waitdone;
        SpiWriteSingleAddressRegister(dev,REG_AX5043_RADIOEVENTMASK0, 0x01); // enable REVRDONE event
        SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK0, 0x40); // enable radio controller irq
}
void transmit_packet_task(struct ax5043 *dev,uint8_t *Buf, uint8_t Length)
{
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON); //AX5043_PWRMODE = AX5043_PWRSTATE_XTAL_ON;    Crystal Oscillator enabled
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, AX5043_PWRSTATE_FIFO_ON); //AX5043_PWRMODE = AX5043_PWRSTATE_FIFO_ON;    FIFO enabled
    Ax5043SetRegisters_TX(dev);
    dev->TxLen = Length;
    dev->TxLen++;
    dev->TXBuff[0] = dev->TxLen;
    memcpy(dev->TXBuff+1, Buf, Length);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFOTHRESH1, 0x00);  //AX5043_FIFOTHRESH1(0x2E) = 0;
    SpiWriteSingleAddressRegister(dev,REG_AX5043_FIFOTHRESH0, 0x80);  //AX5043_FIFOTHRESH0(0x2F) = 0x80;
    dev->ubRFState = trxstate_tx_xtalwait;
    SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK0, 0x00);     //AX5043_IRQMASK0 = 0x00;
    SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK1, 0x01);     //AX5043_IRQMASK1 = 0x01; // enable xtal ready interrupt
    SpiReadSingleAddressRegister(dev,REG_AX5043_POWSTICKYSTAT);
}
void Normal_send(struct ax5043 *dev,uint8_t *Buf, uint8_t Length)
{
    dev->axradio_txbuffer_cnt = 0;
    transmit_packet_task(dev,Buf,Length);
}
void ChangeMaxPower(struct ax5043 *dev)
{
    if(dev->axradio_power_now==0)
    {
        dev->axradio_power_now = 1;
        SpiWriteLongAddressRegister(dev,REG_AX5043_TXPWRCOEFFB1, 0x0C);
        SpiWriteLongAddressRegister(dev,REG_AX5043_TXPWRCOEFFB0, 0x00);
        LOG_D("ChangeMaxPower Now\r\n");
    }
}
void BackNormalPower(struct ax5043 *dev)
{
    if(dev->axradio_power_now)
    {
        dev->axradio_power_now = 0;
        SpiWriteLongAddressRegister(dev,REG_AX5043_TXPWRCOEFFB1, 0x07);
        SpiWriteLongAddressRegister(dev,REG_AX5043_TXPWRCOEFFB0, 0x00);
        LOG_D("BackNormalPower Now\r\n");
    }
}
int16_t axradio_tunevoltage(struct ax5043 *dev)
{
    int16_t r = 0;
    uint8_t cnt = 64;
    do {
        SpiWriteLongAddressRegister(dev,REG_AX5043_GPADCCTRL, 0x84);
        do {} while (SpiReadLongAddressRegister(dev,REG_AX5043_GPADCCTRL) & 0x80);
    } while (--cnt);
    cnt = 32;
    do {
        SpiWriteLongAddressRegister(dev,REG_AX5043_GPADCCTRL, 0x84);
        do {} while (SpiReadLongAddressRegister(dev,REG_AX5043_GPADCCTRL) & 0x80);
        {
            int16_t x = SpiReadLongAddressRegister(dev,REG_AX5043_GPADC13VALUE1) & 0x03;
            x <<= 8;
            x |= SpiReadLongAddressRegister(dev,REG_AX5043_GPADC13VALUE0);
            r += x;
        }
    } while (--cnt);
    return r;
}
uint8_t axradio_adjustvcoi(struct ax5043 *dev,uint8_t rng)
{
    uint8_t offs;
    uint8_t bestrng;
    uint16_t bestval = (uint16_t)~0;
    rng &= 0x7F;
    bestrng = rng;
    for (offs = 0; offs != 16; ++offs) {
        uint16_t val;
        if (!((uint8_t)(rng + offs) & 0xC0)) {
            SpiWriteLongAddressRegister(dev,REG_AX5043_PLLVCOI, (0x80 | (rng + offs)));
            val = axradio_tunevoltage(dev);
            if (val < bestval) {
                bestval = val;
                bestrng = rng + offs;
            }
        }
        if (!offs)
            continue;
        if (!((uint8_t)(rng - offs) & 0xC0)) {
            SpiWriteLongAddressRegister(dev,REG_AX5043_PLLVCOI, (0x80 | (rng - offs)));
            val = axradio_tunevoltage(dev);
            if (val < bestval) {
                bestval = val;
                bestrng = rng - offs;
            }
        }
    }
    if (bestval <= 0x0010)
        return rng | 0x80;
    return bestrng | 0x80;
}
uint8_t axradio_calvcoi(struct ax5043 *dev)
{
    uint8_t i;
    uint8_t r = 0;
    uint16_t vmin = 0xffff;
    uint16_t vmax = 0x0000;
    for (i = 0x40; i != 0;) {
        uint16_t curtune;
        --i;
        SpiWriteLongAddressRegister(dev,REG_AX5043_PLLVCOI, (0x80 | i));
        SpiReadSingleAddressRegister(dev,REG_AX5043_PLLRANGINGA); // clear PLL lock loss
        curtune = axradio_tunevoltage(dev);
        SpiReadSingleAddressRegister(dev,REG_AX5043_PLLRANGINGA); // clear PLL lock loss
        if (curtune > vmax)
            vmax = curtune;
        if (curtune < vmin) {
            vmin = curtune;
            // check whether the PLL is locked
            if (!(0xC0 & (uint8_t)~(SpiReadSingleAddressRegister(dev,REG_AX5043_PLLRANGINGA))))
                r = i | 0x80;
        }
    }
    if (!(r & 0x80) || vmax >= 0xFF00 || vmin < 0x0100 || vmax - vmin < 0x4000)
        return 0;
    return r;
}
uint8_t rf_startup(struct ax5043 *dev)
{
    uint8_t i;
    Ax5043_OFF(dev);
    Ax5043_Reset(dev);
    InitAx5043REG(dev);
    Ax5043SetRegisters_TX(dev);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PLLLOOP, 0x09);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PLLCPI, 0x08);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, AX5043_PWRSTATE_XTAL_ON);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_MODULATION, 0x08);
    SpiWriteLongAddressRegister(dev,REG_AX5043_FSKDEV2, 0x00);
    SpiWriteLongAddressRegister(dev,REG_AX5043_FSKDEV1, 0x00);
    SpiWriteLongAddressRegister(dev,REG_AX5043_FSKDEV0, 0x00);
    RadioXtalON(dev);

    for (i = 0; i < CHANNEL_NUM ;++i)
    {
        uint32_t f = dev->config->axradio_phy_chanfreq[0];
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA0, f);
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA1, (f >> 8));
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA2, (f >> 16));
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA3, (f >> 24));
        dev->ubRFState = trxstate_pll_ranging;
        SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK1, 0x10);     // enable pll autoranging done interrupt
        {
            uint8_t r;
            if ( !(dev->config->axradio_phy_chanpllrnginit[0] & 0xF0) )
            {
                r = dev->config->axradio_phy_chanpllrnginit[0] | 0x10;
            }
            else
            {
                r = 0x18;
                if (i)
                {
                    r = dev->axradio_phy_chanpllrng[i - 1];
                    if ( r & 0x20)
                        r = 0x08;
                    r &= 0x0F;
                    r |= 0x10;
                }
            }
            SpiWriteSingleAddressRegister(dev,REG_AX5043_PLLRANGINGA, r);
        }
        do
        {
            rt_thread_mdelay(10);
        }
        while (dev->ubRFState == trxstate_pll_ranging);
        dev->ubRFState = trxstate_off;
        SpiWriteSingleAddressRegister(dev,REG_AX5043_IRQMASK1, 0x00);
        dev->axradio_phy_chanpllrng[i] = (uint8_t)SpiReadSingleAddressRegister(dev,REG_AX5043_PLLRANGINGA);
    }
    if (dev->config->axradio_phy_vcocalib) {
        Ax5043SetRegisters_TX(dev);
        SpiWriteSingleAddressRegister(dev,REG_AX5043_MODULATION, 0x08);
        SpiWriteLongAddressRegister(dev,REG_AX5043_FSKDEV2, 0x00);
        SpiWriteLongAddressRegister(dev,REG_AX5043_FSKDEV1, 0x00);
        SpiWriteLongAddressRegister(dev,REG_AX5043_FSKDEV0, 0x00);
        SpiWriteSingleAddressRegister(dev,REG_AX5043_PLLLOOP, (SpiReadSingleAddressRegister(dev,REG_AX5043_PLLLOOP) | 0x04));
        {
            uint8_t x = SpiReadLongAddressRegister(dev,REG_AX5043_0xF35);
            x |= 0x80;
            if (2 & (uint8_t)~x)
                ++x;
            SpiWriteLongAddressRegister(dev,REG_AX5043_0xF35, x);
        }
        SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, AX5043_PWRSTATE_SYNTH_TX);
        {
            uint8_t  vcoisave = SpiReadLongAddressRegister(dev,REG_AX5043_PLLVCOI);
            uint8_t j = 2;
            for (i = 0; i < CHANNEL_NUM; ++i) {
                dev->axradio_phy_chanvcoi[i] = 0;
                if (dev->axradio_phy_chanpllrng[i] & 0x20)
                    continue;
                SpiWriteSingleAddressRegister(dev,REG_AX5043_PLLRANGINGA, (dev->axradio_phy_chanpllrng[i] & 0x0F));
                {
                    uint32_t f = dev->config->axradio_phy_chanfreq[i];
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA0, f);
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA1, (f >> 8));
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA2, (f >> 16));
                    SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA3, (f >> 24));
                }
                do {
                    if (dev->config->axradio_phy_chanvcoiinit[0]) {
                        uint8_t x = dev->config->axradio_phy_chanvcoiinit[i];
                        if (!(dev->config->axradio_phy_chanpllrnginit[0] & 0xF0))
                            x += (dev->axradio_phy_chanpllrng[i] & 0x0F) - (dev->config->axradio_phy_chanpllrnginit[i] & 0x0F);
                        dev->axradio_phy_chanvcoi[i] = axradio_adjustvcoi(dev,x);
                    } else {
                        dev->axradio_phy_chanvcoi[i] = axradio_calvcoi(dev);
                    }
                } while (--j);
                j = 1;
            }
            SpiWriteLongAddressRegister(dev,REG_AX5043_PLLVCOI, vcoisave);
        }
    }
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);
    InitAx5043REG(dev);
    Ax5043SetRegisters_RX(dev);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PLLRANGINGA, (dev->axradio_phy_chanpllrng[0] & 0x0F));
    {
        uint32_t f = dev->config->axradio_phy_chanfreq[0];
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA0, f);
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA1, (f >> 8));
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA2, (f >> 16));
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA3, (f >> 24));
    }
    dev->ubRFState = trxstate_rx;
    if (dev->axradio_phy_chanpllrng[0] & 0x20)
        return AXRADIO_ERR_RANGING;
    return AXRADIO_ERR_NOERROR;
}
uint8_t rf_restart(struct ax5043 *dev)
{
    Ax5043_OFF(dev);
    Ax5043_Spi_Reset(dev);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PWRMODE, 0x00);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PINFUNCIRQ, 0x01);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PINFUNCIRQ, 0x00);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PINFUNCIRQ, 0x03);
    InitAx5043REG(dev);
    SpiWriteSingleAddressRegister(dev,REG_AX5043_PLLRANGINGA, (dev->axradio_phy_chanpllrng[0] & 0x0F));
    {
        uint32_t f = dev->config->axradio_phy_chanfreq[0];
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA0, f);
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA1, (f >> 8));
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA2, (f >> 16));
        SpiWriteSingleAddressRegister(dev,REG_AX5043_FREQA3, (f >> 24));
    }
    dev->ubRFState = trxstate_rx;
    return AXRADIO_ERR_NOERROR;
}
