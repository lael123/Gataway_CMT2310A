/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-24     Tobby       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_spi.h"
#include <string.h>
#include "pin_config.h"
#include "base_types.h"
#include "cmt2310a_Reg.h"
#include "cmt2310a_def.h"
#include <stdio.h>

#define DBG_TAG "crc16"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/*
 **************************************************************************************************************
 * 函数名称:     InvertUint8
 * 函数功能:     将uint8类型的数字进行bit全部反转
 * 函数入口:     dBuf    [out]   待反转的值
 srcBuf  [in]    反转后的值
 * 函数返回:        计算CRC的结果
 ***************************************************************************************************************/

static void InvertUint8(unsigned char *dBuf, unsigned char *srcBuf)
{
    int i;
    unsigned char tmp[4];
    tmp[0] = 0;
    for (i = 0; i < 8; i++)
    {
        if (srcBuf[0] & (1 << i))
            tmp[0] |= 1 << (7 - i);
    }
    dBuf[0] = tmp[0];
}

/*
 **************************************************************************************************************
 * 函数名称:     InvertUint16
 * 函数功能:     将uint16类型的数字进行bit全部反转
 * 函数入口:     dBuf    [out]   待反转的值
 srcBuf  [in]    反转后的值
 * 函数返回:        计算CRC的结果
 ***************************************************************************************************************/
static void InvertUint16(unsigned short *dBuf, unsigned short *srcBuf)
{
    int i;
    unsigned short tmp[4];
    tmp[0] = 0;
    for (i = 0; i < 16; i++)
    {
        if (srcBuf[0] & (1 << i))
            tmp[0] |= 1 << (15 - i);
    }
    dBuf[0] = tmp[0];
}

/*
 **************************************************************************************************************
 * 函数名称:     CRC16
 * 函数功能:     计算CRC16
 * 函数入口:     puchMsg    [in]    待计算的数据
 usDataLen  [in]    待计算数据的长度
 * 函数返回:        计算CRC的结果
 ***************************************************************************************************************/
unsigned short CRC16(unsigned char *puchMsg, unsigned int usDataLen)
{
    unsigned short wCRCin = 0xFFFF;        //初始值，根据模式进行设置
    unsigned short wCPoly = 0x8005;        //多项式，根据模式进行设置
    unsigned char wChar = 0;

    while (usDataLen--)
    {
        wChar = *(puchMsg++);

        InvertUint8(&wChar, &wChar);     //输入数据反转，根据模式进行选择是否启用

        wCRCin ^= (wChar << 8);
        for (int i = 0; i < 8; i++)
        {
            if (wCRCin & 0x8000)
                wCRCin = (wCRCin << 1) ^ wCPoly;
            else
                wCRCin = wCRCin << 1;
        }
    }

    InvertUint16(&wCRCin, &wCRCin);        //输出数据反转，根据模式进行选择是否启用

    wCRCin &= 0xFFFF;

    wCRCin ^= 0xFFFF;                    //输出异或，根据模式设置异或的数值

    return (wCRCin);
}


uint16_t crc16_test(int argc, char *argv[])
{
    uint16_t out_data;
    out_data = CRC16(argv, argc);
    return out_data;
}


uint16_t two_char_to_int(unsigned char a, unsigned char b)
{
    uint16_t ret_val = 0;
    ret_val |=  b;
    ret_val <<= 8;
    ret_val |=  a;
    return ret_val;
}



