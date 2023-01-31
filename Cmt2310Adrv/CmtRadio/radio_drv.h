/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-29     Tobby       the first version
 */
#ifndef RADIOCMT2310_RADIOCMT2310_DRV_H_
#define RADIOCMT2310_RADIOCMT2310_DRV_H_
#include <stdint.h>

struct cmt2310
{
    struct rt_spi_device *socket;
    char name[10];
    int ubRssi;
    uint8_t ubRFState;
    uint8_t TXBuff[64];
    uint8_t TxLen;
    uint8_t RXBuff[64];
    uint8_t RxLen;
    uint8_t axradio_power_now;
    uint32_t axradio_txbuffer_cnt;
};

struct rt_spi_device *rf_433_radio_spi_init(void);

#endif /* RADIOCMT2310_RADIOCMT2310_DRV_H_ */
