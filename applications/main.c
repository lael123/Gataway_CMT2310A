/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-13     RT-Thread    first version
 */

#include <rtthread.h>
#include "flashwork.h"
#include "device.h"
#include "key.h"
#include "led.h"
#include "wifi-uart.h"
#include "heart.h"
#include "dog.h"
#include "board.h"
#include "protocol.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

int main(void)
{
    RadioID_Init();
    LOG_I("System Version:%s,Radio ID:%ld,Device Type:%d\r\n",MCU_VER,RadioID_Read(),DeviceType_Read());
    flash_Init();
    led_Init();
    LoadDevice2Memory();
    factory_detect();
    Key_Reponse_Init();
    button_Init();
    Sync_Init();
    rf_433_start();
    RadioDequeueTaskInit();
    Heart_Init();
    WiFi_Init();
    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
