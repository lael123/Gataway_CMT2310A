/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-27     Rick       the first version
 */
#include "rtthread.h"
#include "heart.h"
#include "flashwork.h"
#include "radio_encoder.h"
#include "wifi-api.h"

#define DBG_TAG "heart"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

rt_thread_t heart_t = RT_NULL;
extern Device_Info Global_Device;
extern struct ax5043 rf_433;

void heart_callback(void *parameter)
{
    uint8_t num;
    LOG_D("Heart Check Init Success\r\n");
    while(1)
    {
        rt_thread_mdelay(15*60000);//检测周期
        num = 1;
        while(num <= Global_Device.Num)
        {
            if(Global_Device.ID[num]!=0 && Global_Device.Bind_ID[num]==0)
            {
                Global_Device.HeartRecv[num] = 0;
                LOG_D("Start Heart With %d,Retry num is %d\r\n",Global_Device.ID[num],Global_Device.HeartRetry[num]);
                GatewayDataEnqueue(Global_Device.ID[num],0,0,3,0);//Send
                rt_thread_mdelay(2000);//心跳后等待周期
                if(Global_Device.HeartRecv[num])//RecvFlag
                {
                    Global_Device.HeartCount[num] = 0;
                    Global_Device.HeartRetry[num] = 0;
                    Heart_Upload(Global_Device.ID[num],1);
                    rt_thread_mdelay(2000);//设备与设备之间的间隔
                    num++;
                }
                else
                {
                    if(Global_Device.HeartRetry[num] < 6)
                    {
                        LOG_D("Rerty Again times %d\r\n",Global_Device.HeartRetry[num]++);
                    }
                    else
                    {
                        if(Global_Device.HeartCount[num] < 5)
                        {
                            Global_Device.HeartCount[num]++;
                            LOG_D("%d Rerty Stop,HeartCount is %d\r\n",Global_Device.ID[num],Global_Device.HeartCount[num]);
                        }
                        else
                        {
                            Global_Device.HeartCount[num] = 0;
                            Flash_Set_Heart(Global_Device.ID[num],0);
                            LOG_D("%d Offline\r\n",Global_Device.ID[num]);
                        }
                        Global_Device.HeartRetry[num] = 0;
                        num++;
                    }
                    rt_thread_mdelay(3000);//设备与设备之间的间隔
                }
            }
            else
            {
                num++;
            }
        }
    }
}
void Heart_Init(void)
{
    heart_t = rt_thread_create("heart", heart_callback, RT_NULL, 2048, 10, 10);
    if(heart_t!=RT_NULL)rt_thread_startup(heart_t);
}
