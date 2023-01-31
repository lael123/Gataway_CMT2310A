/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-22     Rick       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include "drv_spi.h"
#include <string.h>
#include "AX5043.h"
#include "Radio_Encoder.h"
#include "Radio_Common.h"
#include "led.h"
#include "flashwork.h"

#define DBG_TAG "RF_EN"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

rt_thread_t Radio_Queue433 = RT_NULL;

uint32_t Self_Id = 0;
uint32_t Self_Default_Id = 40000000;
uint8_t Self_Type = 0;

extern struct ax5043 rf_433;

typedef struct
{
    uint8_t NowNum;
    uint8_t TargetNum;
    uint8_t Type[50];
    uint32_t Taget_Id[50];
    uint32_t Device_Id[50];
    uint8_t Counter[50];
    uint8_t Command[50];
    uint8_t Data[50];
}Radio_Queue;

Radio_Queue Queue_433;

void Start_Learn(void)
{
    RadioSend(99999999,1,3,1);
}
void RadioSend(uint32_t Taget_Id,uint8_t counter,uint8_t Command,uint8_t Data)
{
    char *buf = rt_malloc(64);
    uint8_t check = 0;
    if(counter<255)counter++;
        else counter=0;
    sprintf(buf,"{%08ld,%08ld,%03d,%02d,%d}",Taget_Id,Self_Id,counter,Command,Data);
    for(uint8_t i = 0 ; i < 28 ; i ++)
    {
        check += buf[i];
    }
    buf[28] = ((check>>4) < 10)?  (check>>4) + '0' : (check>>4) - 10 + 'A';
    buf[29] = ((check&0xf) < 10)?  (check&0xf) + '0' : (check&0xf) - 10 + 'A';
    buf[30] = '\r';
    buf[31] = '\n';
    Normal_send(&rf_433,buf,32);
    rf_433_send_timer_start();
    rt_free(buf);
}
void GatewayDataEnqueue(uint32_t target_id,uint32_t device_id,uint8_t rssi,uint8_t control,uint8_t value)
{
    uint8_t NumTemp = Queue_433.TargetNum;
    if(NumTemp<30)
    {
        NumTemp ++;
        LOG_D("Queue Num Increase,Value is %d\r\n",NumTemp);
    }
    else
    {
        LOG_I("Queue is Full,Value is %d\r\n",NumTemp);
        return;
    }
    Queue_433.Type[NumTemp] = 1;
    Queue_433.Taget_Id[NumTemp] = target_id;
    Queue_433.Device_Id[NumTemp] = device_id;
    Queue_433.Counter[NumTemp] = rssi;
    Queue_433.Command[NumTemp] = control;
    Queue_433.Data[NumTemp] = value;
    Queue_433.TargetNum++;
    LOG_D("GatewayDataEnqueue Success\r\n");
}
void GatewayDataSend(uint32_t target_id,uint32_t device_id,uint8_t rssi,uint8_t control,uint8_t value)
{
    char *buf = rt_malloc(64);
    sprintf(buf,"G{%08ld,%08ld,%08ld,%03d,%03d,%02d}G",target_id,Self_Id,device_id,rssi,control,value);
    Normal_send(&rf_433,buf,41);
    rf_433_send_timer_start();
    rt_free(buf);
}
void RadioEnqueue(uint32_t Taget_Id,uint8_t counter,uint8_t Command,uint8_t Data)
{
    uint8_t NumTemp = Queue_433.TargetNum;
    if(NumTemp<30)
    {
        NumTemp ++;
        LOG_D("Queue Num Increase,Value is %d\r\n",NumTemp);
    }
    else
    {
        LOG_I("Queue is Full,Value is %d\r\n",NumTemp);
        return;
    }
    Queue_433.Type[NumTemp] = 0;
    Queue_433.Taget_Id[NumTemp] = Taget_Id;
    Queue_433.Counter[NumTemp] = counter;
    Queue_433.Command[NumTemp] = Command;
    Queue_433.Data[NumTemp] = Data;
    Queue_433.TargetNum++;
    LOG_D("RadioEnqueue Success\r\n");
}
void RadioDequeue(void *paramaeter)
{
    LOG_D("Queue Init Success\r\n");
    while(1)
    {
        if(Queue_433.NowNum == Queue_433.TargetNum)
        {
            Queue_433.NowNum = 0;
            Queue_433.TargetNum = 0;
        }
        else if(Queue_433.TargetNum>0 && Queue_433.TargetNum>Queue_433.NowNum)
        {
            Queue_433.NowNum++;
            switch(Queue_433.Type[Queue_433.NowNum])
            {
            case 0:
                rt_thread_mdelay(50);
                RadioSend(Queue_433.Taget_Id[Queue_433.NowNum],Queue_433.Counter[Queue_433.NowNum],Queue_433.Command[Queue_433.NowNum],Queue_433.Data[Queue_433.NowNum]);
                LOG_D("Normal Send With Now Num %d,Target Num is %d,Target_Id %ld,counter %d,command %d,data %d\r\n",Queue_433.NowNum,Queue_433.TargetNum,Queue_433.Taget_Id[Queue_433.NowNum],Queue_433.Counter[Queue_433.NowNum],Queue_433.Command[Queue_433.NowNum],Queue_433.Data[Queue_433.NowNum]);
                rt_thread_mdelay(100);
                break;
            case 1:
                rt_thread_mdelay(50);
                GatewayDataSend(Queue_433.Taget_Id[Queue_433.NowNum],Queue_433.Device_Id[Queue_433.NowNum],Queue_433.Counter[Queue_433.NowNum],Queue_433.Command[Queue_433.NowNum],Queue_433.Data[Queue_433.NowNum]);
                LOG_I("GatewaySend With Now Num %d,Target Num is %d,Type is %d,Target_Id %ld,Device_Id %ld,control %d,value %d\r\n",Queue_433.NowNum,Queue_433.TargetNum,Queue_433.Type[Queue_433.NowNum],Queue_433.Taget_Id[Queue_433.NowNum],Queue_433.Device_Id[Queue_433.NowNum],Queue_433.Command[Queue_433.NowNum],Queue_433.Data[Queue_433.NowNum]);
                rt_thread_mdelay(100);
                break;
            default:break;
            }
        }
        rt_thread_mdelay(50);
    }
}
void RadioID_Init(void)
{
    int *p;
    p=(int *)(0x0800FFF0);
    Self_Id = *p;
    if(Self_Id==0xFFFFFFFF || Self_Id==0)
    {
        Self_Id = Self_Default_Id;
    }
    if(Self_Id >= 46000001 && Self_Id <= 49999999)
    {
        Self_Type = 1;
    }
}
uint32_t RadioID_Read(void)
{
    return Self_Id;
}
uint8_t DeviceType_Read(void)
{
    return Self_Type;
}
void RadioDequeueTaskInit(void)
{
    Radio_Queue433 = rt_thread_create("Radio_Queue433", RadioDequeue, RT_NULL, 1024, 9, 10);
    if(Radio_Queue433)rt_thread_startup(Radio_Queue433);
}
