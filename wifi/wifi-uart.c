/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-21     Rick       the first version
 */
#include "rtthread.h"
#include "rtdevice.h"
#include "pin_config.h"
#include "wifi-uart.h"
#include "wifi.h"
#include "wifi-service.h"
#include "led.h"

#define DBG_TAG "wifi_uart"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* 用于接收消息的信号量 */
static struct rt_semaphore rx_sem;
static rt_device_t serial;
static rt_thread_t WiFi_Uart_Thread = RT_NULL;
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

#define WiFi_UART_NAME                   "uart1"

uint8_t wifi_status = 0xff;
uint8_t wifi_connected = 0;
uint8_t WiFi_FactoryFlag = 0;

static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    if (size > 0)
    {
        rt_sem_release(&rx_sem);
    }
    return RT_EOK;
}

static char uart_sample_get_char(void)
{
    char ch;

    while (rt_device_read(serial, 0, &ch, 1) == 0)
    {
        rt_sem_control(&rx_sem, RT_IPC_CMD_RESET, RT_NULL);
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
    }
    return ch;
}
/* 数据解析线程 */
void data_parsing(void)
{
    char ch;
    LOG_D("WiFi-Uart Init Success\r\n");

    while (1)
    {
        ch = uart_sample_get_char();
        uart_receive_input(ch);
    }
}
void WiFi_Byte_Send(uint8_t data)
{
    //LOG_RAW("Send %X\r\n",data);
    rt_device_write(serial,0,&data,1);
}
void wifi_uart_init(void)
{
    char uart_name[RT_NAME_MAX];

    rt_strncpy(uart_name, WiFi_UART_NAME, RT_NAME_MAX);

    /* 查找系统中的串口设备 */
    serial = rt_device_find(uart_name);
    if (!serial)
    {
        LOG_D("find %s failed!\n", uart_name);
    }

    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 串口参数配置 */
    config.baud_rate = BAUD_RATE_115200;        //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.parity    = PARITY_NONE;           //无奇偶校验位
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_rx_ind);

    /* 创建 serial 线程 */
    WiFi_Uart_Thread = rt_thread_create("serial", (void (*)(void *parameter))data_parsing, RT_NULL, 512, 7, 10);

    /* 创建成功则启动线程 */
    if (WiFi_Uart_Thread != RT_NULL)
    {
        rt_thread_startup(WiFi_Uart_Thread);
    }
}
void wifi_status_change(uint8_t result)
{
    if(WiFi_FactoryFlag)return;
    if(wifi_status != result)
    {
        wifi_led(result);
        LOG_I("wifi_status is 0x%02X change to 0x%02X\r\n",wifi_status,result);
        wifi_status = result;
        switch(result)
        {
        case 1:
            wifi_connected = 0;
            break;
        case 2:
            wifi_connected = 0;
            break;
        case 3:
            break;
        case 4:
            if(wifi_connected==0)
            {
                wifi_connected = 1;
                Remote_Device_Clear();
                qur_subdev_list();
            }
            break;
        }
    }
}
void wifi_power_on(void)
{
    rt_pin_mode(WIFI_EN,0);
    rt_pin_write(WIFI_EN,1);
}
MSH_CMD_EXPORT(wifi_power_on,wifi_power_on);
void wifi_power_off(void)
{
    rt_pin_mode(WIFI_EN,0);
    rt_pin_write(WIFI_EN,0);
}
MSH_CMD_EXPORT(wifi_power_off,wifi_power_off);
void WiFi_Init(void)
{
    wifi_power_off();
    wifi_protocol_init();
    wifi_uart_init();
    wifi_service_init();
}
void WiFi_FactoryInit(void)
{
    WiFi_FactoryFlag = 1;
    wifi_power_off();
    wifi_protocol_init();
    wifi_uart_init();
    wifi_service_init();
    wifi_test();
}
