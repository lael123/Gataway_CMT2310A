#include <agile_led.h>
#include <stdlib.h>
#include "led.h"
#include "pin_config.h"
#include <agile_led.h>

static agile_led_t *RF_G = RT_NULL;
static agile_led_t *RF_R = RT_NULL;
static agile_led_t *WIFI_R = RT_NULL;
static agile_led_t *WIFI_B = RT_NULL;
static agile_led_t *beep = RT_NULL;
static agile_led_t *beep_k = RT_NULL;

#define DBG_TAG "LED"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

void led_Init(void)
{
    if(RF_G == RT_NULL)
    {
        RF_G = agile_led_create(LED1_PIN, PIN_LOW, "200,200", -1);
    }
    if(RF_R == RT_NULL)
    {
        RF_R = agile_led_create(LED2_PIN, PIN_LOW, "200,200", -1);
    }
    if(WIFI_R == RT_NULL)
    {
        WIFI_R = agile_led_create(LED3_PIN, PIN_LOW, "200,200", -1);
    }
    if(WIFI_B == RT_NULL)
    {
        WIFI_B = agile_led_create(LED4_PIN, PIN_LOW, "200,200", -1);
    }
    if(beep == RT_NULL)
    {
        beep = agile_led_create(BUZZER_PIN, PIN_HIGH, "200,200", -1);
        beep_k = agile_led_create(BUZZER_PIN, PIN_HIGH, "100,100", 1);
    }
}
void beep_start(uint8_t count)
{
    agile_led_stop(beep);
    agile_led_set_light_mode(beep, "200,200", count);
    agile_led_start(beep);
}
void beep_power(uint8_t count)
{
    agile_led_stop(beep);
    agile_led_set_light_mode(beep, "400,400", count);
    agile_led_start(beep);
}
void beep_kick(void)
{
    agile_led_start(beep_k);
}
void wifi_led(uint8_t type)
{
    switch(type)
    {
    case 0://启动失败
        agile_led_stop(WIFI_R);
        agile_led_stop(WIFI_B);
        break;
    case 1://AP慢闪
        agile_led_stop(WIFI_B);
        agile_led_set_light_mode(WIFI_R, "1000,1000", -1);
        agile_led_start(WIFI_R);
        break;
    case 2://已配置未连接路由器
        agile_led_stop(WIFI_B);
        agile_led_set_light_mode(WIFI_R, "150,150", -1);
        agile_led_start(WIFI_R);
        break;
    case 3://已连接路由器未连接互联网
        agile_led_stop(WIFI_R);
        agile_led_set_light_mode(WIFI_B, "150,150", -1);
        agile_led_start(WIFI_B);
        break;
    case 4://已连接互联网
        agile_led_stop(WIFI_R);
        agile_led_set_light_mode(WIFI_B, "200,0", -1);
        agile_led_start(WIFI_B);
        break;
    default:
        break;
    }
}
void wifi_led_factory(uint8_t type)
{
    switch(type)
    {
    case 0://关闭全部
        agile_led_stop(WIFI_R);
        agile_led_stop(WIFI_B);
        break;
    case 1://wifi搜索失败
        agile_led_stop(WIFI_B);
        agile_led_set_light_mode(WIFI_R, "200,0", -1);
        agile_led_start(WIFI_R);
        break;
    case 2://RSSI异常
        agile_led_set_light_mode(WIFI_B, "200,200,0,200", -1);
        agile_led_start(WIFI_B);
        agile_led_set_light_mode(WIFI_R, "0,200,200,200", -1);
        agile_led_start(WIFI_R);
        break;
    case 3://RSSI正常
        agile_led_stop(WIFI_R);
        agile_led_set_light_mode(WIFI_B, "200,0", -1);
        agile_led_start(WIFI_B);
        break;
    default:
        break;
    }
}
void rf_led_resume(agile_led_t *led)
{
    agile_led_set_compelete_callback(RF_G,RT_NULL);
    agile_led_set_light_mode(RF_G, "200,0", -1);
    agile_led_start(RF_G);
}
void rf_led(uint8_t type)
{
    switch(type)
    {
    case 0://AX5043初始化失败
        agile_led_stop(RF_G);
        agile_led_set_light_mode(RF_R, "200,0", -1);
        agile_led_start(RF_R);
        break;
    case 1://AX5043初始化成功
        agile_led_stop(RF_R);
        agile_led_set_light_mode(RF_G, "200,0", -1);
        agile_led_start(RF_G);
        break;
    case 2://AX5043发送
        agile_led_stop(RF_R);
        agile_led_set_light_mode(RF_G, "10,100",1);
        agile_led_set_compelete_callback(RF_G,rf_led_resume);
        agile_led_start(RF_G);
        break;
    case 3://AX5043接收
        agile_led_stop(RF_R);
        agile_led_set_light_mode(RF_G, "10,100",1);
        agile_led_set_compelete_callback(RF_G,rf_led_resume);
        agile_led_start(RF_G);
        break;
    }
}
void rf_led_factory(uint8_t type)
{
    switch(type)
    {
    case 0://AX5043 产测无应答
        agile_led_stop(RF_G);
        agile_led_set_light_mode(RF_R, "200,0", -1);
        agile_led_start(RF_R);
        break;
    case 1://AX5043 RSSI过小
        agile_led_stop(RF_R);
        agile_led_set_light_mode(RF_G, "200,0", -1);
        agile_led_start(RF_G);
        break;
    case 2://AX5043 RSSI正常
        agile_led_stop(RF_R);
        agile_led_set_light_mode(RF_G, "10,100",1);
        agile_led_set_compelete_callback(RF_G,rf_led_resume);
        agile_led_start(RF_G);
        break;
    }
}
void learn_success(void)
{
    agile_led_set_light_mode(beep, "200,200", 5);
    agile_led_start(beep);
}
void learn_fail(void)
{
    LOG_W("Main is Full\r\n");
    agile_led_set_light_mode(beep, "50,50,200,200", 3);
    agile_led_start(beep);
}
void led_test(void)
{
    agile_led_set_light_mode(RF_R, "0,600,1200,600", 1);
    agile_led_set_light_mode(RF_G, "600,600,600,600", 1);
    agile_led_set_light_mode(WIFI_R, "0,600,1200,600", 1);
    agile_led_set_light_mode(WIFI_B, "600,600,600,600", 1);
    agile_led_start(RF_R);
    agile_led_start(RF_G);
    agile_led_start(WIFI_R);
    agile_led_start(WIFI_B);
}
void beep_test(void)
{
    agile_led_set_light_mode(beep, "200,400", 4);
    agile_led_start(beep);
}
