#include <rtthread.h>
#include <rtdevice.h>
#include "dog.h"

#define WDT_DEVICE_NAME    "wdt"

#define DBG_TAG "WatchDog"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static rt_device_t wdg_dev;

void FeedDog(void)
{
    rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
}
int wdt_sample(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t timeout = 5;
    char device_name[RT_NAME_MAX];

    rt_strncpy(device_name, WDT_DEVICE_NAME, RT_NAME_MAX);
    wdg_dev = rt_device_find(device_name);
    if (!wdg_dev)
    {
        LOG_D("find %s failed!\n", device_name);
        return RT_ERROR;
    }
    ret = rt_device_init(wdg_dev);
    if (ret != RT_EOK)
    {
        LOG_D("initialize %s failed!\n", device_name);
        return RT_ERROR;
    }
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
    if (ret != RT_EOK)
    {
        LOG_D("set %s timeout failed!\n", device_name);
        return RT_ERROR;
    }
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    if (ret != RT_EOK)
    {
        LOG_D("start %s failed!\n", device_name);
        return -RT_ERROR;
    }
    LOG_D("WatchDog Init Success\r\n");
    return ret;
}
