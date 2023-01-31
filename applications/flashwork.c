/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-13     Rick       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <spi_flash.h>
#include <drv_spi.h>
#include <string.h>
#include <stdlib.h>
#include "pin_config.h"
#include "flashwork.h"
#include "fal.h"
#include "easyflash.h"
#include "wifi-api.h"

#define DBG_TAG "FLASH"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

rt_spi_flash_device_t fm25q16;
char read_value_temp[64]={0};
Device_Info Global_Device={0};

int flash_Init(void)
{
    rt_err_t status;
    extern rt_spi_flash_device_t rt_sfud_flash_probe(const char *spi_flash_dev_name, const char *spi_dev_name);
    rt_hw_spi_device_attach("spi1", "spi10", GPIOA, GPIO_PIN_4);
    fm25q16 = rt_sfud_flash_probe("norflash0", "spi10");
    if (RT_NULL == fm25q16)
    {
        LOG_E("sfud fail\r\n");
        return RT_ERROR;
    };
    status = fal_init();
    if (status == 0)
    {
        LOG_E("fal_init fail\r\n");
        return RT_ERROR;
    };
    status = easyflash_init();
    if (status != EF_NO_ERR)
    {
        LOG_E("easyflash_init fail\r\n");
        return RT_ERROR;
    };
    LOG_I("Storage Init Success\r\n");
    return RT_EOK;
}
uint32_t Flash_Get_Learn_Nums(void)
{
    uint8_t read_len = 0;
    uint32_t read_value = 0;
    char *keybuf="Learn_Nums";
    memset(read_value_temp,0,64);
    read_len = ef_get_env_blob(keybuf, read_value_temp, 64, NULL);
    if(read_len>0)
    {
        read_value = atol(read_value_temp);
    }
    else
    {
        read_value = 0;
    }
    LOG_D("Reading Key %s value %ld \r\n", keybuf, read_value);//输出
    return read_value;
}
uint32_t Flash_Get_Main_Nums(void)
{
    uint8_t read_len = 0;
    uint32_t read_value = 0;
    char *keybuf="Main_Nums";
    memset(read_value_temp,0,64);
    read_len = ef_get_env_blob(keybuf, read_value_temp, 64, NULL);
    if(read_len>0)
    {
        read_value = atol(read_value_temp);
    }
    else
    {
        read_value = 0;
    }
    LOG_D("Reading Key %s value %ld \r\n", keybuf, read_value);//输出
    return read_value;
}
void Flash_LearnNums_Change(uint32_t value)
{
    const char *keybuf="Learn_Nums";
    char *Temp_ValueBuf = rt_malloc(64);
    sprintf(Temp_ValueBuf, "%ld", value);
    ef_set_env(keybuf, Temp_ValueBuf);
    rt_free(Temp_ValueBuf);
    LOG_D("Writing %ld to key %s\r\n", value,keybuf);
}
void Flash_MainNums_Change(uint32_t value)
{
    const char *keybuf="Main_Nums";
    char *Temp_ValueBuf = rt_malloc(64);
    sprintf(Temp_ValueBuf, "%ld", value);
    ef_set_env(keybuf, Temp_ValueBuf);
    rt_free(Temp_ValueBuf);
    LOG_D("Writing %ld to key %s\r\n", value,keybuf);
}
uint8_t Get_LearnNums_Valid(void)
{
    uint16_t num = 1;
    while(num<MaxSupport)
    {
        if(Global_Device.ID[num] == 0)
        {
            if(num>Global_Device.Num)//新序列号
            {
                Global_Device.Num = num;
                Flash_LearnNums_Change(num);
            }
            return num;
        }
        num++;
    }
    return 0;
}
uint8_t Add_MainNums(void)
{
    uint16_t num = Global_Device.MainNum;
    if(num<MainSupport)
    {
        num++;
        Global_Device.MainNum = num;
        Flash_MainNums_Change(num);
        return num;
    }
    else
    {
        return 0;
    }
}
uint8_t Del_MainNums(void)
{
    uint16_t num = Global_Device.MainNum;
    if(num>0)
    {
        num--;
        Global_Device.MainNum = num;
        Flash_MainNums_Change(num);
        return num;
    }
    else
    {
        return 0;
    }
}
uint8_t Get_MainNums(void)
{
    if(Global_Device.MainNum<MainSupport)
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}
void Flash_ID_Change(uint32_t key,uint32_t value)
{
    char *Temp_KeyBuf = rt_malloc(64);
    sprintf(Temp_KeyBuf, "%ld", key);
    char *Temp_ValueBuf = rt_malloc(64);//申请临时buffer空间
    sprintf(Temp_ValueBuf, "%ld", value);
    ef_set_env(Temp_KeyBuf, Temp_ValueBuf);
    rt_free(Temp_KeyBuf);
    rt_free(Temp_ValueBuf);
    LOG_D("Writing %ld to key %s \r\n", value,Temp_KeyBuf);
}
void Flash_Type_Change(uint32_t Device_ID,uint32_t value)
{
    char *Temp_KeyBuf = rt_malloc(64);
    sprintf(Temp_KeyBuf, "type:%ld", Device_ID);
    char *Temp_ValueBuf = rt_malloc(64);//申请临时buffer空间
    sprintf(Temp_ValueBuf, "%ld", value);
    ef_set_env(Temp_KeyBuf, Temp_ValueBuf);
    rt_free(Temp_KeyBuf);
    rt_free(Temp_ValueBuf);
    LOG_D("Writing %ld to key %s \r\n", value,Temp_KeyBuf);
}
void Flash_Bind_Change(uint32_t Device_ID,uint32_t value)
{
    char *Temp_KeyBuf = rt_malloc(64);
    sprintf(Temp_KeyBuf, "bind:%ld", Device_ID);
    char *Temp_ValueBuf = rt_malloc(64);//申请临时buffer空间
    sprintf(Temp_ValueBuf, "%ld", value);
    ef_set_env(Temp_KeyBuf, Temp_ValueBuf);
    rt_free(Temp_KeyBuf);
    rt_free(Temp_ValueBuf);
    LOG_D("Writing %ld to key %s \r\n", value,Temp_KeyBuf);
}
void Flash_Heart_Change(uint32_t Device_ID,uint32_t value)
{
    char *Temp_KeyBuf = rt_malloc(64);
    sprintf(Temp_KeyBuf, "heart:%ld", Device_ID);
    char *Temp_ValueBuf = rt_malloc(64);//申请临时buffer空间
    sprintf(Temp_ValueBuf, "%ld", value);
    ef_set_env(Temp_KeyBuf, Temp_ValueBuf);
    rt_free(Temp_KeyBuf);
    rt_free(Temp_ValueBuf);
    LOG_D("Writing %ld to key %s \r\n", value,Temp_KeyBuf);
}
void Flash_UploadFlag_Change(uint32_t Device_ID,uint32_t value)
{
    char *Temp_KeyBuf = rt_malloc(64);
    sprintf(Temp_KeyBuf, "upload:%ld", Device_ID);
    char *Temp_ValueBuf = rt_malloc(64);//申请临时buffer空间
    sprintf(Temp_ValueBuf, "%ld", value);
    ef_set_env(Temp_KeyBuf, Temp_ValueBuf);
    rt_free(Temp_KeyBuf);
    rt_free(Temp_ValueBuf);
    LOG_D("Writing %ld to key %s \r\n", value,Temp_KeyBuf);
}
uint8_t MainAdd_Flash(uint32_t Device_ID)
{
    uint8_t num;
    num = Get_LearnNums_Valid();
    if(num == 0)
    {
        return RT_ERROR;
    }
    Global_Device.ID[num] = Device_ID;
    Global_Device.Bind_ID[num] = 0;
    Global_Device.Heart[num] = 1;
    Flash_ID_Change(num,Device_ID);
    Flash_Bind_Change(Device_ID,0);
    Flash_Heart_Change(Device_ID,1);
    Add_MainNums();
    return RT_EOK;
}
uint8_t SlaveAdd_Flash(uint32_t Device_ID,uint32_t Bind_ID)
{
    uint8_t num;
    num = Get_LearnNums_Valid();
    if(num == 0)
    {
        return RT_ERROR;
    }
    Global_Device.ID[num] = Device_ID;
    Global_Device.Bind_ID[num] = Bind_ID;
    Global_Device.Heart[num] = 1;
    Flash_ID_Change(num,Device_ID);
    Flash_Bind_Change(Device_ID,Bind_ID);
    Flash_Heart_Change(Device_ID,1);
    return RT_EOK;
}
uint8_t DoorAdd_Flash(uint32_t Device_ID,uint32_t Bind_ID)
{
    uint8_t num;
    num = Get_LearnNums_Valid();
    if(num == 0)
    {
        return RT_ERROR;
    }
    Global_Device.ID[num] = Device_ID;
    Global_Device.Bind_ID[num] = Bind_ID;
    Global_Device.Heart[num] = 1;
    Flash_ID_Change(num,Device_ID);
    Flash_Bind_Change(Device_ID,Bind_ID);
    Flash_Heart_Change(Device_ID,1);
    return RT_EOK;
}
uint32_t GetBindID(uint32_t Device_ID)
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return 0;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            return Global_Device.Bind_ID[num];
        }
        num--;
    }
    return 0;
}
uint32_t GetDoorID(uint32_t Main_ID)
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return 0;
    }
    while(num)
    {
        if(Global_Device.Bind_ID[num] == Main_ID && Global_Device.ID[num]>=30000000)
        {
            return Global_Device.ID[num];
        }
        num--;
    }
    return 0;
}
uint8_t Del_Device(uint32_t Device_ID)
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return RT_ERROR;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            if(Device_ID>=10000000 && Device_ID<20000000)
            {
                Del_MainNums();
            }
            LOG_I("ID %ld is delete\r\n",Global_Device.ID[num]);
            Remote_Device_Delete(Device_ID);
            Flash_ID_Change(num,0);
            Global_Device.ID[num] = 0;
            Flash_Bind_Change(Device_ID,0);
            Global_Device.Bind_ID[num] = 0;
            Flash_UploadFlag_Change(Device_ID,0);
            Global_Device.UploadFlag[num] = 0;
            break;
        }
        num--;
    }
    return RT_ERROR;
}
uint8_t Del_MainBind(uint32_t Device_ID)
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return RT_ERROR;
    }
    while(num)
    {
        if((Global_Device.Bind_ID[num]==Device_ID) || (Global_Device.ID[num]==Device_ID))
        {
            if(Device_ID>=10000000 && Device_ID<20000000)
            {
                Del_MainNums();
            }
            Local_Delete(Global_Device.ID[num]);
            LOG_I("ID %ld is delete\r\n",Global_Device.ID[num]);
            Flash_ID_Change(num,0);
            Global_Device.ID[num] = 0;
            Flash_Bind_Change(Device_ID,0);
            Global_Device.Bind_ID[num] = 0;
            Flash_UploadFlag_Change(Device_ID,0);
            Global_Device.UploadFlag[num] = 0;
        }
        num--;
    }
    return RT_ERROR;
}
uint8_t Flash_Get_Key_Valid(uint32_t Device_ID)//查询内存中的ID
{
    uint16_t num = Global_Device.Num;
    if(!num)return RT_ERROR;
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)return RT_EOK;
        num--;
    }
    return RT_ERROR;
}
uint32_t Flash_Get_Key_Value(uint8_t type,uint32_t key)
{
    uint8_t read_len = 0;
    uint32_t read_value = 0;
    char *keybuf = rt_malloc(64);
    switch(type)
    {
    case 0:
        sprintf(keybuf, "%ld", key);//将传入的数字转换成数组
        break;
    case 1:
        sprintf(keybuf, "type:%ld", key);//将传入的数字转换成数组
        break;
    case 2:
        sprintf(keybuf, "bind:%ld", key);//将传入的数字转换成数组
        break;
    case 3:
        sprintf(keybuf, "heart:%ld", key);//将传入的数字转换成数组
        break;
    case 4:
        sprintf(keybuf, "upload:%ld", key);//将传入的数字转换成数组
        break;
    }
    memset(read_value_temp,0,64);
    read_len = ef_get_env_blob(keybuf, read_value_temp, 64, NULL);
    if(read_len>0)
    {
        read_value = atol(read_value_temp);
    }
    else
    {
        read_value = 0;
    }
    rt_free(keybuf);//释放临时buffer对应内存空间
    //LOG_D("Reading Key %s value %ld \r\n", keybuf, read_value);//输出
    return read_value;
}
void LoadDevice2Memory(void)
{
    memset(&Global_Device,0,sizeof(Global_Device));
    Global_Device.Num = Flash_Get_Learn_Nums();
    Global_Device.MainNum = Flash_Get_Main_Nums();
    for(uint8_t i=1;i<=Global_Device.Num;i++)
    {
        Global_Device.ID[i] = Flash_Get_Key_Value(0,i);
        if(Global_Device.ID[i])
        {
            Global_Device.Bind_ID[i] = Flash_Get_Key_Value(2,Global_Device.ID[i]);
            Global_Device.Heart[i] = Flash_Get_Key_Value(3,Global_Device.ID[i]);
            Global_Device.UploadFlag[i] = Flash_Get_Key_Value(4,Global_Device.ID[i]);
        }
    }
    LOG_I("Nums is %d",Global_Device.Num);
}
uint8_t Flash_Get_Heart(uint32_t Device_ID)//数据载入到内存中
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return 0;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            return Global_Device.Heart[num];
        }
        num--;
    }
    return 0;
}
uint8_t Flash_Set_Heart(uint32_t Device_ID,uint8_t heart)//数据载入到内存中
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return RT_ERROR;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            if(heart)
            {
                Global_Device.HeartRecv[num] = 1;
                if(Global_Device.Heart[num]==0)
                {
                    Set_Slave_Heart(Device_ID,1);
                    Flash_Heart_Change(Device_ID,1);
                    Global_Device.Heart[num] = 1;
                    LOG_I("Main Plus %ld is online now\r\n",Device_ID);
                }
            }
            else
            {
                if(Global_Device.Heart[num])
                {
                    Set_Slave_Heart(Device_ID,0);
                    Flash_Heart_Change(Device_ID,0);
                    Global_Device.Heart[num] = 0;
                    LOG_W("Main Plus %ld is offline now\r\n",Device_ID);
                }
            }
            return RT_EOK;
        }
        num--;
    }
    return RT_ERROR;
}
uint8_t Flash_Get_UploadFlag(uint32_t Device_ID)//数据载入到内存中
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return 0;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            return Global_Device.UploadFlag[num];
        }
        num--;
    }
    return 0;
}
uint8_t Flash_Set_UploadFlag(uint32_t Device_ID,uint8_t Flag)//数据载入到内存中
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return RT_ERROR;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            Global_Device.UploadFlag[num] = Flag;
            Flash_UploadFlag_Change(Device_ID,Flag);
            return RT_EOK;
        }
        num--;
    }
    return RT_ERROR;
}
uint8_t Flash_Get_Moto(uint32_t Device_ID)//数据载入到内存中
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return 0;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            return Global_Device.Moto[num];
        }
        num--;
    }
    return 0;
}
uint8_t Flash_Set_Moto(uint32_t Device_ID,uint8_t Flag)//数据载入到内存中
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return RT_ERROR;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            Global_Device.Moto[num] = Flag;
            return RT_EOK;
        }
        num--;
    }
    return RT_ERROR;
}
uint8_t Flash_Get_Rssi(uint32_t Device_ID)//数据载入到内存中
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return 0;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            return Global_Device.Rssi[num];
        }
        num--;
    }
    return 0;
}
uint8_t Flash_Set_Rssi(uint32_t Device_ID,uint8_t Flag)//数据载入到内存中
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return RT_ERROR;
    }
    while(num)
    {
        if(Global_Device.ID[num]==Device_ID)
        {
            Global_Device.Rssi[num] = Flag;
            return RT_EOK;
        }
        num--;
    }
    return RT_ERROR;
}
