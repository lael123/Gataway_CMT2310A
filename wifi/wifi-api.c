/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-17     Rick       the first version
 */
#include "wifi.h"
#include "wifi-api.h"
#include "flashwork.h"
#include "stdio.h"
#include "wifi-service.h"
#include "radio_encoder.h"

#define DBG_TAG "WIFI-API"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

char *door_pid = {"emnzq3qxwfplx7db"};
char *slave_pid = {"lnbkva6cip8dw7vy"};
char *main_pid = {"q3xnn9yqt55ifaxm"};

Remote_Info Remote_Device={0};
extern Device_Info Global_Device;

uint8_t Sync_Counter = 1;
rt_timer_t Sync_Request_t = RT_NULL;
rt_timer_t Sync_Timeout_t = RT_NULL;

void Secure_Sync(void)//C1 00 01
{
    if(DeviceType_Read())
    {
        unsigned short length = 0;
        length = set_wifi_uart_byte(length, ALARM_STATE_SET_SUBCMD); //写入子命令0x00
        length = set_wifi_uart_byte(length, 1);
        wifi_uart_write_frame(SECURITY_PROTECT_ALARM_CMD, MCU_TX_VER, length);
    }
}
void WariningUpload(uint32_t from_id,uint32_t device_id,uint8_t type,uint8_t value)
{
    unsigned char *device_id_buf = rt_malloc(20);
    unsigned char *from_id_buf = rt_malloc(20);
    sprintf(device_id_buf,"%ld",device_id);
    sprintf(from_id_buf,"%ld",from_id);
    if(device_id>0)
    {
        switch(type)
        {
            case 0://掉线
               mcu_dp_bool_update(103,value,device_id_buf,my_strlen(device_id_buf)); //BOOL型数据上报;
               break;
            case 1://漏水
                if(value)
                {
                    mcu_dp_bool_update(104,0,device_id_buf,my_strlen(device_id_buf)); //VALUE型数据上报;
                }
                mcu_dp_enum_update(1,value,device_id_buf,my_strlen(device_id_buf)); //BOOL型数据上报;
               break;
            case 2://电量
                mcu_dp_enum_update(102,value,device_id_buf,my_strlen(device_id_buf)); //BOOL型数据上报;
               break;
            case 3://掉落
                mcu_dp_bool_update(104,value,device_id_buf,my_strlen(device_id_buf)); //VALUE型数据上报;
               break;
        }
    }
    else
    {
        switch(type)
        {
            case 0://自检
                if(value == 0)
                {
                    mcu_dp_bool_update(DPID_VALVE1_CHECK_FAIL,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                    mcu_dp_bool_update(DPID_VALVE1_CHECK_SUCCESS,1,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                }
                else if(value == 1)
                {
                    mcu_dp_bool_update(DPID_VALVE2_CHECK_FAIL,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                    mcu_dp_bool_update(DPID_VALVE2_CHECK_SUCCESS,1,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                }
                else if(value == 2)
                {
                    mcu_dp_bool_update(DPID_TEMP_STATE,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                    mcu_dp_bool_update(DPID_LINE_STATE,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                    mcu_dp_bool_update(DPID_VALVE1_CHECK_FAIL,1,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                }
                else if(value == 3)
                {
                    mcu_dp_bool_update(DPID_TEMP_STATE,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                    mcu_dp_bool_update(DPID_LINE_STATE,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                    mcu_dp_bool_update(DPID_VALVE2_CHECK_FAIL,1,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                }
               break;
            case 1://漏水
                if(value)
                {
                    mcu_dp_bool_update(DPID_VALVE1_CHECK_FAIL,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                    mcu_dp_bool_update(DPID_VALVE2_CHECK_FAIL,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                    mcu_dp_bool_update(DPID_TEMP_STATE,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                    mcu_dp_bool_update(DPID_LINE_STATE,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                }
                mcu_dp_bool_update(DPID_DEVICE_ALARM,value,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
               break;
            case 2://掉落
                mcu_dp_bool_update(DPID_LINE_STATE,value,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
               break;
            case 3://NTC
                if(value)
                {
                    mcu_dp_bool_update(DPID_LINE_STATE,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
                }
                mcu_dp_bool_update(DPID_TEMP_STATE,value,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
               break;
        }
    }
	Secure_Sync();//C1 00 01
    rt_free(device_id_buf);
    rt_free(from_id_buf);
}
void CloseWarn_Main(uint32_t device_id)
{
    unsigned char *from_id_buf = rt_malloc(20);
    sprintf(from_id_buf,"%ld",device_id);
    mcu_dp_bool_update(DPID_DEVICE_ALARM,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
    mcu_dp_bool_update(DPID_DELAY_STATE,0,from_id_buf,my_strlen(from_id_buf)); //VALUE型数据上报;
    LOG_I("CloseWarn_Main ID is %ld\r\n",device_id);
    rt_free(from_id_buf);
}
void InitWarn_Main(uint32_t device_id)
{
    unsigned char *from_id_buf = rt_malloc(20);
    sprintf(from_id_buf,"%ld",device_id);
    mcu_dp_bool_update(DPID_SELF_ID,device_id,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
    mcu_dp_bool_update(DPID_DEVICE_ALARM,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
    mcu_dp_bool_update(DPID_VALVE1_CHECK_FAIL,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
    mcu_dp_bool_update(DPID_VALVE2_CHECK_FAIL,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
    mcu_dp_bool_update(DPID_TEMP_STATE,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
    mcu_dp_bool_update(DPID_LINE_STATE,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
    LOG_I("InitWarn_Main ID is %ld\r\n",device_id);
    rt_free(from_id_buf);
}
void CloseWarn_Slave(uint32_t device_id)
{
    unsigned char *from_id_buf = rt_malloc(20);
    sprintf(from_id_buf,"%ld",device_id);
    mcu_dp_enum_update(1,0,from_id_buf,my_strlen(from_id_buf)); //BOOL型数据上报;
    rt_free(from_id_buf);
}
void Remote_Delete(uint32_t device_id)
{
    unsigned char *id_buf = rt_malloc(20);
    sprintf(id_buf,"%ld",GetBindID(device_id));
    GatewayDataEnqueue(GetBindID(device_id),device_id,0,6,0);
    LOG_I("Remote_Delete Success %ld\r\n",device_id);
    Del_Device(device_id);
    if(device_id>=30000000)
    {
        mcu_dp_value_update(DPID_DOOR_ID,0,id_buf,my_strlen(id_buf)); //BOOL型数据上报;
    }
    rt_free(id_buf);
}
void Slave_Heart(uint32_t device_id,uint8_t rssi)
{
    Flash_Set_Rssi(device_id,rssi);
    char *Buf = rt_malloc(20);
    sprintf(Buf,"%ld",device_id);
    mcu_dp_enum_update(101,rssi,Buf,my_strlen(Buf)); //VALUE型数据上报;
    LOG_I("Slave_Heart Device ID is %ld,rssi level is %d\r\n",device_id,rssi);
    rt_free(Buf);
}
void MotoUpload(uint32_t device_id,uint8_t state)
{
    Remote_Delay_WiFi(device_id,0);
    Flash_Set_Moto(device_id,state);
    char *Buf = rt_malloc(20);
    LOG_I("MotoUpload State is %d,device_id is %ld\r\n",state,device_id);
    sprintf(Buf,"%ld",device_id);
    mcu_dp_bool_update(DPID_DEVICE_STATE,state,Buf,my_strlen(Buf)); //VALUE型数据上报;
    rt_free(Buf);
}
void DoorUpload(uint32_t device_id,uint8_t state)
{
    char *DeviceBuf = rt_malloc(20);
    LOG_I("DoorUpload %ld upload %d\r\n",device_id,state);
    sprintf(DeviceBuf,"%ld",device_id);
    mcu_dp_bool_update(104,state,DeviceBuf,my_strlen(DeviceBuf)); //VALUE型数据上报;
    rt_free(DeviceBuf);
}
void Device_Add2Flash_Wifi(uint32_t device_id,uint32_t from_id)
{
    if(device_id>=10000000 && device_id<20000000)
    {
        if(Flash_Get_Key_Valid(device_id) == RT_ERROR)
        {
            MainAdd_Flash(device_id);
        }
        if(Remote_Get_Key_Valid(device_id) == RT_ERROR)
        {
            Flash_Set_UploadFlag(device_id,0);
            Main_Add_WiFi(device_id);
        }
    }
    if(device_id>=20000000 && device_id<30000000)
    {
        if(Flash_Get_Key_Valid(device_id) == RT_ERROR)
        {
            SlaveAdd_Flash(device_id,from_id);
        }
        if(Remote_Get_Key_Valid(device_id) == RT_ERROR)
        {
            Flash_Set_UploadFlag(device_id,0);
            Slave_Add_WiFi(device_id);
        }
    }
    else if(device_id>=30000000 && device_id<40000000)
    {
        if(Flash_Get_Key_Valid(device_id) == RT_ERROR)
        {
            DoorAdd_Flash(device_id,from_id);
        }
        if(Remote_Get_Key_Valid(device_id) == RT_ERROR)
        {
            Flash_Set_UploadFlag(device_id,0);
            Door_Add_WiFi(device_id);
        }
    }
}
uint8_t Set_Slave_Heart(uint32_t Device_ID,uint8_t heart)//数据载入到内存中
{
    uint16_t num = Global_Device.Num;
    if(!num)
    {
        return 0;
    }
    while(num)
    {
        if(Global_Device.Bind_ID[num]==Device_ID)
        {
            if(heart){
                if(Global_Device.Heart[num])
                {
                    Device_Up(Global_Device.ID[num]);
                }
            }
            else {
                Device_Down(Global_Device.ID[num]);
            }
        }
        num--;
    }
    return 0;
}
void DeviceCheck(uint32_t device_id,uint32_t from_id)
{
    Flash_Set_Heart(device_id,1);
}
void Local_Delete(uint32_t device_id)
{
    char *Buf = rt_malloc(20);
    char *Mainbuf = rt_malloc(20);
    sprintf(Buf,"%ld",device_id);
    sprintf(Mainbuf,"%ld",GetBindID(device_id));
    local_subdev_del_cmd(Buf);
    if(device_id>=30000000)
    {
        mcu_dp_value_update(108,0,Mainbuf,my_strlen(Mainbuf)); //BOOL型数据上报;
    }
    rt_free(Buf);
    rt_free(Mainbuf);
}
void Main_Add_WiFi(uint32_t device_id)
{
    char *Buf = rt_malloc(20);
    sprintf(Buf,"%ld",device_id);
    local_add_subdev_limit(1,0,0x01);
    gateway_subdevice_add("1.0",main_pid,0,Buf,10,0);
    LOG_I("Main_Add_WiFi ID is %d\r\n",device_id);
    rt_free(Buf);
}
void Upload_Main_ID(uint32_t device_id)
{
    char *Buf = rt_malloc(20);
    sprintf(Buf,"%ld",device_id);
    mcu_dp_value_update(DPID_SELF_ID,device_id,Buf,my_strlen(Buf)); //BOOL型数据上报;
    mcu_dp_bool_update(DPID_DEVICE_STATE,Flash_Get_Moto(device_id),Buf,my_strlen(Buf)); //VALUE型数据上报;
    LOG_I("Upload_Main_ID is %d\r\n",device_id);
    rt_free(Buf);
}
void Reset_Main_Warn(uint32_t device_id)
{
    char *Buf = rt_malloc(20);
    sprintf(Buf,"%ld",device_id);
    mcu_dp_bool_update(DPID_DEVICE_ALARM,0,Buf,my_strlen(Buf)); //BOOL型数据上报;
    mcu_dp_bool_update(DPID_LINE_STATE,0,Buf,my_strlen(Buf)); //BOOL型数据上报;
    mcu_dp_bool_update(DPID_TEMP_STATE,0,Buf,my_strlen(Buf)); //BOOL型数据上报;
    LOG_I("Reset_Main_Warn ID is %d\r\n",device_id);
    rt_free(Buf);
}
void Slave_Add_WiFi(uint32_t device_id)
{
    char *Buf = rt_malloc(20);
    sprintf(Buf,"%ld",device_id);
    local_add_subdev_limit(1,0,0x01);
    gateway_subdevice_add("1.0",slave_pid,0,Buf,10,0);
    LOG_I("Slave_Add_by WiFi ID is %d\r\n",device_id);
    rt_free(Buf);
}
void Upload_Slave_ID(uint32_t device_id,uint32_t from_id)
{
    char *Buf = rt_malloc(20);
    sprintf(Buf,"%ld",device_id);
    mcu_dp_value_update(107,from_id,Buf,my_strlen(Buf)); //BOOL型数据上报;
    mcu_dp_enum_update(101,Flash_Get_Rssi(device_id),Buf,my_strlen(Buf)); //VALUE型数据上报;
    LOG_I("Upload_Slave_ID is %d\r\n",device_id);
    rt_free(Buf);
}
void Reset_Slave_Warn(uint32_t device_id)
{
    char *Buf = rt_malloc(20);
    sprintf(Buf,"%ld",device_id);
    mcu_dp_enum_update(1,0,Buf,my_strlen(Buf)); //BOOL型数据上报;
    mcu_dp_bool_update(104,0,Buf,my_strlen(Buf)); //VALUE型数据上报;
    LOG_I("Reset_Slave_Value ID is %d\r\n",device_id);
    rt_free(Buf);
}
void Door_Add_WiFi(uint32_t device_id)
{
    char *Doorbuf = rt_malloc(20);
    sprintf(Doorbuf,"%ld",device_id);
    local_add_subdev_limit(1,0,0x01);
    gateway_subdevice_add("1.0",door_pid,0,Doorbuf,10,0);
    LOG_I("Door_Add_WiFi ID is %d\r\n",device_id);
    rt_free(Doorbuf);
}
void Upload_Door_ID(uint32_t device_id,uint32_t from_id)
{
    char *Mainbuf = rt_malloc(20);
    char *Doorbuf = rt_malloc(20);
    sprintf(Mainbuf,"%ld",from_id);
    sprintf(Doorbuf,"%ld",device_id);
    mcu_dp_value_update(107,from_id,Doorbuf,my_strlen(Doorbuf)); //BOOL型数据上报;
    mcu_dp_value_update(108,device_id,Mainbuf,my_strlen(Mainbuf)); //BOOL型数据上报;
    mcu_dp_enum_update(101,Flash_Get_Rssi(device_id),Doorbuf,my_strlen(Doorbuf)); //VALUE型数据上报;
    LOG_I("Upload_Door_ID is %d\r\n",device_id);
    rt_free(Mainbuf);
    rt_free(Doorbuf);
}
void Remote_Delay_WiFi(uint32_t device_id,uint8_t state)
{
    char *Buf = rt_malloc(20);
    LOG_I("Remote_Delay_WiFi %d from %ld is upload\r\n",state,device_id);
    sprintf(Buf,"%ld",device_id);
    mcu_dp_bool_update(106,state,Buf,my_strlen(Buf)); //VALUE型数据上报;
    rt_free(Buf);
}
void Door_Delay_WiFi(uint32_t main_id,uint32_t device_id,uint8_t state)
{
    char *Main_Buf = rt_malloc(20);
    char *Device_Buf = rt_malloc(20);
    LOG_I("Door_Delay_WiFi %d from %ld is upload\r\n",state,device_id);
    sprintf(Main_Buf,"%ld",main_id);
    sprintf(Device_Buf,"%ld",device_id);
    mcu_dp_bool_update(105,state,Device_Buf,my_strlen(Device_Buf)); //VALUE型数据上报;
    mcu_dp_bool_update(106,state,Main_Buf,my_strlen(Main_Buf)); //VALUE型数据上报;
    rt_free(Device_Buf);
    rt_free(Main_Buf);
}
void Warning_WiFi(uint32_t device_id,uint8_t state)
{
    InitWarn_Main(device_id);
}
void Moto_CloseRemote(uint32_t device_id)
{
    LOG_I("Main %d Moto is Remote close\r\n",device_id);
    GatewayDataEnqueue(device_id,0,0,2,0);
}
void Moto_OpenRemote(uint32_t device_id)
{
    LOG_I("Main %d Moto is Remote Open\r\n",device_id);
    GatewayDataEnqueue(device_id,0,0,2,1);
}
void Delay_CloseRemote(uint32_t device_id)
{
    LOG_I("Main %d Delay is Remote close\r\n",device_id);
    GatewayDataEnqueue(device_id,0,0,1,0);
}
void Delay_OpenRemote(uint32_t device_id)
{
    LOG_I("Main %d Delay is Remote Open\r\n",device_id);
    GatewayDataEnqueue(device_id,0,0,1,1);
}
void Main_Rssi_Report(uint32_t device_id,int rssi)
{
    char *id_buf = rt_malloc(20);
    sprintf(id_buf,"%ld",device_id);
    if(rssi<-94)
    {
        mcu_dp_enum_update(DPID_SIGN_STATE,0,id_buf,my_strlen(id_buf));
        LOG_I("Main_Rssi_Report %d is upload,rssi is %d,level is low\r\n",device_id,rssi);
    }
    else if(rssi>=-94 && rssi<-78)
    {
        mcu_dp_enum_update(DPID_SIGN_STATE,1,id_buf,my_strlen(id_buf));
        LOG_I("Main_Rssi_Report %d is upload,rssi is %d,level is mid\r\n",device_id,rssi);
    }
    else {
        mcu_dp_enum_update(DPID_SIGN_STATE,2,id_buf,my_strlen(id_buf));
        LOG_I("Main_Rssi_Report %d is upload,rssi is %d,level is high\r\n",device_id,rssi);
    }
    rt_free(id_buf);
}
void Ack_Report(uint32_t device_id)
{
    GatewayDataEnqueue(device_id,0,0,5,1);
    LOG_I("Ack_Report %d is upload\r\n",device_id);
}
void Self_Bind_Upload(uint32_t device_id)
{
    if(Flash_Get_UploadFlag(device_id)==0)
    {
        Flash_Set_UploadFlag(device_id,1);
        if(device_id>=10000000 && device_id<20000000)
        {
            Upload_Main_ID(device_id);
            Upload_Door_ID(GetDoorID(device_id),device_id);
            Reset_Main_Warn(device_id);
        }
        else if(device_id>=20000000 && device_id<30000000)
        {
            Reset_Slave_Warn(device_id);
            Upload_Slave_ID(device_id,GetBindID(device_id));
        }
        else if(device_id>=30000000 && device_id<40000000)
        {
            Upload_Door_ID(device_id,GetBindID(device_id));
        }
    }
}
void Heart_Upload(uint32_t device_id,uint8_t heart)
{
    char *id = rt_malloc(20);
    sprintf(id,"%ld",device_id);
    heart_beat_report(id,0);
    Flash_Set_Heart(device_id,heart);
    rt_free(id);
}
void Device_Up(uint32_t device_id)
{
    char *id = rt_malloc(32);
    sprintf(id,"%ld",device_id);
    heart_beat_report(id,0);
    user_updata_subden_online_state(0,id,1,1);
    rt_free(id);
}
void Device_Down(uint32_t device_id)
{
    char *id = rt_malloc(32);
    sprintf(id,"%ld",device_id);
    user_updata_subden_online_state(0,id,1,0);
    rt_free(id);
}
void Heart_Request(char *id_buf)
{
    uint32_t id = 0;
    id = atol(id_buf);
    Self_Bind_Upload(id);
    if(id>=20000000 && id<40000000)//如果是子设备
    {
        if(Flash_Get_Heart(GetBindID(id)))//检测子设备所属主控是否在线
        {
            if(Flash_Get_Heart(id))
            {
                Device_Up(id);
            }
            else
            {
                Device_Down(id);
            }
        }
    }
    else
    {
        if(Flash_Get_Heart(id))
        {
            Device_Up(id);
        }
        else
        {
            Device_Down(id);
        }
    }
}
void Remote_Device_Add(uint32_t device_id)
{
    Remote_Device.ID[++Remote_Device.Num]=device_id;
    LOG_I("Remote_Device_Add ID is %ld,Num is %d",device_id,Remote_Device.Num);
}
void Remote_Device_Clear(void)
{
    LOG_D("Remote_Device_Clear\r\n");
    memset(&Remote_Device,0,sizeof(Remote_Device));
}
uint8_t Remote_Get_Key_Valid(uint32_t Device_ID)//查询内存中的ID
{
    uint16_t num = Remote_Device.Num;
    if(!num)return RT_ERROR;
    while(num)
    {
        if(Remote_Device.ID[num]==Device_ID)return RT_EOK;
        num--;
    }
    return RT_ERROR;
}
uint8_t Remote_Device_Delete(uint32_t Device_ID)//查询内存中的ID
{
    uint16_t num = Remote_Device.Num;
    if(!num)return RT_ERROR;
    while(num)
    {
        if(Remote_Device.ID[num]==Device_ID)
        {
            Remote_Device.ID[num] = 0;
            return RT_EOK;
        }
        num--;
    }
    return RT_ERROR;
}
rt_thread_t Sync_t = RT_NULL;
rt_sem_t Sync_Once_Sem = RT_NULL;
uint8_t Sync_Recv = 0;
uint8_t Sync_Start = 0;
uint8_t Get_Main_Valid(uint32_t device_id,uint32_t bind_id)
{
    if(device_id>0 && device_id<20000000 && bind_id==0)
    {
        return RT_EOK;
    }
    return RT_ERROR;
}
uint8_t Get_Next_Main(void)
{
    uint8_t Num;
    Num = Sync_Counter;
    while(Num--)
    {
        if(Get_Main_Valid(Global_Device.ID[Num],Global_Device.Bind_ID[Num])==RT_EOK)
        {
            Sync_Counter = Num;
            return RT_EOK;
        }
    }
    return RT_ERROR;
}
void Sync_Timeout_Callback(void *parameter)
{
    if(Get_Next_Main()==RT_EOK)
    {
        LOG_I("Sync is Success,Go Next one\r\n");
        rt_sem_release(Sync_Once_Sem);
    }
    else
    {
        Sync_Start = 0;
        LOG_I("Sync is Done\r\n");
    }
}
void Sync_t_Callback(void *parameter)
{
    while(1)
    {
        rt_sem_take(Sync_Once_Sem,RT_WAITING_FOREVER);
        Sync_Start = 1;
        rt_thread_mdelay(10000);
        if(Global_Device.ID[Sync_Counter]==0)continue;
        Device_Add2Flash_Wifi(Global_Device.ID[Sync_Counter],0);
        if(Global_Device.SyncRetry[Sync_Counter]<3)
        {
            Sync_Recv = 0;
            GatewayDataEnqueue(Global_Device.ID[Sync_Counter],0,0,4,0);
            LOG_I("Sync_Request %d is download\r\n",Global_Device.ID[Sync_Counter]);
            rt_thread_mdelay(10000);
            if(Sync_Recv==0)
            {
                Global_Device.SyncRetry[Sync_Counter]++;
                LOG_W("Sync_Request %d is fail,Retry num is %d\r\n",Global_Device.ID[Sync_Counter],Global_Device.SyncRetry[Sync_Counter]);
                rt_sem_release(Sync_Once_Sem);
            }
        }
        else {
            rt_timer_stop(Sync_Timeout_t);
            if(Get_Next_Main()==RT_EOK){//同步重试次数用尽切换至下一个设备
                rt_sem_release(Sync_Once_Sem);
            }
            else {//同步重试次数用尽且不存在下一个设备
                Global_Device.SyncRetry[Sync_Counter] = 0;
                LOG_I("Sync is Done\r\n");
            }
        }
    }
}
void Sync_Init(void)
{
    if(Sync_Once_Sem == RT_NULL)
    {
        Sync_Once_Sem = rt_sem_create("Sync_Once_Sem", 0, RT_IPC_FLAG_FIFO);
    }
    if(Sync_Timeout_t == RT_NULL)
    {
        Sync_Timeout_t = rt_timer_create("Sync_Timeout", Sync_Timeout_Callback, RT_NULL, 5000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    }
    if(Sync_t == RT_NULL)
    {
        Sync_t = rt_thread_create("sync", Sync_t_Callback, RT_NULL, 2048, 10, 10);
        rt_thread_startup(Sync_t);
    }
}
void Sync_Request(void)
{
    rt_timer_stop(Sync_Timeout_t);
    Sync_Counter = Global_Device.Num;
    Get_Next_Main();
    rt_sem_release(Sync_Once_Sem);
}
MSH_CMD_EXPORT(Sync_Request,Sync_Request);
void Sync_Refresh(void)
{
    if(Sync_Start)
    {
        Sync_Recv = 1;
        Global_Device.SyncRetry[Sync_Counter] = 0;
        rt_timer_start(Sync_Timeout_t);
        LOG_I("Sync_Refresh\r\n");
    }
}
MSH_CMD_EXPORT(mcu_start_wifitest,mcu_start_wifitest);
