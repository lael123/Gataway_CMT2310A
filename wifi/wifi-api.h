/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-17     Rick       the first version
 */
#ifndef WIFI_WIFI_API_H_
#define WIFI_WIFI_API_H_

#include "rtthread.h"

void WariningUpload(uint32_t from_id,uint32_t device_id,uint8_t type,uint8_t value);
void CloseWarn_Main(uint32_t device_id);
void CloseWarn_Slave(uint32_t device_id);
void Slave_Heart(uint32_t device_id,uint8_t rssi);
void Slave_Linelost(uint32_t device_id,uint8_t data);
void Remote_Delete(uint32_t device_id);
void MotoUpload(uint32_t device_id,uint8_t state);
void DoorUpload(uint32_t device_id,uint8_t state);
void Device_Add2Flash(uint32_t device_id,uint32_t from_id);
void Device_Add2Wifi(uint32_t device_id,uint32_t from_id);
void Device_Add2Flash_Wifi(uint32_t device_id,uint32_t from_id);
void DeviceCheck(uint32_t device_id,uint32_t from_id);
void Local_Delete(uint32_t device_id);
void Main_Add_WiFi(uint32_t device_id);
void Upload_Main_ID(uint32_t device_id);
void Reset_Main_Warn(uint32_t device_id);
void Slave_Add_WiFi(uint32_t device_id);
void Upload_Slave_ID(uint32_t device_id,uint32_t from_id);
void Reset_Slave_Warn(uint32_t device_id);
void Door_Add_WiFi(uint32_t device_id);
void Upload_Door_ID(uint32_t device_id,uint32_t from_id);
void Device_Delete_WiFi(uint32_t device_id);
void Remote_Delay_WiFi(uint32_t device_id,uint8_t state);
void Door_Delay_WiFi(uint32_t main_id,uint32_t device_id,uint8_t state);
void Warning_WiFi(uint32_t device_id,uint8_t state);
void Moto_CloseRemote(uint32_t device_id);
void Moto_OpenRemote(uint32_t device_id);
void Delay_CloseRemote(uint32_t device_id);
void Delay_OpenRemote(uint32_t device_id);
void Main_Rssi_Report(uint32_t device_id,int rssi);
void Ack_Report(uint32_t device_id);
void Device_Up(uint32_t device_id);
void Device_Down(uint32_t device_id);
void Heart_Request(char *id_buf);
void Heart_Upload(uint32_t device_id,uint8_t heart);
void Remote_Device_Clear(void);
void Sync_Request(void);
void Remote_Sync(void);
void Remote_Device_Add(uint32_t device_id);
uint8_t Remote_Get_Key_Valid(uint32_t Device_ID);
void InitWarn_Main(uint32_t device_id);

#endif /* WIFI_WIFI_API_H_ */
