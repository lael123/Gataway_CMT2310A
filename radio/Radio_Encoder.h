/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-22     Rick       the first version
 */
#ifndef RADIO_RADIO_ENCODER_H_
#define RADIO_RADIO_ENCODER_H_

void Check_Wor_Recv(uint32_t From_ID,uint8_t Command,uint8_t Data);
void Tx_Done_Callback(uint8_t *rx_buffer,uint8_t rx_len);
void RadioSend(uint32_t Taget_Id,uint8_t counter,uint8_t Command,uint8_t Data);
void WorSend(uint32_t Taget_Id,uint8_t counter,uint8_t Command,uint8_t Data);
void RadioEnqueue(uint32_t Taget_Id,uint8_t counter,uint8_t Command,uint8_t Data);
void GatewayDataEnqueue(uint32_t target_id,uint32_t device_id,uint8_t rssi,uint8_t control,uint8_t value);
void RadioDequeueTaskInit(void);
void Start_Learn(void);
void rf433_rx_callback(int rssi,uint8_t *buffer,uint8_t len);

#endif /* RADIO_RADIO_ENCODER_H_ */
