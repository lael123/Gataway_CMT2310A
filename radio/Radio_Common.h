#ifndef _RADIO_COMMON_H_
#define _RADIO_COMMON_H_

#include "rtthread.h"
#include "radio_drv.h"

#define CHANNEL_NUM     1

void Ax5043_Reset(struct ax5043 *dev);
char SetChannel(struct ax5043 *dev,uint8_t ubNum);
void vcoi_rng_get(struct ax5043 *dev);
void InitAx5043REG(struct ax5043 *dev);
uint8_t Ax5043SetRegisters_TX(struct ax5043 *dev);
uint8_t Ax5043SetRegisters_RX(struct ax5043 *dev);
void RdioXtalON(struct ax5043 *dev);
void ChangeMaxPower(struct ax5043 *dev);
void BackNormalPower(struct ax5043 *dev);
void SetTransmitMode(struct ax5043 *dev);
void AX5043ReceiverON(struct ax5043 *dev);
void AX5043Receiver_Continuous(struct ax5043 *dev);
void SetReceiveMode(struct ax5043 *dev);
void ReceiveData(struct ax5043 *dev);
void Ax5043_OFF(struct ax5043 *dev);
void transmit_packet_task(struct ax5043 *dev,uint8_t *Buf, uint8_t Length);
uint8_t rf_startup(struct ax5043 *dev);
uint8_t rf_restart(struct ax5043 *dev);
void Normal_send(struct ax5043 *dev,uint8_t *Buf, uint8_t Length);
void Radio_Task_Init(void);
void TransmitData(struct ax5043 *dev);

#endif

