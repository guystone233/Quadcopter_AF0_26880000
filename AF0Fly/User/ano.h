#ifndef __ANO_H__
#define __ANO_H__
#include "stm32f4xx.h"
#include "usart.h"

#define Frame_EulerAngle 0x03
#define Frame_Quaternion 0x04

void FANO_Send_Data(uint8_t funcID,int8_t *data);
void cal_EularAngle(uint16_t *data,double *EulerAngle);
void trans(void);

#endif
