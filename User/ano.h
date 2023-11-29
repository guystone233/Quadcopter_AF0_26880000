#ifndef __ANO_H__
#define __ANO_H__

#include "stm32f4xx.h"
#include "usart.h"

void FANO_Send_Data(uint8_t funcID, int8_t *data);
// void FANO_Send_MAG(int8_t *data);
// void FANO_Send_ACC_GRY(int8_t *data);
void Send_Quaternion_Data(uint8_t *ano_data);
// void Send_Euler_Data(uint8_t *ano_data_euler);

#define Frame_EulerAngle 0x03
#define Frame_Quaternion 0x04

#endif
