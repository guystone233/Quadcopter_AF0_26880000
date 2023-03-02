#ifndef __MPU6050_H
#define __MPU6050_H

void MPU_Init(void);
uint16_t MPU_dataout(uint8_t readaddr1,uint8_t readaddr2);
void MPU_Init2(void);
#endif
