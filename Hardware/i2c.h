#ifndef I2C_H
#define I2C_H

#include "stm32f4xx.h"
#include "string.h"

#define Transmitter 0
#define Receiver 1

void I2C1_SendStart(FunctionalState NewState);
void I2C1_SendStop(FunctionalState NewState);
void I2C1_SendACK(FunctionalState NewState);

void I2C1_CheckBUSY(void);

uint8_t I2C1_ReceiveData(void);
void I2C1_Write7bitAddr(uint8_t addr, uint8_t Direction);
void I2C1_SendData(uint8_t addr);

void I2C1_Write_1Byte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t Data);
uint8_t I2C1_Read_1Byte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr);
uint16_t I2C1_Read_2Byte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr);
void I2C1_Read_multiByte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr, int8_t *data, uint8_t size);
void I2C1_Write_multiByte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr, int8_t *data, uint8_t size);

void EV5(void);
void EV6(char *str);
void EV7(void);
void EV8_2(void);

#endif
