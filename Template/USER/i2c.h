#ifndef __I2C_H
#define __I2C_H

void I2C_Configuration(void);

void I2C_Send_Byte(uint8_t slavead,uint8_t writead,uint8_t data);
uint8_t I2C_Read_Byte(uint8_t slavead,uint8_t writead);
#endif
