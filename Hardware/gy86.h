#ifndef __GY86_H
#define __GY86_H

#include "stm32f4xx.h"
#include "i2c.h"

#define MPU6050_ADDR 0xD0
#define SCL 8
#define SDA 9

#define MPU6050_ADDRESS 0x68

#define MPU6050_AUX_VDDIO 0x01
#define MPU6050_SMPRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

/*Maybe will be used*/
#define MPU6050_USER_CTRL 0X6A
#define MPU6050_FF_THR 0x1D
#define MPU6050_FF_DUR 0x1E
#define MPU6050_FIFO_EN 0x23
#define MPU6050_FIFO_COUNT_H 0x72
#define MPU6050_FIFO_COUNT_L 0x73
#define MPU6050_FIFO_R_W 0x74
#define INT_PIN_CFG 0x37
/*^^^   Reserved    ^^^*/

/*Accelerometer Mesurements*/
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
/*Accelerometer Mesurements*/

/*Temperature Mesurements*/
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
/*Temperature Mesurements*/

/*Gyroscope Measurements*/
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
/*Gyroscope Measurements*/

/*Power Management*/
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
/*Power Management*/

/*Who Am I*/
#define MPU6050_WHO_AM_I 0x75
/*Who Am I*/

/* HMC5883 */
#define AddressHMC5883 (uint8_t)0x3C
#define HMC5883_Addr (uint8_t)0x3C
// 7bit:		0x1E:0001 1110
// 8bit:   read:0x3D:0011 1101 	    write:0x3C:0011 1100  direction receiver = 1
#define AddressHMC5883LWrite (uint8_t)0x3C
#define AddressHMC5883LRead (uint8_t)0x3D

#define ConfigA (uint8_t)0x00
#define ConfigB (uint8_t)0x01
#define ModeRegister (uint8_t)0x02

#define OutputXMSB (uint8_t)0x03
#define OutputXLSB (uint8_t)0x04
#define OutputYMSB (uint8_t)0x05
#define OutputYLSB (uint8_t)0x06
#define OutputZMSB (uint8_t)0x07
#define OutputZLSB (uint8_t)0x08
#define StatusRegister (uint8_t)0x09

void MPU6050_Init(void);
void HMC5883Init(void);

int16_t I2C1_GetMPU6050X(void);
int16_t I2C1_GetMPU6050Y(void);
int16_t I2C1_GetMPU6050Z(void);

int16_t I2C1_GetGyroX(void);
int16_t I2C1_GetGyroY(void);
int16_t I2C1_GetGyroZ(void);

int16_t I2C1_GetHMC5883X(void);
int16_t I2C1_GetHMC5883Y(void);
int16_t I2C1_GetHMC5883Z(void);

void GPIOB_I2C1_Init(uint8_t scl, uint8_t sda);
void I2C1_Init(void);

#endif /* GY86 */
