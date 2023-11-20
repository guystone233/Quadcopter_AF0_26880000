#include "gy86.h"

void GPIOB_I2C1_Init(uint8_t scl, uint8_t sda)
{
    /* ENABLE GPIOB & I2C1_clk */
    RCC->AHB1ENR |= 0x1 << 1; // gpio
    RCC->APB1ENR |= 1 << 21;  // iic
    /* Set pb8 & pb9 as af mode*/
    GPIOB->MODER |= 0x2 << 2 * scl;
    GPIOB->MODER |= 0x2 << 2 * sda;
    /* Set pb8 & pb9 as AF4 */
    GPIOB->AFR[1] |= 0x4 << 4 * 0;
    GPIOB->AFR[1] |= 0x4 << 4 * 1;
    /* Set pb8 & pb9 as up */
    GPIOB->PUPDR |= 0x1 << 2 * scl;
    GPIOB->PUPDR |= 0x1 << 2 * sda;
    /* Set pb8 & pb9 as high speed */
    GPIOB->OSPEEDR |= 0x1 << 2 * scl;
    GPIOB->OSPEEDR |= 0x1 << 2 * sda;
    /* Set pb8 & pb9 as open drain */
    GPIOB->OTYPER |= 0x1 << 1 * scl;
    GPIOB->OTYPER |= 0x1 << 1 * sda;
}

void MPU6050_Init(void)
{
    I2C1_Init();
    I2C1_Write_7bitmode_Register(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    I2C1_Write_7bitmode_Register(MPU6050_ADDR, MPU6050_PWR_MGMT_2, 0x00);
    /*DLPF_CFG=0*/
    I2C1_Write_7bitmode_Register(MPU6050_ADDR, MPU6050_CONFIG, 0x06);
    /*Sample rate is 1KHz now*/
    I2C1_Write_7bitmode_Register(MPU6050_ADDR, MPU6050_SMPRT_DIV, 0x07);
    /*Gyro Scale is from -2000°/s to +2000°/s*/
    I2C1_Write_7bitmode_Register(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x18);
    /*ACCEL Scale is from -16g to +16g*/
    I2C1_Write_7bitmode_Register(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x18);
    /*Disable FIFO*/
    I2C1_Write_7bitmode_Register(MPU6050_ADDR, MPU6050_FIFO_EN, 0x00);
    I2C1_Write_7bitmode_Register(MPU6050_ADDR, INT_PIN_CFG, 0x02);
    HMC5883Init();
}

void HMC5883Init(void)
{
    I2C1_Write_7bitmode_Register(AddressHMC5883, ConfigA, 0x19);      // Outout rate : 75Hz
    I2C1_Write_7bitmode_Register(AddressHMC5883, ConfigB, 0x20);      // Gain : 1.3Ga
    I2C1_Write_7bitmode_Register(AddressHMC5883, ModeRegister, 0x00); // Continuous-Measurement Mode
    // StatusRegister = 0x01 when data prepared
}

void I2C1_Init(void)
{
    /*Init pb8 & pb9 as af mode*/
    GPIOB_I2C1_Init(SCL, SDA);
    /*reset i2c1*/
    I2C1->CR1 |= 1 << 15;
    I2C1->CR1 &= ~(1 << 15);

    /*Init Hardware I2C11     */
    /*Set Clk as 42MHz      */
    I2C1->CR2 &= ~(0x3F);
    I2C1->CR2 |= 0x2A;

    I2C1->CR1 &= ~(0x1 << 0);

    I2C1->TRISE &= ~(0x3F);
    I2C1->TRISE |= 42 + 1;

    /*Set SCL Frequency as 100KHz */
    /*Fscl = CCR * (1/PCLK)       */
    I2C1->CCR &= ~(0xFFF);
    I2C1->CCR |= 1 << 15;
    I2C1->CCR |= 210;

    I2C1->CR1 |= 0x1 << 0;

    // clear ack, smbtype, smbus
    I2C1->CR1 &= ~(0x1 << 10);
    I2C1->CR1 &= ~(0x1 << 3);
    I2C1->CR1 &= ~(0x1 << 1);

    /*ENABLE ack return after receiving an whole byte*/
    I2C1->CR1 |= 0x1 << 10;

    /*ENABLE I2C11           */
    I2C1->CR1 |= 0x1 << 0;

    // ack
    I2C1->CR1 |= 0x1 << 10;
}

int16_t I2C1_GetMPU6050X(void)
{
    int16_t data = 0;
    data = (int16_t)(I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H) << 8) + I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_ACCEL_XOUT_L);
    return data;
}

int16_t I2C1_GetMPU6050Y(void)
{
    int16_t data = 0;
    data = (int16_t)(I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_ACCEL_YOUT_H) << 8) + I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_ACCEL_YOUT_L);
    return data;
}

int16_t I2C1_GetMPU6050Z(void)
{
    int16_t data = 0;
    data = (int16_t)(I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H) << 8) + I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_L);
    return data;
}

int16_t I2C1_GetGyroX(void)
{
    int16_t data = 0;
    data = (int16_t)(I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_GYRO_XOUT_H) << 8) + I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_GYRO_XOUT_L);
    return data;
}

int16_t I2C1_GetGyroY(void)
{
    int16_t data = 0;
    data = (int16_t)(I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_GYRO_YOUT_H) << 8) + I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_GYRO_YOUT_L);
    return data;
}

int16_t I2C1_GetGyroZ(void)
{
    int16_t data = 0;
    data = (int16_t)(I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H) << 8) + I2C1_Read_7bitmode_Register(MPU6050_ADDR, MPU6050_GYRO_ZOUT_L);
    return data;
}

int16_t I2C1_GetHMC5883X(void)
{
    int16_t data = 0;
    data = (int16_t)(I2C1_Read_7bitmode_Register(HMC5883_Addr, OutputXMSB) << 8) + I2C1_Read_7bitmode_Register(HMC5883_Addr, OutputXLSB);
    return data;
}

int16_t I2C1_GetHMC5883Y(void)
{
    int16_t data = 0;
    data = (int16_t)(I2C1_Read_7bitmode_Register(HMC5883_Addr, OutputYMSB) << 8) + I2C1_Read_7bitmode_Register(HMC5883_Addr, OutputYLSB);
    return data;
}

int16_t I2C1_GetHMC5883Z(void)
{
    int16_t data = 0;
    data = (int16_t)(I2C1_Read_7bitmode_Register(HMC5883_Addr, OutputZMSB) << 8) + I2C1_Read_7bitmode_Register(HMC5883_Addr, OutputZLSB);
    return data;
}
