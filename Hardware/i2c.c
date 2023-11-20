#include "i2c.h"

void I2C1_SetStart(FunctionalState NewState)
{
    if (NewState == ENABLE)
    {
        I2C1->CR1 |= 0x1 << 8;
    }
    else
    {
        I2C1->CR1 &= ~(0x1 << 8);
    }
}

void I2C1_SetStop(FunctionalState NewState)
{
    if (NewState == ENABLE)
    {
        I2C1->CR1 |= 0x1 << 9;
    }
    else
    {
        I2C1->CR1 &= ~(0x1 << 9);
    }
}

void I2C1_SendAddr(uint8_t addr)
{
    I2C1->DR = addr;
}

/* @brief SR1:
0SB,1ADDR,2BTF,3ADD10,
4STOPF,6RxNE,7TxE,
8BERR,9ARLO,10AF,11OVR,
12PECERR,14TIMEOUT,15SMBALERT
*/
uint32_t SB_Flag(uint32_t reg_data[2]) // start bit flag 0
{
    return (reg_data[0] >> 0) & 1;
}

uint32_t ADDR_Flag(uint32_t reg_data[2]) // address flag 1
{
    return (reg_data[0] >> 1) & 1;
}

uint32_t BTF_Flag(uint32_t reg_data[2]) // byte transfer finished flag 2
{
    return (reg_data[0] >> 2) & 1;
}

uint32_t RxNE_Flag(uint32_t reg_data[2]) // receive data register not empty flag 6
{
    return (reg_data[0] >> 6) & 1;
}

uint32_t TxE_Flag(uint32_t reg_data[2]) // transmit data register empty flag 7
{
    return (reg_data[0] >> 7) & 1;
}

uint32_t ARLO_Flag(uint32_t reg_data[2]) // arbitration lost flag 9
{
    return (reg_data[0] >> 9) & 1;
}

/* @brief SR2:
0MSL,1BUSY,2TRA,
4GENCALL,5SMBDEFAULT,6SMBHOST,7DUALF,
8-15PEC
*/
uint32_t MSL_Flag(uint32_t reg_data[2]) // master or slave flag 0
{
    return (reg_data[1] >> 0) & 1;
}

uint32_t BUSY_Flag(uint32_t reg_data[2]) // busy flag 1
{
    return (reg_data[1] >> 1) & 1;
}

uint32_t TRA_Flag(uint32_t reg_data[2]) // transmit or receive flag 2
{
    return (reg_data[1] >> 2) & 1;
}

uint8_t I2C1_ReceiveByte(void)
{
    return (uint8_t)I2C1->DR;
}

void I2C1_Write7bitAddr(uint8_t addr, uint8_t Direction)
{
    if (Direction == Receiver)
    {
        addr |= ((uint16_t)0x0001);
    }
    else
    {
        addr &= (uint8_t) ~((uint8_t)I2C_OAR1_ADD0);
    }
    I2C1->DR = addr;
}

void I2C1_Write_7bitmode_Register(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t Data)
{
    I2C1_CheckBUSY();

    I2C1_SetStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Transmitter);
    EV6("MASTER_TRANSMITTER_MODE");

    I2C1_SendAddr(RegisterAddr);
    EV8_2();

    I2C1_SendAddr(Data);
    EV8_2();

    I2C1_SetStop(ENABLE);
}

void I2C1_CheckBUSY(void)
{
    uint32_t reg_data[2];
    reg_data[0] = 0;
    reg_data[1] = I2C1->SR2;
    while (BUSY_Flag(reg_data))
    {
        reg_data[0] = 0;
        reg_data[1] = I2C1->SR2;
    };
}

uint8_t I2C1_Read_7bitmode_Register(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
    uint8_t data = 0;
    I2C1_CheckBUSY();

    I2C1_SetStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Transmitter);
    EV6("MASTER_TRANSMITTER_MODE");

    I2C1_SendAddr(RegisterAddr);
    EV8_2();

    I2C1_SetStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Receiver);
    EV6("MASTER_RECEIVER_MODE");

    I2C1_SetStop(DISABLE);
    I2C1_SendACK(DISABLE);
    EV7();

    data = I2C1_ReceiveByte();

    I2C1_SetStop(ENABLE);
    I2C1_SendACK(ENABLE);
    return data;
}

uint16_t I2C1_Read_2Byte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
    uint16_t data = 0;
    uint8_t dataH, dataL = 0;

		I2C1_SendACK(ENABLE);
		I2C1_SetStop(DISABLE);
	
    I2C1_CheckBUSY();

    I2C1_SetStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Transmitter);
    EV6("MASTER_TRANSMITTER_MODE");

    I2C1_SendAddr(RegisterAddr);
    EV8_2();

    I2C1_SetStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Receiver);
    EV6("MASTER_RECEIVER_MODE");

    I2C1_SetStop(DISABLE);
    // I2C1_SendACK(DISABLE);
    EV7();

    dataH = I2C1_ReceiveByte();

    I2C1_SendACK(DISABLE);
    I2C1_SetStop(ENABLE);
    EV7();

    dataL = I2C1_ReceiveByte();

    I2C1_SendACK(DISABLE);
    I2C1_SetStop(ENABLE);
    I2C1_SendACK(ENABLE);

    data = (uint16_t)(dataH << 8) + dataL;
    return data;
}

void I2C1_SendACK(FunctionalState NewState)
{
    if (NewState == ENABLE)
    {
        I2C1->CR1 |= 0x1 << 10;
    }
    else
    {
        I2C1->CR1 &= ~(0x1 << 10);
    }
}

void EV5(void)
{
    /* BUSY, MSL and SB flag */
    uint32_t reg_data[2];
    reg_data[0] = I2C1->SR1;
    reg_data[1] = I2C1->SR2;
    while (!(
        (BUSY_Flag(reg_data)) && (MSL_Flag(reg_data)) && (SB_Flag(reg_data))))
    {
        reg_data[0] = I2C1->SR1;
        reg_data[1] = I2C1->SR2;
    };
}

void EV6(char *str)
{
    uint32_t reg_data[2];
    reg_data[0] = I2C1->SR1;
    reg_data[1] = I2C1->SR2;
    if (strcmp(str, "MASTER_TRANSMITTER_MODE") == 0)
    {
        /* BUSY, MSL, ADDR, TXE and TRA flags */
        while (!(
            (BUSY_Flag(reg_data)) && (MSL_Flag(reg_data)) && (ADDR_Flag(reg_data)) && (TxE_Flag(reg_data)) && (TRA_Flag(reg_data))))
        {
            reg_data[0] = I2C1->SR1;
            reg_data[1] = I2C1->SR2;
        };
        if (ARLO_Flag(reg_data))
        {
            I2C1->SR1 |= 0x400;
        }
    }
    else if (strcmp(str, "MASTER_RECEIVER_MODE") == 0)
    {
        /* BUSY, MSL and ADDR flags */
        while (!(
            (BUSY_Flag(reg_data)) && (MSL_Flag(reg_data)) && (ADDR_Flag(reg_data))))
        {
            reg_data[0] = I2C1->SR1;
            reg_data[1] = I2C1->SR2;
        };
    }
}

void EV7(void)
{
    /* BUSY, MSL and RXNE flags */
    uint32_t reg_data[2];
    reg_data[0] = I2C1->SR1;
    reg_data[1] = I2C1->SR2;
    while (!(
        (BUSY_Flag(reg_data)) && (MSL_Flag(reg_data)) && (RxNE_Flag(reg_data))))
    {
        reg_data[0] = I2C1->SR1;
        reg_data[1] = I2C1->SR2;
    };
}

void EV8_2(void)
{
    /* TRA, BUSY, MSL, TXE and BTF flags */
    uint32_t reg_data[2];
    reg_data[0] = I2C1->SR1;
    reg_data[1] = I2C1->SR2;
    while (!(
        (TRA_Flag(reg_data)) && (BUSY_Flag(reg_data)) && (MSL_Flag(reg_data)) && (TxE_Flag(reg_data)) && (BTF_Flag(reg_data))))
    {
        reg_data[0] = I2C1->SR1;
        reg_data[1] = I2C1->SR2;
    };
}
