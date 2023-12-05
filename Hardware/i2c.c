#include "i2c.h"
/* @attention
* - I2C1_SendStart
* - I2C1_SendStop
* - I2C1_SendACK
* - I2C1_SendData
* - I2C1_ReceiveData
* - I2C1_Write7bitAddr
*/
void I2C1_SendStart(FunctionalState NewState)
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

void I2C1_SendStop(FunctionalState NewState)
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

void I2C1_SendData(uint8_t addr)
{
    I2C1->DR = addr;
}

uint8_t I2C1_ReceiveData(void)
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

/* @brief SR1:
 * 0SB,1ADDR,2BTF,3ADD10,
 * 4STOPF,6RxNE,7TxE,
 * 8BERR,9ARLO,10AF,11OVR,
 * 12PECERR,14TIMEOUT,15SMBALERT
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
 * 0MSL,1BUSY,2TRA,
 * 4GENCALL,5SMBDEFAULT,6SMBHOST,7DUALF,
 * 8-15PEC
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

/* @brief
 * - I2C1_Write_1Byte_Register
 * - I2C1_Read_1Byte_Register
 * - I2C1_Read_2Byte_Register
 * - I2C1_Read_multiByte_Register
 */

void I2C1_Write_1Byte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t Data)
{
    I2C1_CheckBUSY();

    I2C1_SendStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Transmitter);
    EV6("MASTER_TRANSMITTER_MODE");

    I2C1_SendData(RegisterAddr);
    EV8_2();

    I2C1_SendData(Data);
    EV8_2();

    I2C1_SendStop(ENABLE);
}

uint8_t I2C1_Read_1Byte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
    uint8_t data = 0;
    I2C1_CheckBUSY();

    I2C1_SendStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Transmitter);
    EV6("MASTER_TRANSMITTER_MODE");

    I2C1_SendData(RegisterAddr);
    EV8_2();

    I2C1_SendStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Receiver);
    EV6("MASTER_RECEIVER_MODE");

    I2C1_SendStop(DISABLE);
    I2C1_SendACK(DISABLE);
    EV7();

    data = I2C1_ReceiveData();

    I2C1_SendStop(ENABLE);
    I2C1_SendACK(ENABLE);
    return data;
}

uint16_t I2C1_Read_2Byte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
    uint16_t data = 0;
    uint8_t dataH, dataL = 0;

    I2C1_SendACK(ENABLE);
    I2C1_SendStop(DISABLE);

    I2C1_CheckBUSY();

    I2C1_SendStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Transmitter);
    EV6("MASTER_TRANSMITTER_MODE");

    I2C1_SendData(RegisterAddr);
    EV8_2();

    I2C1_SendStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Receiver);
    EV6("MASTER_RECEIVER_MODE");

    I2C1_SendStop(DISABLE);
    // I2C1_SendACK(DISABLE);
    EV7();

    dataH = I2C1_ReceiveData();

    I2C1_SendACK(DISABLE);
    I2C1_SendStop(ENABLE);
    EV7();

    dataL = I2C1_ReceiveData();

    I2C1_SendACK(DISABLE);
    I2C1_SendStop(ENABLE);
    I2C1_SendACK(ENABLE);

    data = (uint16_t)(dataH << 8) + dataL;
    return data;
}

void I2C1_Read_multiByte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr, int8_t *data, uint8_t size)
{
    // I2C1_SendACK(ENABLE);
    // I2C1_SendStop(DISABLE);

    I2C1_CheckBUSY();

    I2C1_SendStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Transmitter);
    EV6("MASTER_TRANSMITTER_MODE");

    I2C1_SendData(RegisterAddr);
    EV8_2();

    I2C1_SendStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Receiver);
    EV6("MASTER_RECEIVER_MODE");

    I2C1_SendStop(DISABLE);
    I2C1_SendACK(ENABLE);

    for (int i = 0; i < size; i++)
    {
        EV7();
        data[i] = (int8_t)I2C1_ReceiveData();
        
    }

    I2C1_SendStop(ENABLE);
    I2C1_SendACK(DISABLE);
}

void I2C1_Write_multiByte_Register(uint8_t DeviceAddr, uint8_t RegisterAddr, int8_t *data, uint8_t size)
{
    I2C1_CheckBUSY();

    I2C1_SendStart(ENABLE);
    EV5();

    I2C1_Write7bitAddr(DeviceAddr, Transmitter);
    EV6("MASTER_TRANSMITTER_MODE");

    I2C1_SendData(RegisterAddr);
    EV8_2();

    for (int i = 0; i < size; i++)
    {
        I2C1_SendData(data[i]);
        EV8_2();
    }

    I2C1_SendStop(ENABLE);
}

/* @brief
 * - I2C1_CheckBUSY: BUSY flag
 * - EV5: BUSY, MSL and SB flag
 * - EV6:
 *   - MASTER_TRANSMITTER_MODE -> BUSY, MSL, ADDR, TXE and TRA flags
 *   - MASTER_RECEIVER_MODE -> BUSY, MSL and ADDR flags
 * - EV7: BUSY, MSL and RXNE flags
 * - EV8_2: TRA, BUSY, MSL, TXE and BTF flags
 */
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
