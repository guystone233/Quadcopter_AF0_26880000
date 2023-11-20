#include "ano.h"

/* The version of Ano Assistant is V7 */

/* Define the data frame */
int8_t ANO_Send_Data[32] = {0xAA, 0xFF};
uint8_t Data_len, sumcheck, addcheck, data_cnt;

void FANO_Send_Data(uint8_t funcID, int8_t *data)
{
    // 发送数据第一位为帧头
    // 发送数据第二位为目标地址，0xFF为默认
    // 发送数据第三位为功能码
    // 发送数据第四位为数据长度,需要放在data[0]中，data为需要发送的数组
    ANO_Send_Data[3] = data[0];
    ANO_Send_Data[2] = funcID;
    for (int8_t i = 0; i < ANO_Send_Data[3]; i++)
    {
        ANO_Send_Data[i + 4] = data[i + 1];
    }
    int8_t i;
    sumcheck = 0;
    addcheck = 0;
    for (i = 0; i < ANO_Send_Data[3] + 4; i++)
    {
        sumcheck += ANO_Send_Data[i];
        addcheck += sumcheck;
    }
    ANO_Send_Data[4 + ANO_Send_Data[3]] = sumcheck;
    ANO_Send_Data[5 + ANO_Send_Data[3]] = addcheck;
    for (i = 0; i < ANO_Send_Data[3] + 6; i++)
    {
        SendByte(ANO_Send_Data[i]);
    }
}

void Send_Quaternion_Data(uint8_t *ano_data)
{
    FANO_Send_Data(Frame_Quaternion, (uint8_t *)ano_data);
}

void Send_Euler_Data(uint8_t *ano_data_euler)
{
    FANO_Send_Data(Frame_EulerAngle, (uint8_t *)ano_data_euler);
}
