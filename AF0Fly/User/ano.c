#include "ano.h"
#include "stm32f4xx.h"                  // Device header
#include "arm_math.h"


/* The version of Ano Assistant is V7 */

/* Define the data frame */
char ANO_Send_Data[32] = {0xAA,0xFF};
uint8_t Data_len,sumcheck,addcheck,data_cnt;


void FANO_Send_Data(uint8_t funcID,int8_t *data)
{
    //发送数据第一位为帧头
    //发送数据第二位为目标地址，0xFF为默认
    //发送数据第三位为功能码
    //发送数据第四位为数据长度,需要放在data[0]中，data为需要发送的数组
    ANO_Send_Data[3] = data[0];
    ANO_Send_Data[2] = funcID;
    for(int8_t i = 0;i < ANO_Send_Data[3]; i++)
    {
        ANO_Send_Data[i + 4] = data[i + 1];
    }
    int8_t i;
    sumcheck = 0;addcheck = 0;
    for(i = 0;i < ANO_Send_Data[3] + 4; i++)
    {
        sumcheck += ANO_Send_Data[i];
        addcheck += sumcheck;
    }
    ANO_Send_Data[4+ANO_Send_Data[3]] = sumcheck;
    ANO_Send_Data[5+ANO_Send_Data[3]] = addcheck;
    for(i = 0;i < ANO_Send_Data[3] + 6; i++)
    {
        SendByte(ANO_Send_Data[i]);
    }
}

// /*trans mpu6050 data to quaternion */
// double raw,pitch,yaw,a_raw,a_pitch,g_raw,g_pitch,g_yaw;
// uint8_t drdt,dpdt,dydt;

// void cal_EularAngle(uint16_t *data,double *EulerAngle)
// {
//   raw =  atan(data[1]/data[2]);
//   pitch = -atan(data[0]/sqrt(data[1]*data[1]+data[2]*data[2]));

//    /*定义分别绕x,y,z轴转动的raw,pitch,yaw角速度矩阵*/
//     double dr_t[3],dp_t[3],dy_t[3];
    
//     dr_t = {drdt , 0  , 0 };
//     dp_t = {0  , dpdt , 0 };
//     dy_t = {0  , 0  , dydt};
    
//     /*在这里从IMU中获取的三个陀螺仪数据即通过z-y-x轴的旋转次序得到*/
//     /*即：[g_x,g_y,g_z]^T = M_x*M_y*dy_t +M _x*dp_t +dr_t */
//     /*由此可以解出drdt,dpdt,dydt分别为：*/
//     /* [dr/dt]  = {1, (sin(p)*sin(r))/cos(p), (cos(r)*sin(p))/cos(p) } */
//     /* [dp/dt]  = {0,       cos(r)          ,       -sin(r)          } */
//     /* [dy/dt]  = {0, (sin(r))/cos(p)       , (cos(r))/cos(p)        } */
//     double IMU_gyro[3] = {data[3], data[4], data[5]};
//     drdt = IMU_gyro[0]+((sin(pitch)*sin(raw))/cos(pitch))*IMU_gyro[1]+((cos(raw)*sin(pitch))/cos(pitch))*IMU_gyro[2];
//     dpdt = 0+cos(raw)*IMU_gyro[1]-sin(raw)*IMU_gyro[2];
//     dydt = 0+((sin(raw))/cos(pitch))*IMU_gyro[1]+((cos(raw))/cos(pitch))*IMU_gyro[2];
   
   

//   EulerAngle[0] = raw;
//   EulerAngle[1] = pitch;
//   EulerAngle[2] = yaw;
// }

// /*该函数用于将IMU得到的两组（一共6个）uint16_t格式的原始数据，转换为IMU坐标系下对应的四元数形式*/
// /*旋转顺序按照Z-Y-X顺序，推导原理如下：*/
// /*q_x = [cos(pitch/2),  sin(pitch/2)  ,         0    ,         0]*/
// /*q_y = [cos(roll/2) ,      0         , sin(roll/2)] ,         0]*/
// /*q_z = [cos(yaw/2)  ,      0         ,         0    ,sin(yaw/2)]*/
// /*q = q_z*q_y*q_x*/
// /*NOTE:这里用的"*"为四元数乘法符号*/
// void cal_quaternion(uint16_t *data,int16_t *Quaternion)
// {
//         int16_t q0,q1,q2,q3;
        
        
// }

// /*姿态解算*/
// void Attitude_Solve(uint16_t *data)
// {
        
//         float norm;
//         float v_x,v_y,v_z;
//         float a_x,a_y,a_z;
//         float e_x,e_y,e_z;
//         a_x = data[0];
//         a_y = data[1];
//         a_z = data[2];
//         /*计算加速度数据的模borm，并将数据单位化*/
//         norm = sqrt(a_x*a_x+a_y*a_y+a_z*a_z);
//         a_x = a_x/norm;
//         a_y = a_y/norm;
//         a_z = a_z/norm;
// }
// void trans(void)
// {
//     /*下面解释了如何将IMU获取到的陀螺仪和加速度计的三组数据转换为欧拉角和四元数*/
//     /*首先将陀螺仪中获取到的数据转换为IMU坐标系下对应的欧拉角矩阵*/
    
//     /*定义分别绕x,y,z三轴的旋转矩阵*/
//     uint8_t M_x[3][3],M_y[3][3],M_z[3][3];
    
//     M_x = {{1,       0,        0},
//             {0,cos(raw),-sin(raw)},
//             {0,sin(raw),cos(raw)}};
    
//     M_y = {{cos(pitch), 0, sin(pitch)},
//             {0,          1,         0},
//             {-sin(pitch),0,cos(pitch)}};
    
//     M_z = {{cos(yaw),-sin(yaw),0},
//             {sin(yaw), cos(yaw),0},
//             {0,           0,    1}};
    
//     /*定义分别绕x,y,z轴转动的raw,pitch,yaw角速度矩阵*/
//     double dr_t[3],dp_t[3],dy_t[3];
    
//     dr_t = {drdt , 0  , 0 };
//     dp_t = {0  , dpdt , 0 };
//     dy_t = {0  , 0  , dydt};
    
//     /*在这里从IMU中获取的三个陀螺仪数据即通过z-y-x轴的旋转次序得到*/
//     /*即：[g_x,g_y,g_z]^T = M_x*M_y*dy_t +M _x*dp_t +dr_t */
//     /*由此可以解出drdt,dpdt,dydt分别为：*/
//     /* [dr/dt]  = {1, (sin(p)*sin(r))/cos(p), (cos(r)*sin(p))/cos(p) } */
//     /* [dp/dt]  = {0,       cos(r)          ,       -sin(r)          } */
//     /* [dy/dt]  = {0, (sin(r))/cos(p)       , (cos(r))/cos(p)        } */
//     uint8_t IMU_gyro[3];
//     drdt = IMU_gyro[0]+((sin(pitch)*sin(raw))/cos(pitch))*IMU_gyro[1]+((cos(raw)*sin(pitch))/cos(pitch))*IMU_gyro[2];
//     dpdt = 0+cos(raw)*IMU_gyro[1]-sin(raw)*IMU_gyro[2];
//     dydt = 0+((sin(raw))/cos(pitch))*IMU_gyro[1]+((cos(raw))/cos(pitch))*IMU_gyro[2];
    
//     /*根据IMU_gyro的数据，将大地坐标系的角速度转换为IMU坐标系，这里定义为矩阵M_gyro*/
//     uint8_t M_gyro[3][3];
//     M_gyro = {{1,    0    ,    -sin(pitch)     },
//             {0, cos(raw), cos(pitch)*sin(raw) },
//             {0,-sin(raw), cos(pitch)*cos(raw)}};
    
//     /*对M_gyro求逆矩阵，得到IMU坐标系下的角速度转换为大地坐标系对应的角速度矩阵M_gyro_inv*/
//     uint8_t M_gyro_inv[3][3];
//     M_gyro_inv = {{1,   0    ,      sin(pitch)      },
//                 {0, cos(raw), -cos(pitch)*sin(raw) },
//                 {0, sin(raw), cos(pitch)*cos(raw) }};
    
//     /*再将IMU获取到的数据转换为IMU坐标系下对应的欧拉角矩阵*/
    
//     /*定义加速度计对应的旋转矩阵M_acc*/
//     /*其中M_acc=M_x*M_y*M_z*/
//     uint8_t M_acc[3][3];
//     M_acc = {{cos(pitch)*cos(yaw),                                   cos(pitch)*sin(yaw)                   , -sin(pitch)         },
//             {sin(raw)*sin(pitch)*cos(yaw)-cos(raw)*sin(yaw),sin(raw)*sin(pitch)*sin(yaw)+cos(raw)*cos(yaw), sin(raw)*cos(pitch) },
//             {cos(raw)*sin(pitch)*cos(yaw)+sin(raw)*sin(yaw),cos(raw)*sin(pitch)*sin(yaw)-sin(raw)*cos(yaw), cos(raw)*cos(pitch)}};
//     /*定义重力向量acc_g */
//     uint8_t acc_g[3];
//     uint8_t g = 1;
//     acc_g = {0, 0, g};
    
//     /*定义IMU坐标系下加速度对应的矩阵M_acc*/
//     uint8_t IMU_acc[3];
//     IMU_acc = {{M_acc[0][3]*acc_g[0]+M_acc[0][2]*acc_g[1]+M_acc[0][3]*acc_g[2]},
//                 {M_acc[1][3]*acc_g[0]+M_acc[1][2]*acc_g[1]+M_acc[1][3]*acc_g[2]},
//                 {M_acc[2][3]*acc_g[0]+M_acc[2][2]*acc_g[1]+M_acc[2][3]*acc_g[2]}};
// }
