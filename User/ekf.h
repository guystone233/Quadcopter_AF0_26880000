#ifndef EKF_H
#define EKF_H

#include "arm_math.h"
#include <stdlib.h>
#include <string.h>
#include "gy86.h"
#include "os_cpu.h"
#include "usart.h"
#include "os_cfg.h"
#include "ano.h"
#include <math.h>
#include "tasks.h"

#define PI 3.1415927f
// #define G 9.7913f
#define G 1.0f
#define ACC_RATIO 32.0f / 65536.0f * G
#define GYRO_RATIO 4000.0f / 65536.0f / 180.0f * PI
// #define MAG_RATIO 1/1090.0f
#define MAG_RATIO 1.0f
extern int16_t ACC_OFFSET_X;
extern int16_t ACC_OFFSET_Y;
extern int16_t ACC_OFFSET_Z;
extern int16_t GYRO_OFFSET_X;
extern int16_t GYRO_OFFSET_Y;
extern int16_t GYRO_OFFSET_Y;
extern int16_t GYRO_OFFSET_Z;





typedef struct EKF_input
{
    arm_matrix_instance_f32 x_k_prev; // 先验状态估计
    arm_matrix_instance_f32 f;        // 状态转移函数
    arm_matrix_instance_f32 P_k_prev; // 先验状态协方差估计
    arm_matrix_instance_f32 Q;        // 先验过程噪声协方差估计
    arm_matrix_instance_f32 R;        // 观测噪声协方差矩阵R
    arm_matrix_instance_f32 Z_k;      // 观测量
    float32_t halfT;                  // 采样时间
    INT32U tick_k_minus;
} EKF_input;

typedef struct ANO_data
{
    int8_t len;
    int8_t data[9];
} ANO_data;
typedef struct ANO_data_euler
{
    int8_t len;
    int8_t data[7];
} ANO_data_euler;
typedef struct ANO_MPU_data
{
    int8_t len;
    int8_t data[13];
} ANO_MPU_data;

extern ANO_data_euler *ano_data_euler;
extern ANO_MPU_data *ano_mpu_data;
extern ANO_data *ano_data;

void ekf_init(void);
void ekf_calculate(void);

#endif // EKF_H