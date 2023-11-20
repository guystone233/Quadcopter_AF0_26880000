#include "ekf.h"

/*
X: 状态向量，X = [q0 q1 q2 q3 w1 w2 w3]' 四元数 + 角速度偏移 认为两次采样中角速度偏移不变，7*1
PHI: 状态转移矩阵，7*7
H: 观测矩阵，6*7
dH: 观测矩阵Jacobian Matrix，6*7
P: 状态协方差矩阵，7*7
Q: 过程噪声矩阵Q，7*7
R: 观测噪声矩阵R，6*6
Z_k: 观测向量，[ax ay az mx my mz]' , 6*1
K: kalman增益矩阵，7*6
*/
// #define SKIP_MAG_CALIBRATION
#define SendString_serial(x) SendString(x)
// #define SendString_serial(x) SendString("")
EKF_input *EKF_in;
ANO_data *ano_data;
ANO_data_euler *ano_data_euler;
ANO_MPU_data *ano_mpu_data;
float32_t x_prev_data[7] = {0.0f};
float32_t f_data[49] = {0.0f};
float32_t P_prev_data[49] = {0.0f};
float32_t Q_data[49] = {0.0f};
float32_t R_data[36] = {0.0f};
float32_t Z_k_data[6] = {0.0f};

int16_t ACC_OFFSET_X = 0;
int16_t ACC_OFFSET_Y = 0;
int16_t ACC_OFFSET_Z = 0;
int16_t GYRO_OFFSET_X = 0;
int16_t GYRO_OFFSET_Y = 0;
int16_t GYRO_OFFSET_Z = 0;

// void Send_Quaternion_Data()
// {
//     FANO_Send_Data(Frame_Quaternion, (uint8_t *)ano_data);
// }
// void Send_Euler_Data()
// {
//     FANO_Send_Data(Frame_EulerAngle, (uint8_t *)ano_data_euler);
// }
void quaternion_multiply(float32_t q0, float32_t q1, float32_t q2, float32_t q3, float32_t r0, float32_t r1, float32_t r2, float32_t r3, float32_t *result);
void inline quaternion_multiply(float32_t q0, float32_t q1, float32_t q2, float32_t q3, float32_t r0, float32_t r1, float32_t r2, float32_t r3, float32_t *result)
{
    result[0] = q0 * r0 - q1 * r1 - q2 * r2 - q3 * r3;
    result[1] = q0 * r1 + q1 * r0 + q2 * r3 - q3 * r2;
    result[2] = q0 * r2 - q1 * r3 + q2 * r0 + q3 * r1;
    result[3] = q0 * r3 + q1 * r2 - q2 * r1 + q3 * r0;
    return;
}

void print_martix(arm_matrix_instance_f32 *matrix, char *name)
{
    char out[100];
    sprintf(out, "%s: \r\n", name);
    SendString_serial(out);
    for (int i = 0; i < matrix->numRows; i++)
    {
        for (int j = 0; j < matrix->numCols; j++)
        {
            sprintf(out, "%f, ", matrix->pData[i * matrix->numCols + j]);
            SendString_serial(out);
        }
        SendString_serial("\r\n");
    }
    return;
}

void print_var(float32_t var, char *name)
{
    char out[100];
    sprintf(out, "%s: %f\r\n", name, var);
    SendString_serial(out);
    return;
}

// int matrix_init (arm_matrix_instance_f32 *matrix, uint32_t row, uint32_t col) {
//     matrix->pData = (float32_t *)malloc(row * col * sizeof(float32_t));
//     if(matrix->pData == NULL){
//         // 处理内存分配失败的情况
//         return -1;
//     }
//     memset(matrix->pData, 0, row * col * sizeof(float32_t));
//     arm_mat_init_f32(matrix, row, col, matrix->pData);
// 		return 0;
// }

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void normalize(int n, float32_t *arr)
{
    float32_t norm = 0.0f;
    for (int i = 0; i < n; i++)
    {
        norm += arr[i] * arr[i];
    }
    norm = invSqrt(norm);
    for (int i = 0; i < n; i++)
    {
        arr[i] *= norm;
    }
    return;
}

void ekf_init()
{
    EKF_in = (EKF_input *)malloc(sizeof(EKF_input));
    ano_data = (ANO_data *)malloc(sizeof(ANO_data));
    ano_mpu_data = (ANO_MPU_data *)malloc(sizeof(ANO_MPU_data));
    ano_data->len = 9;
    memset(ano_data->data, 0, 9 * sizeof(int8_t));
    ano_data->data[1] = 1;
    ano_mpu_data->len = 13;
    memset(ano_mpu_data->data, 0, 13 * sizeof(int8_t));
    ano_data_euler = (ANO_data_euler *)malloc(sizeof(ANO_data_euler));
    ano_data_euler->len = 7;
    memset(ano_data_euler->data, 0, 7 * sizeof(int8_t));

    // 初始化EKF_in结构体
    arm_mat_init_f32(&(EKF_in->x_k_prev), 7, 1, x_prev_data);
    arm_mat_init_f32(&(EKF_in->f), 7, 7, f_data);
    arm_mat_init_f32(&(EKF_in->P_k_prev), 7, 7, P_prev_data);
    arm_mat_init_f32(&(EKF_in->Q), 7, 7, Q_data);
    arm_mat_init_f32(&(EKF_in->R), 6, 6, R_data);
    arm_mat_init_f32(&(EKF_in->Z_k), 6, 1, Z_k_data);
    // if (matrix_init(&(EKF_in->x_k_prev), 7, 1) != 0) {
    //     SendString_serial("EKF_in->x_k_prev initialization failed\r\n");
    // }
    // if (matrix_init(&(EKF_in->f), 7, 7) != 0) {
    //     SendString_serial("EKF_in->f initialization failed\r\n");
    // }
    // if (matrix_init(&(EKF_in->P_k_prev), 7, 7) != 0) {
    //     SendString_serial("EKF_in->P_k_prev initialization failed\r\n");
    // }
    // if (matrix_init(&(EKF_in->Q), 7, 7) != 0) {
    //     SendString_serial("EKF_in->Q initialization failed\r\n");
    // }
    // if (matrix_init(&(EKF_in->R), 6, 6) != 0) {
    //     SendString_serial("EKF_in->R initialization failed\r\n");
    // }
    // if (matrix_init(&(EKF_in->Z_k), 6, 1) != 0) {
    //     SendString_serial("EKF_in->Z_k initialization failed\r\n");
    // }

    // 初始化状态向量
    EKF_in->x_k_prev.pData[0] = 1.0f;
    // EKF_in->x_k_prev.pData[1] = EKF_in->x_k_prev.pData[2] = EKF_in->x_k_prev.pData[3] = 0.0f;
    // EKF_in->x_k_prev.pData[4] = EKF_in->x_k_prev.pData[5] = EKF_in->x_k_prev.pData[6] = 0.0f;

    // 初始化状态协方差矩阵
    // EKF_in->P_k_prev.pData[0] = EKF_in->P_k_prev.pData[5] = EKF_in->P_k_prev.pData[10] = EKF_in->P_k_prev.pData[15] = 0.125f;
    // EKF_in->P_k_prev.pData[1] = EKF_in->P_k_prev.pData[2] = EKF_in->P_k_prev.pData[3] = EKF_in->P_k_prev.pData[4] = EKF_in->P_k_prev.pData[6] = EKF_in->P_k_prev.pData[7] = EKF_in->P_k_prev.pData[8] = EKF_in->P_k_prev.pData[9] = EKF_in->P_k_prev.pData[11] = EKF_in->P_k_prev.pData[12] = EKF_in->P_k_prev.pData[13] = EKF_in->P_k_prev.pData[14] = 0.0003f;
    for (int i = 0; i < 49; i++)
    {
        if (i % 8 == 0)
            EKF_in->P_k_prev.pData[i] = 0.125f;
        else
            EKF_in->P_k_prev.pData[i] = 0.003f;
    }

    // 初始化过程噪声矩阵
    // EKF_in->Q_k_prev.pData[0] = EKF_in->Q_k_prev.pData[5] = EKF_in->Q_k_prev.pData[10] = EKF_in->Q_k_prev.pData[15] = 0.000001f;
    // EKF_in->Q_k_prev.pData[1] = EKF_in->Q_k_prev.pData[2] = EKF_in->Q_k_prev.pData[3] = EKF_in->Q_k_prev.pData[4] = EKF_in->Q_k_prev.pData[6] = EKF_in->Q_k_prev.pData[7] = EKF_in->Q_k_prev.pData[8] = EKF_in->Q_k_prev.pData[9] = EKF_in->Q_k_prev.pData[11] = EKF_in->Q_k_prev.pData[12] = EKF_in->Q_k_prev.pData[13] = EKF_in->Q_k_prev.pData[14] = 0.0f;
    for (int i = 0; i < 49; i++)
    {
        if (i % 8 == 0)
            EKF_in->Q.pData[i] = 0.000001f;
        else
            EKF_in->Q.pData[i] = 0.0f;
    }

    // 初始化观测噪声矩阵
    // EKF_in->R.pData[0] = EKF_in->R.pData[4] = EKF_in->R.pData[8] = 0.125f;
    // EKF_in->R.pData[1] = EKF_in->R.pData[2] = EKF_in->R.pData[3] = EKF_in->R.pData[5] = EKF_in->R.pData[6] = EKF_in->R.pData[7] = 0.0f;
    for (int i = 0; i < 36; i++)
    {
        if (i % 7 == 0)
            EKF_in->R.pData[i] = 0.125f;
        else
            EKF_in->R.pData[i] = 0.0f;
    }
    // float32_t R[9] = {0.125f, 0.0f, 0.0f,
    //                   0.0f, 0.125f, 0.0f,
    //                   0.0f, 0.0f, 0.125f};
    // free(EKF_in->R.pData);
    // EKF_in->R.pData = R;
    EKF_in->tick_k_minus = OSTimeGet();
    // Send_Quaternion_Data();
}
void ekf_update()
{
    int16_t accx_raw = 0, accy_raw = 0, accz_raw = 0;
    int16_t gyrox_raw = 0, gyroy_raw = 0, gyroz_raw = 0;
    int16_t magx_raw = 0, magy_raw = 0, magz_raw = 0;
    float32_t accx = 0, accy = 0, accz = 0;
    float32_t gyrox = 0, gyroy = 0, gyroz = 0;
    float32_t magx = 0, magy = 0, magz = 0;
    // accx_raw = I2C1_GetMPU6050X() + ACC_OFFSET_X;
    // accy_raw = I2C1_GetMPU6050Y() + ACC_OFFSET_Y;
    // accz_raw = I2C1_GetMPU6050Z() + ACC_OFFSET_Z;
    // gyrox_raw = I2C1_GetGyroX() - GYRO_OFFSET_X;
    // gyroy_raw = I2C1_GetGyroY() - GYRO_OFFSET_Y;
    // gyroz_raw = I2C1_GetGyroZ() - GYRO_OFFSET_Z;
    // magx_raw = I2C1_GetHMC5883X();
    // magy_raw = I2C1_GetHMC5883Y();
    // magz_raw = I2C1_GetHMC5883Z();
    accx_raw = accx_read + ACC_OFFSET_X;
    accy_raw = accy_read + ACC_OFFSET_Y;
    accz_raw = accz_read + ACC_OFFSET_Z;
    gyrox_raw = gyrox_read - GYRO_OFFSET_X;
    gyroy_raw = gyroy_read - GYRO_OFFSET_Y;
    gyroz_raw = gyroz_read - GYRO_OFFSET_Z;
    magx_raw = magx_read;
    magy_raw = magy_read;
    magz_raw = magz_read;

    accx = accx_raw * ACC_RATIO;
    accy = accy_raw * ACC_RATIO;
    accz = accz_raw * ACC_RATIO;
    gyrox = gyrox_raw * GYRO_RATIO - EKF_in->x_k_prev.pData[4];
    gyroy = gyroy_raw * GYRO_RATIO - EKF_in->x_k_prev.pData[5];
    gyroz = gyroz_raw * GYRO_RATIO - EKF_in->x_k_prev.pData[6];
    magx = magx_raw * MAG_RATIO;
    magy = magy_raw * MAG_RATIO;
    magz = magz_raw * MAG_RATIO;

    ano_mpu_data -> data[0] = accx_raw & 0xff;
    ano_mpu_data -> data[1] = (accx_raw >> 8) & 0xff;
    ano_mpu_data -> data[2] = accy_raw & 0xff;
    ano_mpu_data -> data[3] = (accy_raw >> 8) & 0xff;
    ano_mpu_data -> data[4] = accz_raw & 0xff;
    ano_mpu_data -> data[5] = (accz_raw >> 8) & 0xff;
    ano_mpu_data -> data[6] = gyrox_raw & 0xff;
    ano_mpu_data -> data[7] = (gyrox_raw >> 8) & 0xff;
    ano_mpu_data -> data[8] = gyroy_raw & 0xff;
    ano_mpu_data -> data[9] = (gyroy_raw >> 8) & 0xff;
    ano_mpu_data -> data[10] = gyroz_raw & 0xff;
    ano_mpu_data -> data[11] = (gyroz_raw >> 8) & 0xff;
    ano_mpu_data -> data[12] = 0;
    // FANO_Send_Data(0x01, (uint8_t *)ano_mpu_data);

    // print_var(gyrox, "gyrox");
    // print_var(gyroy, "gyroy");
    // print_var(gyroz, "gyroz");

    // print_var(EKF_in -> x_k_prev.pData[4], "gyrox_prev");
    // print_var(EKF_in -> x_k_prev.pData[5], "gyroy_prev");
    // print_var(EKF_in -> x_k_prev.pData[6], "gyroz_prev");

    INT32U tick2 = OSTimeGet();

    EKF_in->Z_k.pData[0] = accx;
    EKF_in->Z_k.pData[1] = accy;
    EKF_in->Z_k.pData[2] = accz;
    EKF_in->Z_k.pData[3] = magx;
    EKF_in->Z_k.pData[4] = magy;
    EKF_in->Z_k.pData[5] = magz;
    normalize(3, &(EKF_in->Z_k.pData[0]));
    normalize(3, &(EKF_in->Z_k.pData[3]));
    EKF_in->halfT = (float)(tick2 - EKF_in->tick_k_minus) / (float)OS_TICKS_PER_SEC / 2.0f;
    EKF_in->tick_k_minus = tick2;

#define f(i) EKF_in->f.pData[i]
#define halfT EKF_in->halfT
    f(0) = 1.0f;
    f(1) = -gyrox * halfT;
    f(2) = -gyroy * halfT;
    f(3) = -gyroz * halfT;
    f(4) = 0.0f;
    f(5) = 0.0f;
    f(6) = 0.0f;
    f(7) = gyrox * halfT;
    f(8) = 1.0f;
    f(9) = gyroz * halfT;
    f(10) = -gyroy * halfT;
    f(11) = 0.0f;
    f(12) = 0.0f;
    f(13) = 0.0f;
    f(14) = gyroy * halfT;
    f(15) = -gyroz * halfT;
    f(16) = 1.0f;
    f(17) = gyrox * halfT;
    f(18) = 0.0f;
    f(19) = 0.0f;
    f(20) = 0.0f;
    f(21) = gyroz * halfT;
    f(22) = gyroy * halfT;
    f(23) = -gyrox * halfT;
    f(24) = 1.0f;
    f(25) = 0.0f;
    f(26) = 0.0f;
    f(27) = 0.0f;
    f(28) = 0.0f;
    f(29) = 0.0f;
    f(30) = 0.0f;
    f(31) = 0.0f;
    f(32) = 1.0f;
    f(33) = 0.0f;
    f(34) = 0.0f;
    f(35) = 0.0f;
    f(36) = 0.0f;
    f(37) = 0.0f;
    f(38) = 0.0f;
    f(39) = 0.0f;
    f(40) = 1.0f;
    f(41) = 0.0f;
    f(42) = 0.0f;
    f(43) = 0.0f;
    f(44) = 0.0f;
    f(45) = 0.0f;
    f(46) = 0.0f;
    f(47) = 0.0f;
    f(48) = 1.0f;
#undef f
#undef halfT
}

void h_k_func(arm_matrix_instance_f32 *x_k, arm_matrix_instance_f32 *h, float32_t bx, float32_t bz)
{
    float32_t tmp_2q1q3_minus_2q0q2 = 2 * (x_k->pData[1] * x_k->pData[3] - x_k->pData[0] * x_k->pData[2]);
    float32_t tmp_2q2q3_plus_2q0q1 = 2 * (x_k->pData[2] * x_k->pData[3] + x_k->pData[0] * x_k->pData[1]);
    float32_t tmp_2q1q2_plus_2q0q3 = 2 * (x_k->pData[1] * x_k->pData[2] + x_k->pData[0] * x_k->pData[3]);
    float32_t tmp_1_minus_2q1q1_minus_2q2q2 = 1.0f - 2 * (x_k->pData[1] * x_k->pData[1] + x_k->pData[2] * x_k->pData[2]);
    float32_t tmp_2q2q3_minus_2q0q1 = 2 * (x_k->pData[2] * x_k->pData[3] - x_k->pData[0] * x_k->pData[1]);
    float32_t tmp_1_minus_2q1q1_minus_2q3q3 = 1.0f - 2 * (x_k->pData[1] * x_k->pData[1] + x_k->pData[3] * x_k->pData[3]);
    h->pData[0] = tmp_2q1q3_minus_2q0q2;
    h->pData[1] = tmp_2q2q3_plus_2q0q1;
    h->pData[2] = tmp_1_minus_2q1q1_minus_2q2q2;
#ifndef SKIP_MAG_CALIBRATION
    h->pData[3] = bx * tmp_2q1q2_plus_2q0q3 + bz * tmp_2q1q3_minus_2q0q2;
    h->pData[4] = bx * tmp_1_minus_2q1q1_minus_2q3q3 + bz * tmp_2q2q3_plus_2q0q1;
    h->pData[5] = bx * tmp_2q2q3_minus_2q0q1 + bz * tmp_1_minus_2q1q1_minus_2q2q2;
#endif
#ifdef SKIP_MAG_CALIBRATION
    h->pData[3] = 1.0f;
    h->pData[4] = 1.0f;
    h->pData[5] = 1.0f;
#endif
    return;
}

void H_k_func(arm_matrix_instance_f32 *x_k, arm_matrix_instance_f32 *H_matrix, float32_t bx, float32_t bz)
{
    // Ha1=2*-q2*g; /*对观测方程系数矩阵相关偏导数值预先计算*/
    // Ha2=2*q3*g;/*对观测方程系数矩阵相关偏导数值预先计算*/
    // Ha3=2*-q0*g;/*对观测方程系数矩阵相关偏导数值预先计算*/
    // Ha4=2*q1*g;	/*对观测方程系数矩阵相关偏导数值预先计算*/
    // Hb1=2*(bx*q0-bz*q2);/*对观测方程系数矩阵相关偏导数值预先计算*/
    // Hb2=2*(bx*q1+bz*q3);/*对观测方程系数矩阵相关偏导数值预先计算*/
    // Hb3=2*(-bx*q2-bz*q0);/*对观测方程系数矩阵相关偏导数值预先计算*/
    // Hb4=2*(-bx*q3+bz*q1);/*对观测方程系数矩阵相关偏导数值预先计算*/
    float32_t Ha1, Ha2, Ha3, Ha4, Hb1, Hb2, Hb3, Hb4;
    Ha1 = 2 * (-x_k->pData[2]);
    Ha2 = 2 * (x_k->pData[3]);
    Ha3 = 2 * (-x_k->pData[0]);
    Ha4 = 2 * (x_k->pData[1]);
#ifndef SKIP_MAG_CALIBRATION
    // Hb1 = 2 * (bx * x_k->pData[0] - bz * x_k->pData[2]);
    // Hb2 = 2 * (bx * x_k->pData[1] + bz * x_k->pData[3]);
    // Hb3 = 2 * (-bx * x_k->pData[2] - bz * x_k->pData[0]);
    // Hb4 = 2 * (-bx * x_k->pData[3] + bz * x_k->pData[1]);
    Hb1 = 2 * (bx * x_k->pData[3] - bz * x_k->pData[2]);
    Hb2 = 2 * (bx * x_k->pData[2] + bz * x_k->pData[3]);
    Hb3 = 2 * (bx * x_k->pData[1] - bz * x_k->pData[0]);
    Hb4 = 2 * (bx * x_k->pData[0] + bz * x_k->pData[1]);
#endif
#ifdef SKIP_MAG_CALIBRATION
    Hb1 = Hb2 = Hb3 = Hb4 = 0;
#endif
#define H(i) H_matrix->pData[i]
    H(0) = Ha1;
    H(1) = Ha2;
    H(2) = Ha3;
    H(3) = Ha4;
    H(7) = Ha4;
    H(8) = -Ha3;
    H(9) = Ha2;
    H(10) = -Ha1;
    H(14) = -Ha3;
    H(15) = -Ha4;
    H(16) = Ha1;
    H(17) = Ha2;
    H(21) = Hb1;
    H(22) = Hb2;
    H(23) = Hb3;
    H(24) = Hb4;
    H(28) = Hb4;
    H(29) = -Hb3;
    H(30) = Hb2;
    H(31) = -Hb1;
    H(35) = -Hb3;
    H(36) = -Hb4;
    H(37) = Hb1;
    H(38) = Hb2;
#undef H
    return;
}

void K_func(arm_matrix_instance_f32 *P_k_prev, arm_matrix_instance_f32 *H_k, arm_matrix_instance_f32 *R_k, arm_matrix_instance_f32 *K)
{
    arm_matrix_instance_f32 H_k_trans, temp_6x6, temp_7x6;
    float32_t H_k_trans_data[42] = {0.0f};
    float32_t temp_6x6_data[36] = {0.0f};
    float32_t temp_7x6_data[42] = {0.0f};
    arm_mat_init_f32(&H_k_trans, 7, 6, (float32_t *)H_k_trans_data);
    arm_mat_init_f32(&temp_6x6, 6, 6, (float32_t *)temp_6x6_data);
    arm_mat_init_f32(&temp_7x6, 7, 6, (float32_t *)temp_7x6_data);
    arm_mat_trans_f32(H_k, &H_k_trans);
    arm_mat_mult_f32(P_k_prev, &H_k_trans, &temp_7x6);
    arm_mat_mult_f32(H_k, &temp_7x6, &temp_6x6);
    arm_mat_add_f32(&temp_6x6, R_k, &temp_6x6);
    arm_mat_inverse_f32(&temp_6x6, &temp_6x6);

    arm_mat_mult_f32(&H_k_trans, &temp_6x6, K);
    arm_mat_mult_f32(P_k_prev, K, K);
    return;
}

void ekf_calculate()
{
    ekf_update(EKF_in);
    // float32_t euler_x, euler_y, euler_z;
    // euler_x = asin(2 * (EKF_in -> x_k_prev.pData[2] * EKF_in -> x_k_prev.pData[3] + EKF_in -> x_k_prev.pData[0] * EKF_in -> x_k_prev.pData[1]));
    // arm_atan2_f32(2 * (EKF_in -> x_k_prev.pData[0] * EKF_in -> x_k_prev.pData[2] - EKF_in -> x_k_prev.pData[1] * EKF_in -> x_k_prev.pData[3]), 1 - 2 * (EKF_in -> x_k_prev.pData[2] * EKF_in -> x_k_prev.pData[2] + EKF_in -> x_k_prev.pData[1] * EKF_in -> x_k_prev.pData[1]), &euler_y);
    // arm_atan2_f32(2 * (EKF_in -> x_k_prev.pData[0] * EKF_in -> x_k_prev.pData[3] - EKF_in -> x_k_prev.pData[1] * EKF_in -> x_k_prev.pData[2]), 1 - 2 * (EKF_in -> x_k_prev.pData[1] * EKF_in -> x_k_prev.pData[1] + EKF_in -> x_k_prev.pData[3] * EKF_in -> x_k_prev.pData[3]), &euler_z);
    // euler_x = euler_x * 180 / PI;
    // euler_y = euler_y * 180 / PI;
    // euler_z = euler_z * 180 / PI;
    // ano_data_euler -> data[0] = (int16_t)(euler_x*100) & 0xff;
    // ano_data_euler -> data[1] = ((int16_t)(euler_x*100) >> 8) & 0xff;
    // ano_data_euler -> data[2] = (int16_t)(euler_y*100) & 0xff;
    // ano_data_euler -> data[3] = ((int16_t)(euler_y*100) >> 8) & 0xff;
    // ano_data_euler -> data[4] = (int16_t)(euler_z*100) & 0xff;
    // ano_data_euler -> data[5] = ((int16_t)(euler_z*100) >> 8) & 0xff;
    // ano_data_euler -> data[6] = 0;
    // Send_Euler_Data();
    arm_matrix_instance_f32 I_7x7, x_k_minus, P_k_minus, F_k;
    float32_t x_k_minus_data[7] = {0.0f}, P_k_minus_data[49] = {0.0f}, F_k_data[49] = {0.0f};
    float32_t I_7x7_data[49] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    // for(int i = 0; i < 7; i++){
    //     x_k_minus_data[i] = EKF_in -> x_k_prev.pData[i];
    // }
    arm_mat_init_f32(&I_7x7, 7, 7, (float32_t *)I_7x7_data);
    arm_mat_init_f32(&x_k_minus, 7, 1, (float32_t *)x_k_minus_data);
    arm_mat_init_f32(&P_k_minus, 7, 7, (float32_t *)P_k_minus_data);
    arm_mat_init_f32(&F_k, 7, 7, (float32_t *)F_k_data);

    // $\hat{x_k^-}=f(hat{x_{k-1}^-}$
    arm_mat_mult_f32(&(EKF_in->f), &(EKF_in->x_k_prev), &x_k_minus);
    normalize(4, &(x_k_minus.pData[0]));

    for (int i = 0; i < 49; i++)
    {
        F_k_data[i] = EKF_in->f.pData[i];
    }
    // #ifndef SKIP_MAG_CALIBRATION
    // #define F(i) F_k.pData[i]
    // #define halfT EKF_in -> halfT
    // F(4) = halfT * x_prev_data[1]; F(5) = halfT * x_prev_data[2]; F(6) = halfT * x_prev_data[3];
    // F(11) = -halfT * x_prev_data[0]; F(12) = halfT * x_prev_data[3]; F(13) = -halfT * x_prev_data[2];
    // F(18) = -halfT * x_prev_data[3]; F(19) = -halfT * x_prev_data[0]; F(21) = halfT * x_prev_data[1];
    // F(25) = halfT * x_prev_data[2]; F(26) = -halfT * x_prev_data[1]; F(27) = -halfT * x_prev_data[0];
    // #undef F
    // #undef halfT
    // #endif

    // $P_k^- = F_k P_{k-1}^- F_k^T + Q$
    arm_mat_trans_f32(&F_k, &F_k);
    arm_mat_mult_f32(&(EKF_in->P_k_prev), &F_k, &P_k_minus);
    arm_mat_trans_f32(&F_k, &F_k);
    arm_mat_mult_f32(&F_k, &P_k_minus, &P_k_minus);
    arm_mat_add_f32(&P_k_minus, &(EKF_in->Q), &P_k_minus);

    arm_matrix_instance_f32 h_x_k, H_k, K_k, x_k, P_k;
    float32_t h_k_data[6] = {0.0f}, H_k_data[42] = {0.0f}, K_k_data[42] = {0.0f}, x_k_data[7] = {0.0f}, P_k_data[49] = {0.0f};
    arm_mat_init_f32(&h_x_k, 6, 1, (float32_t *)h_k_data);
    arm_mat_init_f32(&H_k, 6, 7, (float32_t *)H_k_data);
    arm_mat_init_f32(&K_k, 7, 6, (float32_t *)K_k_data);
    arm_mat_init_f32(&x_k, 7, 1, (float32_t *)x_k_data);
    arm_mat_init_f32(&P_k, 7, 7, (float32_t *)P_k_data);

    float32_t m_n[4] = {0.0f};
    float32_t bx = 0.0f, bz = 0.0f;
    quaternion_multiply(0.0f, Z_k_data[3], Z_k_data[4], Z_k_data[5], x_k_minus_data[0], -x_k_minus_data[1], -x_k_minus_data[2], -x_k_minus_data[3], m_n);
    quaternion_multiply(m_n[0], m_n[1], m_n[2], m_n[3], x_k_minus_data[0], x_k_minus_data[1], x_k_minus_data[2], x_k_minus_data[3], m_n);
    arm_sqrt_f32(m_n[1] * m_n[1] + m_n[2] * m_n[2], &bx);
    bz = m_n[3];
    // print_var(bx,"bx");
    // print_var(bz,"bz");
    h_k_func(&x_k_minus, &h_x_k, bx, bz);
    H_k_func(&x_k_minus, &H_k, bx, bz);
    K_func(&P_k_minus, &H_k, &(EKF_in->R), &K_k);

    arm_mat_sub_f32(&(EKF_in->Z_k), &h_x_k, &h_x_k);
    arm_mat_mult_f32(&K_k, &h_x_k, &x_k);
    arm_mat_add_f32(&x_k, &x_k_minus, &x_k);
    arm_mat_mult_f32(&K_k, &H_k, &P_k);
    arm_mat_sub_f32(&I_7x7, &P_k, &P_k);
    arm_mat_mult_f32(&P_k, &(EKF_in->P_k_prev), &P_k);

    normalize(4, &(x_k.pData[0]));

    for (int i = 0; i < 7; i++)
    {
        EKF_in->x_k_prev.pData[i] = x_k.pData[i];
        if (i < 4)
        {
            int16_t quaternion_data = (int16_t)(x_k.pData[i] * 10000);
            ano_data->data[2 * i + 1] = (quaternion_data) >> 8;
            ano_data->data[2 * i] = (quaternion_data) & 0xFF;
        }
    }
    for (int i = 0; i < 49; i++)
    {
        EKF_in->P_k_prev.pData[i] = P_k.pData[i];
    }

    // Send_Quaternion_Data();
    // print_martix(&(EKF_in -> x_k_prev), "x_k_prev");
    return;
}