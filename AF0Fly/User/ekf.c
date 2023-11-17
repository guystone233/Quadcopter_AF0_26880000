#include "ekf.h"

/*
X: 状态向量，4*1
PHI: 状态转移矩阵，4*4
H: 观测矩阵，3*4
dH: 观测矩阵Jacobian Matrix，3*4
P: 状态协方差矩阵，4*4
Q: 过程噪声矩阵Q，4*4
R: 观测噪声矩阵R，3*3
Z: 观测向量，3*1
Zkk_1: 先验观测估计，3*1
K: kalman增益矩阵，4*3
*/

EKF_input EKF_in;
ANO_data ano_data;
ANO_data_euler ano_data_euler;
ANO_MPU_data ano_mpu_data;

#define DEBUG_PRINT_EKF
// #define SKIP_FIRST_FILTER
#define SKIP_SECOND_FILTER
// #define PRINT_IMU

float32_t x_prev_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float32_t A_data[16] = {0.0f};
float32_t P_prev_data[16] = {0.125f, 0.0003f, 0.0003f, 0.0003f,
                            0.0003f, 0.125f, 0.0003f, 0.0003f,
                            0.0003f, 0.0003f, 0.125f, 0.0003f,
                            0.0003f, 0.0003f, 0.0003f, 0.125f};
float32_t Q_data[16] = {0.0001f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0001f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0001f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.0001f};
float32_t R_data[9] = {1.125f, 0.0f, 0.0f,
                       0.0f, 1.125f, 0.0f,
                       0.0f, 0.0f, 1.125f};
float32_t Z_1_data[3] = {0.0f};
float32_t Z_2_data[3] = {0.0f};

void Send_Quaternion_Data();
inline void Send_Quaternion_Data(){
    #ifndef DEBUG_PRINT_EKF
    FANO_Send_Data(Frame_Quaternion, (uint8_t *)&ano_data);
    #endif
}

void Send_Euler_Data();
inline void Send_Euler_Data(){
    #ifndef DEBUG_PRINT_EKF
    FANO_Send_Data(Frame_EulerAngle, (uint8_t *)&ano_data_euler);
    #endif
}

void Send_MPU_Data();
inline void Send_MPU_Data(){
    #ifndef DEBUG_PRINT_EKF
    FANO_Send_Data(0x01, (uint8_t *)&ano_mpu_data);
    #endif
}

void quaternion_multiply(float32_t q0, float32_t q1, float32_t q2, float32_t q3, float32_t r0, float32_t r1, float32_t r2, float32_t r3, float32_t *result){
    result[0] = q0 * r0 - q1 * r1 - q2 * r2 - q3 * r3;
    result[1] = q0 * r1 + q1 * r0 + q2 * r3 - q3 * r2;
    result[2] = q0 * r2 - q1 * r3 + q2 * r0 + q3 * r1;
    result[3] = q0 * r3 + q1 * r2 - q2 * r1 + q3 * r0;
    return;
}

void print_martix(arm_matrix_instance_f32 *matrix, char *name) {
    #ifdef DEBUG_PRINT_EKF
    char out[100];
    sprintf(out, "%s: \r\n", name);
    SendString(out);
    for(int i = 0; i < matrix->numRows; i++){
        for(int j = 0; j < matrix->numCols; j++){
            sprintf(out, "%f, ", matrix->pData[i * matrix->numCols + j]);
            SendString(out);
        }
        SendString("\r\n");
    }
    return;
    #endif
    #ifndef DEBUG_PRINT_EKF
    return;
	#endif
}

void print_var(float32_t var, char *name) {
    #ifdef DEBUG_PRINT_EKF
    char out[100];
    sprintf(out, "%s: %f\r\n", name, var);
    SendString(out);
    return;
    #endif
    #ifndef DEBUG_PRINT_EKF
    return;
    #endif
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void normalize (int n, float32_t *arr) {
    float32_t norm = 0.0f;
    for (int i = 0; i < n; i++) {
        norm += arr[i] * arr[i];
    }
    norm = invSqrt(norm);
    for (int i = 0; i < n; i++) {
        arr[i] *= norm;
    }
    return;
}

void write_8bit(int16_t data, int8_t *arr);
inline void write_8bit(int16_t data, int8_t *arr) {
    arr[0] = data & 0xff;
    arr[1] = (data >> 8) & 0xff;
    return;
}

void ekf_init (){
    ano_data.len = 9;
    memset(ano_data.data, 0, 9 * sizeof(int8_t));
    ano_data.data[1] = 1;
    ano_mpu_data.len = 13;
    memset(ano_mpu_data.data, 0, 13 * sizeof(int8_t));
    ano_data_euler.len = 7;
    memset(ano_data_euler.data, 0, 7 * sizeof(int8_t));

    // 初始化EKF_in结构体
    arm_mat_init_f32(&(EKF_in.x_k_prev), 4, 1, x_prev_data);
    arm_mat_init_f32(&(EKF_in.A_k), 4, 4, A_data);
    arm_mat_init_f32(&(EKF_in.P_k_prev), 4, 4, P_prev_data);
    arm_mat_init_f32(&(EKF_in.Q_k_prev), 4, 4, Q_data);
    arm_mat_init_f32(&(EKF_in.R), 3, 3, R_data);
    arm_mat_init_f32(&(EKF_in.Z_1), 3, 1, Z_1_data);
    arm_mat_init_f32(&(EKF_in.Z_2), 3, 1, Z_2_data);

    // // 初始化状态向量
    // EKF_in->x_k_prev.pData[0] = 1.0f;
    // EKF_in->x_k_prev.pData[1] = 0.0f;
    // EKF_in->x_k_prev.pData[2] = 0.0f;
    // EKF_in->x_k_prev.pData[3] = 0.0f;
    // // float32_t X[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    // // arm_mat_init_f32(&(EKF_in->x_k_prev), 4, 1, X);

    // // char out[100];
    // // sprintf(out, "init,x_k_prev: %f, %f, %f, %f\r\n", EKF_in -> x_k_prev.pData[0], EKF_in -> x_k_prev.pData[1], EKF_in -> x_k_prev.pData[2], EKF_in -> x_k_prev.pData[3]);
    // // SendString(out);

    // // 初始化状态协方差矩阵
    // /*
    // kf->P <<0.125, 0.0003, 0.0003, 0.0003,
	// 		0.0003, 0.125, 0.0003, 0.0003,
	// 		0.0003, 0.0003, 0.125, 0.0003,
	// 		0.0003, 0.0003, 0.0003, 0.125;
    // */
    // EKF_in->P_k_prev.pData[0] = EKF_in->P_k_prev.pData[5] = EKF_in->P_k_prev.pData[10] = EKF_in->P_k_prev.pData[15] = 0.125f;
    // EKF_in->P_k_prev.pData[1] = EKF_in->P_k_prev.pData[2] = EKF_in->P_k_prev.pData[3] = EKF_in->P_k_prev.pData[4] = EKF_in->P_k_prev.pData[6] = EKF_in->P_k_prev.pData[7] = EKF_in->P_k_prev.pData[8] = EKF_in->P_k_prev.pData[9] = EKF_in->P_k_prev.pData[11] = EKF_in->P_k_prev.pData[12] = EKF_in->P_k_prev.pData[13] = EKF_in->P_k_prev.pData[14] = 0.0003f;

    // // 初始化过程噪声矩阵
    // EKF_in->Q_k_prev.pData[0] = EKF_in->Q_k_prev.pData[5] = EKF_in->Q_k_prev.pData[10] = EKF_in->Q_k_prev.pData[15] = 0.000001f;
    // EKF_in->Q_k_prev.pData[1] = EKF_in->Q_k_prev.pData[2] = EKF_in->Q_k_prev.pData[3] = EKF_in->Q_k_prev.pData[4] = EKF_in->Q_k_prev.pData[6] = EKF_in->Q_k_prev.pData[7] = EKF_in->Q_k_prev.pData[8] = EKF_in->Q_k_prev.pData[9] = EKF_in->Q_k_prev.pData[11] = EKF_in->Q_k_prev.pData[12] = EKF_in->Q_k_prev.pData[13] = EKF_in->Q_k_prev.pData[14] = 0.0f;
    // // float32_t Q[16] = {0.0003f, 0.0f, 0.0f, 0.0f,
    // //                    0.0f, 0.0003f, 0.0f, 0.0f,
    // //                    0.0f, 0.0f, 0.0003f, 0.0f,
    // //                    0.0f, 0.0f, 0.0f, 0.0003f};
    // // free(EKF_in->Q_k_prev.pData);
    // // EKF_in->Q_k_prev.pData = Q;

    // // 初始化观测噪声矩阵
    // EKF_in->R.pData[0] = EKF_in->R.pData[4] = EKF_in->R.pData[8] = 0.125f;
    // EKF_in->R.pData[1] = EKF_in->R.pData[2] = EKF_in->R.pData[3] = EKF_in->R.pData[5] = EKF_in->R.pData[6] = EKF_in->R.pData[7] = 0.0f;
    // // float32_t R[9] = {0.125f, 0.0f, 0.0f,
    // //                   0.0f, 0.125f, 0.0f,
    // //                   0.0f, 0.0f, 0.125f};
    // // free(EKF_in->R.pData);
    // // EKF_in->R.pData = R;
    EKF_in.tick_k_minus = OSTimeGet();
    Send_Quaternion_Data();
}
void ekf_update() {
    int16_t accx_raw = 0, accy_raw = 0, accz_raw = 0;
    int16_t gyrox_raw = 0, gyroy_raw = 0, gyroz_raw = 0;
    int16_t magx_raw = 0, magy_raw = 0, magz_raw = 0;
    float32_t accx = 0, accy = 0, accz = 0;
    float32_t gyrox = 0, gyroy = 0, gyroz = 0;
    float32_t magx = 0, magy = 0, magz = 0;
    accx_raw = I2C1_GetMPU6050X() + ACC_OFFSET_X;
    accy_raw = I2C1_GetMPU6050Y() + ACC_OFFSET_Y;
    accz_raw = I2C1_GetMPU6050Z() + ACC_OFFSET_Z;
    gyrox_raw = I2C1_GetGyroX() - GYRO_OFFSET_X;
    gyroy_raw = I2C1_GetGyroY() - GYRO_OFFSET_Y;
    gyroz_raw = I2C1_GetGyroZ() - GYRO_OFFSET_Z;
    magx_raw = I2C1_GetHMC5883X();
    magy_raw = I2C1_GetHMC5883Y();
    magz_raw = I2C1_GetHMC5883Z();
    accx = accx_raw * ACC_RATIO;
    accy = accy_raw * ACC_RATIO;
    accz = accz_raw * ACC_RATIO;
    gyrox = gyrox_raw * GYRO_RATIO;
    gyroy = gyroy_raw * GYRO_RATIO;
    gyroz = gyroz_raw * GYRO_RATIO;
    magx = magx_raw * MAG_RATIO;
    magy = magy_raw * MAG_RATIO;
    magz = magz_raw * MAG_RATIO;

    #ifdef PRINT_IMU
    write_8bit(accx_raw, &(ano_mpu_data.data[0]));
    write_8bit(accy_raw, &(ano_mpu_data.data[2]));
    write_8bit(accz_raw, &(ano_mpu_data.data[4]));
    write_8bit(gyrox_raw, &(ano_mpu_data.data[6]));
    write_8bit(gyroy_raw, &(ano_mpu_data.data[8]));
    write_8bit(gyroz_raw, &(ano_mpu_data.data[10]));
    ano_mpu_data.data[12] = 0;
    Send_MPU_Data();
    #endif

    INT32U tick2 = OSTimeGet();

    EKF_in.Z_1.pData[0] = accx;
    EKF_in.Z_1.pData[1] = accy;
    EKF_in.Z_1.pData[2] = accz;
    normalize(3, EKF_in.Z_1.pData);
    EKF_in.Z_2.pData[0] = magx;
    EKF_in.Z_2.pData[1] = magy;
    EKF_in.Z_2.pData[2] = magz;
    normalize(3, EKF_in.Z_2.pData);

    EKF_in.halfT = (float)(tick2 - EKF_in.tick_k_minus) / (float)OS_TICKS_PER_SEC / 2.0f;
    #define f(x) EKF_in.A_k.pData[x]
    #define halfT EKF_in.halfT
    f(0) = 1.0f;           f(1) = -gyrox * halfT; f(2) = -gyroy * halfT;  f(3) = -gyroz * halfT;
    f(4) = gyrox * halfT;  f(5) = 1.0f;           f(6) = gyroz * halfT;   f(7) = -gyroy * halfT;
    f(8) = gyroy * halfT;  f(9) = -gyroz * halfT; f(10) = 1.0f;           f(11) = gyrox * halfT;
    f(12) = gyroz * halfT; f(13) = gyroy * halfT; f(14) = -gyrox * halfT; f(15) = 1.0f;
    #undef f
    #undef halfT
    // print_martix(&EKF_in.A_k,"A_k");
    EKF_in.tick_k_minus = tick2;
    
}

void h_1(arm_matrix_instance_f32 *x_k, arm_matrix_instance_f32 *h){
    h->pData[0] = 2 * (x_k->pData[1] * x_k->pData[3] - x_k->pData[0] * x_k->pData[2]);
    h->pData[1] = 2 * (x_k->pData[2] * x_k->pData[3] + x_k->pData[0] * x_k->pData[1]);
    // h->pData[2] = x_k->pData[0] * x_k->pData[0] - x_k->pData[1] * x_k->pData[1] - x_k->pData[2] * x_k->pData[2] + x_k->pData[3] * x_k->pData[3];
    h->pData[2] = 1 - 2 * (x_k->pData[1] * x_k->pData[1] + x_k->pData[2] * x_k->pData[2]);
    return;
}

void h_2(arm_matrix_instance_f32 *x_k, arm_matrix_instance_f32 *h){
    h->pData[0] = 2 * (x_k->pData[1] * x_k->pData[2] + x_k->pData[0] * x_k->pData[3]);
    h->pData[1] = x_k->pData[0] * x_k->pData[0] - x_k->pData[1] * x_k->pData[1] + x_k->pData[2] * x_k->pData[2] - x_k->pData[3] * x_k->pData[3];
    h->pData[2] = 2 * (x_k->pData[2] * x_k->pData[3] - x_k->pData[0] * x_k->pData[1]);
    return;
}

void H_1(arm_matrix_instance_f32 *x_k, arm_matrix_instance_f32 *H){
    H->pData[0] = -2 * x_k->pData[2];
    H->pData[1] = 2 * x_k->pData[3];
    H->pData[2] = -2 * x_k->pData[0];
    H->pData[3] = 2 * x_k->pData[1];
    H->pData[4] = 2 * x_k->pData[1];
    H->pData[5] = 2 * x_k->pData[0];
    H->pData[6] = 2 * x_k->pData[3];
    H->pData[7] = 2 * x_k->pData[2];
    H->pData[8] = 2 * x_k->pData[0];
    H->pData[9] = -2 * x_k->pData[1];
    H->pData[10] = -2 * x_k->pData[2];
    H->pData[11] = 2 * x_k->pData[3];
    return;
}

void H_2(arm_matrix_instance_f32 *x_k, arm_matrix_instance_f32 *H){
    H->pData[0] = 2 * x_k->pData[1];
    H->pData[1] = 2 * x_k->pData[0];
    H->pData[2] = 2 * x_k->pData[3];
    H->pData[3] = 2 * x_k->pData[2];
    H->pData[4] = -2 * x_k->pData[2];
    H->pData[5] = 2 * x_k->pData[3];
    H->pData[6] = 2 * x_k->pData[0];
    H->pData[7] = -2 * x_k->pData[1];
    H->pData[8] = -2 * x_k->pData[1];
    H->pData[9] = -2 * x_k->pData[0];
    H->pData[10] = 2 * x_k->pData[1];
    H->pData[11] = 2 * x_k->pData[2];
    return;
}

void K(arm_matrix_instance_f32 *P_k_prev, arm_matrix_instance_f32 *H_k, arm_matrix_instance_f32 *R_k, arm_matrix_instance_f32 *K){
    arm_matrix_instance_f32 H_k_trans, temp;
    float32_t H_k_trans_data[12] = {0.0f};
    float32_t temp_data[9] = {0.0f};
    arm_mat_init_f32(&H_k_trans, 4, 3, (float32_t *)H_k_trans_data);
    arm_mat_init_f32(&temp, 3, 3, (float32_t *)temp_data);

    arm_mat_trans_f32(H_k, &H_k_trans);
    arm_mat_mult_f32(P_k_prev, &H_k_trans, &temp);
    arm_mat_mult_f32(H_k, &temp, &temp);
    arm_mat_add_f32(&temp, R_k, &temp);
    arm_mat_inverse_f32(&temp, &temp);
    arm_mat_mult_f32(&H_k_trans, &temp, K);
    arm_mat_mult_f32(P_k_prev, K, K);
    return;
}

void ekf_calculate(){
    ekf_update(EKF_in);
    // print_martix(&(EKF_in.x_k_prev), "x_k_prev");
    // float32_t euler_roll, euler_pitch, euler_yaw;
    // arm_atan2_f32(2 * (EKF_in.x_k_prev.pData[2] * EKF_in.x_k_prev.pData[3] + EKF_in.x_k_prev.pData[0] * EKF_in.x_k_prev.pData[1]), 1 - 2 * (EKF_in.x_k_prev.pData[2] * EKF_in.x_k_prev.pData[2] + EKF_in.x_k_prev.pData[1] * EKF_in.x_k_prev.pData[1]), &euler_roll);
    // euler_pitch = asin(-2 * (EKF_in.x_k_prev.pData[0] * EKF_in.x_k_prev.pData[2] - EKF_in.x_k_prev.pData[1] * EKF_in.x_k_prev.pData[3]));
    // arm_atan2_f32(2 * (EKF_in.x_k_prev.pData[1] * EKF_in.x_k_prev.pData[2] + EKF_in.x_k_prev.pData[0] * EKF_in.x_k_prev.pData[3]), 1 - 2 * (EKF_in.x_k_prev.pData[2] * EKF_in.x_k_prev.pData[2] + EKF_in.x_k_prev.pData[3] * EKF_in.x_k_prev.pData[3]), &euler_yaw);
    // euler_roll = euler_roll * 180 / PI;
    // euler_pitch = euler_pitch * 180 / PI;
    // euler_yaw = euler_yaw * 180 / PI;
    // print_var(euler_roll, "euler_roll");
    // print_var(euler_pitch, "euler_pitch");
    // print_var(euler_yaw, "euler_yaw");
    // write_8bit((int16_t)(euler_roll * 100), &(ano_data_euler.data[0]));
    // write_8bit((int16_t)(euler_pitch * 100), &(ano_data_euler.data[2]));
    // write_8bit((int16_t)(euler_yaw * 100), &(ano_data_euler.data[4]));
    // ano_data_euler.data[6] = 0;
    // Send_Euler_Data();
    arm_matrix_instance_f32 I_4x4, x_k_minus, P_k_minus;
    float32_t x_k_minus_data[4] = {0.0f}, P_k_minus_data[16] = {0.0f};
    float32_t I_4x4_data[16] = {1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f};
    arm_mat_init_f32(&I_4x4, 4, 4, (float32_t *)I_4x4_data);
    arm_mat_init_f32(&x_k_minus, 4, 1, (float32_t *)x_k_minus_data);
    arm_mat_init_f32(&P_k_minus, 4, 4, (float32_t *)P_k_minus_data);

    arm_mat_mult_f32(&EKF_in.A_k, &EKF_in.x_k_prev, &x_k_minus);
    normalize(4, x_k_minus.pData);

    arm_mat_trans_f32(&EKF_in.A_k, &EKF_in.A_k);
    arm_mat_mult_f32(&EKF_in.P_k_prev, &EKF_in.A_k, &P_k_minus);
    arm_mat_trans_f32(&EKF_in.A_k, &EKF_in.A_k);
    arm_mat_mult_f32(&EKF_in.A_k, &P_k_minus, &P_k_minus);
    arm_mat_add_f32(&P_k_minus, &EKF_in.Q_k_prev, &P_k_minus);

    #ifndef SKIP_FIRST_FILTER
    arm_matrix_instance_f32 h_k_1, H_k_1, K_k_1, x_k_1, P_k_1;
    float32_t h_k_1_data[3] = {0.0f}, H_k_1_data[12] = {0.0f}, K_k_1_data[12] = {0.0f}, x_k_1_data[4] = {0.0f}, P_k_1_data[16] = {0.0f};
    arm_mat_init_f32(&h_k_1, 3, 1, (float32_t *)h_k_1_data);
    arm_mat_init_f32(&H_k_1, 3, 4, (float32_t *)H_k_1_data);
    arm_mat_init_f32(&K_k_1, 4, 3, (float32_t *)K_k_1_data);
    arm_mat_init_f32(&x_k_1, 4, 1, (float32_t *)x_k_1_data);
    arm_mat_init_f32(&P_k_1, 4, 4, (float32_t *)P_k_1_data);

    h_1(&x_k_minus,&h_k_1);
    H_1(&x_k_minus,&H_k_1);
    K(&P_k_minus, &H_k_1, &(EKF_in.R), &K_k_1);
    // print_martix(&K_k_1,"K");
    arm_mat_sub_f32(&EKF_in.Z_1, &h_k_1, &h_k_1);
    arm_mat_mult_f32(&K_k_1, &h_k_1, &x_k_1);
    x_k_1.pData[3] = 0.0f;
    arm_mat_add_f32(&x_k_minus, &x_k_1, &x_k_1);

    arm_mat_mult_f32(&K_k_1, &H_k_1, &K_k_1);
    arm_mat_sub_f32(&I_4x4, &K_k_1, &K_k_1);
    arm_mat_mult_f32(&K_k_1, &P_k_minus, &P_k_1);
    print_martix(&P_k_1,"P_k");
    #endif

    #ifndef SKIP_SECOND_FILTER
    arm_matrix_instance_f32 h_k_2, H_k_2, K_k_2, x_k_2, P_k_2;

    float32_t h_k_2_data[3] = {0.0f}, H_k_2_data[12] = {0.0f}, K_k_2_data[12] = {0.0f}, x_k_2_data[4] = {0.0f}, P_k_2_data[16] = {0.0f};
    arm_mat_init_f32(&h_k_2, 3, 1, (float32_t *)h_k_2_data);
    arm_mat_init_f32(&H_k_2, 3, 4, (float32_t *)H_k_2_data);
    arm_mat_init_f32(&K_k_2, 4, 3, (float32_t *)K_k_2_data);
    arm_mat_init_f32(&x_k_2, 4, 1, (float32_t *)x_k_2_data);
    arm_mat_init_f32(&P_k_2, 4, 4, (float32_t *)P_k_2_data);

    // float32_t m_k[4]={0.0f};
    // float32_t bx = 0.0f, bz = 0.0f;
    // // quaternion_multiply(0.0f,Z_k_data[3],Z_k_data[4],Z_k_data[5],x_k_minus_data[0],-x_k_minus_data[1],-x_k_minus_data[2],-x_k_minus_data[3],m_n);
    // // quaternion_multiply(m_n[0],m_n[1],m_n[2],m_n[3],x_k_minus_data[0],x_k_minus_data[1],x_k_minus_data[2],x_k_minus_data[3],m_n);
    // quaternion_multiply(0.0f, EKF_in.Z_2_data.pData[0], EKF_in.Z_2_data.pData[1], EKF_in.Z_2_data.pData[2]

    h_2(&x_k_minus,&h_k_2);
    H_2(&x_k_minus,&H_k_2);
    K(&P_k_1, &H_k_2, &(EKF_in.R), &K_k_2);
    arm_mat_sub_f32(&EKF_in.Z_2, &h_k_2, &h_k_2);
    arm_mat_mult_f32(&K_k_2, &h_k_2, &x_k_2);
    x_k_2.pData[0] = x_k_2.pData[1] = x_k_2.pData[2] = 0.0f;
    arm_mat_add_f32(&x_k_1, &x_k_2, &x_k_2);
    normalize(4, x_k_2.pData);

    arm_mat_mult_f32(&K_k_2, &H_k_2, &K_k_2);
    arm_mat_sub_f32(&I_4x4, &K_k_2, &K_k_2);
    arm_mat_mult_f32(&K_k_2, &P_k_1, &P_k_2);

    for(int i = 0; i < 4; i++){
        EKF_in.x_k_prev.pData[i] = x_k_2.pData[i];
        write_8bit((int16_t)(x_k_2.pData[i] * 10000), &(ano_data.data[2 * i]));
    }
    for(int i = 0; i < 16; i++){
        EKF_in.P_k_prev.pData[i] = P_k_2.pData[i];
    }
    #endif

    #ifdef SKIP_SECOND_FILTER
        #ifndef SKIP_FIRST_FILTER
        normalize(4, x_k_1.pData);
        for(int i = 0; i < 4; i++){
            EKF_in.x_k_prev.pData[i] = x_k_1.pData[i];
            write_8bit((int16_t)(x_k_1.pData[i] * 10000), &(ano_data.data[2 * i]));
        }
        for(int i = 0; i < 16; i++){
            EKF_in.P_k_prev.pData[i] = P_k_1.pData[i];
        }
        #endif
        #ifdef SKIP_FIRST_FILTER
        for(int i = 0; i < 4; i++){
            EKF_in.x_k_prev.pData[i] = x_k_minus.pData[i];
            write_8bit((int16_t)(x_k_minus.pData[i] * 10000), &(ano_data.data[2 * i]));
        }
        for(int i = 0; i < 16; i++){
            EKF_in.P_k_prev.pData[i] = P_k_minus.pData[i];
        }
        #endif
    #endif
    Send_Quaternion_Data();
    return;

}