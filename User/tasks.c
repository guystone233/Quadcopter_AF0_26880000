#include "tasks.h"

int16_t accx_read = 0, accy_read = 0, accz_read = 0;
int16_t gyrox_read = 0, gyroy_read = 0, gyroz_read = 0;
int16_t magx_read = 0, magy_read = 0, magz_read = 0;
int16_t tmp = 0;

float Pitch,Roll,Yaw;
long Temp;
ANO_data_euler ANO_data1;
short imu_data[6] = {0};


float inner_kp = 0.1f;
float inner_ki = 0.0f;
float inner_kd = 0.1f;

float inner_pitch_integrator = 0.0f;
float inner_roll_integrator = 0.0f;
// float inner_yaw_integrator = 0.0f;

float inner_pitch_lasterror = 0.0f;
float inner_roll_lasterror = 0.0f;
// float inner_yaw_lasterror = 0.0f;

float inner_pitch_output = 0.0f;
float inner_roll_output = 0.0f;

float inner_gyro_roll = 0.0f;
float inner_gyro_pitch = 0.0f;

float motor1 = 0.0f;
float motor2 = 0.0f;
float motor3 = 0.0f;
float motor4 = 0.0f;

float outer_kp = 0.1f;
float outer_ki = 0.0f;
float outer_kd = 0.1f;

float outer_pitch_integrator = 0.0f;
float outer_roll_integrator = 0.0f;
// float inner_yaw_integrator = 0.0f;

float outer_pitch_lasterror = 0.0f;
float outer_roll_lasterror = 0.0f;
// float inner_yaw_lasterror = 0.0f;

float outer_pitch_output = 0.0f;
float outer_roll_output = 0.0f;

float outer_rx_updown = 0.0f;
float outer_rx_yaw = 0.0f;
float outer_rx_pitch = 0.0f;
float outer_rx_roll = 0.0f;

float outer_read_roll = 0.0f;
float outer_read_pitch = 0.0f;
float outer_read_yaw = 0.0f;

// int8_t data[18] = {0};
char data[USART1_RX_BUF_SIZE] = {0};

/* Delay */
INT16U Task1000HZDelay = 50;
INT16U Task500HZDelay = 30;
INT16U Task250HZDelay = 10;

// INT16U gy86_delay = 150;
// INT16U kalman_delay = 145;
// INT16U send_delay = 80;
// INT16U oled_delay = 15;

// INT16U inner_loop_delay = 10;
// INT16U motor_delay = 5;

// INT16U outer_loop_delay = 3;
// INT16U blink_delay = 1;
/* Delay End */

/* Time */
INT32U gy86_time = 0;
INT32U kalman_time = 0;
INT32U send_time = 0;
INT32U receive_time = 0;
INT32U oled_time = 0;

INT32U inner_loop_time = 0;
INT32U motor_time = 0;

INT32U outer_loop_time = 0;
INT32U blink_time = 0;
/* Time End */

/* Task Stacks */
OS_STK Task1000HZStk[800];
// OS_STK GY86TaskStk[100];
// OS_STK KalmanTaskStk[300];
// OS_STK SendTaskStk[100];
// OS_STK OLEDTaskStk[100];

OS_STK Task500HZStk[800];
// OS_STK InnerLoopTaskStk[100];
// OS_STK MotorTaskStk[100];

OS_STK Task250HZStk[800];
// OS_STK OuterLoopTaskStk[100];
// OS_STK BlinkTaskStk[100];
/* Task Stacks End */

/*
*********************************************************************************************************
*                                             1000Hz TASKS
*
* Includes:
* 0. Task1000HZ
* 1. GY86Task
* 2. KalmanTask
* 3. SendTask
* 4. OLEDTask
*********************************************************************************************************
*/

void Task1000HZ(void *p_arg)
{
	while (1)
	{
		// GY86Task();
		KalmanTask();
		// MotorTask();
        SendTask();
		ReceiveTask();
		// OLEDTask();
		OSTimeDlyHMSM(0, 0, 0, Task1000HZDelay);
	}
}

void GY86Task()
{
	// Gy86: MPU6050(Accelerometer, Gyroscope), HMC5883(Magnetometer)
	INT32U tick1 = OSTimeGet();

	// I2C1_GetAll(data);
	// accx_read = (int16_t)((data[0] << 8) + data[1]);
	// accy_read = (int16_t)((data[2] << 8) + data[3]);
	// accz_read = (int16_t)((data[4] << 8) + data[5]);
	// tmp = (int16_t)((data[6] << 8) + data[7]) * 10;
	// gyrox_read = (int16_t)((data[8] << 8) + data[9]);
	// gyroy_read = (int16_t)((data[10] << 8) + data[11]);
	//gyroz_read = (int16_t)((data[12] << 8) + data[13]);
	accx_read = (int16_t) I2C1_GetAccX();
	accy_read = (int16_t) I2C1_GetAccY();
	accz_read = (int16_t) I2C1_GetAccZ();
	gyrox_read = (int16_t) I2C1_GetGyroX();
	gyroy_read = (int16_t) I2C1_GetGyroY();
	gyroz_read = (int16_t) I2C1_GetGyroZ();

#ifdef GY86
	magx_read = (int16_t) I2C1_GetMagX();
	magy_read = (int16_t) I2C1_GetMagY();
	magz_read = (int16_t) I2C1_GetMagZ();
#endif
	

	INT32U tick2 = OSTimeGet();
	gy86_time = tick2 - tick1;
}

void KalmanTask()
{
	INT32U tick1 = OSTimeGet();

	// ekf_calculate();

	MPU6050_Get_Euler_IMU(&Pitch,&Roll,&Yaw,&Temp,imu_data);
	// USART1_printf("Pitch : %.4f     ",(float)Pitch );
	// USART1_printf("Roll : %.4f    ",(float)Roll );
	// USART1_printf("Yaw : %.4f   \r\n",(float)Yaw );
	
	ANO_data1.len = 7;
	// ano_data_euler -> data[0] = (int16_t)(euler_x*100) & 0xff;
	// ano_data_euler -> data[1] = ((int16_t)(euler_x*100) >> 8) & 0xff;
	// ano_data_euler -> data[2] = (int16_t)(euler_y*100) & 0xff;
	// ano_data_euler -> data[3] = ((int16_t)(euler_y*100) >> 8) & 0xff;
	// ano_data_euler -> data[4] = (int16_t)(euler_z*100) & 0xff;
	// ano_data_euler -> data[5] = ((int16_t)(euler_z*100) >> 8) & 0xff;
	// ano_data_euler -> data[6] = 0;
	ANO_data1.data[0] = (int16_t)(Pitch*100) & 0xff;
	ANO_data1.data[1] = ((int16_t)(Pitch*100) >> 8) & 0xff;
	ANO_data1.data[2] = (int16_t)(-Roll*100) & 0xff;
	ANO_data1.data[3] = ((int16_t)(-Roll*100) >> 8) & 0xff;
	ANO_data1.data[4] = (int16_t)(-Yaw*100) & 0xff;
	ANO_data1.data[5] = ((int16_t)(-Yaw*100) >> 8) & 0xff;
	ANO_data1.data[6] = 0;
	FANO_Send_Data(Frame_EulerAngle,(uint8_t *) &ANO_data1);

	INT32U tick2 = OSTimeGet();
	kalman_time = tick2 - tick1;
	// USART1_printf("time : %d\r\n",kalman_time);
}

void SendTask()
{
	INT32U tick1 = OSTimeGet();

	// FANO_Send_Data(0x01, (uint8_t *)ano_mpu_data);
	// FANO_Send_MAG(data);
	// FANO_Send_Data(Frame_Quaternion, (uint8_t *)ano_data);
	//FANO_Send_Data(Frame_EulerAngle, (uint8_t *)ano_data_euler);
	// char out[100];
	// sprintf(out, "\r\n[chan:%d;%d;%d;%d;%d;%d]\r\n", ppm_CCR1data[1], ppm_CCR1data[2], ppm_CCR1data[3], ppm_CCR1data[4], ppm_CCR1data[5], ppm_CCR1data[6]);
	// SendString(out);

	USART1_printf("\r\n[in:%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f]\r\n",
		inner_gyro_roll, inner_gyro_pitch,
		inner_pitch_lasterror, inner_roll_lasterror,
		inner_pitch_integrator, inner_roll_integrator,
		inner_pitch_output, inner_roll_output,
		motor1, motor2, motor3, motor4);

	USART1_printf("\r\n[out:%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f]\r\n",
		outer_read_pitch, outer_read_roll, outer_read_yaw,
		outer_rx_yaw, outer_rx_pitch, outer_rx_roll, outer_rx_updown,
		outer_pitch_lasterror, outer_roll_lasterror,
		outer_pitch_integrator, outer_roll_integrator,
		outer_pitch_output, outer_roll_output);

	USART1_printf("\r\n[pid:%.2f;%.2f;%.2f;%.2f;%.2f;%.2f]\r\n",
		inner_kp, inner_ki, inner_kd,
		outer_kp, outer_ki, outer_kd);

	USART1_printf("\r\n[time:%d;%d;%d;%d;%d;%d\r\n",
		kalman_time, send_time, receive_time,
		inner_loop_time, motor_time,
		outer_loop_time);

	INT32U tick2 = OSTimeGet();
	send_time = tick2 - tick1;
}


void ReceiveTask() {
	INT32U tick1 = OSTimeGet();

	strcpy(data, USART1_RX_BUF);

	char *p = strtok(data, ";");
	int k = 0;
	while (p != NULL) {
		switch (k) {
			case 0:
				inner_kp = atof(p);
				break;
			case 1:
				inner_ki = atof(p);
				break;
			case 2:
				inner_kd = atof(p);
				break;
			case 3:
				outer_kp = atof(p);
				break;
			case 4:
				outer_ki = atof(p);
				break;
			case 5:
				outer_kd = atof(p);
				break;
			default:
				break;
		}
		p = strtok(NULL, ";");
		k++;
	}
	if (k == 6) {
		inner_pitch_integrator = 0.0f;
		inner_roll_integrator = 0.0f;
		outer_pitch_integrator = 0.0f;
		outer_roll_integrator = 0.0f;
	}

	


	INT32U tick2 = OSTimeGet();
	receive_time = tick2 - tick1;

}
void OLEDTask()
{
	int mode = 2;

	INT32U tick1 = OSTimeGet();

	if (mode == 0) // GY86 Mode
	{
		OLED_ShowString(1, 1, "T:");
		OLED_ShowNum(1, 3, OSTime, 5);

		OLED_ShowString(2, 1, "X");
		OLED_ShowSignedNum(2, 2, accx_read, 4);
		OLED_ShowSignedNum(2, 7, gyrox_read, 4);
		OLED_ShowSignedNum(2, 12, magx_read, 4);

		OLED_ShowString(3, 1, "Y");
		OLED_ShowSignedNum(3, 2, accy_read, 4);
		OLED_ShowSignedNum(3, 7, gyroy_read, 4);
		OLED_ShowSignedNum(3, 12, magy_read, 4);

		OLED_ShowString(4, 1, "Z");
		OLED_ShowSignedNum(4, 2, accz_read, 4);
		OLED_ShowSignedNum(4, 7, gyroz_read, 4);
		OLED_ShowSignedNum(4, 12, magz_read, 4);
	}
	else if (mode == 1) // Motor Mode
	{
		OLED_ShowString(1, 1, "T:");
		OLED_ShowNum(1, 3, OSTime, 5);

		OLED_ShowNum(2, 1, ppm_CCR1data[1], 4);
		OLED_ShowNum(2, 6, ppm_CCR1data[2], 4);
		OLED_ShowNum(3, 1, ppm_CCR1data[3], 4);
		OLED_ShowNum(3, 6, ppm_CCR1data[4], 4);
		OLED_ShowNum(4, 1, ppm_CCR1data[5], 4);
		OLED_ShowNum(4, 6, ppm_CCR1data[6], 4);
	}
	else if (mode == 2) // Time Mode
	{
		OLED_ShowString(1, 1, "G:");
		OLED_ShowNum(1, 3, gy86_time, 4);
		OLED_ShowString(1, 9, "K:");
		OLED_ShowNum(1, 11, kalman_time, 4);
		OLED_ShowString(2, 1, "S:");
		OLED_ShowNum(2, 3, send_time, 4);
		OLED_ShowString(2, 9, "O:");
		OLED_ShowNum(2, 11, oled_time, 4);

		OLED_ShowString(3, 1, "I:");
		OLED_ShowNum(3, 3, inner_loop_time, 4);
		OLED_ShowString(3, 9, "M:");
		OLED_ShowNum(3, 11, motor_time, 4);

		OLED_ShowString(4, 1, "O:");
		OLED_ShowNum(4, 3, outer_loop_time, 4);
		OLED_ShowNum(4, 9, OSTime, 5);
		
	}

	INT32U tick2 = OSTimeGet();
	oled_time = tick2 - tick1;
}

/*
*********************************************************************************************************
*                                             500Hz TASKS
*
* Includes:
* 0. Task500HZ
* 1.InnerLoopTask
* 2.MotorTask
*********************************************************************************************************
*/
void Task500HZ(void *p_arg)
{
	while (1)
	{
		InnerLoopTask();
		MotorTask();
		OSTimeDlyHMSM(0, 0, 0, Task500HZDelay);
	}
}


void InnerLoopTask()
{
	INT32U tick1 = OSTimeGet();

	// TODO: Inner Loop
	inner_gyro_roll = (float) imu_data[0] * 4000.0f / 65536.0f;
	inner_gyro_pitch = (float) imu_data[1] * 4000.0f / 65536.0f;

	float diff_roll = outer_roll_output - inner_gyro_roll;
	float diff_pitch = outer_pitch_output - inner_gyro_pitch;

	if(inner_pitch_integrator + diff_pitch < 2000 && inner_pitch_integrator + diff_pitch > -2000) {
		inner_pitch_integrator += diff_pitch;
	} else {
		inner_pitch_integrator = inner_pitch_integrator;
	}

	if(inner_roll_integrator + diff_roll < 2000 && inner_roll_integrator + diff_roll > -2000) {
		inner_roll_integrator += diff_roll;
	} else {
		inner_roll_integrator = inner_roll_integrator;
	}

	inner_pitch_output = inner_kp * diff_pitch + inner_ki * inner_pitch_integrator + inner_kd * (diff_pitch - inner_pitch_lasterror);
	inner_roll_output = inner_kp * diff_roll + inner_ki * inner_roll_integrator + inner_kd * (diff_roll - inner_roll_lasterror);

	inner_pitch_lasterror = diff_pitch;
	inner_roll_lasterror = diff_roll;

	motor1 = outer_rx_updown - inner_pitch_output + inner_roll_output;
	motor2 = outer_rx_updown - inner_pitch_output - inner_roll_output;
	motor3 = outer_rx_updown + inner_pitch_output - inner_roll_output;
	motor4 = outer_rx_updown + inner_pitch_output + inner_roll_output;

	

	INT32U tick2 = OSTimeGet();
	inner_loop_time = tick2 - tick1;
}

void MotorTask()
{
	INT32U tick1 = OSTimeGet();
	PWM_output(motor1, motor2, motor3, motor4);

	INT32U tick2 = OSTimeGet();
	motor_time = tick2 - tick1;
}

/*
*********************************************************************************************************
*                                             250Hz TASKS
*
* Includes:
* 0. Task250HZ
* 1. OuterLoopTask
* 2. BlinkTask
*********************************************************************************************************
*/
void Task250HZ(void *p_arg)
{
	while (1)
	{
		OuterLoopTask();
		BlinkTask();
		OSTimeDlyHMSM(0, 0, 0, Task250HZDelay);
	}
}





void OuterLoopTask()
{
	INT32U tick1 = OSTimeGet();

	if (ppm_CCR1data[1] < 1000)
	{
		outer_rx_yaw = 0.0f;
		outer_rx_pitch = 0.0f;
		outer_rx_roll = 0.0f;
		outer_rx_updown = 0.0f;
	}
	else
	{
		outer_rx_yaw = (float) ((ppm_CCR1data[4] - 1515.0f)/505.0f *100.0f); // >1515: yaw right; <1515: yaw left
		outer_rx_pitch = - (float) ((ppm_CCR1data[2] - 1523.0f)/503.0f *45.0f); // >1523: pitch forward; <1523: pitch backward
		outer_rx_roll = (float) ((ppm_CCR1data[1] -1523.0f)/503.0f *45.0f); // >1523: roll right; <1523: roll left
		outer_rx_updown = (float)((ppm_CCR1data[3] - 1017.0f)/1009.0f *100.0f);
	}
	// outer_rx_yaw = (float) ((ppm_CCR1data[4] - 1515.0f)/505.0f *100.0f); // >1515: yaw right; <1515: yaw left
	// outer_rx_pitch = - (float) ((ppm_CCR1data[2] - 1523.0f)/503.0f *45.0f); // >1523: pitch forward; <1523: pitch backward
	// outer_rx_roll = (float) ((ppm_CCR1data[1] -1523.0f)/503.0f *45.0f); // >1523: roll right; <1523: roll left
	// outer_rx_updown = (float)((ppm_CCR1data[3] - 1017.0f)/1009.0f *100.0f);

	outer_read_roll = (float) ((ANO_data1.data[0] + (ANO_data1.data[1] << 8))/100.0f);
	outer_read_pitch = (float) ((ANO_data1.data[2] + (ANO_data1.data[3] << 8))/100.0f);
	outer_read_yaw = (float) ((ANO_data1.data[4] + (ANO_data1.data[5] << 8))/100.0f);

	// pitch forward: 0~-180; pitch backward: 0~180
	// roll left: 0~-180; roll right: 0~180
	// yaw left: 0~-180; yaw right: 0~180

	float diff_roll = outer_rx_roll - outer_read_roll;
	float diff_pitch = outer_rx_pitch - outer_read_pitch;
	float diff_yaw = outer_rx_yaw - outer_read_yaw;

	if((outer_pitch_integrator + diff_pitch) < 2000 && (outer_pitch_integrator + diff_pitch) > -2000) {
		outer_pitch_integrator += diff_pitch;
	} else {
		outer_pitch_integrator = outer_pitch_integrator;
	}
	
	if((outer_roll_integrator + diff_roll) < 2000 && (outer_roll_integrator + diff_roll) > -2000) {
		outer_roll_integrator += diff_roll;
	} else {
		outer_roll_integrator = outer_roll_integrator;
	}

	outer_pitch_output = outer_kp * diff_pitch + outer_ki * outer_pitch_integrator + outer_kd * (diff_pitch - outer_pitch_lasterror);
	outer_roll_output = outer_kp * diff_roll + outer_ki * outer_roll_integrator + outer_kd * (diff_roll - outer_roll_lasterror);

	outer_pitch_output = outer_rx_pitch;
	outer_roll_output = outer_rx_roll;

	outer_pitch_lasterror = diff_pitch;
	outer_roll_lasterror = diff_roll;

	

	INT32U tick2 = OSTimeGet();
	outer_loop_time = tick2 - tick1;
}

void BlinkTask()
{
	BOOLEAN flag = 0;

	INT32U tick1 = OSTimeGet();

	if (flag == 0)
		LED_On();
	else
		LED_Off();

	flag = !flag;

	INT32U tick2 = OSTimeGet();
	blink_time = tick2 - tick1;
}