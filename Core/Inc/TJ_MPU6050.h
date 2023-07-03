/*
 * TJ_MPU6050.h
 *
 *  Created on: Dec 12, 2022
 *      Author: huynh
 */

#ifndef INC_TJ_MPU6050_H_
#define INC_TJ_MPU6050_H_

//Header Files
#include "stm32f1xx_hal.h"  //dpending on your board
#include <string.h>
#include <stdbool.h>	//Boolean
#include <math.h>			//Pow()

//Define Registers
#define WHO_AM_I_REG			0x75
#define MPU_ADDR					0x68
#define PWR_MAGT_1_REG		0x6B
#define CONFIG_REG				0x1A
#define GYRO_CONFIG_REG		0x1B
#define ACCEL_CONFIG_REG	0x1C
#define SMPLRT_DIV_REG		0x19
#define INT_STATUS_REG		0x3A
#define ACCEL_XOUT_H_REG	0x3B
#define TEMP_OUT_H_REG		0x41
#define GYRO_XOUT_H_REG		0x43
#define FIFO_EN_REG 			0x23
#define INT_ENABLE_REG 		0x38
#define I2CMACO_REG 			0x23
#define USER_CNT_REG			0x6A
#define FIFO_COUNTH_REG 	0x72
#define FIFO_R_W_REG 			0x74


//TypeDefs and Enums

//0- measure offset
typedef struct{
	int AX;
	int AY;
	int AZ;
	int GX;
	int GY;
	int GZ;
	int MX;
	int MY;
	int MZ;
}Offset;
//1- MPU Configuration
typedef struct
{
	uint8_t ClockSource;
	uint8_t Gyro_Full_Scale;
	uint8_t Accel_Full_Scale;
	uint8_t CONFIG_DLPF;
	bool 		Sleep_Mode_Bit;

}MPU_ConfigTypeDef;
//2- Clock Source ENUM
enum PM_CLKSEL_ENUM
{
	Internal_8MHz 	= 0x00,
	X_Axis_Ref			= 0x01,
	Y_Axis_Ref			= 0x02,
	Z_Axis_Ref			= 0x03,
	Ext_32_768KHz		= 0x04,
	Ext_19_2MHz			= 0x05,
	TIM_GENT_INREST	= 0x07
};
//3- Gyro Full Scale Range ENUM (deg/sec)
enum gyro_FullScale_ENUM
{
	FS_SEL_250 	= 0x00,
	FS_SEL_500 	= 0x01,
	FS_SEL_1000 = 0x02,
	FS_SEL_2000	= 0x03
};
//4- Accelerometer Full Scale Range ENUM (1g = 9.81m/s2)
enum accel_FullScale_ENUM
{
	AFS_SEL_2g	= 0x00,
	AFS_SEL_4g  = 0x01,
	AFS_SEL_8g  = 0x02,
	AFS_SEL_16g = 0x03
};
//5- Digital Low Pass Filter ENUM
enum DLPF_CFG_ENUM
{
	DLPF_260A_256G_Hz = 0x00,
	DLPF_184A_188G_Hz = 0x01,
	DLPF_94A_98G_Hz 	= 0x02,
	DLPF_44A_42G_Hz 	= 0x03,
	DLPF_21A_20G_Hz 	= 0x04,
	DLPF_10_Hz 				= 0x05,
	DLPF_5_Hz 				= 0x06
};
//6- e external Frame Synchronization ENUM
enum EXT_SYNC_SET_ENUM
{
	input_Disable = 0x00,
	TEMP_OUT_L		= 0x01,
	GYRO_XOUT_L		= 0x02,
	GYRO_YOUT_L		= 0x03,
	GYRO_ZOUT_L		= 0x04,
	ACCEL_XOUT_L	= 0x05,
	ACCEL_YOUT_L	= 0x06,
	ACCEL_ZOUT_L	= 0x07
};

//7. Raw data typedef
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}RawData_Def;

//8. Scaled data typedef
typedef struct
{
	float x;
	float y;
	float z;
}ScaledData_Def;
//////////////
typedef struct {
	float axg;
	float ayg;
	float azg;
	float gxrs;
	float gyrs;
	float gzrs;
	float mx;
	float my;
	float mz;
}updateQuater;

typedef struct {
	volatile float q0;
	volatile float q1;
	volatile float q2;
	volatile float q3;
}q_volatile;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}rawpitchyaw;

typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float Kp_roll;
	float Ki_roll;
	float Kd_roll;
	float Kp_pitch;
	float Ki_pitch;
	float Kd_pitch;
	float Kp_yaw;
	float Ki_yaw;
	float Kd_yaw;
	float error;
	float roll_error;
	float pitch_error;
	float yaw_error;
	float previous_error;
	float previous_error_roll;
	float previous_error_pitch;
	float previous_error_yaw;
	double timePrev;
	double elapsedTime;
	float pid_p_roll;
	float pid_i_roll;
	float pid_d_roll;
	float pid_p_pitch;
	float pid_i_pitch;
	float pid_d_pitch;
	float pid_p_yaw;
	float pid_i_yaw;
	float pid_d_yaw;
	float pid_p;
	float pid_i;
	float pid_d;
	double time;
	float PID_total;
	float PID_total_roll;
	float PID_total_pitch;
	float PID_total_yaw;
	float pwmLeft;
	float pwmRight;
	float pwm_RF_2;
	float pwm_RB_3;
	float pwm_LB_4;
	float pwm_LF_1;
	int count_error_pid;
}PID_param;

//typedef struct {
//	volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
//}integralFB;

//Function Prototype
//0- measure offset
void Calibration(Offset *offset, int *status);
//1- i2c Handler
void MPU6050_Init(I2C_HandleTypeDef *I2Chnd);
//2- i2c Read
void I2C_Read(uint8_t ADDR, uint8_t *i2cBuf, uint8_t NofData);
//3- i2c Write 8 Bit
void I2C_Write8(uint8_t ADDR, uint8_t data);
//4- MPU6050 Initialaztion Configuration
void MPU6050_Config(MPU_ConfigTypeDef *config);
//5- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void);
//6- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue);
//7- External Frame Sync.
uint8_t MPU6050_Get_FSYNC(void);
//8- Set External Frame Sync.
void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync);
//9- Get Accel Raw Data
void MPU6050_Get_Accel_RawData(RawData_Def *rawDef);//************
//10- Get Accel scaled data
void MPU6050_Get_Accel_Scale(ScaledData_Def *scaledDef);//***********
//11- Get Accel calibrated data
void MPU6050_Get_Accel_Cali(ScaledData_Def *CaliDef);
//12- Get Gyro Raw Data
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef);
//13- Get Gyro scaled data
void MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef);
//14- Accel Calibration
void _Accel_Cali(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);


/////////////////
// set up
// 1.
//void MPU6050SelfTest(float *destination);
//loop
//1.
void updateQuaternion(RawData_Def myAccelRaw, RawData_Def myGyroRaw, RawData_Def myMagRaw,Offset offset, updateQuater *upQua);
//2.
void MahonyAHRSupdateIMU(updateQuater upQua, float sampleFreq, q_volatile *qVol);
void MahonyAHRSupdate(updateQuater upQua, float sampleFreq, q_volatile *qVol);

// 3.
void mpu6050_getRollPitchYaw(q_volatile qVol, rawpitchyaw *rpy);

void MPU6050_Get_Accel_RawData_to_CaliBration(RawData_Def *rawDef);
void MPU6050_Get_Gyro_RawData_to_CaliBration(RawData_Def *rawDef);
void MPU6050_Get_Mag_RawData(RawData_Def *rawDef);

// PID
void PID(rawpitchyaw *angle, PID_param *pid, int rop, int Init_CCR, float desired_roll_angle, float desired_pitch_angle, float desired_yaw_angle);

#endif /* INC_TJ_MPU6050_H_ */
