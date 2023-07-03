/*
 * TJ_MPU6050.c
 *
 *  Created on: Dec 12, 2022
 *      Author: huynh
 */


/*
library name: 	MPU6050 6 axis module
written by: 		T.Jaber
Date Written: 	25 Mar 2019
Last Modified: 	20 April 2019 by Mohamed Yaqoob
Description: 		MPU6050 Module Basic Functions Device Driver library that use HAL libraries.
References:
								- MPU6050 Registers map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
								- Jeff Rowberg MPU6050 library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

* Copyright (C) 2019 - T. Jaber
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.

*/

//references:
//- PID for roll & pitch:
//		https://forum.arduino.cc/t/mpu6050-pid-with-dmp/856663
//		https://forum.arduino.cc/t/need-help-with-quadcopter-pid-and-pwm/437796




//Header files
#include "TJ_MPU6050.h"

//Library Variable
//1- I2C Handle
static I2C_HandleTypeDef i2cHandler;
//2- Accel & Gyro Scaling Factor
static float accelScalingFactor, gyroScalingFactor;
//3- Bias varaibles
static float A_X_Bias = 0.0f;
static float A_Y_Bias = 0.0f;
static float A_Z_Bias = 0.0f;

static int16_t GyroRW[3];
static int16_t MagRW[3];

// For Quaternion function
#define twoKpDef  1.0f//(2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  0.01f//(2.0f * 0.0f) // 2 * integral gain
float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;



//Fucntion Definitions
//0- measure offset
void Calibration(Offset *offset, int *status){
	*status = 0;
	RawData_Def myAccelRaw, myGyroRaw, myMagRaw;
		for(int i = 0 ; i < 500 ; i++)
		{
			MPU6050_Get_Accel_RawData_to_CaliBration(&myAccelRaw);
			MPU6050_Get_Gyro_RawData_to_CaliBration(&myGyroRaw);
			MPU6050_Get_Mag_RawData(&myMagRaw);
			HAL_Delay(10);
			*status = *status + 1;
			//printf("%i\n\r",*status);
		    // Sum data
			offset->AX += myAccelRaw.x;
			offset->AY += myAccelRaw.y;
			offset->AZ += myAccelRaw.z;
			offset->GX += myGyroRaw.x;
			offset->GY += myGyroRaw.y;
			offset->GZ += myGyroRaw.z;
			offset->MX += myMagRaw.x;
			offset->MY += myMagRaw.y;
			offset->MZ += myMagRaw.z;
		}
		  // Average Data
		offset->AX /= 500;
		offset->AY /= 500;
		offset->AZ /= 500;
		offset->GX /= 500;
		offset->GY /= 500;
		offset->GZ /= 500;
		offset->MX /= 500;
		offset->MY /= 500;
		offset->MZ /= 500;
}
//1- i2c Handler
void MPU6050_Init(I2C_HandleTypeDef *I2Chnd)
{
	//Copy I2C CubeMX handle to local library
	memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));
}

//2- i2c Read
void I2C_Read(uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)
{
	uint8_t i2cBuf[2];
	uint8_t MPUADDR;
	//Need to Shift address to make it proper to i2c operation
	MPUADDR = (MPU_ADDR<<1);
	i2cBuf[0] = ADDR;
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cBuf, 1, 10);
	HAL_I2C_Master_Receive(&i2cHandler, MPUADDR, i2cBif, NofData, 100);
}

//3- i2c Write
void I2C_Write8(uint8_t ADDR, uint8_t data)
{
	uint8_t i2cData[2];
	i2cData[0] = ADDR;
	i2cData[1] = data;
	uint8_t MPUADDR = (MPU_ADDR<<1);
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cData, 2,100);
}

//4- MPU6050 Initialaztion Configuration
void MPU6050_Config(MPU_ConfigTypeDef *config)
{
	uint8_t Buffer = 0;
	//Clock Source
	//Reset Device
	I2C_Write8(PWR_MAGT_1_REG, 0x80);
	HAL_Delay(100);
	Buffer = config ->ClockSource & 0x07; //change the 7th bits of register
	Buffer |= (config ->Sleep_Mode_Bit << 6) &0x40; // change only the 7th bit in the register
	I2C_Write8(PWR_MAGT_1_REG, Buffer);
	HAL_Delay(100); // should wait 10ms after changeing the clock setting.

	//Set the Digital Low Pass Filter
	Buffer = 0;
	Buffer = config->CONFIG_DLPF & 0x07;
	I2C_Write8(CONFIG_REG, Buffer);

	//Select the Gyroscope Full Scale Range
	Buffer = 0;
	Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
	I2C_Write8(GYRO_CONFIG_REG, Buffer);

	//Select the Accelerometer Full Scale Range
	Buffer = 0;
	Buffer = (config->Accel_Full_Scale << 3) & 0x18;
	I2C_Write8(ACCEL_CONFIG_REG, Buffer);
	//Set SRD To Default
	MPU6050_Set_SMPRT_DIV(0x04);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Buffer = 0x00;
//	I2C_Write8(0x6A, Buffer);	// Disable i2c master mode
//	HAL_Delay(10);
//
//	Buffer = 0x02;
//	I2C_Write8(0x37, Buffer);	// Enable i2c master bypass mode
//	HAL_Delay(10);
//
//	Buffer = 0x18;
//    HAL_I2C_Mem_Write(&i2cHandler, 0x1E << 1, 0x00, 1, &Buffer, 1, 100);    // Sample rate = 75Hz
//    HAL_Delay(10);
//
//    Buffer = 0x60;
//    HAL_I2C_Mem_Write(&i2cHandler, 0x1E << 1, 0x01, 1, &Buffer, 1, 100);    // Full scale = +/- 2.5 Gauss
//    HAL_Delay(10);
//
//    Buffer = 0x00;
//    HAL_I2C_Mem_Write(&i2cHandler, 0x1E << 1, 0x02, 1, &Buffer, 1, 100);        // Continuous measurement mode
//    HAL_Delay(10);
//
//    Buffer = 0x00;
//    HAL_I2C_Mem_Write(&i2cHandler, 0xD0, 0x37, 1, &Buffer, 1, 100);          // Disable i2c master bypass mode
//    HAL_Delay(10);
//
//    Buffer = 0x22;
//    HAL_I2C_Mem_Write(&i2cHandler, 0xD0, 0x6A, 1, &Buffer, 1, 100);        // Enable i2c master mode
//    HAL_Delay(10);
//
//    //Configure the MPU6050 to automatically read the magnetometer
//
//    Buffer = 0x1E | 0x80;
//    HAL_I2C_Mem_Write(&i2cHandler, 0xD0, 0x25, 1, &Buffer, 1, 100);        // Access Slave into read mode
//    HAL_Delay(10);
//
//    Buffer = 0x03;
//    HAL_I2C_Mem_Write(&i2cHandler, 0xD0, 0x26, 1, &Buffer, 1, 100);         // Slave REG for reading to take place
//    HAL_Delay(10);
//
//    Buffer = 0x80 | 0x06;
//    HAL_I2C_Mem_Write(&i2cHandler, 0xD0, 0x27, 1, &Buffer, 1, 100);        // Number of data bytes
//    HAL_Delay(10);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//Accelerometer Scaling Factor, Set the Accelerometer and Gyroscope Scaling Factor
	switch (config->Accel_Full_Scale)
	{
		case AFS_SEL_2g:
			accelScalingFactor = (2000.0f/32768.0f);
			break;

		case AFS_SEL_4g:
			accelScalingFactor = (4000.0f/32768.0f);
				break;

		case AFS_SEL_8g:
			accelScalingFactor = (8000.0f/32768.0f);
			break;

		case AFS_SEL_16g:
			accelScalingFactor = (16000.0f/32768.0f);
			break;

		default:
			break;
	}
	//Gyroscope Scaling Factor
	switch (config->Gyro_Full_Scale)
	{
		case FS_SEL_250:
			gyroScalingFactor = 250.0f/32768.0f;
			break;

		case FS_SEL_500:
				gyroScalingFactor = 500.0f/32768.0f;
				break;

		case FS_SEL_1000:
			gyroScalingFactor = 1000.0f/32768.0f;
			break;

		case FS_SEL_2000:
			gyroScalingFactor = 2000.0f/32768.0f;
			break;

		default:
			break;
	}

}

//5- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void)
{
	uint8_t Buffer = 0;

	I2C_Read(SMPLRT_DIV_REG, &Buffer, 1);
	return Buffer;
}

//6- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue)
{
	I2C_Write8(SMPLRT_DIV_REG, SMPRTvalue);
}

////7- Get External Frame Sync.
//uint8_t MPU6050_Get_FSYNC(void)
//{
//	uint8_t Buffer = 0;
//
//	I2C_Read(CONFIG_REG, &Buffer, 1);
//	Buffer &= 0x38;
//	return (Buffer>>3);
//}
//
////8- Set External Frame Sync.
//void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync)
//{
//	uint8_t Buffer = 0;
//	I2C_Read(CONFIG_REG, &Buffer,1);
//	Buffer &= ~0x38;
//
//	Buffer |= (ext_Sync <<3);
//	I2C_Write8(CONFIG_REG, Buffer);
//
//}

//9- Get Accel Raw Data
void MPU6050_Get_Accel_RawData(RawData_Def *rawDef)
{
	uint8_t i2cBuf[2];
	uint8_t AcceArr[6], GyroArr[6];

	I2C_Read(INT_STATUS_REG, &i2cBuf[1],1);
	if((i2cBuf[1]&&0x01))
	{
		I2C_Read(ACCEL_XOUT_H_REG, AcceArr,6);

		//Accel Raw Data
		rawDef->x = ((AcceArr[0]<<8) + AcceArr[1]); // x-Axis
		rawDef->y = ((AcceArr[2]<<8) + AcceArr[3]); // y-Axis
		rawDef->z = ((AcceArr[4]<<8) + AcceArr[5]); // z-Axis
		//Gyro Raw Data
		I2C_Read(GYRO_XOUT_H_REG, GyroArr,12);
		GyroRW[0] = ((GyroArr[0]<<8) + GyroArr[1]);
		GyroRW[1] = (GyroArr[2]<<8) + GyroArr[3];
		GyroRW[2] = ((GyroArr[4]<<8) + GyroArr[5]);
	}
}

//10- Get Accel scaled data (g unit of gravity, 1g = 9.81m/s2)
void MPU6050_Get_Accel_Scale(ScaledData_Def *scaledDef)
{

	RawData_Def AccelRData;
	MPU6050_Get_Accel_RawData(&AccelRData);

	//Accel Scale data
	scaledDef->x = ((AccelRData.x+0.0f)*accelScalingFactor);
	scaledDef->y = ((AccelRData.y+0.0f)*accelScalingFactor);
	scaledDef->z = ((AccelRData.z+0.0f)*accelScalingFactor);
	HAL_Delay(100);
}

//11- Get Accel calibrated data
void MPU6050_Get_Accel_Cali(ScaledData_Def *CaliDef)
{
	ScaledData_Def AccelScaled;
	MPU6050_Get_Accel_Scale(&AccelScaled);

	//Accel Scale data
	CaliDef->x = (AccelScaled.x) - A_X_Bias; // x-Axis
	CaliDef->y = (AccelScaled.y) - A_Y_Bias;// y-Axis
	CaliDef->z = (AccelScaled.z) - A_Z_Bias;// z-Axis
}
//12- Get Gyro Raw Data
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef)
{

	//Accel Raw Data
	rawDef->x = GyroRW[0];
	rawDef->y = GyroRW[1];
	rawDef->z = GyroRW[2];

}

void MPU6050_Get_Mag_RawData(RawData_Def *rawDef)
{
	uint8_t MagArr[6];
	I2C_Read(0x49, MagArr,6);
	MagRW[0] =  ((MagArr[0]<<8) + MagArr[1]);
	MagRW[1] =	((MagArr[2]<<8) + MagArr[3]);
	MagRW[2] =	((MagArr[4]<<8) + MagArr[5]);
	//Accel Raw Data
	rawDef->x = MagRW[0];
	rawDef->y = MagRW[1];
	rawDef->z = MagRW[2];

}

//13- Get Gyro scaled data
void MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef)
{
	RawData_Def myGyroRaw;
	MPU6050_Get_Gyro_RawData(&myGyroRaw);

	//Gyro Scale data
	scaledDef->x = (myGyroRaw.x)*gyroScalingFactor; // x-Axis
	scaledDef->y = (myGyroRaw.y)*gyroScalingFactor; // y-Axis
	scaledDef->z = (myGyroRaw.z)*gyroScalingFactor; // z-Axis
}

//14- Accel Calibration
void _Accel_Cali(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
	//1* X-Axis calibrate
	A_X_Bias		= (x_max + x_min)/2.0f;

	//2* Y-Axis calibrate
	A_Y_Bias		= (y_max + y_min)/2.0f;

	//3* Z-Axis calibrate
	A_Z_Bias		= (z_max + z_min)/2.0f;
}



//////////////////
// set up
// 1.
//void MPU6050SelfTest(float *destination){
//   uint8_t rawData[4];
//   uint8_t selfTest[6];
//   float factoryTrim[6];
//
//   // Configure the accelerometer for self-test
////   writeByte(0x68, 0x1C, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
////   writeByte(0x68, 0x1C,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
//   HAL_Delay(250);  // Delay a while to let the device execute the self-test
////   rawData[0] = readByte(0x68, 0x0D); // X-axis self-test results -----------> khac code arduino
////   rawData[1] = readByte(0x68, 0x0E); // Y-axis self-test results
////   rawData[2] = readByte(0x68, 0x0F); // Z-axis self-test results
////   rawData[3] = readByte(0x68, 0x10); // Mixed-axis self-test results
//   // Extract the acceleration test results first
//   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
//   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
//   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer
//   // Extract the gyration test results first
//   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
//   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
//   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
//   // Process results to allow final comparison with factory set values
//   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
//   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
//   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
//   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
//   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
//   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation
//
// //  Output self-test results and factory trim calculation if desired
// //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
// //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
// //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
// //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);
//
// // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
// // To get to percent, must multiply by 100 and subtract result from 100
//   for (int i = 0; i < 6; i++) {
//	 destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
//   }
//}
// A.
void updateQuaternion(RawData_Def myAccelRaw, RawData_Def myGyroRaw, RawData_Def myMagRaw, Offset offset, updateQuater *upQua){
//	  upQua->axg = (float)(myAccelRaw.x - 71) / 4096.0; // offset.AX
//	  upQua->ayg = (float)(myAccelRaw.y - 13) / 4096.0; // offset.AY
//	  upQua->azg = (float)(myAccelRaw.z - (-491)) / 4096.0; // offset.AZ
//	  upQua->gxrs = (float)((float)(myGyroRaw.x - (-953)) / 16.384 * 0.01745329); //degree to radians - offset.GX
//	  upQua->gyrs = (float)((float)(myGyroRaw.y - (-64)) / 16.384 * 0.01745329); //degree to radians - offset.GY
//	  upQua->gzrs = (float)((float)(myGyroRaw.z - 6) / 16.384 * 0.01745329); //degree to radians - offset.GZ

	// update: 11/3/2023
	  upQua->axg = (float)(myAccelRaw.x - offset.AX) / 4096.0; // offset.AX //35
	  upQua->ayg = (float)(myAccelRaw.y - offset.AY) / 4096.0; // offset.AY
	  upQua->azg = (float)(myAccelRaw.z - offset.AZ) / 4096.0; // offset.AZ
	  upQua->gxrs = (float)((float)(myGyroRaw.x - offset.GX) / 16.384 * 0.01745329); //degree to radians - offset.GX
	  upQua->gyrs = (float)((float)(myGyroRaw.y - offset.GY) / 16.384 * 0.01745329); //degree to radians - offset.GY
	  upQua->gzrs = (float)((float)(myGyroRaw.z - offset.GZ) / 16.384 * 0.01745329); //degree to radians - offset.GZ
	  upQua->mx = (float)(myMagRaw.x - offset.MX) / 660.0; // offset.AX
	  upQua->my = (float)(myMagRaw.y - offset.MY) / 660.0; // offset.AY
	  upQua->mz = (float)(myMagRaw.z - offset.MZ) / 660.0; // offset.AZ
//	  upQua->gxrs = 20;
//	  HAL_Delay(2000);
//	  upQua->gxrs = 20; //(myGyroRaw.x - offset.GX);


	  // Degree to Radians Pi / 180 = 0.01745329 0.01745329251994329576923690768489
}
// B. -------------------------------> ham nay can check -> vĂ  .
void MahonyAHRSupdateIMU(updateQuater upQua, float sampleFreq, q_volatile *qVol){
	  float norm;
	  float halfvx, halfvy, halfvz;
	  float halfex, halfey, halfez;
	  float qa, qb, qc;

	  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	  if(!((upQua.axg == 0.0f) && (upQua.ayg == 0.0f) && (upQua.azg == 0.0f))) {

	    // Normalise accelerometer measurement
	    norm = sqrt(upQua.axg * upQua.axg + upQua.ayg * upQua.ayg + upQua.azg * upQua.azg);
	    upQua.axg /= norm;
	    upQua.ayg /= norm;
	    upQua.azg /= norm;

	    // Estimated direction of gravity and vector perpendicular to magnetic flux
	    halfvx = qVol->q1 * qVol->q3 - qVol->q0 * qVol->q2;
	    halfvy = qVol->q0 * qVol->q1 + qVol->q2 * qVol->q3;
	    halfvz = qVol->q0 * qVol->q0 - 0.5f + qVol->q3 * qVol->q3;

	    // Error is sum of cross product between estimated and measured direction of gravity
	    halfex = (upQua.ayg * halfvz - upQua.azg * halfvy);
	    halfey = (upQua.azg * halfvx - upQua.axg * halfvz);
	    halfez = (upQua.axg * halfvy - upQua.ayg * halfvx);

	    // Compute and apply integral feedback if enabled
	    if(twoKi > 0.0f) {
	      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
	      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
	      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
	      upQua.gxrs += integralFBx;  // apply integral feedback
	      upQua.gyrs += integralFBy;
	      upQua.gzrs += integralFBz;
	    }
	    else {
	      integralFBx = 0.0f; // prevent integral windup
	      integralFBy = 0.0f;
	      integralFBz = 0.0f;
	    }

	    // Apply proportional feedback
	    upQua.gxrs += twoKp * halfex;
	    upQua.gyrs += twoKp * halfey;
	    upQua.gzrs += twoKp * halfez;
	  }

	  // Integrate rate of change of quaternion
	  upQua.gxrs *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
	  upQua.gyrs *= (0.5f * (1.0f / sampleFreq));
	  upQua.gzrs *= (0.5f * (1.0f / sampleFreq));
	  qa = qVol->q0;
	  qb = qVol->q1;
	  qc = qVol->q2;
	  qVol->q0 += (-qb * upQua.gxrs - qc * upQua.gyrs - qVol->q3 * upQua.gzrs);
	  qVol->q1 += (qa * upQua.gxrs + qc * upQua.gzrs - qVol->q3 * upQua.gyrs);
	  qVol->q2 += (qa * upQua.gyrs - qb * upQua.gzrs + qVol->q3 * upQua.gxrs);
	  qVol->q3 += (qa * upQua.gzrs + qb * upQua.gyrs - qc * upQua.gxrs);

	  // Normalise quaternion
	  norm = sqrt(qVol->q0 * qVol->q0 + qVol->q1 * qVol->q1 + qVol->q2 * qVol->q2 + qVol->q3 * qVol->q3);
	  qVol->q0 /= norm;
	  qVol->q1 /= norm;
	  qVol->q2 /= norm;
	  qVol->q3 /= norm;
}
/// c.
void mpu6050_getRollPitchYaw(q_volatile qVol, rawpitchyaw *rpy){	//57.29577951 = 180/pi
	  rpy->yaw   = -atan2(2.0f * (qVol.q1 * qVol.q2 + qVol.q0 * qVol.q3), qVol.q0 * qVol.q0 + qVol.q1 * qVol.q1 - qVol.q2 * qVol.q2 - qVol.q3 * qVol.q3) * 57.29577951;
	  rpy->pitch = asin(2.0f * (qVol.q1 * qVol.q3 - qVol.q0 * qVol.q2)) * 57.29577951;
	  rpy->roll  = atan2(2.0f * (qVol.q0 * qVol.q1 + qVol.q2 * qVol.q3), qVol.q0 * qVol.q0 - qVol.q1 * qVol.q1 - qVol.q2 * qVol.q2 + qVol.q3 * qVol.q3) * 57.29577951;
}

///////
void MPU6050_Get_Accel_RawData_to_CaliBration(RawData_Def *rawDef)
{
	uint8_t i2cBuf[2];
	uint8_t AcceArr[6], GyroArr[6];

	I2C_Read(INT_STATUS_REG, &i2cBuf[1],1);
	if((i2cBuf[1]&&0x01))
	{
		I2C_Read(ACCEL_XOUT_H_REG, AcceArr,6);

		//Accel Raw Data
		rawDef->x = ((AcceArr[0]<<8) + AcceArr[1]); // x-Axis
		rawDef->y = ((AcceArr[2]<<8) + AcceArr[3]); // y-Axis
		rawDef->z = ((AcceArr[4]<<8) + AcceArr[5]) - 4096; // z-Axis
		//Gyro Raw Data
		I2C_Read(GYRO_XOUT_H_REG, GyroArr,6);
		GyroRW[0] = ((GyroArr[0]<<8) + GyroArr[1]);
		GyroRW[1] = (GyroArr[2]<<8) + GyroArr[3];
		GyroRW[2] = ((GyroArr[4]<<8) + GyroArr[5]);
	}
}
void MPU6050_Get_Gyro_RawData_to_CaliBration(RawData_Def *rawDef)
{
	//Accel Raw Data
	rawDef->x = GyroRW[0];
	rawDef->y = GyroRW[1];
	rawDef->z = GyroRW[2];

}

// pid
void PID(rawpitchyaw *angle, PID_param *pid, int rop, int Init_CCR, float desired_roll_angle, float desired_pitch_angle, float desired_yaw_angle)
{
      // the previous time is stored before the actual time read
//    pid->time = HAL_GetTick();  // actual time read
////    pid->elapsedTime = (pid->time - pid->timePrev) / 1000;
////    if (pid->elapsedTime == 0.0f)
////	{
////	pid->count_error_pid++;
////	pid->elapsedTime = 0.001;
////	}
//    pid->elapsedTime = (pid->time - pid->timePrev) / 1000.0;
//    pid->timePrev = pid->time;

    pid->elapsedTime = 0.001;

    // into PID
	pid->roll_error = angle->roll - desired_roll_angle;//0.0f; // desired_angle = 0
	pid->pitch_error = angle->pitch - desired_pitch_angle;//0.0f; // desired_angle = 0
	pid->yaw_error = angle->yaw - desired_yaw_angle;//0.0f; // desired_angle = 0

    pid->pid_p_roll = pid->Kp_roll*pid->roll_error;
    pid->pid_p_pitch = pid->Kp_pitch*pid->pitch_error; // tach Kp roll & pitch
    pid->pid_p_yaw = pid->Kp_yaw*pid->yaw_error; // tach Kp roll & pitch

    if (-3 < pid->roll_error && pid->roll_error < 3)
    {
      pid->pid_i_roll = pid->pid_i_roll+(pid->Ki_roll*pid->roll_error);
    }
    if (-3 < pid->pitch_error && pid->pitch_error < 3)
    {
      pid->pid_i_pitch = pid->pid_i_pitch+(pid->Ki_pitch*pid->pitch_error);
    }
    if (-3 < pid->yaw_error && pid->yaw_error < 3)
    {
      pid->pid_i_yaw = pid->pid_i_yaw+(pid->Ki_yaw*pid->yaw_error);
    }



    pid->pid_d_roll = pid->Kd_roll*((pid->roll_error - pid->previous_error_roll)/pid->elapsedTime);
    pid->pid_d_pitch = pid->Kd_pitch*((pid->pitch_error - pid->previous_error_pitch)/pid->elapsedTime);
    pid->pid_d_yaw = pid->Kd_yaw*((pid->yaw_error - pid->previous_error_yaw)/pid->elapsedTime);


    /*The final PID values is the sum of each of this 3 parts*/
    pid->PID_total_roll = pid->pid_p_roll + pid->pid_i_roll + pid->pid_d_roll;
    pid->PID_total_pitch = pid->pid_p_pitch + pid->pid_i_pitch + pid->pid_d_pitch;
    pid->PID_total_yaw = pid->pid_p_yaw + pid->pid_i_yaw + pid->pid_d_yaw;

    if(pid->PID_total_roll < -1000)    {pid->PID_total_roll = 0;}
    if(pid->PID_total_roll > 1000)    {pid->PID_total_roll = 0;}
    if(pid->PID_total_pitch < -1000)    {pid->PID_total_pitch = 0;}
    if(pid->PID_total_pitch > 1000)    {pid->PID_total_pitch = 0;}
    if(pid->PID_total_yaw < -1000)    {pid->PID_total_yaw = 0;}
    if(pid->PID_total_yaw > 1000)    {pid->PID_total_yaw = 0;}
//    pid->pwm_RF_2 = 1150 - pid->PID_total_roll - pid->PID_total_pitch; // throttle = 1300
//    pid->pwm_RB_3 = 1150 - pid->PID_total_roll + pid->PID_total_pitch;
//    pid->pwm_LB_4 = 1150 + pid->PID_total_roll + pid->PID_total_pitch; // throttle = 1300
//    pid->pwm_LF_1 = 1150 + pid->PID_total_roll - pid->PID_total_pitch;


//  1 | 2
//  --o--
//	4 | 3
    pid->pwm_LF_1 = Init_CCR - pid->PID_total_roll - pid->PID_total_pitch + pid->PID_total_yaw; // throttle = 1300
    pid->pwm_RF_2 = Init_CCR + pid->PID_total_roll - pid->PID_total_pitch - pid->PID_total_yaw;
    pid->pwm_RB_3 = Init_CCR + pid->PID_total_roll + pid->PID_total_pitch + pid->PID_total_yaw; // throttle = 1300
    pid->pwm_LB_4 = Init_CCR - pid->PID_total_roll + pid->PID_total_pitch - pid->PID_total_yaw;


    /*Once again we map the PWM values to be sure that we won't pass the min
    and max values. Yes, we've already maped the PID values. But for example, for
    throttle value of 1300, if we sum the max PID value we would have 2300us and
    that will mess up the ESC.*/
    //Right
    if(pid->pwm_RF_2 < 1000)    {        pid->pwm_RF_2 = 1000;    }
    if(pid->pwm_RF_2 > 2000)    {        pid->pwm_RF_2 = 2000;    }
    //Left
    if(pid->pwm_LB_4 < 1000)    {    	pid->pwm_LB_4 = 1000;    }
    if(pid->pwm_LB_4 > 2000)    {    	pid->pwm_LB_4 = 2000;    }
    //Right
    if(pid->pwm_RB_3 < 1000)    {        pid->pwm_RB_3 = 1000;    }
    if(pid->pwm_RB_3 > 2000)    {        pid->pwm_RB_3 = 2000;    }
    //Left
    if(pid->pwm_LF_1 < 1000)    {    	pid->pwm_LF_1 = 1000;    }
    if(pid->pwm_LF_1 > 2000)    {    	pid->pwm_LF_1 = 2000;    }

    /*Finnaly using the servo function we create the PWM pulses with the calculated
    width for each pulse*/
//    htim2.Instance->CCR1 = pid->pwmLeft;
//    htim2.Instance->CCR2 = pid->pwmRight;
    pid->previous_error_roll = pid->roll_error; //Remember to store the previous error.
    pid->previous_error_pitch = pid->pitch_error; //Remember to store the previous error.
    pid->previous_error_yaw = pid->yaw_error; //Remember to store the previous error.
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MahonyAHRSupdate(updateQuater upQua, float sampleFreq, q_volatile *qVol) {
	float Norm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((upQua.mx == 0.0f) && (upQua.my == 0.0f) && (upQua.mz == 0.0f)) {
		MahonyAHRSupdateIMU(upQua, sampleFreq, qVol);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((upQua.axg == 0.0f) && (upQua.ayg == 0.0f) && (upQua.azg == 0.0f))) {

		// Normalise accelerometer measurement
		Norm = sqrt(upQua.axg * upQua.axg + upQua.ayg * upQua.ayg + upQua.azg * upQua.azg);
	    upQua.axg /= Norm;
	    upQua.ayg /= Norm;
	    upQua.azg /= Norm;

		// Normalise magnetometer measurement
		Norm = sqrt(upQua.mx * upQua.mx + upQua.my * upQua.my + upQua.mz * upQua.mz);
		upQua.mx /= Norm;
		upQua.my /= Norm;
		upQua.mz /= Norm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = qVol->q0 * qVol->q0;
        q0q1 = qVol->q0 * qVol->q1;
        q0q2 = qVol->q0 * qVol->q2;
        q0q3 = qVol->q0 * qVol->q3;
        q1q1 = qVol->q1 * qVol->q1;
        q1q2 = qVol->q1 * qVol->q2;
        q1q3 = qVol->q1 * qVol->q3;
        q2q2 = qVol->q2 * qVol->q2;
        q2q3 = qVol->q2 * qVol->q3;
        q3q3 = qVol->q3 * qVol->q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (upQua.mx * (0.5f - q2q2 - q3q3) + upQua.my * (q1q2 - q0q3) + upQua.mz * (q1q3 + q0q2));
        hy = 2.0f * (upQua.mx * (q1q2 + q0q3) + upQua.my * (0.5f - q1q1 - q3q3) + upQua.mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (upQua.mx * (q1q3 - q0q2) + upQua.my * (q2q3 + q0q1) + upQua.mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (upQua.ayg * halfvz - upQua.azg * halfvy) + (upQua.my * halfwz - upQua.mz * halfwy);
		halfey = (upQua.azg * halfvx - upQua.axg * halfvz) + (upQua.mz * halfwx - upQua.mx * halfwz);
		halfez = (upQua.axg * halfvy - upQua.ayg * halfvx) + (upQua.mx * halfwy - upQua.my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			upQua.gxrs += integralFBx;	// apply integral feedback
			upQua.gyrs += integralFBy;
			upQua.gzrs += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
	    upQua.gxrs += twoKp * halfex;
	    upQua.gyrs += twoKp * halfey;
	    upQua.gzrs += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	upQua.gxrs *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	upQua.gyrs *= (0.5f * (1.0f / sampleFreq));
	upQua.gzrs *= (0.5f * (1.0f / sampleFreq));
	qa = qVol->q0;
	qb = qVol->q1;
	qc = qVol->q2;
	qVol->q0 += (-qb * upQua.gxrs - qc * upQua.gyrs - qVol->q3 * upQua.gzrs);
	qVol->q1 += (qa * upQua.gxrs + qc * upQua.gzrs - qVol->q3 * upQua.gyrs);
	qVol->q2 += (qa * upQua.gyrs - qb * upQua.gzrs + qVol->q3 * upQua.gxrs);
	qVol->q3 += (qa * upQua.gzrs + qb * upQua.gyrs - qc * upQua.gxrs);

	// Normalise quaternion
	  Norm = sqrt(qVol->q0 * qVol->q0 + qVol->q1 * qVol->q1 + qVol->q2 * qVol->q2 + qVol->q3 * qVol->q3);
	  qVol->q0 /= Norm;
	  qVol->q1 /= Norm;
	  qVol->q2 /= Norm;
	  qVol->q3 /= Norm;
}
