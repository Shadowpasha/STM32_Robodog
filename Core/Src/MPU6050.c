/*
 * MPU6050.c
 *
 *  Created on: Mar 27, 2023
 *      Author: amera
 */

#include "MPU6050.h"


void MPUInit(MPU6050_t *mpu6050, I2C_HandleTypeDef *hi2c,uint8_t rate, uint8_t AccelerometerSensitivity, uint8_t GyroscopeSensitivity, float sample_time, float alpha){

	uint8_t data[2];
	uint8_t temp;
	mpu6050->hi2c = hi2c;
	mpu6050->alpha = alpha;
	mpu6050->sample_time = sample_time;


	data[0] = MPU6050_PWR_MGMT_1;
	data[1] = 0x00;
	HAL_I2C_Master_Transmit(mpu6050->hi2c,(uint16_t)0xD0 , (uint8_t *)data, 2, 1000);

	/* Set sample rate to 1kHz */
	data[0] = MPU6050_SMPLRT_DIV;
	data[1] = rate;
	HAL_I2C_Master_Transmit(mpu6050->hi2c,(uint16_t)0xD0,(uint8_t *)data, 2,1000);

	/* Config accelerometer */
	uint8_t reg = MPU6050_ACCEL_CONFIG;
	HAL_I2C_Master_Transmit(mpu6050->hi2c, (uint16_t)0xD0, &reg, 1, 1000);
	HAL_I2C_Master_Receive(mpu6050->hi2c, (uint16_t)0xD0, &temp, 1, 1000);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	HAL_I2C_Master_Transmit(mpu6050->hi2c, (uint16_t)0xD0,&temp, 1, 1000);

	switch (AccelerometerSensitivity) {
	case MPU6050_Accelerometer_2G:
		mpu6050->Acc_Mult = (float)1 / MPU6050_ACCE_SENS_2;
		break;
	case MPU6050_Accelerometer_4G:
		mpu6050->Acc_Mult = (float)1 / MPU6050_ACCE_SENS_4;
		break;
	case MPU6050_Accelerometer_8G:
		mpu6050->Acc_Mult = (float)1 / MPU6050_ACCE_SENS_8;
		break;
	case MPU6050_Accelerometer_16G:
		mpu6050->Acc_Mult = (float)1 / MPU6050_ACCE_SENS_16;
		break;
	default:
		break;
	}

	/* Config Gyroscope */
	reg = MPU6050_GYRO_CONFIG;
	HAL_I2C_Master_Transmit(mpu6050->hi2c, (uint16_t)0xD0, &reg, 1, 1000);
	HAL_I2C_Master_Receive(mpu6050->hi2c, (uint16_t)0xD0, &temp, 1, 1000);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity  << 3;
	HAL_I2C_Master_Transmit(mpu6050->hi2c, (uint16_t)0xD0,&temp, 1, 1000);

	switch (GyroscopeSensitivity) {
	case MPU6050_Gyroscope_250s:
		mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
		break;
	case MPU6050_Gyroscope_500s:
		mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
		break;
	case MPU6050_Gyroscope_1000s:
		mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
		break;
	case MPU6050_Gyroscope_2000s:
		mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
		break;
	default:
		break;
	}

}

void MPUSetOffsets(MPU6050_t *mpu6050, int16_t ACCx, int16_t ACCy, int16_t ACCz, int16_t GYROx, int16_t GYROy, int16_t GYROz){

	mpu6050->Acc_offsets[0] = ACCx;
	mpu6050->Acc_offsets[1] = ACCy;
	mpu6050->Acc_offsets[2] = ACCz;

	mpu6050->Gyro_offsets[0] = GYROx;
	mpu6050->Gyro_offsets[1] = GYROy;
	mpu6050->Gyro_offsets[2] = GYROz;

}

void MPUReqAccGyro(MPU6050_t *mpu6050){

	uint8_t reg = MPU6050_ACCEL_XOUT_H;

	HAL_I2C_Master_Transmit(mpu6050->hi2c, (uint16_t)0xD0, &reg, 1, 20);
	HAL_I2C_Master_Receive_IT(mpu6050->hi2c, (uint16_t)0xD0, mpu6050->buff, 14);

}

void MPUHandlebuff(MPU6050_t *mpu6050){

	mpu6050->Acc[0] = (int16_t)(mpu6050->buff[0] << 8 | mpu6050->buff[1]);
	mpu6050->Acc[1] = (int16_t)(mpu6050->buff[2] << 8 | mpu6050->buff[3]);
	mpu6050->Acc[2] = (int16_t)(mpu6050->buff[4] << 8 | mpu6050->buff[5]);

	mpu6050->Acc_f[0] = (mpu6050->Acc[0] + mpu6050->Acc_offsets[0]) * mpu6050->Acc_Mult;
	mpu6050->Acc_f[1] = (mpu6050->Acc[1] + mpu6050->Acc_offsets[1]) * mpu6050->Acc_Mult;
	mpu6050->Acc_f[2] = (mpu6050->Acc[2] + mpu6050->Acc_offsets[2]) * mpu6050->Acc_Mult;

	mpu6050->Gyro[0] = (int16_t)(mpu6050->buff[8] << 8 | mpu6050->buff[9]);
	mpu6050->Gyro[1] = (int16_t)(mpu6050->buff[10] << 8 | mpu6050->buff[11]);
	mpu6050->Gyro[2] = (int16_t)(mpu6050->buff[12] << 8 | mpu6050->buff[13]);

	mpu6050->Gyro_f[0] = (mpu6050->Gyro[0] + mpu6050->Gyro_offsets[0]) * mpu6050->Gyro_Mult;
	mpu6050->Gyro_f[1] = (mpu6050->Gyro[1] + mpu6050->Gyro_offsets[1]) * mpu6050->Gyro_Mult;
	mpu6050->Gyro_f[2] = (mpu6050->Gyro[2] + mpu6050->Gyro_offsets[2]) * mpu6050->Gyro_Mult;

}

void CompPitchRoll(MPU6050_t *mpu6050){

	float acc_pitch = atan ((mpu6050->Acc_f[1]) / (mpu6050->Acc_f[2])) *  57.3248;
	float acc_roll = - atan ((mpu6050->Acc_f[0]) / (mpu6050->Acc_f[2])) * 57.3248;

	mpu6050->pitch = (acc_pitch * mpu6050->alpha) + ((mpu6050->pitch + mpu6050->Gyro_f[0]*mpu6050->sample_time)*(1-mpu6050->alpha));
	mpu6050->roll = (acc_roll * mpu6050->alpha) + ((mpu6050->roll + mpu6050->Gyro_f[1]*mpu6050->sample_time)*(1-mpu6050->alpha));

}

