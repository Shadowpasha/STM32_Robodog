/*
 * common.c
 *
 *  Created on: Dec 1, 2022
 *      Author: amera
 */
#include "common.h"

Srv_Drv_t servodriver;
uint32_t ADCValue[4];
char data[100];
float height;
float x_offset,y_offset,yaw_offset;
int smoothdelay, servostep;
float step_height, step_length;
char control;
LEG_t legFL,legFR,legBL,legBR;
uint32_t step_time;
MPU6050_t imu;
QMC_t qmc;

void SysInit(){

	//Peripheral Initialization
	HAL_ADC_Start_DMA(&hadc1, ADCValue, 4);
	LegInit(&legFL, 0, 0);
	height = 0.16;
	step_time = 500;
	step_length= 0.04;
	step_height = 0.04;
	//	legFL.hip_angle = 60.0;
	//	legFL.knee_angle = 71.0;

	HAL_Delay(50);
	ServoDriverInit(&servodriver, &hi2c3, 0x40);
	MPUInit(&imu, &hi2c2, MPU6050_DataRate_1KHz, MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s, 0.005, 0.5);
	MPUSetOffsets(&imu, -4600, 14900, 10154, 1150, 160, 920);
//	QMC_init(&qmc, &hi2c, 50);

	SetSmoothSpeed(0.01);
	HAL_TIM_Base_Start_IT(&htim6);
	MPUReqAccGyro(&imu);
	//	HAL_UART_Receive_IT(&huart3, &control, 1);


}

uint16_t MAP_Angle2Pulse(float input, float AngleMin, float AngleMax, int PulseMin, int PulseMax)
{
	uint16_t result = ((((input - AngleMin)*(float)(PulseMax - PulseMin))/(AngleMax - AngleMin)) + PulseMin);

	if(result > PulseMax)
		result = PulseMax;
	else if(result < PulseMin)
		result = PulseMin;

	return result;
}

void ControlLoop(){

	LegUpdate(&legFL,legFL.x_offset, legFL.y_offset, height + legFL.z_offset, legFL.roll_offset, legFL.pitch_offset, legFL.yaw_offset);
	LegUpdate(&legFR,legFR.x_offset, legFR.y_offset, height + legFR.z_offset, legFR.roll_offset, legFR.pitch_offset, legFR.yaw_offset);
	LegUpdate(&legBL,legBL.x_offset, legBL.y_offset, height + legBL.z_offset, legBL.roll_offset, legBL.pitch_offset, legBL.yaw_offset);
	LegUpdate(&legBR,legBR.x_offset, legBR.y_offset, height + legBR.z_offset, legBR.roll_offset, legBR.pitch_offset, legBR.yaw_offset);
}

void LegUpdate(LEG_t *leg,float x_dist, float y_dist, float heightz, float roll, float pitch, float yaw){

//	Yaw Calculations
	float y_origin,x_origin;
	if(leg->side == 0)
		y_origin = y_dist + Robotwidth/2.0;
	else
		y_origin =  y_dist - Robotwidth/2.0;

	if(leg->front == 0)
		x_origin = x_dist + Robotlength/2.0;
	else
		x_origin = x_dist - Robotlength/2.0;

	float original_yaw = atan2f(y_origin,x_origin);
	float radius = sqrtf(powf(y_origin,2) + powf(x_origin,2));

	float new_yaw = original_yaw + DEGTORAD(yaw);
	float y_yaw = radius * sinf(new_yaw);
	float x_yaw = radius * cosf(new_yaw);

	if(leg->side == 0)
		y_yaw = y_yaw - Robotwidth/2.0;
	else
		y_yaw =  y_yaw + Robotwidth/2.0;

	if(leg->front == 0)
		x_yaw = x_yaw - Robotlength/2.0;
	else
		x_yaw = x_yaw + Robotlength/2.0;

	//Pitch Calculations
	float height_offset_pitch = sinf(DEGTORAD(pitch)) * (Robotlength/2.0);
	float cal_width_pitch = cosf(DEGTORAD(pitch)) * (Robotlength/2.0);

	float dst_tognd_pitch, deltax;
	if (leg->front == 0){
		dst_tognd_pitch = heightz + height_offset_pitch;
		deltax = (Robotlength/2.0 + x_yaw) - cal_width_pitch;
	}else{
		dst_tognd_pitch = heightz - height_offset_pitch;
		deltax = cal_width_pitch - (Robotlength/2.0 - x_yaw);
	}

	float x_angle_offset = atanf(deltax/dst_tognd_pitch);
	float dst_tognd_pitch_x = sqrtf(powf(deltax,2) + powf(dst_tognd_pitch,2));

	float pitch_x = -sinf(DEGTORAD(pitch) - x_angle_offset) * dst_tognd_pitch_x;
	float pitch_z = cosf(DEGTORAD(pitch) - x_angle_offset) * dst_tognd_pitch_x;


	//Roll Calculations
	float height_offset_roll = sinf(DEGTORAD(roll)) * (Robotwidth/2.0);
	float cal_width_roll = cosf(DEGTORAD(roll)) * (Robotwidth/2.0);

	float dst_tognd_roll, deltay;
	if (leg->side == 0){
		dst_tognd_roll = pitch_z + height_offset_roll;
		deltay = (Robotwidth/2.0 + y_yaw) - cal_width_roll;
	}else{
		dst_tognd_roll = pitch_z - height_offset_roll;
		deltay = cal_width_roll - (Robotwidth/2.0 - y_yaw);
	}

	float y_angle_offset = atanf(deltay/dst_tognd_roll);
	float dst_tognd_roll_y = sqrtf(powf(deltay,2) + powf(dst_tognd_roll,2));

	float roll_y = -sin(DEGTORAD(roll) - y_angle_offset) * dst_tognd_roll_y;
	float roll_z = cos(DEGTORAD(roll) - y_angle_offset) * dst_tognd_roll_y;

	 //Final Angle Calculations
	float y_angle = atanf(roll_y/roll_z);
	float y_height = sqrtf(powf(roll_y,2) + powf(roll_z,2));
	float x_angle = atanf(pitch_x/y_height);
	float x_height = sqrtf(powf(pitch_x,2) + powf(y_height,2));

	if (leg->side == 0)
		leg->des_shoulder_angle = 90.0 + RADTODEG(y_angle);
	else
		leg->des_shoulder_angle = 90.0 - RADTODEG(y_angle);

	leg->des_hip_angle = 90.0 - RADTODEG(acosf((powf(x_height,2)+powf(UpperLeg,2)-powf(LowerLeg,2))/(2.0*UpperLeg*x_height))) + RADTODEG(x_angle);
	leg->des_knee_angle =  RADTODEG(acosf((powf(LowerLeg,2)+powf(UpperLeg,2)-powf(x_height,2))/(2.0*UpperLeg*LowerLeg))) ;

}

void LegInit(LEG_t *leg, uint8_t side, uint8_t front){
	leg->des_hip_angle  =  60.0;
	leg->des_knee_angle  =  73.1;
	leg->hip_angle = 60.0;
	leg->knee_angle = 73.1;
	leg->comply_angle = 0.0;
	leg->x_offset = 0.0;
	leg->y_offset = 0.0;
	leg->z_offset = 0.0;
	leg->roll_offset = 0.0;
	leg->pitch_offset = 0.0;
	leg->yaw_offset = 0.0;
	leg->side = side;
	leg->front = front;
}

void LegStepLoop(LEG_t *leg){
	StepLoop(leg->des_hip_angle, &(leg->hip_angle));
	StepLoop(leg->des_knee_angle, &(leg->knee_angle));
	StepLoop(leg->des_shoulder_angle, &(leg->shoulder_angle));
}

void LegSetOffsets(LEG_t *leg, float x_offset, float y_offset, float z_offset, float roll_offset, float pitch_offset, float yaw_offset){
	leg->x_offset = x_offset;
	leg->y_offset = y_offset;
	leg->z_offset = z_offset;
	leg->roll_offset = roll_offset;
	leg->pitch_offset = pitch_offset;
	leg->yaw_offset = yaw_offset;
}

void LegComply(LEG_t *leg, uint32_t force){

	if(force < 400){
		leg->onground = 0;
		leg->comply_angle = 0.0;
	}else{
		leg->onground = 1;
		if(force > 650){
			leg->comply_angle = ((force - 650)/300.0)*15.0;
		}else{
			leg->comply_angle = 0.0;
		}
	}
}

void SetSmoothSpeed(float SmoothSpeed){


	smoothdelay = (int) SmoothSpeed;

	if(smoothdelay <= 0){
		smoothdelay = 1;
	}

	servostep = (int)(1.0/SmoothSpeed);

	if(servostep <= 0){
		servostep = 1;
	}

}


void StepLoop(float desired, float *current){

	if(*current != desired){
		if(desired > *current){
			*current+=(float)servostep;
			if(*current > desired){
				*current = desired;
			}
		}else if (desired < *current){
			*current-=(float)servostep;
			if(*current < desired){
				*current = desired;
			}
		}
	}

}
