/*
 * common.h
 *
 *  Created on: Dec 1, 2022
 *      Author: amera
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

//#include "pca9685.h"
#include "servo_driver.h"
#include "i2c.h"
#include "adc.h"
#include "usart.h"
#include "main.h"
#include "tim.h"
#include "MPU6050.h"
#include "QMC5883.h"

#define LowerLeg 0.145
#define UpperLeg 0.122
#define Robotlength 0.26
#define Robotwidth 0.26
#define RADTODEG(x) x*57.2925f
#define DEGTORAD(x) x/57.2925f

#define FLHipmin 110
#define FLHipmax 500

#define FLKneemin 130
#define FLKneemax 350

enum { FLShoulder = 0, FLHip, FLKnee, FRShoulder, FRHip, FRKnee, BLShoulder, BLHip, BLKnee, BRShoulder, BRHip, BRKnee};

typedef struct {
float des_hip_angle;
float des_knee_angle;
float des_shoulder_angle;

float hip_angle;
float knee_angle;
float shoulder_angle;

float x_offset,y_offset,z_offset,roll_offset,pitch_offset,yaw_offset;

float y_angle,y_height,x_angle,x_height;

uint8_t side, front;

uint8_t onground;
float comply_angle;

}LEG_t;

extern Srv_Drv_t servodriver;
extern uint32_t ADCValue[4];
extern float height;
extern float x_offset,y_offset,yaw_offset;
extern float step_height, step_length;
extern uint32_t step_time;

extern char data[100];
extern char control;
extern int smoothdelay, servostep;

extern LEG_t legFL,legFR,legBL,legBR;
extern MPU6050_t imu;
extern QMC_t qmc;

extern void SysInit();
extern void ControlLoop();
void LegUpdate(LEG_t *leg,float x_dist, float y_dist, float height, float roll, float pitch, float yaw);
void LegInit(LEG_t *leg, uint8_t side, uint8_t front);
void LegStepLoop(LEG_t *leg);
void LegSetOffsets(LEG_t *leg, float x_offset, float y_offset, float z_offset, float roll_offset, float pitch_offset, float yaw_offset);
extern void SetSmoothpeed();
extern void SetSmoothSpeed(float SmoothSpeed);
extern uint16_t MAP_Angle2Pulse(float input, float AngleMin, float AngleMax, int PulseMin, int PulseMax);
void StepLoop(float desired, float *current);
void LegComply(LEG_t *leg, uint32_t force);
#endif /* INC_COMMON_H_ */
