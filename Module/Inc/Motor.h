#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "pid.h"

#define PI 3.1415926535f
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define TARGETCURRENT_TO_A 0.001220703125f

typedef struct _MOTOR
{
	float angle, speed, current;
	uint8_t temp;

	float targetValue;
    float targetCurrent;
	float targetSpeed;
	float targetAngle;
    float lastAngle;
	float totalAngle;

	PID speedPID;		
	CascadePID anglePID;

	float predicted_power;

	float k1, k2, P0;//预测功率参数

	uint8_t Is_Lost;
	uint16_t LostCnt;
} SingleMotor;

extern SingleMotor motor_3508;

void Motor_Speed_Current_Calc(SingleMotor *motor);
void Motor_Angle_Current_Calc(SingleMotor *motor , float CurrentAngle, float CurrentSpeed);

#endif
