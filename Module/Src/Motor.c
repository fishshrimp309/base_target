#include "Motor.h"
#include "can.h"
#include "bsp_can.h"
#include "rc.h"
#include "cmsis_os.h"

SingleMotor motor_3508;

void Motor_Init(void)
{
    pid_init(&motor_3508.speedPID, 10, 0, 0, 4000, 15000);
    pid_init(&motor_3508.anglePID.outer, 0, 0, 0, 0, 0);
    pid_init(&motor_3508.anglePID.inner, 0, 0, 0, 0, 0);
}

void Motor_Angle_Calc(void)
{
    motor_3508.targetAngle = rc.ch2 * 0.01f;

    if((rc.ch2 > -0.05f) && (rc.ch2 < 0.05f) )
    {
        pid_clear(&motor_3508.speedPID);
    }
}

void Motor_Speed_Current_Calc(SingleMotor *motor)
{
    motor->speedPID.output = pid_calc(&motor->speedPID, motor->targetSpeed, motor->speed);
    motor->targetValue = motor->speedPID.output;
    motor->targetCurrent = motor->targetValue * TARGETCURRENT_TO_A;
}

void Motor_Angle_Current_Calc(SingleMotor *motor, float CurrentAngle, float CurrentSpeed)
{
    float angle = CurrentAngle;
    float error = motor->targetAngle - angle;
    if(error > PI)
        angle += 2 * PI;
    else if(error < -PI)
        angle -= 2 * PI;
    
    motor->targetSpeed = pid_calc(&motor->anglePID.outer, motor->targetAngle, angle);
    motor->anglePID.output = pid_calc(&motor->anglePID.inner, motor->targetSpeed, CurrentSpeed);
    motor->targetValue = motor->anglePID.output;
    motor->targetCurrent = motor->targetValue * TARGETCURRENT_TO_A;
}

void OS_MotorCallback(void *argument)
{
    osDelay(400);
    Motor_Init();
    for(;;)
    {
        if(!isStop_flag)
        {
           set_motor_current(&hcan1, CAN_CHASSIS_ID,motor_3508.targetValue , 0, 0, 0);
        }
        else
        {
            set_motor_current(&hcan1, CAN_CHASSIS_ID, 0, 0, 0, 0);
            set_motor_current(&hcan1, CAN_GIMBAL_ID,  0, 0, 0, 0);
            set_motor_current(&hcan2, CAN_GIMBAL_ID,  0, 0, 0, 0);
            set_motor_current(&hcan2, CAN_SHOOTER_ID, 0, 0, 0, 0);
        }
        osDelay(1);
    }
}
