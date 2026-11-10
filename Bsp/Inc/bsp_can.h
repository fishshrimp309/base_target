#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"

#define CAN_CHASSIS_ID   0x200  //M3508 (1-4)
#define CAN_GIMBAL_ID    0x1ff  //GM6020 (1-4)/3508 (5-8)
#define CAN_SHOOTER_ID   0x200  //2006(1-4)/M3508 (1-4)

#define ANGLE_RAW_TO_RAD 0.00076699039f//2*PI / 8192
#define SPEED_RPM_TO_RAD_S 0.10471975512f//2*PI / 60
#define CURRENT_TO_A 0.001f//mA -> A 

typedef struct
{
    uint16_t rotor_pulse;
    int16_t  rotor_rpm;
    int16_t  torque_current;
} motor_info_t;

void can_user_init(CAN_HandleTypeDef* hcan);
void set_motor_current(CAN_HandleTypeDef* hcan,uint32_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
#endif
