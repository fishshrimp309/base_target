#ifndef __RANDOM_H
#define __RANDOM_H

#include "main.h"
#include "Motor.h"


#define PINION_TEETH 25.0f//输出轴齿数
#define RANDOM_RANGE_TEETH 25.0f//单侧齿数

#define GEAR_3508      (3591.0f / 187.0f)
#define M3508_MAX_RPM  400.0f
#define TEETH_PER_RAD   (PINION_TEETH / (2.0f * PI * GEAR_3508))

#define RAD_PER_TEETH   ((2.0f * PI * GEAR_3508) / PINION_TEETH)

#define MAX_TEETH_PER_S ((M3508_MAX_RPM / 60.0f) * PINION_TEETH)//最大齿速度（齿/s）

#define MOVE_TIME_MIN_MS 250u//最小换位时间

#define MIN_STEP_TEETH 15.0f//最小换位差

void OS_RamdomCallback(void *argument);
void Random_MoveTo(float goal_teeth, uint32_t time_ms);
void Random_MovePoint(uint32_t time_ms);
void Random_MoveContinuous(uint32_t time_ms);
uint8_t Random_IsBusy(void);

#endif
