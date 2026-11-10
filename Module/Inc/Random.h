#ifndef __RANDOM_H
#define __RANDOM_H

#include "main.h"
#include "Motor.h"

/* 电机输出轴齿数：输出轴转一圈，齿条走 PINION_TEETH 齿 */
#define PINION_TEETH   25.0f
/* 靶子单侧行程齿数（上电位置为 0，左右对称） */
#define RANDOM_RANGE_TEETH 30.0f

/* M3508 内部减速比 3591:187。CAN 反馈是减速前的转子值，位置换算要除以它 */
#define GEAR_3508      (3591.0f / 187.0f)
/* 输出轴（减速后）转速上限 rpm */
#define M3508_MAX_RPM  400.0f

/* 齿 / 转子rad（反馈是减速前转子角度，所以 = 齿数 / (2π × 减速比)） */
#define TEETH_PER_RAD   (PINION_TEETH / (2.0f * PI * GEAR_3508))
/* 转子rad / 齿 */
#define RAD_PER_TEETH   ((2.0f * PI * GEAR_3508) / PINION_TEETH)
/* 最大齿数速度（齿/s）：只与输出转速和齿数有关，与减速比无关 */
#define MAX_TEETH_PER_S ((M3508_MAX_RPM / 60.0f) * PINION_TEETH)

/* 换位最短时间 */
#define MOVE_TIME_MIN_MS 100u

/* 位置环 P（齿/s per 齿） */
#define POS_KP 6.0f

/* 速度环（raw 电流 per 转子 rad/s） */
#define SPEED_PID_KP       60.0f
#define SPEED_PID_KI       1.0f
#define SPEED_PID_KD       0.0f
#define SPEED_PID_I_MAX    8000.0f
#define SPEED_PID_OUT_MAX  14000.0f

/*
 * 方向整定：电机接线不同，正电流可能让齿条往负方向走。
 * 若位置环正反馈（越纠越远、飞车），把 POS_KP 改成负值；
 * 若速度环振荡/跑飞，把 SPEED_PID_KP 改成负值。二者独立。
 */

void OS_RamdomCallback(void *argument);
void Random_MoveTo(float goal_teeth, uint32_t time_ms);
void Random_MoveToRandom(uint32_t time_ms);
/* 连续随机移动：每个控制周期调用一次，一段走完自动接下一段 */
void Random_MoveContinuous(uint32_t time_ms);
uint8_t Random_IsBusy(void);

#endif
