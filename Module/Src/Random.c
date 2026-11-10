#include "Random.h"
#include "Judge.h"
#include "bsp_can.h"
#include "can.h"
#include "rc.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 一次换位的全部状态 */
static float    start_teeth;      /* 本次换位起点（齿） */
static float    goal_teeth;       /* 本次换位目标（齿） */
static uint32_t start_move_t0_ms; /* 本次换位开始时刻 */
static uint32_t move_ms;          /* 本次换位时长 */
static uint8_t last_sw1 = 0xFF; 

/* 随机数状态 */
static uint32_t s_rng;

/* ------------------------------ 随机数 ------------------------------ */
static float rand_01(void)
{
    uint32_t x = s_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng = x;
    return (float)(x >> 8) * (1.0f / 16777216.0f);
}

/* ------------------------------ 位置反馈 ------------------------------ */
/* 当前齿数位置：totalAngle 累计的是减速前转子弧度 */
static float pos_teeth(void)
{
    return motor_3508.totalAngle * TEETH_PER_RAD;
}

/* 用 CAN 中断维护的 motor_3508.angle（转子rad，0~2π）做多圈展开，
 * 累加到 totalAngle（float，转子弧度）。不接触编码器原始计数。 */
static void feedback(void)
{
    float a = motor_3508.angle;
    float d = a - motor_3508.lastAngle;

    if (d > PI)       d -= 2.0f * PI;
    else if (d < -PI) d += 2.0f * PI;

    motor_3508.lastAngle  = a;
    motor_3508.totalAngle += d;
}

/* ------------------------------ 1ms 周期执行 ------------------------------ */
static void update(void)
{
    uint32_t now = HAL_GetTick();
    float ref, vel = 0.0f;

    feedback();

    /* 换位中：最小 jerk 曲线给位置和速度前馈；结束后停在目标点 */
    if ((now - start_move_t0_ms) < move_ms)
    {
        float u  = (float)(now - start_move_t0_ms) / (float)move_ms;
        float u2 = u * u;
        float k  = u2 * u * (10.0f - 15.0f * u + 6.0f * u2);  /* 10u^3-15u^4+6u^5 */
        float dk = 30.0f * u2 * (1.0f - u) * (1.0f - u);      /* dk/du */
        float d  = goal_teeth - start_teeth;

        ref = start_teeth + d * k;
        vel = d * dk / ((float)move_ms * 0.001f);
    }
    else
    {
        ref = goal_teeth;
    }

    /* 位置 P + 速度前馈 -> 速度环 -> 电流 */
    vel += POS_KP * (ref - pos_teeth());
    if (vel > MAX_TEETH_PER_S)       vel = MAX_TEETH_PER_S;
    else if (vel < -MAX_TEETH_PER_S) vel = -MAX_TEETH_PER_S;

    motor_3508.targetSpeed = vel * RAD_PER_TEETH;
}

/* ------------------------------ 对外接口 ------------------------------ */
void Random_MoveTo(float target_teeth, uint32_t time_ms)
{
    float d;

    if (target_teeth > RANDOM_RANGE_TEETH)       target_teeth = RANDOM_RANGE_TEETH;
    else if (target_teeth < -RANDOM_RANGE_TEETH) target_teeth = -RANDOM_RANGE_TEETH;

    start_teeth      = pos_teeth();
    goal_teeth       = target_teeth;
    start_move_t0_ms = HAL_GetTick();

    d = target_teeth - start_teeth;
    if (d < 0.0f) d = -d;

    /* 按最大齿数速度估算最短时间，再与调用者给出的时间上限取较小值 */
    move_ms = (uint32_t)(1.875f * d / MAX_TEETH_PER_S * 1000.0f);
    if (move_ms < MOVE_TIME_MIN_MS) move_ms = MOVE_TIME_MIN_MS;
    if (move_ms > time_ms)          move_ms = time_ms;
    if (move_ms == 0u)              move_ms = 1u;
}


#define MIN_STEP_TEETH 10.0f
void Random_MoveToRandom(uint32_t time_ms)
{
    float   cur = pos_teeth();
    float   g   = cur;
    uint8_t i;

    for (i = 0; i < 20u; i++)
    {
        g = -RANDOM_RANGE_TEETH + rand_01() * (2.0f * RANDOM_RANGE_TEETH);
        if (g - cur >= MIN_STEP_TEETH || cur - g >= MIN_STEP_TEETH)
            break;
    }

    Random_MoveTo(g, time_ms);
}

void Random_MoveContinuous(uint32_t time_ms)
{
    if (!Random_IsBusy()) 
        Random_MoveToRandom(time_ms);
}

uint8_t Random_IsBusy(void)
{
    return ((HAL_GetTick() - start_move_t0_ms) < move_ms) ? 1u : 0u;
}

void Rocker_Ctrl(void)
{
    motor_3508.targetSpeed = -rc.ch0 * 600;
}

void Random_Fix_Ctrl(void)
{
    if (last_sw1 == 3 && rc.sw1 == 1)
    {
        Random_MoveToRandom(2000);
    }
    
    last_sw1 = rc.sw1;
    update();
}

void Random_Move_Ctrl(void)
{
    static int8_t  enable = -1; 
    static uint8_t  state = 1;//1连续随机移动 2发射后移动并静止10s
    static uint32_t launch_ms = 0;

    if (rc.roll < -400)
    {
        enable *= -1;
    }
    
    if (enable == 1)
    {
        if (last_sw1 == 3 && rc.sw1 == 1)
        {
            Random_MoveToRandom(600);
            launch_ms = HAL_GetTick();
            state     = 2;
        }
        else if (state == 1)
        {
            Random_MoveContinuous(1000); 
        }
        else if (state == 2 && ((USER_JudgeData.hit_myself == 1) || ((HAL_GetTick() - launch_ms) >= 10000)))
        {
            state = 1; 
            USER_JudgeData.hit_myself = 0;
        }
    }

    last_sw1 = rc.sw1;
    update();
}

void Terminal_Random_Move_Ctrl(void)
{
    static int8_t enable = -1;
    static uint8_t state = 1;
    static uint32_t launch_ms = 0;

    if(rc.roll < -400)
    {
        enable *= -1;
    }

    if(enable == 1)
    {
        if(last_sw1 == 3 && rc.sw1 == 1)
        {
            launch_ms = HAL_GetTick();
            state = 2;
        }
        else if(state == 1)
        {
            Random_MoveContinuous(1000);
        }
        else if(state == 2)
        {
            if((HAL_GetTick() - launch_ms) > 1200)
            {
                Random_MoveToRandom(600);
                state = 3;
            }
        }
        else if(state == 3 && ((USER_JudgeData.hit_myself == 1) || ((HAL_GetTick() - launch_ms) > 10000)))
        {
            state = 1;
			USER_JudgeData.hit_myself = 0;
        }
    }

    last_sw1 = rc.sw1;
    update();
}

void Task_RandomCallback(void)
{
	if((rc.ch3 < 0.3) && (rc.ch3 > -0.3))
	{
		if(rc.sw2 == 2)
		{
			Random_Fix_Ctrl();
		}
		else if (rc.sw2 == 3)
		{
?			Random_Move_Ctrl();
		}
		else if (rc.sw2 == 1)
		{
			Terminal_Random_Move_Ctrl();
		}
	}
	else if(rc.ch3 > 0.5)
	{
		Rocker_Ctrl();
	}
        
    Motor_Speed_Current_Calc(&motor_3508);
}

void OS_RamdomCallback(void *argument)
{
    (void)argument;
    osDelay(100);

    s_rng = 0x9E3779B9u ^ (uint32_t)HAL_GetTick()
          ^ (uint32_t)(motor_3508.angle * 1000000.0f)
          ^ (uint32_t)(motor_3508.speed * 1000.0f);
    if (s_rng == 0u) s_rng = 1u;

    motor_3508.lastAngle  = motor_3508.angle;
    motor_3508.totalAngle = 0.0f;
    start_teeth      = 0.0f;
    goal_teeth       = 0.0f;
    start_move_t0_ms = HAL_GetTick();
    move_ms          = 0u;

    for (;;)
    {
        Task_RandomCallback();
        osDelay(1);
    }
}
