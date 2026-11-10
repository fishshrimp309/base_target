#ifndef _PID_H
#define _PID_H

#include "main.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float target;
  float measure;
  float err[2];   // 0,error 1, last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}PID;

typedef struct _CascadePID
{
	PID inner;	   // 内环
	PID outer;	   // 外环
	float output;  // 串级输出，等于inner.output
} CascadePID;

void pid_init(PID *pid,float kp,float ki,float kd,float i_max,float out_max);
float pid_calc(PID *pid, float target, float measure);
void pid_clear(PID *pid);

#endif
