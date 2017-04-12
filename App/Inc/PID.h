#ifndef __PID_H__
#define __PID_H__

typedef struct pid_t{
    float SetPoint;//设定目标值
    long SumError;//误差累计
    float Kp;
    float Ki;
    float Kd;
    float LastError;//Error[-1]
    float PrevError;//Error[-2]
}PID_t;

void PID_Init(PID_t *sptr, float kp, float ki, float kd, float point);
float IncPID_Calc1(float nextpoint, PID_t *sptr);//增量式PID1
float IncPID_Calc2(float nextpoint, PID_t *sptr);//增量式PID2
float LocPID_Calc(float nextpoint, PID_t *sptr);//位置式PID

#endif