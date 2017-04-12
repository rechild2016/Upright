#include "PID.h"

void PID_Init(PID_t *sptr, float kp, float ki, float kd, float point)
{
	sptr->SumError = 0;
	sptr->LastError = 0;
	sptr->PrevError = 0;
	sptr->Kp = kp;
	sptr->Ki = ki;
	sptr->Kd = kd;
	sptr->SetPoint = point;
}

float IncPID_Calc1(float nextpoint, PID_t *sptr)
{
      float iError, iIncPid;
      iError = sptr->SetPoint - nextpoint;//当前误差
      iIncPid = sptr->Kp*iError 
                - sptr->Ki*sptr->LastError 
                + sptr->Kd*sptr->PrevError;//增量计算
      sptr->PrevError = sptr->LastError;//存储误差
      sptr->LastError = iError;//存储误差
      return iIncPid;
}

float IncPID_Calc2(float nextpoint, PID_t *sptr)
{
    float iError, iIncPid;
    iError = sptr->SetPoint - nextpoint;//当前误差E(k)
    iIncPid = sptr->Kp*(iError-sptr->LastError) 
              + sptr->Ki*iError 
              + sptr->Kd*(iError-2*sptr->LastError+sptr->PrevError);//增量计算
    sptr->PrevError = sptr->LastError;//存储误差
    sptr->LastError = iError;           //存储误差
    return iIncPid;
}

float LocPID_Calc(float nextpoint, PID_t *sptr)
{
    float iError, dError, iLocPid;
    iError = sptr->SetPoint - nextpoint;  //偏差
    sptr->SumError += iError;   //积分
    dError = iError - sptr->LastError;     //微分
    sptr->LastError = iError; 
    iLocPid = sptr->Kp * iError  //比例项
              + sptr->Ki * sptr->SumError  //积分项
              + sptr->Kd * dError; //微分项
    return iLocPid;
}
