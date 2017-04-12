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
      iError = sptr->SetPoint - nextpoint;//��ǰ���
      iIncPid = sptr->Kp*iError 
                - sptr->Ki*sptr->LastError 
                + sptr->Kd*sptr->PrevError;//��������
      sptr->PrevError = sptr->LastError;//�洢���
      sptr->LastError = iError;//�洢���
      return iIncPid;
}

float IncPID_Calc2(float nextpoint, PID_t *sptr)
{
    float iError, iIncPid;
    iError = sptr->SetPoint - nextpoint;//��ǰ���E(k)
    iIncPid = sptr->Kp*(iError-sptr->LastError) 
              + sptr->Ki*iError 
              + sptr->Kd*(iError-2*sptr->LastError+sptr->PrevError);//��������
    sptr->PrevError = sptr->LastError;//�洢���
    sptr->LastError = iError;           //�洢���
    return iIncPid;
}

float LocPID_Calc(float nextpoint, PID_t *sptr)
{
    float iError, dError, iLocPid;
    iError = sptr->SetPoint - nextpoint;  //ƫ��
    sptr->SumError += iError;   //����
    dError = iError - sptr->LastError;     //΢��
    sptr->LastError = iError; 
    iLocPid = sptr->Kp * iError  //������
              + sptr->Ki * sptr->SumError  //������
              + sptr->Kd * dError; //΢����
    return iLocPid;
}
