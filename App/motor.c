#include "motor.h"
#include "common.h"
#include "MK60_FTM.h"
#include "MK60_port.h"
#include "MK60_gpio.h"

#define LDeath 100               //�ױ�
#define RDeath 90

void motor_init()
{    
    ftm_pwm_init(FTM3, FTM_CH0,MOTOR_HZ,0);//����
    ftm_pwm_init(FTM3, FTM_CH3,MOTOR_HZ,0);
    ftm_pwm_init(FTM3, FTM_CH4,MOTOR_HZ,0);//�ҵ��
    ftm_pwm_init(FTM3, FTM_CH5,MOTOR_HZ,0);
}

//����һ��������ٶ�
// ���룺 ������  �ٶ� 
void motor_control(int Lspeed,int Rspeed)
{
  if(Lspeed>850)Lspeed=850;//��֤��ռ�Ⱥ���������ܳ��ֳ�����ѭ���ڼ��ʧ�ܱ�����
  else if(Lspeed<-850)Lspeed=-850;//�����к���Ҫ��1000����ΪFTM1_PRECISON���õ�Ϊ1000u  
  
  if(Rspeed>850)Rspeed=850;
  else if(Rspeed<-850)Rspeed=-850;
//����
    if(Lspeed>=0)
      {
      ftm_pwm_duty(FTM3, FTM_CH4,0);
      ftm_pwm_duty(FTM3, FTM_CH5,Lspeed + LDeath);
    }
    else
    {
      ftm_pwm_duty(FTM3, FTM_CH5,0);
      ftm_pwm_duty(FTM3, FTM_CH4,-Lspeed + LDeath);// FTMģ���  ͨ����  ռ�ձ�
    }

  //�ҵ��
    if(Rspeed>=0)
   {  
     ftm_pwm_duty(FTM3, FTM_CH3,Rspeed + RDeath);// FTMģ���  ͨ����  ռ�ձ�
     ftm_pwm_duty(FTM3, FTM_CH0,0);        
   }
   else
   {
      ftm_pwm_duty(FTM3, FTM_CH0,-Rspeed + RDeath);
      ftm_pwm_duty(FTM3, FTM_CH3,0);      
   }
 
  
}
