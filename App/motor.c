#include "motor.h"
#include "common.h"
#include "MK60_FTM.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
//��������105  �ҵ��100
#define LDeath 100
#define RDeath 98

void motor_init()
{    
    ftm_pwm_init(FTM3, FTM_CH0,MOTOR_HZ,0);//����
    ftm_pwm_init(FTM3, FTM_CH3,MOTOR_HZ,0);
    ftm_pwm_init(FTM3, FTM_CH4,MOTOR_HZ,0);//�ҵ��
    ftm_pwm_init(FTM3, FTM_CH5,MOTOR_HZ,0);
}

//����һ��������ٶ�
// ���룺 ������  �ٶ� 
void motor_control(unsigned char ch,int speed)
{
  if(speed>950)speed=950;//��֤��ռ�Ⱥ���������ܳ��ֳ�����ѭ���ڼ��ʧ�ܱ�����
  else if(speed<-950)speed=-950;//�����к���Ҫ��1000����ΪFTM1_PRECISON���õ�Ϊ1000u  
  switch(ch)
  {
  case Rmotor:
    {
      if(speed>=0)
      {  
        ftm_pwm_duty(FTM3, FTM_CH3,0);// FTMģ���  ͨ����  ռ�ձ�
        ftm_pwm_duty(FTM3, FTM_CH0,speed + RDeath);
              
      }
      else
      {
        ftm_pwm_duty(FTM3, FTM_CH0,0);
        ftm_pwm_duty(FTM3, FTM_CH3,-speed + RDeath);
          
      }
    }
    break;
  case Lmotor:
    {
      if(speed>=0)
      {
        ftm_pwm_duty(FTM3, FTM_CH5,speed + LDeath);
        ftm_pwm_duty(FTM3, FTM_CH4,0);// FTMģ���  ͨ����  ռ�ձ�
      }
      else
      {
        ftm_pwm_duty(FTM3, FTM_CH4,-speed + LDeath);
        ftm_pwm_duty(FTM3, FTM_CH5,0);
      }
    }
    break;
  }
}
