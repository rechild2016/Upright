#include "motor.h"
#include "common.h"
#include "MK60_FTM.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
//左电机死区105  右电机100
#define LDeath 100
#define RDeath 98

void motor_init()
{    
    ftm_pwm_init(FTM3, FTM_CH0,MOTOR_HZ,0);//左电机
    ftm_pwm_init(FTM3, FTM_CH3,MOTOR_HZ,0);
    ftm_pwm_init(FTM3, FTM_CH4,MOTOR_HZ,0);//右电机
    ftm_pwm_init(FTM3, FTM_CH5,MOTOR_HZ,0);
}

//设置一个电机的速度
// 输入： 电机序号  速度 
void motor_control(unsigned char ch,int speed)
{
  if(speed>950)speed=950;//保证空占比合理，否则可能出现程序死循环在检测失败报错当中
  else if(speed<-950)speed=-950;//这两行很重要，1000是因为FTM1_PRECISON设置的为1000u  
  switch(ch)
  {
  case Rmotor:
    {
      if(speed>=0)
      {  
        ftm_pwm_duty(FTM3, FTM_CH3,0);// FTM模块号  通道号  占空比
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
        ftm_pwm_duty(FTM3, FTM_CH4,0);// FTM模块号  通道号  占空比
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
