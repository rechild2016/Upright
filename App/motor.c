#include "motor.h"
#include "common.h"
#include "MK60_FTM.h"
#include "MK60_port.h"
#include "MK60_gpio.h"

#define LDeath 110               //易变
#define RDeath 90

void motor_init()
{    
    ftm_pwm_init(FTM3, FTM_CH0,MOTOR_HZ,0);//左电机
    ftm_pwm_init(FTM3, FTM_CH3,MOTOR_HZ,0);
    ftm_pwm_init(FTM3, FTM_CH4,MOTOR_HZ,0);//右电机
    ftm_pwm_init(FTM3, FTM_CH5,MOTOR_HZ,0);
}

//设置一个电机的速度
// 输入： 电机序号  速度 
void motor_control(int Lspeed,int Rspeed)
{
  if(Lspeed>850)Lspeed=850;//保证空占比合理，否则可能出现程序死循环在检测失败报错当中
  else if(Lspeed<-850)Lspeed=-850;//这两行很重要，1000是因为FTM1_PRECISON设置的为1000u  
  
  if(Rspeed>850)Rspeed=850;
  else if(Rspeed<-850)Rspeed=-850;
//右电机
    if(Rspeed>=0)
      {
      ftm_pwm_duty(FTM3, FTM_CH4,0);
      ftm_pwm_duty(FTM3, FTM_CH5,Rspeed + RDeath);
    }
    else
    {
      ftm_pwm_duty(FTM3, FTM_CH5,0);
      ftm_pwm_duty(FTM3, FTM_CH4,-Rspeed + RDeath);// FTM模块号  通道号  占空比
    }

  //左电机
    if(Lspeed>=0)
   {  
      ftm_pwm_duty(FTM3, FTM_CH0,Lspeed + LDeath);
      ftm_pwm_duty(FTM3, FTM_CH3,0);       
    }
    else
    {
      ftm_pwm_duty(FTM3, FTM_CH3,-Lspeed + LDeath);// FTM模块号  通道号  占空比
      ftm_pwm_duty(FTM3, FTM_CH0,0);
        
    }
 
  
}
