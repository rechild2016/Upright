#ifndef __MOTOR_H__
#define __MOTOR_H__
//电机控制程序

//滑行模式下，频率应该是 30~100。
//常规模式下，频率应该是 20k 左右
#if 0
#define MOTOR_HZ    (50)
#else
#define MOTOR_HZ    (20*1000)
#endif




typedef enum
{
   up,
   down,
} state_car;



extern void motor_init();
extern void motor_control(int Lspeed,int Rspeed);

#endif