#ifndef __MOTOR_H__
#define __MOTOR_H__
//������Ƴ���

//����ģʽ�£�Ƶ��Ӧ���� 30~100��
//����ģʽ�£�Ƶ��Ӧ���� 20k ����
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