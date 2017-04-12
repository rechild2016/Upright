#include "CarControl.h"
#include "PID.h"
#include "Gyro.h"
#include "include.h"


#define CAR_SPEED_CONSTANT (0.1)
Car_Info_t Car_Info;
PID_t SpeedPID;
int32 CarSpeed;

extern float Straigth_Kp;
extern float Straigth_Kd;
extern float SpeedKp;
extern float SpeedKi;
extern float EX_Angle;
extern float EX_DeltaAngle;
extern float Acc_Offset;       //���ٶȼ�ƫ����
extern float Gyro_Offset;	 //������ƫ���� 
extern  int32 LPulseSum,RPulseSum;//�������ۻ�ֵ
void CarInit()
{
    key_init(KEY_U);   key_init(KEY_D); 
    key_init(KEY_L);   key_init(KEY_R);
    key_init(KEY_B);
    
    Car_Info.Upright_PWM=0;
    Car_Info.Kp_ang=Straigth_Kp;//ֱ����PD
    Car_Info.Kd_ang=Straigth_Kd;
    Car_Info.Acc_Offset=Acc_Offset;
    Car_Info.Gyro_Offset=Gyro_Offset;
    PID_Init(&SpeedPID,SpeedKp,SpeedKi,0,0);//�ٶȻ���ʼ��

}
//ֱ������
void Straigth()
{
  Car_Info.Upright_PWM=(int)(Straigth_Kp * Car_Info.Angle - Straigth_Kd * Car_Info.Angle_dot);
}

void EncoderLearn()//ͨ���������������ٶ�offsetֵ  ά���ȶ���ԭλ
{
  Acc_Offset+=(int)((LPulseSum+RPulseSum)/1000);
  LPulseSum=0;
  RPulseSum=0;
}

//�ٶȻ�����
void SpeedControl()
{
  CarSpeed=(LPulseSum+RPulseSum)*0.05;
  LPulseSum=0;
  RPulseSum=0;
  CarSpeed*=CAR_SPEED_CONSTANT;
  Car_Info.Speed_PWM=LocPID_Calc(CarSpeed,&SpeedPID);
}
