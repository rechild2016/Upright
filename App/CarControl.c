#include "CarControl.h"
#include "PID.h"
#include "Gyro.h"

Car_Info_t Car_Info;
PID_t SpeedPID;
extern float StraigthKp;
extern float StraigthKd;
extern float EX_Angle;
extern float EX_DeltaAngle;
extern float Acc_Offset;       //加速度计偏移量
extern float Gyro_Offset;	 //陀螺仪偏移量 
 
void CarInit()
{
 Car_Info.Upright_PWM=0;
 Car_Info.Kp_ang=StraigthKp;//直立的PD
 Car_Info.Kd_ang=StraigthKd;
 Car_Info.Acc_Offset=Acc_Offset;
 Car_Info.Gyro_Offset=Gyro_Offset;
// PID_Init(&SpeedPID,);
  
}
//直立控制
void Straigth()
{
  Car_Info.Upright_PWM=(int)(StraigthKp * Car_Info.Angle \
                              + StraigthKd * Car_Info.Angle_dot);
}
