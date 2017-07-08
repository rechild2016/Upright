#include "CarControl.h"
#include "PID.h"
#include "Gyro.h"
#include "include.h"


#define CAR_SPEED_CONSTANT (0.1)
Car_Info_t Car_Info;
PID_t SpeedPID;
PID_t DirPID;

int CarSpeed=0;


extern float Upright_Kp[5];
extern float Upright_Kd[5];
extern float SpeedKp;
extern float SpeedKi;
extern float EX_Angle;
extern float EX_DeltaAngle;
extern float Acc_Offset;       //加速度计偏移量
extern float Gyro_Offset;	 //陀螺仪偏移量 
extern  int16 LPulseSum,RPulseSum;//编码器累积值
extern float DirKp;        //方向PID
extern float DirKd;
void CarInit()
{
    key_init(KEY_U);   key_init(KEY_D); 
    key_init(KEY_L);   key_init(KEY_R);
    key_init(KEY_B);
    
    GyroInit();         //陀螺仪初始化
    Car_Info.Upright_PWM=0;
    Car_Info.Acc_Offset=Acc_Offset;
    Car_Info.Gyro_Offset=Gyro_Offset;
    PID_Init(&SpeedPID,SpeedKp,SpeedKi,0,0);//速度环初始化
    PID_Init(&DirPID,DirKp,0,DirKd,80);

}
//直立控制
uint8 t=0;
void Straigth()
{
  
  int diff_angle=(int)fabs(Car_Info.Angle);
  if(diff_angle>80 )t=4;//范围过大
  else{
    t=diff_angle/16;
  }
  Car_Info.Upright_PWM=-(int)(Upright_Kp[t] * Car_Info.Angle - Upright_Kd[t] * Car_Info.Angle_dot);
  //Car_Info.Upright_PWM=(int)(Straigth_Kp * Car_Info.Angle - Straigth_Kd * Car_Info.Angle_dot);
}

void EncoderLearn()//通过编码器调整加速度offset值  维持稳定在原位
{
  Acc_Offset+=(int)((LPulseSum+RPulseSum)/1000);
  LPulseSum=0;
  RPulseSum=0;
}

//速度环控制
void SpeedControl()
{
  CarSpeed=(int)((LPulseSum+RPulseSum)*0.5);
  LPulseSum=0;
  RPulseSum=0;
  CarSpeed *=CAR_SPEED_CONSTANT;
  Car_Info.OldSpeed=Car_Info.NewSpeed;
  Car_Info.NewSpeed=(int )LocPID_Calc(CarSpeed,&SpeedPID);
}
