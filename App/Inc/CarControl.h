#ifndef __CARCONTROL_H__
#define __CARCONTROL_H__

typedef struct          //定义一个结构体，存储加速度和陀螺仪以及其他直立参数的值
{
	  float Angle;         //角度
	  float Angle_dot;     //角速度
	  float Acc_Offset;		//加速度计偏移量
	  float Gyro_Offset;	//陀螺仪偏移量

	  float Kp_ang;//直立KP
	  float Kd_ang;//直立KD
	  int   Upright_PWM;//直立PWM
	  
	  float Car_Speed_Set;//速度设置
	  float Kp_speed;//速度KP
	  float Ki_speed;//速度KI
	  int   Speed_PWM;//速度PWM
          int   OldSpeed;
          int   NewSpeed;
	  
}Car_Info_t;

void CarInit();
void Straigth();
void EncoderLearn();
void SpeedControl();
#endif