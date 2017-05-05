#ifndef __CARCONTROL_H__
#define __CARCONTROL_H__

typedef struct          //����һ���ṹ�壬�洢���ٶȺ��������Լ�����ֱ��������ֵ
{
	  float Angle;         //�Ƕ�
	  float Angle_dot;     //���ٶ�
	  float Acc_Offset;		//���ٶȼ�ƫ����
	  float Gyro_Offset;	//������ƫ����

	  float Kp_ang;//ֱ��KP
	  float Kd_ang;//ֱ��KD
	  int   Upright_PWM;//ֱ��PWM
	  
	  float Car_Speed_Set;//�ٶ�����
	  float Kp_speed;//�ٶ�KP
	  float Ki_speed;//�ٶ�KI
	  int   Speed_PWM;//�ٶ�PWM
          int   OldSpeed;
          int   NewSpeed;
	  
}Car_Info_t;

void CarInit();
void Straigth();
void EncoderLearn();
void SpeedControl();
#endif