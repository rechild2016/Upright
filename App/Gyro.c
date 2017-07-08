#include "common.h"
#include "Gyro.h"
#include "MK60_adc.h"

#define agnle_Lowpass  0.7
#define Angle_dot_Lowpass  0.1
  //float agnle_Lowpass =0.7;     //0.7�Ϻø��ټ��ٶȼ�    
  //float Angle_dot_Lowpass =0.1;   //0.4

  float Acc_Smp;        //���ٶ�����
  float Gyro_Smp;       //����������
  
  float Angle_Smp;      //�ǶȲɼ�
  float Angle_dot_Smp;  //���ٶȲɼ�
  float Angle;          //�Ƕ�
  float Angle_dot;      //���ٶ�
  float Acc_Offset;     //���ٶȼ�ƫ����
  float Gyro_Offset;	//������ƫ����
  
 extern uint16 acc_init[5];
 extern uint16 gyro_init[5];
  
 void GyroInit()
 {
    adc_init(XOUT);
    adc_init(YOUT);
    adc_init(ZOUT);
    adc_init(Gyro1);         //�Ǽ��ٶ�  Angular1
    adc_init(Gyro2);  
    adc_init(ANGLE);
 }
void Parameters_Init()
{      
    Acc_Offset = 1075;        //��С���ȶ��󳵺��㣬����ǰ�� 1085 1090 ������������������������
    Gyro_Offset = 1725;      //1680    ����������
   
    Acc_Smp = Acc_Offset;
    Gyro_Smp = Gyro_Offset;
    
    Angle_Smp = Acc_Offset;
    Angle_dot_Smp = Gyro_Offset;
    
    Angle = 0;
    Angle_dot = 0;
    
}

 void AngleAcceleration_AD (uint16 *AdColle)
{
  /*********************�������Ĳ���֪ʶ�ɼ���z�᷽��ļ��ٶ�ֵ*******************/
   
    AdColle[0] = adc_once(Gyro1, ADC_12bit) ;  //x��������
    AdColle[1]=adc_once(ANGLE,ADC_12bit); 
    AdColle[2]=adc_once( ZOUT, ADC_12bit);    //MMA8451 ZOUT  
    AdColle[3]=adc_once( YOUT, ADC_12bit);
    AdColle[4]=adc_once( XOUT, ADC_12bit);
    AdColle[5]=adc_once( Gyro2, ADC_12bit);     //z��������
}

void ReadSensorData()
{
    uint16 acc_smp =0 , gyro_smp =0 ; 
   
    for(int i = 0; i < 5; i ++)
    {
       acc_smp += (float)acc_init[i];//���ٶȼƲ���
        gyro_smp += (float)gyro_init[i];//�����ǲ���
    }
    Acc_Smp = acc_smp/5;     //�ɼ��ļ��ٶ�ֵ
    Gyro_Smp = gyro_smp/5;   //�ɼ��������ǵ�ֵ
 
    Angle_Smp = Angle_Smp * agnle_Lowpass + Acc_Smp * (1 - agnle_Lowpass);
    Angle_dot_Smp = Angle_dot_Smp * Angle_dot_Lowpass + Gyro_Smp * (1.0 - Angle_dot_Lowpass);
}

//__�����˲�__EX_________��Ҫ��һ���Ƕȵ�90��
float EX_Angle = 0;
float EX_DeltaAngle = 0;
float EX_GravityAdjustTime = 2.0;
float EX_angle_k = 1.0;       //��������������������������
float EX_gyro_k = 0.12;  

void Complement_Filter_Ex(float angle,float gyro)     //�����˲��ĳ���
{
  angle *= EX_angle_k;
  gyro *= EX_gyro_k;
  Angle = EX_Angle;
  Angle_dot = gyro;
  EX_DeltaAngle = (angle - EX_Angle)/EX_GravityAdjustTime;
  EX_Angle += (-gyro+EX_DeltaAngle)*0.05;
}
