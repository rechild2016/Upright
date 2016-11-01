#include "common.h"
#include "Gyro.h"
  #define agnle_Lowpass (0.3)     //0.7�Ϻø��ټ��ٶȼ�    
  #define Angle_dot_Lowpass (0.01)   //0.4

  float Acc_Smp;        //���ٶ�����
  float Gyro_Smp;       //����������
  float Angle_Smp;        //�ǶȲɼ�
  float Angle_dot_Smp;  //���ٶȲɼ�
  float Angle;         //�Ƕ�
  float Angle_dot;     //���ٶ�
  float Acc_Offset;		//���ٶȼ�ƫ����
  float Gyro_Offset;	//������ƫ����
  
 extern uint16 acc_init[6];
 extern uint16 gyro_init[6];
  
 void AngleAcceleration_AD (uint16 *AdColle)
{
  /*********************�������Ĳ���֪ʶ�ɼ���z�᷽��ļ��ٶ�ֵ*******************/
    AdColle[2]=adc_once( ZOUT, ADC_12bit);    //MMA8451 ZOUT  
    AdColle[3]=adc_once( YOUT, ADC_12bit);    //MMA8451 YOUT  
    AdColle[0] = adc_once(Gyro1, ADC_12bit) ;  //AR1
    AdColle[1] = adc_once(Gyro2, ADC_12bit) ;  //AR2  
}


void ReadSensorData()
{
    float acc_smp =0 , gyro_smp =0 ; 
   
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

void Parameters_Init()
{      
    Acc_Offset = 2000;        //��С���ȶ���ǰ�㣬���˺���  1720  1890 ������������������������
    Gyro_Offset = 2300;      //һ��ʼǰ������𽥻ָ�����Ϊ��������ֵ���ˣ�������С��    ����������
   
    Acc_Smp = Acc_Offset;
    Gyro_Smp = Gyro_Offset;
    
    Angle_Smp = Acc_Offset;
    Angle_dot_Smp = Gyro_Offset;
    
    Angle = 0;
    Angle_dot = 0;
    
}


//__�����˲�__EX_________��Ҫ��һ���Ƕȵ�90��
float EX_Angle = 0;
float EX_DeltaAngle = 0;
float EX_GravityAdjustTime = 2.0;
float EX_angle_k = 1.0;       //��������������������������
float EX_gyro_k = 0.35;  

void Complement_Filter_Ex(float angle,float gyro)     //�����˲��ĳ���
{
  angle *= EX_angle_k;
  //bias *=0.998;//��������Ʈ��ͨ�˲���x�ξ�ֵ�� 
  //bias +=gyro*0.002;
  gyro *= EX_gyro_k;// - bias;
  Angle = EX_Angle;
  Angle_dot = gyro;
  EX_DeltaAngle = (angle - EX_Angle)/EX_GravityAdjustTime;
  EX_Angle += (gyro+EX_DeltaAngle)*0.05;
}
