#include "common.h"
#include "Gyro.h"
#include "MK60_adc.h"

#define agnle_Lowpass  0.7
#define Angle_dot_Lowpass  0.1
  //float agnle_Lowpass =0.7;     //0.7较好跟踪加速度计    
  //float Angle_dot_Lowpass =0.1;   //0.4

  float Acc_Smp;        //加速度数据
  float Gyro_Smp;       //陀螺仪数据
  
  float Angle_Smp;      //角度采集
  float Angle_dot_Smp;  //角速度采集
  float Angle;          //角度
  float Angle_dot;      //角速度
  float Acc_Offset;     //加速度计偏移量
  float Gyro_Offset;	//陀螺仪偏移量
  
 extern uint16 acc_init[5];
 extern uint16 gyro_init[5];
  
 void GyroInit()
 {
    adc_init(XOUT);
    adc_init(YOUT);
    adc_init(ZOUT);
    adc_init(Gyro1);         //角加速度  Angular1
    adc_init(Gyro2);  
    adc_init(ANGLE);
 }
void Parameters_Init()
{      
    Acc_Offset = 1075;        //给小了稳定后车后倾，大了前倾 1085 1090 ！！！！！！！！！！！！
    Gyro_Offset = 1725;      //1680    ！！！！！
   
    Acc_Smp = Acc_Offset;
    Gyro_Smp = Gyro_Offset;
    
    Angle_Smp = Acc_Offset;
    Angle_dot_Smp = Gyro_Offset;
    
    Angle = 0;
    Angle_dot = 0;
    
}

 void AngleAcceleration_AD (uint16 *AdColle)
{
  /*********************这个程序的部分知识采集了z轴方向的加速度值*******************/
   
    AdColle[0] = adc_once(Gyro1, ADC_12bit) ;  //x轴陀螺仪
    AdColle[1]=adc_once(ANGLE,ADC_12bit); 
    AdColle[2]=adc_once( ZOUT, ADC_12bit);    //MMA8451 ZOUT  
    AdColle[3]=adc_once( YOUT, ADC_12bit);
    AdColle[4]=adc_once( XOUT, ADC_12bit);
    AdColle[5]=adc_once( Gyro2, ADC_12bit);     //z轴陀螺仪
}

void ReadSensorData()
{
    uint16 acc_smp =0 , gyro_smp =0 ; 
   
    for(int i = 0; i < 5; i ++)
    {
       acc_smp += (float)acc_init[i];//加速度计采样
        gyro_smp += (float)gyro_init[i];//陀螺仪采样
    }
    Acc_Smp = acc_smp/5;     //采集的加速度值
    Gyro_Smp = gyro_smp/5;   //采集的陀螺仪的值
 
    Angle_Smp = Angle_Smp * agnle_Lowpass + Acc_Smp * (1 - agnle_Lowpass);
    Angle_dot_Smp = Angle_dot_Smp * Angle_dot_Lowpass + Gyro_Smp * (1.0 - Angle_dot_Lowpass);
}

//__互补滤波__EX_________需要归一化角度到90内
float EX_Angle = 0;
float EX_DeltaAngle = 0;
float EX_GravityAdjustTime = 2.0;
float EX_angle_k = 1.0;       //！！！！！！！！！！！！！
float EX_gyro_k = 0.12;  

void Complement_Filter_Ex(float angle,float gyro)     //互补滤波的程序
{
  angle *= EX_angle_k;
  gyro *= EX_gyro_k;
  Angle = EX_Angle;
  Angle_dot = gyro;
  EX_DeltaAngle = (angle - EX_Angle)/EX_GravityAdjustTime;
  EX_Angle += (-gyro+EX_DeltaAngle)*0.05;
}
