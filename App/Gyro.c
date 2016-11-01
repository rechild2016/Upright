#include "common.h"
#include "Gyro.h"
  #define agnle_Lowpass (0.3)     //0.7较好跟踪加速度计    
  #define Angle_dot_Lowpass (0.01)   //0.4

  float Acc_Smp;        //加速度数据
  float Gyro_Smp;       //陀螺仪数据
  float Angle_Smp;        //角度采集
  float Angle_dot_Smp;  //角速度采集
  float Angle;         //角度
  float Angle_dot;     //角速度
  float Acc_Offset;		//加速度计偏移量
  float Gyro_Offset;	//陀螺仪偏移量
  
 extern uint16 acc_init[6];
 extern uint16 gyro_init[6];
  
 void AngleAcceleration_AD (uint16 *AdColle)
{
  /*********************这个程序的部分知识采集了z轴方向的加速度值*******************/
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
       acc_smp += (float)acc_init[i];//加速度计采样
      gyro_smp += (float)gyro_init[i];//陀螺仪采样
    }
    Acc_Smp = acc_smp/5;     //采集的加速度值
   Gyro_Smp = gyro_smp/5;   //采集的陀螺仪的值
 
    Angle_Smp = Angle_Smp * agnle_Lowpass + Acc_Smp * (1 - agnle_Lowpass);
    Angle_dot_Smp = Angle_dot_Smp * Angle_dot_Lowpass + Gyro_Smp * (1.0 - Angle_dot_Lowpass);
}

void Parameters_Init()
{      
    Acc_Offset = 2000;        //给小了稳定后车前倾，大了后倾  1720  1890 ！！！！！！！！！！！！
    Gyro_Offset = 2300;      //一开始前倾而后逐渐恢复是因为陀螺仪中值大了，后倾则小了    ！！！！！
   
    Acc_Smp = Acc_Offset;
    Gyro_Smp = Gyro_Offset;
    
    Angle_Smp = Acc_Offset;
    Angle_dot_Smp = Gyro_Offset;
    
    Angle = 0;
    Angle_dot = 0;
    
}


//__互补滤波__EX_________需要归一化角度到90内
float EX_Angle = 0;
float EX_DeltaAngle = 0;
float EX_GravityAdjustTime = 2.0;
float EX_angle_k = 1.0;       //！！！！！！！！！！！！！
float EX_gyro_k = 0.35;  

void Complement_Filter_Ex(float angle,float gyro)     //互补滤波的程序
{
  angle *= EX_angle_k;
  //bias *=0.998;//陀螺仪零飘低通滤波；x次均值； 
  //bias +=gyro*0.002;
  gyro *= EX_gyro_k;// - bias;
  Angle = EX_Angle;
  Angle_dot = gyro;
  EX_DeltaAngle = (angle - EX_Angle)/EX_GravityAdjustTime;
  EX_Angle += (gyro+EX_DeltaAngle)*0.05;
}
