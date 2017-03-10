#include "common.h"
#include "include.h"

 uint16 acc_init[6];
 uint16 gyro_init[6];
 uint16 AngleAcceleArry[6];
 float var[6];
 int Rpulse=0,Lpulse=0; //左右编码器
 
 extern float Acc_Smp;        //加速度数据
 extern float Gyro_Smp;       //陀螺仪数据
 extern float Acc_Offset;		//加速度计偏移量
 extern float Gyro_Offset;	//陀螺仪偏移量
 extern float Angle_Smp;        //角度采集
 extern float Angle;         //角度
 extern float Angle_dot;   
 extern float EX_Angle;
 extern float EX_DeltaAngle;
 extern float Angle_dot_Smp;  //角速度采集
 
 void vcan_sendware(uint8 *wareaddr, uint32 waresize);

int PIT0InteruptEventCount=0;

void PIT0_IRQHandler(void);
int Lspeed=0;
int Rspeed=0;

void main()
{
    adc_init(XOUT);
    adc_init(YOUT);
    adc_init(ZOUT);
    adc_init(Gyro1);         //角加速度  Angular1
    adc_init(Gyro2);  
    adc_init(ANGLE);
    ftm_quad_init(FTM1); 
    ftm_quad_init(FTM2); 
    motor_init();
    Parameters_Init();  
    
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);
    
    uart_init(UART4,115200);
    enable_irq(PIT0_IRQn);
    pit_init_ms(PIT0, 1);                //初始化PIT0，定时时间为： 5ms
    EnableInterrupts;                    //中断允许   
    while(1)
    { 
        DELAY_MS(10);
        var[0] = EX_Angle;       //Car_Info.Acc_Smp;
        var[1] = EX_DeltaAngle;
        var[2] = Angle_Smp;
        var[3] = Angle_dot_Smp;
        var[4] = -Angle;
        var[5] =  Angle_dot;
        vcan_sendware((uint8_t *)var, sizeof(var));//上位机发送函数
    }

}

void PIT0_IRQHandler(void)//1ms进一次中断
{       
  AngleAcceleration_AD (AngleAcceleArry);  //陀螺仪和加速度计的采集
  
  acc_init[PIT0InteruptEventCount]=AngleAcceleArry[2];//加速度计//z轴
  gyro_init[PIT0InteruptEventCount]=AngleAcceleArry[5];//陀螺仪 
  if(PIT0InteruptEventCount==5)   //直立的控制
  {
          ReadSensorData();
          Complement_Filter_Ex((float)(Acc_Offset-Angle_Smp), 
                            (float)(Angle_dot_Smp - Gyro_Offset));//角度 角速度
  }
  else if(PIT0InteruptEventCount==4)   //编码器
  {
      Rpulse=-ftm_quad_get(FTM1);//反向安装
      ftm_quad_clean(FTM1);
      Lpulse=ftm_quad_get(FTM2);
      ftm_quad_clean(FTM2);
    
  }
  else if(PIT0InteruptEventCount==3)   //编码器
  {
    motor_control(Lmotor,Lspeed);
    motor_control(Rmotor,Rspeed);
  }   
      PIT0InteruptEventCount++;
      if( PIT0InteruptEventCount==6)
          PIT0InteruptEventCount=0;
       
      PIT_Flag_Clear(PIT0);//清中断标志位
}

void vcan_sendware(uint8 *wareaddr, uint32 waresize)
{
    #define CMD_WARE     3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //yy_摄像头串口调试 使用的命令
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //yy_摄像头串口调试 使用的命令
    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送命令
    uart_putbuff(VCAN_PORT, wareaddr, waresize); //再发送图像
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //先发送命令
}

