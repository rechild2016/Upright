#include "common.h"
#include "include.h"

 uint16 acc_init[6];
 uint16 gyro_init[6];
 uint16 AngleAcceleArry[6];
 uint8 KeyFree=0;       //检测按键松开
 float var[6];
 float StraigthKp=2.5;//直立PID
 float StraigthKd=4.0;
 int Rpulse=0,Lpulse=0; //左右编码器
 int Lspeed,Rspeed;
 
 extern PID_t StraigthPID;
 extern Car_Info_t Car_Info;
 
 extern float Acc_Offset;       //加速度计偏移量
 extern float Gyro_Offset;	 //陀螺仪偏移量 
 extern float Angle_Smp;        //角度采集
 extern float Angle_dot_Smp;    //角速度采集
  
 extern float Angle;            //角度
 extern float Angle_dot;   
 extern float EX_Angle;
 extern float EX_DeltaAngle;
 
  Site_t site_x_1 = {7,10};
  Site_t site_y_1 = {7,30};
  Site_t site_z_1 = {7,50};
  uint8 modle_key=0;
  
 void vcan_sendware(uint8 *wareaddr, uint32 waresize);

  int PIT0InteruptEventCount=0;

  void PIT0_IRQHandler(void);
  void ParameterSet();


void main()
{

    key_init(KEY_U);
    key_init(KEY_D); 
    key_init(KEY_L);
    key_init(KEY_R);
    key_init(KEY_B);
    CarInit();
    GyroInit();
    LCD_init();
    ftm_quad_init(FTM1); 
    ftm_quad_init(FTM2); 
    motor_init();
    Parameters_Init();  
    uart_init(UART4,115200);
     
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); 
    enable_irq(PIT0_IRQn);
    pit_init_ms(PIT0,1);                //初始化PIT0，定时时间为： 5ms
    EnableInterrupts;                    //中断允许   
    
    while(1)
    { 
        DELAY_MS(20);
       // motor_control(Lspeed,Rspeed);
        var[0] = -EX_Angle;       //Car_Info.Acc_Smp;
        var[1] = EX_DeltaAngle;
        var[2] = Car_Info.Upright_PWM;
        var[3] = Angle_dot_Smp;
        var[4] = -Angle;
        var[5] =  Angle_dot;
        vcan_sendware((uint8_t *)var, sizeof(var));//上位机发送函数
        ParameterSet();
    }

}

void PIT0_IRQHandler(void)//1ms进一次中断
{       
  AngleAcceleration_AD (AngleAcceleArry);  //陀螺仪和加速度计的采集
  acc_init[PIT0InteruptEventCount]=AngleAcceleArry[2];//加速度计//z轴
  gyro_init[PIT0InteruptEventCount]=AngleAcceleArry[5];//陀螺仪 
  
  if(PIT0InteruptEventCount==5)   //直立的控制
  {
          Straigth();
          motor_control(Car_Info.Upright_PWM,Car_Info.Upright_PWM);
  }  
  else if(PIT0InteruptEventCount==4)   //陀螺仪
  {
     ReadSensorData();
     Complement_Filter_Ex((float)(Acc_Offset-Angle_Smp), 
                      (float)(Angle_dot_Smp - Gyro_Offset));//角度 角速度
     Car_Info.Angle=EX_Angle;              //角度
     Car_Info.Angle_dot=EX_DeltaAngle;     //角速度
  }   
     
  else if(PIT0InteruptEventCount==3)   //编码器
  {
      Rpulse=-ftm_quad_get(FTM2);//反向安装
      ftm_quad_clean(FTM2);
      Lpulse=ftm_quad_get(FTM1);
      ftm_quad_clean(FTM1);
    
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

//屏幕按键设置参数
void ParameterSet()
{
  if(KeyFree==1)
  {
    //选择要调整的参数
    if(key_check(KEY_R) == KEY_DOWN)
    {
        modle_key++;
    }
    else if(key_check(KEY_L)== KEY_DOWN)
    {
        modle_key--;
    }
    modle_key %=3;
    //调整参数
    switch(modle_key)
    {
    case 0:
      {
        if(key_check(KEY_U) == KEY_DOWN)
          StraigthKp+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           StraigthKp-=0.1;
        
        LCD_num_C (site_x_1, StraigthKp*10 , FCOLOUR , GREEN);
        LCD_num_C (site_y_1, StraigthKd*10 , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, Acc_Offset , FCOLOUR , BCOLOUR);
        break;
      }
    case 1:
      {
        if(key_check(KEY_U) == KEY_DOWN)
          StraigthKd+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           StraigthKd-=0.1;
         LCD_num_C (site_x_1, StraigthKp*10 , FCOLOUR , BCOLOUR);
         LCD_num_C (site_y_1, StraigthKd*10 , FCOLOUR , GREEN);
         LCD_num_C (site_z_1, Acc_Offset , FCOLOUR , BCOLOUR);
        break;
      }
    case 2:
      {
        if(key_check(KEY_U) == KEY_DOWN)
          Acc_Offset++;
        else if(key_check(KEY_D) == KEY_DOWN)
           Acc_Offset--;
         LCD_num_C (site_x_1, StraigthKp*10 , FCOLOUR , BCOLOUR);
         LCD_num_C (site_y_1, StraigthKd*10 , FCOLOUR , BCOLOUR);
         LCD_num_C (site_z_1, Acc_Offset , FCOLOUR , GREEN);
        break;
      }
      
    }
  }
    if((key_check(KEY_U) == KEY_UP) &&(key_check(KEY_D) == KEY_UP) \
          && (key_check(KEY_R) == KEY_UP) && (key_check(KEY_L) == KEY_UP))
      KeyFree=1;//按键已松开
  else KeyFree=0;
  
    //LCD_num_C (site_z_1, Car_Info.Upright_PWM , FCOLOUR , BCOLOUR);
}

