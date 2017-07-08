#include "common.h"
#include "include.h"

 
 uint16 acc_init[5];
 uint16 gyro_init[5];
 uint16 AngleAcceleArry[6];

 uint8 limit=55;
 int lspeed=0,rspeed=0;
 uint8 img[Img_H][Img_W];
 uint8 imgbuff[CAMERA_SIZE]={1};   
 int imglen;
 extern  Point  MidLine[Img_H];

 int Lspeed,Rspeed;
 int Rpulse=0,Lpulse=0; //左右编码器
 int16 LPulseSum=0,RPulseSum=0;//编码器累积值
 int PIT0InteruptEventCount=0;

 extern uint8 t;        //直立模糊表下标
 float Upright_Kp[5]={15, 16.0, 16.5,13.5, 5};//直立模糊PID 16.5  15  20  10 5
 float Upright_Kd[5]={8, 8.0, 5, 4, 0};      //   11.3 8  5  4  0
 float SpeedKp=4.5;     //速度PID   4
 float SpeedKi=0.12;     //  0.4
 float DirKp=1.5;        //方向PID
 float DirKd=0.6;
 float DirKp2=2.3;
 
 int CarRate=35;  
 int HighSpeed=50;
 int LowSpeed=30;
 
 signed int dirdiff[3]={0};
 float dir[3]={0};
 float dir_next=0;
 int dirref=46;//方向的标准值
 int dircounter=0;
 float var[6];
 uint8 KeyFree=0;       //检测按键松开
 uint8 modle_key=0;
 uint8 LCDShowMode=0;
 
 extern PID_t StraigthPID;      //直立PID
 extern PID_t SpeedPID;         //速度PID
 extern PID_t DirPID;           //方向PID
 extern Car_Info_t Car_Info;
 
 extern float Acc_Offset;       //加速度计偏移量
 extern float Gyro_Offset;	 //陀螺仪偏移量 
 extern float Angle_Smp;        //角度采集
 extern float Angle_dot_Smp;    //角速度采集
  
 extern float Angle;            //角度
 extern float Angle_dot;   
 extern float EX_Angle;
 extern float EX_DeltaAngle;
 extern int CarSpeed;
 
  Site_t site_x_1 = {7,10};
  Site_t site_y_1 = {7,30};
  Site_t site_z_1 = {7,50};
  Site_t site_x_2 = {48,10};
  Site_t site_y_2 = {48,30};
  Site_t site_z_2 = {48,50};
  Site_t site_w_1 = {24,70};
  Site_t site_w_2 ={32,70};
  Site_t site1     = {80, 90};
  Site_t site2     = {40, 90};
  

void PIT0_IRQHandler(void);
void ParameterSet();
void PORTA_IRQHandler();
void DMA0_IRQHandler();

void AllInit()
{
    CarInit();
    Parameters_Init();  
    LCD_init();
    camera_init(imgbuff); 
    SCCB_WriteByte(OV7725_CNST,limit);          //设置阈值
    ftm_quad_init(FTM1);     
    ftm_quad_init(FTM2); 
    motor_init();
    uart_init(UART4,115200);
    gpio_init(PTC15,GPO,0);
    
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置LPTMR的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置LPTMR的中断服务函数为 PORTA_IRQHandler 
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); 
    
    enable_irq(PIT0_IRQn);
    pit_init_ms(PIT0,1);                //初始化PIT0，定时时间为： 5ms
    EnableInterrupts;                    //中断允许   
}

void main()
{
    int i;
    Site_t site     = {0, 0};                           //显示图像左上角位置
    Site_t midline[60];
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
    Size_t size;                   //显示区域图像大小
    size.H = CAMERA_H;
    size.W = CAMERA_W;
    
    AllInit();                
    
    while(1)
    { 
      /*//发送数据到上位机
        var[0] = Car_Info.Speed_PWM;       //Car_Info.Acc_Smp;
        var[1] = CarSpeed;
        var[2]=SpeedPID.SetPoint;
        //var[2] = Angle_Smp;
        var[3] = Angle_dot_Smp;
        var[4] = -Angle;                //最终角度
        var[5] =  Angle_dot;            //最终角速度
        //vcan_sendware((uint8_t *)var, sizeof(var));//上位机发送函数
        */
       // motor_control(lspeed,rspeed);
        gpio_set(PTC15,1);
        camera_get_img();       //摄像头获取图像到imgbuff[CAMERA_SIZE]   //用时10ms
        //vcan_sendimg(imgbuff,CAMERA_SIZE);
        imglen=ImageProcess();  //图像处理程序
        gpio_set(PTC15,0); 
        
        dirdiff[0]=0;
        dirdiff[1]=0;
        dirdiff[2]=0;
        for(i=1;i<imglen;i++)
        {
          midline[i].x=MidLine[imglen-i].x;
          midline[i].y=Img_H-(imglen-i);
          dirdiff[i/20]+=(MidLine[imglen-i].x-dirref);
        }
       
        dir[0]=(1.5*dirdiff[0]+0.5*dirdiff[1])/20;      //确定方向PID参数
        dir[1]=(0.5*dirdiff[0]+1.5*dirdiff[1])/20;
        dir[2]=(0.8*dirdiff[1]+1.2*dirdiff[2])/20;
        dircounter=0;
        
        if(imglen<10)
        {
          dir[0]=0;
          CarRate=0;
        }
        else CarRate=HighSpeed;
        
        if(imglen<30)DirPID.Kp=DirKp2;
        else DirPID.Kp=DirKp;
        
        //显示信息参数
       if(key_check(KEY_B) == KEY_DOWN)LCDShowMode=1;//修改显示的内容
       if(LCDShowMode)
       {
         if(key_check(KEY_U) == KEY_DOWN)
         {
           limit++;
           SCCB_WriteByte(OV7725_CNST,limit); 
         }
         else if(key_check(KEY_D) == KEY_DOWN)
         {
           limit--;
           SCCB_WriteByte(OV7725_CNST,limit);          //设置阈值
         }
          LCD_Img_Binary_Z(site, size, imgbuff, imgsize);
          LCD_points(midline,imglen,RED);
          LCD_num_C(site1,imglen,FCOLOUR,BCOLOUR);
          LCD_num_C(site2,limit,FCOLOUR,BCOLOUR);
       }
       else{
         ParameterSet();//屏幕按键调参  
         vcan_sendimg(imgbuff,CAMERA_SIZE);
       }

       
    }

}

uint8 EncoderLearnPeriod=0;
uint8 SpeedControlPeriod=0;
uint8 DirPeriod=0;
float SpeedChange=0;
void PIT0_IRQHandler(void)//1ms进一次中断
{       
  AngleAcceleration_AD (AngleAcceleArry);  //陀螺仪和加速度计的采集
  
  acc_init[PIT0InteruptEventCount]=AngleAcceleArry[2];//加速度计//z轴
  gyro_init[PIT0InteruptEventCount]=AngleAcceleArry[5];//陀螺仪 
  
  if(PIT0InteruptEventCount==4)   //直立的控制
  {
    Straigth();//直立控制
    motor_control(Car_Info.Upright_PWM - Car_Info.Speed_PWM - 1.2*Car_Info.DirPWM,
                  Car_Info.Upright_PWM - Car_Info.Speed_PWM + Car_Info.DirPWM);
    //motor_control(Car_Info.Upright_PWM  - Car_Info.DirPWM,
      //            Car_Info.Upright_PWM  + Car_Info.DirPWM);
   // motor_control(-Car_Info.Speed_PWM,-Car_Info.Speed_PWM);
  }  
  else if(PIT0InteruptEventCount==3)   //陀螺仪
  {
     ReadSensorData();
     Complement_Filter_Ex((float)(Acc_Offset-Angle_Smp), 
                      (float)(Angle_dot_Smp - Gyro_Offset));//角度 角速度
     Car_Info.Angle=Angle;              //角度
     Car_Info.Angle_dot=Angle_dot;     //角速度
  }   
     
  else if(PIT0InteruptEventCount==2)   //编码器
  {
      Rpulse=-ftm_quad_get(FTM1);    ftm_quad_clean(FTM1);
      Lpulse=ftm_quad_get(FTM2);     ftm_quad_clean(FTM2);
      LPulseSum+=Lpulse;             RPulseSum+=Rpulse;
      SpeedControlPeriod++;
      if(SpeedControlPeriod>=20)//100ms执行一次速度控制
      {
        SpeedPID.SetPoint=CarRate;//设置车速期望
        SpeedControl();
        SpeedChange=Car_Info.NewSpeed-Car_Info.OldSpeed;
        SpeedControlPeriod=0;
      }
    Car_Info.Speed_PWM=(int)(Car_Info.OldSpeed + (SpeedControlPeriod+1) * SpeedChange/20);
  }
  else if(PIT0InteruptEventCount==1)
  {
   
     Car_Info.DirPWM=(int )LocPID_Calc(dir[dircounter],&DirPID); 
     if(dircounter<2)
       dircounter++;
  }
  else if(PIT0InteruptEventCount==0)
  {
    
  }
  else PIT0InteruptEventCount=0;
  
  PIT0InteruptEventCount++;        // 0--4循环
  if( PIT0InteruptEventCount==5)
      PIT0InteruptEventCount=0;
       
  PIT_Flag_Clear(PIT0);//清中断标志位
}


void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
      
    }
#endif


}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
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
    modle_key %=6;
    //调整参数
    switch(modle_key)
    {
    case 0://直立kp
      {
        if(key_check(KEY_U) == KEY_DOWN)
          Upright_Kp[t]+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           Upright_Kp[t]-=0.1;
        
        LCD_num_C (site_x_1, (int)( Upright_Kp[t]*10) , FCOLOUR , GREEN);
        LCD_num_C (site_y_1, (int)( Upright_Kd[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.SetPoint) , FCOLOUR , BCOLOUR);
        break;
      }
    case 1://直立kd
      {
        if(key_check(KEY_U) == KEY_DOWN)
           Upright_Kd[t]+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
            Upright_Kd[t]-=0.1;
        LCD_num_C (site_x_1, (int)( Upright_Kp[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( Upright_Kd[t]*10) , FCOLOUR , GREEN);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.SetPoint) , FCOLOUR , BCOLOUR);
        break;
      }
    case 2://加速度offset
      {
        if(key_check(KEY_U) == KEY_DOWN)
          Acc_Offset++;
        else if(key_check(KEY_D) == KEY_DOWN)
           Acc_Offset--;
        LCD_num_C (site_x_1, (int)( Upright_Kp[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( Upright_Kd[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , GREEN);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.SetPoint) , FCOLOUR , BCOLOUR);
        break;
      }
    case 3://速度kp
      {
        if(key_check(KEY_U) == KEY_DOWN)
          SpeedPID.Kp+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           SpeedPID.Kp-=0.1;        
        LCD_num_C (site_x_1, (int)( Upright_Kp[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( Upright_Kd[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , GREEN);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.SetPoint) , FCOLOUR , BCOLOUR);
        break;
      }
    case 4://直立ki
      {
        if(key_check(KEY_U) == KEY_DOWN)
          SpeedPID.Ki+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           SpeedPID.Ki-=0.1;
        LCD_num_C (site_x_1, (int)( Upright_Kp[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( Upright_Kd[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , GREEN);
        LCD_num_C (site_z_2, (int)(SpeedPID.SetPoint) , FCOLOUR , BCOLOUR);
        break;
      }
    case 5://直立kd
      {
        if(key_check(KEY_U) == KEY_DOWN)
          CarRate += 5;
        else if(key_check(KEY_D) == KEY_DOWN)
          CarRate -= 5;
        LCD_num_C (site_x_1, (int)( Upright_Kp[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( Upright_Kd[t]*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.SetPoint) , FCOLOUR , GREEN);
        break;
      }
    }
  }
  //避免连按
    if((key_check(KEY_U) == KEY_UP) &&(key_check(KEY_D) == KEY_UP) \
          && (key_check(KEY_R) == KEY_UP) && (key_check(KEY_L) == KEY_UP))
      KeyFree=1;//按键已松开
  else KeyFree=0;
 
}

