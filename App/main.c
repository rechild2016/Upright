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
 int Rpulse=0,Lpulse=0; //���ұ�����
 int16 LPulseSum=0,RPulseSum=0;//�������ۻ�ֵ
 int PIT0InteruptEventCount=0;

 extern uint8 t;        //ֱ��ģ�����±�
 float Upright_Kp[5]={15, 16.0, 16.5,13.5, 5};//ֱ��ģ��PID 16.5  15  20  10 5
 float Upright_Kd[5]={8, 8.0, 5, 4, 0};      //   11.3 8  5  4  0
 float SpeedKp=4.5;     //�ٶ�PID   4
 float SpeedKi=0.12;     //  0.4
 float DirKp=1.5;        //����PID
 float DirKd=0.6;
 float DirKp2=2.3;
 
 int CarRate=35;  
 int HighSpeed=50;
 int LowSpeed=30;
 
 signed int dirdiff[3]={0};
 float dir[3]={0};
 float dir_next=0;
 int dirref=46;//����ı�׼ֵ
 int dircounter=0;
 float var[6];
 uint8 KeyFree=0;       //��ⰴ���ɿ�
 uint8 modle_key=0;
 uint8 LCDShowMode=0;
 
 extern PID_t StraigthPID;      //ֱ��PID
 extern PID_t SpeedPID;         //�ٶ�PID
 extern PID_t DirPID;           //����PID
 extern Car_Info_t Car_Info;
 
 extern float Acc_Offset;       //���ٶȼ�ƫ����
 extern float Gyro_Offset;	 //������ƫ���� 
 extern float Angle_Smp;        //�ǶȲɼ�
 extern float Angle_dot_Smp;    //���ٶȲɼ�
  
 extern float Angle;            //�Ƕ�
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
    SCCB_WriteByte(OV7725_CNST,limit);          //������ֵ
    ftm_quad_init(FTM1);     
    ftm_quad_init(FTM2); 
    motor_init();
    uart_init(UART4,115200);
    gpio_init(PTC15,GPO,0);
    
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler 
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); 
    
    enable_irq(PIT0_IRQn);
    pit_init_ms(PIT0,1);                //��ʼ��PIT0����ʱʱ��Ϊ�� 5ms
    EnableInterrupts;                    //�ж�����   
}

void main()
{
    int i;
    Site_t site     = {0, 0};                           //��ʾͼ�����Ͻ�λ��
    Site_t midline[60];
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //ͼ���С
    Size_t size;                   //��ʾ����ͼ���С
    size.H = CAMERA_H;
    size.W = CAMERA_W;
    
    AllInit();                
    
    while(1)
    { 
      /*//�������ݵ���λ��
        var[0] = Car_Info.Speed_PWM;       //Car_Info.Acc_Smp;
        var[1] = CarSpeed;
        var[2]=SpeedPID.SetPoint;
        //var[2] = Angle_Smp;
        var[3] = Angle_dot_Smp;
        var[4] = -Angle;                //���սǶ�
        var[5] =  Angle_dot;            //���ս��ٶ�
        //vcan_sendware((uint8_t *)var, sizeof(var));//��λ�����ͺ���
        */
       // motor_control(lspeed,rspeed);
        gpio_set(PTC15,1);
        camera_get_img();       //����ͷ��ȡͼ��imgbuff[CAMERA_SIZE]   //��ʱ10ms
        //vcan_sendimg(imgbuff,CAMERA_SIZE);
        imglen=ImageProcess();  //ͼ�������
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
       
        dir[0]=(1.5*dirdiff[0]+0.5*dirdiff[1])/20;      //ȷ������PID����
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
        
        //��ʾ��Ϣ����
       if(key_check(KEY_B) == KEY_DOWN)LCDShowMode=1;//�޸���ʾ������
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
           SCCB_WriteByte(OV7725_CNST,limit);          //������ֵ
         }
          LCD_Img_Binary_Z(site, size, imgbuff, imgsize);
          LCD_points(midline,imglen,RED);
          LCD_num_C(site1,imglen,FCOLOUR,BCOLOUR);
          LCD_num_C(site2,limit,FCOLOUR,BCOLOUR);
       }
       else{
         ParameterSet();//��Ļ��������  
         vcan_sendimg(imgbuff,CAMERA_SIZE);
       }

       
    }

}

uint8 EncoderLearnPeriod=0;
uint8 SpeedControlPeriod=0;
uint8 DirPeriod=0;
float SpeedChange=0;
void PIT0_IRQHandler(void)//1ms��һ���ж�
{       
  AngleAcceleration_AD (AngleAcceleArry);  //�����Ǻͼ��ٶȼƵĲɼ�
  
  acc_init[PIT0InteruptEventCount]=AngleAcceleArry[2];//���ٶȼ�//z��
  gyro_init[PIT0InteruptEventCount]=AngleAcceleArry[5];//������ 
  
  if(PIT0InteruptEventCount==4)   //ֱ���Ŀ���
  {
    Straigth();//ֱ������
    motor_control(Car_Info.Upright_PWM - Car_Info.Speed_PWM - 1.2*Car_Info.DirPWM,
                  Car_Info.Upright_PWM - Car_Info.Speed_PWM + Car_Info.DirPWM);
    //motor_control(Car_Info.Upright_PWM  - Car_Info.DirPWM,
      //            Car_Info.Upright_PWM  + Car_Info.DirPWM);
   // motor_control(-Car_Info.Speed_PWM,-Car_Info.Speed_PWM);
  }  
  else if(PIT0InteruptEventCount==3)   //������
  {
     ReadSensorData();
     Complement_Filter_Ex((float)(Acc_Offset-Angle_Smp), 
                      (float)(Angle_dot_Smp - Gyro_Offset));//�Ƕ� ���ٶ�
     Car_Info.Angle=Angle;              //�Ƕ�
     Car_Info.Angle_dot=Angle_dot;     //���ٶ�
  }   
     
  else if(PIT0InteruptEventCount==2)   //������
  {
      Rpulse=-ftm_quad_get(FTM1);    ftm_quad_clean(FTM1);
      Lpulse=ftm_quad_get(FTM2);     ftm_quad_clean(FTM2);
      LPulseSum+=Lpulse;             RPulseSum+=Rpulse;
      SpeedControlPeriod++;
      if(SpeedControlPeriod>=20)//100msִ��һ���ٶȿ���
      {
        SpeedPID.SetPoint=CarRate;//���ó�������
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
  
  PIT0InteruptEventCount++;        // 0--4ѭ��
  if( PIT0InteruptEventCount==5)
      PIT0InteruptEventCount=0;
       
  PIT_Flag_Clear(PIT0);//���жϱ�־λ
}


void PORTA_IRQHandler()
{
    uint8  n;    //���ź�
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //ʹ�����ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
      
    }
#endif


}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}


//��Ļ�������ò���
void ParameterSet()
{
  if(KeyFree==1)
  {
    //ѡ��Ҫ�����Ĳ���
    if(key_check(KEY_R) == KEY_DOWN)
    {
        modle_key++;
    }
    else if(key_check(KEY_L)== KEY_DOWN)
    {
        modle_key--;
    }
    modle_key %=6;
    //��������
    switch(modle_key)
    {
    case 0://ֱ��kp
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
    case 1://ֱ��kd
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
    case 2://���ٶ�offset
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
    case 3://�ٶ�kp
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
    case 4://ֱ��ki
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
    case 5://ֱ��kd
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
  //��������
    if((key_check(KEY_U) == KEY_UP) &&(key_check(KEY_D) == KEY_UP) \
          && (key_check(KEY_R) == KEY_UP) && (key_check(KEY_L) == KEY_UP))
      KeyFree=1;//�������ɿ�
  else KeyFree=0;
 
}

