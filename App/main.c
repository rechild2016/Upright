#include "common.h"
#include "include.h"
/**************************************************

���ĳ��� ���˱��Ϻ�ͣ��
******************************************************/
 #define ImgMap(x,y) ((img[(x)][(y)/8])>>(7-(y)%8))&0x01
#define Img_H 60
#define Img_W 20


 uint8 limit=64;
 int lspeed=0,rspeed=0;

 uint8 imgbuff[CAMERA_SIZE]={1};   
 uint8 img[60][20]={0};

 extern uint8 t;        //ֱ��ģ�����±�
 float Upright_Kp[5]={16, 16.3, 16.5,13.5, 5};//ֱ��ģ��PID 16.5  15  20  10 5
 float Upright_Kd[5]={ 8,  8.0,    5,   4, 0};      //   11.3 8  5  4  0
 float SpeedKp=7.0;     //�ٶ�PID   4
 float SpeedKi=0.12;     //  0.4
 float DirKp=2.90;        //����PID
 float DirKp2=2.90;        //����PID
 float DirKd=40;
 float DirSetPoint=80;        //С������ƫ  ��������ƫ
   
 float CarRate=0;  
 int HighSpeed=42;
 int LowSpeed=40;
 int CarGo=0;
 
 extern int midpoint_before;

  extern unsigned char ConGraph[Img_H][Img_W];
 int Lspeed,Rspeed;
 int Rpulse=0,Lpulse=0; //���ұ�����
 int16 LPulseSum=0,RPulseSum=0;//�������ۻ�ֵ
 int PIT0InteruptEventCount=0;
 
  uint16 acc_init[5];
 uint16 gyro_init[5];
 uint16 AngleAcceleArry[6];
 
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
  Site_t site2     = {7, 30};
  Size_t imgsize  = {CAMERA_W, CAMERA_H};             //ͼ���С
  Size_t size_2   = {CAMERA_W*0.8, CAMERA_H};
  Site_t site_1   = {0, 60};  

void PIT0_IRQHandler(void);
void ParameterSet();
void PORTA_IRQHandler();
void DMA0_IRQHandler();

int BlockModeCounter=0;//�ϰ�������
int ZebraModeCounter=0;//�����߼�����
void AllInit()
{
    led_init(LED0);led_init(LED1);
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

Site_t site     = {0, 0};                           //��ʾͼ�����Ͻ�λ��
uint8 Stop=0;
int Stopt=0;
int RoadMode=0;
int CarStop=0;

void main()
{
    
    uint32  i=0,j=0,x=0,y=0,tt=0;
    int x0;
    AllInit();                
    
    while(1)
    { 
   
        gpio_set(PTC15,1);
        camera_get_img();       //����ͷ��ȡͼ��imgbuff[CAMERA_SIZE]   //��ʱ10ms
         for(i=0;i<Img_H;i++)
          for(j=0;j<Img_W;j++)
            img[i][j]=imgbuff[i*20+j];
         
        memset(ConGraph, 0, 1200);
        for(i=0;i<10;i++)
        {
          x0=CarpetSearch(59-i);
          if(x0>10 && x0<150)
          {
            DFS(59-i,(x0+7)/8);
            break;
          }
       }
       
       for(i=0;i<60;i++)//����ͨ�������
        {
          for(j=0;j<20;j++)
          {
            if(ConGraph[i][j]==0)
            {
              img[i][j]=0xff; 
              
            }
            imgbuff[i*20+j]=img[i][j];
          }
        }
        img_extract((uint8 *) ImageBuffer2, (uint8 *) imgbuff, 60*20);//��ѹͼ��
        ImageBuffer2[59][0]=0;
        for(i=0;i<PROW;i++)//��  ͼ��У��
        {
            for(j=0;j<PCOLUMN;j++)//��
            {
                x=mapX[i][j];
                y=59-mapY[i][j];
                processBuf1[59-i][j]=*(pProcess+y*SCOLUMN+x);
            }
        }
       
        imageProcess(p1);//ͼ����
       
        gpio_set(PTC15,0); 
        if(JudgeMode==4)
        { 
          if( RoadMode!=4)//������
            RoadMode=4;
          else if(CarStop==1)
          {
           Stopt=1;
          }
        }
        else RoadMode=0;
        if(Stopt>0)
        {
          if(Stopt<60)Stopt++;
          else{
            STOP=1;//����ͣ��
          }
        }
        //��ʾ��Ϣ����
       if(key_check(KEY_B) == KEY_DOWN)
       {       
         LCDShowMode=!LCDShowMode;//�޸���ʾ������
         LCD_clear(RED);
         DELAY_MS(200); 
       }
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
         LCD_Img_gray_Z(site_1, size_2, (uint8*)p1, imgsize);//��ʾ��ѹ��ĳ�ʼͼ��
         LCD_line_display(site_1);
         LCD_num_C(site2,leadYEnd-leadYStart,FCOLOUR,BCOLOUR);
         CarGo=1;
       }
       else{
         ParameterSet();//��Ļ��������  
         
       }
        if(CarGo!=0)
        {  
          if(CarRate<HighSpeed)
            CarRate+=0.5;
          if(tt<600 )tt++;
            else CarStop=1;//����ͣ��
          
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
    if(STOP==1)motor_control(0,0);
    else{
    Straigth();//ֱ������
    motor_control(Car_Info.Upright_PWM - Car_Info.Speed_PWM - Car_Info.DirPWM,
                  Car_Info.Upright_PWM - Car_Info.Speed_PWM + Car_Info.DirPWM);
   // motor_control(Car_Info.Upright_PWM  - Car_Info.DirPWM,
    //              Car_Info.Upright_PWM  + Car_Info.DirPWM);
   // motor_control(Car_Info.Upright_PWM,Car_Info.Upright_PWM);
    }
  }  
  else if(PIT0InteruptEventCount==3)   //������
  {
     ReadSensorData();
     Complement_Filter_Ex((float)(Acc_Offset-Angle_Smp), 
                      (float)(Angle_dot_Smp - Gyro_Offset));//�Ƕ� ���ٶ�
     Car_Info.Angle=Angle;              //�Ƕ�
     Car_Info.Angle_dot=Angle_dot;     //���ٶ�
  }   
     
  else if(PIT0InteruptEventCount==2)   //�ٶȻ��������
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
  
  else if(PIT0InteruptEventCount==1)//����
  {
    if(LineType==1)DirPID.Kp=DirKp;//ֱ��
    else DirPID.Kp=DirKp2;//����

    DirPID.SetPoint=DirSetPoint;
     
    Car_Info.DirPWM=(int )LocPID_Calc(midpoint_before,&DirPID); 

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
          DirKp+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           DirKp-=0.1;
        
        LCD_num_C (site_x_1, (int)( DirKp*10) , FCOLOUR , GREEN);
        LCD_num_C (site_y_1, (int)( DirKp2*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)DirSetPoint , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(HighSpeed) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(Acc_Offset) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(limit) , FCOLOUR , BCOLOUR);
        break;
      }
    case 1://ֱ��kd
      {
        if(key_check(KEY_U) == KEY_DOWN)
           DirKp2+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
            DirKp2-=0.1;
        LCD_num_C (site_x_1, (int)( DirKp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( DirKp2*10) , FCOLOUR , GREEN);
        LCD_num_C (site_z_1, (int)  DirSetPoint , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(HighSpeed) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(Acc_Offset) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(limit) , FCOLOUR , BCOLOUR);
        break;
      }
    case 2://���ٶ�offset
      {
        if(key_check(KEY_U) == KEY_DOWN)
          DirSetPoint++;
        else if(key_check(KEY_D) == KEY_DOWN)
           DirSetPoint--;
        LCD_num_C (site_x_1, (int)( DirKp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( DirKp2*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)DirSetPoint , FCOLOUR , GREEN);
        LCD_num_C (site_x_2, (int)(HighSpeed) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(Acc_Offset) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(limit) , FCOLOUR , BCOLOUR);
        break;
      }
    case 3://����kp
      {
        if(key_check(KEY_U) == KEY_DOWN)
          HighSpeed++;
        else if(key_check(KEY_D) == KEY_DOWN)
          HighSpeed--;        
        LCD_num_C (site_x_1, (int)( DirKp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( DirKp2*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)DirSetPoint , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(HighSpeed) , FCOLOUR ,GREEN);
        LCD_num_C (site_y_2, (int)(Acc_Offset) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(limit) , FCOLOUR , BCOLOUR);
        break;
      }
    case 4://����kd
      {
        if(key_check(KEY_U) == KEY_DOWN)
          Acc_Offset++;
        else if(key_check(KEY_D) == KEY_DOWN)
           Acc_Offset--;
        LCD_num_C (site_x_1, (int)( DirKp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( DirKp2*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int) DirSetPoint , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(HighSpeed) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(Acc_Offset) , FCOLOUR , GREEN);
        LCD_num_C (site_z_2, (int)(limit) , FCOLOUR , BCOLOUR);
        break;
      }
    case 5://����setpoint
      {
        if(key_check(KEY_U) == KEY_DOWN)
          limit++;
        else if(key_check(KEY_D) == KEY_DOWN)
          limit--;
        LCD_num_C (site_x_1, (int)( DirKp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)( DirKp2*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int) DirSetPoint , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(HighSpeed) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(Acc_Offset) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(limit) , FCOLOUR , GREEN);
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


