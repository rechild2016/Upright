#include "common.h"
#include "include.h"

 uint16 acc_init[6];
 uint16 gyro_init[6];
 uint16 AngleAcceleArry[6];
 uint8 KeyFree=0;       //��ⰴ���ɿ�
 float var[6];

 int Rpulse=0,Lpulse=0; //���ұ�����
 int Lspeed,Rspeed;
 int32 LPulseSum=0,RPulseSum=0;//�������ۻ�ֵ
 
 float Straigth_Kp=5.0;//ֱ��PID
 float Straigth_Kd=30.0;
 float SpeedKp=0.0;
 float SpeedKi=0.0;
 extern PID_t StraigthPID;
 extern PID_t SpeedPID;
 extern Car_Info_t Car_Info;
 
 extern float Acc_Offset;       //���ٶȼ�ƫ����
 extern float Gyro_Offset;	 //������ƫ���� 
 extern float Angle_Smp;        //�ǶȲɼ�
 extern float Angle_dot_Smp;    //���ٶȲɼ�
  
 extern float Angle;            //�Ƕ�
 extern float Angle_dot;   
 extern float EX_Angle;
 extern float EX_DeltaAngle;
 
  Site_t site_x_1 = {7,10};
  Site_t site_y_1 = {7,30};
  Site_t site_z_1 = {7,50};
  Site_t site_x_2 = {48,10};
  Site_t site_y_2 = {48,30};
  Site_t site_z_2 = {48,50};
  Site_t site_w_1 = {24,70};
  Site_t site_w_2 ={32,70};
  uint8 modle_key=0;
  
 void vcan_sendware(uint8 *wareaddr, uint32 waresize);

  int PIT0InteruptEventCount=0;

  void PIT0_IRQHandler(void);
  void ParameterSet();


void main()
{
    CarInit();
    GyroInit();          
    Parameters_Init();  
    LCD_init();
    ftm_quad_init(FTM1);     
    ftm_quad_init(FTM2); 
    motor_init();
    uart_init(UART4,115200);
     
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); 
    enable_irq(PIT0_IRQn);
    pit_init_ms(PIT0,1);                //��ʼ��PIT0����ʱʱ��Ϊ�� 5ms
    EnableInterrupts;                    //�ж�����   
    
    while(1)
    { 
        DELAY_MS(30);
      // motor_control(Lspeed,Rspeed);
        var[0] = Car_Info.Upright_PWM;       //Car_Info.Acc_Smp;
        var[1] = EX_DeltaAngle;
        var[2] = Angle_Smp;
        var[3] = Angle_dot_Smp;
        var[4] = -Angle;                //���սǶ�
        var[5] =  Angle_dot;            //���ս��ٶ�
        vcan_sendware((uint8_t *)var, sizeof(var));//��λ�����ͺ���
        ParameterSet();//��Ļ��������
        if(EX_Angle>=0)
          LCD_num_C (site_w_1, (int)(EX_Angle) , FCOLOUR , BCOLOUR);
        else 
        {
          LCD_char  (site_w_1, '-' , FCOLOUR , BCOLOUR);
          LCD_num_C (site_w_2, (int)(-EX_Angle) , FCOLOUR , BCOLOUR);
        }
    }

}

uint8 EncoderLearnPeriod=0;
uint8 SpeedControlPeriod=0;
void PIT0_IRQHandler(void)//1ms��һ���ж�
{       
  AngleAcceleration_AD (AngleAcceleArry);  //�����Ǻͼ��ٶȼƵĲɼ�
  acc_init[PIT0InteruptEventCount]=AngleAcceleArry[2];//���ٶȼ�//z��
  gyro_init[PIT0InteruptEventCount]=AngleAcceleArry[5];//������ 
  
  if(PIT0InteruptEventCount==5)   //ֱ���Ŀ���
  {
          Straigth();//ֱ������
          motor_control(Car_Info.Upright_PWM+Car_Info.Speed_PWM,
                          Car_Info.Upright_PWM+Car_Info.Speed_PWM);
  }  
  else if(PIT0InteruptEventCount==4)   //������
  {
     ReadSensorData();
     Complement_Filter_Ex((float)(Acc_Offset-Angle_Smp), 
                      (float)(Angle_dot_Smp - Gyro_Offset));//�Ƕ� ���ٶ�
     Car_Info.Angle=Angle;              //�Ƕ�
     Car_Info.Angle_dot=Angle_dot;     //���ٶ�
  }   
     
  else if(PIT0InteruptEventCount==3)   //������
  {
      Rpulse=-ftm_quad_get(FTM2);//����װ
      ftm_quad_clean(FTM2);
      Lpulse=ftm_quad_get(FTM1);
      ftm_quad_clean(FTM1);
      LPulseSum+=Lpulse;
      RPulseSum+=Rpulse;
      SpeedControlPeriod++;
      if(SpeedControlPeriod>=20)//100msִ��һ���ٶȿ���
      {
        SpeedControl();
        SpeedControlPeriod=0;
      }

  }
  
  PIT0InteruptEventCount++;
  if( PIT0InteruptEventCount==6)
      PIT0InteruptEventCount=0;
       
      PIT_Flag_Clear(PIT0);//���жϱ�־λ
}

void vcan_sendware(uint8 *wareaddr, uint32 waresize)
{
    #define CMD_WARE     3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //yy_����ͷ���ڵ��� ʹ�õ�����
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //yy_����ͷ���ڵ��� ʹ�õ�����
    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //�ȷ�������
    uart_putbuff(VCAN_PORT, wareaddr, waresize); //�ٷ���ͼ��
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //�ȷ�������
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
          Straigth_Kp+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           Straigth_Kp-=0.1;
        
        LCD_num_C (site_x_1, (int)(Straigth_Kp*10) , FCOLOUR , GREEN);
        LCD_num_C (site_y_1, (int)(Straigth_Kd*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.Kd*10) , FCOLOUR , BCOLOUR);
        break;
      }
    case 1://ֱ��kd
      {
        if(key_check(KEY_U) == KEY_DOWN)
          Straigth_Kd+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           Straigth_Kd-=0.1;
        LCD_num_C (site_x_1, (int)(Straigth_Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)(Straigth_Kd*10) , FCOLOUR , GREEN);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.Kd*10) , FCOLOUR , BCOLOUR);
        break;
      }
    case 2://���ٶ�offset
      {
        if(key_check(KEY_U) == KEY_DOWN)
          Acc_Offset++;
        else if(key_check(KEY_D) == KEY_DOWN)
           Acc_Offset--;
        LCD_num_C (site_x_1, (int)(Straigth_Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)(Straigth_Kd*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , GREEN);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.Kd*10) , FCOLOUR , BCOLOUR);
        break;
      }
    case 3://�ٶ�kp
      {
        if(key_check(KEY_U) == KEY_DOWN)
          SpeedPID.Kp+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           SpeedPID.Kp-=0.1;        
        LCD_num_C (site_x_1, (int)(Straigth_Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)(Straigth_Kd*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , GREEN);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.Kd*10) , FCOLOUR , BCOLOUR);
        break;
      }
    case 4://ֱ��ki
      {
        if(key_check(KEY_U) == KEY_DOWN)
          SpeedPID.Ki+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           SpeedPID.Ki-=0.1;
        LCD_num_C (site_x_1, (int)(Straigth_Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)(Straigth_Kd*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , GREEN);
        LCD_num_C (site_z_2, (int)(SpeedPID.Kd*10) , FCOLOUR , BCOLOUR);
        break;
      }
    case 5://ֱ��kd
      {
        if(key_check(KEY_U) == KEY_DOWN)
          SpeedPID.Kd+=0.1;
        else if(key_check(KEY_D) == KEY_DOWN)
           SpeedPID.Kd-=0.1;
        LCD_num_C (site_x_1, (int)(Straigth_Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_1, (int)(Straigth_Kd*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_1, (int)Acc_Offset , FCOLOUR , BCOLOUR);
        LCD_num_C (site_x_2, (int)(SpeedPID.Kp*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_y_2, (int)(SpeedPID.Ki*10) , FCOLOUR , BCOLOUR);
        LCD_num_C (site_z_2, (int)(SpeedPID.Kd*10) , FCOLOUR , GREEN);
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
