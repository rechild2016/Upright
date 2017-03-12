#include "common.h"
#include "include.h"

 uint16 acc_init[6];
 uint16 gyro_init[6];
 uint16 AngleAcceleArry[6];
 uint8 KeyFree=0;       //��ⰴ���ɿ�
 float var[6];
 float StraigthKp=2.5;//ֱ��PID
 float StraigthKd=4.0;
 int Rpulse=0,Lpulse=0; //���ұ�����
 int Lspeed,Rspeed;
 
 extern PID_t StraigthPID;
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
    pit_init_ms(PIT0,1);                //��ʼ��PIT0����ʱʱ��Ϊ�� 5ms
    EnableInterrupts;                    //�ж�����   
    
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
        vcan_sendware((uint8_t *)var, sizeof(var));//��λ�����ͺ���
        ParameterSet();
    }

}

void PIT0_IRQHandler(void)//1ms��һ���ж�
{       
  AngleAcceleration_AD (AngleAcceleArry);  //�����Ǻͼ��ٶȼƵĲɼ�
  acc_init[PIT0InteruptEventCount]=AngleAcceleArry[2];//���ٶȼ�//z��
  gyro_init[PIT0InteruptEventCount]=AngleAcceleArry[5];//������ 
  
  if(PIT0InteruptEventCount==5)   //ֱ���Ŀ���
  {
          Straigth();
          motor_control(Car_Info.Upright_PWM,Car_Info.Upright_PWM);
  }  
  else if(PIT0InteruptEventCount==4)   //������
  {
     ReadSensorData();
     Complement_Filter_Ex((float)(Acc_Offset-Angle_Smp), 
                      (float)(Angle_dot_Smp - Gyro_Offset));//�Ƕ� ���ٶ�
     Car_Info.Angle=EX_Angle;              //�Ƕ�
     Car_Info.Angle_dot=EX_DeltaAngle;     //���ٶ�
  }   
     
  else if(PIT0InteruptEventCount==3)   //������
  {
      Rpulse=-ftm_quad_get(FTM2);//����װ
      ftm_quad_clean(FTM2);
      Lpulse=ftm_quad_get(FTM1);
      ftm_quad_clean(FTM1);
    
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
    modle_key %=3;
    //��������
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
      KeyFree=1;//�������ɿ�
  else KeyFree=0;
  
    //LCD_num_C (site_z_1, Car_Info.Upright_PWM , FCOLOUR , BCOLOUR);
}

