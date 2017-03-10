#include "common.h"
#include "include.h"

 uint16 acc_init[6];
 uint16 gyro_init[6];
 uint16 AngleAcceleArry[6];
 float var[6];
 int Rpulse=0,Lpulse=0; //���ұ�����
 
 extern float Acc_Smp;        //���ٶ�����
 extern float Gyro_Smp;       //����������
 extern float Acc_Offset;		//���ٶȼ�ƫ����
 extern float Gyro_Offset;	//������ƫ����
 extern float Angle_Smp;        //�ǶȲɼ�
 extern float Angle;         //�Ƕ�
 extern float Angle_dot;   
 extern float EX_Angle;
 extern float EX_DeltaAngle;
 extern float Angle_dot_Smp;  //���ٶȲɼ�
 
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
    adc_init(Gyro1);         //�Ǽ��ٶ�  Angular1
    adc_init(Gyro2);  
    adc_init(ANGLE);
    ftm_quad_init(FTM1); 
    ftm_quad_init(FTM2); 
    motor_init();
    Parameters_Init();  
    
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);
    
    uart_init(UART4,115200);
    enable_irq(PIT0_IRQn);
    pit_init_ms(PIT0, 1);                //��ʼ��PIT0����ʱʱ��Ϊ�� 5ms
    EnableInterrupts;                    //�ж�����   
    while(1)
    { 
        DELAY_MS(10);
        var[0] = EX_Angle;       //Car_Info.Acc_Smp;
        var[1] = EX_DeltaAngle;
        var[2] = Angle_Smp;
        var[3] = Angle_dot_Smp;
        var[4] = -Angle;
        var[5] =  Angle_dot;
        vcan_sendware((uint8_t *)var, sizeof(var));//��λ�����ͺ���
    }

}

void PIT0_IRQHandler(void)//1ms��һ���ж�
{       
  AngleAcceleration_AD (AngleAcceleArry);  //�����Ǻͼ��ٶȼƵĲɼ�
  
  acc_init[PIT0InteruptEventCount]=AngleAcceleArry[2];//���ٶȼ�//z��
  gyro_init[PIT0InteruptEventCount]=AngleAcceleArry[5];//������ 
  if(PIT0InteruptEventCount==5)   //ֱ���Ŀ���
  {
          ReadSensorData();
          Complement_Filter_Ex((float)(Acc_Offset-Angle_Smp), 
                            (float)(Angle_dot_Smp - Gyro_Offset));//�Ƕ� ���ٶ�
  }
  else if(PIT0InteruptEventCount==4)   //������
  {
      Rpulse=-ftm_quad_get(FTM1);//����װ
      ftm_quad_clean(FTM1);
      Lpulse=ftm_quad_get(FTM2);
      ftm_quad_clean(FTM2);
    
  }
  else if(PIT0InteruptEventCount==3)   //������
  {
    motor_control(Lmotor,Lspeed);
    motor_control(Rmotor,Rspeed);
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

