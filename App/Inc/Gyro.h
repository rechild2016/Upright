#ifndef __GYRO_H__
#define __GYRO_H__
#include "common.h"
#include  "MK60_adc.h" 

#define XOUT    ADC1_DM0
#define YOUT    ADC0_SE16
#define ZOUT    ADC0_SE17//PTE24
#define Gyro1   ADC1_SE16//D3
#define Gyro2   ADC1_DP0
#define ANGLE   ADC0_SE18//PTE25

void AngleAcceleration_AD (uint16 *AdColle);
void ReadSensorData();
void Parameters_Init();
void Complement_Filter_Ex(float angle,float gyro);     //»¥²¹ÂË²¨µÄ³ÌÐò

#endif
