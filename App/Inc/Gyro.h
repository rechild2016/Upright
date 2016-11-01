#ifndef __GYRO_H__
#define __GYRO_H__
#include "common.h"
#include  "MK60_adc.h" 

#define XOUT    ADC0_DM0
#define YOUT    ADC0_SE16
#define ZOUT    ADC0_SE17

#define Gyro1   ADC1_SE16
#define Gyro2   ADC0_DP0
#define Ang     ADC0_SE18

void AngleAcceleration_AD (uint16 *AdColle);
void ReadSensorData();
void Parameters_Init();
void Complement_Filter_Ex(float angle,float gyro);     //»¥²¹ÂË²¨µÄ³ÌÐò

#endif
