#ifndef __HWT101_H
#define __HWT101_H

#include "main.h"

uint8_t IsIMU_Start(void);
extern float absangle;
void HWT_Init(void);
void ParseHWT101Yaw(void);
float GetYaw(void);
extern float fGyro[3]; 
extern float pos_x;
extern float pos_y;
extern float zangle;
extern float xangle;
extern float yangle;
extern float w_z;

void Data_Analyse(uint8_t rec);
void Update_zero(void);
#endif
