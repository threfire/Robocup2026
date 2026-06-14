#ifndef __GRAYSENSOR_H
#define __GRAYSENSOR_H

#include "stm32f4xx.h"                  // Device header

extern unsigned char Anolog[8];

void Graysensor_Init(void);
unsigned char Graysensor_Getdata(void);

#endif
