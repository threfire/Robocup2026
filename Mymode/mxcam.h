#ifndef __MXCAM_H
#define __MXCAM_H

#include "stm32f4xx.h" // Device header

#include <stdint.h>

typedef struct {
	int16_t x;
	int16_t y;
} Relative_coord;

extern Relative_coord Camera_err;

// 函数声明
void Process_usart1_packet(void);

#endif
