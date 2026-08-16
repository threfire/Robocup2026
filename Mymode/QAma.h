#ifndef __QAMA_H
#define __QAMA_H

#include "main.h"

typedef struct {
	uint8_t R;
	uint8_t G;
	uint8_t B;
	uint8_t color;
	uint16_t H;
	uint8_t S;
	uint8_t V;
	uint32_t sequence;
}Scan_information;

extern volatile Scan_information scan_information;
extern int detected_colors[5];

// 任务 1 二维码到移动方案映射
extern int task1_scheme[16][5];
extern int task2_scheme[6][3];
void Update_Materialcolor(void);
void Get_Materialcolor (uint8_t ch);

#endif
