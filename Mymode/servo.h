#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

#define DJ_NUM 3 /* 舵机数量 */
extern uint8_t startServo;
typedef struct
{
    // uint8_t valid; // 有效 TODO
    uint16_t aim;  // 执行目标
    uint16_t time; // 执行时间
    float cur;     // 当前值
    float inc;     // 增量
} servo_t;

extern servo_t duoji_doing[DJ_NUM];

//相关函数声明*******//
void Servo_SetPulseWidth(uint16_t pulse_us);
void servo_init(void);
void duoji_doing_set(uint8_t index, int aim, int time); /* 设置舵机参数 */
void servo_inc_offset(uint8_t index);                  /* 设置舵机每次增加的偏移量 */
void servo_tick(void);
void jiazhuadongzuo(void);
void wuliaopanxuanzhuan(void);
void Servo_Action_Group(uint8_t num);
#endif
