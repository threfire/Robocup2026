/* uart_manager.h */
#ifndef UART_MANAGER_H
#define UART_MANAGER_H

#include "main.h"
#include "stm32f4xx.h" // Device header

// 接收状态机状态定义
typedef enum {
	UART_WAIT_HEAD1,    // 等待第一个包头
	UART_WAIT_HEAD2,    // 等待第二个包头
	UART_RECEIVING_DATA // 接收数据阶段
} UART_State;

// 通用接收结构体（每个串口独立实例）
typedef struct {
	UART_State state;          // 当前状态机状态
	UART_HandleTypeDef *huart; // 绑定的串口句柄
	uint8_t buffer[9];         // 数据缓冲区（根据协议长度）
	uint8_t index;             // 当前缓冲位置
} UART_Receiver;

// 初始化函数
void UART_Manager_Init(void);
void DMA_Usart_Send(uint8_t *buf, uint8_t len);
void DMA_resend(void);
void Usart4_DMA_IDLE_Init(void);

#endif
