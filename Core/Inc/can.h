/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"

/**
 * @brief CAN 接收环形队列槽位数量。
 */
#define CAN_RX_QUEUE_SIZE 16U

/**
 * @brief CAN 接收帧缓存。
 */
typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
} CAN_RxFrame_t;

typedef struct {
	__IO CAN_RxFrame_t rxQueue[CAN_RX_QUEUE_SIZE];
	__IO uint8_t rxQueueHead;
	__IO uint8_t rxQueueTail;
	
	__IO CAN_TxHeaderTypeDef CAN_TxMsg;
	__IO uint8_t txData[32];
}CAN_t;
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
extern __IO CAN_t can;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void USER_CAN1_Filter_Init(void);
void can_SendCmd(__IO uint8_t *cmd, uint8_t len);

/**
 * @brief 接收 FIFO0 中的全部 CAN 帧并写入环形队列。
 */
void can_ReceiveFifo0(void);

/**
 * @brief 读取一帧 CAN 接收数据。
 *
 * @param rx_header CAN 帧头输出地址。
 * @param frame_data CAN 数据输出地址，长度为 8 字节。
 * @return 读取到数据时返回 true。
 */
bool can_GetRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *frame_data);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

