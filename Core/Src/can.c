/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
__IO CAN_t can = {0};

/** CAN 发送环形队列槽位数量。 */
#define CAN_TX_QUEUE_SIZE 128U

/** CAN 发送队列帧。 */
typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t data[8];
} CAN_TxFrame_t;

/** CAN 待发送帧队列。 */
static CAN_TxFrame_t can_tx_queue[CAN_TX_QUEUE_SIZE];
/** CAN 发送队列写入下标。 */
static volatile uint8_t can_tx_head;
/** CAN 发送队列读取下标。 */
static volatile uint8_t can_tx_tail;
/** CAN 发送队列满时累计丢弃的完整命令数量。 */
volatile uint32_t can_tx_overflow_count;

/** 尝试将队列中的待发帧填入空闲邮箱。 */
static void can_tx_kick(void) {
	uint32_t mailbox;

	while ((can_tx_tail != can_tx_head) &&
	        (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)) {
		CAN_TxFrame_t *frame = &can_tx_queue[can_tx_tail];

		if (HAL_CAN_AddTxMessage(&hcan1, &frame->header,
		            frame->data, &mailbox) != HAL_OK) {
			return;
		}

		can_tx_tail++;
		if (can_tx_tail >= CAN_TX_QUEUE_SIZE) {
			can_tx_tail = 0U;
		}
	}
}
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 7;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
	HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */
	HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief   初始化滤波器
 * @param   无
 * @retval  无
 */
void USER_CAN1_Filter_Init(void) {
	// 过滤器结构体
	CAN_FilterTypeDef sFilterConfig;

	// 设置STM32的帧ID - 扩展帧格式 - 不过滤任何数据帧
	__IO uint8_t id_o, im_o;
	__IO uint16_t id_l, id_h, im_l, im_h;
	id_o = (0x00);
	id_h = (uint16_t)((uint16_t)id_o >> 5);               // 高3位
	id_l = (uint16_t)((uint16_t)id_o << 11) | CAN_ID_EXT; // 低5位
	im_o = (0x00);
	im_h = (uint16_t)((uint16_t)im_o >> 5);
	im_l = (uint16_t)((uint16_t)im_o << 11) | CAN_ID_EXT;

	// 过滤器参数
	sFilterConfig.FilterBank = 0;                      // 过滤器1
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位过滤器位宽
	sFilterConfig.FilterIdHigh = id_h;     // 过滤器标识符的高16位值
	sFilterConfig.FilterIdLow = id_l;      // 过滤器标识符的低16位值
	sFilterConfig.FilterMaskIdHigh = im_h; // 过滤器屏蔽标识符的高16位值
	sFilterConfig.FilterMaskIdLow = im_l;  // 过滤器屏蔽标识符的低16位值
	sFilterConfig.FilterFIFOAssignment =
	        CAN_RX_FIFO0;                    // 指向过滤器的FIFO为0
	sFilterConfig.FilterActivation = ENABLE; // 使能过滤器
	sFilterConfig.SlaveStartFilterBank =
	        0; // 从过滤器配置，用来选择从过滤器的寄存器编号

	// 配置并自检
	while (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
		;
}

/**
 * @brief 接收 FIFO0 中的全部 CAN 帧并写入环形队列。
 */
void can_ReceiveFifo0(void) {
	CAN_RxHeaderTypeDef rx_header;
	uint8_t frame_data[8] = {0U};
	uint8_t queue_head;
	uint8_t next_head;
	uint8_t i;

	while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header,
		            frame_data) != HAL_OK) {
			return;
		}

		queue_head = can.rxQueueHead;
		next_head = queue_head + 1U;
		if (next_head >= CAN_RX_QUEUE_SIZE) {
			next_head = 0U;
		}
		if (next_head == can.rxQueueTail) {
			continue;
		}

		can.rxQueue[queue_head].header = rx_header;
		for (i = 0U; i < 8U; i++) {
			can.rxQueue[queue_head].data[i] = frame_data[i];
		}
		can.rxQueueHead = next_head;
	}
}

/**
 * @brief 从 CAN 接收环形队列中读取一帧数据。
 *
 * @param rx_header CAN 帧头输出地址。
 * @param frame_data CAN 数据输出地址，长度为 8 字节。
 * @return 读取到数据时返回 true。
 */
bool can_GetRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *frame_data) {
	uint8_t queue_tail;
	uint8_t next_tail;
	uint8_t i;

	if ((rx_header == NULL) || (frame_data == NULL)) {
		return false;
	}

	queue_tail = can.rxQueueTail;
	if (queue_tail == can.rxQueueHead) {
		return false;
	}

	*rx_header = can.rxQueue[queue_tail].header;
	for (i = 0U; i < 8U; i++) {
		frame_data[i] = can.rxQueue[queue_tail].data[i];
	}
	next_tail = queue_tail + 1U;
	if (next_tail >= CAN_RX_QUEUE_SIZE) {
		next_tail = 0U;
	}
	can.rxQueueTail = next_tail;

	return true;
}

/**
 * @brief 将命令按现有扩展帧协议写入非阻塞发送队列。
 *
 * @param cmd 命令数据，前两个字节依次为设备地址和功能码。
 * @param len 命令总字节数。
 */
void can_SendCmd(__IO uint8_t *cmd, uint8_t len) {
	uint8_t data_index = 0U;
	uint8_t data_length;
	uint8_t frame_count;
	uint8_t free_count;
	uint8_t pack_num = 0U;
	uint8_t primask;

	if ((cmd == NULL) || (len <= 2U)) {
		return;
	}

	data_length = (uint8_t)(len - 2U);
	frame_count = (uint8_t)((data_length + 6U) / 7U);
	primask = (uint8_t)__get_PRIMASK();
	__disable_irq();

	free_count = (can_tx_tail > can_tx_head) ?
	        (uint8_t)(can_tx_tail - can_tx_head - 1U) :
	        (uint8_t)(CAN_TX_QUEUE_SIZE - can_tx_head + can_tx_tail - 1U);
	if (free_count < frame_count) {
		can_tx_overflow_count++;
		if (primask == 0U) {
			__enable_irq();
		}
		return;
	}

	while (data_index < data_length) {
		CAN_TxFrame_t *frame = &can_tx_queue[can_tx_head];
		uint8_t payload_length = (uint8_t)(data_length - data_index);
		uint8_t i;

		if (payload_length > 7U) {
			payload_length = 7U;
		}
		frame->header.StdId = 0U;
		frame->header.ExtId = ((uint32_t)cmd[0] << 8U) | pack_num;
		frame->header.IDE = CAN_ID_EXT;
		frame->header.RTR = CAN_RTR_DATA;
		frame->header.DLC = (uint32_t)payload_length + 1U;
		frame->header.TransmitGlobalTime = DISABLE;
		frame->data[0] = cmd[1];
		for (i = 0U; i < payload_length; i++) {
			frame->data[i + 1U] = cmd[data_index + i + 2U];
		}
		data_index = (uint8_t)(data_index + payload_length);
		pack_num++;
		can_tx_head++;
		if (can_tx_head >= CAN_TX_QUEUE_SIZE) {
			can_tx_head = 0U;
		}
	}

	can_tx_kick();
	if (primask == 0U) {
		__enable_irq();
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		can_tx_kick();
	}
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		can_tx_kick();
	}
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		can_tx_kick();
	}
}

/* USER CODE END 1 */
