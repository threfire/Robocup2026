/* uart_manager.c */
#include "usart_manager.h"
#include "lcd_spi_154.h"
#include "usart.h"
#include "mxcam.h" 
#include<string.h> 	// 包含各串口处理模块
  // 其他串口

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;


//

// 各串口接收器实例声明
static UART_Receiver usart3_receiver = {
    .state = UART_WAIT_HEAD1,
    .huart = &huart3,
    .index = 0
};

static UART_Receiver usart1_receiver = {
    .state = UART_WAIT_HEAD1,
    .huart = &huart1,
    .index = 0
};

/********************************************************
* 函数名：UART_Manager_Init
* 功能：初始化所有串口的接收中断
* 参数：无
* 说明：在main函数开始处调用
********************************************************/
void UART_Manager_Init(void) {
    // 初始化USART3
    HAL_UART_Receive_IT(usart3_receiver.huart, &usart3_receiver.buffer[0], 1);
    
    // 初始化USART1（示例）
    HAL_UART_Receive_IT(usart1_receiver.huart, &usart1_receiver.buffer[0], 1);
}

/********************************************************
* 函数名：HAL_UART_RxCpltCallback
* 功能：统一的中断回调函数（覆盖弱函数）
* 参数：huart-触发中断的串口句柄
* 说明：自动路由到对应串口的处理函数
********************************************************/


/*
*********************************************************************************************************
* 函 数 名: DMA_Usart_Send
* 功能说明: 串口发送功能函数
* 形  参: buf，len
* 返 回 值: 无
*********************************************************************************************************
*/
void DMA_Usart_Send(uint8_t *buf,uint8_t len)//串口发送封装
{
 if(HAL_UART_Transmit_DMA(&huart1, buf,len)!= HAL_OK)   //判断是否发送正常，如果出现异常则进入异常中断函数
  {
   Error_Handler();
  }

}



/*
*********************************************************************************************************
* 函 数 名: DMA_Usart1_Read
* 功能说明: 串口接收功能函数
* 形  参: Data,len
* 返 回 值: 无
*********************************************************************************************************
*/
void DMA_Usart1_Read(uint8_t *Data,uint8_t len)//串口接收封装
{
	HAL_UART_Receive_DMA(&huart1,Data,len);//重新打开DMA接收
}

void DMA_resend(void)
{
	if(recv_end_flag == 1)  //接收完成标志
		{
			
			
//			DMA_Usart_Send(rx_buffer, rx_len);
			rx_len = 0;//清除计数
			recv_end_flag = 0;//清除接收结束标志位
//			for(uint8_t i=0;i<rx_len;i++)
//				{
//					rx_buffer[i]=0;//清接收缓存
//				}
			memset(rx_buffer,0,rx_len);
  }
		HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);//重新打开DMA接收
}
/*
*********************************************************************************************************
* 函 数 名: DMA_Usart1_Read
* 功能说明: 串口接收功能函数
* 形  参: Data,len
* 返 回 值: 无
*********************************************************************************************************
*/
void Usart4_DMA_IDLE_Init(void)//DMA_IDLE_INIT
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart4); 											// 清除IDLE标志
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE); 							// 使能串UART1 IDLE中断
}
