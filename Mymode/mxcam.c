#include "stm32f4xx.h"
#include "mxcam.h"
#include "usart.h"
#include "main.h"
#include "control.h"

Relative_coord Camera_err;
/********************************************************
 * 函数名：process_usart1_packet
 * 功能：处理USART1的完整数据包
 * 参数：data-完整数据包指针
 * 说明：从uart_manager调用
 ********************************************************/
void Process_usart1_packet(void) {
	uint8_t camera_packet[9] = {0};
	if (!recv_end_flag || rx_len < sizeof(camera_packet)) {
		return;
	}

	for (uint8_t i = 0; i < 9; i++) {
		camera_packet[i] = rx_buffer[i];
	}
	if (camera_packet[0] == 0xb3 && camera_packet[1] == 0xb3) {
		if (camera_packet[2] == 0x02) {
			int16_t x_coord = camera_packet[3] + camera_packet[4];
			int16_t y_coord = camera_packet[5] + camera_packet[6];
			Camera_err.x = 180 - x_coord;	//加是目标点往右
			Camera_err.y = 124 - y_coord;	//加是目标点往前
			Is_coordgot = 1U;
		}
	}
}
