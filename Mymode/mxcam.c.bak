#include "stm32f4xx.h"                  // Device header
#include "mxcam.h"
#include "usart.h"
#include "main.h"

//物料： 数据包data = bytes([0xb3, 0xb3, 0x03, int(Data[0]), int(Data[1]), int(Data[2]), int(Data[3]), int(Data[4]), 0x5b])
//int(Data[0]), int(Data[1])存物料横坐标，如果Data[1]<255,则Data[0]为0，如果Data[1]>255,则超出部分存到Data[0]，且Data[1]=255
//int(Data[2]), int(Data[3])存物料竖坐标，如果Data[3]<255,则Data[2]为0，如果Data[3]>255,则超出部分存到Data[2]，且Data[3]=255
//由于目前屏幕分辨率设置为240*240，所以坐标只读Data[1]和Data[3]即可，Data[4]为颜色12345分别对应红绿蓝黑白
//圆环： 数据包data = bytes([0xb3, 0xb3, 0x02, int(Data[0]), int(Data[1]), int(Data[2]), int(Data[3]), 0, 0x5b])
//坐标和物料一样，所以坐标只读Data[1]和Data[3]即可，为横纵坐标
// 320
// *y
// *
// *
// *	  (160,160)
// *
// *
// *
//0***************x320
extern uint8_t cammode;
extern uint8_t lookmode_centrl;
Relative_coord Camera_err;
/********************************************************
* 函数名：process_usart1_packet
* 功能：处理USART1的完整数据包
* 参数：data-完整数据包指针
* 说明：从uart_manager调用
********************************************************/
void Process_usart1_packet(void) {
    uint8_t camera_packet[9] = {0};
	for	(uint8_t i = 0;i<9;i++){
		camera_packet[i] = rx_buffer[i];
	}
	if(camera_packet [0] == 0xb3 && camera_packet [1] == 0xb3){
		if(camera_packet [2] == 0x02){
			int16_t x_coord = camera_packet [3]+camera_packet [4];
			int16_t y_coord = camera_packet [5]+camera_packet [6];
			Camera_err.x = 158-x_coord;//偏右就加偏左就减
			Camera_err.y = 110-y_coord;
		}
	}
    // 解析坐标
	

    
    // 调用实际处理函数（示例）
    // display_material(packet.x, packet.y, packet.color);
}
