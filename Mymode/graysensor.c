#include "graysensor.h"
#include "hardware_iic.h"
#include "stdio.h"
#include "string.h"
#include "lcd_spi_154.h"


unsigned char Anolog[8]={0};
unsigned char Normal[8]={0};

void Graysensor_Init(void)
{
	while(Ping())
	{
		HAL_Delay(1);
		LCD_DisplayText(24, 48,"Ping Faild!\r\n");
	}
	LCD_DisplayText(24, 48,"Ping Succseful!\r\n");
}

unsigned char Graysensor_Getdata(void)
{
	unsigned char Digtal;
	Digtal=IIC_Get_Digtal();
	Anolog[0] = (Digtal>>0)&0x01;
	Anolog[1] = (Digtal>>1)&0x01;
	Anolog[2] = (Digtal>>2)&0x01;
	Anolog[3] = (Digtal>>3)&0x01;
	Anolog[4] = (Digtal>>4)&0x01;
	Anolog[5] = (Digtal>>5)&0x01;
	Anolog[6] = (Digtal>>6)&0x01;
	Anolog[7] = (Digtal>>7)&0x01;
	return Digtal;
}
