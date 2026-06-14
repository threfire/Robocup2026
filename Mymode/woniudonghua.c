#include "woniudonghua.h"
#include "lcd_spi_154.h"
uint8_t donghua_tick;
uint8_t imagenum = 0;
uint16_t count = 0;
void donghuatick(void)
{
	count ++;
	if(count > 100)
	{
		donghua_tick++;
		donghua_tick %= 120;
		imagenum ++;
		imagenum %= 3;
		count = 0;
	}

}

void qiehuan(void)
{
	if(imagenum == 0)LCD_DrawImage( donghua_tick, 72, 120, 120, Image_woniu_80x64) ;
	else if (imagenum == 1)LCD_DrawImage( donghua_tick, 72, 120, 120, Image_woniu2_80x64) ;
	else LCD_DrawImage( donghua_tick, 72, 120, 120, Image_woniu3_80x64) ;
	if(donghua_tick == 0)LCD_ClearRect(120,72,120,120);
}
