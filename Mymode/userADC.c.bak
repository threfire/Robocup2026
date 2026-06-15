#include "stm32f4xx.h"                  // Device header
#include "useradc.h"

uint16_t adc_value;

void AdcGet_Init(void)
{
	HAL_ADC_Start_IT(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc1)
	{
		__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC);
		adc_value = HAL_ADC_GetValue(hadc);

		// 重新开启ADC中断
		HAL_ADC_Start_IT(&hadc1);
	}
}
