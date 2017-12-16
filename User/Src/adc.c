#include "nordic_common.h"
#include "nrf.h"
#include "adc.h"


void adc_init(void)
{
	NRF_ADC->CONFIG = (2 << 0) //ADC转换精度10位
	                | (2 << 2) //ADC测量值为输入的1/3
	                | (0 << 5) //选择内部1.2V为参考电压
	                | (4 << 8);//选择AIN2(P0.1)为ADC的输入

	//使能adc end事件中断
//	NRF_ADC->INTENSET = 0x01;

	NRF_ADC->ENABLE = 0x01;
}

uint16_t adc_start(void)
{
	float value = 0;

	//开始转换
	NRF_ADC->TASKS_START = 0x01;

	while(NRF_ADC->BUSY & 1);
	value = NRF_ADC->RESULT * 1.0;
	value = value*1.2/1024;
	value *= 3;
//	app_trace_log("ADC: %f\r\n",value);
	return (uint16_t)(value * 1000);
}


