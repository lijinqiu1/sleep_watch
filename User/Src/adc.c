#include "nordic_common.h"
#include "nrf.h"
#include "adc.h"
#include "nrf_gpio.h"


void adc_init(void)
{
    nrf_gpio_cfg_input(1, NRF_GPIO_PIN_NOPULL);
    NRF_ADC->INTENSET   = ADC_INTENSET_END_Msk;  
	NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit                            << ADC_CONFIG_RES_Pos)     |  
                      (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |  
                      (ADC_CONFIG_REFSEL_VBG                           << ADC_CONFIG_REFSEL_Pos)  |  
                      (ADC_CONFIG_PSEL_AnalogInput2                    << ADC_CONFIG_PSEL_Pos)    |  
                      (ADC_CONFIG_EXTREFSEL_None                       << ADC_CONFIG_EXTREFSEL_Pos);  

	//使能adc end事件中断
//	NRF_ADC->INTENSET = 0x01;
    NRF_ADC->EVENTS_END = 0;  
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;  

}

uint16_t adc_start(void)
{
	uint16_t  value = 0;

	//开始转换
	NRF_ADC->TASKS_START = 0x01;

	while(NRF_ADC->EVENTS_END == 0);
    NRF_ADC->EVENTS_END     = 0;  
	value = NRF_ADC->RESULT;
    NRF_ADC->TASKS_STOP     = 1;  
	return value;
}


