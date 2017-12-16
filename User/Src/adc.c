#include "nordic_common.h"
#include "nrf.h"
#include "adc.h"


void adc_init(void)
{
	NRF_ADC->CONFIG = (2 << 0) //ADCת������10λ
	                | (2 << 2) //ADC����ֵΪ�����1/3
	                | (0 << 5) //ѡ���ڲ�1.2VΪ�ο���ѹ
	                | (4 << 8);//ѡ��AIN2(P0.1)ΪADC������

	//ʹ��adc end�¼��ж�
//	NRF_ADC->INTENSET = 0x01;

	NRF_ADC->ENABLE = 0x01;
}

uint16_t adc_start(void)
{
	float value = 0;

	//��ʼת��
	NRF_ADC->TASKS_START = 0x01;

	while(NRF_ADC->BUSY & 1);
	value = NRF_ADC->RESULT * 1.0;
	value = value*1.2/1024;
	value *= 3;
//	app_trace_log("ADC: %f\r\n",value);
	return (uint16_t)(value * 1000);
}


