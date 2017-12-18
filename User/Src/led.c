#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "led.h"


/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static Led_Work_Status_t cur_led_status = LED_IDLE;
void leds_init(void)
{
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_cfg_output(LED_GREEN);
    nrf_gpio_pin_set(LED_RED);
    nrf_gpio_pin_set(LED_GREEN);
}

void leds_process_init(Led_Work_Status_t status)
{
	if (status >= cur_led_status)
	{
		return ;
	}
	switch(status)
	{
	case LED_WORK_POWER_LOW:
		break;
	case LED_WORK_POWER_CHARGING:
		break;
	case LED_WORK_POWER_CHARGE_COMPLETE:
		break;
	case LED_WORK_BLE_DATA_TRAING_ERROR:
		break;
	case LED_WORK_BLE_DATA_TRAING:
		break;
	case LED_WORK_DATA_FULL:
		break;
	case LED_WORK_BEGIN:
		break;
	case LED_WORK_END:
		break;
	}
}

void leds_process(void)
{

}

