#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "battery.h"
#include "adc.h"
#include "led.h"
static Battery_Charge_Status_e battery_charge_cur_status = BATTERY_NOT_CHARGE;
static Battery_Charge_Status_e battery_charge_last_status = BATTERY_NOT_CHARGE;
static uint16_t battery_value;
void battery_init(void)
{
    nrf_gpio_cfg_input(BQ24040_CHG_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(BQ24040_PG_PIN, NRF_GPIO_PIN_PULLUP);
    adc_init();
}

void battery_manager(void)
{
    if (!nrf_gpio_pin_read(BQ24040_PG_PIN))
    {
        if(nrf_gpio_pin_read(BQ24040_CHG_PIN))
        {//割窮嶄
            battery_charge_cur_status = BATTERY_CHARGING;
            battery_charge_last_status = battery_charge_cur_status;
			leds_process_init(LED_WORK_POWER_CHARGING);
        }
        else
        {//割窮頼撹
            battery_charge_cur_status = BATTERY_CHARGE_COMPLETE;
            battery_charge_last_status = battery_charge_cur_status;
			leds_process_init(LED_WORK_POWER_CHARGE_COMPLETE);
        }
    }
    else
    {
//		if (battery_va，，，，，，，，，，，，，，，，，，，，，，，，，，，，，lue < BATTER_VALUE_LOW)
//		{
//			battery_charge_cur_status = BATTERY_VALUE_LOW;
//            if (battery_charge_cur_status != battery_charge_last_status)
//            {
//                battery_charge_last_status = battery_charge_cur_status;
//			    leds_process_init(LED_WORK_POWER_LOW);
//            }
//		}
//		else
		{
        	battery_charge_cur_status = BATTERY_NOT_CHARGE;
            if (battery_charge_cur_status != battery_charge_last_status)
            {
                battery_charge_last_status = battery_charge_cur_status;
			    leds_process_init(LED_IDLE);
            }
		}
    }
}

Battery_Charge_Status_e battery_get_charege_status(void)
{
    return battery_charge_cur_status;
}

uint16_t battery_get_value(void)
{
    battery_value = adc_start();
	return battery_value;
}
