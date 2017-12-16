#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "battery.h"
#include "adc.h"

static Battery_Charge_Status_e battery_charge_cur_status = BATTERY_NOT_CHARGE;
static uint16_t battery_value;
void battery_init(void)
{
    nrf_gpio_cfg_input(BQ24040_CHG_PIN, GPIO_PIN_CNF_PULL_Pullup);
    nrf_gpio_cfg_input(BQ24040_PG_PIN, GPIO_PIN_CNF_PULL_Pullup);
    adc_init();
}

void battery_manager(void)
{
    battery_value = adc_start();
    if (!nrf_gpio_pin_read(BQ24040_PG_PIN))
    {
        if(nrf_gpio_pin_read(BQ24040_CHG_PIN))
        {//充电中
            battery_charge_cur_status = BATTERY_CHARGING;
        }
        else
        {//充电完成
            battery_charge_cur_status = BATTERY_CHARGE_COMPLETE;
        }
    }
    else
    {
        battery_charge_cur_status = BATTERY_NOT_CHARGE;
    }
}

Battery_Charge_Status_e battery_get_charege_status(void)
{
    return battery_charge_cur_status;
}

uint16_t battery_get_value(void)
{
	return battery_value;
}