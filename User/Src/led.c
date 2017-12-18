#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "led.h"
#include "pwm.h"
#include "calender.h"

/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static Led_Work_Status_t cur_led_status = LED_IDLE;
static UTCTime begin_TimeSeconds = 0;
static Led_Flash_Method_t led_flash_method = {
    .flash_time = 0,
    .flash_rate = 0,
    .flash_period = 0,
    .flash_led = 0,
    };
void leds_init(void)
{
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_cfg_output(LED_GREEN);
    nrf_gpio_pin_set(LED_RED);
    nrf_gpio_pin_set(LED_GREEN);
}

void leds_process_init(Led_Work_Status_t status)
{
	if ((status >= cur_led_status) && (status != LED_IDLE))
	{
		return ;
	}
    cur_led_status = status;
	switch(status)
	{
	case LED_WORK_POWER_LOW:
        led_flash_method.flash_rate = LED_FLASH_RATE_1HZ;
        led_flash_method.flash_time = 0xff;
        led_flash_method.flash_period = 0;
        led_flash_method.flash_led = LED_FLASH_LED_RED;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_POWER_CHARGING:
        led_flash_method.flash_rate = LED_FLASH_RATE_BREATH;
        led_flash_method.flash_time = LED_FLASH_TIME_LONG;
        led_flash_method.flash_led = LED_FLASH_LED_GREEN;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_POWER_CHARGE_COMPLETE:
        led_flash_method.flash_rate = LED_FLASH_RATE_ON;
        led_flash_method.flash_time = LED_FLASH_TIME_LONG;
        led_flash_method.flash_led = LED_FLASH_LED_GREEN;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_BLE_DATA_TRAING_ERROR:
        led_flash_method.flash_rate = LED_FLASH_RATE_FAST;
        led_flash_method.flash_time = 5;
        led_flash_method.flash_led = LED_FLASH_LED_RED;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_BLE_DATA_TRAING:
        led_flash_method.flash_rate = LED_FLASH_RATE_FAST;
        led_flash_method.flash_time = LED_FLASH_TIME_LONG;
        led_flash_method.flash_led = LED_FLASH_LED_GREEN;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
    case LED_WORK_BLE_CONNECTED:
        led_flash_method.flash_rate = LED_FLASH_RATE_ON;
        led_flash_method.flash_time = 5;
        led_flash_method.flash_led = LED_FLASH_LED_RED;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
        break;
	case LED_WORK_DATA_FULL:
        led_flash_method.flash_rate = LED_FLASH_RATE_1HZ;
        led_flash_method.flash_time = 10;
        led_flash_method.flash_led = LED_FLASH_LED_BOTH;
        led_flash_method.flash_period = 30;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_BEGIN:
        led_flash_method.flash_rate = LED_FLASH_RATE_1HZ;
        led_flash_method.flash_time = 10;
        led_flash_method.flash_led = LED_FLASH_LED_GREEN;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_END:
        led_flash_method.flash_rate = LED_FLASH_RATE_1HZ;
        led_flash_method.flash_time = 10;
        led_flash_method.flash_led = LED_FLASH_LED_RED;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
    case LED_IDLE:
        led_flash_method.flash_rate = LED_FLASH_RATE_OFF;
        led_flash_method.flash_time = 0;
        led_flash_method.flash_led = LED_FLASH_LED_BOTH;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
        break;
	}
}

void leds_process(void)
{
    static UTCTime TimeSeconds_rate = 0;
    static uint8_t flag = 0;
    if((led_flash_method.flash_time == LED_FLASH_TIME_LONG) || 
        (TimeSeconds < begin_TimeSeconds + led_flash_method.flash_time))
    {
        switch(led_flash_method.flash_rate)
        {
        case LED_FLASH_RATE_1HZ:
            pwm_led_stop();
            if (TimeSeconds_rate != TimeSeconds)
            {
                TimeSeconds_rate = TimeSeconds;
                if (led_flash_method.flash_led == LED_FLASH_LED_RED)
                {
                    nrf_gpio_pin_set(LED_GREEN);`
                    nrf_gpio_pin_toggle(LED_RED);
                }
                else if (led_flash_method.flash_led == LED_FLASH_LED_GREEN)
                {
                    nrf_gpio_pin_set(LED_RED);
                    nrf_gpio_pin_toggle(LED_GREEN);
                }
                else if (led_flash_method.flash_led == LED_FLASH_LED_BOTH)
                {
                    if (flag)
                    {
                        nrf_gpio_pin_set(LED_RED);
                        nrf_gpio_pin_clear(LED_GREEN);
                        flag = 0;
                    }
                    else
                    {
                        nrf_gpio_pin_set(LED_GREEN);
                        nrf_gpio_pin_clear(LED_RED);
                        flag = 1;
                    }
                }
            }
        break;
        case LED_FLASH_RATE_BREATH:
            if (led_flash_method.flash_led == LED_FLASH_LED_RED)
            { 
                pwm_led_start(LED_RED);
            }
            else if (led_flash_method.flash_led == LED_FLASH_LED_GREEN)
            { 
                pwm_led_start(LED_GREEN);
            }
        break;
        case LED_FLASH_RATE_FAST:
            pwm_led_stop();
            if (led_flash_method.flash_led == LED_FLASH_LED_RED)
            {
                nrf_gpio_pin_set(LED_GREEN);
                nrf_gpio_pin_toggle(LED_RED);
            }
            else if (led_flash_method.flash_led == LED_FLASH_LED_GREEN)
            {
                nrf_gpio_pin_set(LED_RED);
                nrf_gpio_pin_toggle(LED_GREEN);
            }
            else if (led_flash_method.flash_led == LED_FLASH_LED_BOTH)
            {
                if (flag)
                {
                    nrf_gpio_pin_set(LED_RED);
                    nrf_gpio_pin_clear(LED_GREEN);
                    flag = 0;
                }
                else
                {
                    nrf_gpio_pin_set(LED_GREEN);
                    nrf_gpio_pin_clear(LED_RED);
                    flag = 1;
                }
            }
        break;
        case LED_FLASH_RATE_ON:
            pwm_led_stop();
            if (led_flash_method.flash_led == LED_FLASH_LED_RED)
            {
                nrf_gpio_pin_set(LED_GREEN);
                nrf_gpio_pin_clear(LED_RED);
            }
            else if (led_flash_method.flash_led == LED_FLASH_LED_GREEN)
            {
                nrf_gpio_pin_set(LED_RED);
                nrf_gpio_pin_clear(LED_GREEN);
            }
            else if (led_flash_method.flash_led == LED_FLASH_LED_BOTH)
            {
                nrf_gpio_pin_clear(LED_RED);
                nrf_gpio_pin_clear(LED_GREEN);
            }
        break;
        case LED_FLASH_RATE_OFF:
            pwm_led_stop();
            nrf_gpio_pin_set(LED_RED);
            nrf_gpio_pin_set(LED_GREEN);
        break;
        }
    }
    else
    {
        if (TimeSeconds > (led_flash_method.flash_period + begin_TimeSeconds)
        {
            begin_TimeSeconds = TimeSeconds;
        }
        pwm_led_stop();
        nrf_gpio_pin_set(LED_RED);
        nrf_gpio_pin_set(LED_GREEN);
    }
}

