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
static Led_Work_Status_t cur_led_status = LED_WORK_IDLE;
//static UTCTime begin_TimeSeconds = 0;
static Led_Flash_Method_t cur_led_flash_method = {
    .flash_time = 0,
    .flash_rate = 0,
    .flash_period = 0,
    .flash_led = 0,
    .flash_end_time = 0,
    };
//static uint32_t cur_led_work_status = 0;
//static uint32_t last_led_work_status = 0;

const Led_Flash_Method_t led_flash_method[LED_WORK_MAX] = {
{                     0,                  0,LED_FLASH_PERIOD_ONE_SHOT,                 0,0},//usb disconnect
{     LED_FLASH_RATE_ON,LED_FLASH_TIME_LONG,LED_FLASH_PERIOD_ONE_SHOT,LED_FLASH_LED_BLUE,0},//charge complete
{ LED_FLASH_RATE_BREATH,LED_FLASH_TIME_LONG,LED_FLASH_PERIOD_ONE_SHOT,LED_FLASH_LED_BLUE,0},//charging
{    LED_FLASH_RATE_1HZ,LED_FLASH_TIME_LONG,LED_FLASH_PERIOD_ONE_SHOT, LED_FLASH_LED_RED,0},//power low
{    LED_FLASH_RATE_1HZ,                 10,                       30,LED_FLASH_LED_BOTH,0},//data full
{   LED_FLASH_RATE_FAST,                  5,LED_FLASH_PERIOD_ONE_SHOT, LED_FLASH_LED_RED,0},//data send error
{   LED_FLASH_RATE_FAST,LED_FLASH_TIME_LONG,LED_FLASH_PERIOD_ONE_SHOT,LED_FLASH_LED_BLUE,0},//data sending
{     LED_FLASH_RATE_ON,                  5,LED_FLASH_PERIOD_ONE_SHOT, LED_FLASH_LED_RED,0},//ble connected
{    LED_FLASH_RATE_1HZ,                  5,LED_FLASH_PERIOD_ONE_SHOT,LED_FLASH_LED_BLUE,0},//adv start
{    LED_FLASH_RATE_1HZ,                 10,LED_FLASH_PERIOD_ONE_SHOT,LED_FLASH_LED_BLUE,0},//work begin
{    LED_FLASH_RATE_1HZ,                 10,LED_FLASH_PERIOD_ONE_SHOT, LED_FLASH_LED_RED,0},//work end
{                     0,                  0,LED_FLASH_PERIOD_ONE_SHOT,                 0,0},//led idle
};


void leds_init(void)
{
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_cfg_output(LED_BLUE);
    nrf_gpio_pin_set(LED_RED);
    nrf_gpio_pin_set(LED_BLUE);
}
#if 0
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
        led_flash_method.flash_time = LED_FLASH_TIME_LONG;
        led_flash_method.flash_period = 0;
        led_flash_method.flash_led = LED_FLASH_LED_RED;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_POWER_CHARGING:
        led_flash_method.flash_rate = LED_FLASH_RATE_BREATH;
        led_flash_method.flash_time = LED_FLASH_TIME_LONG;
        led_flash_method.flash_led = LED_FLASH_LED_BLUE;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_POWER_CHARGE_COMPLETE:
        led_flash_method.flash_rate = LED_FLASH_RATE_ON;
        led_flash_method.flash_time = LED_FLASH_TIME_LONG;
        led_flash_method.flash_led = LED_FLASH_LED_BLUE;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_BLE_DATA_SEND_ERROR:
        led_flash_method.flash_rate = LED_FLASH_RATE_FAST;
        led_flash_method.flash_time = 5;
        led_flash_method.flash_led = LED_FLASH_LED_RED;
        led_flash_method.flash_period = 0;
        begin_TimeSeconds = TimeSeconds;
		break;
	case LED_WORK_BLE_DATA_SENDING:
        led_flash_method.flash_rate = LED_FLASH_RATE_FAST;
        led_flash_method.flash_time = LED_FLASH_TIME_LONG;
        led_flash_method.flash_led = LED_FLASH_LED_BLUE;
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
	case LED_WORK_BLE_ADV:
        led_flash_method.flash_rate = LED_FLASH_RATE_1HZ;
        led_flash_method.flash_time = 10;
        led_flash_method.flash_led = LED_FLASH_LED_BLUE;
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
        led_flash_method.flash_led = LED_FLASH_LED_BLUE;
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
    case LED_WORK_IDLE:
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
                    nrf_gpio_pin_set(LED_BLUE);
                    nrf_gpio_pin_toggle(LED_RED);
                }
                else if (led_flash_method.flash_led == LED_FLASH_LED_BLUE)
                {
                    nrf_gpio_pin_set(LED_RED);
                    nrf_gpio_pin_toggle(LED_BLUE);
                }
                else if (led_flash_method.flash_led == LED_FLASH_LED_BOTH)
                {
                    if (flag)
                    {
                        nrf_gpio_pin_set(LED_RED);
                        nrf_gpio_pin_clear(LED_BLUE);
                        flag = 0;
                    }
                    else
                    {
                        nrf_gpio_pin_set(LED_BLUE);
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
            else if (led_flash_method.flash_led == LED_FLASH_LED_BLUE)
            { 
                pwm_led_start(LED_BLUE);
            }
        break;
        case LED_FLASH_RATE_FAST:
            pwm_led_stop();
            if (led_flash_method.flash_led == LED_FLASH_LED_RED)
            {
                nrf_gpio_pin_set(LED_BLUE);
                nrf_gpio_pin_toggle(LED_RED);
            }
            else if (led_flash_method.flash_led == LED_FLASH_LED_BLUE)
            {
                nrf_gpio_pin_set(LED_RED);
                nrf_gpio_pin_toggle(LED_BLUE);
            }
            else if (led_flash_method.flash_led == LED_FLASH_LED_BOTH)
            {
                if (flag)
                {
                    nrf_gpio_pin_set(LED_RED);
                    nrf_gpio_pin_clear(LED_BLUE);
                    flag = 0;
                }
                else
                {
                    nrf_gpio_pin_set(LED_BLUE);
                    nrf_gpio_pin_clear(LED_RED);
                    flag = 1;
                }
            }
        break;
        case LED_FLASH_RATE_ON:
            pwm_led_stop();
            if (led_flash_method.flash_led == LED_FLASH_LED_RED)
            {
                nrf_gpio_pin_set(LED_BLUE);
                nrf_gpio_pin_clear(LED_RED);
            }
            else if (led_flash_method.flash_led == LED_FLASH_LED_BLUE)
            {
                nrf_gpio_pin_set(LED_RED);
                nrf_gpio_pin_clear(LED_BLUE);
            }
            else if (led_flash_method.flash_led == LED_FLASH_LED_BOTH)
            {
                nrf_gpio_pin_clear(LED_RED);
                nrf_gpio_pin_clear(LED_BLUE);
            }
        break;
        case LED_FLASH_RATE_OFF:
            pwm_led_stop();
            nrf_gpio_pin_set(LED_RED);
            nrf_gpio_pin_set(LED_BLUE);
        break;
        }
    }
    else
    {
        if ((led_flash_method.flash_period!= 0)&&
			(TimeSeconds > (led_flash_method.flash_period + begin_TimeSeconds)))
        {
            begin_TimeSeconds = TimeSeconds;
        }
		else
		{
			cur_led_status = LED_IDLE;
		}
        pwm_led_stop();
        nrf_gpio_pin_set(LED_RED);
        nrf_gpio_pin_set(LED_BLUE);
    }
}

#else
void leds_process_init(Led_Work_Status_t status)
{
    if (status < cur_led_status)
    {
        cur_led_flash_method.flash_rate    = led_flash_method[status].flash_rate;
        cur_led_flash_method.flash_time    = led_flash_method[status].flash_time;
        cur_led_flash_method.flash_period  = led_flash_method[status].flash_period;
        cur_led_flash_method.flash_led     = led_flash_method[status].flash_led;
        cur_led_status = status;
        if(cur_led_flash_method.flash_rate == LED_FLASH_RATE_BREATH)
        {
            if (cur_led_flash_method.flash_led == LED_FLASH_LED_RED)
            { 
                pwm_led_start(LED_RED);
            }
            else if (cur_led_flash_method.flash_led == LED_FLASH_LED_BLUE)
            { 
                pwm_led_start(LED_BLUE);
            }
        }
		else
		{
			pwm_led_stop();
			nrf_gpio_pin_set(LED_BLUE);
			nrf_gpio_pin_set(LED_RED);
		}
    }
    else if (status == LED_WORK_END && cur_led_status == LED_WORK_BEGIN)
    {
        cur_led_flash_method.flash_rate    = led_flash_method[status].flash_rate;
        cur_led_flash_method.flash_time    = led_flash_method[status].flash_time;
        cur_led_flash_method.flash_period  = led_flash_method[status].flash_period;
        cur_led_flash_method.flash_led     = led_flash_method[status].flash_led;
        cur_led_status = status;
    }
}

void leds_process_cancel(void)
{
    pwm_led_stop();
    nrf_gpio_pin_set(LED_BLUE);
    nrf_gpio_pin_set(LED_RED);
    cur_led_status = LED_WORK_IDLE;
}

void leds_process(void)
{
    static uint8_t flag = 0;
    if (cur_led_status != LED_WORK_IDLE)
    {
        if(cur_led_flash_method.flash_time > 0)
        {
            if(cur_led_flash_method.flash_rate == LED_FLASH_RATE_1HZ)
            {
                switch(cur_led_flash_method.flash_led)
                {
                case LED_FLASH_LED_BLUE:
                    nrf_gpio_pin_toggle(LED_BLUE);
                    nrf_gpio_pin_set(LED_RED);
                break;
                case LED_FLASH_LED_RED:
                    nrf_gpio_pin_toggle(LED_RED);
                    nrf_gpio_pin_set(LED_BLUE);
                break;
                case LED_FLASH_LED_BOTH:
                    if (flag == 0)
                    {
                        flag = 1;
                        nrf_gpio_pin_set(LED_BLUE);
                        nrf_gpio_pin_clear(LED_RED);
                    }
                    else
                    {
                        flag = 0;
                        nrf_gpio_pin_set(LED_BLUE);
                        nrf_gpio_pin_clear(LED_RED);
                    }
                break;
                }
            }
            else if (cur_led_flash_method.flash_rate == LED_FLASH_RATE_ON)
            {
                switch(cur_led_flash_method.flash_led)
                {
                case LED_FLASH_LED_BLUE:
                    nrf_gpio_pin_clear(LED_BLUE);
                    nrf_gpio_pin_set(LED_RED);
                break;
                case LED_FLASH_LED_RED:
                    nrf_gpio_pin_clear(LED_RED);
                    nrf_gpio_pin_set(LED_BLUE);
                break;
                case LED_FLASH_LED_BOTH:
                break;
                }
            }
        }
        else
        {
            if (cur_led_flash_method.flash_period == LED_FLASH_PERIOD_ONE_SHOT)
            {//led闪烁结束
                pwm_led_stop();
                nrf_gpio_pin_set(LED_BLUE);
                nrf_gpio_pin_set(LED_RED);
                cur_led_status = LED_WORK_IDLE;
            }
            else if(cur_led_flash_method.flash_period -- == 0)
            {//周期性闪烁
                cur_led_flash_method.flash_rate    = led_flash_method[cur_led_status].flash_rate;
                cur_led_flash_method.flash_time    = led_flash_method[cur_led_status].flash_time;
                cur_led_flash_method.flash_period  = led_flash_method[cur_led_status].flash_period;
                cur_led_flash_method.flash_led     = led_flash_method[cur_led_status].flash_led;
            }
        }
        if(cur_led_flash_method.flash_time != LED_FLASH_TIME_LONG)
        {
            cur_led_flash_method.flash_time --;
        }
    }
}

void leds_process_flash_fast(void)
{
    if(cur_led_flash_method.flash_rate == LED_FLASH_RATE_FAST)
    {
        switch(cur_led_flash_method.flash_led)
        {
        case LED_FLASH_LED_BLUE:
            nrf_gpio_pin_toggle(LED_BLUE);
            nrf_gpio_pin_set(LED_RED);
        break;
        case LED_FLASH_LED_RED:
            nrf_gpio_pin_toggle(LED_RED);
            nrf_gpio_pin_set(LED_BLUE);
        break;
        }

    }
}

void leds_process_flash_breath(void)
{
    if(cur_led_flash_method.flash_rate == LED_FLASH_RATE_BREATH)
    {
        if (cur_led_flash_method.flash_led == LED_FLASH_LED_RED)
        { 
            pwm_led_start(LED_RED);
        }
        else if (cur_led_flash_method.flash_led == LED_FLASH_LED_BLUE)
        { 
            pwm_led_start(LED_BLUE);
        }
    }
}
#endif

Led_Work_Status_t leds_get_cur_status(void)
{
    return cur_led_status;
}

