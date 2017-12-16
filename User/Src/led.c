#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "led.h"

/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
void leds_init(void)
{
    nrf_gpio_cfg_output(LED0);
    nrf_gpio_cfg_output(LED1);
    nrf_gpio_pin_set(LED0);
    nrf_gpio_pin_set(LED1);
}


void leds_process(void)
{

}

