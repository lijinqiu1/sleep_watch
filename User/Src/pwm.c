#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
#include "softdevice_handler.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "pwm.h"
#include "queue.h"

static PwmStatusEN_t pwm_status = PWM_STATUS_READY;
static uint8_t pwm_moto_strong = 0;
/** @brief Function for handling timer 2 peripheral interrupts.
 */
void TIMER2_IRQHandler(void)
{
    static bool cc0_turn = false; /**< Keeps track of which CC register to be used. */
	static uint32_t next_sample = 32;
	static bool dir = 0;
	static uint8_t cont = 0;
	//呼吸灯
    if ((NRF_TIMER2->EVENTS_COMPARE[1] != 0) &&
       ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
    {
        // Sets the next CC1 value
        NRF_TIMER2->EVENTS_COMPARE[1] = 0;
        NRF_TIMER2->CC[1]             = (NRF_TIMER2->CC[1] + MAX_SAMPLE_LEVELS);

        // Every other interrupt CC0 and CC2 will be set to their next values.
		if (cont++ > 200)
		{
			if (dir == 0)
			{
				next_sample += 16;
			}
			else
			{
				next_sample -= 16;
			}
			if (next_sample == 240)
			{
				dir = 1;
			}
			else if (next_sample == 16)
			{
				dir = 0;
			}
			cont = 0;
		}
        if (cc0_turn)
        {
            NRF_TIMER2->CC[0] = NRF_TIMER2->CC[1] + next_sample;
        }
        else
        {
            NRF_TIMER2->CC[2] = NRF_TIMER2->CC[1] + next_sample;
        }
        // Next turn the other CC will get its value.
        cc0_turn = !cc0_turn;
    }
}

void TIMER1_IRQHandler(void)
{
    static bool cc0_turn = false; /**< Keeps track of which CC register to be used. */
	//呼吸灯
    if ((NRF_TIMER1->EVENTS_COMPARE[1] != 0) &&
       ((NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
    {
        // Sets the next CC1 value
        NRF_TIMER1->EVENTS_COMPARE[1] = 0;
        NRF_TIMER1->CC[1]             = (NRF_TIMER1->CC[1] + MAX_SAMPLE_LEVELS);

        if (cc0_turn)
        {
            NRF_TIMER1->CC[0] = NRF_TIMER1->CC[1] + pwm_moto_strong;
        }
        else
        {
            NRF_TIMER1->CC[2] = NRF_TIMER1->CC[1] + pwm_moto_strong;
        }
        // Next turn the other CC will get its value.
        cc0_turn = !cc0_turn;
    }
}

void pwm_led_start(void)
{
	if (pwm_status == PWM_STATUS_READY)
	{
		//判断当前是否正在工作

	}
	//gpiote_init
	// Connect GPIO input buffers and configure PWM_OUTPUT_PIN_NUMBER as an output.
    nrf_gpio_cfg_output(PWM_LED_PIN);

    nrf_gpio_pin_clear(PWM_LED_PIN);
    // Configure GPIOTE channel 0 to toggle the PWM pin state
    // @note Only one GPIOTE task can be connected to an output pin.
    nrf_gpiote_task_config(0, PWM_LED_PIN, \
                           NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
	//ppi_init
    // Configure PPI channel 0 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[0] match.
	sd_ppi_channel_assign(0,&NRF_TIMER2->EVENTS_COMPARE[0],&NRF_GPIOTE->TASKS_OUT[0]);

    // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[1] match.
	sd_ppi_channel_assign(1,&NRF_TIMER2->EVENTS_COMPARE[1],&NRF_GPIOTE->TASKS_OUT[0]);

    // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[2] match.
	sd_ppi_channel_assign(2,&NRF_TIMER2->EVENTS_COMPARE[2],&NRF_GPIOTE->TASKS_OUT[0]);


    // Enable PPI channels 0-2.
	sd_ppi_channel_enable_set((PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                    | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos)
                    | (PPI_CHEN_CH2_Enabled << PPI_CHEN_CH2_Pos));
    //timer2_init
	NRF_TIMER2->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER2->PRESCALER = TIMER_PRESCALERS;

    // Clears the timer, sets it to 0.
    NRF_TIMER2->TASKS_CLEAR = 1;

    // Load the initial values to TIMER2 CC registers.
    NRF_TIMER2->CC[0] = MAX_SAMPLE_LEVELS + 8;
    NRF_TIMER2->CC[1] = MAX_SAMPLE_LEVELS;
    // CC2 will be set on the first CC1 interrupt.
    NRF_TIMER2->CC[2] = 0;

    // Interrupt setup.
    NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
    // Enabling constant latency as indicated by PAN 11 "HFCLK: Base current with HFCLK
    // running is too high" found at Product Anomaly document found at
    // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
    //
    // @note This example does not go to low power mode therefore constant latency is not needed.
    //       However this setting will ensure correct behaviour when routing TIMER events through


    // Enable interrupt on Timer 2.
    NVIC_EnableIRQ(TIMER2_IRQn);

    // Start the timer.
    NRF_TIMER2->TASKS_START = 1;
}

void pwm_led_stop(void)
{
	NRF_TIMER2->TASKS_START = 0;
	nrf_gpiote_unconfig(0);
	sd_ppi_channel_enable_clr((PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                    | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos)
                    | (PPI_CHEN_CH2_Enabled << PPI_CHEN_CH2_Pos));
}

void pwm_moto_init(void)
{
	//gpiote_init
	// Connect GPIO input buffers and configure PWM_OUTPUT_PIN_NUMBER as an output.
    nrf_gpio_cfg_output(PWM_MOTO_PIN);

    nrf_gpio_pin_clear(PWM_MOTO_PIN);
    // Configure GPIOTE channel 0 to toggle the PWM pin state
    // @note Only one GPIOTE task can be connected to an output pin.
    nrf_gpiote_task_config(1, PWM_MOTO_PIN, \
                           NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
	//ppi_init
    // Configure PPI channel 0 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[0] match.
	sd_ppi_channel_assign(3,&NRF_TIMER1->EVENTS_COMPARE[0],&NRF_GPIOTE->TASKS_OUT[1]);

    // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[1] match.
	sd_ppi_channel_assign(4,&NRF_TIMER1->EVENTS_COMPARE[1],&NRF_GPIOTE->TASKS_OUT[1]);

    // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[2] match.
	sd_ppi_channel_assign(5,&NRF_TIMER1->EVENTS_COMPARE[2],&NRF_GPIOTE->TASKS_OUT[1]);


    // Enable PPI channels 0-2.
	sd_ppi_channel_enable_set((PPI_CHEN_CH3_Enabled << PPI_CHEN_CH3_Pos)
                    | (PPI_CHEN_CH4_Enabled << PPI_CHEN_CH4_Pos)
                    | (PPI_CHEN_CH5_Enabled << PPI_CHEN_CH5_Pos));
    //timer2_init
	NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER1->PRESCALER = TIMER_PRESCALERS;

    // Clears the timer, sets it to 0.
    NRF_TIMER1->TASKS_CLEAR = 1;

    // Load the initial values to TIMER2 CC registers.
    NRF_TIMER1->CC[0] = MAX_SAMPLE_LEVELS + 8;
    NRF_TIMER1->CC[1] = MAX_SAMPLE_LEVELS;
    // CC2 will be set on the first CC1 interrupt.
    NRF_TIMER1->CC[2] = 0;

    // Interrupt setup.
    NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
    // Enabling constant latency as indicated by PAN 11 "HFCLK: Base current with HFCLK
    // running is too high" found at Product Anomaly document found at
    // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
    //
    // @note This example does not go to low power mode therefore constant latency is not needed.
    //       However this setting will ensure correct behaviour when routing TIMER events through


    // Enable interrupt on Timer 2.
    NVIC_EnableIRQ(TIMER1_IRQn);

    // Start the timer.
//    NRF_TIMER1->TASKS_START = 1;
}

void pwm_moto_deinit(void)
{
	NRF_TIMER1->TASKS_START = 0;
	nrf_gpiote_unconfig(1);
	sd_ppi_channel_enable_clr((PPI_CHEN_CH3_Enabled << PPI_CHEN_CH3_Pos)
                    | (PPI_CHEN_CH4_Enabled << PPI_CHEN_CH4_Pos)
                    | (PPI_CHEN_CH5_Enabled << PPI_CHEN_CH5_Pos));
}

void pwm_moto_start(void)
{
	NRF_TIMER1->TASKS_START = 1;
}

void pwm_moto_stop(void)
{
	NRF_TIMER1->TASKS_START = 0;
}
//设置马达强度
void pwm_moto_setpower(uint8_t power)
{
	pwm_moto_strong = power;
}

