#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
#include "softdevice_handler.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "pwm.h"
#include "queue.h"
#include "app_trace.h"

static uint8_t pwm_led_status = 0;
static uint8_t pwm_moto_strong = 0;
static const uint8_t moto_strong[4] = {MOTO_STRONG_ONE,MOTO_STRONG_TWO,
									MOTO_STRONG_THREE,MOTO_STRONG_FULL};
static uint8_t moto_strong_index = 0;
static uint8_t moto_time = 0; //记录震动时长
static uint8_t moto_count = 0; //记录震动次数
static bool tim2_cc0_turn = false; /**< Keeps track of which CC register to be used. */
static bool tim1_cc0_turn = false; /**< Keeps track of which CC register to be used. */

//#define USB_ONLY_ONE_TIMER

/** @brief Function for handling timer 2 peripheral interrupts.
 */
void TIMER2_IRQHandler(void)
{
    
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
        if (tim2_cc0_turn)
        {
            NRF_TIMER2->CC[0] = NRF_TIMER2->CC[1] + next_sample;
        }
        else
        {
            NRF_TIMER2->CC[2] = NRF_TIMER2->CC[1] + next_sample;
        }
        // Next turn the other CC will get its value.
        tim2_cc0_turn = !tim2_cc0_turn;

    }
}

void TIMER1_IRQHandler(void)
{
	//呼吸灯
    if ((NRF_TIMER1->EVENTS_COMPARE[1] != 0) &&
       ((NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
    {
        // Sets the next CC1 value
        NRF_TIMER1->EVENTS_COMPARE[1] = 0;
        NRF_TIMER1->CC[1]             = (NRF_TIMER1->CC[1] + MAX_SAMPLE_LEVELS);

        if (tim1_cc0_turn)
        {
            NRF_TIMER1->CC[0] = NRF_TIMER1->CC[1] + pwm_moto_strong;
        }
        else
        {
            NRF_TIMER1->CC[2] = NRF_TIMER1->CC[1] + pwm_moto_strong;
        }
        // Next turn the other CC will get its value.
        tim1_cc0_turn = !tim1_cc0_turn;
    }
}

void pwm_led_start(uint32_t led)
{
    if (pwm_led_status != PWM_STATUS_READY)
    {
        return ;
    }
    tim2_cc0_turn = 0;

    pwm_led_status = PWM_LED_MODE;
	//gpiote_init
	// Connect GPIO input buffers and configure PWM_OUTPUT_PIN_NUMBER as an output.
    nrf_gpio_cfg_output(led);

    nrf_gpio_pin_clear(led);
    // Configure GPIOTE channel 0 to toggle the PWM pin state
    // @note Only one GPIOTE task can be connected to an output pin.
    nrf_gpiote_task_config(0, led, \
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
    if (pwm_led_status == 0)
    {
        return;
    }
    pwm_led_status = PWM_STATUS_READY;
	NRF_TIMER2->TASKS_START = 0;
	nrf_gpiote_unconfig(0);
	sd_ppi_channel_enable_clr((PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                    | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos)
                    | (PPI_CHEN_CH2_Enabled << PPI_CHEN_CH2_Pos));
}

static void pwm_moto_init(void)
{
    tim1_cc0_turn = 0;
	//gpiote_init
	// Connect GPIO input buffers and configure PWM_OUTPUT_PIN_NUMBER as an output.
    nrf_gpio_cfg_output(PWM_MOTO_PIN);

    nrf_gpio_pin_clear(PWM_MOTO_PIN);
//    nrf_gpio_pin_set(PWM_MOTO_PIN);
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
	NRF_TIMER1->CC[0] = MAX_SAMPLE_LEVELS + pwm_moto_strong;
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
	NRF_TIMER1->TASKS_START = 1;
    app_trace_log("%s!\n",__FUNCTION__);
}

static void pwm_moto_deinit(void)
{
	NRF_TIMER1->TASKS_START = 0;
	nrf_gpiote_unconfig(1);
	sd_ppi_channel_enable_clr((PPI_CHEN_CH3_Enabled << PPI_CHEN_CH3_Pos)
                    | (PPI_CHEN_CH4_Enabled << PPI_CHEN_CH4_Pos)
                    | (PPI_CHEN_CH5_Enabled << PPI_CHEN_CH5_Pos));
    nrf_gpio_cfg_output(PWM_MOTO_PIN);
	nrf_gpio_pin_clear(PWM_MOTO_PIN);
    app_trace_log("%s!\n",__FUNCTION__);
}


void alarm_init(void)
{
	moto_time = 0;
	moto_count = 0;
	moto_strong_index = 0;
    app_trace_log("%s!\n",__FUNCTION__);
}

void alarm_process(void)
{
	if (moto_count < 10)
	{
		switch(moto_time)
		{
		case 0:
			if (system_params.moto_strong == MOTO_LEVEL_AUTO)
			{
				pwm_moto_strong = moto_strong[moto_strong_index];
                app_trace_log("%s index %d time %d count %d!\n",__FUNCTION__,moto_strong_index,moto_time,moto_count);
				if (moto_strong_index < 3)
				{
					moto_strong_index++;
				}
			}
			else
			{
				pwm_moto_strong = system_params.moto_strong;
			}
    		pwm_moto_init();
			moto_time++;
			break;
		case 3:
			pwm_moto_deinit();
            app_trace_log("%s index %d time %d count %d!\n",__FUNCTION__,moto_strong_index,moto_time,moto_count);
			moto_time++;
			break;
		case 8:
            app_trace_log("%s index %d time %d count %d!\n",__FUNCTION__,moto_strong_index,moto_time,moto_count);
			moto_time = 0;
			moto_count++;
			break;
		default:
			moto_time++;
			break;
		}
	}
}

void alarm_case(void)
{
	pwm_moto_deinit();
    app_trace_log("%s!\n",__FUNCTION__);
}
