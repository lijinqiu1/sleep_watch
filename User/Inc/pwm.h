#ifndef __PWM_H__
#define __PWM_H__

//pwm
#define MAX_SAMPLE_LEVELS (256UL)  /**< Maximum number of sample levels. */
#define TIMER_PRESCALERS  6U       /**< Prescaler setting for timer. */

#define PWM_BUSY      1
#define PWM_READY     0

enum pwm_status
{
	PWM_STATUS_READY,
	PWM_STATUS_MOTO,
	PWM_STATUS_LED,
}PwmStatusEN_t;

void pwm_led_init(uint32_t pin_number);
void pwm_led_deinit(void);
#endif
