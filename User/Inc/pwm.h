#ifndef __PWM_H__
#define __PWM_H__

//pwm
#define MAX_SAMPLE_LEVELS (256UL)  /**< Maximum number of sample levels. */
#define TIMER_PRESCALERS   6U       /**< Prescaler setting for timer. */

#define MOTO_STRONG_ZERO     0
#define MOTO_STRONG_ONE      64
#define MOTO_STRONG_TWO      128
#define MOTO_STRONG_THREE    192
#define MOTO_STRONG_FULL     256

#define PWM_LED_PIN          12
#define PWM_MOTO_PIN         16


#define DEFAULT_MOTO_STRONG MOTO_LEVEL_AUTO

typedef enum pwm_moto_level
{
	MOTO_LEVEL_0,
	MOTO_LEVEL_25,
	MOTO_LEVEL_50,
	MOTO_LEVEL_75,
	MOTO_LEVEL_100,
	MOTO_LEVEL_AUTO,
}PwmMotoLevel_t;

typedef enum pwm_status
{
	PWM_STATUS_READY,
	PWM_STATUS_MOTO,
	PWM_STATUS_LED,
}PwmStatusEN_t;

void pwm_led_start(void);
void pwm_led_stop(void);
void pwm_moto_init(void);
void pwm_moto_start(void);
void pwm_moto_setpower(uint8_t power);
void pwm_moto_stop(void);

#endif
