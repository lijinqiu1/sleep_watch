#ifndef __LED_H__
#define __LED_H__


#define LED1                            6                                           /**< LED to indicate advertising state. */
#define LED0                            7                                           /**< LED to indicate connected state. */

typedef enum LED_WORK_STATUS
{
    LED_WORK_POWER_LED,
    LED_WORK_POWER_CHARGING,
    LED_WORK_POWER_CHARGE_COMPLETE,
    LED_WORK_BLE_DATA_TRAING,
    LED_WORK_BLE_DATA_TRAING_ERROR,
    LED_WORK_DATA_FULL,
    LED_WORK_BEGIN,
    LED_WORK_END,
}Led_Work_Status_t;

void leds_init(void);


#endif

