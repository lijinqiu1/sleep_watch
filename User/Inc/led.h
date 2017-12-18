#ifndef __LED_H__
#define __LED_H__


#define LED_RED                              6                                           /**< LED to indicate advertising state. */
#define LED_GREEN                            7                                           /**< LED to indicate connected state. */

typedef struct LED_FLASH_METHOD
{
	uint8_t flash_count; //闪烁次数
	uint8_t flash_time;  //闪烁时长
	uint8_t flash_period; //闪烁周期
}Led_Flash_Method_t;

typedef enum LED_WORK_STATUS
{
    LED_WORK_POWER_CHARGE_COMPLETE,
    LED_WORK_POWER_CHARGING,
    LED_WORK_POWER_LOW,
    LED_WORK_BLE_DATA_TRAING_ERROR,
    LED_WORK_BLE_DATA_TRAING,
    LED_WORK_DATA_FULL,
    LED_WORK_BEGIN,
    LED_WORK_END,
	LED_IDLE,
}Led_Work_Status_t;

void leds_init(void);
void leds_process_init(Led_Work_Status_t status);
void leds_process(void);

#endif

