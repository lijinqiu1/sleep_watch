#ifndef __LED_H__
#define __LED_H__


#define LED_RED                              6                                           /**< LED to indicate advertising state. */
#define LED_GREEN                            7                                           /**< LED to indicate connected state. */
#define LED_FLASH_RATE_ON                 0xff
#define LED_FLASH_RATE_BREATH             0xfe
#define LED_FLASH_RATE_1HZ                0xfd
#define LED_FLASH_RATE_FAST               0xfc
#define LED_FLASH_RATE_OFF                0xfb
#define LED_FLASH_LED_RED                 0x01
#define LED_FLASH_LED_GREEN               0x02
#define LED_FLASH_LED_BOTH                0x03
#define LED_FLASH_LED_NONE                0x00
#define LED_FLASH_TIME_LONG               0xff

typedef struct LED_FLASH_METHOD
{
	uint8_t flash_rate; //闪烁频率 0xff 常亮, 0xfe 呼吸灯, 0xfd 1Hz, 0xfc flash fast,
	uint8_t flash_time;  //闪烁时长 单位s 0xff flash forever
	uint8_t flash_period; //闪烁周期  单位分钟
	uint8_t flash_led;    //闪烁的led，1:led_red 2:led_green 3:red and green 0:both red and green off
	uint32_t flash_end_time;
}Led_Flash_Method_t;

typedef enum LED_WORK_STATUS
{
    LED_WORK_POWER_USB_DISCONNECTED,
    LED_WORK_POWER_CHARGE_COMPLETE,
    LED_WORK_POWER_CHARGING,
    LED_WORK_POWER_LOW,
    LED_WORK_DATA_FULL,
    LED_WORK_BLE_DATA_TRAING_ERROR,
    LED_WORK_BLE_DATA_TRAING,
    LED_WORK_BLE_CONNECTED,
    LED_WORK_BEGIN,
    LED_WORK_END,
	LED_IDLE,
}Led_Work_Status_t;

void leds_init(void);
void leds_process_init(Led_Work_Status_t status);
void leds_process(void);

#endif

