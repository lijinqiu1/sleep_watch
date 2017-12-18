#ifndef __BATTERY_H__
#define __BATTERY_H__

#define BQ24040_PG_PIN      11
#define BQ24040_CHG_PIN     12

typedef enum BATTERY_CHARGE_STATUS
{
    BATTERY_NOT_CHARGE,
    BATTERY_CHARGING,
    BATTERY_CHARGE_COMPLETE,
    BATTERY_VALUE_LOW,
}Battery_Charge_Status_e;

void battery_init(void);
void battery_manager(void);
void battery_get_status(void);
uint16_t battery_get_value(void);

#endif
