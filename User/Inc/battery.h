#ifndef __BATTERY_H__
#define __BATTERY_H__

#define BQ24040_PG_PIN      11
#define BQ24040_CHG_PIN     12

#define BATTER_VALUE_LOW    768
#define BATTER_VALUE_20     794
#define BATTER_VALUE_40     820
#define BATTER_VALUE_60     845
#define BATTER_VALUE_80     870
#define BATTER_VALUE_100    880

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
Battery_Charge_Status_e battery_get_charege_status(void);


#endif
