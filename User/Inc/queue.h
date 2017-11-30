#ifndef __QUEUE_H__
#define __QUEUE_H__


//队列结构体
typedef struct {
	uint16_t entries;
	uint16_t tx_point;
	uint16_t rx_point;
	uint16_t Reserved;
}queue_t;

typedef struct {
	uint16_t angle; //干涉角度
	uint16_t time; //干涉时间
	uint8_t mac_add[6]; //设备的MAC地址
	uint16_t device_bonded; //设备绑定标志
	uint8_t moto_strong;
	uint8_t Reserved;
}system_params_t;

typedef struct {
	uint8_t year;
	uint8_t mon;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t second;
	uint16_t angle;
}queue_items_t;

/**************************干涉条件******************************/
#define DEFAULT_ALARM_ANGLE				75  /**< 默认干涉角度 单位: °>**/
#define DEFAULT_ALARM_TIME              120 /**< 默认干涉时间 单位: s >**/

/**************************存储队列******************************/
//flash block 大小
#define FLASH_BLOCK               1024
//Flash block 数量
#define FLASH_BLOCK_NUM           6
//flash空间大小
#define FLASH_SIZE                (FLASH_BLOCK * FLASH_BLOCK_NUM)
//队列信息占用空间(用于存储当前队列状态)
#define QUEUE_STATUS_SIZE         sizeof(queue_t)
//队列信息开始地址
#define QUEUE_STATUS_ADDRESS      0
//系统信息占用空间数量(用于存储系统信息，干涉条件，绑定信息等)
#define SYSTEM_PARAMS_SIZE         sizeof(system_params_t)
//系统信息开始地址
#define SYSTEM_PARAMS_ADDRESS     (QUEUE_STATUS_ADDRESS + QUEUE_STATUS_SIZE)
//队列起始地址
#define QUEUE_BEGING_ADDRESS      (QUEUE_STATUS_SIZE+SYSTEM_PARAMS_SIZE)
//队列元素大小
#define QUEUE_ITEM_SIZE           sizeof(queue_items_t)
//每一个block中包含的元素个数
#define QUEUE_BLOCK_ITEMS_COUNT (FLASH_BLOCK / QUEUE_ITEM_SIZE)
//队列最大深度
#define QUEUE_MAX_ENTRIES  	      ((FLASH_SIZE - QUEUE_BEGING_ADDRESS)/QUEUE_ITEM_SIZE)
//当前队列深度
#define QUEUE_ENTRIES_NUM         QUEUE_MAX_ENTRIES

typedef enum {
	QUEUE_STATUS_UPDATING,
	QUEUE_STATUS_UPDATE_READY,
	QUEUE_STATUS_UPDATE_FAILED,
	QUEUE_STATUS_LOADING,
	QUEUE_STATUS_LOAD_SUCCESS,
	QUEUE_STATUS_LOAD_ERROR,
}queue_status_t;

extern queue_t queue_entries;
extern system_params_t system_params;

void queue_init(void);
uint8_t queue_pop(queue_items_t *item);
void queue_push(queue_items_t *item);
uint16_t queue_get_entries(void);
void system_params_save(system_params_t * params);
void queue_sync(void);


#endif
