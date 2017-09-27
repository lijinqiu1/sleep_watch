#ifndef __QUEUE_H__
#define __QUEUE_H__

//**************************存储队列******************************
//flash block 大小
#define FLASH_BLOCK               1024
//Flash block 数量
#define FLASH_BLOCK_NUM           6
//flash空间大小
#define FLASH_SIZE                (FLASH_BLOCK * FLASH_BLOCK_NUM)
//队列信息占用空间(用于存储当前队列状态)
#define QUEUE_STATUS_SIZE         8
//系统信息占用空间数量(用于存储系统信息，干涉条件，绑定信息等)
#define SYSTEM_PARAM_SIZE         16
//队列起始地址
#define QUEUE_BEGING_ADDRESS      (QUEUE_STATUS_SIZE+SYSTEM_PARAM_SIZE)
//队列元素大小
#define QUEUE_ITEM_SIZE           8
//每一个block中包含的元素个数
#define QUEUE_BLOCK_ITEMS_COUNT (FLASH_BLOCK / QUEUE_ITEM_SIZE)
//队列最大深度
#define QUEUE_MAX_ENTRIES  	      ((FLASH_SIZE - QUEUE_BEGING_ADDRESS)/QUEUE_ITEM_SIZE)
//当前队列深度
#define QUEUE_ENTRIES_NUM         QUEUE_MAX_ENTRIES
#if QUEUE_ENTRIES_NUM > QUEUE_MAX_ENTRIES
	#error "entries number must litter QUEUE_MAX_ENTRIES!"
#endif


//队列结构体
typedef struct {
	uint16_t entries;
	uint16_t tx_point;
	uint16_t rx_point;
	uint16_t Reserved;
}queue_t;

typedef struct {
	uint16_t angle; //干涉角度
	uint16_t timer; //干涉时间
	uint8_t mac_add[6]; //设备的MAC地址
	uint16_t device_bonded; //设备绑定标志
	uint16_t Reserved[2];
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


#endif
