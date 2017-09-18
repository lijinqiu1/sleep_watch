#ifndef __QUEUE_H__
#define __QUEUE_H__

//**************************存储队列******************************
//第一个元素用于存储队列信息。
#define QUEUE_MAX_ENTRIES  	8000
//每一个block中包含的元素个数
#define QUEUE_BLOCK_ITEMS_COUNT 128

//队列结构体
typedef struct {
	uint16_t entries;
	uint16_t tx_point;
	uint16_t rx_point;
	uint16_t Reserved;
}queue_t;

typedef struct {
	uint8_t year;
	uint8_t mon;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t second;
	uint8_t angle;
	uint8_t Reserved;
}queue_items_t;


typedef enum {
	QUEUE_STATUS_UPDATING,
	QUEUE_STATUS_UPDATE_READY,
	QUEUE_STATUS_UPDATE_FAILED,
}queue_status_t;

void queue_init(void);

#endif
