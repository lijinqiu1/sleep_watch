#ifndef __QUEUE_H__
#define __QUEUE_H__

//**************************�洢����******************************
//��һ��Ԫ�����ڴ洢������Ϣ��
#define QUEUE_MAX_ENTRIES  	10
//ÿһ��block�а�����Ԫ�ظ���
#define QUEUE_BLOCK_ITEMS_COUNT 128

//���нṹ��
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
	uint16_t angle;
}queue_items_t;


typedef enum {
	QUEUE_STATUS_UPDATING,
	QUEUE_STATUS_UPDATE_READY,
	QUEUE_STATUS_UPDATE_FAILED,
}queue_status_t;

extern queue_t queue_entries;

void queue_init(void);
uint8_t queue_pop(queue_items_t *item);
void queue_push(queue_items_t *item);
uint16_t queue_get_entries(void);


#endif
