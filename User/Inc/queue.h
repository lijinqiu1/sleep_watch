#ifndef __QUEUE_H__
#define __QUEUE_H__

//**************************�洢����******************************
//flash block ��С
#define FLASH_BLOCK               1024
//Flash block ����
#define FLASH_BLOCK_NUM           6
//flash�ռ��С
#define FLASH_SIZE                (FLASH_BLOCK * FLASH_BLOCK_NUM)
//������Ϣռ�ÿռ�(���ڴ洢��ǰ����״̬)
#define QUEUE_STATUS_SIZE         8
//ϵͳ��Ϣռ�ÿռ�����(���ڴ洢ϵͳ��Ϣ����������������Ϣ��)
#define SYSTEM_PARAM_SIZE         16
//������ʼ��ַ
#define QUEUE_BEGING_ADDRESS      (QUEUE_STATUS_SIZE+SYSTEM_PARAM_SIZE)
//����Ԫ�ش�С
#define QUEUE_ITEM_SIZE           8
//ÿһ��block�а�����Ԫ�ظ���
#define QUEUE_BLOCK_ITEMS_COUNT (FLASH_BLOCK / QUEUE_ITEM_SIZE)
//����������
#define QUEUE_MAX_ENTRIES  	      ((FLASH_SIZE - QUEUE_BEGING_ADDRESS)/QUEUE_ITEM_SIZE)
//��ǰ�������
#define QUEUE_ENTRIES_NUM         QUEUE_MAX_ENTRIES
#if QUEUE_ENTRIES_NUM > QUEUE_MAX_ENTRIES
	#error "entries number must litter QUEUE_MAX_ENTRIES!"
#endif


//���нṹ��
typedef struct {
	uint16_t entries;
	uint16_t tx_point;
	uint16_t rx_point;
	uint16_t Reserved;
}queue_t;

typedef struct {
	uint16_t angle; //����Ƕ�
	uint16_t timer; //����ʱ��
	uint8_t mac_add[6]; //�豸��MAC��ַ
	uint16_t device_bonded; //�豸�󶨱�־
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
