#include <string.h>
#include <stdint.h>
#include "queue.h"
#include "nrf.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "pstorage.h"

//��������
//������Ϣ����
queue_t queue_entries;
//��ǰ�����blockλ��
uint16_t current_block_num;
//������Ϣ����
queue_items_t queue_items_buff[QUEUE_BLOCK_ITEMS_COUNT];
queue_status_t queue_status = QUEUE_STATUS_UPDATE_READY;
//*************************flash�洢*******************************
//�޸�PSTORAGE_DATA_START_ADDR�궨�壬���ֳ�8k���ڴ洢flash���ݡ

pstorage_handle_t block_id;
static void flash_cb(pstorage_handle_t * handle, uint8_t op_code, uint32_t result,
	uint8_t * p_data, uint32_t data_len)
{
	switch(op_code)
	{
	case PSTORAGE_UPDATE_OP_CODE:
		if (result == NRF_SUCCESS){
			queue_status = QUEUE_STATUS_UPDATE_READY;
		}
		else{
		}
		break;
	default:
		break;
	}
}
#if 1
void queue_init(void)
{
	pstorage_handle_t dest_block_id;
	uint32_t err_code;
	pstorage_module_param_t module_param;
	module_param.block_count = 8;
	module_param.block_size = 1024;
	module_param.cb = flash_cb;
	pstorage_init();
	err_code = pstorage_register(&module_param, &block_id);
	APP_ERROR_CHECK(err_code);
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_load((uint8_t*)&queue_entries, &dest_block_id, sizeof(queue_t),0);
	if (queue_entries.entries == 0xFFFF && queue_entries.tx_point == 0xFFFF \
		&& queue_entries.rx_point == 0xFFFF) {
	//��һ�ο���flash����û�д洢����
		memset((char*)&queue_entries,0x00,sizeof(queue_entries));
	//��ʼ��������Ϊ1�����ڱ����������
		queue_entries.entries = 1;
		queue_entries.tx_point = 1;
		queue_entries.rx_point   = 1;
	}
}

void queue_push(queue_items_t *item)
{
	pstorage_size_t offset;
	pstorage_size_t block_num;
	pstorage_handle_t dest_block_id;
	if (queue_status != QUEUE_STATUS_UPDATE_READY)
	{
		return ;
	}

	block_num = queue_entries.tx_point / QUEUE_BLOCK_ITEMS_COUNT;
	offset = (queue_entries.tx_point % QUEUE_BLOCK_ITEMS_COUNT) * sizeof(queue_items_t);

	//���б���
	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)item,sizeof(queue_items_t),offset);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);

	if (queue_entries.entries != QUEUE_MAX_ENTRIES)
	{//����δ��
		queue_entries.entries++;
	}
	else
	{
		queue_entries.rx_point++;
		if (queue_entries.rx_point % QUEUE_MAX_ENTRIES == 0)
		{
			queue_entries.rx_point = 1;
		}
	}

	queue_entries.tx_point ++;
	if (queue_entries.tx_point % QUEUE_MAX_ENTRIES == 0)
	{//�������е�һ������Ϊ��һ�����ڴ洢������Ϣ
		queue_entries.tx_point = 1;
	}
	//������Ϣ����
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,sizeof(queue_t),0);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
}

void queue_pop(queue_items_t *item)
{
	pstorage_size_t offset;
	pstorage_size_t block_num;
	pstorage_handle_t dest_block_id;

	if (queue_entries.entries == 1)
	{
		return ;
	}

	block_num = queue_entries.tx_point / QUEUE_BLOCK_ITEMS_COUNT;
	offset = (queue_entries.tx_point % QUEUE_BLOCK_ITEMS_COUNT) * sizeof(queue_items_t);

	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
	pstorage_load((uint8_t*)&item, &dest_block_id, sizeof(queue_items_t),offset);

	if (queue_entries.entries != 1)
	{
		queue_entries.entries--;
	}
	queue_entries.rx_point ++;
	if (queue_entries.rx_point % QUEUE_MAX_ENTRIES == 0)
	{
		queue_entries.rx_point = 1;
	}
	//������Ϣ����
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,sizeof(queue_t),0);
	queue_status = QUEUE_STATUS_UPDATING;
}
#else
void queue_init(void)
{
	pstorage_handle_t dest_block_id;
	uint32_t err_code;
	pstorage_module_param_t module_param;
	module_param.block_count = 8;
	module_param.block_size = 1024;
	module_param.cb = flash_cb;
	pstorage_init();
	err_code = pstorage_register(&module_param, &block_id);
	APP_ERROR_CHECK(err_code);
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_load((uint8_t*)&queue_entries, &dest_block_id, sizeof(queue_t),0);
	if (queue_entries.entries == 0xFFFF && queue_entries.tx_point == 0xFFFF \
		&& queue_entries.rx_point == 0xFFFF) {
	//��һ�ο���flash����û�д洢����
		memset((char*)&queue_entries,0x00,sizeof(queue_entries));
	//��ʼ��������Ϊ1�����ڱ����������
		queue_entries.entries = 1;
		queue_entries.tx_point = 1;
		queue_entries.rx_point   = 1;
	}
	//��ʼ���������
	current_block_num = queue_entries.tx_point / QUEUE_BLOCK_ITEMS_COUNT;
	pstorage_block_identifier_get(&block_id, current_block_num, &dest_block_id);
	pstorage_load((uint8_t*)&queue_items_buff, &dest_block_id,\
		sizeof(queue_items_t) * QUEUE_BLOCK_ITEMS_COUNT,0);
}

void queue_push(queue_items_t *item)
{
	pstorage_size_t offset;
	pstorage_size_t block_num;
	pstorage_handle_t dest_block_id;
	if (queue_status != QUEUE_STATUS_UPDATE_READY)
	{
		return ;
	}

	block_num = queue_entries.tx_point / QUEUE_BLOCK_ITEMS_COUNT;
	offset = (queue_entries.tx_point % QUEUE_BLOCK_ITEMS_COUNT) * sizeof(queue_items_t);

	//���б���
	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)item,sizeof(queue_items_t),offset);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
	while(queue_status == QUEUE_STATUS_UPDATING);


	if (queue_entries.entries != QUEUE_MAX_ENTRIES)
	{//����δ��
		queue_entries.entries++;
	}
	else
	{
		queue_entries.rx_point++;
		if (queue_entries.rx_point % QUEUE_MAX_ENTRIES == 0)
		{
			queue_entries.rx_point = 1;
		}
	}

	queue_entries.tx_point ++;
	if (queue_entries.tx_point % QUEUE_MAX_ENTRIES == 0)
	{//�������е�һ������Ϊ��һ�����ڴ洢������Ϣ
		queue_entries.tx_point = 1;
	}

	//������Ϣ����
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,sizeof(queue_t),0);
	queue_status = QUEUE_STATUS_UPDATING;
}

void queue_pop(queue_items_t *item)
{
	pstorage_size_t offset;
	pstorage_size_t block_num;
	pstorage_handle_t dest_block_id;

	if (queue_entries.entries == 1)
	{
		return ;
	}

	block_num = queue_entries.tx_point / QUEUE_BLOCK_ITEMS_COUNT;
	offset = (queue_entries.tx_point % QUEUE_BLOCK_ITEMS_COUNT) * sizeof(queue_items_t);

	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
	pstorage_load((uint8_t*)&item, &dest_block_id, sizeof(queue_items_t),offset);

	if (queue_entries.entries != 1)
	{
		queue_entries.entries--;
	}
	queue_entries.rx_point ++;
	if (queue_entries.rx_point % QUEUE_MAX_ENTRIES == 0)
	{
		queue_entries.rx_point = 1;
	}
	//������Ϣ����
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,sizeof(queue_t),0);
	queue_status = QUEUE_STATUS_UPDATING;
}
#endif

