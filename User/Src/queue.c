#include <string.h>
#include <stdint.h>
#include "queue.h"
#include "nrf.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "pstorage.h"

//变量声明
//系统参数信息
system_params_t system_params;
//队列信息变量
queue_t queue_entries;
//当前保存的block位置
uint16_t current_block_num;
//队列信息缓存
//queue_items_t queue_items_buff[QUEUE_BLOCK_ITEMS_COUNT];
queue_status_t queue_status = QUEUE_STATUS_UPDATE_READY;
//*************************flash存储*******************************
//修改PSTORAGE_DATA_START_ADDR宏定义，划分出8k用于存储flash数据�

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
			queue_status = QUEUE_STATUS_UPDATE_FAILED;
		}
		break;
	case PSTORAGE_LOAD_OP_CODE:
		if (result == NRF_SUCCESS){
			queue_status = QUEUE_STATUS_UPDATE_READY;
		}
		else{
			queue_status = QUEUE_STATUS_LOAD_ERROR;
		}
		break;
	default:
		break;
	}
}

void queue_init(void)
{
	pstorage_handle_t dest_block_id;
	uint32_t err_code;
	pstorage_module_param_t module_param;

	module_param.block_count = FLASH_BLOCK_NUM;
	module_param.block_size = FLASH_BLOCK;
	module_param.cb = flash_cb;

	err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

	err_code = pstorage_register(&module_param, &block_id);
	APP_ERROR_CHECK(err_code);

	//获取队列信息
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_load((uint8_t*)&queue_entries, &dest_block_id, QUEUE_STATUS_SIZE ,QUEUE_STATUS_ADDRESS);
	//获取系统信息
	pstorage_load((uint8_t*)&system_params, &dest_block_id, SYSTEM_PARAMS_SIZE,SYSTEM_PARAMS_ADDRESS);
	if (queue_entries.entries == 0xFFFF || queue_entries.tx_point == 0xFFFF \
		|| queue_entries.rx_point == 0xFFFF) {
	//第一次开机flash里面没有存储数据
		memset((char*)&queue_entries,0x00,sizeof(queue_entries));
	}
	if (system_params.angle == 0xFFFF || system_params.time )
	{
		system_params.angle =   DEFAULT_ALARM_ANGLE;
		system_params.time = DEFAULT_ALARM_TIME;
	}
}

void queue_push(queue_items_t *item)
{
	pstorage_size_t offset;
	pstorage_size_t block_num;
	pstorage_handle_t dest_block_id;
	while (queue_status != QUEUE_STATUS_UPDATE_READY);

RETRY:
	block_num = ((queue_entries.tx_point * QUEUE_ITEM_SIZE) + QUEUE_BEGING_ADDRESS) / FLASH_BLOCK;
	offset = ((queue_entries.tx_point * QUEUE_ITEM_SIZE) + QUEUE_BEGING_ADDRESS) % FLASH_BLOCK;
	//队列保存
	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)item,QUEUE_ITEM_SIZE,offset);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
	if (queue_status != QUEUE_STATUS_UPDATE_READY)
	{//如果存储失败调至下一个扇区
		if (queue_entries.entries == QUEUE_ENTRIES_NUM)
		{
			queue_entries.rx_point++;
			if (queue_entries.rx_point % QUEUE_ENTRIES_NUM == 0)
			{
				queue_entries.rx_point = 0;
			}
		}
		queue_entries.tx_point ++;
		if (queue_entries.tx_point % QUEUE_ENTRIES_NUM == 0)
		{
			queue_entries.tx_point = 0;
		}
		goto RETRY;
	}
	if (queue_entries.entries != QUEUE_ENTRIES_NUM)
	{//队列未满
		queue_entries.entries++;
	}
	else
	{
		queue_entries.rx_point++;
		if (queue_entries.rx_point % QUEUE_ENTRIES_NUM == 0)
		{
			queue_entries.rx_point = 0;
		}
	}

	queue_entries.tx_point ++;
	if (queue_entries.tx_point % QUEUE_ENTRIES_NUM == 0)
	{
		queue_entries.tx_point = 0;
	}
/*
	//队列信息保存
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,QUEUE_STATUS_SIZE,QUEUE_STATUS_ADDRESS);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
*/
}

uint8_t queue_pop(queue_items_t *item)
{
	pstorage_size_t offset;
	pstorage_size_t block_num;
	pstorage_handle_t dest_block_id;

	while (queue_status != QUEUE_STATUS_UPDATE_READY);

	if (queue_entries.entries == 0)
	{
		return 1;
	}
RETRY:
	block_num = ((queue_entries.rx_point * QUEUE_ITEM_SIZE) + QUEUE_BEGING_ADDRESS) / FLASH_BLOCK;
	offset = ((queue_entries.rx_point * QUEUE_ITEM_SIZE) + QUEUE_BEGING_ADDRESS) % FLASH_BLOCK;


	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
	queue_status = QUEUE_STATUS_LOADING;
	pstorage_load((uint8_t*)item, &dest_block_id, QUEUE_ITEM_SIZE,offset);
	while(queue_status == QUEUE_STATUS_LOADING);
	if (queue_status != QUEUE_STATUS_LOAD_SUCCESS)
	{//如果存储失败调至下一个扇区
		queue_entries.rx_point ++;
		if (queue_entries.rx_point % QUEUE_ENTRIES_NUM == 0)
		{
			queue_entries.rx_point = 0;
		}
		goto RETRY;
	}
	if (queue_entries.entries != 0)
	{
		queue_entries.entries--;
	}
	queue_entries.rx_point ++;
	if (queue_entries.rx_point % QUEUE_ENTRIES_NUM == 0)
	{
		queue_entries.rx_point = 0;
	}
/*
	//队列信息保存
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,QUEUE_STATUS_SIZE,QUEUE_STATUS_ADDRESS);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
*/
	return 0;
}

//用于保存队列信息,避免重复写入同一块flash扇区
void queue_sync(void)
{
	pstorage_handle_t dest_block_id;
	//队列信息保存
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,QUEUE_STATUS_SIZE,QUEUE_STATUS_ADDRESS);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
	return ;
}
uint16_t queue_get_entries(void)
{
	return queue_entries.entries;
}

//存储系统参数
void system_params_save(system_params_t * params)
{
	pstorage_handle_t dest_block_id;
	while (queue_status != QUEUE_STATUS_UPDATE_READY);

	pstorage_block_identifier_get(&block_id, 0,&dest_block_id);
	pstorage_update(&dest_block_id, (uint8_t *)params,sizeof(system_params_t),sizeof(queue_t));
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
}

