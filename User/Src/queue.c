#include <string.h>
#include <stdint.h>
#include "nrf.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "pstorage.h"
#include "pwm.h"
#include "queue.h"

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

//存储接受报文信息
uint8_t queue_message_data[10][20];
uint8_t queue_message_entries;
uint8_t queue_message_tx;
uint8_t queue_message_rx;
//*************************flash存储*******************************
//修改PSTORAGE_DATA_START_ADDR宏定义，划分出8k用于存储flash数据?

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
	if ( system_params.time[0] == 0xFF)
	{
		memset(system_params.time, 0x00, 6);
	}
	if ( system_params.moto_strong == 0xFF )
	{
		system_params.moto_strong = DEFAULT_MOTO_STRONG;
	}
    if (system_params.device_bonded == 0xFF)
    {
        system_params.device_bonded = 0;
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
#if defined(DEBUG_APP)
	uint16_t angle[60] = {
			 00, 10, 20, 30, 40, 50, 60, 70, 80, 90,
			100,140,180,140,100, 50, 00, 50,100,140,
			180,240,280,320,360, 50, 40, 50, 40, 50,
		     40, 50, 40, 50, 40, 50, 40, 50, 40, 50,
		     40, 50, 40, 50, 40, 50, 40, 50, 40, 50,
		     40, 80, 90, 80, 90, 80, 90, 80, 90, 80,
			};
	if (queue_entries.entries == 0)
	{
		queue_entries.entries = 60;
		return 1;
	}
	queue_entries.entries --;
	item->year = 0x11;
	item->mon = 0x01;
	item->day = 0x01;
	item->hour = 0x01;
	item->min = 59 - queue_entries.entries;
	item->second = 0x00;
	item->angle = angle[59 - queue_entries.entries] * 100;
#else
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
	if (queue_status != QUEUE_STATUS_UPDATE_READY)
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
#endif
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

uint8_t queue_is_full(void)
{
	if ((QUEUE_ENTRIES_NUM - queue_entries.entries) > (20 * 10))
	{//队列未满
		return 0;
	}
	else
	{//队列满
		return 1;
	}
}

void queue_message_init(void)
{
    queue_message_rx = 0;
    queue_message_tx = 0;
    queue_message_entries = 0;
}

void queue_message_push(uint8_t *message)
{
    memcpy(queue_message_data[queue_message_tx],message,20);
    queue_message_tx++;
    if (queue_message_tx == 10)
    {
        queue_message_tx = 0;
    }
    if(queue_message_entries != 10)
    {
        queue_message_entries ++;
    }
    else
    {
        queue_message_rx++;
        if (queue_message_rx == 10)
        {
            queue_message_rx = 0;
        }
    }
    
}

uint8_t queue_message_pop(uint8_t *message)
{
    if (queue_message_entries == 0)
    {
        return 1;
    }
    memcpy(queue_message_data[queue_message_rx],message,20);
    queue_message_rx++;
    if (queue_message_rx == 10)
    {
        queue_message_rx = 0;
    }
    queue_message_entries--;
    return 0;
}

