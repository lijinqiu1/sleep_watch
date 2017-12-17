#include <string.h>
#include <stdint.h>
#include "nrf.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "pstorage.h"
#include "pwm.h"
#include "queue.h"

//±‰¡ø…˘√˜
//œµÕ≥≤Œ ˝–≈œ¢
system_params_t system_params;
//∂”¡––≈œ¢±‰¡ø
queue_t queue_entries;
//µ±«∞±£¥ÊµƒblockŒª÷√
uint16_t current_block_num;
//∂”¡––≈œ¢ª∫¥Ê
//queue_items_t queue_items_buff[QUEUE_BLOCK_ITEMS_COUNT];
queue_status_t queue_status = QUEUE_STATUS_UPDATE_READY;
//*************************flash¥Ê¥¢*******************************
//–ﬁ∏ƒPSTORAGE_DATA_START_ADDR∫Í∂®“Â£¨ªÆ∑÷≥ˆ8k”√”⁄¥Ê¥¢flash ˝æ›°

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

	//ªÒ»°∂”¡––≈œ¢
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_load((uint8_t*)&queue_entries, &dest_block_id, QUEUE_STATUS_SIZE ,QUEUE_STATUS_ADDRESS);
	//ªÒ»°œµÕ≥–≈œ¢
	pstorage_load((uint8_t*)&system_params, &dest_block_id, SYSTEM_PARAMS_SIZE,SYSTEM_PARAMS_ADDRESS);
	if (queue_entries.entries == 0xFFFF || queue_entries.tx_point == 0xFFFF \
		|| queue_entries.rx_point == 0xFFFF) {
	//µ⁄“ª¥Œø™ª˙flash¿Ô√Ê√ª”–¥Ê¥¢ ˝æ›
		memset((char*)&queue_entries,0x00,sizeof(queue_entries));
	}
	if (system_params.angle == 0xFFFF)
	{
		system_params.angle =   DEFAULT_ALARM_ANGLE;
	}
	if ( system_params.time == 0xFFFF)
	{
		system_params.time = DEFAULT_ALARM_TIME;
	}
	if ( system_params.moto_strong == 0xFFFF )
	{
		system_params.moto_strong = DEFAULT_MOTO_STRONG;
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
	//∂”¡–±£¥Ê
	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)item,QUEUE_ITEM_SIZE,offset);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
	if (queue_status != QUEUE_STATUS_UPDATE_READY)
	{//»Áπ˚¥Ê¥¢ ß∞‹µ˜÷¡œ¬“ª∏ˆ…»«¯
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
	{//∂”¡–Œ¥¬˙
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
	//∂”¡––≈œ¢±£¥Ê
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
	if (queue_status != QUEUE_STATUS_LOAD_SUCCESS)
	{//»Áπ˚¥Ê¥¢ ß∞‹µ˜÷¡œ¬“ª∏ˆ…»«¯
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
	//∂”¡––≈œ¢±£¥Ê
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,QUEUE_STATUS_SIZE,QUEUE_STATUS_ADDRESS);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
*/
#endif
	return 0;
}

//”√”⁄±£¥Ê∂”¡––≈œ¢,±‹√‚÷ÿ∏¥–¥»ÎÕ¨“ªøÈflash…»«¯
void queue_sync(void)
{
	pstorage_handle_t dest_block_id;
	//∂”¡––≈œ¢±£¥Ê
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

//¥Ê¥¢œµÕ≥≤Œ ˝
void system_params_save(system_params_t * params)
{
	pstorage_handle_t dest_block_id;
	while (queue_status != QUEUE_STATUS_UPDATE_READY);

	pstorage_block_identifier_get(&block_id, 0,&dest_block_id);
	pstorage_update(&dest_block_id, (uint8_t *)params,sizeof(system_params_t),sizeof(queue_t));
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
}

