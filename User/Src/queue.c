#include <string.h>
#include <stdint.h>
#include "queue.h"
#include "nrf.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "pstorage.h"

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
#if 1
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
	
	queue_entries.entries = 3;

//	//ªÒ»°∂”¡––≈œ¢
//	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
//	pstorage_load((uint8_t*)&queue_entries, &dest_block_id, QUEUE_STATUS_SIZE ,QUEUE_STATUS_ADDRESS);
//	//ªÒ»°œµÕ≥–≈œ¢
//	pstorage_load((uint8_t*)&system_params, &dest_block_id, SYSTEM_PARAMS_SIZE,SYSTEM_PARAMS_ADDRESS);
//	if (queue_entries.entries == 0xFFFF || queue_entries.tx_point == 0xFFFF \
//		|| queue_entries.rx_point == 0xFFFF) {
//	//µ⁄“ª¥Œø™ª˙flash¿Ô√Ê√ª”–¥Ê¥¢ ˝æ›
//		memset((char*)&queue_entries,0x00,sizeof(queue_entries));
//	}
//	if (system_params.angle == 0xFFFF || system_params.time )
//	{
//		system_params.angle =   DEFAULT_ALARM_ANGLE;
//		system_params.time = DEFAULT_ALARM_TIME;
//	}
}

void queue_push(queue_items_t *item)
{
	pstorage_size_t offset;
	pstorage_size_t block_num;
	pstorage_handle_t dest_block_id;
	while (queue_status != QUEUE_STATUS_UPDATE_READY);

//	block_num = queue_entries.tx_point / QUEUE_BLOCK_ITEMS_COUNT;
//	offset = (queue_entries.tx_point % QUEUE_BLOCK_ITEMS_COUNT) * sizeof(queue_items_t);

	block_num = ((queue_entries.tx_point * QUEUE_ITEM_SIZE) + QUEUE_BEGING_ADDRESS) / FLASH_BLOCK;
	offset = ((queue_entries.tx_point * QUEUE_ITEM_SIZE) + QUEUE_BEGING_ADDRESS) % FLASH_BLOCK;
	//∂”¡–±£¥Ê
	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)item,QUEUE_ITEM_SIZE,offset);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
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
	//∂”¡––≈œ¢±£¥Ê
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,QUEUE_STATUS_SIZE,QUEUE_STATUS_ADDRESS);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);
}

uint8_t queue_pop(queue_items_t *item)
{
	pstorage_size_t offset;
	pstorage_size_t block_num;
	pstorage_handle_t dest_block_id;

	while (queue_status != QUEUE_STATUS_UPDATE_READY);

	if (queue_entries.entries == 0)
	{
		queue_entries.entries = 3;
		return 1;
	}
	
	item->year = 0x11;
	item->mon = 0x0b;
	item->day = 0x12;
	item->hour = 0x00;
	item->min = 0x00;
	item->second = 0x00;
	item->angle = 1000;
	if (queue_entries.entries != 0)
	{
		queue_entries.entries--;
	}
	return 0;
//	if (queue_entries.entries == 0)
//	{
//		return 1;
//	}

////	block_num = queue_entries.rx_point / QUEUE_BLOCK_ITEMS_COUNT;
////	offset = (queue_entries.rx_point % QUEUE_BLOCK_ITEMS_COUNT) * sizeof(queue_items_t);

//	block_num = ((queue_entries.rx_point * QUEUE_ITEM_SIZE) + QUEUE_BEGING_ADDRESS) / FLASH_BLOCK;
//	offset = ((queue_entries.rx_point * QUEUE_ITEM_SIZE) + QUEUE_BEGING_ADDRESS) % FLASH_BLOCK;


//	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
//	queue_status = QUEUE_STATUS_LOADING;
//	pstorage_load((uint8_t*)item, &dest_block_id, QUEUE_ITEM_SIZE,offset);
//	while(queue_status == QUEUE_STATUS_LOADING);
//	if (queue_entries.entries != 0)
//	{
//		queue_entries.entries--;
//	}
//	queue_entries.rx_point ++;
//	if (queue_entries.rx_point % QUEUE_ENTRIES_NUM == 0)
//	{
//		queue_entries.rx_point = 0;
//	}
//	//∂”¡––≈œ¢±£¥Ê
//	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
//	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,QUEUE_STATUS_SIZE,QUEUE_STATUS_ADDRESS);
//	queue_status = QUEUE_STATUS_UPDATING;
//	while(queue_status == QUEUE_STATUS_UPDATING);
	return 0;
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
	//µ⁄“ª¥Œø™ª˙flash¿Ô√Ê√ª”–¥Ê¥¢ ˝æ›
		memset((char*)&queue_entries,0x00,sizeof(queue_entries));
	//≥ı º∂”¡– ˝¡øŒ™1£¨”√”⁄±£¥Ê∂”¡– ˝æ›
		queue_entries.entries = 1;
		queue_entries.tx_point = 1;
		queue_entries.rx_point   = 1;
	}
	//≥ı ºªØª∫¥Ê∂”¡–
//	current_block_num = queue_entries.tx_point / QUEUE_BLOCK_ITEMS_COUNT;
//	pstorage_block_identifier_get(&block_id, current_block_num, &dest_block_id);
//	pstorage_load((uint8_t*)&queue_items_buff, &dest_block_id,\
//		sizeof(queue_items_t) * QUEUE_BLOCK_ITEMS_COUNT,0);
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

	//∂”¡–±£¥Ê
	pstorage_block_identifier_get(&block_id, block_num, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)item,sizeof(queue_items_t),offset);
	queue_status = QUEUE_STATUS_UPDATING;
	while(queue_status == QUEUE_STATUS_UPDATING);


	if (queue_entries.entries != QUEUE_MAX_ENTRIES)
	{//∂”¡–Œ¥¬˙
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
	{//Ã¯π˝∂”¡–µ⁄“ªÃı£¨“ÚŒ™µ⁄“ªÃı”√”⁄¥Ê¥¢∂”¡––≈œ¢
		queue_entries.tx_point = 1;
	}

	//∂”¡––≈œ¢±£¥Ê
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
	//∂”¡––≈œ¢±£¥Ê
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_update(&dest_block_id,(uint8_t *)&queue_entries,sizeof(queue_t),0);
	queue_status = QUEUE_STATUS_UPDATING;
}
#endif
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

