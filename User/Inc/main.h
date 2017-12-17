#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************事件定义***************************************/
#define EVENT_KEY_PRESSED               (uint32_t)(0x00000001 << 0)                  /**< 按键事件 >**/
#define EVENT_MESSAGE_RECEIVED          (uint32_t)(0x00000001 << 1)					 /**< 通信事件 >**/
#define EVENT_DATA_SENDING				(uint32_t)(0x00000001 << 2)					 /**< 数据发送事件 >**/
#define EVENT_DATA_SENDED               (uint32_t)(0x00000001 << 3)					 /**< 数据发送完成事件 >**/
#define EVENT_QUEUE_PUSH				(uint32_t)(0x00000001 << 4)                  /**< 角度存储 >**/
#define EVENT_ALARM_HAPPEN              (uint32_t)(0x00000001 << 5)                  /**< 报警产生 >**/
#define EVENT_ADV_START                 (uint32_t)(0x00000001 << 6)                  /**< 停止广播 >**/
#define EVENT_ADV_STOP                  (uint32_t)(0x00000001 << 7)                  /**< 开始广播 >**/
#define EVENT_BATTRY_VALUE              (uint32_t)(0x00000001 << 8)                  /**< 获取电池电压 >**/
#define EVENT_LIS3DH_VALUE              (uint32_t)(0x00000001 << 9)                  /**< 获取3轴数据 >**/
#define EVENT_TILT_PUSH                 (uint32_t)(0x00000001 <<10)                  /**< 角度值存储 >**/
#define EVENT_BLE_DISCONNECT            (uint32_t)(0x00000001 <<11)                  /**< 断开蓝牙连接 >**/
#define EVENT_DATA_SYNC                 (uint32_t)(0x00000001 <<12)                  /**< 同步队列数据 >**/
#define EVENT_POWER_LOW                 (uint32_t)(0x00000001 <<13)                  /**< 电池电压低 >**/
#define EVENT_POWER_CHARGING            (uint32_t)(0x00000001 <<14)                  /**< 电池充电 >**/ 
#define EVENT_POWER_CHARGE_COMPLETE     (uint32_t)(0x00000001 <<15)                  /**< 电池充电完成 >**/

#endif

