#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************�¼�����***************************************/
#define EVENT_KEY_PRESSED               (uint32_t)(0x00000001 << 0)                  /**< �����¼� >**/
#define EVENT_MESSAGE_RECEIVED          (uint32_t)(0x00000001 << 1)					 /**< ͨ���¼� >**/
#define EVENT_DATA_SENDING				(uint32_t)(0x00000001 << 2)					 /**< ���ݷ����¼� >**/
#define EVENT_DATA_SENDED               (uint32_t)(0x00000001 << 3)					 /**< ���ݷ�������¼� >**/
#define EVENT_QUEUE_PUSH				(uint32_t)(0x00000001 << 4)                  /**< �Ƕȴ洢 >**/
#define EVENT_ALARM_HAPPEN              (uint32_t)(0x00000001 << 5)                  /**< �������� >**/
#define EVENT_ADV_START                 (uint32_t)(0x00000001 << 6)                  /**< ֹͣ�㲥 >**/
#define EVENT_ADV_STOP                  (uint32_t)(0x00000001 << 7)                  /**< ��ʼ�㲥 >**/
#define EVENT_BATTRY_VALUE              (uint32_t)(0x00000001 << 8)                  /**< ��ȡ��ص�ѹ >**/
#define EVENT_LIS3DH_VALUE              (uint32_t)(0x00000001 << 9)                  /**< ��ȡ3������ >**/
#define EVENT_TILT_PUSH                 (uint32_t)(0x00000001 <<10)                  /**< �Ƕ�ֵ�洢 >**/
#define EVENT_BLE_DISCONNECT            (uint32_t)(0x00000001 <<11)                  /**< �Ͽ��������� >**/
#define EVENT_DATA_SYNC                 (uint32_t)(0x00000001 <<12)                  /**< ͬ���������� >**/
#define EVENT_POWER_LOW                 (uint32_t)(0x00000001 <<13)                  /**< ��ص�ѹ�� >**/
#define EVENT_POWER_CHARGING            (uint32_t)(0x00000001 <<14)                  /**< ��س�� >**/ 
#define EVENT_POWER_CHARGE_COMPLETE     (uint32_t)(0x00000001 <<15)                  /**< ��س����� >**/

#endif

