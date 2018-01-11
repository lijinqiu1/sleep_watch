/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "nrf_gpiote.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "softdevice_handler.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_util_platform.h"
#include "app_gpiote.h"
#include "app_trace.h"
#include "simple_uart.h"
#include "lis3dh_driver.h"
#include "calender.h"
#include "queue.h"
#include "pstorage.h"
#include "device_manager.h"
#include "pwm.h"
#include "adc.h"
#include "led.h"
#include "battery.h"
#include "main.h"

#define SOFT_VERSION     20180111-1

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define WAKEUP_BUTTON_PIN               BUTTON_0                                    /**< Button used to wake up the application. */

#define DEVICE_NAME                     "Watch"                                     /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_ADV_INTERVAL_FAST            MSEC_TO_UNITS(25, UNIT_0_625_MS)               /**< Fast advertising interval (25 ms.). */
#define APP_ADV_INTERVAL_SLOW            MSEC_TO_UNITS(2000, UNIT_0_625_MS)             /**< Slow advertising interval (2 seconds). */
#define APP_FAST_ADV_TIMEOUT             30                                             /**< The duration of the fast advertising period (in seconds). */
#define APP_SLOW_ADV_TIMEOUT             180                                            /**< The duration of the slow advertising period (in seconds). */
#define APP_DIRECTED_ADV_TIMEOUT         5                                              /**< number of direct advertisement (each lasting 1.28seconds). */


#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            4                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(300, UNIT_10_MS)              /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_DISPLAY_ONLY                /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define START_STRING                    "Start...\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define LIS3DH_SMAPLE_RATE				1											/**< 三轴加速度采样频率 单位:秒 >**/

#define ANGLE_SMAPLE_RATE				180											/**< 角度数据采样频率 单位:秒 >**/
//gpiote
#define MAX_USERS						1
//蓝牙拦截配对密码
#define PAIR_PASS_WORD                  "123456"

#if defined (ADV_GENERAL)
static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
#endif
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
#if defined (ADV_WHITELIST) || defined(ADV_BOND)
static dm_application_instance_t        m_app_handle;                              /**< Application identifier allocated by device manager */
static dm_handle_t                      m_bonded_peer_handle;                          /**< Device reference handle to the current bonded central. */
static uint8_t                          m_direct_adv_cnt;                              /**< Counter of direct advertisements. */
static uint8_t                          m_advertising_mode;                            /**< Variable to keep track of when we are advertising. */
static ble_gap_addr_t                   m_ble_addr;                                    /**< Variable for getting and setting of BLE device address. */
#endif
static bool                             m_memory_access_in_progress = false;        /**< Flag to keep track of ongoing operations on persistent memory. */

//gpiote user identifier
static app_gpiote_user_id_t gpiote_user_id;
static void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low);

//周期性时间
static app_timer_id_t p_timer;
//配对password
static  dm_handle_t                      m_dm_handle;                                       /**< Identifes the peer that is currently connected. */
static  app_timer_id_t                   m_sec_req_timer_id;
//按键防抖
static app_timer_id_t                    m_key_tiemr_id;
#define SECURITY_REQUEST_DELAY          APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Delay after connection until Security Request is sent, if necessary (ticks). */


static void period_cycle_process(void * p_context);

static uint32_t g_event_status = 0;                                             //事件存储变量
static bool g_status_key_pressed = false;                                       //记录按键事件
static bool g_status_adv = false;                                               //广播状态
static bool g_status_work = false;                                              //设备工作状态
static bool g_status_data_send = false;                                         //发送数据
static bool g_status_ble_connect = false;                                       //蓝牙连接状态
static bool g_status_alarm_status = false;                                      //报警状态
static bool g_status_bond_info_received = false;                                //接收到绑定信息
static bool g_status_tilt_init_flag = false;                                    //角度值初始化
static float g_cur_Tilt;                                                        //当前倾角变化值
static uint8_t rec_data_buffer[20];                                             //缓存接收到的数据
static uint8_t rec_data_length;

typedef enum
{
	CMD_SET_TIME = 0x01,
	CMD_REQUEST_DATA,
	CMD_SEND_DATA,
	CMD_SEND_DATA_COMPLETED,
	CMD_SET_ALARM,
	CMD_DEVICE_BOND,
	CMD_GET_BATTERY,
}MESSAGE_CMD_ID_t;

typedef enum
{
    BLE_NO_ADV,               /**< No advertising running. */
    BLE_DIRECTED_ADV,         /**< Direct advertising to the latest central. */
    BLE_FAST_ADV_WHITELIST,   /**< Advertising with whitelist. */
    BLE_FAST_ADV,             /**< Fast advertising running. */
    BLE_SLOW_ADV,             /**< Slow advertising running. */
    BLE_SLEEP,                /**< Go to system-off. */
} ble_advertising_mode_t;

/**@brief     Error handler function, which is called when an error has occurred.
 *
 * @warning   This handler is an example only and does not fit a final product. You need to analyze
 *            how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);
	app_trace_log("0x%x %d %s\r\n", error_code,line_num,p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief       Assert macro callback function.
 *
 * @details     This function will be called in case of an assert in the SoftDevice.
 *
 * @warning     This handler is an example only and does not fit a final product. You need to
 *              analyze how your product is supposed to react in case of Assert.
 * @warning     On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling the Security Request timer timeout.
 *
 * @details This function will be called each time the Security Request timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    uint32_t             err_code;
    dm_security_status_t status;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = dm_security_status_req(&m_dm_handle, &status);
        APP_ERROR_CHECK(err_code);

        // In case the link is secured by the peer during timeout, the request is not sent.
        if (status == NOT_ENCRYPTED)
        {
            err_code = dm_security_setup_req(&m_dm_handle);
            APP_ERROR_CHECK(err_code);
        }
    }
}

static void key_req_timeout_handler(void * p_context)
{

}


/**@brief   Function for Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
	uint32_t err_code;
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

	err_code = app_timer_create(&p_timer,
		                        APP_TIMER_MODE_REPEATED,
		                        period_cycle_process);
	APP_ERROR_CHECK(err_code);

	// Create Security Request timer.
	err_code = app_timer_create(&m_sec_req_timer_id,
								APP_TIMER_MODE_SINGLE_SHOT,
								sec_req_timeout_handler);
	APP_ERROR_CHECK(err_code);

    //按键防抖定时器
	err_code = app_timer_create(&m_key_tiemr_id,
		                        APP_TIMER_MODE_SINGLE_SHOT,
		                        key_req_timeout_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(p_timer,APP_TIMER_TICKS(1000,APP_TIMER_PRESCALER),NULL);
	APP_ERROR_CHECK(err_code);
}


/**@brief   Function for the GAP initialization.
 *
 * @details This function will setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

	ble_opt_t      static_options;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
#if defined (ADV_WHITELIST) || defined(ADV_BOND)

	//初始化蓝牙匹配码功能
	uint8_t passkey[] = PAIR_PASS_WORD;
	static_options.gap.passkey.p_passkey = passkey;
	err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &static_options);
	APP_ERROR_CHECK(err_code);
#endif

}
#if defined(ADV_WHITELIST) || defined(ADV_BOND)
/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 *
 * @param[in]  adv_flags  Indicates which type of advertisement to use, see @ref BLE_GAP_DISC_MODES.
 */
static void advertising_init(uint8_t adv_flags)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    ble_uuid_t adv_uuids[] = { { BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE } };

    err_code = sd_ble_gap_address_get(&m_ble_addr);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &m_ble_addr);
    APP_ERROR_CHECK(err_code);

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(adv_flags);
    advdata.flags.p_data            = &adv_flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}
#else
/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

	//无限广播模式
	//uint8_t			flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	//广播超时模式
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    uint8_t i;
    for(i = 0; i < length;i++)
    {
        app_trace_log("%02x ",p_data[i]);
    }
    app_trace_log("\n");
	app_trace_log("%s,%d,length:%d\r\n",__FUNCTION__,__LINE__,length);
	if ((p_data[0] == 0xA5) && (p_data[length - 1] == 0x80))
	{
		//memcpy(rec_data_buffer,p_data,length);
        //rec_data_length = length;
        queue_message_push(p_data);
		g_event_status |= EVENT_MESSAGE_RECEIVED;
	}
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_nus_init_t   nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
#if defined (ADV_GENERAL)
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
#endif
}


/**@brief       Function for handling an event from the Connection Parameters Module.
 *
 * @details     This function will be called for all events in the Connection Parameters Module
 *              which are passed to the application.
 *
 * @note        All this function does is to disconnect. This could have been done by simply setting
 *              the disconnect_on_fail config parameter, but instead we use the event handler
 *              mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
		app_trace_log("%s %d sd_ble_gap_disconnect\r\n",__FUNCTION__,__LINE__);
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief       Function for handling errors from the Connection Parameters module.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
#if defined (ADV_BOND)

static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    ble_gap_whitelist_t  whitelist;
    uint32_t             count;

	err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }
    // Initialize advertising parameters with defaults values
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.p_whitelist = NULL;

    adv_params.interval = APP_ADV_INTERVAL_FAST;
    adv_params.timeout  = APP_FAST_ADV_TIMEOUT;

	ble_gap_addr_t		 * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
	ble_gap_irk_t		 * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

	whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
	whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
	whitelist.pp_addrs	 = p_whitelist_addr;
	whitelist.pp_irks	 = p_whitelist_irk;

	err_code = dm_whitelist_create(&m_app_handle, &whitelist);
	APP_ERROR_CHECK(err_code);

	if ((whitelist.addr_count != 0) || (whitelist.irk_count != 0))
	{
		if (system_params.device_bonded == 0xFF)
		{
            err_code = dm_device_delete_all(&m_app_handle);
 			APP_ERROR_CHECK(err_code);
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
		}
		else
		{
			adv_params.fp		   = BLE_GAP_ADV_FP_FILTER_CONNREQ;
			adv_params.p_whitelist = &whitelist;
			advertising_init(BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
		}
	}

    // Start advertising.
    err_code = sd_ble_gap_adv_start(&adv_params);
	while (err_code != NRF_SUCCESS)
	{
		err_code = sd_ble_gap_adv_start(&adv_params);
	}
    APP_ERROR_CHECK(err_code);
}

#elif defined (ADV_WHITELIST)
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    ble_gap_whitelist_t  whitelist;
    ble_gap_addr_t       peer_address;
    uint32_t             count;

    // Verify if there is any flash access pending, if yes delay starting advertising until
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }
    // Initialize advertising parameters with defaults values
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.p_whitelist = NULL;

    // Configure advertisement according to current advertising state.
    if (m_advertising_mode == BLE_DIRECTED_ADV)
    {
        err_code = dm_peer_addr_get(&m_bonded_peer_handle, &peer_address);
        if (err_code != NRF_SUCCESS)
        {
            m_advertising_mode = BLE_FAST_ADV_WHITELIST;
        }
    }
    switch (m_advertising_mode)
    {
        case BLE_NO_ADV:
            m_advertising_mode = BLE_FAST_ADV_WHITELIST;
            // Fall through.

        case BLE_FAST_ADV_WHITELIST:
        {
            ble_gap_addr_t       * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t        * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

            whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
            whitelist.pp_addrs   = p_whitelist_addr;
            whitelist.pp_irks    = p_whitelist_irk;

            err_code = dm_whitelist_create(&m_app_handle, &whitelist);
            APP_ERROR_CHECK(err_code);

            if ((whitelist.addr_count != 0) || (whitelist.irk_count != 0))
            {
                adv_params.fp          = BLE_GAP_ADV_FP_FILTER_CONNREQ;
                adv_params.p_whitelist = &whitelist;

                advertising_init(BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
                m_advertising_mode = BLE_FAST_ADV;
            }
            else
            {
                m_advertising_mode = BLE_SLOW_ADV;
            }

            adv_params.interval = APP_ADV_INTERVAL_FAST;
            adv_params.timeout  = APP_FAST_ADV_TIMEOUT;
			app_trace_log("%s ,%d, BLE_FAST_ADV_WHITELIST\r\n",__FUNCTION__,__LINE__);
            break;
        }

        case BLE_DIRECTED_ADV:
            adv_params.p_peer_addr = &peer_address;
            adv_params.type        = BLE_GAP_ADV_TYPE_ADV_DIRECT_IND;
            adv_params.timeout     = 0;

            m_direct_adv_cnt--;
            if (m_direct_adv_cnt == 0)
            {
                m_advertising_mode = BLE_FAST_ADV_WHITELIST;
            }
			app_trace_log("%s ,%d, BLE_DIRECTED_ADV\r\n",__FUNCTION__,__LINE__);
            break;

        case BLE_FAST_ADV:
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);

            adv_params.interval = APP_ADV_INTERVAL_FAST;
            adv_params.timeout  = APP_FAST_ADV_TIMEOUT;
            m_advertising_mode  = BLE_SLOW_ADV;
			app_trace_log("%s ,%d, BLE_FAST_ADV\r\n",__FUNCTION__,__LINE__);
            break;

        case BLE_SLOW_ADV:
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);

            adv_params.interval = APP_ADV_INTERVAL_SLOW;
            adv_params.timeout  = APP_SLOW_ADV_TIMEOUT;
            m_advertising_mode  = BLE_SLEEP;
			app_trace_log("%s ,%d, BLE_SLOW_ADV\r\n",__FUNCTION__,__LINE__);
            break;

        default:
            // No implementation needed.
            break;
    }

    // Start advertising.
    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}


#elif defined(ADV_GENERAL)
/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;

    //adv_params.timeout     = 0;//广播超时 三分钟APP_ADV_TIMEOUT_IN_SECONDS

    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}
#endif
/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
#if !defined(ADV_GENERAL)
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           api_result_t           event_result)
{
    uint32_t err_code = NRF_SUCCESS;

    m_dm_handle = *p_handle;
//    APP_ERROR_CHECK(event_result);
    switch (p_event->event_id)
    {
#if defined (ADV_WHITELIST)
		case DM_EVT_DEVICE_CONTEXT_LOADED: // Fall through.
        case DM_EVT_SECURITY_SETUP_COMPLETE:
			if (event_result == NRF_SUCCESS)
            	m_bonded_peer_handle = (*p_handle);
            break;
#else
        case DM_EVT_CONNECTION:
            // Start Security Request timer.
//            if (m_dm_handle.device_id != DM_INVALID_ID)
//            {

//            }
//            else
//			{
				err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            	APP_ERROR_CHECK(err_code);
//            }
            break;
#endif
        default:
            break;
    }
    return NRF_SUCCESS;
}
#endif
/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
#if defined (ADV_GENERAL)
    ble_gap_enc_info_t *             p_enc_info;
#endif

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
#if defined (ADV_WHITELIST)
			m_advertising_mode = BLE_NO_ADV;
#endif
			g_event_status |= EVENT_BLE_CONNECTED;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			g_event_status |= EVENT_BLE_DISCONNECTED;
#if defined (ADV_WHITELIST)
			m_advertising_mode = BLE_DIRECTED_ADV;
            m_direct_adv_cnt   = APP_DIRECTED_ADV_TIMEOUT;
#endif
#if defined (DEBUG_MODE)
			advertising_start();
#endif
            break;
        case BLE_GAP_EVT_TIMEOUT:
#if defined (ADV_BOND)
			advertising_start();
#elif defined (ADV_WHITELIST)
			if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
			{
				if (m_advertising_mode == BLE_SLEEP)
				{
					m_advertising_mode = BLE_NO_ADV;
					advertising_start();
				}
				else
				{
					advertising_start();
				}
			}
#elif defined (ADV_GENERAL)
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {

                // Configure buttons with sense level low as wakeup source.
//                nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
//                                         BUTTON_PULL,
//                                         NRF_GPIO_PIN_SENSE_LOW);

                // Go to system-off mode (this function will not return; wakeup will cause a reset)
//                err_code = sd_power_system_off();
//                APP_ERROR_CHECK(err_code);
            }
#endif
            break;
#if defined (ADV_GENERAL)

    	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

#elif defined (ADV_WHITELIST) || defined (ADV_BOND)
            
        case BLE_GAP_EVT_PASSKEY_DISPLAY:
            // Don't send delayed Security Request if security procedure is already in progress.
            app_trace_log("%s, %d, passkey:%c%c%c%c%c%c\r\n",__FUNCTION__,__LINE__,p_ble_evt->evt.gap_evt.params.passkey_display.passkey[0],\
            p_ble_evt->evt.gap_evt.params.passkey_display.passkey[1],\
            p_ble_evt->evt.gap_evt.params.passkey_display.passkey[2],\
            p_ble_evt->evt.gap_evt.params.passkey_display.passkey[3],\
            p_ble_evt->evt.gap_evt.params.passkey_display.passkey[4],\
            p_ble_evt->evt.gap_evt.params.passkey_display.passkey[5]);
            break;
        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            if (m_auth_status.auth_status != BLE_GAP_SEC_STATUS_SUCCESS)
            {
                app_trace_log("%s %d sd_ble_gap_disconnect\r\n",__FUNCTION__,__LINE__);
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
                APP_ERROR_CHECK(err_code);
                g_status_ble_connect = false;
            }
            break;
#endif


		case BLE_GAP_EVT_CONN_SEC_UPDATE:

			break;
		default:
			// No implementation needed.
			break;
    }
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                advertising_start();
            }
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief       Function for dispatching a S110 SoftDevice event to all modules with a S110
 *              SoftDevice event handler.
 *
 * @details     This function is called from the S110 SoftDevice event interrupt handler after a
 *              S110 SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
#if defined(ADV_WHITELIST) || defined(ADV_BOND)
	dm_ble_evt_handler(p_ble_evt);
#endif
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
	pstorage_sys_event_handler(sys_evt);
	on_sys_evt(sys_evt);
}
/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}
//gpiote handler function
static void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
	if (event_pins_high_to_low & (uint32_t)(1<<BUTTON_1))
	{
		if (g_status_data_send == false) {
			//数据传输功能时按键无效
			//button 1 pressed
			g_status_key_pressed = true;
			//app_timer_start(m_key_tiemr_id,APP_TIMER_TICKS(2000,APP_TIMER_PRESCALER),NULL);
		}
	}
}



/**@brief Function for the Device Manager initialization.
 */
#if !defined(ADV_GENERAL)
static void device_manager_init(void)
{
    uint32_t                err_code;
    dm_init_param_t         init_data;
    dm_application_param_t  register_param;

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    init_data.clear_persistent_data = false;

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.timeout      = SEC_PARAM_TIMEOUT;
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}
#endif
/**@brief  Function for configuring the buttons.
 */
static void buttons_init(void)
{
#if 0
	uint32_t low_to_high_bitmask = (uint32_t)(1 << BUTTON_1);
	uint32_t high_to_low_bitmask = (uint32_t)(1 << BUTTON_1);
	uint32_t err_code;

	nrf_gpio_cfg_sense_input(BUTTON_1,
						 BUTTON_PULL,
						 NRF_GPIO_PIN_SENSE_LOW);
	//初始化gpio引脚中断
	APP_GPIOTE_INIT(MAX_USERS);
	err_code = app_gpiote_user_register(&gpiote_user_id,
										low_to_high_bitmask,
										high_to_low_bitmask,
										gpiote_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_gpiote_user_enable(gpiote_user_id);
	APP_ERROR_CHECK(err_code);
#else
    nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_NOPULL);
#endif
}


/**@brief  Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


///**@brief  Function for initializing the UART module.
// */
//static void uart_init(void)
//{
//    /**@snippet [UART Initialization] */
//    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);

//    NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos;

//    NVIC_SetPriority(UART0_IRQn, APP_IRQ_PRIORITY_LOW);
//    NVIC_EnableIRQ(UART0_IRQn);
//    /**@snippet [UART Initialization] */
//}


/**@brief   Function for handling UART interrupts.
 *
 * @details This function will receive a single character from the UART and append it to a string.
 *          The string will be be sent over BLE when the last character received was a 'new line'
 *          i.e '\n' (hex 0x0D) or if the string has reached a length of @ref NUS_MAX_DATA_LENGTH.
 */
//void UART0_IRQHandler(void)
//{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//    static uint8_t index = 0;
//    uint32_t err_code;

//    /**@snippet [Handling the data received over UART] */

//    data_array[index] = simple_uart_get();
//    index++;

//    if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN - 1)))
//    {
//        err_code = ble_nus_send_string(&m_nus, data_array, index + 1);
//        if (err_code != NRF_ERROR_INVALID_STATE)
//        {
//            APP_ERROR_CHECK(err_code);
//        }

//        index = 0;
//    }

//    /**@snippet [Handling the data received over UART] */
//}


/***********************************睡眠姿态管理***********************************/
void sleep_manage(void)
{
	static uint8_t last_sleep_post = 0;
	static uint8_t cur_sleep_post = 0;
	static uint32_t cur_timeseconds = 0;
	static float cur_Tilt = 0; //记录报警时刻的角度值
	if(g_status_tilt_init_flag == true)
	{
		if ((ALARM_SLEEP_POSE_ONE_BEGIN <= g_cur_Tilt)&&
			(g_cur_Tilt <= ALARM_SLEEP_POSE_ONE_END))
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_ONE;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_TWO_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_TWO;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_THREE_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_THREE;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_FOUR_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_FOUR;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_FIVE_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_FIVE;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_SIX_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_SIX;
		}
		cur_timeseconds = TimeSeconds;
		g_status_tilt_init_flag = false;
	}
	else
	{
		if ((ALARM_SLEEP_POSE_ONE_BEGIN <= g_cur_Tilt)&&
			(g_cur_Tilt <= ALARM_SLEEP_POSE_ONE_END))
		{
			cur_sleep_post = ALARM_SLEEP_POSE_ONE;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_TWO_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_TWO;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_THREE_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_THREE;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_FOUR_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_FOUR;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_FIVE_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_FIVE;
		}
		else if (g_cur_Tilt <= ALARM_SLEEP_POSE_SIX_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_SIX;
		}
		if ((last_sleep_post == cur_sleep_post) &&
			((TimeSeconds - cur_timeseconds ) >= (system_params.time[cur_sleep_post] * 60)))
		{
			if (g_status_alarm_status == false)
			{
				cur_Tilt = g_cur_Tilt;
				g_status_alarm_status = true;
				alarm_init();
			}
			else if ((g_status_alarm_status == true)&&
				(fabs(cur_Tilt - g_cur_Tilt) > 15.00))
			{//角度变化超过5度，解除报警
				cur_Tilt = g_cur_Tilt;
				g_status_alarm_status = false;
				cur_timeseconds = TimeSeconds;
				alarm_case();
			}
		}
		else if(last_sleep_post != cur_sleep_post)
		{//睡姿变换报警取消
		    if (g_status_alarm_status)
		    {
			    g_status_alarm_status = false;
				alarm_case();
		    }
			last_sleep_post = cur_sleep_post;
			cur_timeseconds = TimeSeconds;
		}
	}
}

/*******************************周期事件处理函数*******************************/
//周期事件处理函数
static void period_cycle_process(void * p_context)
{
	static uint32_t lis3dh_timer = 0;//三轴传感器采样频率
	static uint8_t key_timer = 0;	//按键计时器
	uint8_t key_status;             //按键状态
	static uint16_t angle_timer = 0;	//角度采样频率
	static uint16_t data_send_completed = 0; //用于传输数据结束后关闭蓝牙连接
	static uint16_t battery_timer = 0; //电池电量
	static uint8_t one_shot_timer = 0;
	//模拟日历
	TimeSeconds ++;


	//按键处理
	//if (g_status_key_pressed == 1)
	{
		key_status = nrf_gpio_pin_read(BUTTON_1);
		if (key_status == 0)
		{
			key_timer ++;
            g_status_key_pressed = true;
			if(key_timer > 10)
			{//长按超过10s进如恢复出厂设置
				// On assert, the system can only recover with a reset.
				//NVIC_SystemReset();
                memset((uint8_t *)&system_params,0xFF,sizeof(system_params_t));
                system_params_save(&system_params);
			}
		}
		else if (g_status_key_pressed == true)
		{
			g_status_key_pressed = false;
			if(key_timer >= 4)
			{
				//长按
				g_event_status |= EVENT_KEY_PRESS_LONG;
			}
			else
			{
				//短按
				g_event_status |= EVENT_KEY_PRESS_SHOT;
			}
			key_timer = 0;
		}
	}


	if ((g_status_work == true)/*&&(lis3dh_timer++ >= LIS3DH_SMAPLE_RATE)*/)
	{//使用三轴加速度采样
		lis3dh_timer = 0;
		if (angle_timer++ >= (ANGLE_SMAPLE_RATE-1))
		{
			angle_timer = 0;
			//存储数据
			g_event_status |= EVENT_TILT_PUSH;
		}
		g_event_status |= EVENT_LIS3DH_VALUE;
	}

	if (g_status_ble_connect == true)
	{
		if ((g_event_status & EVENT_DATA_SENDED) && (data_send_completed++ >= 30))
		{
			//数据传输完成后30s断开蓝牙连接
			data_send_completed = 0;
			g_event_status |= EVENT_BLE_SHUT_CONNECT;
			g_event_status&= ~EVENT_DATA_SENDED;
		}
	}

	//电池电量
	if (battery_timer ++ > 30)
	{
		g_event_status |= EVENT_BATTRY_VALUE;
		battery_timer = 0;
	}
    battery_manager();
	//检查存储状态
	if (queue_is_full())
	{
		g_event_status |= EVENT_DATA_FULL;
	}

	if ((g_status_ble_connect == true) &&
		(system_params.device_bonded == true) &&
		(g_status_bond_info_received == false) &&
		(one_shot_timer ++ > 30))
	{
		g_event_status |= EVENT_BLE_SHUT_CONNECT;
	}
	else if (g_status_ble_connect == false)
	{
		one_shot_timer = 0;
		g_status_bond_info_received = false;
	}

	if (g_status_alarm_status)
	{
		alarm_process();
//		nrf_gpio_cfg_output(PWM_MOTO_PIN);
//		nrf_gpio_pin_set(PWM_MOTO_PIN);
	}
}

//*****************倾角计算**************************
#define PI 3.1415926
#if 0
//计算与水平面夹角，结果范围0~180
static float calculateTilt_A(float ax, float ay, float az)
{
	float g = 9.80665;
	float temp;
	float Tiltangle = 0;
	temp = ((sqrt(2)/2)*g/10);
	if (az > temp)
	{
		Tiltangle = (1-ay*ay) - (1-ax*ax);
		if (Tiltangle < 0) {
			Tiltangle = - Tiltangle;
		}

		Tiltangle = acos(sqrt(Tiltangle));
		Tiltangle = Tiltangle/PI*180;
		Tiltangle = 90 - Tiltangle;

	}
	else
	{
		Tiltangle = asin(az);
		Tiltangle = Tiltangle/PI*180;
		Tiltangle = 90-Tiltangle;
	}
	return Tiltangle;
}

//返回角度差
//flag = 1重新获取角度基准值，flag = 0开始计算
//以Y轴作为转动轴
static float calculateTilt_run_A(float ax, float ay, float az)
{
	//初始基准角度值
	static float First_Tiltangle = 0;
	//上一时刻角度值
//	static float last_Tiltangle = 0;
	float Tiltangle = 0;
	//三轴初始位置 1:>=0, 0:<0
	static uint8_t flag_x;
//	static uint8_t flag_y;
//	static uint8_t flag_z;
	//转动趋势,顺时针转动1，逆时针转动0
//	static uint8_t trend = 0;

	if (tilt_init_flag == 1)
	{
		tilt_init_flag = 0;
		First_Tiltangle = calculateTilt_A(ax,ay,az);
		if (ax < 0)
		{
			First_Tiltangle = 360 - First_Tiltangle;
		}
		flag_x = (ax >= 0);
	}
	Tiltangle = calculateTilt_A(ax,ay,az);
	if (ax < 0)
	{
		Tiltangle = 360 - Tiltangle;
	}
	if (First_Tiltangle < Tiltangle)
	{
		if (flag_x ^ (ax >= 0))
		{
			return (360 -(Tiltangle - First_Tiltangle));
		}
		else
			return (Tiltangle - First_Tiltangle);
	}
	else
	{
		if (flag_x ^ (ax >= 0))
		{
			return (360 -(First_Tiltangle - Tiltangle));
		}
		else
			return (First_Tiltangle - Tiltangle);
	}

}
#endif
static float calculateTilt_B(float ax, float ay, float az)
{
	/*
	*关于计算前后转动角度超值问题，记录前一时刻x,y,加速度正负，由正变负+90°
	*/
	float temp;
	float g = 9.80665;
	float Tiltangle = 0;
	temp = ((sqrt(2)/2)*g/10);
	if (fabs(ay) < temp)
	{
		Tiltangle = asin(fabs(ay));
		Tiltangle = Tiltangle/PI*180;
	}
	else
	{
		Tiltangle = acos(fabs(ay));
		Tiltangle = Tiltangle/PI*180;
		Tiltangle = 90-Tiltangle;
	}
	if((az < 0)&&(ay < 0))
	{
		Tiltangle = 90 + Tiltangle;
	}
	else if ((az > 0)&&(ay < 0))
	{
		Tiltangle = 270 - Tiltangle;
	}
	else if ((az > 0) && (ay > 0))
	{
		Tiltangle = 270 + Tiltangle;
	}
    else if ((az < 0) && (ay > 0))
    {
        Tiltangle = 90 - Tiltangle;
    }

	return Tiltangle;
}
#if 0
//返回角度差
//flag = 1重新获取角度基准值，flag = 0开始计算
//以Y轴作为转动轴
static float calculateTilt_run_B(float ax, float ay, float az)
{
	//初始基准角度值
	static float First_Tiltangle = 0;
	float Tiltangle = 0;
    float Tiltangle_return = 0;
	//三轴初始位置 1:>=0, 0:<0
//	static uint8_t flag_x;

	if (tilt_init_flag == 1)
	{
		tilt_init_flag = 0;
		First_Tiltangle = calculateTilt_B(ax,ay,az);
		if (ax < 0)
		{
			First_Tiltangle = 360 - First_Tiltangle;
		}
//		flag_x = (ax >= 0);
	}
	Tiltangle = calculateTilt_B(ax,ay,az);
	app_trace_log("First_Tiltangle %f,Tiltangle %f\n",First_Tiltangle,Tiltangle);
    Tiltangle_return = First_Tiltangle - Tiltangle;
	if (Tiltangle_return < -180.0)
	{
		return (360 + Tiltangle_return);
	}
    else if (Tiltangle_return > 180)
    {
        return (360 - Tiltangle_return);
    }
    else if (Tiltangle_return < 0)
    {
        return -Tiltangle_return;
    }
    else
        return Tiltangle_return;

}
#endif
//报文处理函数
static void message_process(uint8_t *ch)
{
	uint8_t cmd_id;
	UTCTimeStruct tm;
	uint32_t err_code;
	uint8_t data_array[20];
    uint16_t battery;
	if (ch[0] != 0xA5)
	{
		return ;
	}
	cmd_id = ch[2];

	switch(cmd_id)
	{
	case CMD_SET_TIME:
		//设置时间
		tm.year = 2000 + ch[3];
		tm.month = ch[4];
		tm.day = ch[5];
		tm.hour = ch[6];
		tm.minutes = ch[7];
		tm.seconds = ch[8];
		TimeSeconds = ConvertUTCSecs(&tm);
		//发送响应报文
		data_array[0] = 0xA5;
		data_array[1] = 0x01;
		data_array[2] = CMD_SET_TIME;
		data_array[3] = 0x80;
		err_code = ble_nus_send_string(&m_nus, data_array, 4);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
		break;
	case CMD_REQUEST_DATA:
		//开始上传数据
		g_event_status |= EVENT_DATA_SENDING;
		break;
	case CMD_SET_ALARM:
		//设置干涉条件
		//干涉条件有6个
		memcpy(system_params.time,&ch[3],0x06);
		//发送响应报文
		data_array[0] = 0xA5;
		data_array[1] = 0x01;
		data_array[2] = CMD_SET_ALARM;
		data_array[3] = 0x80;
		err_code = ble_nus_send_string(&m_nus, data_array, 4);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
		system_params_save(&system_params);
		break;
	case CMD_DEVICE_BOND:

		if(ch[3] == 0x01)
		{
			if(system_params.device_bonded == 0)
			{
				//设备绑定
				system_params.device_bonded = 0x01;
				memcpy((char *)system_params.mac_add,&ch[4],11);
				system_params_save(&system_params);
				g_status_bond_info_received = true;
				//发送响应报文
				data_array[0] = 0xA5;
				data_array[1] = 0x01;
				data_array[2] = CMD_DEVICE_BOND;
				data_array[3] = 0x80;
				err_code = ble_nus_send_string(&m_nus, data_array, 4);
		        if (err_code != NRF_ERROR_INVALID_STATE)
		        {
		            APP_ERROR_CHECK(err_code);
		        }
			}
			else
			{
				if(memcmp(&ch[4],(char *)system_params.mac_add,11)==0)
				{
					g_status_bond_info_received = true;
				}
			}
		}
		else if (ch[3] == 0x02)
		{
			//设备解绑
			system_params.device_bonded = 0xFF;
			memset((char *)system_params.mac_add,0xFF,11);
			system_params_save(&system_params);
			//发送响应报文
			data_array[0] = 0xA5;
			data_array[1] = 0x01;
			data_array[2] = CMD_DEVICE_BOND;
			data_array[3] = 0x80;
			err_code = ble_nus_send_string(&m_nus, data_array, 4);
	        if (err_code != NRF_ERROR_INVALID_STATE)
	        {
	            APP_ERROR_CHECK(err_code);
	        }
		}
		break;
	case CMD_GET_BATTERY:
		data_array[0] = 0xA5;
		data_array[1] = 0x02;
		data_array[2] = CMD_GET_BATTERY;
        if(battery < BATTER_VALUE_20)
        {
            data_array[3] = 0;
        }
        else if(battery < BATTER_VALUE_40)
        {
            data_array[3] = 20;
        }
        else if(battery < BATTER_VALUE_60)
        {
            data_array[3] = 40;
        }
        else if(battery < BATTER_VALUE_80)
        {
            data_array[3] = 60;
        }
        else if(battery < BATTER_VALUE_100)
        {
            data_array[3] = 80;
        }
        else
        {
            data_array[3] = 100;
        }
		data_array[4] = 0x80;
		err_code = ble_nus_send_string(&m_nus, data_array, 5);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
		break;
	default:
		break;
	}
}
#if defined (QUEUE_TEST)
//队列测试代码
uint8_t test[100];
extern pstorage_handle_t block_id;
void queue_test(void)
{
	uint16_t i = 0;
	queue_items_t item;
	pstorage_handle_t dest_block_id;
	memset(&item, 0x00, sizeof(queue_items_t));

	app_trace_log("%d  %d  %d\r\n",queue_entries.entries,queue_entries.rx_point,\
					queue_entries.tx_point);

	for(i = 0; i< 1045; i++)
	{
		queue_push(&item);
		item.angle ++;
	}
	memset(&item, 0x00, sizeof(queue_items_t));
	for(i = 0; i < 1045; i++)
	{
		if(queue_pop(&item))
		{
			printf("end\r\n");
			break;
		}
		else
		{
			app_trace_log("%d\r\n",item.angle);
		}
        power_manage();
	}

	app_trace_log("%d  %d  %d\r\n",queue_entries.entries,queue_entries.rx_point,\
				queue_entries.tx_point);
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_load(test, &dest_block_id, sizeof(test),0);

}
#endif
/**@brief  Application main function.
 */
int main(void)
{
    // Initialize
    AxesRaw_t Axes_Raw_Data = {0};
    uint8_t response;
    float ax,ay,az;
    UTCTimeStruct time;
    queue_items_t item;
    uint8_t data_array[20];
    uint32_t err_code;
    timers_init();
    app_trace_init();
    ble_stack_init();
	//flash初始化
	queue_init();
    //通信报文缓存队列
    queue_message_init();
#if defined (ADV_WHITELIST)||defined(ADV_BOND)
	device_manager_init();
#endif
    gap_params_init();
    services_init();
#if defined (ADV_WHITELIST)||defined(ADV_BOND)
    advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
#else
	advertising_init();
#endif
    conn_params_init();
    sec_params_init();
    app_trace_log(START_STRING);
#if !defined (DEBUG_APP)
    leds_init();
	battery_init();
    battery_manager();
	battery_get_value();
	//gpiote初始化
    buttons_init();
	//马达驱动初始化
    alarm_init();
	LIS3DH_Init();
#else
	queue_pop(&item);
#endif
//    alarm_start();
#if defined (DEBUG_MODE)
	advertising_start();
#endif
//	LIS3DH_GetWHO_AM_I(&data);
//	simple_uart_put(data);
#if defined (QUEUE_TEST)
	queue_test();
#endif

    // Enter main loop
    for (;;)
    {
        //led 管理
        leds_process();
		if (g_status_work)
		{
			sleep_manage();
		}
		if (g_event_status & EVENT_LIS3DH_VALUE)
		{
			response = LIS3DH_GetAccAxesRaw(&Axes_Raw_Data);
			if (response == 1) {
				ConvertUTCTime(&time,TimeSeconds);
				ax = Axes_Raw_Data.AXIS_X/16384.0;
				ay = Axes_Raw_Data.AXIS_Y/16384.0;
				az = Axes_Raw_Data.AXIS_Z/16384.0;
				app_trace_log("X=%6f Y=%6f Z=%6f \r\n",
					ax,ay,az);
				g_cur_Tilt = calculateTilt_B(ax,ay,az);
				app_trace_log("Tilt = %6f \r\n", g_cur_Tilt);
				app_trace_log("y:%d m:%d d:%d h:%d m:%d s:%d\r\n",\
					          time.year,time.month,time.day,time.hour,time.minutes,time.seconds);
			}
			//存储角度值
			if (g_event_status & EVENT_TILT_PUSH)
			{
				ConvertUTCTime(&time,TimeSeconds);
				item.year = time.year - 2000;
				item.mon = time.month;
				item.day = time.day;
				item.hour = time.hour;
				item.min = time.minutes;
				item.second = time.seconds;
				item.angle = (uint16_t)(g_cur_Tilt*100);
				queue_push(&item);
				g_event_status &= ~ EVENT_TILT_PUSH;
				app_trace_log("data push y:%d m:%d d:%d h:%d m:%d s:%d\r\n",\
					          time.year,time.month,time.day,time.hour,time.minutes,time.seconds);
			}
			g_event_status &= ~EVENT_LIS3DH_VALUE;
		}
		if (g_event_status & EVENT_KEY_PRESS_SHOT)
		{
			if (g_status_work != true)
			{
				if (g_status_adv == true)
				{
					g_event_status |= EVENT_ADV_STOP;
                    if(battery_get_charege_status() == BATTERY_NOT_CHARGE)
					    g_event_status |= EVENT_BEGIN_WORK;
                    app_trace_log("adv stop %d\n",__LINE__);
				}
				else if ((g_status_ble_connect == true)
					&&(g_status_data_send != true))
				{
					g_event_status |= EVENT_BLE_SHUT_CONNECT;
					g_event_status |= EVENT_BEGIN_WORK;
                    app_trace_log("connect true %d\n",__LINE__);
				}
                else if (battery_get_charege_status() == BATTERY_NOT_CHARGE)
                {
                    g_event_status |= EVENT_BEGIN_WORK;
                }
			}
			else
			{
				g_event_status |= EVENT_END_WORK;
			}
			g_event_status &= ~EVENT_KEY_PRESS_SHOT;

		}
		else if(g_event_status & EVENT_KEY_PRESS_LONG)
		{
			if ((g_status_work != true) &&
					(g_status_ble_connect != true) &&
					(g_status_adv    != true))
			{
				g_event_status |= EVENT_ADV_START;
                app_trace_log("adv start \n");
			}
			g_event_status &= ~(EVENT_KEY_PRESS_LONG);
		}
        if (g_event_status & EVENT_BEGIN_WORK)
        {//开始工作
            if (system_params.device_bonded == true)
            {
            	leds_process_init(LED_WORK_BEGIN);
    			// begin to work
    			g_status_work = true;
    			//初始化角度值
    			g_status_tilt_init_flag = true;
                app_trace_log("work start \n");
            }
            g_event_status &= ~(EVENT_BEGIN_WORK);
        }

        if (g_event_status & EVENT_END_WORK)
        {//停止工作
            if (leds_get_cur_status() == LED_WORK_BEGIN)
            {
                leds_process_init(LED_IDLE);
            }
            leds_process_init(LED_WORK_END);
			// stop work
			g_status_work = false;
			//初始化角度值
			g_status_tilt_init_flag = false;
            if (g_status_alarm_status)
            {
                g_status_alarm_status = false;
				alarm_case();
            }
			//数据发送完成同步队列信息
			g_event_status |= EVENT_DATA_SYNC;
            g_event_status &= ~(EVENT_END_WORK);
            app_trace_log("work stop \n");
        }

        if (g_event_status & EVENT_DATA_SENDING)
        {//开始发送数据
    		g_status_data_send = true;
            leds_process_init(LED_WORK_BLE_DATA_TRAING);
            g_event_status &= ~(EVENT_DATA_SENDING);
        }

		if (g_event_status & EVENT_MESSAGE_RECEIVED)
		{//有数据接收
		    if (queue_message_pop(rec_data_buffer))
		    {
			    g_event_status &= ~(EVENT_MESSAGE_RECEIVED);
            }
			else
			{
				message_process(rec_data_buffer);
    		    if (rec_data_buffer[2] == 0x02)
    		    {
                    app_trace_log("begin send data\n");
                }
			}
		}

		if (g_event_status & EVENT_ADV_START)
		{//开始广播
			if ((g_status_work != true) && (g_status_adv!= true))
			{
				advertising_start();
				g_status_adv = true;
			}
			g_event_status &= ~(EVENT_ADV_START);
		}

		if (g_event_status & EVENT_ADV_STOP)
		{//停止广播
			err_code = sd_ble_gap_adv_stop();
			APP_ERROR_CHECK(err_code);
			g_status_adv = false;
			g_event_status &= ~(EVENT_ADV_STOP);
		}

		if (g_event_status & EVENT_BLE_SHUT_CONNECT)
		{//关闭蓝牙连接
			#if !defined(DEBUG_APP)
            
			err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
			APP_ERROR_CHECK(err_code);
			#endif
			app_trace_log("%s %d sd_ble_gap_disconnect\r\n",__FUNCTION__,__LINE__);
			g_event_status &= ~EVENT_BLE_SHUT_CONNECT;
		}

		if (g_event_status & EVENT_BLE_CONNECTED)
		{//蓝牙连接
			g_status_adv = false;
			g_status_ble_connect = true;
			leds_process_init(LED_WORK_BLE_CONNECTED);
			g_event_status &= ~EVENT_BLE_CONNECTED;
            app_trace_log("connected \n");
		}
		if (g_event_status & EVENT_BLE_DISCONNECTED)
		{//蓝牙连接断开
			g_status_ble_connect = false;
			g_event_status &= ~EVENT_BLE_DISCONNECTED;
            app_trace_log("disconnected \n");
		}
		if (g_event_status & EVENT_DATA_SYNC)
		{//数据同步
			queue_sync();
			g_event_status &= ~EVENT_DATA_SYNC;
		}

        if (g_event_status & EVENT_BATTRY_VALUE)
        {//更新电池电量
            battery_get_value();
            g_event_status &= ~EVENT_BATTRY_VALUE;
        }

		if (g_event_status & EVENT_DATA_FULL)
		{
			leds_process_init(LED_WORK_DATA_FULL);
			g_event_status &= ~EVENT_DATA_FULL;
		}
		//数据发送
		if (g_status_data_send == true)
		{//开始发送数据
			if (g_status_ble_connect == true)
			{
				if (queue_pop(&item))
				{
					//发送完成
					g_status_data_send = false;
                    leds_process_init(LED_IDLE);
					g_event_status |= EVENT_DATA_SENDED;
					//发送传输完成报文
					data_array[0] = 0xA5;
					data_array[1] = 0x01;
					data_array[2] = CMD_SEND_DATA_COMPLETED;
					data_array[3] = 0x80;
					err_code = ble_nus_send_string(&m_nus, data_array, 4);
			        if (err_code != NRF_ERROR_INVALID_STATE)
			        {
			            APP_ERROR_CHECK(err_code);
			        }
					//数据发送完成同步队列信息
					g_event_status |= EVENT_DATA_SYNC;
                    app_trace_log("data send end\n");
				}
				else
				{
					data_array[0] = 0xA5;
					data_array[1] = 0x09;
					data_array[2] = CMD_SEND_DATA;
					data_array[3] = item.year;
					data_array[4] = item.mon;
					data_array[5] = item.day;
					data_array[6] = item.hour;
					data_array[7] = item.min;
					data_array[8] = item.second;
					data_array[9] = (uint8_t)(item.angle & 0x00ff);
					data_array[10] = (uint8_t)((item.angle >> 8) & 0x00ff);
					data_array[11] = 0x80;
					err_code = ble_nus_send_string(&m_nus, data_array, 12);
			        while (err_code != NRF_SUCCESS)
			        {
			            err_code = ble_nus_send_string(&m_nus, data_array, 12);
			        }
                    app_trace_log("data sending\n");
				}
			}
			else
			{
				g_status_data_send = false;
			}
		}
        power_manage();
//        nrf_gpio_pin_toggle(LED_RED);
    }
}

/**
 * @}
 */
