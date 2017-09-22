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
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "simple_uart.h"
#include "boards.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "app_util_platform.h"
#include "spi_master.h"
#include "lis3dh_driver.h"
#include "calender.h"
#include "queue.h"
#include "pstorage.h"
#include "app_gpiote.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/


#define WAKEUP_BUTTON_PIN               BUTTON_0                                    /**< Button used to wake up the application. */

#define ADVERTISING_LED_PIN_NO          LED_0                                       /**< LED to indicate advertising state. */
#define CONNECTED_LED_PIN_NO            LED_1                                       /**< LED to indicate connected state. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               16                                          /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               60                                          /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< slave latency. */
#define CONN_SUP_TIMEOUT                400                                         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define START_STRING                    "Start...\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define LIS3DH_SMAPLE_RATE				1											/**< 三轴加速度采样频率 单位:秒 >**/

#define ANGLE_SMAPLE_RATE				300											/**< 角度数据采样频率 单位:秒 >**/
//gpiote
#define MAX_USERS						1

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */

//gpiote user identifier
static app_gpiote_user_id_t gpiote_user_id;
static void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low);

/***********************************事件定义***************************************/
#define EVENT_KEY_PRESSED               (uint32_t)(0x00000001 << 0)                  /**< 按键事件 >**/
#define EVENT_MESSAGE_RECEIVED          (uint32_t)(0x00000001 << 1)					 /**< 通信事件 >**/
#define EVENT_DATA_SENDING				(uint32_t)(0x00000001 << 2)					 /**< 数据发送事件 >**/
#define EVENT_DATA_SENDED               (uint32_t)(0x00000001 << 3)					 /**< 数据发送完成事件 >**/
static uint32_t event_status = 0;      //时间存储变量
static bool key_pressed = false;         //记录按键事件
static bool adv_status = false;          //广播状态
static bool work_status = false;         //设备工作状态
static bool message_received = false;  //信息接收
static bool data_send_status = false;  //发送数据
static uint8_t tilt_init_flag = false; //角度值初始化
static float Tilt;                       //当前倾角变化值
static uint8_t rec_data_buffer[20];      //缓存接收到的数据
static bool ble_connect_status = false;  //蓝牙连接状态

//干涉条件
static uint8_t alarm_angle;//干涉角度
static uint8_t alarm_timer;//干涉时间
typedef enum
{
	CMD_SET_TIME = 0x01,
	CMD_REQUEST_DATA,
	CMD_SEND_DATA,
	CMD_SEND_DATA_COMPLETED,
	CMD_SET_ALARM,
}MESSAGE_CMD_ID_t;
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


/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
//	nrf_gpio_cfg_output(SPIM1_SS_PIN);
}


/**@brief   Function for Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
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
}


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
	//广播超时模式
//    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
	//无限广播模式
	uint8_t			flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

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


/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	memcpy(rec_data_buffer,p_data,length);
	message_received = true;
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
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
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
    adv_params.timeout     = 0;//广播超时 三分钟APP_ADV_TIMEOUT_IN_SECONDS

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
			adv_status = false;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

//            advertising_start();

            break;

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

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

                // Configure buttons with sense level low as wakeup source.
                nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                                         BUTTON_PULL,
                                         NRF_GPIO_PIN_SENSE_LOW);

                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
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
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
	pstorage_sys_event_handler(sys_evt);
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
		if (data_send_status == false) {
			//数据传输功能时按键无效
			//button 1 pressed
			key_pressed = 1;
		}
	}
}


/**@brief  Function for configuring the buttons.
 */
static void buttons_init(void)
{
	uint32_t low_to_high_bitmask = (uint32_t)(1 << BUTTON_1);
	uint32_t high_to_low_bitmask = (uint32_t)(1 << BUTTON_1);
	uint32_t err_code;

//    nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
//                             BUTTON_PULL,
//                             NRF_GPIO_PIN_SENSE_LOW);
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
}


/**@brief  Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief  Function for initializing the UART module.
 */
static void uart_init(void)
{
    /**@snippet [UART Initialization] */
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);

    NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos;

    NVIC_SetPriority(UART0_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(UART0_IRQn);
    /**@snippet [UART Initialization] */
}


/**@brief   Function for handling UART interrupts.
 *
 * @details This function will receive a single character from the UART and append it to a string.
 *          The string will be be sent over BLE when the last character received was a 'new line'
 *          i.e '\n' (hex 0x0D) or if the string has reached a length of @ref NUS_MAX_DATA_LENGTH.
 */
void UART0_IRQHandler(void)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t err_code;

    /**@snippet [Handling the data received over UART] */

    data_array[index] = simple_uart_get();
    index++;

    if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN - 1)))
    {
        err_code = ble_nus_send_string(&m_nus, data_array, index + 1);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }

        index = 0;
    }

    /**@snippet [Handling the data received over UART] */
}
/******************SPI Driver**********************/
static bool transmission_completed = 0; //spi传输状态
static uint8_t rx_buffer[2];
static uint8_t tx_buffer[2];
static void SPI_Master_Event_Handler(spi_master_evt_t spi_master_evt)
{
	switch (spi_master_evt.evt_type)
	{
		case SPI_MASTER_EVT_TRANSFER_COMPLETED:
			transmission_completed = true;//传输完成
			break;
		default :
			break;
	}
}

static void SPI_Init(void)
{
	spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;

	spi_config.SPI_Freq = SPI_FREQUENCY_FREQUENCY_K125;
	spi_config.SPI_Pin_SCK = SPIM1_SCK_PIN;
	spi_config.SPI_Pin_MISO = SPIM1_MISO_PIN;
	spi_config.SPI_Pin_MOSI = SPIM1_MOSI_PIN;
	spi_config.SPI_Pin_SS = SPIM1_SS_PIN;
	spi_config.SPI_CONFIG_CPOL = SPI_CONFIG_CPOL_ActiveLow;
	spi_config.SPI_CONFIG_CPHA = SPI_CONFIG_CPHA_Trailing;
	spi_config.SPI_CONFIG_ORDER = SPI_CONFIG_ORDER_MsbFirst;

	uint32_t err_code = spi_master_open(SPI_MASTER_1, &spi_config);
	if (err_code != NRF_SUCCESS)
	{

	}

	spi_master_evt_handler_reg(SPI_MASTER_1,SPI_Master_Event_Handler);

//	nrf_gpio_pin_clear(SPIM0_SS_PIN);
}

uint8_t SPI_Mems_Read_Reg(uint8_t reg)
{
	uint32_t err_code = 0;
	tx_buffer[0] = reg;
	tx_buffer[1] = 0;
	err_code = spi_master_send_recv(SPI_MASTER_1,tx_buffer,2,rx_buffer,2);
	if (err_code != NRF_SUCCESS)
	{

	}
	while( transmission_completed==0);
	transmission_completed = 0;
	return rx_buffer[1];
}

void SPI_Mems_Write_Reg(uint8_t WriteAddr, uint8_t Data)
{
	uint32_t err_code = 0;
	tx_buffer[0] = WriteAddr;
	tx_buffer[1] = Data;
	err_code = spi_master_send_recv(SPI_MASTER_1,tx_buffer,2,rx_buffer,2);
	if (err_code != NRF_SUCCESS)
	{

	}
	while( transmission_completed==0);
	transmission_completed = 0;
}
//3轴传感器初始化
static void LIS3DH_Init(void)
{
	//设置采样率
	LIS3DH_SetODR(LIS3DH_ODR_100Hz);

	//设置工作模式
	LIS3DH_SetMode(LIS3DH_NORMAL);

	//设置扫描范围 正负2g
	LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);

	//使能3轴
	LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE);
}
//****************周期事件处理函数*********************
static app_timer_id_t p_timer;
static bool lis3dh_flag = 0;
//周期事件处理函数
static void period_cycle_process(void * p_context)
{
	static uint32_t lis3dh_timer = 0;//三轴传感器采样频率
	static uint8_t key_timer = 0;	//按键计时器
	uint8_t key_status;             //按键状态
	static uint16_t angle_timer = 0;	//角度采样频率
	static uint16_t data_send_completed = 0; //用于传输数据结束后关闭蓝牙连接
	static uint16_t alarm_timer_cont = 0; //当角度大于干涉角度时开始计时，超时后报警
	queue_items_t item;
	UTCTimeStruct time;
	uint32_t err_code;
	//模拟日历
	TimeSeconds ++;


	//按键处理
	if (key_pressed == 1)
	{
		key_status = nrf_gpio_pin_read(BUTTON_1);
		if (key_status == 0)
		{
			key_timer ++;
		}
		else
		{
			key_pressed = 0;
			if(key_timer < 2)
			{
				//短按
				if (adv_status == true)
				{
					sd_ble_gap_adv_stop();
					adv_status = false;
				}
				if (work_status == false)
				{
					// begin to work
					work_status = true;
					//初始化角度值
					tilt_init_flag = true;
				}
				else
				{
					// stop work
					work_status = false;
				}
			}
			else
			{
				//长按
				if (adv_status == false)
				{
					adv_status = true;
					advertising_start();
				}
			}
			key_timer = 0;
		}
	}


	if ((work_status == true)&&(lis3dh_timer++ >= LIS3DH_SMAPLE_RATE))
	{//使用三轴加速度采样
		lis3dh_timer = 0;
		lis3dh_flag = 1;
	}

	if ((event_status&EVENT_DATA_SENDED) && (data_send_completed++ > 30))
	{
		//数据传输完成后30s断开蓝牙连接
		event_status&= ~EVENT_DATA_SENDED;
		data_send_completed = 0;
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
	}
	if ((work_status == true)&&(angle_timer++ >= ANGLE_SMAPLE_RATE))
	{
		if((Tilt > alarm_angle)&&(alarm_timer_cont++ > alarm_timer))
		{//开始报警

		}
		else
		{//取消报警

		}
		//存储角度值
		ConvertUTCTime(&time,TimeSeconds);
		item.year = time.year - 2000;
		item.mon = time.month;
		item.day = time.day;
		item.hour = time.hour;
		item.min = time.minutes;
		item.second = time.seconds;
		item.angle = (uint16_t)(Tilt*100);
//		queue_push(&item);
	}
}
//****************周期事件初始化*********************
static void period_cycle_process_init(void)
{
	app_timer_create(&p_timer,APP_TIMER_MODE_REPEATED,period_cycle_process);

	app_timer_start(p_timer,APP_TIMER_TICKS(1000,APP_TIMER_PRESCALER),NULL);
}

//*****************倾角计算**************************
#define PI 3.1415926
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
static float calculateTilt_run(float ax, float ay, float az)
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

//static float calculateTilt_B(float ax, float ay, float az)
//{
//	/*
//	*关于计算前后转动角度超值问题，记录前一时刻x,y,加速度正负，由正变负+90°
//	*/
//	float temp;
//	float Tiltangle = 0;
//	temp = sqrt(ax*ax + ay*ay) / az;
//	Tiltangle = atan(temp);
//	Tiltangle = Tiltangle/PI*180;
//	return Tiltangle;
//}

//报文处理函数
static void message_process(uint8_t *ch)
{
	uint8_t cmd_id;
	UTCTimeStruct tm;
	uint32_t err_code;
	uint8_t data_array[20];
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
		data_send_status = true;
		break;
	case CMD_SET_ALARM:
		//设置干涉条件
		alarm_angle = ch[3];
		alarm_timer = ch[4];
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
		break;
	default:
		break;
	}
}

//队列测试代码
uint8_t test[100];
extern pstorage_handle_t block_id;
void queue_test(void)
{
	uint8_t i = 0;
	uint8_t printf_buffer[26];
	queue_items_t item;
	pstorage_handle_t dest_block_id;
	memset(&item, 0x00, sizeof(queue_items_t));

	sprintf(printf_buffer,"%d  %d  %d\r\n",queue_entries.entries,queue_entries.rx_point,\
					queue_entries.tx_point);
		simple_uart_putstring(printf_buffer);

	for(i = 0; i< 10; i++)
	{
		queue_push(&item);
		item.year ++;
	}
	memset(&item, 0x00, sizeof(queue_items_t));
	for(i = 0; i < 10; i++)
	{
		if(queue_pop(&item))
		{
			sprintf(printf_buffer,"end\r\n");
			simple_uart_putstring(printf_buffer);
			break;
		}
		else 
		{
			sprintf(printf_buffer,"%d\r\n",item.year);
			simple_uart_putstring(printf_buffer);
		}
	}

	queue_init();
	sprintf(printf_buffer,"%d  %d  %d\r\n",queue_entries.entries,queue_entries.rx_point,\
				queue_entries.tx_point);
	simple_uart_putstring(printf_buffer);
	pstorage_block_identifier_get(&block_id, 0, &dest_block_id);
	pstorage_load(test, &dest_block_id, sizeof(test),0);

}
/**@brief  Application main function.
 */
int main(void)
{
    // Initialize
//    uint8_t data;
	AxesRaw_t Axes_Raw_Data = {0};
	uint8_t printf_buffer[26];
	uint8_t response;
	float ax,ay,az;
	queue_items_t item;
	uint8_t data_array[20];
	uint32_t err_code;
    leds_init();
    timers_init();
    uart_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();
    simple_uart_putstring(START_STRING);
	SPI_Init();
	//周期处理线程
	period_cycle_process_init();
	//flash初始化
	queue_init();
	//gpiote初始化
    buttons_init();
//    advertising_start();

//	LIS3DH_GetWHO_AM_I(&data);
//	simple_uart_put(data);

	LIS3DH_Init();
	queue_test();

    // Enter main loop
    for (;;)
    {
		if (lis3dh_flag == 1) {
			lis3dh_flag = 0;
			response = LIS3DH_GetAccAxesRaw(&Axes_Raw_Data);
			if (response == 1) {
				ax = Axes_Raw_Data.AXIS_X/16384.0;
				ay = Axes_Raw_Data.AXIS_Y/16384.0;
				az = Axes_Raw_Data.AXIS_Z/16384.0;
				sprintf((char *)printf_buffer, "X=%6f Y=%6f Z=%6f \r\n",
					ax,ay,az);
				simple_uart_putstring(printf_buffer);
				Tilt = calculateTilt_run(ax,ay,az);
				sprintf((char *)printf_buffer, "Tilt = %6f \r\n", Tilt);
				simple_uart_putstring(printf_buffer);
			}
		}
		if (message_received == true)
		{//有数据接收
			message_received = false;
			message_process(rec_data_buffer);
		}

		if (data_send_status == true)
		{//开始发送数据
			if (queue_pop(&item))
			{
				//发送完成
				data_send_status = false;
				//led常亮
				nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
				event_status |= EVENT_DATA_SENDED;
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
			}
			else
			{
				//传输数据是led闪烁
				nrf_gpio_pin_toggle(CONNECTED_LED_PIN_NO);
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
				err_code = ble_nus_send_string(&m_nus, data_array, 4);
		        while (err_code != NRF_ERROR_INVALID_STATE)
		        {
		            err_code = ble_nus_send_string(&m_nus, data_array, 4);
		        }
			}
		}
        power_manage();
    }
}

/**
 * @}
 */
