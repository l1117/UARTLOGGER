/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

//#include <stdint.h>
//#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_nus.h"
//#include "app_uart.h"
#include "simple_uart.h"
#include "app_util_platform.h"
//#include "bsp.h"
#include "pstorage.h"
//#include "app_scheduler.h"
#include "ble_bas.h"
//#include "nrf_delay.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define WAKEUP_BUTTON_ID                0                                           /**< Button used to wake up the application. */

#define APP_ADV_INTERVAL                1600*1  //0x4000                                        /**< 10.24 secs  The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
static uint16_t adv_interval =  1600;
#define APP_ADV_TIMEOUT_IN_SECONDS      0    //180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                      /**< Value of the RTC1 PRESCALER register. */
//#define APP_TIMER_MAX_TIMERS            (2 + BSP_APP_TIMERS_NUMBER)                 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_MAX_TIMERS            (3)                 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of simultaneously GPIOTE users. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(24, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   4   //4                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(400, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define START_STRING                    "Start...\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
//static ble_gatts_char_handles_t m_char_handles;                                     /**< Handles of local characteristic (as provided by the BLE stack).*/
static uint16_t                  m_service_handle;                                   /**< Handle of local service (as provided by the BLE stack).*/
static bool                      m_is_notifying_enabled = false;                     /**< Variable to indicate whether the notification is enabled by the peer.*/
//static app_timer_id_t            m_notif_timer_id;                                   /**< Notification timer. */
static ble_bas_t                        m_bas;                                     /**< Structure used to identify the battery service. */



#define UART_POWER  	 8  //3       //5TM Power
//#define DEVICE_NAME                     "CSTDataLog"                               /**< Name of device. Will be included in the advertising data. */
uint8_t device_name[15]  = "5T:" ;
uint32_t baudrate_select = 1200;
//#define ADC_AIN						  ADC_CONFIG_PSEL_AnalogInput5  //pin04   BEACON
#define ADC_AIN						  ADC_CONFIG_PSEL_AnalogInput4  //pin03  
	
#define LED_POWER  				      20
#define COUNTER_STEP            18  
//#define DATA_PERIODIC         300
uint32_t data_periodic = 				256; //16;  //180;    //60;
static uint32_t timer_counter = 0;
static uint8_t data_array[32];  //[BLE_NUS_MAX_DATA_LEN+4];   //+4 for timecounter
#define PSTORAGE_PAGE_SIZE		1024
#define DATA_LOG_LEN					16
static pstorage_block_t pstorage_wait_handle = 0;
static pstorage_handle_t       flash_base_handle;
static uint32_t 	pstorage_block_id, pstorage_next_adv;
static uint16_t connect_time_out = 0 ;
static pstorage_handle_t 		flash_handle ;
static app_timer_id_t  					weakup_meantimer_id	;
static uint16_t    		 notify_id = (PSTORAGE_MAX_APPLICATIONS*PSTORAGE_PAGE_SIZE/DATA_LOG_LEN - 1);
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS        1200                                      /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION         3                                         /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS       270                                       /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)
uint16_t battery_start(uint32_t AnalogInput)
{
    NRF_ADC->INTENSET   = AAR_INTENSET_END_Disabled;   				//ADC  Interrupt disabled. Must be disable.
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                          (AnalogInput                   << ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;
		NRF_ADC->TASKS_START = 1;							//Start ADC sampling
    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
				uint8_t adc_delay=0;
        while (!NRF_ADC->EVENTS_END && adc_delay<0xff)  adc_delay++;
    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.

        NRF_ADC->TASKS_STOP     = 1;
				NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Disabled;
        return  ADC_RESULT_IN_MILLI_VOLTS(NRF_ADC->RESULT); // + DIODE_FWD_VOLT_DROP_MILLIVOLTS;
//		sd_clock_hfclk_release();
}


/**@brief Function for assert macro callback.
 *
 * @details     This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning     On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NVIC_SystemReset();
}
static void char_notify(void)
{
    uint32_t err_code;
    static uint16_t len = 16;
		err_code = NRF_SUCCESS;
		static uint8_t notify_data[16];
    // Send value if connected and notifying.
    if ((m_conn_handle != BLE_CONN_HANDLE_INVALID) && m_is_notifying_enabled)
    {
//				ble_gatts_value_t gatts_value;
//				memset(&gatts_value, 0, sizeof(gatts_value));
//				gatts_value.len     = sizeof(uint32_t);
//				gatts_value.offset  = 0;
//				gatts_value.p_value = (uint8_t *)&notify_id;
				ble_gatts_hvx_params_t 		hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle   = m_bas.notify_handles.value_handle;  //   m_char_handles.value_handle;
        hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset   = 0;
        hvx_params.p_len    = &len;
//        hvx_params.p_data   = (uint8_t *) flash_load;  //m_char_value;
//				pstorage_handle_t flash_handle;

				while(true){
					err_code = pstorage_block_identifier_get(&flash_base_handle,(notify_id
							% (PSTORAGE_MAX_APPLICATIONS*PSTORAGE_PAGE_SIZE/DATA_LOG_LEN)), &flash_handle);
					APP_ERROR_CHECK(err_code);
					err_code = pstorage_load(notify_data, &flash_handle, DATA_LOG_LEN, 0);
					APP_ERROR_CHECK(err_code);
					hvx_params.p_data   = ((uint8_t *) notify_data);  //m_char_value;
					err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
					if ((err_code == BLE_ERROR_NO_TX_BUFFERS) ||
							(err_code == NRF_ERROR_INVALID_STATE) ||
							(err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING))	break;
					else if ((err_code != NRF_SUCCESS))
							{
									APP_ERROR_HANDLER(err_code);
							}
//		 			sd_ble_gatts_value_set(m_bas.conn_handle, m_bas.Vtm_date_time_handles.value_handle, &gatts_value);
				  if (notify_id-- == 0){
								m_is_notifying_enabled =  false;
						    break;
					 }
					}

		}
}
static void on_write(ble_evt_t * p_ble_evt)
{
//		uint32_t err_code;
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    if ((p_evt_write->handle == m_bas.Vtm_date_time_handles.value_handle) && (p_evt_write->len == 2))
				notify_id = *(uint16_t *)p_evt_write->data;
		else if ((p_evt_write->handle == m_bas.Nrf_name_handles.value_handle) && (p_evt_write->len == 4))
			{
				device_name[0] = *p_evt_write->data;
				device_name[1] = *(p_evt_write->data+1);
				device_name[2] = *(p_evt_write->data+2);
				device_name[3] = *(p_evt_write->data+3);
			}
    else if ((p_evt_write->handle == m_bas.notify_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written. Start notifications
        m_is_notifying_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
        if (m_is_notifying_enabled) char_notify();
    }
}

 char char_hex( char bHex){
		bHex&=0x0f;
    if((bHex<10))
        bHex += '0';
    else bHex += 'A'-10;
//    else bHex = 0xff;
    return bHex;
}


/**@brief   Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    		ble_gap_addr_t  	p_addr;
    sd_ble_gap_address_get	(	&p_addr	);
//		sprintf(device_name, "CSTNET_%4s:%x%x%x%x%x%x",sadvname,p_addr.addr[5],p_addr.addr[4],p_addr.addr[3],p_addr.addr[2],p_addr.addr[1],p_addr.addr[0]);
//    for (i=7;sadvname[i-7];i++) device_name[i]=sadvname[i-7];
		uint8_t i = 3;
		device_name[i++] = char_hex(p_addr.addr[5]>>4);		device_name[i++] = char_hex(p_addr.addr[5]);
		device_name[i++] = char_hex(p_addr.addr[4]>>4);		device_name[i++] = char_hex(p_addr.addr[4]);
		device_name[i++] = char_hex(p_addr.addr[3]>>4);		device_name[i++] = char_hex(p_addr.addr[3]);
		device_name[i++] = char_hex(p_addr.addr[2]>>4);		device_name[i++] = char_hex(p_addr.addr[2]);
		device_name[i++] = char_hex(p_addr.addr[1]>>4);		device_name[i++] = char_hex(p_addr.addr[1]);
		device_name[i++] = char_hex(p_addr.addr[0]>>4);		device_name[i++] = char_hex(p_addr.addr[0]);
//		err_code = sd_ble_gap_device_name_set(&sec_mode,
//                                          (const uint8_t *)device_name ,
//                                          15);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          device_name,
                                          15);
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
 *          Also builds a structure to be passed to the stack when starting the advertising.
 */
static void advertising_init(void)
{
    static uint32_t      err_code;
    static ble_advdata_t advdata;
    static ble_advdata_t scanrsp;
    static uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    static ble_advdata_manuf_data_t  manuf_data;

//    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_SHORT_NAME;
		advdata.short_name_len          = 8;
    advdata.include_appearance      = false;
    advdata.flags                   = flags;
//    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
//    advdata.uuids_complete.p_uuids  = adv_uuids;
    memset(&scanrsp, 0, sizeof(scanrsp));
		manuf_data.company_identifier   = 0x5881;
		manuf_data.data.size						= 24;  //BLE_NUS_MAX_DATA_LEN+6;
		manuf_data.data.p_data    			= data_array;
		scanrsp.p_manuf_specific_data		= &manuf_data;
    data_array[20] = device_name[0];
    data_array[21] = device_name[1];
    data_array[22] = device_name[2];
    data_array[23] = device_name[3];
		*(uint32_t *)(data_array) = timer_counter;      

    err_code = ble_advdata_set(&advdata, &scanrsp);
//    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(uint16_t adv_interval, uint16_t adv_timeout)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    // Start advertising.
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = adv_interval;
    adv_params.timeout     = adv_timeout;
    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    advertising_init();

//    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//    APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */

//void uart_event_handle(app_uart_evt_t * p_event)

static void get_data(void)
{
    static uint8_t uart_data[16];  //why must be static?
		static uint8_t log_data[16];
    uint8_t uart_index = 0, cr;
    uint32_t err_code;
		NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
		simple_uart_config(NULL, 4, NULL, 16, false);
		nrf_gpio_pin_set (UART_POWER);												//Open power and UART for 5TM 
		while (simple_uart_get_with_timeout(4, &cr));					//clear RX buffer, must be!
		memset(uart_data,0,16);
		memset(log_data,0,16);
		for (uint8_t i=0; i<50; i++){										//Get date from 5tm 
				if (simple_uart_get_with_timeout(4, &cr))  {
						uart_data[uart_index++] = cr;
						if ((uart_data[uart_index - 1] == '\n') || ((uart_index) >= 16))
								{
										sscanf((const char *)(uart_data), "%hx%hx%hx\n",(uint16_t *)(log_data+4),(uint16_t *)(log_data+6),(uint16_t *)(log_data+8));
										break;
										}
						}
				}
		NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);
		nrf_gpio_pin_clear (UART_POWER);
		int32_t nrf_temp;
		sd_temp_get(&nrf_temp);  													//Get Cpu tempreature
		*(uint32_t *)(log_data) = timer_counter;      
		*(uint16_t *)(log_data+10) = battery_start(ADC_AIN);
		*(uint32_t *)(log_data+12) = nrf_temp;
		err_code = pstorage_store(&flash_handle, log_data, DATA_LOG_LEN, 0 );
		APP_ERROR_CHECK(err_code);
//		if (*(uint16_t *)(data_array+18) == 0xFFFF) {
//				pstorage_next_adv++;
//				for (uint8_t i=0; i<16; i++) data_array[i+4] = log_data[i];
//				*(uint16_t *)(data_array+18) = 0x0000;
//		}
//		else *(uint16_t *)(data_array+18) = (uint16_t)(pstorage_block_id-pstorage_next_adv);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void service_add(void)
{
    uint32_t       			err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_DEVICE_INFORMATION_SERVICE);
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_service_handle);
    APP_ERROR_CHECK(err_code);
    // cccd_md init characteristic
        memset(&cccd_md, 0, sizeof(cccd_md));
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
				memset(&char_md, 0, sizeof(char_md));
				char_md.p_char_user_desc  = NULL;
				char_md.p_char_pf         = NULL;
				char_md.p_user_desc_md    = NULL;
				char_md.p_cccd_md         = (char_md.char_props.notify) ? &cccd_md : NULL;
				char_md.p_sccd_md         = (char_md.char_props.broadcast) ? &cccd_md : NULL;

				memset(&attr_md, 0, sizeof(attr_md));
				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
				attr_md.vloc       = BLE_GATTS_VLOC_STACK;
				attr_md.rd_auth    = 0;
				attr_md.wr_auth    = 0;
				attr_md.vlen       = 0;
// char notify
				memset(&attr_char_value, 0, sizeof(attr_char_value));
				char_md.char_props.read   = 0;
		    char_md.char_props.write  = 0;
				char_md.char_props.notify = 1;
				attr_char_value.p_attr_md = &attr_md;
				attr_char_value.init_len  = 16;   //sizeof(uint16_t);
				attr_char_value.init_offs = 0;
				attr_char_value.max_len   = 16; //sizeof(uint32_t);
				attr_char_value.p_value   = NULL; 
				BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ALERT_NOTIFICATION_CONTROL_POINT_CHAR);
				attr_char_value.p_uuid    = &ble_uuid;
				err_code = sd_ble_gatts_characteristic_add(m_bas.service_handle, &char_md,
																									 &attr_char_value,
																									 &m_bas.notify_handles);
				
// char data_time
				memset(&attr_char_value, 0, sizeof(attr_char_value));
				char_md.char_props.read   = 1;
		    char_md.char_props.write  = 1;
				char_md.char_props.notify = 0;
				attr_char_value.p_attr_md = &attr_md;
				attr_char_value.init_len  = 2;   //sizeof(uint16_t);
				attr_char_value.init_offs = 0;
				attr_char_value.max_len   = 2; //sizeof(uint32_t);
				attr_char_value.p_value   = NULL; 
				BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ALERT_CATEGORY_ID_CHAR);
				attr_char_value.p_uuid    = &ble_uuid;
				err_code = sd_ble_gatts_characteristic_add(m_bas.service_handle, &char_md,
																									 &attr_char_value,
																									 &m_bas.Vtm_date_time_handles);
// char name
				attr_char_value.init_len  = sizeof(uint32_t);
				char_md.p_cccd_md         = (char_md.char_props.notify) ? &cccd_md : NULL;
				char_md.p_sccd_md         = (char_md.char_props.broadcast) ? &cccd_md : NULL;
				attr_char_value.max_len   = sizeof(uint32_t);
				BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_MANUFACTURER_NAME_STRING_CHAR);
				attr_char_value.p_uuid    = &ble_uuid;
				err_code = sd_ble_gatts_characteristic_add(m_bas.service_handle, &char_md,
																									 &attr_char_value,
																									 &m_bas.Nrf_name_handles);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
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

static void weakup_meantimeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		sd_ble_gap_adv_stop();
	  adv_interval = 1600;
	  advertising_start(adv_interval , 10);
//	  APP_ERROR_CHECK(err_code);
//		adv_sleep_secs = 1800; //10;
}



/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_sec_keyset_t      s_sec_keyset;
    ble_gap_enc_info_t *             p_enc_info;
    switch (p_ble_evt->header.evt_id)
    {
				case BLE_GAP_EVT_SCAN_REQ_REPORT:
						if (p_ble_evt->evt.gap_evt.params.scan_req_report.peer_addr.addr[0] == 0x58 && 
									p_ble_evt->evt.gap_evt.params.scan_req_report.peer_addr.addr[1] == 0x81){
								 if (pstorage_next_adv < pstorage_block_id){
												err_code = pstorage_block_identifier_get(&flash_base_handle,(pstorage_next_adv
														% (PSTORAGE_MAX_APPLICATIONS*PSTORAGE_PAGE_SIZE/DATA_LOG_LEN)), &flash_handle);
												APP_ERROR_CHECK(err_code);
												pstorage_load(data_array+4, &flash_handle, DATA_LOG_LEN, 0);
												*(uint16_t *)(data_array+18) = (uint16_t)(pstorage_block_id - pstorage_next_adv);
												advertising_init();
												pstorage_next_adv ++;
											  if (!(pstorage_next_adv < pstorage_block_id)) {
															err_code = sd_ble_gap_adv_stop();
															app_timer_stop(weakup_meantimer_id);
															adv_interval = 0;
												}
												else { 
															app_timer_stop(weakup_meantimer_id);
															app_timer_start(weakup_meantimer_id,  APP_TIMER_TICKS(1100, 0), NULL);
															if ((adv_interval > 200) && ((pstorage_block_id - pstorage_next_adv) > 16)){
																		sd_ble_gap_adv_stop();
																		adv_interval = 150;
																		advertising_start(adv_interval, 10);
			//															advertising_start(160, 0);
			//															break;  //for not to be lost records;
															}
												}
									}	else 	{
											err_code = sd_ble_gap_adv_stop();
											app_timer_stop(weakup_meantimer_id);
											adv_interval = 0;
											}
								}
						break;
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            connect_time_out = 10;
//						nrf_gpio_pin_set (LED_PIN);												//Led on.
//						application_timers_start();
						ble_gap_conn_params_t   gap_conn_params;
						gap_conn_params.min_conn_interval = ((4 * 10) / 5);  //= 10
						gap_conn_params.max_conn_interval = (4 * 24) / 5;
						gap_conn_params.slave_latency     = 4;		//SLAVE_LATENCY;
						gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(400, UNIT_10_MS) ;	//CONN_SUP_TIMEOUT;
						err_code=sd_ble_gap_conn_param_update(m_conn_handle,&gap_conn_params);
            APP_ERROR_CHECK(err_code);
						ble_gatts_value_t gatts_value;
						memset(&gatts_value, 0, sizeof(gatts_value));
						gatts_value.len     = sizeof(uint32_t);
						gatts_value.offset  = 0;
// Updata server data for display  
						gatts_value.p_value = (uint8_t *)&notify_id;
						err_code = sd_ble_gatts_value_set(m_bas.conn_handle, m_bas.Vtm_date_time_handles.value_handle, &gatts_value);
						gatts_value.p_value = (uint8_t *)device_name;
						err_code = sd_ble_gatts_value_set(m_bas.conn_handle,m_bas.Nrf_name_handles.value_handle, &gatts_value);
            break;
						
        case BLE_EVT_TX_COMPLETE:
            char_notify();
						break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ble_evt);
            break;
			
        case BLE_GAP_EVT_DISCONNECTED:
//            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						connect_time_out = 0;
						adv_interval = 1600;
            advertising_start(adv_interval, 10);
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            s_sec_keyset.keys_periph.p_enc_key = NULL;
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params,
                                                   &s_sec_keyset);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            // TODO: Adoptation to s110v8.0.0, is this needed anymore ?
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            if (s_sec_keyset.keys_periph.p_enc_key != NULL)
            {
                p_enc_info = &s_sec_keyset.keys_periph.p_enc_key->enc_info;
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device.
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            { 
							adv_interval = 0;
////                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
////                APP_ERROR_CHECK(err_code);
////                // Configure buttons with sense level low as wakeup source.
////                err_code = bsp_buttons_enable(1 << WAKEUP_BUTTON_ID);
////                APP_ERROR_CHECK(err_code);
////                // Go to system-off mode (this function will not return; wakeup will cause a reset).
////                err_code = sd_power_system_off();    
////                APP_ERROR_CHECK(err_code);
//  							pstorage_load(data_array+4, &flash_handle, DATA_LOG_LEN, 0);
//								advertising_start(APP_ADV_INTERVAL , 0);

            }
            break;
        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
//    ble_conn_params_on_ble_evt(p_ble_evt);
//    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}
//static bool                                  m_memory_access_in_progress = false;       /**< Flag to keep track of ongoing operations on persistent memory. */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);

//    switch(sys_evt)
//    {
//        case NRF_EVT_FLASH_OPERATION_SUCCESS:
//						break;
//        case NRF_EVT_FLASH_OPERATION_ERROR:

//            if (m_memory_access_in_progress)
//            {
//                m_memory_access_in_progress = false;
//                advertising_start(APP_ADV_INTERVAL, 0);
//            }
//            break;

//        default:
//            // No implementation needed.
//            break;
//    }
}


/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    ble_opt_t ble_opt;
    ble_opt.gap_opt.scan_req_report.enable = 1;
		err_code = sd_ble_opt_set(BLE_GAP_OPT_SCAN_REQ_REPORT,&ble_opt);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

}


/**@brief  Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


static void weakup_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		uint16_t err_code;
		timer_counter ++;
		if (!(timer_counter % data_periodic)) {
				if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
							err_code = sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
							APP_ERROR_CHECK(err_code);
							m_conn_handle = BLE_CONN_HANDLE_INVALID;
							connect_time_out = 0;
				} else sd_ble_gap_adv_stop();
				err_code = pstorage_block_identifier_get(&flash_base_handle,(pstorage_block_id
							% (PSTORAGE_MAX_APPLICATIONS*PSTORAGE_PAGE_SIZE/DATA_LOG_LEN)), &flash_handle);
				APP_ERROR_CHECK(err_code);
				get_data();
				pstorage_block_id++;
				err_code = pstorage_block_identifier_get(&flash_base_handle,(pstorage_block_id
							% (PSTORAGE_MAX_APPLICATIONS*PSTORAGE_PAGE_SIZE/DATA_LOG_LEN)), &flash_handle);
				APP_ERROR_CHECK(err_code);
				if (!(flash_handle.block_id % PSTORAGE_PAGE_SIZE)){
						pstorage_clear(&flash_handle,PSTORAGE_PAGE_SIZE);
						APP_ERROR_CHECK(err_code);
						}
				advertising_init();
				adv_interval = 1600;
			  advertising_start(adv_interval, 10);
				}
		 else if (adv_interval >= 1600) advertising_init();  //updata timercounter
		 else if (connect_time_out){
				connect_time_out--;
				if (!connect_time_out){
						err_code = sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
					 m_conn_handle = BLE_CONN_HANDLE_INVALID;
						}
				}
//		else advertising_init();

    // Into next connection interval. Send one notification.
//				char_notify();
}
static void example_cb_handler(pstorage_handle_t  * handle,
															 uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
		if(handle->block_id == pstorage_wait_handle) { 
//				pstorage_wait_flag = 0; 
		}  //If we are waiting for this callback, clear the wait flag.
}


/**@brief  Application main function.
 */
int main(void)
{
//    uint8_t start_string[] = START_STRING;
    uint32_t err_code;
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
		static app_timer_id_t       weakup_id;         
    err_code = app_timer_create(&weakup_id,
                                APP_TIMER_MODE_REPEATED,
                                weakup_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&weakup_meantimer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                weakup_meantimeout_handler);
    APP_ERROR_CHECK(err_code);
//    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
    ble_stack_init();
//			NRF_GPIO->PIN_CNF[10] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//																							| (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
//																							| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
//																							| (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
//																							| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
//		nrf_gpio_pin_clear(10);

		NRF_GPIO->PIN_CNF[UART_POWER] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//                                            | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
		nrf_gpio_pin_clear (UART_POWER);
		nrf_delay_ms(200);
//		#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
//		#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

//    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
		pstorage_module_param_t		 param;
		pstorage_init();
		param.block_size  = DATA_LOG_LEN; //PSTORAGE_PAGE_SIZE;                   //Select block size of 16 bytes
		param.block_count = PSTORAGE_MAX_APPLICATIONS * (PSTORAGE_PAGE_SIZE/DATA_LOG_LEN) ;   //Select 10 blocks, total of 160 bytes
		param.cb          = example_cb_handler;   								//Set the pstorage callback handler
		err_code = pstorage_register(&param, &flash_base_handle);
    APP_ERROR_CHECK(err_code);
//		timer_counter = 0x30000;
//		pstorage_block_id = timer_counter/data_periodic;
// test code will be delete		
		for (pstorage_block_id = 0; pstorage_block_id < (PSTORAGE_MAX_APPLICATIONS*PSTORAGE_PAGE_SIZE/DATA_LOG_LEN); pstorage_block_id++){
					err_code = pstorage_block_identifier_get(&flash_base_handle,pstorage_block_id, &flash_handle);
					APP_ERROR_CHECK(err_code);
		      pstorage_load(data_array+4, &flash_handle, DATA_LOG_LEN, 0);
					APP_ERROR_CHECK(err_code);
					if (*(uint64_t * )(data_array+8) == 0xffffffffffffffff) break;
					timer_counter = (uint32_t)data_array;
		}
//		timer_counter = pstorage_block_id * data_periodic;
//test code end. 		
		if (!(pstorage_block_id % 64)){
					err_code = pstorage_block_identifier_get(&flash_base_handle, pstorage_block_id, &flash_handle);
					APP_ERROR_CHECK(err_code);
					err_code = pstorage_clear(&flash_handle, PSTORAGE_PAGE_SIZE);
					APP_ERROR_CHECK(err_code);
		}
    gap_params_init();
    service_add();
    conn_params_init();
    sec_params_init();
    err_code = app_timer_start(weakup_id,  APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
//    printf("%s",start_string);
//		*(uint16_t *)(data_array+18)=0xffff;  //tell on_event not to be load again.
		get_data();
		pstorage_load(data_array+4, &flash_base_handle, DATA_LOG_LEN, 0);
//    advertising_init();
		pstorage_block_id ++;
//    pstorage_load(data_array+4, &flash_base_handle, DATA_LOG_LEN, 0);
//		*(uint32_t *)data_array = timer_counter;
    advertising_start(adv_interval, data_periodic);
    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}


/** 
 * @}
 */
