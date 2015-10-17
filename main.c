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
static uint16_t timer_adv_interval = 0;
#define APP_ADV_TIMEOUT_IN_SECONDS      0    //180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0x00FF  //0                                      /**< Value of the RTC1 PRESCALER register. */
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

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_POWER  	 12 //8  //3       //5TM Power
//#define DEVICE_NAME                     "CSTDataLog"                               /**< Name of device. Will be included in the advertising data. */
//uint8_t device_name[15]  = "5T:" ;
uint32_t baudrate_select = 1200;
//#define ADC_AIN						  ADC_CONFIG_PSEL_AnalogInput5  //pin04   BEACON
#define ADC_AIN						  ADC_CONFIG_PSEL_AnalogInput4  //pin03  
#define LED_POWER  				      20
//#define DATA_PERIODIC         300
uint32_t data_periodic = 				60; //1024; //256; //16;  //180;    //60;
static uint32_t timer_counter = 0;
static uint8_t data_array[32];  // flash load packet
static uint8_t data_5888[32];  //adv broadcast packet
#define PSTORAGE_PAGE_SIZE		1024
#define DATA_LOG_LEN					16
static pstorage_block_t pstorage_wait_handle = 0;
static pstorage_handle_t       flash_base_handle;
static uint32_t 	pstorage_block_id, pstorage_next_adv;
static pstorage_handle_t 				flash_handle ;
static app_timer_id_t  					weakup_meantimer_id	;
static app_timer_id_t           weakup_id;         
static uint16_t 	company_id = 0x5888;
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS        1200                                      /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION         3                                         /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS       270                                       /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)
				
uint16_t battery_start(uint32_t AnalogInput)
{
		sd_clock_hfclk_request();
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
	 	sd_clock_hfclk_release();
    return  ADC_RESULT_IN_MILLI_VOLTS(NRF_ADC->RESULT); // + DIODE_FWD_VOLT_DROP_MILLIVOLTS;
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

 char char_hex( char bHex){
		bHex&=0x0f;
    if((bHex<10))
        bHex += '0';
    else bHex += 'A'-10;
//    else bHex = 0xff;
    return bHex;
}

static void advertising_init(void)
{
    static uint32_t      err_code;
    static ble_advdata_t advdata;
    static ble_advdata_t scanrsp;
    static uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    static ble_advdata_manuf_data_t  manuf_data;

//    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_NO_NAME;
//		advdata.short_name_len          = 16;
//    advdata.include_appearance      = false;
    advdata.flags                   = flags;
//    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
//    advdata.uuids_complete.p_uuids  = adv_uuids;
    memset(&scanrsp, 0, sizeof(scanrsp));
		manuf_data.company_identifier   = company_id; //0x5881;
		manuf_data.data.size						= 24;  //BLE_NUS_MAX_DATA_LEN+6;
		if (company_id == 0x5888)  {
			manuf_data.data.p_data	= data_5888;
			*(uint32_t *)(data_5888) = timer_counter + timer_adv_interval /1600;      
		}
		else	{
			manuf_data.data.p_data  = data_array;
			*(uint32_t *)(data_array) = timer_counter + timer_adv_interval /1600;      
		}
			scanrsp.p_manuf_specific_data		= &manuf_data;
//		advdata.p_manuf_specific_data		= &manuf_data;
    data_array[20] = '5';
    data_array[21] = 'T';
    data_array[22] = 'M';
    data_array[23] = '-';
    err_code = ble_advdata_set(&advdata, &scanrsp);
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
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_SCAN_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = adv_interval;
    adv_params.timeout     = adv_timeout;
    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}

static void get_data(void)
{
    static uint8_t uart_data[16];  //why must be static?
		static uint8_t log_data[16];
    uint8_t uart_index = 0, cr;
    uint32_t err_code;
		NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
//		simple_uart_config(NULL, 4, NULL, 16, false);
		simple_uart_config(NULL, 16, NULL, 4, false);
		nrf_gpio_pin_set (UART_POWER);												//Open power and UART for 5TM 
		while (simple_uart_get_with_timeout(4, &cr));					//clear RX buffer, must be!
		memset(uart_data,0,16);
//		memset(log_data,0,16);
		memset(data_5888,0,20);
		for (uint8_t i=0; i<50; i++){										//Get date from 5tm 
				if (simple_uart_get_with_timeout(4, &cr))  {
						uart_data[uart_index++] = cr;
						if ((uart_data[uart_index - 1] == '\n') || ((uart_index) >= 16))
								{
//										sscanf((const char *)(uart_data), "%hx%hx%hx\n",(uint16_t *)(log_data+4),(uint16_t *)(log_data+6),(uint16_t *)(log_data+8));
										sscanf((const char *)(uart_data), "%hx%hx%hx\n",(uint16_t *)(data_5888+8),(uint16_t *)(data_5888+10),(uint16_t *)(data_5888+12));
										break;
										}
						}
				}
		NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);
		nrf_gpio_pin_clear (UART_POWER);
		int32_t nrf_temp;
		sd_temp_get(&nrf_temp);  													//Get Cpu tempreature
//		*(uint32_t *)(log_data) = timer_counter;      
//		*(uint16_t *)(log_data+10) = battery_start(ADC_AIN);
//		*(uint32_t *)(log_data+12) = nrf_temp;
//		err_code = pstorage_store(&flash_handle, log_data, DATA_LOG_LEN, 0 );
		*(uint32_t *)(data_5888 +4) = timer_counter;      
		*(uint16_t *)(data_5888+14) = battery_start(ADC_AIN);
		*(uint32_t *)(data_5888+16) = nrf_temp;
		err_code = pstorage_store(&flash_handle, data_5888 + 4 , DATA_LOG_LEN, 0 );
		APP_ERROR_CHECK(err_code);
		company_id = 0x5888;
}

static void weakup_meantimeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		sd_ble_gap_adv_stop();
		company_id = 0x5888;
	  adv_interval = 1600;
//	  advertising_init();
//	  advertising_start(adv_interval , 10);
}

/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t            err_code;
    switch (p_ble_evt->header.evt_id)
    {
				case BLE_GAP_EVT_SCAN_REQ_REPORT:
						if (p_ble_evt->evt.gap_evt.params.scan_req_report.peer_addr.addr[0] == 0x58 && 
									p_ble_evt->evt.gap_evt.params.scan_req_report.peer_addr.addr[1] == 0x81){
								if (company_id == 0x5882) pstorage_next_adv ++;
								else company_id = 0x5882;  
								 if (pstorage_next_adv < pstorage_block_id){
												err_code = pstorage_block_identifier_get(&flash_base_handle,(pstorage_next_adv
														% (PSTORAGE_MAX_APPLICATIONS*PSTORAGE_PAGE_SIZE/DATA_LOG_LEN)), &flash_handle);
												APP_ERROR_CHECK(err_code);
												pstorage_load(data_array+4, &flash_handle, DATA_LOG_LEN, 0);
												*(uint16_t *)(data_array+18) = (uint16_t)(pstorage_block_id - pstorage_next_adv);
												app_timer_stop(weakup_meantimer_id);
												app_timer_start(weakup_meantimer_id,  APP_TIMER_TICKS(3100, APP_TIMER_PRESCALER), NULL);
												if ((adv_interval > 200) && ((pstorage_block_id - pstorage_next_adv) > 16)){
															sd_ble_gap_adv_stop();
															adv_interval = BLE_GAP_ADV_NONCON_INTERVAL_MIN;   //not less then 160 will cause err_code 007  in BLE_GAP_ADV_TYPE_ADV_SCAN_IND mode.
															advertising_start(adv_interval, 20);
												}
									}	else 	{
											err_code = sd_ble_gap_adv_stop();
											app_timer_stop(weakup_meantimer_id);
											adv_interval = 0;
											}
								}
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

static void weakup_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		uint16_t err_code;
		timer_counter +=data_periodic;
		timer_adv_interval = 0;
		sd_ble_gap_adv_stop();

		if (!(timer_counter % 900)) {
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
				    if (((pstorage_block_id / (PSTORAGE_PAGE_SIZE/DATA_LOG_LEN)) % PSTORAGE_MAX_APPLICATIONS )== 
											((pstorage_next_adv / (PSTORAGE_PAGE_SIZE/DATA_LOG_LEN)) % PSTORAGE_MAX_APPLICATIONS ))
													pstorage_next_adv = (pstorage_next_adv / (PSTORAGE_PAGE_SIZE/DATA_LOG_LEN) + 1) * (PSTORAGE_PAGE_SIZE/DATA_LOG_LEN) ;
						}
//				advertising_init();
//				adv_interval = 1600;
//			  advertising_start(adv_interval, 10);
				}
//		 else {
    adv_interval = 1600;
		company_id = 0x5888;
    advertising_init();  //updata timercounter
		advertising_start(adv_interval, 20);
//		 }
		app_timer_stop(weakup_meantimer_id);
		err_code = app_timer_start(weakup_meantimer_id,  APP_TIMER_TICKS(500, APP_TIMER_PRESCALER), NULL);
		APP_ERROR_CHECK(err_code);
	 
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

void ble_on_radio_active_evt(bool radio_active)
{
		if (radio_active) {
			timer_adv_interval += adv_interval;
			advertising_init();
		}
}

/**@brief  Application main function.
 */
int main(void)
{
    uint32_t err_code;
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = app_timer_create(&weakup_id,
                                APP_TIMER_MODE_REPEATED,
                                weakup_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&weakup_meantimer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                weakup_meantimeout_handler);
    APP_ERROR_CHECK(err_code);
    ble_stack_init();
	  err_code = sd_ble_gap_tx_power_set(4);
    APP_ERROR_CHECK(err_code);
	
//			NRF_GPIO->PIN_CNF[10] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//																							| (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
//																							| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
//																							| (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
//																							| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
//		nrf_gpio_pin_clear(10);

		NRF_GPIO->PIN_CNF[UART_POWER] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
		nrf_gpio_pin_clear (UART_POWER);
//		nrf_delay_ms(200);
		pstorage_module_param_t		 param;
		pstorage_init();
		param.block_size  = DATA_LOG_LEN; //PSTORAGE_PAGE_SIZE;                   //Select block size of 16 bytes
		param.block_count = PSTORAGE_MAX_APPLICATIONS * (PSTORAGE_PAGE_SIZE/DATA_LOG_LEN) ;   //Select 10 blocks, total of 160 bytes
		param.cb          = example_cb_handler;   								//Set the pstorage callback handler
		err_code = pstorage_register(&param, &flash_base_handle);
    APP_ERROR_CHECK(err_code);
		pstorage_block_id = 0;
		pstorage_next_adv = 0;
		err_code = pstorage_clear(&flash_base_handle, PSTORAGE_PAGE_SIZE);
		APP_ERROR_CHECK(err_code);

//		timer_counter = -1;
    err_code = app_timer_start(weakup_id,  APP_TIMER_TICKS(data_periodic*1000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
		err_code = ble_radio_notification_init(NRF_APP_PRIORITY_LOW,
																					 NRF_RADIO_NOTIFICATION_DISTANCE_1740US,
																					 ble_on_radio_active_evt);
		APP_ERROR_CHECK(err_code);
		timer_counter = 0 - data_periodic;
		weakup_timeout_handler(NULL);
    for (;;)
    {
				err_code = sd_app_evt_wait();
				APP_ERROR_CHECK(err_code);    
		}
}


/** 
 * @}
 */
