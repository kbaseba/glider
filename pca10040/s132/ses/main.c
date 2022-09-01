/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_drv_power.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "bmp3.h"
#include "common.h"
#include "nrf_drv_gpiote.h"

# define UART_PRINTING_ENABLED 1

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

#define HDC2010_ADDR          (0x40U)

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define DEVICE_NAME                     "GLIDER1"                          /**< Name of device. Will be included in the advertising data. */
#define APP_TEMPERATURE_UUID            0x0918

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
const uint8_t BEACON_ADDR[BLE_GAP_ADDR_LEN] = {0x9E, 0xB8, 0xB6, 0x61, 0x3C, 0x3A};

APP_TIMER_DEF(advertisingUpdateTimer);

//#define UART_PRINTING_ENABLED                     //Enable to see SAADC output on UART. Comment out for low power operation.
#define UART_TX_BUF_SIZE 256                      //UART TX buffer size. 
#define UART_RX_BUF_SIZE 1                        //UART RX buffer size. 
#define RTC_FREQUENCY 32                          //Determines the RTC frequency and prescaler
#define RTC_CC_VALUE 32                            //Determines the RTC interrupt frequency and thereby the SAADC sampling frequency
#define SAADC_CALIBRATION_INTERVAL 5              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 1                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_4X  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 1                        //Set to 1 to enable BURST mode, otherwise set to 0.


void saadc_init(void);

const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_calibrate = false;     
uint16_t vbat_adc; 

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;
	
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
#ifdef UART_PRINTING_ENABLED
    NRF_LOG_INFO("Starting ADC.");
#endif //UART_PRINTING_ENABLED     
        nrf_drv_saadc_sample();                                        //Trigger the SAADC SAMPLE task
	
#ifdef UART_PRINTING_ENABLED                         
        LEDS_INVERT(BSP_LED_0_MASK);                                   //Toggle LED1 to indicate SAADC sampling start
#endif //UART_PRINTING_ENABLED 			
        err_code = nrf_drv_rtc_cc_set(&rtc,0,RTC_CC_VALUE,true);       //Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match
        APP_ERROR_CHECK(err_code);
        nrf_drv_rtc_counter_clear(&rtc);                               //Clear the RTC counter to start count from zero
    }
}

static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t rtc_config;
    rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);       //Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this applicaiton. Passing NULL here for RTC configuration means that configuration will be taken from the sdk_config.h file.
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc,0,RTC_CC_VALUE,true);           //Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and RTC_FREQUENCY constant in top of main.c
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);                                          //Enable RTC
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
#ifdef UART_PRINTING_ENABLED 	
        LEDS_INVERT(BSP_LED_1_MASK);                                                                    //Toggle LED2 to indicate SAADC buffer full		
#endif //UART_PRINTING_ENABLED 
        if((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0)                                  //Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
        {
            nrf_drv_saadc_abort();                                                                      // Abort all ongoing conversions. Calibration cannot be run if SAADC is busy
            m_saadc_calibrate = true;                                                                   // Set flag to trigger calibration in main context when SAADC is stopped
        }
        
        uint32_t adc_avg = 0;
        for (int i = 0; i < p_event->data.done.size; i++)
        {
            adc_avg += p_event->data.done.p_buffer[i];                                     //Print the SAADC result on UART
        }
        vbat_adc = adc_avg/p_event->data.done.size;

#ifdef UART_PRINTING_ENABLED
        NRF_LOG_INFO("ADC event number: %d\r\n",(int)m_adc_evt_counter);                                //Print the event number on UART

        for (int i = 0; i < p_event->data.done.size; i++)
        {
            NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[i]);                                     //Print the SAADC result on UART
        }
#endif //UART_PRINTING_ENABLED      
        
        if(m_saadc_calibrate == false)
        {
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
            APP_ERROR_CHECK(err_code);
        }
        
        m_adc_evt_counter++;
  
    }
    else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
 #ifdef UART_PRINTING_ENABLED 
        LEDS_INVERT(BSP_LED_2_MASK);                                                                    //Toggle LED3 to indicate SAADC calibration complete
 #endif //UART_PRINTING_ENABLED        
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);             //Need to setup both buffers, as they were both removed with the call to nrf_drv_saadc_abort before calibration.
        APP_ERROR_CHECK(err_code);
        
#ifdef UART_PRINTING_ENABLED
        NRF_LOG_INFO("SAADC calibration complete ! \r\n");                                              //Print on UART
#endif //UART_PRINTING_ENABLED	
        
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;
	
    //Configure SAADC
    saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);
		
    //Configure SAADC channel
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config.acq_time = NRF_SAADC_ACQTIME_40US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config.pin_p = NRF_SAADC_INPUT_AIN2;                                          //Select the input pin for the channel. AIN2 pin maps to physical pin P0.04.
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin

	
    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
		
    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000;                                            //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.		
    }

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);

    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(err_code);

}


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};

// Temperature data
#define APP_TEMPERATURE_LENGTH          0x05
#define APP_TEMPERATURE_SERVICE_UUID    0x16
#define APP_TEMPERATURE_UUID_LE         0x09, 0x18

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    //ble_uuid_t adv_uuids[] = 
    //{
    //    {APP_TEMPERATURE_UUID, BLE_UUID_TYPE_BLE}
    //};

    //// Temperature Service Data
    //uint8_t temp_data[] = 
    //{
    //    APP_TEMPERATURE_LENGTH,
    //    APP_TEMPERATURE_SERVICE_UUID,
    //    APP_TEMPERATURE_UUID_LE,
    //    0x00
    //};
    //uint8_array_t temperature_data_array;
    //temperature_data_array.p_data = (uint8_t *) temp_data;
    //temperature_data_array.size = sizeof(temp_data);

    //ble_advdata_service_data_t service_data;                        // Structure to hold Service Data.
    //service_data.service_uuid = APP_TEMPERATURE_UUID;                // Temp UUID to allow discoverability on iOS devices.
    //service_data.data = temperature_data_array;                     // Array for service advertisement data.

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME; 
    advdata.flags                   = flags;

    //advdata.uuids_more_available.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    //advdata.uuids_more_available.p_uuids  = adv_uuids;
    //advdata.p_service_data_array    = &service_data;                // Pointer to Service Data structure.
    //advdata.service_data_count      = 1;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 27,
       .sda                = 26,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

#define START_WAIT_TIME_MS 499
#define STOP_WAIT_TIME_MS 1
// #define STOP_WAIT_TIME_MS 10000

static void advertisingUpdateTimerHandler(void * p_context)
{
    static bool restart = true;

    if (restart) {
        // payload
        m_adv_data.adv_data.p_data[0] = 0x1E; // length

        for (int i = 1; i < m_adv_data.adv_data.len; i++) {
            m_adv_data.adv_data.p_data[i] = i;
        }

        // start advertising with m_adv_data and m_adv_params
        sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);

        advertising_start();

    } else {
        sd_ble_gap_adv_stop(m_adv_handle);
    }

    app_timer_start(advertisingUpdateTimer, APP_TIMER_TICKS(restart ? START_WAIT_TIME_MS : STOP_WAIT_TIME_MS), NULL);
    restart = !restart;
}



#define THRESHOLD 102000
#define DELTA_THRESH 0
#define SUPERVISORY_PIN 7
#define WATCHDOG_PIN 12
#define MOSFET_PIN 17
#define SWITCH_PIN 29
#define INTRPT_PIN 12

uint8_t transitioned = 0;
int32_t last_press = 101325;
int32_t curr_press = 101325;
int32_t delta;

void input_pin_handle(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // Configure tasks upon interrupt in this event handler
    
}

void gpio_init()
{
    ret_code_t err_code;
    
    // Init GPIOTE
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // Configure pins
    nrf_gpio_cfg_output(SWITCH_PIN);
    nrf_gpio_cfg_output(WATCHDOG_PIN);
    nrf_gpio_cfg_output(MOSFET_PIN);
    nrf_gpio_cfg_input(SUPERVISORY_PIN, NRF_GPIO_PIN_NOPULL);
    
    // Config struct
    //lotohi - active high
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    in_config.pull = NRF_GPIO_PIN_NOPULL;
    
    // Init interrupt_pin
    err_code = nrf_drv_gpiote_in_init(INTRPT_PIN, &in_config, input_pin_handle);
    APP_ERROR_CHECK(err_code);

    // Enable Interrupt
    nrf_drv_gpiote_in_event_enable(INTRPT_PIN, true);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.

    log_init();

    twi_init();
    gpio_init();
    
    
    /**************************************************/
    
    //ble_stack_init();
    //timers_init();

    //// CUSTOM MAC ADDRESS
    //ble_gap_addr_t gap_addr;
    //gap_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    //memcpy(&gap_addr.addr, BEACON_ADDR, sizeof(gap_addr.addr));
    //gap_addr.addr[5] |= 0xc0; // 2 MSBit must be '11' for RANDOM_STATIC address, see v4.0, Vol 3, Part C, chapter 10.8
    //ret_code_t err_code = sd_ble_gap_addr_set(&gap_addr);
    //APP_ERROR_CHECK(err_code);
    ////-------------------

    //// SET NAME
    //ble_gap_conn_sec_mode_t sec_mode;
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    //err_code = sd_ble_gap_device_name_set(&sec_mode,
    //                                 (const uint8_t *)DEVICE_NAME,
    //                                  strlen(DEVICE_NAME));
    //APP_ERROR_CHECK(err_code);
    ////-------------------

    //advertising_init();
    
    //err_code = app_timer_create(&advertisingUpdateTimer,
    //    APP_TIMER_MODE_SINGLE_SHOT,
    //    advertisingUpdateTimerHandler);
    //APP_ERROR_CHECK(err_code);

    //app_timer_start(advertisingUpdateTimer, APP_TIMER_TICKS(START_WAIT_TIME_MS), NULL);

    /**************************************************/


    // BMP Config
    int8_t rslt;
    uint16_t settings_sel;
    struct bmp3_dev dev;
    struct bmp3_data data = { 0 };
    struct bmp3_settings settings = { 0 };
    struct bmp3_status status = { { 0 } };

    /* Interface reference is given as a parameter
     *         For I2C : BMP3_I2C_INTF
     *         For SPI : BMP3_SPI_INTF
     */
    rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

    rslt = bmp3_init(&dev);
    bmp3_check_rslt("bmp3_init", rslt);

    settings.int_settings.drdy_en = BMP3_ENABLE;
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    // Sample temp/press 1/1.28s
    settings.odr_filter.odr = BMP3_ODR_0_78_HZ;
    // Sample temp/press 1/640ms
    //settings.odr_filter.odr = BMP3_ODR_1_5_HZ;
    // Sample temp/press 1/10.24s
    //settings.odr_filter.odr = BMP3_ODR_0_1_HZ;

    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                   BMP3_SEL_DRDY_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);
   
    power_management_init();
    nrf_pwr_mgmt_run();

    while (true)
    {       
        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        /* Read temperature and pressure data iteratively based on data ready interrupt */
        if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE))
        {
            /*
             * First parameter indicates the type of data to be read
             * BMP3_PRESS_TEMP : To read pressure and temperature data
             * BMP3_TEMP       : To read only temperature data
             * BMP3_PRESS      : To read only pressure data
             */
            rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
            bmp3_check_rslt("bmp3_get_sensor_data", rslt);

            /* NOTE : Read status register again to clear data ready interrupt status */
            rslt = bmp3_get_status(&status, &dev);
            bmp3_check_rslt("bmp3_get_status", rslt);

            NRF_LOG_INFO("T: %d deg C, P: %d Pa Delta: %d Pa\n", (data.temperature), (data.pressure), delta);
            NRF_LOG_FLUSH();

            if (transitioned == 0) 
            {
                last_press = curr_press;
                curr_press = data.pressure;
                delta = last_press - curr_press;

                /**** Insert Conditional Here ****
                  1. Make sure delta is less than 0
                  2. Compare data.pressure against a certain threshold (undecided)

                  Read supervisory, if supervisory is high (yellow from volt monitor) toggle output pin to mosfet
                */
                //if (delta < DELTA_THRESH && data.pressure > THRESHOLD)
                if (data.temperature >= 26)
                {
                    // Toggle switch pin high if conditional passes
                    nrf_gpio_pin_set(SWITCH_PIN);
                    // Toggle watchdog timer pin
                    nrf_gpio_pin_set(WATCHDOG_PIN);
                    nrf_delay_us(1);
                    nrf_gpio_pin_clear(WATCHDOG_PIN);

                    // Check supervisory
                    if (nrf_gpio_pin_read(SUPERVISORY_PIN) == 1) 
                    {
                        // Trigger mosfet pin
                        nrf_gpio_pin_set(MOSFET_PIN);
                        nrf_delay_ms(100);
                        nrf_gpio_pin_clear(MOSFET_PIN);
                        // Toggle switch pin
                        nrf_gpio_pin_clear(SWITCH_PIN);
                        // Set global transitioned to 1
                        transitioned = 1;
                    
                    } else {
                        nrf_pwr_mgmt_run();
                    }
                  
                } else {
                    nrf_pwr_mgmt_run();
                }


            } else {
                /**** Send temp data over BLE ****/
                
                nrf_pwr_mgmt_run();
            }
            
            // Unconditional sleep
            nrf_pwr_mgmt_run();

        } else
        {
            nrf_pwr_mgmt_run();
        }
    }
}


/**
 * @}
 */