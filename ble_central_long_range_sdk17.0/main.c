/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "app_scheduler.h"
#include "ble_image_transfer_service_c.h"
#include "ble_file_transfer_service_c.h"

#include "app_display.h"
#include "packet_error_rate.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define PHY_SELECTION_LED               BSP_BOARD_LED_0                                 /**< LED indicating which phy is in use. */
#define OUTPUT_POWER_SELECTION_LED      BSP_BOARD_LED_1                                 /**< LED indicating at which ouput power the radio is transmitting */
#define CONNECTION_STATE_LED            BSP_BOARD_LED_2
#define SCAN_LED                        BSP_BOARD_LED_3                                 /**< LED indicting if the device is advertising non-connectable advertising or not. */

#define PHY_SELECTION_BUTTON                  BSP_BUTTON_0
#define PHY_SELECTION_BUTTON_EVENT            BSP_EVENT_KEY_0
#define OUTPUT_POWER_SELECTION_BUTTON         BSP_BUTTON_1
#define OUTPUT_POWER_SELECTION_BUTTON_EVENT   BSP_EVENT_KEY_1
#define SCAN_SELECTION_BUTTON                 BSP_BUTTON_2
#define SCAN_SELECTION_BUTTON_EVENT           BSP_EVENT_KEY_2
#define CHANNEL_MAP_SURVEY_BUTTON             BSP_BUTTON_3
#define CHANNEL_MAP_SURVEY_BUTTON_EVENT       BSP_EVENT_KEY_3

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE 20 /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE 10 /**< Maximum number of events in the scheduler queue. */
#endif

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define SCAN_DURATION_WHITELIST     3000                                /**< Duration of the scanning in units of 10 milliseconds. */
#define SCAN_DURATION_WITHOUT_WHITELIST  0                                   /**< Duration of the scanning in units of 10 milliseconds. */

#define TARGET_UUID                 BLE_UUID_ITS_SERVICE                /**< Target device uuid that application is looking for. */

/**@brief Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
        do                           \
        {                            \
                (*(DST))   = (SRC)[1];   \
                (*(DST)) <<= 8;          \
                (*(DST))  |= (SRC)[0];   \
        } while (0)



static void display_update(void);


app_display_content_t m_application_state = {0};                                           /**< Struct containing the dynamic content of the display */


BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

BLE_ITS_C_ARRAY_DEF(m_ble_its_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);                /**< BLE Nordic Image Transfer Service (ITS) client instance. */
BLE_FTS_C_DEF(m_ble_fts_c);                /**< BLE Nordic File Transfer Service (FTS) client instance. */


#define PSR_UPDATE_INTERVAL   APP_TIMER_TICKS(1000)
APP_TIMER_DEF(m_psr_update_timer_id);                    /**< Timer used to toggle LED for "scan mode" indication on the dev.kit. */


static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint16_t m_ble_its_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint16_t m_ble_fts_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static uint8_t rssi_count = 0;

/**@brief Names which the central applications will scan for, and which will be advertised by the peripherals.
 *  if these are set to empty strings, the UUIDs defined below will be used
 */
static char const m_target_periph_name[] = "DeviceUART";      /**< If you want to connect to a peripheral using a given advertising name, type its name here. */
static bool is_connect_per_addr = true;            /**< If you want to connect to a peripheral with a given address, set this to true and put the correct address in the variable below. */

static ble_gap_addr_t const m_target_periph_addr =
{
        /* Possible values for addr_type:
           BLE_GAP_ADDR_TYPE_PUBLIC,
           BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
           BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE,
           BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE. */
        .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
        .addr = {0xC3, 0x11, 0x11, 0x11, 0x11, 0xFF}
};

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
        .uuid = BLE_UUID_NUS_SERVICE,
        .type = NUS_SERVICE_UUID_TYPE
};

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_param_coded_phy =
{
        .extended       = 1,
        .active        = 0x01,
        .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
        .window        = NRF_BLE_SCAN_SCAN_WINDOW,
        .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
        .timeout       = SCAN_DURATION_WITHOUT_WHITELIST,
        .scan_phys     = BLE_GAP_PHY_CODED,
        .report_incomplete_evts = 0,
};

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_param_1MBps =
{
        .active        = 0x01,
        .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
        .window        = NRF_BLE_SCAN_SCAN_WINDOW,
        .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
        .timeout       = SCAN_DURATION_WITHOUT_WHITELIST,
        .scan_phys     = BLE_GAP_PHY_1MBPS,
};


static void scan_start(void);

/********************************************************************************/
/********************************************************************************/


static void request_phy(uint16_t c_handle, uint8_t phy)
{
        ble_gap_phys_t phy_req;
        phy_req.tx_phys = phy;
        phy_req.rx_phys = phy;
        sd_ble_gap_phy_update(c_handle, &phy_req);
}


/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for starting scanning. */
static void scan_start(void)
{
        ret_code_t ret;

        // Stop the scanning
        nrf_ble_scan_stop();

        switch(m_application_state.phy)
        {
        case APP_PHY_CODED:
        {
                NRF_LOG_INFO("Starting scan on coded phy.");
                ret = nrf_ble_scan_params_set(&m_scan, &m_scan_param_coded_phy);
                APP_ERROR_CHECK(ret);
                break;
        }
        case APP_PHY_1M:
        {
                NRF_LOG_INFO("Starting scan on 1Mbps.");
                ret = nrf_ble_scan_params_set(&m_scan, &m_scan_param_1MBps);
                APP_ERROR_CHECK(ret);
                break;
        }
        default:
        {
                NRF_LOG_INFO("Phy selection did not match setup. Scan not started.");
        }
        break;

        }
        m_application_state.app_state = APP_STATE_SCANNING;
        display_update();

        NRF_LOG_INFO("\n Scanning Start\n");

        ret = nrf_ble_scan_start(&m_scan);
        APP_ERROR_CHECK(ret);

        ret = bsp_indication_set(BSP_INDICATE_SCANNING);
        APP_ERROR_CHECK(ret);
}



/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
        ret_code_t err_code;

        switch(p_scan_evt->scan_evt_id)
        {

        case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
        {
        } break;
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
                err_code = p_scan_evt->params.connecting_err.err_code;
                APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
                ble_gap_evt_connected_t const * p_connected =
                        p_scan_evt->params.connected.p_connected;
                // Scan is automatically stopped by the connection.
                NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                             p_connected->peer_addr.addr[0],
                             p_connected->peer_addr.addr[1],
                             p_connected->peer_addr.addr[2],
                             p_connected->peer_addr.addr[3],
                             p_connected->peer_addr.addr[4],
                             p_connected->peer_addr.addr[5]
                             );
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
        } break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
                break;

        case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
                break;

        default:
                break;
        }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
        ret_code_t err_code;
        nrf_ble_scan_init_t init_scan;

        memset(&init_scan, 0, sizeof(init_scan));

        init_scan.p_scan_param     = &m_scan_param_coded_phy;
        init_scan.connect_if_match = true;
        init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

        NRF_LOG_INFO("Scanning Init");

        err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
        APP_ERROR_CHECK(err_code);

        if (strlen(m_target_periph_name) != 0)
        {
                err_code = nrf_ble_scan_filter_set(&m_scan,
                                                   SCAN_NAME_FILTER,
                                                   m_target_periph_name);
                APP_ERROR_CHECK(err_code);
        }

        if (is_connect_per_addr)
        {
                err_code = nrf_ble_scan_filter_set(&m_scan,
                                                   SCAN_ADDR_FILTER,
                                                   m_target_periph_addr.addr);
                APP_ERROR_CHECK(err_code);
        }


        // err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
        // APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_ADDR_FILTER, false);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
        NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                      p_evt->conn_handle,
                      p_evt->conn_handle);
        ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
        ble_its_c_on_db_disc_evt(&m_ble_its_c[p_evt->conn_handle], p_evt);
        ble_fts_c_on_db_disc_evt(&m_ble_fts_c, p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
        ret_code_t ret_val;

        NRF_LOG_DEBUG("Receiving data.");
        NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

        for (uint32_t i = 0; i < data_len; i++)
        {
                do
                {
                        ret_val = app_uart_put(p_data[i]);
                        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
                        {
                                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                                APP_ERROR_CHECK(ret_val);
                        }
                } while (ret_val == NRF_ERROR_BUSY);
        }
        if (p_data[data_len-1] == '\r')
        {
                while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint16_t index = 0;
        uint32_t ret_val;

        switch (p_event->evt_type)
        {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') ||
                    (data_array[index - 1] == '\r') ||
                    (index >= (m_ble_nus_max_data_len)))
                {
                        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                        NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                        do
                        {
                                ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
                                if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
                                {
                                        APP_ERROR_CHECK(ret_val);
                                }
                        } while (ret_val == NRF_ERROR_RESOURCES);

                        index = 0;
                }
                break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
                NRF_LOG_ERROR("Communication error occurred while handling UART.");
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}


/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
        ret_code_t err_code;

        switch (p_ble_nus_evt->evt_type)
        {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_INFO("Discovery complete.");
                err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
                APP_ERROR_CHECK(err_code);

                err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("Connected to device with Nordic UART Service.");
                break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
                ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
                break;

        case BLE_NUS_C_EVT_DISCONNECTED:
                NRF_LOG_INFO("Disconnected.");
                scan_start();
                break;
        }
}
/**@snippet [Handling events from the ble_nus_c module] */

static void ble_its_c_evt_handler(ble_its_c_t *p_ble_its_c, ble_its_c_evt_t const *p_ble_its_evt)
{
        ret_code_t err_code;
        uint32_t receive_byte = 0;

        switch (p_ble_its_evt->evt_type)
        {
        case BLE_ITS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_DEBUG("ITS Service: Discovery complete.");
                err_code = ble_its_c_handles_assign(p_ble_its_c, p_ble_its_evt->conn_handle, &p_ble_its_evt->handles);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("ITS Notification is enabled!!");
                err_code = ble_its_c_tx_notif_enable(p_ble_its_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("ITS Image Notification is enabled !!!");
                err_code = ble_its_c_img_info_notif_enable(p_ble_its_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_INFO("Connected to device with Nordic ITS Service.");
                break;

        case BLE_ITS_C_EVT_ITS_TX_EVT:
                NRF_LOG_DEBUG("ITS Receive the number of bytes = %04d", receive_byte);
                //NRF_LOG_DEBUG("BLE_ITS_C_EVT_ITS_TX_EVT %04d", receive_byte);
                break;

        case BLE_ITS_C_EVT_ITS_IMG_INFO_EVT:
                //NRF_LOG_INFO("ITS Image Info: the number of bytes = %04d", receive_byte);
                //NRF_LOG_DEBUG("BLE_ITS_C_EVT_ITS_IMG_INFO_EVT %04d", receive_byte);
                break;

        case BLE_ITS_C_EVT_ITS_RX_COMPLETE_EVT:
                NRF_LOG_DEBUG("RX COMPLETE");
                break;

        case BLE_ITS_C_EVT_DISCONNECTED:
                NRF_LOG_DEBUG("Disconnected.");
                break;
        }
}


static void ble_fts_c_evt_handler(ble_fts_c_t *p_ble_fts_c, ble_fts_c_evt_t const *p_ble_fts_evt)
{
        ret_code_t err_code;

        switch (p_ble_fts_evt->evt_type)
        {
        case BLE_EVT_FTS_C_DISCOVERY_COMLETE:
                NRF_LOG_DEBUG("FTS Service: Discovery complete.");
                err_code = ble_fts_c_handles_assign(p_ble_fts_c, p_ble_fts_evt->conn_handle, &p_ble_fts_evt->handles);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("FTS Data Notification is enabled!!");
                err_code = ble_fts_c_tx_data_notif_enable(p_ble_fts_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("FTS Command Notification is enabled !!!");
                err_code = ble_fts_c_tx_cmd_notif_enable(p_ble_fts_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_INFO("Connected to device with Nordic FTS Service.");
                break;

        case BLE_EVT_FTS_C_TX_DATA:               /**< Event indicating that the central has received data something from a peer. */
                NRF_LOG_DEBUG("BLE_EVT_FTS_C_TX_DATA");
                NRF_LOG_DEBUG("Receive data from device");
                NRF_LOG_HEXDUMP_DEBUG(p_ble_fts_evt->p_data, p_ble_fts_evt->data_len);

                break;

        case BLE_EVT_FTS_C_TX_CMD:                       /**< Event indicating that the central has received command something from a peer. */
                NRF_LOG_DEBUG("BLE_EVT_FTS_C_TX_CMD");
                NRF_LOG_DEBUG("Receive cmd from device");
                NRF_LOG_HEXDUMP_DEBUG(p_ble_fts_evt->p_data, p_ble_fts_evt->data_len);

                break;
        case BLE_EVT_FTS_C_RX_CMD:
                NRF_LOG_DEBUG("BLE_EVT_FTS_C_RX_CMD");
                NRF_LOG_HEXDUMP_DEBUG(p_ble_fts_evt->p_data, p_ble_fts_evt->data_len);
                break;

        case BLE_EVT_FTS_C_RX_DATA:
                NRF_LOG_DEBUG("BLE_EVT_FTS_C_RX_DATA");
                NRF_LOG_HEXDUMP_DEBUG(p_ble_fts_evt->p_data, p_ble_fts_evt->data_len);
                break;

        case BLE_EVT_FTS_C_RX_DATA_COMPLETE:               /**< Event indicating that the central has written data to peripheral completely. */
                NRF_LOG_DEBUG("BLE_EVT_FTS_C_RX_DATA_COMPLETE");
                break;

        case BLE_EVT_FTS_C_CONNECTED:
                break;

        case BLE_EVT_FTS_C_DISCONNECTED:
                NRF_LOG_DEBUG("Disconnected.");

                break;
        }
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        ret_code_t err_code;
        ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = ble_its_c_handles_assign(&m_ble_its_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = ble_fts_c_handles_assign(&m_ble_fts_c, p_gap_evt->conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = sd_ble_gap_rssi_start(p_ble_evt->evt.gap_evt.conn_handle, 1, 2);
                APP_ERROR_CHECK(err_code);

                err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
                APP_ERROR_CHECK(err_code);

                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

                // start discovery of services. The NUS Client waits for a discovery result
                err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
                APP_ERROR_CHECK(err_code);

                packet_error_rate_reset_counter();
                packet_error_rate_detect_enable();

                m_application_state.app_state = APP_STATE_CONNECTED;
                display_update();

                // Current state is scanning, not trying to connect. Start blinking associated LED
                err_code = app_timer_start(m_psr_update_timer_id, PSR_UPDATE_INTERVAL, NULL);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GAP_EVT_DISCONNECTED:

                NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);

                packet_error_rate_detect_disable();

                // Current state is scanning, not trying to connect. Start blinking associated LED
                err_code = app_timer_stop(m_psr_update_timer_id);
                APP_ERROR_CHECK(err_code);

                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                scan_start();
                display_update();

                break;

        case BLE_GAP_EVT_TIMEOUT:
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
                {
                        NRF_LOG_INFO("Connection Request timed out.");
                }
                break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
                // Pairing not supported.
                err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
                // Accepting parameters requested by peer.
                err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                        &p_gap_evt->params.conn_param_update_request.conn_params);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
                NRF_LOG_DEBUG("PHY update request.");
                ble_gap_phys_t const phys =
                {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                NRF_LOG_DEBUG("GATT Client Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                NRF_LOG_DEBUG("GATT Server Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
                //NRF_LOG_INFO("BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE");
                break;

        case BLE_GAP_EVT_RSSI_CHANGED:
                if (rssi_count % 10 == 0)
                {
                        packet_error_rate_timeout_handler();
                        NRF_LOG_INFO("RSSI = %d (dBm), PSR = %03d%%", p_ble_evt->evt.gap_evt.params.rssi_changed.rssi, get_packet_success_rate());

                        m_application_state.rssi[p_ble_evt->evt.gap_evt.conn_handle] = p_ble_evt->evt.gap_evt.params.rssi_changed.rssi;
                        display_update();

                }
                rssi_count++;

        default:
                break;
        }
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

        ble_cfg_t ble_cfg;
        // Configure the GATTS attribute table.
        memset(&ble_cfg, 0x00, sizeof(ble_cfg));
        ble_cfg.gap_cfg.role_count_cfg.periph_role_count = NRF_SDH_BLE_PERIPHERAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_SDH_BLE_CENTRAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.qos_channel_survey_role_available = true; /* Enable channel survey role */

        err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, &ram_start);
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_GAP_CFG_ROLE_COUNT.",
                              nrf_strerror_get(err_code));
        }

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
        APP_ERROR_CHECK(err_code);

        ble_gap_addr_t ble_address = {.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
                                      .addr_id_peer = 0,
                                      .addr = {0xC2, 0x11, 0x11, 0x11, 0x11, 0xFF}};
        err_code = sd_ble_gap_addr_set(&ble_address);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
        {
                NRF_LOG_INFO("ATT MTU exchange completed.");
                m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                m_ble_its_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                m_ble_fts_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;

                NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
        }
        else if (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED)
        {
                NRF_LOG_INFO("Data length updated to %u bytes.", p_evt->params.data_length);
        }
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
}


#ifdef WITH_LCD_ILI9341
static void display_update()
{
        static bool first_update = true;
        if(first_update)
        {
                m_application_state.main_title = "Central LRange";
                app_display_create_main_screen(&m_application_state);
                first_update = false;
        }
        else
        {
                app_display_update_main_screen(&m_application_state);
        }
        app_display_update();
}
#else
static void display_update()
{

}
#endif


static void ble_go_to_idle(void)
{
        ret_code_t err_code;

        NRF_LOG_INFO("%s : App_state %d", m_application_state.app_state);
        switch(m_application_state.app_state)
        {
        case APP_STATE_SCANNING:
                NRF_LOG_INFO("Scanning Stop");
                nrf_ble_scan_stop();
                break;

        case APP_STATE_CONNECTED:
                if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
                {
                        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                        APP_ERROR_CHECK(err_code);
                }
                break;

                // case APP_STATE_DISCONNECTED:
                //         err_code = app_timer_stop(m_restart_advertising_timer_id);
                //         APP_ERROR_CHECK(err_code);
                //         break;
        }
        m_application_state.app_state = APP_STATE_IDLE;
        display_update();
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {

        case PHY_SELECTION_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        NRF_LOG_INFO("PHY_SELECTION_BUTTON");
                        switch (m_application_state.app_state)
                        {
                        case APP_STATE_CONNECTED:
                        {

                        }
                        break;
                        case APP_STATE_IDLE:
                        {
                                m_application_state.phy = (m_application_state.phy + 1) % APP_PHY_LIST_END;
                                switch(m_application_state.phy)
                                {
                                case APP_PHY_CODED:
                                {
                                        NRF_LOG_INFO("Starting scan on coded phy.");
                                        break;
                                }
                                case APP_PHY_1M:
                                {
                                        NRF_LOG_INFO("Starting scan on 1Mbps.");
                                        break;
                                }
                                default:
                                {
                                        NRF_LOG_INFO("Phy selection did not match setup. Scan not started.");
                                }
                                break;
                                }
                                display_update();
                                break;
                        }
                        case APP_STATE_SCANNING:
                        {
                                m_application_state.phy = (m_application_state.phy + 1) % APP_PHY_LIST_END;
                                switch(m_application_state.phy)
                                {
                                case APP_PHY_CODED:
                                {
                                        NRF_LOG_INFO("Starting scan on coded phy.");
                                        break;
                                }
                                case APP_PHY_1M:
                                {
                                        NRF_LOG_INFO("Starting scan on 1Mbps.");
                                        break;
                                }
                                default:
                                {
                                        NRF_LOG_INFO("Phy selection did not match setup. Scan not started.");
                                }
                                break;
                                }
                                scan_start();
                                display_update();
                        }
                        }
                }
                break;

        case OUTPUT_POWER_SELECTION_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        NRF_LOG_INFO("OUTPUT_POWER_SELECTION_BUTTON");
                        m_application_state.tx_power = (m_application_state.tx_power + 1) % 3;
                        int8_t tx_power = (int8_t)(m_application_state.tx_power * 4);
                        if(m_application_state.app_state == APP_STATE_CONNECTED)
                        {
                                err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, tx_power);
                                APP_ERROR_CHECK(err_code);
                                NRF_LOG_INFO("TX Power for Connection: %d", tx_power);
                        }
                        else
                        {
                                // Set the correct TX power.
                                err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, NULL, tx_power);
                                APP_ERROR_CHECK(err_code);

                                NRF_LOG_INFO("TX Power for Advertising: %d", tx_power);
                        }
                        display_update();
                }
                break;

        case SCAN_SELECTION_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        NRF_LOG_INFO("SCAN_SELECTION_BUTTON");
                        if(m_application_state.app_state == APP_STATE_IDLE)
                        {
                                scan_start();
                        }
                        else
                        {
                                ble_go_to_idle();
                        }
                }
                break;
        case CHANNEL_MAP_SURVEY_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {


                }
                break;

        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
        ret_code_t err_code;

        //The array must be static because a pointer to it will be saved in the button handler module.
        static app_button_cfg_t buttons[] =
        {
                // {HOST_RX_DATA_SEND_BUTTON, false, BUTTON_PULL, button_event_handler},
                // {HOST_RX_CMD_SEND_BUTTON,  false, BUTTON_PULL, button_event_handler},
                {PHY_SELECTION_BUTTON, false, BUTTON_PULL, button_event_handler},
                {OUTPUT_POWER_SELECTION_BUTTON,  false, BUTTON_PULL, button_event_handler},
                {SCAN_SELECTION_BUTTON, false, BUTTON_PULL, button_event_handler},
                {CHANNEL_MAP_SURVEY_BUTTON, false, BUTTON_PULL, button_event_handler}
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the UART. */
static void uart_init(void)
{
        ret_code_t err_code;

        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
                .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);

        APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
        ret_code_t err_code;
        ble_nus_c_init_t init;

        init.evt_handler   = ble_nus_c_evt_handler;
        init.error_handler = nus_error_handler;
        init.p_gatt_queue  = &m_ble_gatt_queue;

        err_code = ble_nus_c_init(&m_ble_nus_c, &init);
        APP_ERROR_CHECK(err_code);
}

static void its_c_init(void)
{
        ret_code_t err_code;
        ble_its_c_init_t its_init;

        its_init.evt_handler = ble_its_c_evt_handler;
        for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
        {
                err_code = ble_its_c_init(&m_ble_its_c[i], &its_init);
                APP_ERROR_CHECK(err_code);
        }
}

static void fts_c_init(void)
{
        ret_code_t err_code;
        ble_fts_c_init_t fts_init;

        fts_init.evt_handler = ble_fts_c_evt_handler;
        err_code = ble_fts_c_init(&m_ble_fts_c, &fts_init);
        APP_ERROR_CHECK(err_code);
}

static void psr_update_timeout_handler(void * p_context)
{
        ret_code_t err_code;

        uint8_t psr_value;

        m_application_state.psr = get_packet_success_rate();
        psr_value = m_application_state.psr;

        err_code = ble_fts_c_rx_cmd_send(&m_ble_fts_c, &psr_value, sizeof(psr_value));

        display_update();
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);

        // Creating the timers used to indicate the state/selection mode of the board.
        err_code = app_timer_create(&m_psr_update_timer_id, APP_TIMER_MODE_REPEATED, psr_update_timeout_handler);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
        ret_code_t err_code;
        err_code = nrf_pwr_mgmt_init();
        APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
        ble_db_discovery_init_t db_init;

        memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

        db_init.evt_handler  = db_disc_handler;
        db_init.p_gatt_queue = &m_ble_gatt_queue;

        ret_code_t err_code = ble_db_discovery_init(&db_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
                nrf_pwr_mgmt_run();
        }
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

int main(void)
{
        // Initialize.
        log_init();
        timer_init();
        //uart_init();

        buttons_init();
        db_discovery_init();
        power_management_init();
        ble_stack_init();
        scheduler_init();
        gatt_init();

        nus_c_init();
        its_c_init();
        fts_c_init();
        
        scan_init();

        // Start execution.
        NRF_LOG_INFO("\n\n\n\n");
        NRF_LOG_INFO("Long range demo  --central-.");
#ifdef WITH_LCD_ILI9341
        app_display_init(&m_application_state);
#endif
        display_update();


        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}
