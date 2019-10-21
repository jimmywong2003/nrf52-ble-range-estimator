/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
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
#include "sdk_common.h"

#if NRF_MODULE_ENABLED(BLE_FTS)
#include "ble_file_transfer_service.h"
#include "ble_srv_common.h"
#include "ble.h"
#include "nrf_gpio.h"

#define NRF_LOG_MODULE_NAME ble_file_transfer_service
#if BLE_FTS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL BLE_FTS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR BLE_FTS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_FTS_CONFIG_DEBUG_COLOR
#else // BLE_FTS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 4
#endif // BLE_FTS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define BLE_UUID_FTS_SERVICE     0x0001

#define BLE_UUID_FTS_TX_DATA_CHARACTERISTIC 0x0002 /**< The UUID of the TX Characteristic. */
#define BLE_UUID_FTS_RX_DATA_CHARACTERISTIC 0x0003 /** < The UUID of the RX Command Characteristic. */
#define BLE_UUID_FTS_TX_CMD_CHARACTERISTIC 0x0004 /**< The UUID of the TX Command Characteristic. */
#define BLE_UUID_FTS_RX_CMD_CHARACTERISTIC 0x0005 /**< The UUID of the RX Characteristic. */

#define BLE_FTS_MAX_RX_DATA_CHAR_LEN BLE_FTS_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_FTS_MAX_TX_DATA_CHAR_LEN BLE_FTS_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */



#define FTS_BASE_UUID                                                                                                  \
        {                                                                                                              \
                {                                                                                                      \
                        0x3F, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E \
                }                                                                                                      \
        } /**< Used vendor specific UUID. */

static volatile uint32_t file_size = 0, file_pos = 0, m_max_data_length = 20;
static uint8_t *file_data;
static ble_fts_t *m_its;

// static ble_fts_img_info_t m_image_info;
static uint32_t m_file_object_sent_byte = 0;


/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_fts     Nordic FILE Transfer Service Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_fts_t *p_fts, ble_evt_t const *p_ble_evt)
{
        ret_code_t err_code;

        if (p_fts != NULL)
        {
                p_fts->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

                if (p_fts->evt_handler != NULL)
                {
                        NRF_LOG_INFO("On Connect");
                        ble_fts_evt_t fts_evt;
                        fts_evt.evt_type = BLE_FTS_EVT_CONNECTED;
                        p_fts->evt_handler(p_fts, &fts_evt);
                }
        }
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_fts     Nordic FILE Transfer Service Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_fts_t *p_fts, ble_evt_t const *p_ble_evt)
{
        UNUSED_PARAMETER(p_ble_evt);
        ble_fts_evt_t fts_evt;
        fts_evt.evt_type = BLE_FTS_EVT_DISCONNECTED;
        p_fts->evt_handler(p_fts, &fts_evt);
        p_fts->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S110 SoftDevice.
 *
 * @param[in] p_fts     Nordic FILE Transfer Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_fts_t *p_fts, ble_evt_t const *p_ble_evt)
{
        ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

        if (
                (p_evt_write->handle == p_fts->tx_data_handles.cccd_handle) &&
                (p_evt_write->len == 2))
        {
                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                        p_fts->tx_data_is_notification_enabled = true;
                        NRF_LOG_INFO("Ready TX Data Notification");
                        ble_fts_evt_t fts_evt;
                        fts_evt.evt_type = BLE_FTS_EVT_TX_DATA_READY;
                        fts_evt.p_data = p_evt_write->data;
                        fts_evt.data_len = p_evt_write->len;
                        p_fts->evt_handler(p_fts, &fts_evt);
                }
                else
                {
                        p_fts->tx_data_is_notification_enabled = false;
                }
        }
        else if (
                (p_evt_write->handle == p_fts->tx_cmd_handles.cccd_handle) &&
                (p_evt_write->len == 2))
        {
                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                        p_fts->tx_cmd_is_notification_enabled = true;
                        NRF_LOG_INFO("Ready TX Command Notification");
                        ble_fts_evt_t fts_evt;
                        fts_evt.evt_type = BLE_FTS_EVT_TX_CMD_READY;
                        fts_evt.p_data = p_evt_write->data;
                        fts_evt.data_len = p_evt_write->len;
                        p_fts->evt_handler(p_fts, &fts_evt);
                }
                else
                {
                        p_fts->tx_cmd_is_notification_enabled = false;
                }
        }
        else if (
                (p_evt_write->handle == p_fts->rx_cmd_handles.value_handle) &&
                (p_fts->evt_handler != NULL))
        {
                ble_fts_evt_t fts_evt;
                fts_evt.evt_type = BLE_FTS_EVT_RX_CMD_RECEIVED;
                fts_evt.p_data = p_evt_write->data;
                fts_evt.data_len = p_evt_write->len;
                p_fts->evt_handler(p_fts, &fts_evt);
        }
        else if (
                (p_evt_write->handle == p_fts->rx_data_handles.value_handle) &&
                (p_fts->evt_handler != NULL))
        {
                ble_fts_evt_t fts_evt;
                fts_evt.evt_type = BLE_FTS_EVT_RX_DATA_RECEIVED;
                fts_evt.p_data = p_evt_write->data;
                fts_evt.data_len = p_evt_write->len;
                p_fts->evt_handler(p_fts, &fts_evt);
        }
        else
        {
                // Do Nothing. This event is not relevant for this service.
        }
}



/**@brief Function for adding TX characteristic.
 *
 * @param[in] p_fts       Nordic FILE Transfer Service Service structure.
 * @param[in] p_fts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_data_char_add(ble_fts_t *p_fts, const ble_fts_init_t *p_fts_init)
{
        /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
        ble_gatts_char_md_t char_md;
        ble_gatts_attr_md_t cccd_md;
        ble_gatts_attr_t attr_char_value;
        ble_uuid_t ble_uuid;
        ble_gatts_attr_md_t attr_md;

        memset(&cccd_md, 0, sizeof(cccd_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

        cccd_md.vloc = BLE_GATTS_VLOC_STACK;

        memset(&char_md, 0, sizeof(char_md));

        char_md.char_props.notify = 1;
        char_md.p_char_user_desc = NULL;
        char_md.p_char_pf = NULL;
        char_md.p_user_desc_md = NULL;
        char_md.p_cccd_md = &cccd_md;
        char_md.p_sccd_md = NULL;

        ble_uuid.type = p_fts->uuid_type;
        ble_uuid.uuid = BLE_UUID_FTS_TX_DATA_CHARACTERISTIC;

        memset(&attr_md, 0, sizeof(attr_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen = 1;

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len = sizeof(uint8_t);
        attr_char_value.init_offs = 0;
        attr_char_value.max_len = BLE_FTS_MAX_TX_DATA_CHAR_LEN;

        return sd_ble_gatts_characteristic_add(p_fts->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_fts->tx_data_handles);
        /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

/**@brief Function for adding TX characteristic.
 *
 * @param[in] p_fts       Nordic FILE Transfer Service Service structure.
 * @param[in] p_fts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_cmd_char_add(ble_fts_t *p_fts, const ble_fts_init_t *p_fts_init)
{
        /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
        ble_gatts_char_md_t char_md;
        ble_gatts_attr_md_t cccd_md;
        ble_gatts_attr_t attr_char_value;
        ble_uuid_t ble_uuid;
        ble_gatts_attr_md_t attr_md;

        memset(&cccd_md, 0, sizeof(cccd_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

        cccd_md.vloc = BLE_GATTS_VLOC_STACK;

        memset(&char_md, 0, sizeof(char_md));

        char_md.char_props.notify = 1;
        char_md.p_char_user_desc = NULL;
        char_md.p_char_pf = NULL;
        char_md.p_user_desc_md = NULL;
        char_md.p_cccd_md = &cccd_md;
        char_md.p_sccd_md = NULL;

        ble_uuid.type = p_fts->uuid_type;
        ble_uuid.uuid = BLE_UUID_FTS_TX_CMD_CHARACTERISTIC;

        memset(&attr_md, 0, sizeof(attr_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen = 1;

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len = 1;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len = BLE_FTS_TX_MAX_CMD_LEN;

        return sd_ble_gatts_characteristic_add(p_fts->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_fts->tx_cmd_handles);
        /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_fts       Nordic FILE Transfer Service Service structure.
 * @param[in] p_fts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_cmd_char_add(ble_fts_t *p_fts, const ble_fts_init_t *p_fts_init)
{
        ble_gatts_char_md_t char_md;
        ble_gatts_attr_t attr_char_value;
        ble_uuid_t ble_uuid;
        ble_gatts_attr_md_t attr_md;

        memset(&char_md, 0, sizeof(char_md));

        char_md.char_props.write = 1;
        char_md.char_props.write_wo_resp = 1;
        char_md.p_char_user_desc = NULL;
        char_md.p_char_pf = NULL;
        char_md.p_user_desc_md = NULL;
        char_md.p_cccd_md = NULL;
        char_md.p_sccd_md = NULL;

        ble_uuid.type = p_fts->uuid_type;
        ble_uuid.uuid = BLE_UUID_FTS_RX_CMD_CHARACTERISTIC;

        memset(&attr_md, 0, sizeof(attr_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen = 1;

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len = 1;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len = BLE_FTS_RX_MAX_CMD_LEN;

        return sd_ble_gatts_characteristic_add(p_fts->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_fts->rx_cmd_handles);
}

/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_fts       Nordic FILE Transfer Service Service structure.
 * @param[in] p_fts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_data_char_add(ble_fts_t *p_fts, const ble_fts_init_t *p_fts_init)
{
        ble_gatts_char_md_t char_md;
        ble_gatts_attr_t attr_char_value;
        ble_uuid_t ble_uuid;
        ble_gatts_attr_md_t attr_md;

        memset(&char_md, 0, sizeof(char_md));

        char_md.char_props.write = 1;
        char_md.char_props.write_wo_resp = 1;
        char_md.p_char_user_desc = NULL;
        char_md.p_char_pf = NULL;
        char_md.p_user_desc_md = NULL;
        char_md.p_cccd_md = NULL;
        char_md.p_sccd_md = NULL;

        ble_uuid.type = p_fts->uuid_type;
        ble_uuid.uuid = BLE_UUID_FTS_RX_DATA_CHARACTERISTIC;

        memset(&attr_md, 0, sizeof(attr_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen = 1;

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len = 1;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len = BLE_FTS_MAX_RX_DATA_CHAR_LEN;

        return sd_ble_gatts_characteristic_add(p_fts->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_fts->rx_data_handles);
}

static uint32_t push_data_packets()
{
        uint32_t return_code = NRF_SUCCESS;
        uint32_t packet_length = m_max_data_length;
        uint32_t packet_size = 0;

        NRF_LOG_INFO("file_size %x, file_pos %x", file_size, file_pos);

        while (return_code == NRF_SUCCESS)
        {
                if (file_pos > file_size)
                {
                        break;
                }
                if ((file_size - file_pos) > packet_length)
                {
                        packet_size = packet_length;
                }
                else if ((file_size - file_pos) > 0)
                {
                        packet_size = file_size - file_pos;
                }

                if (packet_size > 0)
                {
                        return_code = ble_fts_tx_data_send(m_its, &file_data[file_pos], packet_size);
                        if (return_code == NRF_SUCCESS)
                        {
                                file_pos += packet_size;
                                m_file_object_sent_byte += packet_size;
                        }
                }
                else
                {
                        file_size = 0;
                        break;
                }
        }
        return return_code;
}

static volatile bool nrf_error_resources = false;

void ble_fts_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
        if ((p_context == NULL) || (p_ble_evt == NULL))
        {
                return;
        }

//        NRF_LOG_INFO("p_ble_evt->header.evt_id = %x", p_ble_evt->header.evt_id);
        ble_fts_t *p_fts = (ble_fts_t *)p_context;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                on_connect(p_fts, p_ble_evt);
                break;

        case BLE_GAP_EVT_DISCONNECTED:
                on_disconnect(p_fts, p_ble_evt);
                break;

        case BLE_GATTS_EVT_WRITE:
                on_write(p_fts, p_ble_evt);
                break;

        case BLE_GATTS_EVT_HVC:
                NRF_LOG_DEBUG("BLE_GATTS_EVT_HVC");
                break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        {
                if (file_size > 0)
                {
                        push_data_packets();
                }
                else
                {
                        ble_fts_evt_t fts_evt;
                        fts_evt.evt_type = BLE_FTS_EVT_TX_DATA_COMPLETE;
                        p_fts->evt_handler(p_fts, &fts_evt);
                }
                nrf_error_resources = false;
        }
        break;

        default:
                // No implementation needed.
                break;
        }
}

uint32_t ble_fts_init(ble_fts_t *p_fts, const ble_fts_init_t *p_fts_init)
{
        uint32_t err_code;
        ble_uuid_t ble_uuid;
        ble_uuid128_t fts_base_uuid = FTS_BASE_UUID;

        VERIFY_PARAM_NOT_NULL(p_fts);
        VERIFY_PARAM_NOT_NULL(p_fts_init);

        // Initialize the service structure.
        p_fts->conn_handle = BLE_CONN_HANDLE_INVALID;
        p_fts->evt_handler = p_fts_init->evt_handler;
        p_fts->tx_data_is_notification_enabled = false;
        p_fts->tx_cmd_is_notification_enabled = false;


        /**@snippet [Adding proprietary Service to S110 SoftDevice] */
        // Add a custom base UUID.
        err_code = sd_ble_uuid_vs_add(&fts_base_uuid, &p_fts->uuid_type);
        VERIFY_SUCCESS(err_code);

        ble_uuid.type = p_fts->uuid_type;
        ble_uuid.uuid = BLE_UUID_FTS_SERVICE;

        // Add the service.
        err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                            &ble_uuid,
                                            &p_fts->service_handle);
        /**@snippet [Adding proprietary Service to S110 SoftDevice] */
        VERIFY_SUCCESS(err_code);

        // Add the RX Data Characteristic.
        err_code = rx_data_char_add(p_fts, p_fts_init);
        VERIFY_SUCCESS(err_code);

        // Add the TX Data Characteristic.
        err_code = tx_data_char_add(p_fts, p_fts_init);
        VERIFY_SUCCESS(err_code);

        // Add the RX Commmd Characteristic.
        err_code = rx_cmd_char_add(p_fts, p_fts_init);
        VERIFY_SUCCESS(err_code);

        // Add the TX Command Characteristic.
        err_code = tx_cmd_char_add(p_fts, p_fts_init);
        VERIFY_SUCCESS(err_code);

//
//        NRF_LOG_INFO("Handle 0x%04x", p_fts->tx_data_handles);          /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
//        NRF_LOG_INFO("Handle 0x%04x", p_fts->rx_data_handles);
//        NRF_LOG_INFO("Handle 0x%04x", p_fts->tx_cmd_handles);
//        NRF_LOG_INFO("Handle 0x%04x", p_fts->rx_cmd_handles);



        return NRF_SUCCESS;
}

uint32_t ble_fts_tx_data_send(ble_fts_t *p_fts, uint8_t *p_data, uint16_t length)
{
        ble_gatts_hvx_params_t hvx_params;
        uint32_t err_code;

        if (nrf_error_resources)
        {
                return NRF_ERROR_RESOURCES;
        }

        VERIFY_PARAM_NOT_NULL(p_fts);

        if ((p_fts->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_fts->tx_data_is_notification_enabled))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        if (length > BLE_FTS_MAX_DATA_LEN)
        {
                return NRF_ERROR_INVALID_PARAM;
        }

        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle = p_fts->tx_data_handles.value_handle;
        hvx_params.p_data = p_data;
        hvx_params.p_len = &length;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

        err_code = sd_ble_gatts_hvx(p_fts->conn_handle, &hvx_params);

        if (err_code == NRF_ERROR_RESOURCES)
        {
                nrf_error_resources = true;
        }
        return err_code;
}

uint32_t ble_fts_tx_cmd_send(ble_fts_t *p_fts, uint8_t * p_cmd, uint16_t length)
{
        uint8_t data_buf[length];
        ble_gatts_hvx_params_t hvx_params;

        VERIFY_PARAM_NOT_NULL(p_fts);

        if ((p_fts->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_fts->tx_cmd_is_notification_enabled))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        memcpy(&data_buf[0], p_cmd, length);

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_fts->tx_cmd_handles.value_handle;
        hvx_params.p_data = data_buf;
        hvx_params.p_len = &length;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

        return sd_ble_gatts_hvx(p_fts->conn_handle, &hvx_params);
}

uint32_t ble_fts_tx_data_send_file(ble_fts_t *p_fts, uint8_t *p_data, uint32_t data_length, uint32_t max_packet_length)
{
        uint32_t err_code;

        if ((p_fts->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_fts->tx_cmd_is_notification_enabled))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        if (file_size != 0)
        {
                return NRF_ERROR_BUSY;
        }

        m_file_object_sent_byte = 0;

        file_size = data_length;
        file_pos = 0;
        file_data = p_data;
        m_max_data_length = max_packet_length;
        m_its = p_fts;

        err_code = push_data_packets();
        if (err_code == NRF_ERROR_RESOURCES)
                return NRF_SUCCESS;
        return err_code;
}

uint32_t ble_fts_tx_data_send_file_fragment(ble_fts_t *p_fts, uint8_t *p_data, uint32_t data_length)
{
        uint32_t err_code;

        if ((p_fts->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_fts->tx_data_is_notification_enabled))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        if (file_size != 0)
        {
                return NRF_ERROR_BUSY;
        }

        err_code = ble_fts_tx_data_send(p_fts, p_data, data_length);
        return err_code;
}

bool ble_fts_file_transfer_busy(void)
{
        return file_size != 0;
}

#endif // NRF_MODULE_ENABLED(BLE_NUS)
