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

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_FTS_C)
#include <stdlib.h>

#include "ble.h"
#include "ble_file_transfer_service_c.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"

#include "nrf_gpio.h"

#define NRF_LOG_MODULE_NAME ble_file_transfer_service_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define TX_BUFFER_MASK 0x07                 /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE (TX_BUFFER_MASK + 1) /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH BLE_CCCD_VALUE_LEN /**< Length of the write message for CCCD. */
#define WRITE_MESSAGE_LENGTH BLE_CCCD_VALUE_LEN /**< Length of the write message for CCCD. */

typedef enum
{
        READ_REQ, /**< Type identifying that this tx_message is a read request. */
        WRITE_REQ /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
        uint8_t gattc_value[WRITE_MESSAGE_LENGTH]; /**< The message to write. */
        ble_gattc_write_params_t gattc_params;     /**< GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
        uint16_t conn_handle; /**< Connection handle to be used when transmitting this message. */
        tx_request_t type;    /**< Type of this message, i.e. read or write message. */
        union {
                uint16_t read_handle;     /**< Read request message. */
                write_params_t write_req; /**< Write request message. */
        } req;
} tx_message_t;

static tx_message_t m_tx_buffer[TX_BUFFER_SIZE]; /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t m_tx_insert_index = 0;           /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t m_tx_index = 0;                  /**< Current index in the transmit buffer from where the next message to be transmitted resides. */

static volatile uint32_t m_file_size = 0, m_file_pos = 0, m_max_data_length = 20;
static uint8_t *m_file_data;
static ble_fts_c_t *m_fts_c;
static uint32_t count = 0;
static volatile bool nrf_error_resources = false;

static void tx_buffer_process(void)
{
        if (m_tx_index != m_tx_insert_index)
        {
                uint32_t err_code;

                if (m_tx_buffer[m_tx_index].type == READ_REQ)
                {
                        err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                                     m_tx_buffer[m_tx_index].req.read_handle,
                                                     0);
                }
                else
                {
                        err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                                      &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
                }
                if (err_code == NRF_SUCCESS)
                {
                        NRF_LOG_DEBUG("SD Read/Write API returns Success..");
                        m_tx_index++;
                        m_tx_index &= TX_BUFFER_MASK;
                }
                else
                {
                        NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be "
                                      "attempted again..");
                }
        }
}

void ble_fts_c_on_db_disc_evt(ble_fts_c_t *p_ble_fts_c, ble_db_discovery_evt_t *p_evt)
{
        ble_fts_c_evt_t fts_c_evt;
        memset(&fts_c_evt, 0, sizeof(ble_fts_c_evt_t));

        ble_gatt_db_char_t *p_chars = p_evt->params.discovered_db.charateristics;

        // Check if the ITS was discovered.
        if ((p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE) && (p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_FTS_SERVICE) && (p_evt->params.discovered_db.srv_uuid.type == p_ble_fts_c->uuid_type))
        {
                for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
                {
                        switch (p_chars[i].characteristic.uuid.uuid)
                        {
                        case BLE_UUID_FTS_RX_CMD_CHARACTERISTIC:
                                fts_c_evt.handles.fts_rx_cmd_handle = p_chars[i].characteristic.handle_value;
                                break;

                        case BLE_UUID_FTS_RX_DATA_CHARACTERISTIC:
                                fts_c_evt.handles.fts_rx_data_handle = p_chars[i].characteristic.handle_value;
                                break;

                        case BLE_UUID_FTS_TX_DATA_CHARACTERISTIC:
                                fts_c_evt.handles.fts_tx_data_handle = p_chars[i].characteristic.handle_value;
                                fts_c_evt.handles.fts_tx_data_cccd_handle = p_chars[i].cccd_handle;
                                break;

                        case BLE_UUID_FTS_TX_CMD_CHARACTERISTIC:
                                fts_c_evt.handles.fts_tx_cmd_handle = p_chars[i].characteristic.handle_value;
                                fts_c_evt.handles.fts_tx_cmd_cccd_handle = p_chars[i].cccd_handle;
                                break;

                        default:
                                break;
                        }
                }
                NRF_LOG_DEBUG("ITS Service discovered at peer.");
                if (p_ble_fts_c->evt_handler != NULL)
                {
                        if (p_ble_fts_c->conn_handle != BLE_CONN_HANDLE_INVALID)
                        {
                                fts_c_evt.conn_handle = p_evt->conn_handle;
                                fts_c_evt.evt_type = BLE_EVT_FTS_C_DISCOVERY_COMLETE;
                                p_ble_fts_c->evt_handler(p_ble_fts_c, &fts_c_evt);
                        }
                }
        }
}

static void on_write_rsp(ble_fts_c_t *p_ble_fts_c, ble_evt_t const *p_ble_evt)
{
        // Check if the event if on the link for this instance
        if (p_ble_fts_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
        {
                return;
        }
        // Check if there is any message to be sent across to the peer and send it.
        tx_buffer_process();
}

static void on_hvx(ble_fts_c_t *p_ble_fts_c, ble_evt_t const *p_ble_evt)
{
        // Check if the event is on the link for this instance
        if (p_ble_fts_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
        {
                return;
        }

        if ((p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_fts_c->handles.fts_tx_data_handle) && (p_ble_fts_c->evt_handler != NULL))
        {
                //if (p_ble_evt->evt.gattc_evt.params.hvx.len == 1)
                {
                        ble_fts_c_evt_t ble_fts_c_evt;

                        ble_fts_c_evt.evt_type = BLE_EVT_FTS_C_TX_DATA;
                        ble_fts_c_evt.conn_handle = p_ble_fts_c->conn_handle;
                        ble_fts_c_evt.p_data = (uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
                        ble_fts_c_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;
                        p_ble_fts_c->evt_handler(p_ble_fts_c, &ble_fts_c_evt);
                }
        }

        // Check if this is a Button notification.
        if ((p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_fts_c->handles.fts_tx_cmd_handle) && (p_ble_fts_c->evt_handler != NULL))
        {
                //if (p_ble_evt->evt.gattc_evt.params.hvx.len == 1)
                {
                        ble_fts_c_evt_t ble_fts_c_evt;

                        ble_fts_c_evt.evt_type = BLE_EVT_FTS_C_TX_CMD;
                        ble_fts_c_evt.conn_handle = p_ble_fts_c->conn_handle;
                        ble_fts_c_evt.p_data = (uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
                        ble_fts_c_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;
                        p_ble_fts_c->evt_handler(p_ble_fts_c, &ble_fts_c_evt);
                }
        }
}

uint32_t ble_fts_c_init(ble_fts_c_t *p_ble_fts_c, ble_fts_c_init_t *p_ble_fts_c_init)
{
        uint32_t err_code;
        ble_uuid_t fts_uuid;
        ble_uuid128_t fts_base_uuid = FTS_BASE_UUID;

        VERIFY_PARAM_NOT_NULL(p_ble_fts_c);
        VERIFY_PARAM_NOT_NULL(p_ble_fts_c_init);

        p_ble_fts_c->conn_handle = BLE_CONN_HANDLE_INVALID;
        p_ble_fts_c->handles.fts_tx_data_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_fts_c->handles.fts_rx_cmd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_fts_c->handles.fts_rx_data_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_fts_c->handles.fts_tx_cmd_handle = BLE_GATT_HANDLE_INVALID;

        p_ble_fts_c->evt_handler = p_ble_fts_c_init->evt_handler;

        err_code = sd_ble_uuid_vs_add(&fts_base_uuid, &p_ble_fts_c->uuid_type);
        VERIFY_SUCCESS(err_code);

        fts_uuid.type = p_ble_fts_c->uuid_type;
        fts_uuid.uuid = BLE_UUID_FTS_SERVICE;


        return ble_db_discovery_evt_register(&fts_uuid);
}


static bool debug_print = false;

static uint32_t push_c_data_packets()
{
        uint32_t return_code = NRF_SUCCESS;
        uint32_t packet_length = m_max_data_length;
        uint32_t packet_size = 0;

        //NRF_LOG_INFO("%d", count++);


        while (return_code == NRF_SUCCESS)
        {
                if (debug_print)
                        NRF_LOG_INFO("%04x, %04x, %04x", m_file_size, m_file_pos, packet_size );


                if (m_file_pos > m_file_size)
                {
                        break;
                }

                if ((m_file_size - m_file_pos) > packet_length)
                {
                        packet_size = packet_length;
                }
                else if ((m_file_size - m_file_pos) > 0)
                {
                        packet_size = m_file_size - m_file_pos;
                }

                if (packet_size > 0)
                {
                        return_code = ble_fts_c_rx_data_send(m_fts_c, &m_file_data[m_file_pos], packet_size);
                        if (return_code == NRF_SUCCESS)
                        {
                                m_file_pos += packet_size;
                        }
                }
                else
                {
                        m_file_size = 0;
                        break;
                }
        }

        return return_code;
}


void ble_fts_c_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
        ble_fts_c_t *p_ble_fts_c = (ble_fts_c_t *)p_context;

        if ((p_ble_fts_c == NULL) || (p_ble_evt == NULL))
        {
                return;
        }

        if ((p_ble_fts_c->conn_handle != BLE_CONN_HANDLE_INVALID) && (p_ble_fts_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle))
        {
                return;
        }

        switch (p_ble_evt->header.evt_id)
        {

        case BLE_GATTC_EVT_HVX:
                on_hvx(p_ble_fts_c, p_ble_evt);
                break;

        case BLE_GATTC_EVT_WRITE_RSP:
                on_write_rsp(p_ble_fts_c, p_ble_evt);
                break;

        case BLE_GAP_EVT_DISCONNECTED:
                if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_fts_c->conn_handle && p_ble_fts_c->evt_handler != NULL)
                {
                        ble_fts_c_evt_t fts_c_evt;

                        fts_c_evt.evt_type = BLE_EVT_FTS_C_DISCONNECTED;

                        p_ble_fts_c->conn_handle = BLE_CONN_HANDLE_INVALID;
                        p_ble_fts_c->evt_handler(p_ble_fts_c, &fts_c_evt);
                }
                break;
        case BLE_GAP_EVT_CONNECTED:
                if (p_ble_fts_c->evt_handler != NULL)
                {
                        ble_fts_c_evt_t fts_c_evt;

                        fts_c_evt.evt_type = BLE_EVT_FTS_C_CONNECTED;

                        p_ble_fts_c->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                        p_ble_fts_c->evt_handler(p_ble_fts_c, &fts_c_evt);
                }
                break;

        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:

                if (debug_print)
                {
                        NRF_LOG_INFO("BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE %04x", m_file_size);
                }

                if (m_file_size > 0)
                {
                        push_c_data_packets();
                }
                else
                {
                        ble_fts_c_evt_t fts_c_evt;
                        fts_c_evt.evt_type = BLE_EVT_FTS_C_RX_DATA_COMPLETE;
                        p_ble_fts_c->evt_handler(p_ble_fts_c, &fts_c_evt);
                }
                nrf_error_resources = false;
                break;

        default:
                // No implementation needed.
                break;
        }
}

/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t cccd_handle, bool enable)
{
        NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
                      cccd_handle, conn_handle);

        tx_message_t *p_msg;
        uint16_t cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

        p_msg = &m_tx_buffer[m_tx_insert_index++];
        m_tx_insert_index &= TX_BUFFER_MASK;

        p_msg->req.write_req.gattc_params.handle = cccd_handle;
        p_msg->req.write_req.gattc_params.len = WRITE_MESSAGE_LENGTH;
        p_msg->req.write_req.gattc_params.p_value = p_msg->req.write_req.gattc_value;
        p_msg->req.write_req.gattc_params.offset = 0;
        p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
        p_msg->req.write_req.gattc_value[0] = LSB_16(cccd_val);
        p_msg->req.write_req.gattc_value[1] = MSB_16(cccd_val);
        p_msg->conn_handle = conn_handle;
        p_msg->type = WRITE_REQ;

        tx_buffer_process();
        return NRF_SUCCESS;
}

uint32_t ble_fts_c_tx_data_notif_enable(ble_fts_c_t *p_ble_fts_c)
{
        VERIFY_PARAM_NOT_NULL(p_ble_fts_c);

        if ((p_ble_fts_c->conn_handle == BLE_CONN_HANDLE_INVALID) || (p_ble_fts_c->handles.fts_tx_data_cccd_handle == BLE_GATT_HANDLE_INVALID))
        {
                return NRF_ERROR_INVALID_STATE;
        }
        NRF_LOG_DEBUG("ble_image: ble_fts_c_tx_data_notif_enable");
        return cccd_configure(p_ble_fts_c->conn_handle, p_ble_fts_c->handles.fts_tx_data_cccd_handle, true);
}

uint32_t ble_fts_c_tx_cmd_notif_enable(ble_fts_c_t *p_ble_fts_c)
{
        VERIFY_PARAM_NOT_NULL(p_ble_fts_c);

        if ((p_ble_fts_c->conn_handle == BLE_CONN_HANDLE_INVALID) || (p_ble_fts_c->handles.fts_tx_cmd_cccd_handle == BLE_GATT_HANDLE_INVALID))
        {
                return NRF_ERROR_INVALID_STATE;
        }
        NRF_LOG_DEBUG("ble_image: ble_fts_c_tx_cmd_notif_enable");
        return cccd_configure(p_ble_fts_c->conn_handle, p_ble_fts_c->handles.fts_tx_cmd_cccd_handle, true);
}

uint32_t ble_fts_c_handles_assign(ble_fts_c_t *p_ble_fts,
                                  uint16_t conn_handle,
                                  ble_fts_c_handles_t const *p_peer_handles)
{
        VERIFY_PARAM_NOT_NULL(p_ble_fts);

        p_ble_fts->conn_handle = conn_handle;
        if (p_peer_handles != NULL)
        {
                p_ble_fts->handles.fts_tx_data_handle = p_peer_handles->fts_tx_data_handle;
                p_ble_fts->handles.fts_tx_data_cccd_handle = p_peer_handles->fts_tx_data_cccd_handle;

                p_ble_fts->handles.fts_tx_cmd_handle = p_peer_handles->fts_tx_cmd_handle;
                p_ble_fts->handles.fts_tx_cmd_cccd_handle = p_peer_handles->fts_tx_cmd_cccd_handle;

                p_ble_fts->handles.fts_rx_cmd_handle = p_peer_handles->fts_rx_cmd_handle;
                p_ble_fts->handles.fts_rx_data_handle = p_peer_handles->fts_rx_data_handle;

        }
        return NRF_SUCCESS;
}

uint32_t ble_fts_c_rx_cmd_send(ble_fts_c_t * p_ble_fts_c, uint8_t * cmd_data, uint16_t length)
{
        VERIFY_PARAM_NOT_NULL(p_ble_fts_c);

        uint8_t data_buf[length];

        if (p_ble_fts_c->conn_handle == BLE_CONN_HANDLE_INVALID)
        {
                NRF_LOG_WARNING("Connection handle invalid.");
                return NRF_ERROR_INVALID_STATE;
        }
        memcpy(&data_buf[0], cmd_data, length);
        NRF_LOG_DEBUG("%s: handle = %04x, len = %d", __func__, p_ble_fts_c->handles.fts_rx_cmd_handle, length);
        ble_gattc_write_params_t const write_params =
        {
                .write_op = BLE_GATT_OP_WRITE_CMD,
                .flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
                .handle = p_ble_fts_c->handles.fts_rx_cmd_handle,
                .offset = 0,
                .len = length,
                .p_value = data_buf
        };
        ret_code_t ret = sd_ble_gattc_write(p_ble_fts_c->conn_handle, &write_params);
        if (ret != 0)
        {
                return ret;
        }
        APP_ERROR_CHECK(ret);
        return ret;

}



uint32_t ble_fts_c_rx_data_send_object(ble_fts_c_t *p_ble_fts_c, uint8_t *p_data, uint32_t data_length, uint8_t max_packet_length)
{
        uint32_t err_code = NRF_SUCCESS;
        NRF_LOG_INFO("%s addr %08x, data_len = %04x, max_len = %04x", __func__, p_data, data_length, max_packet_length);

        if (data_length == 0x1E4 || data_length == 0x1EC)
        {
                debug_print = true;
        }


        VERIFY_PARAM_NOT_NULL(p_ble_fts_c);

        if (m_file_size != 0)
        {
                return NRF_ERROR_BUSY;
        }

        count = 0;
        m_file_size = data_length;
        m_file_pos = 0;
        m_file_data = p_data;
        m_max_data_length = max_packet_length;
        m_fts_c = p_ble_fts_c;

        err_code = push_c_data_packets();
        if (err_code == NRF_ERROR_RESOURCES)
                return NRF_SUCCESS;

        return err_code;
}

uint32_t ble_fts_c_rx_data_send(ble_fts_c_t *p_ble_fts_c, uint8_t *p_string, uint16_t length)
{
        VERIFY_PARAM_NOT_NULL(p_ble_fts_c);

        if (length > BLE_FTS_MAX_DATA_LEN)
        {
                NRF_LOG_WARNING("Content too long.");
                return NRF_ERROR_INVALID_PARAM;
        }
        if (p_ble_fts_c->conn_handle == BLE_CONN_HANDLE_INVALID)
        {
                NRF_LOG_WARNING("Connection handle invalid.");
                return NRF_ERROR_INVALID_STATE;
        }

        ble_gattc_write_params_t const write_params =
        {
                .write_op = BLE_GATT_OP_WRITE_CMD,
                .flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
                .handle = p_ble_fts_c->handles.fts_rx_data_handle,
                .offset = 0,
                .len = length,
                .p_value = p_string
        };

        return sd_ble_gattc_write(p_ble_fts_c->conn_handle, &write_params);
}

uint32_t ble_fts_c_rx_data_send_object_fragment(ble_fts_c_t *p_fts_c, uint8_t *p_data, uint32_t data_length)
{
        uint32_t err_code;

        if ((p_fts_c->conn_handle == BLE_CONN_HANDLE_INVALID))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        if (m_file_size != 0)
        {
                return NRF_ERROR_BUSY;
        }

        err_code = ble_fts_c_rx_data_send(p_fts_c, p_data, data_length);
        return err_code;
}

bool ble_fts_c_file_transfer_busy(void)
{
        return m_file_size != 0;
}

#endif // NRF_MODULE_ENABLED(BLE_FTS_C)
