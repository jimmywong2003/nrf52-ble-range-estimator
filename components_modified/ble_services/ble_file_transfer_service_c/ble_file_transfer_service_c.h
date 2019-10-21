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
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFFTS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**@file
 *
 * @defgroup ble_its Nordic File Transfer Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Nordic File Transfer Service implementation.
 *
 * @details The Nordic File Transfer Service is a simple GATT-based service with TX and RX characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note The application must propagate SoftDevice events to the Nordic File Transfer Service module
 *       by calling the ble_fts_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_FILE_TRANSFER_SERVICE_C_H__
#define BLE_FILE_TRANSFER_SERVICE_C_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"

#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__
    #ifdef PACKED
        #undef PACKED
    #endif

    #define PACKED(TYPE) TYPE __attribute__ ((packed))
#endif


#define BLE_FTS_C_DEF(_name)                                                                      \
        static ble_fts_c_t _name;                                                                           \
        NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                             BLE_FTS_C_BLE_OBSERVER_PRIO,                                                   \
                             ble_fts_c_on_ble_evt, &_name)

#define BLE_FTS_C_ARRAY_DEF(_name, _cnt)                 \
        static ble_fts_c_t _name[_cnt];                          \
        NRF_SDH_BLE_OBSERVERS(_name ## _obs,                     \
                              BLE_FTS_C_BLE_OBSERVER_PRIO,       \
                              ble_fts_c_on_ble_evt, &_name, _cnt)

#define FTS_BASE_UUID                  {{0x3F, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */

#define BLE_UUID_FTS_SERVICE           0x0001                      /**< The UUID of the Nordic File Transfer Service. */
#define BLE_UUID_FTS_TX_DATA_CHARACTERISTIC 0x0002                      /**< The UUID of the TX Characteristic. */
#define BLE_UUID_FTS_RX_DATA_CHARACTERISTIC 0x0003                 /**< The UUID of the RX DATA Characteristic. */
#define BLE_UUID_FTS_TX_CMD_CHARACTERISTIC 0x0004
#define BLE_UUID_FTS_RX_CMD_CHARACTERISTIC 0x0005                      /**< The UUID of the RX Characteristic. */

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_FTS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic File Transfer service module. */
#else
    #define BLE_FTS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic File Transfer service module. */
    #warning NRF_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

#define BLE_FTS_TX_MAX_CMD_LEN (20)
#define BLE_FTS_RX_MAX_CMD_LEN (20)

typedef enum
{
        BLE_EVT_FTS_C_DISCOVERY_COMLETE, /**< Event indicating that the NUS service and its characteristics was found. */
        BLE_EVT_FTS_C_TX_DATA,           /**< Event indicating that the central has received something from a peer. */
        BLE_EVT_FTS_C_RX_DATA,           /**< Event indicating that the central has received something from a peer. */
        BLE_EVT_FTS_C_RX_CMD,            /**< Event indicating that the central has written to peripheral. */
        BLE_EVT_FTS_C_RX_DATA_COMPLETE,       /**< Event indicating that the central has written to peripheral completely. */
        BLE_EVT_FTS_C_TX_CMD,       /**< Event indicating that the central has received something from a peer. */
        BLE_EVT_FTS_C_CONNECTED,        /**< Event indicating that the NUS server has disconnected. */
        BLE_EVT_FTS_C_DISCONNECTED      /**< Event indicating that the NUS server has disconnected. */
} ble_fts_c_evt_type_t;


/**@brief Handles on the connected peer device needed to interact with it. */
typedef PACKED ( struct ble_fts_c_handles_s
{
        uint16_t fts_tx_data_handle;  /**< Handle of the NUS TX characteristic as provided by a discovery. */
        uint16_t fts_tx_data_cccd_handle; /**< Handle of the CCCD of the NUS TX characteristic as provided by a discovery. */
        uint16_t fts_rx_cmd_handle;  /**< Handle of the NUS RX characteristic as provided by a discovery. */
        uint16_t fts_rx_data_handle;  /**< Handle of the NUS RX Data characteristic as provided by a discovery. */
        uint16_t fts_tx_cmd_handle;  /**< Handle of the NUS RX characteristic as provided by a discovery. */
        uint16_t fts_tx_cmd_cccd_handle;  /**< Handle of the NUS RX characteristic as provided by a discovery. */
} ) ble_fts_c_handles_t;

/**@brief Structure containing the NUS event data received from the peer. */
typedef PACKED ( struct
{
        ble_fts_c_evt_type_t evt_type;
        uint16_t conn_handle;
        uint16_t max_data_len;
        uint8_t * p_data;
        uint8_t data_len;
        ble_fts_c_handles_t handles;  /**< Handles on which the Nordic File Transfer service characteristics was discovered on the peer device. This will be filled if the evt_type is @ref BLE_NUS_C_EVT_DISCOVERY_COMPLETE.*/
} ) ble_fts_c_evt_t;

// Forward declaration of the ble_nus_t type.
typedef struct ble_fts_c_s ble_fts_c_t;

typedef void (* ble_fts_c_evt_handler_t)(ble_fts_c_t * p_ble_fts_c, ble_fts_c_evt_t const * p_evt);

/**@brief FTS Client structure. */
PACKED ( struct ble_fts_c_s
{
        uint8_t uuid_type;                  /**< UUID type. */
        uint16_t conn_handle;               /**< Handle of the current connection. Set with @ref ble_nus_c_handles_assign when connected. */
        ble_fts_c_handles_t handles;        /**< Handles on the connected peer device needed to interact with it. */
        ble_fts_c_evt_handler_t evt_handler; /**< Application event handler to be called when there is an event related to the NUS. */
});

/**@brief FTS Client initialization structure. */
typedef PACKED ( struct
{
        ble_fts_c_evt_handler_t evt_handler;
}) ble_fts_c_init_t;

uint32_t ble_fts_c_init(ble_fts_c_t * p_ble_fts_c, ble_fts_c_init_t * p_ble_fts_c_init);

void ble_fts_c_on_db_disc_evt(ble_fts_c_t * p_ble_fts_c, ble_db_discovery_evt_t * p_evt);

void ble_fts_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_fts_c_tx_data_notif_enable(ble_fts_c_t * p_ble_fts_c);

uint32_t ble_fts_c_tx_cmd_notif_enable(ble_fts_c_t * p_ble_fts_c);

uint32_t ble_fts_c_handles_assign(ble_fts_c_t *               p_ble_fts_c,
                                  uint16_t conn_handle,
                                  ble_fts_c_handles_t const * p_peer_handles);

uint32_t ble_fts_c_rx_data_send(ble_fts_c_t *p_ble_fts_c, uint8_t * p_string, uint16_t length);

// uint32_t ble_fts_c_rx_cmd_send(ble_fts_c_t * p_ble_fts_c, ble_fts_c_rx_cmd_t * img_info);

//uint32_t ble_fts_c_rx_cmd_send(ble_fts_c_t * p_ble_fts_c, ble_fts_c_cmd_data_t * cmd_data);
uint32_t ble_fts_c_rx_cmd_send(ble_fts_c_t * p_ble_fts_c, uint8_t * cmd_data, uint16_t length);

uint32_t ble_fts_c_rx_data_send_object(ble_fts_c_t *p_ble_fts_c, uint8_t *p_data, uint32_t data_length, uint8_t max_packet_length);

uint32_t ble_fts_c_rx_data_send_object_fragment(ble_fts_c_t *p_fts_c, uint8_t *p_data, uint32_t data_length);


#ifdef __cplusplus
}
#endif

#endif // BLE_FILE_TRANSFER_SERVICE_C_H__

/** @} */
