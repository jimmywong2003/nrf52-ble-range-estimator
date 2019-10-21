#ifndef PACKET_ERROR_RATE_H__
#define PACKET_ERROR_RATE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
#include "nrf_sdh_ble.h"

#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NRF_TIMER_TX_READY      NRF_TIMER1
#define NRF_TIMER_RX_CRCOK      NRF_TIMER2

#define TIMER_RELOAD_CC_NUM 0
#define TIMER_RELOAD  0

#define GPIO_PER_PIN_FOR_TX_READY   28 // This pin will toggle once when radio has TXed or RXed the ADDRESS part of a PDU.
#define GPIO_PER_PIN_FOR_RX_CRCOK   29 //This pin will toggle once when radio has detected a CRC error.

#define GPIOTE_CHANNEL_TX_READY   5  // GPIO channel for the task to toggle GPIO_PIN_FOR_RX_READY.
#define GPIOTE_CHANNEL_RX_CRCOK   6  // GPIO channel for the task to toggle GPIO_PIN_FOR_RX_CRCOK.

#define PPI_CHANNEL_FOR_TX_READY_GPIO_EVT  5  // PPI Channel for connecting the ADDRESS event to GPIOTE_CHANNEL_ADDRESS.
#define PPI_CHANNEL_FOR_RX_CRCOK_GPIO_EVT  6  // PPI Channel for connecting the CRCERROR event to GPIOTE_CHANNEL_CRCERROR.
#define PPI_CHANNEL_FOR_TX_READY_TIMER_EVT 7  // PPI Channel for connecting the ADDRESS event to GPIOTE_CHANNEL_ADDRESS.
#define PPI_CHANNEL_FOR_RX_CRCOK_TIMER_EVT 8  // PPI Channel for connecting the CRCERROR event to GPIOTE_CHANNEL_CRCERROR.


typedef struct packet_error_s
{
        uint32_t radio_packet_success_rate;
        uint32_t radio_packet_ready;
        uint32_t radio_packet_crcok;
} packet_error_t;



void packet_error_rate_detect_enable(void);

void packet_error_rate_detect_disable(void);

void packet_error_rate_reset_counter(void);

uint32_t packet_error_rate_timeout_handler(void);

uint32_t get_packet_success_rate(void);

void get_accumlated_packet_success_rate(packet_error_t *per);

#ifdef __cplusplus
}
#endif


#endif
