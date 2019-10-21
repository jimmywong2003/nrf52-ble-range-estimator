#ifndef CHANNEL_SURVEY_H__
#define CHANNEL_SURVEY_H__

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

uint32_t channel_survey_get_report_event(ble_gap_evt_qos_channel_survey_report_t *channel_survey_report);

uint32_t connection_channel_survey_start(void);

uint32_t connection_channel_survey_stop(void);

uint32_t channel_map_request_update(uint16_t conn_handle, uint8_t first_best_channel_number);

bool get_channel_map_status(void);

#ifdef __cplusplus
}
#endif

#endif
