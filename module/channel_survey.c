#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "nrf_ble_gatt.h"
#include "app_scheduler.h"

#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_default_backends.h"

#define BLE_GAP_QOS_CHANNEL_SURVERY_INTERVAL      0
#define NUM_REPORTS_BEFORE_AVERAGE_PRINTOUT       100

#define ADVERTISING_CHANNEL_37      37
#define ADVERTISING_CHANNEL_38      38
#define ADVERTISING_CHANNEL_39      39

typedef struct
{
        uint8_t index;
        bool channel_enable;
        float ch_energy;
} channel_energy_t;

static uint32_t m_num_channel_survey_reports_received;
static channel_energy_t ch_config_energy[BLE_GAP_CHANNEL_COUNT];
static channel_energy_t m_average_ch_energy[BLE_GAP_CHANNEL_COUNT];
static bool update_channel_survey_status = false;

bool get_channel_map_status(void)
{
        return update_channel_survey_status;
}

uint32_t connection_channel_survey_start(void)
{
        ret_code_t err_code = sd_ble_gap_qos_channel_survey_start(BLE_GAP_QOS_CHANNEL_SURVERY_INTERVAL);
        APP_ERROR_CHECK(err_code);
        return err_code;
}

uint32_t connection_channel_survey_stop(void)
{
        ret_code_t err_code = sd_ble_gap_qos_channel_survey_stop();
        APP_ERROR_CHECK(err_code);
        return err_code;
}

static void sort_channel_survey(void)
{
        int n, c, d;
        float t;
        uint8_t index = 0;
        n = BLE_GAP_CHANNEL_COUNT;
        for (c = 1; c <= n - 1; c++)
        {
                d = c;
                while ( d > 0 && m_average_ch_energy[d-1].ch_energy > m_average_ch_energy[d].ch_energy)
                {
                        t          = m_average_ch_energy[d].ch_energy;
                        index      = m_average_ch_energy[d].index;
                        m_average_ch_energy[d].ch_energy   = m_average_ch_energy[d-1].ch_energy;
                        m_average_ch_energy[d].index       = m_average_ch_energy[d-1].index;
                        m_average_ch_energy[d-1].ch_energy = t;
                        m_average_ch_energy[d-1].index     = index;
                        d--;
                }
        }
        return;
}

void update_and_sort_channel_survey_handler(void * p_event_data, uint16_t size)
{
        NRF_LOG_INFO("Channel energy report:\n--------------\n");
        for (uint8_t i = 0; i < BLE_GAP_CHANNEL_COUNT; i++)
        {
                int8_t channel_energy = m_average_ch_energy[i].ch_energy;
                m_average_ch_energy[i].index = i;
        }
        NRF_LOG_DEBUG("00-04: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[0].ch_energy, \
                      (int8_t)m_average_ch_energy[1].ch_energy, \
                      (int8_t)m_average_ch_energy[2].ch_energy, \
                      (int8_t)m_average_ch_energy[3].ch_energy, \
                      (int8_t)m_average_ch_energy[4].ch_energy  \
                      );
        NRF_LOG_DEBUG("05-09: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[5].ch_energy, \
                      (int8_t)m_average_ch_energy[6].ch_energy, \
                      (int8_t)m_average_ch_energy[7].ch_energy, \
                      (int8_t)m_average_ch_energy[8].ch_energy, \
                      (int8_t)m_average_ch_energy[9].ch_energy  \
                      );
        NRF_LOG_DEBUG("10-14: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[10].ch_energy, \
                      (int8_t)m_average_ch_energy[11].ch_energy, \
                      (int8_t)m_average_ch_energy[12].ch_energy, \
                      (int8_t)m_average_ch_energy[13].ch_energy, \
                      (int8_t)m_average_ch_energy[14].ch_energy  \
                      );
        NRF_LOG_DEBUG("15-19: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[15].ch_energy, \
                      (int8_t)m_average_ch_energy[16].ch_energy, \
                      (int8_t)m_average_ch_energy[17].ch_energy, \
                      (int8_t)m_average_ch_energy[18].ch_energy, \
                      (int8_t)m_average_ch_energy[19].ch_energy  \
                      );
        NRF_LOG_DEBUG("20-24: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[20].ch_energy, \
                      (int8_t)m_average_ch_energy[21].ch_energy, \
                      (int8_t)m_average_ch_energy[22].ch_energy, \
                      (int8_t)m_average_ch_energy[23].ch_energy, \
                      (int8_t)m_average_ch_energy[24].ch_energy  \
                      );
        NRF_LOG_DEBUG("25-29: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[25].ch_energy, \
                      (int8_t)m_average_ch_energy[26].ch_energy, \
                      (int8_t)m_average_ch_energy[27].ch_energy, \
                      (int8_t)m_average_ch_energy[28].ch_energy, \
                      (int8_t)m_average_ch_energy[29].ch_energy  \
                      );

        NRF_LOG_DEBUG("30-34: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[30].ch_energy, \
                      (int8_t)m_average_ch_energy[31].ch_energy, \
                      (int8_t)m_average_ch_energy[32].ch_energy, \
                      (int8_t)m_average_ch_energy[33].ch_energy, \
                      (int8_t)m_average_ch_energy[34].ch_energy  \
                      );
        NRF_LOG_DEBUG("35-39: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[35].ch_energy, \
                      (int8_t)m_average_ch_energy[36].ch_energy, \
                      (int8_t)m_average_ch_energy[37].ch_energy, \
                      (int8_t)m_average_ch_energy[38].ch_energy, \
                      (int8_t)m_average_ch_energy[39].ch_energy  \
                      );

        sort_channel_survey();

        NRF_LOG_DEBUG("Top Best Clean Channel Index:");

        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[0].index,  \
                      m_average_ch_energy[1].index,  \
                      m_average_ch_energy[2].index,  \
                      m_average_ch_energy[3].index,  \
                      m_average_ch_energy[4].index   \
                      );

        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[5].index,  \
                      m_average_ch_energy[6].index,  \
                      m_average_ch_energy[7].index,  \
                      m_average_ch_energy[8].index,  \
                      m_average_ch_energy[9].index   \
                      );

        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[10].index,  \
                      m_average_ch_energy[11].index,  \
                      m_average_ch_energy[12].index,  \
                      m_average_ch_energy[13].index,  \
                      m_average_ch_energy[14].index   \
                      );

        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[15].index,  \
                      m_average_ch_energy[16].index,  \
                      m_average_ch_energy[17].index,  \
                      m_average_ch_energy[18].index,  \
                      m_average_ch_energy[19].index   \
                      );
        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[20].index,  \
                      m_average_ch_energy[21].index,  \
                      m_average_ch_energy[22].index,  \
                      m_average_ch_energy[23].index,  \
                      m_average_ch_energy[24].index   \
                      );
        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[25].index,  \
                      m_average_ch_energy[26].index,  \
                      m_average_ch_energy[27].index,  \
                      m_average_ch_energy[28].index,  \
                      m_average_ch_energy[29].index   \
                      );
        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[30].index,  \
                      m_average_ch_energy[31].index,  \
                      m_average_ch_energy[32].index,  \
                      m_average_ch_energy[33].index,  \
                      m_average_ch_energy[34].index   \
                      );
        NRF_LOG_DEBUG("%02d %02d",  \
                      m_average_ch_energy[35].index,  \
                      m_average_ch_energy[36].index
                      );

        update_channel_survey_status = true;

}



uint32_t channel_map_request_update(uint16_t conn_handle, uint8_t first_best_channel_number)
{
        ret_code_t err_code;
        uint8_t number_channel_request = 0;
        ble_gap_opt_ch_map_t channel_map = {0};

        if (conn_handle == BLE_CONN_HANDLE_INVALID)
        {
                NRF_LOG_ERROR("Failure: because of disconnection!");
                return -1;
        }

        NRF_LOG_DEBUG("channel_map_request_update!!");

        channel_map.conn_handle = conn_handle;  //we get conn_handle on a CONNECT event

        for (uint8_t i=0; i < BLE_GAP_CHANNEL_COUNT; i++)
        {
                uint8_t freq_index = m_average_ch_energy[i].index;

                if (freq_index != ADVERTISING_CHANNEL_37 && freq_index != ADVERTISING_CHANNEL_38 && freq_index != ADVERTISING_CHANNEL_39)
                {
                        if (freq_index < 8)
                        {
                                channel_map.ch_map[0] |= 1 << (freq_index);
                                //NRF_LOG_INFO("< 8, %d %x", freq_index, 1 << (freq_index));
                        }
                        else if (freq_index < 16)
                        {
                                channel_map.ch_map[1] |= 1 << (freq_index-8);
                                // NRF_LOG_INFO("< 16, %d %x", freq_index, 1 << (freq_index - 8));
                        }
                        else if (freq_index < 24)
                        {
                                channel_map.ch_map[2] |= 1 << (freq_index-16);
                                // NRF_LOG_INFO("< 24, %d %x", freq_index, 1 << (freq_index - 16));
                        }
                        else if (freq_index < 32)
                        {
                                channel_map.ch_map[3] |= 1 << (freq_index-24);
                                // NRF_LOG_INFO("< 32, %d %x", freq_index, 1 << (freq_index - 24));
                        }
                        else
                        {
                                channel_map.ch_map[4] |= 1 << (freq_index-32);
                                // NRF_LOG_INFO("%d %x", freq_index, 1 << (freq_index - 32));
                        }
                        number_channel_request++;
                }
                if (number_channel_request > first_best_channel_number)
                {
                        break;
                }
        }
        NRF_LOG_HEXDUMP_DEBUG(channel_map.ch_map, 5);

        err_code = sd_ble_opt_set(BLE_GAP_OPT_CH_MAP, (ble_opt_t *)&channel_map);
        APP_ERROR_CHECK(err_code);

        update_channel_survey_status = false;
}

static void process_channel_survey_report(ble_gap_evt_qos_channel_survey_report_t * p_report)
{
        for (uint8_t i = 0; i < BLE_GAP_CHANNEL_COUNT; i++)
        {
                float energy_sample = p_report->channel_energy[i];

                if (energy_sample == BLE_GAP_POWER_LEVEL_INVALID)
                {
                        continue;
                }

                if (m_num_channel_survey_reports_received == 0)
                {
                        m_average_ch_energy[i].ch_energy = energy_sample;
                }
                else
                {
                        m_average_ch_energy[i].ch_energy =  (m_num_channel_survey_reports_received * m_average_ch_energy[i].ch_energy + energy_sample)
                                                           / (m_num_channel_survey_reports_received + 1);
                }
        }
}

uint32_t channel_survey_get_report_event(ble_gap_evt_qos_channel_survey_report_t *channel_survey_report)
{
        uint32_t err_code;

        process_channel_survey_report(channel_survey_report);
        m_num_channel_survey_reports_received++;

        if (m_num_channel_survey_reports_received > NUM_REPORTS_BEFORE_AVERAGE_PRINTOUT)
        {
                err_code = sd_ble_gap_qos_channel_survey_stop();
                APP_ERROR_CHECK(err_code);

                //NRF_LOG_INFO("Stop Channel Survey received count %d", m_num_channel_survey_reports_received);

                err_code = app_sched_event_put(NULL, 0, update_and_sort_channel_survey_handler);
                APP_ERROR_CHECK(err_code);

                m_num_channel_survey_reports_received = 0;
        }

}
