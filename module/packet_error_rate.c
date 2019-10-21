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
#include "nrf_pwr_mgmt.h"

#include "packet_error_rate.h"

#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_default_backends.h"

static uint32_t u32_radio_packet_ready_per_interval = 0;
static uint32_t u32_radio_packet_crcok_per_interval = 0;

static packet_error_t m_accumlated_radio_packet;

static uint32_t radio_packet_success_rate = 0;
static uint32_t radio_packet_ready = 0;
static uint32_t radio_packet_crcok = 0;

void timer_1_init()
{
        NRF_TIMER_TX_READY->TASKS_CLEAR = 1;
        NRF_TIMER_TX_READY->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
        NRF_TIMER_TX_READY->PRESCALER = 4;
        NRF_TIMER_TX_READY->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
        NRF_TIMER_TX_READY->MODE = TIMER_MODE_MODE_LowPowerCounter << TIMER_MODE_MODE_Pos;
        NRF_TIMER_TX_READY->CC[TIMER_RELOAD_CC_NUM] = TIMER_RELOAD;
        NRF_TIMER_TX_READY->TASKS_START = 1;
}

void timer_2_init()
{
        NRF_TIMER_RX_CRCOK->TASKS_CLEAR = 1;
        NRF_TIMER_RX_CRCOK->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
        NRF_TIMER_RX_CRCOK->PRESCALER = 4;
        NRF_TIMER_RX_CRCOK->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
        NRF_TIMER_RX_CRCOK->MODE = TIMER_MODE_MODE_LowPowerCounter << TIMER_MODE_MODE_Pos;
        NRF_TIMER_RX_CRCOK->CC[TIMER_RELOAD_CC_NUM] = TIMER_RELOAD;
        NRF_TIMER_RX_CRCOK->TASKS_START = 1;
}

void packet_error_rate_reset_counter(void)
{
        memset(&m_accumlated_radio_packet, 0, sizeof(m_accumlated_radio_packet));
        radio_packet_success_rate = 0;
        radio_packet_ready = 0;
        radio_packet_crcok = 0;
}

void packet_error_rate_detect_enable(void)
{
        timer_1_init();
        timer_2_init();

        /* Configure the GPIO pins as output. */
        NRF_GPIO->DIRSET       = ( 1UL << GPIO_PER_PIN_FOR_TX_READY       |
                                   1UL << GPIO_PER_PIN_FOR_RX_CRCOK );

        /* Reset, set, and reset the pins once. This step useful for
         * checking using logic analyzer if the pin configurations work on init. */
        NRF_GPIO->OUTCLR       = ( 1UL << GPIO_PER_PIN_FOR_TX_READY       |
                                   1UL << GPIO_PER_PIN_FOR_RX_CRCOK );

        NRF_GPIO->OUTSET       = ( 1UL << GPIO_PER_PIN_FOR_TX_READY       |
                                   1UL << GPIO_PER_PIN_FOR_RX_CRCOK );

        NRF_GPIO->OUTCLR       = ( 1UL << GPIO_PER_PIN_FOR_TX_READY       |
                                   1UL << GPIO_PER_PIN_FOR_RX_CRCOK );


        /* Assign one GPIOTE channel for each GPIO pin. Configure the channel in task mode so the polarity
         * of the connected GPIO pin is toggled when tasked. */

        NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_TX_READY] =
                (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
                (GPIO_PER_PIN_FOR_TX_READY << GPIOTE_CONFIG_PSEL_Pos)              |
                (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos);

        NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_RX_CRCOK] =
                (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
                (GPIO_PER_PIN_FOR_RX_CRCOK << GPIOTE_CONFIG_PSEL_Pos)             |
                (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos);

#if defined (NRF52840_XXAA)
        /* Connect NRF_RADIO->EVENTS_ADDRESS event via a PPI channel to the GPIOTE task corresponding to
         * GPIO_PIN_FOR_RX_READY. This will make the pin toggle on EVENTS_ADDRESS event.*/
        NRF_PPI->CH[PPI_CHANNEL_FOR_TX_READY_GPIO_EVT].TEP = (uint32_t)(&NRF_GPIOTE->TASKS_OUT[GPIOTE_CHANNEL_TX_READY]);
        NRF_PPI->CH[PPI_CHANNEL_FOR_TX_READY_GPIO_EVT].EEP = (uint32_t)(&NRF_RADIO->EVENTS_TXREADY);//EVENTS_TXREADY);//EVENTS_PAYLOAD);//EVENTS_ADDRESS);

        /* Connect NRF_RADIO->EVENTS_CRCERROR event via a PPI channel to the GPIOTE task corresponding to
         * GPIO_PIN_FOR_RX_CRCOK. This will make the pin toggle on EVENTS_CRCERROR event.*/
        NRF_PPI->CH[PPI_CHANNEL_FOR_RX_CRCOK_GPIO_EVT].TEP = (uint32_t)(&NRF_GPIOTE->TASKS_OUT[GPIOTE_CHANNEL_RX_CRCOK]);
        NRF_PPI->CH[PPI_CHANNEL_FOR_RX_CRCOK_GPIO_EVT].EEP = (uint32_t)(&NRF_RADIO->EVENTS_CRCOK);

        NRF_PPI->CH[PPI_CHANNEL_FOR_TX_READY_TIMER_EVT].TEP = (uint32_t)(&NRF_TIMER_TX_READY->TASKS_COUNT);//GPIOTE->TASKS_OUT[GPIOTE_CHANNEL_RX_READY]);
        NRF_PPI->CH[PPI_CHANNEL_FOR_TX_READY_TIMER_EVT].EEP = (uint32_t)(&NRF_RADIO->EVENTS_TXREADY);//CRCOK);//EVENTS_PAYLOAD);//EVENTS_ADDRESS);

        NRF_PPI->CH[PPI_CHANNEL_FOR_RX_CRCOK_TIMER_EVT].TEP = (uint32_t)(&NRF_TIMER_RX_CRCOK->TASKS_COUNT);
        NRF_PPI->CH[PPI_CHANNEL_FOR_RX_CRCOK_TIMER_EVT].EEP = (uint32_t)(&NRF_RADIO->EVENTS_CRCOK);//PAYLOAD);//CRCERROR);

#else
        /* Connect NRF_RADIO->EVENTS_ADDRESS event via a PPI channel to the GPIOTE task corresponding to
         * GPIO_PIN_FOR_RX_READY. This will make the pin toggle on EVENTS_ADDRESS event.*/
        NRF_PPI->CH[PPI_CHANNEL_FOR_TX_READY_GPIO_EVT].TEP = (uint32_t)(&NRF_GPIOTE->TASKS_OUT[GPIOTE_CHANNEL_TX_READY]);
        NRF_PPI->CH[PPI_CHANNEL_FOR_TX_READY_GPIO_EVT].EEP = (uint32_t)(&NRF_RADIO->EVENTS_READY);//EVENTS_TXREADY);//EVENTS_PAYLOAD);//EVENTS_ADDRESS);

        /* Connect NRF_RADIO->EVENTS_CRCERROR event via a PPI channel to the GPIOTE task corresponding to
         * GPIO_PIN_FOR_RX_CRCOK. This will make the pin toggle on EVENTS_CRCERROR event.*/
        NRF_PPI->CH[PPI_CHANNEL_FOR_RX_CRCOK_GPIO_EVT].TEP = (uint32_t)(&NRF_GPIOTE->TASKS_OUT[GPIOTE_CHANNEL_RX_CRCOK]);
        NRF_PPI->CH[PPI_CHANNEL_FOR_RX_CRCOK_GPIO_EVT].EEP = (uint32_t)(&NRF_RADIO->EVENTS_CRCOK);

        NRF_PPI->CH[PPI_CHANNEL_FOR_TX_READY_TIMER_EVT].TEP = (uint32_t)(&NRF_TIMER_TX_READY->TASKS_COUNT);//GPIOTE->TASKS_OUT[GPIOTE_CHANNEL_RX_READY]);
        NRF_PPI->CH[PPI_CHANNEL_FOR_TX_READY_TIMER_EVT].EEP = (uint32_t)(&NRF_RADIO->EVENTS_READY);//CRCOK);//EVENTS_PAYLOAD);//EVENTS_ADDRESS);

        NRF_PPI->CH[PPI_CHANNEL_FOR_RX_CRCOK_TIMER_EVT].TEP = (uint32_t)(&NRF_TIMER_RX_CRCOK->TASKS_COUNT);
        NRF_PPI->CH[PPI_CHANNEL_FOR_RX_CRCOK_TIMER_EVT].EEP = (uint32_t)(&NRF_RADIO->EVENTS_CRCOK);//PAYLOAD);//CRCERROR);
#endif 
        /* Enable the PPI channels configured above. */
        NRF_PPI->CHENSET =  (1 << PPI_CHANNEL_FOR_TX_READY_GPIO_EVT)  |
                           (1 << PPI_CHANNEL_FOR_RX_CRCOK_GPIO_EVT) |
                           (1 << PPI_CHANNEL_FOR_TX_READY_TIMER_EVT) |
                           (1 << PPI_CHANNEL_FOR_RX_CRCOK_TIMER_EVT);
}

void packet_error_rate_detect_disable(void)
{
        /* Disable the PPI channels configured above. */
        NRF_PPI->CHENCLR =  (1 << PPI_CHANNEL_FOR_TX_READY_GPIO_EVT)  |
                           (1 << PPI_CHANNEL_FOR_RX_CRCOK_GPIO_EVT) |
                           (1 << PPI_CHANNEL_FOR_TX_READY_TIMER_EVT) |
                           (1 << PPI_CHANNEL_FOR_RX_CRCOK_TIMER_EVT);

        NRF_TIMER_TX_READY->TASKS_CLEAR = 1;
        NRF_TIMER_RX_CRCOK->TASKS_CLEAR = 1;

        NRF_TIMER_TX_READY->TASKS_STOP = 1;
        NRF_TIMER_RX_CRCOK->TASKS_STOP = 1;

        u32_radio_packet_ready_per_interval = 0;
        u32_radio_packet_crcok_per_interval = 0;
}

uint32_t packet_error_rate_timeout_handler(void)
{
        //UNUSED_PARAMETER(p_context);
        NRF_TIMER_TX_READY->TASKS_CAPTURE[0] = 1;
        NRF_TIMER_RX_CRCOK->TASKS_CAPTURE[0] = 1;

#if defined (NRF52840_XXAA)
        radio_packet_ready = NRF_TIMER_TX_READY->CC[0] - u32_radio_packet_ready_per_interval;
#else
        radio_packet_ready = NRF_TIMER_TX_READY->CC[0]/2 - u32_radio_packet_ready_per_interval;
#endif
        radio_packet_crcok = NRF_TIMER_RX_CRCOK->CC[0] - u32_radio_packet_crcok_per_interval;

        radio_packet_success_rate = (radio_packet_crcok * 100)/(radio_packet_ready);

#if defined (NRF52840_XXAA)
        u32_radio_packet_ready_per_interval = NRF_TIMER_TX_READY->CC[0];
#else
        u32_radio_packet_ready_per_interval = NRF_TIMER_TX_READY->CC[0]/2;
#endif
        u32_radio_packet_crcok_per_interval = NRF_TIMER_RX_CRCOK->CC[0];

        //printf("%02d\n", radio_packet_success_rate);
        NRF_LOG_DEBUG("PER Timeout %d, %d", radio_packet_ready, radio_packet_crcok);
        
        //NRF_LOG_INFO("Packet Success Rate %02d %%\n", radio_packet_success_rate);

        // NRF_TIMER_TX_READY->TASKS_CLEAR = 1;
        // NRF_TIMER_RX_CRCOK->TASKS_CLEAR = 1;

        //NRF_LOG_INFO( "Float " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(radio_packet_success_rate));

        m_accumlated_radio_packet.radio_packet_crcok += radio_packet_crcok;
        m_accumlated_radio_packet.radio_packet_ready += radio_packet_ready;

        NRF_LOG_DEBUG("PER accumulated %d, %d", m_accumlated_radio_packet.radio_packet_ready, m_accumlated_radio_packet.radio_packet_crcok);

        m_accumlated_radio_packet.radio_packet_success_rate = (m_accumlated_radio_packet.radio_packet_crcok * 100)/(m_accumlated_radio_packet.radio_packet_ready);

        if (radio_packet_ready > 0xFFFF0000)
        {
                u32_radio_packet_ready_per_interval = 0;
                u32_radio_packet_crcok_per_interval = 0;
                packet_error_rate_detect_disable();
                packet_error_rate_detect_enable();
        }
        return radio_packet_success_rate;
}

uint32_t get_packet_success_rate(void)
{
        return radio_packet_success_rate;
}

void get_accumlated_packet_success_rate(packet_error_t *per)
{
      *per = m_accumlated_radio_packet;
}

