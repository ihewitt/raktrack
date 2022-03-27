
#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"

#include "app_timer.h"
#include "stdint.h"

#include "ble_advdata.h"
#include "nrf.h"
#include "nrf_ble_scan.h"
#include "nrf_delay.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_sdh_ble.h"

#include "ant_channel_config.h"
#include "ant_interface.h"
#include "ant_key_manager.h"
#include "ant_search_config.h"
#include "ant_state_indicator.h"

#include "SEGGER_RTT.h"

#include <math.h>
#include <stdio.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ant.h"

#define ANTPLUS_NETWORK_NUM   0
#define APP_ANT_OBSERVER_PRIO 1

#define CHANNEL_CORE 0
#define CHANNEL_HRM  1

#define CORE_DEVICE_TYPE     0x7Fu   // 127
#define CORE_ANTPLUS_RF_FREQ 0x39u   // 57
#define CORE_MSG_PERIOD      0x4000u // 16384 // HR uses - 0x3F0Cu?

#define HRM_DEVICE_TYPE      0x78u   // 0x7Fu     // 127
#define HRM_ANTPLUS_RF_FREQ  0x39u   // 57
#define HRM_MSG_PERIOD       0x1F86u // 0x4000u // 16384
#define APP_BLE_CONN_CFG_TAG 1

ant_state_t m_ant_state;

// ANT message
// Just get basic data
// TODO add battery stats and quality
static void message_received_handler(uint8_t ant_channel, ant_evt_t *p_ant_evt)
{
    switch (p_ant_evt->message.ANT_MESSAGE_ucMesgID) {
        // Broadcast data received
        case MESG_BROADCAST_DATA_ID:
            if (ant_channel == CHANNEL_HRM) { // HRM
                {
                    uint8_t hrm = p_ant_evt->message.ANT_MESSAGE_aucMesgData[8];

                    if (hrm > 0) {
                        m_ant_state.hrm = hrm;
                        m_ant_state.data |= S_HR;
                    }
                }
            }
            if (ant_channel == CHANNEL_CORE) // CORE
            {
                int page = p_ant_evt->message.ANT_MESSAGE_aucMesgData[1];
                if (page == 1) {
                    uint16_t core =
                        p_ant_evt->message.ANT_MESSAGE_aucMesgData[7] +
                        p_ant_evt->message.ANT_MESSAGE_aucMesgData[8] * 256;
                    uint16_t skin =
                        p_ant_evt->message.ANT_MESSAGE_aucMesgData[4] +
                        ((p_ant_evt->message.ANT_MESSAGE_aucMesgData[5] & 0xf0)
                         << 4);

                    if (skin != 0x800) {
                        if (skin > 0) {
                            m_ant_state.skin = skin / 20.0;
                            m_ant_state.data |= S_SK;
                        }
                    }
                    if (core != 0x8000) {
                        if (core > 0) {
                            m_ant_state.core = core / 100.0;
                            m_ant_state.data |= S_CR;
                        }
                    }
                }
                else if (page == 0) {
                // int q =
                // p_ant_evt->message.ANT_MESSAGE_aucMesgData[3];
                // NRF_LOG_INFO("QU %d", q);
                }
            }
            break;

        default: break;
    }
}

// what should search handler actually do?
static void goto_search_handler(uint8_t channel)
{
    // search again?
    NRF_LOG_INFO("ANT search Fail %d", channel);
}

// ANT event
void ant_evt_handler(ant_evt_t *p_ant_evt, void *p_context)
{
    switch (p_ant_evt->event) {
        case EVENT_RX_FAIL_GO_TO_SEARCH: // No idea
            NRF_LOG_INFO("FAIL_SEARCH");
            goto_search_handler(p_ant_evt->channel);
            break;

        case EVENT_RX:
            message_received_handler(p_ant_evt->channel, p_ant_evt);
            break;

        default: break;
    }
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler,
                     NULL);

void ant_profile_setup(int hrm, int core)
{
    ret_code_t err_code;

    int channel = 0;

    for (int i = 0; i <= 1; ++i) {

        if (i == 0) channel = CHANNEL_HRM;
        if (i == 1) channel = CHANNEL_CORE;

        ant_channel_config_t broadcast_channel_config = {
            .channel_number = channel, // BROADCAST_CHANNEL_NUMBER,
            .channel_type = CHANNEL_TYPE_SLAVE,
            .ext_assign = 0x00,
            .rf_freq = (channel == CHANNEL_HRM) ? HRM_ANTPLUS_RF_FREQ
                                                : CORE_ANTPLUS_RF_FREQ,
            .transmission_type = 0, // CHAN_ID_TRANS_TYPE,
            .device_type = (channel == CHANNEL_HRM)
                               ? HRM_DEVICE_TYPE
                               : CORE_DEVICE_TYPE, // CHAN_ID_DEV_TYPE,
            .device_number = (channel == CHANNEL_HRM) ? hrm : core, //
            .channel_period =
                (channel == CHANNEL_HRM) ? HRM_MSG_PERIOD : CORE_MSG_PERIOD,
            .network_number = ANTPLUS_NETWORK_NUM, // ANTPLUS_NETWORK_NUM,
        };
#if 1
        ant_search_config_t ant_search_config =
            DEFAULT_ANT_SEARCH_CONFIG(channel);
        ant_search_config.high_priority_timeout =
            ANT_HIGH_PRIORITY_TIMEOUT_DISABLE;
#endif

        err_code = ant_channel_init(&broadcast_channel_config);
        APP_ERROR_CHECK(err_code);

#if 1
        ant_search_config.channel_number = channel;
        err_code = ant_search_init(&ant_search_config);
        APP_ERROR_CHECK(err_code);
#endif

        NRF_LOG_INFO("Open");

        // Open channel.
        err_code = sd_ant_channel_open(channel);
        APP_ERROR_CHECK(err_code);
    }
}


NRF_BLE_SCAN_DEF(m_scan); /**< Scanning module instance. */

void ble_scan_start(void)
{
    NRF_LOG_INFO("Starting scan.");
    APP_ERROR_CHECK(nrf_ble_scan_start(&m_scan));
}

void print_address(const ble_gap_evt_adv_report_t *p_adv_report)
{
    NRF_LOG_INFO("addr: %02x:%02x:%02x:%02x:%02x:%02x",
                 p_adv_report->peer_addr.addr[5],
                 p_adv_report->peer_addr.addr[4],
                 p_adv_report->peer_addr.addr[3],
                 p_adv_report->peer_addr.addr[2],
                 p_adv_report->peer_addr.addr[1],
                 p_adv_report->peer_addr.addr[0]);
}

void print_name(const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint16_t    offset = 0;
    static char name[64] = {0};

    uint16_t length = ble_advdata_search(p_adv_report->data.p_data,
                                         p_adv_report->data.len,
                                         &offset,
                                         BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);
    if (length == 0) {
        // Look for the short local name if it was not found as complete.
        length = ble_advdata_search(p_adv_report->data.p_data,
                                    p_adv_report->data.len,
                                    &offset,
                                    BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME);
    }

    if (length != 0) {
        memcpy(name, &p_adv_report->data.p_data[offset], length);
        NRF_LOG_INFO("name: %s", NRF_LOG_PUSH(name));
    }
}

void parse_manufacturer_data(const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint16_t offset = 0;
    uint16_t length =
        ble_advdata_search(p_adv_report->data.p_data,
                           p_adv_report->data.len,
                           &offset,
                           BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA);

    if (length > 12) {

        uint32_t epoch = 946684800 - 28800; // off by 8hrs
        uint8_t *data = &p_adv_report->data.p_data[offset] + 2;

        uint32_t time = epoch + *(uint32_t *)&data[2];
        int      age = data[1] / 6; // sample age mins

        int state = data[8];
        int phase = data[9];

        if (state == 0) {
            // fresh sensor
        }
        else if (state == 1) {
            // linking
        }
        else {
            int life = 300 * *(uint16_t *)&data[6]; // Age of sensor
            int span = 15 - life / (60 * 60 * 24);

            //  int   unk = data[11];        // unknown
            float glu = data[10] / 10.0; // gluc mmol

            if (phase == 4) { // countdown phase
                              //                int wt = data[10];
                // ("Warming " + (59 - wt) + " mins");
            }
            else if (phase == 0xc) { // Stabilising
                                     // "stabilizing";
            }
            else if (phase == 0x7) { // Running
                                     // "ok";
            }

            if (state != 2) {
                // error/event?
            }
            m_ant_state.gluc = glu;
            m_ant_state.data |= S_GL;

            NRF_LOG_INFO("Date: %d Days: %d Age: %d Glu: " NRF_LOG_FLOAT_MARKER
                         "",
                         time,
                         span,
                         age,
                         NRF_LOG_FLOAT(m_ant_state.gluc));
        }
    }
}

uint8_t m_aidex[6];

static void scan_evt_handler(scan_evt_t const *p_scan_evt)
{
    if (p_scan_evt->scan_evt_id == NRF_BLE_SCAN_EVT_SCAN_TIMEOUT) {
        NRF_LOG_INFO("Scan timed out.");
        ble_scan_start();
        return;
    }

    const ble_gap_evt_adv_report_t *p_adv_report =
        p_scan_evt->params.filter_match.p_adv_report;
    if (memcmp(p_adv_report->peer_addr.addr, m_aidex, 6) == 0) {

        print_address(p_scan_evt->params.filter_match.p_adv_report);
        print_name(p_scan_evt->params.filter_match.p_adv_report);

        parse_manufacturer_data(p_scan_evt->params.filter_match.p_adv_report);
    }
}

#define SCAN_DURATION_WITELIST 3000
static ble_gap_scan_params_t const m_scan_param = {
    // Whats the power difference?
    .active = 0x0, // active or inactive?
    .interval = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL, // BLE_GAP_SCAN_FP_WHITELIST,
    .timeout = SCAN_DURATION_WITELIST,
    .scan_phys = BLE_GAP_PHY_1MBPS,
};

void ble_scan_init(uint8_t *aidex)
{
    memset(&m_ant_state, 0, sizeof(ant_state_t));
    memcpy(m_aidex, aidex, 6);

    ret_code_t                 err_code;
    static nrf_ble_scan_init_t init_scan;
    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.p_scan_param = &m_scan_param;
    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
}
