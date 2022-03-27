#include "boards.h"

#include "stdint.h"

#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrfx_uarte.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "app_scheduler.h"
#include "nrf_atomic.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "gps.h"

#define CMD_TIMEOUT (500) // ms

// PIN_BG96_DTR 26

#define GPS_TXD_PIN     6  // 8
#define GPS_RXD_PIN     8  // 9
#define GPS_CTS_PIN     11 // 8
#define GPS_RTS_PIN     7  // 9
#define GSM_PWR_ON_PIN  39 // 6
#define GSM_RESET_PIN   28 // 14
#define GSM_PWRKEY_PIN  2  // 15
#define GSM_DISABLE_PIN 29 // airplane mode
#define GSM_STATUS      31

// RTC0-2
// TIMER0-4
// TIMER1 - apptimer?

// RTC0 - softdevice
// RTC1 - app-timer2
// RTC2 - free

// gsm uart
#define UART0_TMR      2
#define UART0_RTC_TOUT 2 // NRF_LIBUARTE_PERIPHERAL_NOT_USED // 2
#define UART0_TMR_TOUT NRF_LIBUARTE_PERIPHERAL_NOT_USED

// gps uart
#define UART1_TMR      3
#define UART1_RTC_TOUT NRF_LIBUARTE_PERIPHERAL_NOT_USED //
#define UART1_TMR_TOUT NRF_LIBUARTE_PERIPHERAL_NOT_USED // 4

nrfx_uarte_t uartgsm = NRFX_UARTE_INSTANCE(0);
nrfx_uarte_t uartgps = NRFX_UARTE_INSTANCE(1);

nav_t m_gpsdata;
gsm_t m_gsmdata;

char m_apn[16];
char m_user[16];
char m_pwd[16];

/*
$GPGGA,160603.00,5114.999239,N,00000.173018,E,1,10,0.8,141.4,M,47.0,M,,*68
$GPRMC,160603.00,A,5114.999239,N,00000.173018,E,0.0'
*/

float GpsToDecimalDegrees(const char *nmeaPos, char quadrant)
{
    float v = 0;
    if (strlen(nmeaPos) > 5) {
        char integerPart[3 + 1];
        int  digitCount = (nmeaPos[4] == '.' ? 2 : 3);
        memcpy(integerPart, nmeaPos, digitCount);
        integerPart[digitCount] = 0;
        nmeaPos += digitCount;
        v = atoi(integerPart) + atof(nmeaPos) / 60.;
        if (quadrant == 'W' || quadrant == 'S') v = -v;
    }
    return v;
}

int parse_comma_delimited_str(char *string, char **fields, int max_fields)
{
    int i = 0;
    fields[i++] = string;
    while ((i < max_fields) && NULL != (string = strchr(string, ','))) {
        *string = '\0';
        fields[i++] = ++string;
    }
    return --i;
}

// Parse UART NMEA feed
void GPS_Parse(char *ptr)
{
    char *field[20];
    char  temp[256];
    strncpy(temp, ptr, 255);
    temp[255] = 0;
    //    NRF_LOG_INFO("%s", NRF_LOG_PUSH(temp));
    char *e = 0, *p = temp;
    while (*p && p < &temp[255]) {
        if ((e = strchr(p, '\n')) || (e = (p + strlen(p)))) *e = '\0';

        int flds = parse_comma_delimited_str(p, field, 20);

        if (flds > 9) {
            if (strcmp(field[0], "$GPGGA") == 0) {
                if (field[1]) strncpy(m_gpsdata.time, field[1], 6);
                if (field[2])
                    m_gpsdata.latitude =
                        GpsToDecimalDegrees(field[2], *field[3]);
                if (field[4])
                    m_gpsdata.longitude =
                        GpsToDecimalDegrees(field[4], *field[5]);
                m_gpsdata.altitude = atof(field[9]);
                m_gpsdata.sats = atoi(field[7]);
                m_gpsdata.fix = *field[6];
                // precision field[8]
            }
            else if (strcmp(field[0], "$GPRMC") == 0) {
                if (field[1]) strncpy(m_gpsdata.time, field[1], 6);
                m_gpsdata.val = *field[2];
                if (field[3])
                    m_gpsdata.latitude =
                        GpsToDecimalDegrees(field[3], *field[4]); // ignore?
                if (field[5])
                    m_gpsdata.longitude =
                        GpsToDecimalDegrees(field[5], *field[6]);
                if (field[9] && strlen(field[9]) >= 6) {
                    // ddmmyy to yymmdd
                    m_gpsdata.date[0] = field[9][4];
                    m_gpsdata.date[1] = field[9][5];
                    m_gpsdata.date[2] = field[9][2];
                    m_gpsdata.date[3] = field[9][3];
                    m_gpsdata.date[4] = field[9][0];
                    m_gpsdata.date[5] = field[9][1];
                };
                if (field[7]) m_gpsdata.speed = atof(field[7]);
            }
        }
        p = e + 1;
    }
}

// GPS debug uart3 feed
static char gps_buf[1024];
static int  gps_ch = 0;
static char gps_byte;

void gps_event_handler(nrfx_uarte_event_t const *p_evt, void *context)
{
    // Only ever receive on the gps uart
    switch (p_evt->type) {
        case NRFX_UARTE_EVT_RX_DONE:
            gps_buf[gps_ch] = gps_byte;
            nrfx_uarte_rx(&uartgps, &gps_byte, 1); // does this need nudging?

            if (gps_buf[gps_ch] == '\n' || gps_buf[gps_ch] == 0) {
                gps_buf[gps_ch] = 0;
                GPS_Parse(gps_buf);
                gps_ch = 0;
            }
            else {
                gps_ch++;
                if (gps_ch > 1024) gps_ch = 0;
            }

            break;
        case NRFX_UARTE_EVT_TX_DONE: break;
        case NRFX_UARTE_EVT_ERROR:
            nrfx_uarte_rx(&uartgps, &gps_byte, 1); // Retry on error
            break;
    }
}


// TODO change flags
volatile bool sending = false;

static int  gsm_ch = 0;
static char gsm_buf[1024];
static char gsm_byte;

volatile bool received = false;
volatile bool sent = false;

#define K_ACTIVATED_MSK    (1U << 1)
#define STR_ACT            "+QIACT: 1,1,1"
#define K_DEACTIVATED_MSK  (1U << 2)
#define STR_DEACT          "+QIURC: \"pdpdeact\""
#define K_REGISTERED_MSK   (1U << 3)
#define STR_REG            "+CREG: 1"
#define K_DEREGISTERED_MSK (1U << 4)
#define STR_DEREG          "+CREG: 0"
#define K_SENTDATA_MSK     (1U << 5)

static nrf_atomic_u32_t m_events;

static char expected[64] = {0}; // TODO rework
static char GSM_RSP[1600] = {0};

// TODO catch all URC and set flags
static void gsm_event_handler(nrfx_uarte_event_t const *p_evt, void *context)
{
    switch (p_evt->type) {
        case NRFX_UARTE_EVT_RX_DONE:           
            gsm_buf[gsm_ch] = gsm_byte;
            gsm_buf[gsm_ch + 1] = 0;
            nrfx_uarte_rx(&uartgsm, &gsm_byte, 1);

            // No newline on send marker so use special check
            if ((*expected == '>') && (gsm_buf[gsm_ch] == '>')) {
                UNUSED_RETURN_VALUE(
                    nrf_atomic_u32_or(&m_events, K_SENTDATA_MSK));
                *expected = 0;
                received = true;
                gsm_ch = 0;
            }
            else if (gsm_buf[gsm_ch] == '\r' || gsm_buf[gsm_ch] == '\n' ||
                     gsm_buf[gsm_ch] == 0) {
                
                gsm_buf[gsm_ch] = 0; //terminate

                if (strlen(gsm_buf)) {
                    int pt;
                    if (!received)
                        strcpy(GSM_RSP, gsm_buf); // always save response

                    // TODO split expected and received flags?
                    if (expected[0]) {
                        pt = strncmp(gsm_buf, expected, strlen(expected));
                        if (pt == 0 && !received) {
                            *expected = 0;
                            received = true; // flag got what we expected
                        }
                    }
                    else
                        received = true;

                    // Deactivated
                    if (0 ==
                        strncmp(gsm_buf, STR_DEACT, sizeof(STR_DEACT) - 1)) {
                        UNUSED_RETURN_VALUE(
                            nrf_atomic_u32_or(&m_events, K_DEACTIVATED_MSK));
                    }
                    else if (0 ==
                             strncmp(gsm_buf, STR_ACT, sizeof(STR_ACT) - 1)) {
                        UNUSED_RETURN_VALUE(
                            nrf_atomic_u32_or(&m_events, K_ACTIVATED_MSK));
                    }
                    else if (0 ==
                             strncmp(gsm_buf, STR_DEREG, sizeof(STR_DEREG))) {
                        UNUSED_RETURN_VALUE(
                            nrf_atomic_u32_or(&m_events, K_DEREGISTERED_MSK));
                    }
                    else if (0 ==
                             strncmp(gsm_buf, STR_REG, sizeof(STR_REG) - 1)) {
                        UNUSED_RETURN_VALUE(
                            nrf_atomic_u32_or(&m_events, K_REGISTERED_MSK));
                    }
                }
                gsm_ch = 0;
                gsm_buf[gsm_ch] = 0;
            }
            else {
                gsm_ch++;
                if (gsm_ch > 1024) gsm_ch = 0;
            }
            break;

        case NRFX_UARTE_EVT_TX_DONE: sent = true; break;
        case NRFX_UARTE_EVT_ERROR:
            nrfx_uarte_rx(&uartgsm, &gsm_byte, 1); // Retry on error
            break;
    }
}

/// Initialise
void GPS_Pins()
{
    nrf_gpio_cfg_input(GSM_STATUS, NRF_GPIO_PIN_PULLUP);

    nrf_gpio_cfg_output(GSM_PWR_ON_PIN);
    nrf_gpio_cfg_output(GSM_RESET_PIN);
    nrf_gpio_cfg_output(GSM_PWRKEY_PIN);
    nrf_gpio_cfg_output(GSM_DISABLE_PIN);
}

void idle_state_handle(void);

volatile bool timedout = false;

APP_TIMER_DEF(m_timeout_timer_id);

static void timeout_timer_handler(void *p_context)
{
    // clear flag
    NRF_LOG_INFO("timedout!");
    //app_timer_stop(m_timeout_timer_id);
    timedout = true;
}

static void create_timers()
{
    NRF_LOG_INFO("Create timers");
    ret_code_t err_code;
    err_code = app_timer_create( // APP_TIMER_MODE_SINGLE_SHOT
        &m_timeout_timer_id,
        APP_TIMER_MODE_SINGLE_SHOT, // APP_TIMER_MODE_REPEATED,
        timeout_timer_handler);
    APP_ERROR_CHECK(err_code);
}
void GPS_Init()
{
    //    int err_code;
    memset(&m_gpsdata, 0, sizeof(nav_t));
    memset(&m_gsmdata, 0, sizeof(gsm_t));

    // Timeout handler
    create_timers();

#if 1
    nrfx_uarte_config_t nrf_gps_config = //
        {.pseltxd = 20,
         .pselrxd = 19,
         .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
         .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
         .p_context = 0,
         .baudrate = NRF_UARTE_BAUDRATE_115200,
         .hwfc = NRF_UARTE_HWFC_DISABLED,
         .interrupt_priority = APP_IRQ_PRIORITY_HIGH};

    nrfx_uarte_init(&uartgps, &nrf_gps_config, gps_event_handler);
    nrfx_uarte_rx(&uartgps, &gps_byte, 1);
#endif

    nrfx_uarte_config_t nrf_gsm_config = //
        {.pseltxd = GPS_TXD_PIN,
         .pselrxd = GPS_RXD_PIN,
         .pselcts = GPS_CTS_PIN,
         .pselrts = GPS_RTS_PIN,
         .p_context = 0,
         .baudrate = NRF_UARTE_BAUDRATE_115200,
         .hwfc = NRF_UARTE_HWFC_ENABLED,
         .interrupt_priority = APP_IRQ_PRIORITY_HIGH};

    nrfx_uarte_init(&uartgsm, &nrf_gsm_config, gsm_event_handler);
    nrfx_uarte_rx(&uartgsm, &gsm_byte, 1);

}

// Contains delays. make sure softdevice is off.
void GPS_On()
{
    NRF_LOG_INFO("GSM_PowerUp");

    uint32_t status;

    status = nrf_gpio_pin_read(GSM_STATUS);
    NRF_LOG_INFO("State: powered off (%d)", status);

    nrf_gpio_pin_write(GSM_DISABLE_PIN, 0); // activate
    nrf_gpio_pin_write(GSM_PWR_ON_PIN, 1);  // antenna power
    nrf_gpio_pin_write(GSM_PWRKEY_PIN, 0);
    nrf_gpio_pin_write(GSM_RESET_PIN, 0);
    nrf_delay_ms(100);
    nrf_gpio_pin_write(GSM_PWRKEY_PIN, 1);
    nrf_delay_ms(600);
    nrf_gpio_pin_write(GSM_PWRKEY_PIN, 0);

    NRF_LOG_INFO("GSM wait");

    for (int wait = 1000; !status && wait; wait--) {
        status = nrf_gpio_pin_read(GSM_STATUS);
        nrf_delay_ms(1);
    };
    nrf_delay_ms(600);
}

// TODO clean timeouts
// Takes string buffer, terminates and sends
int GSM_Sendl(uint8_t *buffer, int len)
{
    sent = false;

    while (nrfx_uarte_tx_in_progress(&uartgsm))
        ;
    nrfx_err_t err_code = nrfx_uarte_tx(&uartgsm, buffer, len);
    APP_ERROR_CHECK(err_code);

    // separate send timeoout?
    while (!sent) // Make sure we're done?
        idle_state_handle();

    return err_code;
}

// Short string send
int GSM_Send(char *str, bool term)
{
    char work[64];
    if (term) {
        strcpy(work, str);
        strcat(work, "\r\n"); // TODO move this
        str = work;
    }

    int len = strlen(str);
    int ret = GSM_Sendl(str, len);
    return ret;
}

// response buffer
// TODO get result codes for errors!
int GSM_Wait(char *cmd, bool term, char *rsp_value, char *exp, int timeout_ms)
{
    static bool wait = false;
    if (wait) return -1; // TODO redo with proper lock

    wait = true;
    received = false;
    timedout = false;

    int err_code = 0;
    if (exp)
        strcpy(expected, exp); // string to wait for, tidy this up/make smarter
    else
        expected[0] = 0;

    if (cmd && term) {
        NRF_LOG_INFO("GSM_Wait [%s] (%dms)", NRF_LOG_PUSH(cmd), timeout_ms);
    }
    else {
        NRF_LOG_INFO("GSM_Wait (%dms)", timeout_ms);
    }

    app_timer_start(m_timeout_timer_id, APP_TIMER_TICKS(timeout_ms), NULL);

    // send
    if (cmd) GSM_Send(cmd, term);

    while (!received && !timedout) {
        app_sched_execute(); // May be slow, keep scheduler running?,
                             // but if so so block gsm events?
        idle_state_handle(); // keep sd running
    }

    if (exp && !received) {
        NRF_LOG_INFO("Expected '%s' failed, got '%s'",
                     NRF_LOG_PUSH(exp),
                     NRF_LOG_PUSH(GSM_RSP));
    }
    else if (!received)
        NRF_LOG_INFO("Not received");

    if (!received && timedout) err_code = -1;
    if (!timedout) app_timer_stop(m_timeout_timer_id);

    if (rsp_value) *rsp_value = 0;
    if (received && rsp_value) { strcpy(rsp_value, GSM_RSP); }
    wait = false;

    return err_code;
}

int GSM_WaitOK(char *cmd, char *rsp_value, int timeout_ms)
{
    // Use flags to check for OK and ERROR instead
    return GSM_Wait(cmd, true, rsp_value, "OK", timeout_ms);
}

// TODO Strip this back
int GPS_Config()
{
    int retval = -1;

    // If we've got a send open, kill it.
    GSM_WaitOK("\x1a", NULL, CMD_TIMEOUT);
    GSM_WaitOK("\x1b", NULL, CMD_TIMEOUT);

    // Make sure we're 'OK' now
    int retry = 5;
    do {
        retval = GSM_WaitOK("AT", NULL, CMD_TIMEOUT);
    } while (retval && retry);
    if (retval) return retval;

    retval = GSM_WaitOK("ATE0", NULL, CMD_TIMEOUT); // Echo off
    if (retval) { return retval; }
    retval = GSM_WaitOK("AT+QGPSEND", NULL, CMD_TIMEOUT);
    // Fail is ok, might not be active

    retval = GSM_WaitOK("AT+QGPSCFG=\"gpsnmeatype\",3", NULL, CMD_TIMEOUT);
    if (retval) { return retval; }
    retval = GSM_WaitOK("AT+QGPSCFG=\"gnssconfig\",3", NULL, CMD_TIMEOUT);
    if (retval) { return retval; }
    retval =
        GSM_WaitOK("AT+QGPSCFG=\"outport\",\"auxnmea\"", NULL, CMD_TIMEOUT);
    if (retval) { return retval; }
    retval = GSM_WaitOK("AT+QURCCFG=\"urcport\",\"uart1\"", NULL, CMD_TIMEOUT);
    if (retval) { return retval; }

    // Actually only effective after reboot. but just set anyway.
    retval = GSM_WaitOK("AT+QGPSCFG=\"autogps\",1", NULL, CMD_TIMEOUT);
    if (retval) { return retval; }

    retval = GSM_WaitOK("AT+QGPS=1", NULL, CMD_TIMEOUT);
    if (retval) { return retval; }

    return retval;
}

// TODO cleanup
int GSM_CheckSimCmd(void)
{
    int         retval = -1;
    static char cmd[64];

    strcpy(m_gsmdata.imei, "PROTOTYPE"); // TODO read from sim

    // Check SIM
    retval = GSM_Wait("AT+CPIN?", true, cmd, "+CPIN", 5 * 1000);

    if (retval >= 0) {
        if (NULL != strstr(cmd, "READY")) { retval = 0; }
        else {
            retval = -1;
        }
    }
    NRF_LOG_INFO("returned %s %d", NRF_LOG_PUSH(cmd), retval);

    return retval;
}
// TODO sort timeouts
// work out how to make async
int GSM_Register(char *apn, char *user, char *pwd)
{
    int retry;

    // store apn details for register
    strcpy(m_apn, apn);
    strcpy(m_user, user);
    strcpy(m_pwd, pwd);

    for (retry = 4; (GSM_CheckSimCmd() != 0) && (retry > 0); --retry)
        ; // Only continue if we have a SIM

    if (retry == 0) return -1;

    GSM_WaitOK("ATH", NULL, CMD_TIMEOUT); // Hangup

    GSM_WaitOK("AT+CMEE=1", NULL, CMD_TIMEOUT); // Numeric errors
    GSM_WaitOK("AT+CREG=1", NULL, CMD_TIMEOUT); // Registrations
    GSM_WaitOK("AT+CREG?", NULL, CMD_TIMEOUT);  // Check reg status

    return 0;
}

int GSM_CloseSocketCmd(void)
{
    int retval = -1;

    static char cmd[64];
    retval = GSM_WaitOK("AT+QICLOSE=0", cmd, 20*1000); //check max. do we get ok immediately?
    return retval;
}

// Open socket
int GSM_OpenTCPSocket(char *ip, uint16_t DestPort)
{
    int         retval = -1; //
    int         sck = 0;
    static char cmd[64];
    sprintf(
        cmd, "AT+QIOPEN=1,%d,\"TCP\",\"%s\",%d,0,0", sck, ip, (int)DestPort);

    static char rsp[64];
    rsp[0] = 0;
    retval = GSM_Wait((uint8_t *)cmd,
                      true,
                      rsp,
                      "+QIOPEN: 0,",
                      30 * 1000); // Check max time we can use, need multiplier?

    if (retval == 0) {
        NRF_LOG_INFO("Open return %d", retval);
        if (strstr(rsp, "0,0")) // new
            retval = 0;
        else if (strstr(rsp, "0,563")) // still open?
            retval = 0;
        else
            retval = -1;
    }
    else {
        NRF_LOG_INFO("Open return %d", retval);
        GSM_CloseSocketCmd();
    }

    return retval;
}

int GSM_SendDataCmd(char *data, uint16_t len)
{
    int         retval = -1;
    static char cmd[64];
    int         sck = 0;

    sprintf(cmd, "AT+QISEND=%d", sck);
    retval = GSM_Wait(cmd, true, NULL, ">", 5 * 1000); // TODO move to use flags for this

    // block everything until send concluded
    /*if (retval >= 0)*/ {
        NRF_LOG_INFO("GSM_SEND_DATA");

        // TODO rework this send terminate
        uint8_t junk[1024];
        memcpy(junk, data, len);
        junk[len] = 0x1a;
        junk[len + 1] = 0;

        // GSM_Sendl((uint8_t *)junk, len + 1);

        retval = GSM_Wait(junk,
                          false,
                          cmd,
                          "SEND ",
                          20 * 1000); // 10 * 1000); // whats a sane timeout

        if (retval >= 0) { // TODO fix return
            if (strstr(cmd, "SEND OK")) {
                NRF_LOG_INFO("Sent");
                retval = 0;
            }
            else {
                NRF_LOG_INFO("Not sent");
                retval = -1;
            }
        }
    }
    return retval;
}

// TODO Add re connection logic
int UploadData(char *msg, char *server, int port)
{
    NRF_LOG_INFO("UploadData");
    static bool upload = false;
    if (upload) return -1; // TODO redo with lock
    upload = true;

    uint32_t ret = GSM_OpenTCPSocket(server, port); // TODO move to parameters

    if (ret == 0) { // Send data string
        ret = GSM_SendDataCmd(msg, strlen(msg));
        GSM_WaitOK("AT", NULL, CMD_TIMEOUT); //Check ACK
        GSM_CloseSocketCmd(); //This is slow
        NRF_LOG_INFO("Closed");
    }

    upload = false;
    return ret;
}

void GSM_events()
{
    if (!m_gsmdata.dat_on && !m_gsmdata.mob_on) {
        //Test this, work out if it's necessary
        // NRF_LOG_INFO("No conn. ask for re-reg?");
        // GSM_WaitOK("AT+CREG?", NULL, CMD_TIMEOUT);
    }

    uint32_t events = nrf_atomic_u32_fetch_store(&m_events, 0);
    if (events & K_DEACTIVATED_MSK) {
        NRF_LOG_INFO("Deactivated");
        m_gsmdata.dat_on = false;
        GSM_WaitOK("AT+QIACT=1", NULL, 5 * 1000);   // reactivate
        GSM_WaitOK("AT+QIACT?", NULL, CMD_TIMEOUT); // check activate?
    }

    if (events & K_DEREGISTERED_MSK) {
        NRF_LOG_INFO("Deregistered");
        // how to re-register??
        GSM_WaitOK("AT+CREG?", NULL, CMD_TIMEOUT);
        m_gsmdata.mob_on = false;
    }

    if (events & K_ACTIVATED_MSK) {
        NRF_LOG_INFO("Activated");
        m_gsmdata.dat_on = true;
    }

    if (events & K_REGISTERED_MSK) {
        NRF_LOG_INFO("Registered");
        m_gsmdata.mob_on = true;
        // on registration define APN
        char cmd[64];
        sprintf(
            cmd, "AT+QICSGP=1,1,\"%s\",\"%s\",\"%s\",1", m_apn, m_user, m_pwd);
        GSM_WaitOK(cmd, NULL, CMD_TIMEOUT);
        GSM_WaitOK("AT+QIACT=1", NULL, 5 * 1000);   // check activate time
        GSM_WaitOK("AT+QIACT?", NULL, CMD_TIMEOUT); // check activate
    }
}
