#include "math.h"
#include "stdint.h"
#include "stdio.h"

#include "boards.h"

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_saadc.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_soc.h"
#include "nrfx_gpiote.h"
#include "nrfx_twim.h"

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "SEGGER_RTT.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_sdh_ble.h"

#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_util_platform.h"

#include "ant_key_manager.h"

#include "diskio_blkdev.h"
#include "ff.h"
#include "nrf_block_dev.h"
#include "nrf_block_dev_empty.h"
#include "nrf_block_dev_qspi.h"
#include "nrf_block_dev_ram.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_drv_usbd.h"

#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_msc.h"
#include "app_usbd_string_desc.h"

#include "ant.h"
#include "gps.h"
#include "oled.h"

#define APP_BLE_CONN_CFG_TAG 1
#define ANTPLUS_NETWORK_NUM  0

#define LED_PIN 12
#define CHG_DET (32 + 8)
#define BAT_SET (32 + 9)

#define BUT1 (32 + 1)
#define BUT2 (32 + 2)

/* Flash config */
/* onboard flash pins
1.15 -dio0
1.14 -dio1
1.13 -dio2
1.12 -dio3
1.11 -clk
1.10 -cs*/

static void msc_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                app_usbd_msc_user_event_t    event);

NRF_BLOCK_DEV_QSPI_DEFINE(
    m_block_dev_qspi,
    NRF_BLOCK_DEV_QSPI_CONFIG(512, NRF_BLOCK_DEV_QSPI_FLAG_CACHE_WRITEBACK,
                              NRF_DRV_QSPI_DEFAULT_CONFIG),
    NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "QSPI", "1.00"));

#define BLOCKDEV_LIST() (NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev))

#define ENDPOINT_LIST() APP_USBD_MSC_ENDPOINT_LIST(1, 1)

#define MSC_WORKBUFFER_SIZE (1024)
APP_USBD_MSC_GLOBAL_DEF(m_app_msc, 0, msc_user_ev_handler, ENDPOINT_LIST(),
                        BLOCKDEV_LIST(), MSC_WORKBUFFER_SIZE);

// global disable IO on USB
static bool  m_usb_connected = false;
static FATFS m_filesystem;

static bool fatfs_init(void)
{
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    memset(&m_filesystem, 0, sizeof(FATFS));

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] = {DISKIO_BLOCKDEV_CONFIG(
        NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev), NULL)};

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (QSPI)...");
    disk_state = disk_initialize(0);
    if (disk_state) {
        NRF_LOG_ERROR("Disk initialization failed.");
        return false;
    }

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&m_filesystem, "", 1);
    if (ff_result != FR_OK) {
        if (ff_result == FR_NO_FILESYSTEM) // make a fresh one if it's empty
        {
            static uint8_t buf[512];
            ff_result = f_mkfs("", FM_FAT, 1024, buf, sizeof(buf));
            if (ff_result != FR_OK) {
                NRF_LOG_ERROR("Mkfs failed.");
                return false;
            }
            ff_result = f_mount(&m_filesystem, "", 1); // remount
            if (ff_result != FR_OK) {
                NRF_LOG_ERROR("Mount failed.");
                return false;
            }
        }
        else {
            NRF_LOG_ERROR("Mount failed: %u", ff_result);
        }
        return false;
    }

    return true;
}

static void fatfs_uninit(void)
{
    NRF_LOG_INFO("Un-initializing disk 0 (QSPI)...");
    UNUSED_RETURN_VALUE(disk_uninitialize(0));
}

int file_open(FIL *file, char *filename)
{
    FRESULT ff_result;

    NRF_LOG_INFO("Writing to file %s ...", NRF_LOG_PUSH(filename));
    ff_result = f_open(file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK) {
        NRF_LOG_INFO("Unable to open or create file.");
        return -1;
    }
    return 0;
}

void file_close(FIL *file) { (void)f_close(file); }

int file_load(FIL *file, char *buffer, int len)
{
    FRESULT      ff_result;
    unsigned int bytes_read;
    ff_result = f_read(file, buffer, len, &bytes_read);
    if (ff_result == FR_OK)
        return bytes_read;
    else
        return -1;
}

// Trivial file append
void file_save(FIL *file, char *text)
{
    FRESULT  ff_result;
    uint32_t bytes_written;

    ff_result = f_write(file, text, strlen(text), (UINT *)&bytes_written);
    if (ff_result != FR_OK) { NRF_LOG_INFO("Write failed\r\n."); }
    else {
        NRF_LOG_INFO("%d bytes written.", bytes_written);
    }

    return;
}

static void msc_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                app_usbd_msc_user_event_t    event)
{
    UNUSED_PARAMETER(p_inst);
    UNUSED_PARAMETER(event);
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event) {
        case APP_USBD_EVT_DRV_SUSPEND: break;
        case APP_USBD_EVT_DRV_RESUME: break;
        case APP_USBD_EVT_STARTED: break;
        case APP_USBD_EVT_STOPPED:
            UNUSED_RETURN_VALUE(fatfs_init());
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");
            if (!nrf_drv_usbd_is_enabled()) {
                fatfs_uninit();
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            m_usb_connected = false;
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            m_usb_connected = true;
            break;
        default: break;
    }
}

// dev ANT+ network key
uint8_t key[8] = {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45};

static void softdevice_setup(void)
{
    NRF_LOG_INFO("sdh");

    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("enabled");

    ASSERT(nrf_sdh_is_enabled());

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    NRF_LOG_INFO("ble");
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("%04x", ram_start);
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    NRF_LOG_INFO("%04x", ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("ant");
    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("key");
    err_code = ant_custom_key_set(ANTPLUS_NETWORK_NUM, key);
    APP_ERROR_CHECK(err_code);
}

void sensors_init(const nrfx_twim_t *m_twi);

///////////////////////////////

APP_TIMER_DEF(m_gps_timer);
APP_TIMER_DEF(m_upl_timer);
APP_TIMER_DEF(m_scr_timer);
APP_TIMER_DEF(m_detection_delay_timer_id);

// Decide what makes sense
// Refresh screen each sec
#define SCR_INTERVAL APP_TIMER_TICKS(1 * 1000)
// read GPS every 10 secs? faster?
#define GPS_INTERVAL APP_TIMER_TICKS(10 * 1000)
// Upload each minute? slower?
#define UPL_INTERVAL    APP_TIMER_TICKS(60 * 1000)
#define SCREEN_OFF_TIME 60 // turn display off

// GPS state
extern nav_t m_gpsdata;
extern gsm_t m_gsmdata;

struct config_t {
    char server[16];
    int  port;

    int hrm;  // ant+ hrm
    int core; // ant+ coretemp

    uint8_t aidex[6]; // uid of aidex transmitter

    char apn[16];
    char user[16];
    char pwd[16];

} m_config =
//
//Hardcode our defaults, alternatively these can be written to "config.ini"
//see load_config() function for options
//
{.apn = "apnname",
              .user = "username",
              .pwd = "password",
              .server = "12.34.56.78",
              .port = 8080,
              .hrm = 12345, // Ant+ id for hrm
              .core = 24321, //Ant+ id for core
              .aidex = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06}};

// device state
static struct state_t {
    bool  run;    // running
    bool  gps_on; // live gps
    bool  chg;    // charging
    int   bat;
    float temp;
    float humid;
    char  volt[8];
} m_state;


extern ant_state_t m_ant_state;

static uint8_t tmp[16];

int  screen_page = 1; // control fields
void UpdateScreen()
{
    static int active = 0;
    if (OLED_state()) { // Dont bother if not turned on
        active++;
        if (active > SCREEN_OFF_TIME) { // Dont bother if timed out
            active = 0;
            OLED_off();
        }
        else {
            // read state and draw onto screen
            OLED_clear();

            // Layout icons
            if (m_state.gps_on) drawIcon(2, 0, ICON_GPS);    // gps 'A'
            if (m_gsmdata.dat_on) drawIcon(5, 0, ICON_DATA); // active
            if (m_gsmdata.mob_on)
                drawIcon(8, 0, ICON_SIGHI); // gsm TODO get strength

            if (m_usb_connected) drawIcon(0, 6, ICON_DISK); // usb mode

            if (m_state.chg)
                drawIcon(14, 6, ICON_BAT_CH); // Charging
            else if (m_state.bat > 80)
                drawIcon(14, 6, ICON_BAT_H);
            else if (m_state.bat > 50)
                drawIcon(14, 6, ICON_BAT_M);
            else
                drawIcon(14, 6, ICON_BAT_L);

            if (m_state.run)
                drawIcon(14, 2, ICON_PLAY);
            else
                drawIcon(14, 2, ICON_PAUSE);

            // Overlay text
            // Satellites
            snprintf(tmp, 16, "%2d", m_gpsdata.sats);
            drawString(0, 0, tmp);

            // Show time
            if (m_gpsdata.val == 'A') { // TODO change to use flag
                drawLetter(11, 0, m_gpsdata.time[0]);
                drawLetter(12, 0, m_gpsdata.time[1]);
                drawLetter(13, 0, ':');
                drawLetter(14, 0, m_gpsdata.time[2]);
                drawLetter(15, 0, m_gpsdata.time[3]);
            }

            // 23.7c#42%###123b
            if (screen_page == 0) {
                snprintf(tmp,
                         16,
                         "%2.1f\x81 %2.0f%% %d\x80",
                         m_state.temp,
                         m_state.humid,
                         m_ant_state.hrm);
                drawString(0, 2, tmp);

                snprintf(tmp,
                         16,
                         "%2.1f\x82 %2.1f\x81 %2.1f\x81",
                         m_ant_state.gluc,
                         m_ant_state.core,
                         m_ant_state.skin);
                drawString(0, 4, tmp);
            }
            else if (screen_page == 1) {
                snprintf(tmp,
                         16,
                         "  %d\x80  %2.1f\x81",
                         m_ant_state.hrm,
                         m_ant_state.core);
                drawString(0, 3, tmp);
            }
            else if (screen_page == 2) {
                snprintf(tmp,
                         16,
                         " %2.1f\x81  %2.0f%%",
                         m_state.temp,
                         m_state.humid);
                drawString(0, 3, tmp);
            }

            if (m_state.bat > 0) {
                snprintf(tmp, 16, "%d%%", m_state.bat);
                drawString(11, 6, tmp);
            }

            // ticker so we can see we're working
            static int ct;
            snprintf(tmp,
                     16,
                     "%d %c%c%c",
                     (ct++) & 0x7,
                     (m_ant_state.data & S_HR) ? '\x80' : ' ',
                     (m_ant_state.data & S_CR) ? '\x81' : ' ',
                     (m_ant_state.data & S_GL) ? '\x82' : ' ');
            drawString(2, 6, tmp);

            OLED_show(); // copies buffer onto LCD
        }
    }
}

void read_temperature(float *, float *);       // board temp and humd
void read_position(float *, float *, float *); // board gyro

char buffer[256]; // Current GPS string

// Every 10s collate and save the current data
void event_handler_gps(void *p_event_data, uint16_t event_size)
{
    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);

    // Kick off a   battery sample
    APP_ERROR_CHECK(nrf_drv_saadc_sample());
    // get temp
    read_temperature(&m_state.temp, &m_state.humid);

    /* Can't think of a use for this yet
        float x, y, z;
        read_position(&x, &y, &z);
        char tmp[32];
        sprintf(tmp, "%f %f %f", x, y, z);
        NRF_LOG_INFO("3d %s", NRF_LOG_PUSH(tmp));*/

    if (m_gpsdata.val == 'A')
        m_state.gps_on = true;
    else
        m_state.gps_on = false;

    // Cache what we will upoad
    sprintf(buffer,
            "*GPS,%s,"
            "%s%s,"
            "%f,%f,"                      // lat,lon
            "%c,%f,"                      // lock,speed
            "%f,,%d,"                     // alt,bat, TODO include accuracy
            "%d,"                         // hr
            "%2.1f,%2.1f,%2.1f,%2.1f#\n", // glu,core,tem,hum
            m_gsmdata.imei,               // id//TODO get sim imei
            m_gpsdata.date,
            m_gpsdata.time,
            m_gpsdata.latitude,
            m_gpsdata.longitude,
            m_gpsdata.val == 'A' ? 'A' : 'V', //[a|v]
            m_gpsdata.speed,                  // speed
            m_gpsdata.altitude,
            m_state.bat,
            m_ant_state.hrm,
            m_ant_state.gluc,
            m_ant_state.core,
            m_state.temp,
            m_state.humid);

    // clear data flags
    m_ant_state.data = 0;
    // add logic to clear out old expired data
    //     m_ant_state.hrm = 0;
    //     m_ant_state.core = 0;
    //     m_ant_state.gluc = 0;

    // Log the data
    if (m_state.gps_on && m_state.run && m_usb_connected == false) {
        FIL file;
        file_open(&file, "gpslog.csv");
        file_save(&file, buffer);
        file_close(&file);
    }
}

static void timer_handler_gps(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    app_sched_event_put(NULL, 0, event_handler_gps);
}

static char upload[1024 * 6]; // TODO rework to cachefile

// Each minute buffer current data, slow to upload every 5 mins?
void event_handler_upload(void *p_event_data, uint16_t event_size)
{
    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);
    // static int count;
    if (m_state.run) { // tracking on
        if (strlen(buffer) + strlen(upload) > 1024) upload[0] = 0; // bin it

        strcat(upload, buffer);
        // upload buffer
        /*if (count++ > 5)*/ //slow down?
        {
            NRF_LOG_INFO("Attempt upload dat %d state %d",
                         m_gsmdata.dat_on,
                         m_state.run);
            if (m_gsmdata.dat_on) // data on
            {
                NRF_LOG_INFO("Try upload buffer %d", strlen(upload));
                if (UploadData(upload, m_config.server, m_config.port) == 0) {
                    upload[0] = 0;
                }
            }
            // upload event
            // count = 0; // 5 min upload
        }
    }
}

static void timer_handler_upload(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    app_sched_event_put(NULL, 0, event_handler_upload);
}

// battery, appears to be:
// 4308 min 7100 max ?
// charge - 6313 to 7260


int pc_table[] = {}; //?

static nrf_saadc_value_t sample_buffer[3];

// TODO calibrate this
void process_power(int val)
{
    NRF_LOG_INFO("Raw %d", val);

    // LiPo bounds
    int pc = (val - 4400) * 100 / (7100 - 4400); // is this good enough?
    // Convert to %, Store state

    m_state.bat = pc;
    //strcpy(m_state.volt, tmp); //calibrate and calculate voltage

    int chg_det = nrf_gpio_pin_read(CHG_DET);
    m_state.chg = (chg_det == 0);
}


void event_handler_screen(void *p_event_data, uint16_t event_size)
{
    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);

    UpdateScreen(); // redraw screen
}

// 1 second timer
static void timer_handler_screen(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    app_sched_event_put(NULL, 0, event_handler_screen);
}

void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        int val = 0;
        // read samples
        for (int i = 0; i < 3; i++) {
            val = val + sample_buffer[i];
        }
        // reset sample buffer
        APP_ERROR_CHECK(nrf_drv_saadc_buffer_convert(sample_buffer, 3));

        val = val / 3; // average
        process_power(val);
    }
}

// ADC for battery
void saadc_init(void)
{
    nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
    saadc_config.interrupt_priority = 7;

    APP_ERROR_CHECK(nrf_drv_saadc_init(&saadc_config, saadc_callback));

    nrf_saadc_channel_config_t channel_config_0 =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

    APP_ERROR_CHECK(nrf_drv_saadc_channel_init(0, &channel_config_0));
    APP_ERROR_CHECK(nrf_drv_saadc_buffer_convert(sample_buffer, 3));
}

static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(1);
void twi_event_handler(nrfx_twim_evt_t const *p_event, void *p_context);
void twi_init(void)
{
    ret_code_t err_code;

    const nrfx_twim_config_t twi_config = {.scl = NRF_GPIO_PIN_MAP(0, 13),
                                           .sda = NRF_GPIO_PIN_MAP(0, 14),
                                           .frequency = NRF_TWIM_FREQ_400K,
                                           .interrupt_priority =
                                               APP_IRQ_PRIORITY_MID,
                                           .hold_bus_uninit = false

    };

    err_code = nrfx_twim_init(&m_twi, &twi_config, twi_event_handler, NULL);
    nrf_delay_ms(100);
    APP_ERROR_CHECK(err_code);
    nrfx_twim_enable(&m_twi);
}

void screen_event(void *p_event_data, uint16_t event_size)
{
    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);
    OLED_on();
    UpdateScreen();
}

void PowerDown()
{
    OLED_off();
    fatfs_uninit();
    nrf_gpio_cfg_sense_input(BUT1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    sd_power_system_off();

    while (1) {
        if (!NRF_LOG_PROCESS()) { nrf_pwr_mgmt_run(); }
    }
}

// Play button
void button1(int held)
{
    bool but2 = !nrfx_gpiote_in_is_set(BUT2);

    if (held > 15) {
        if (but2) { PowerDown(); }

        m_state.run = !m_state.run; // toggle state
        app_sched_event_put(NULL, 0, screen_event);
    }
}

// screen button turn on. or change page.
void button2(int held)
{
    if (held > 5) {         // 0.25s
        if (OLED_state()) { // if on, cycle page.
            screen_page++;
            if (screen_page > 2) screen_page = 0;

            app_sched_event_put(NULL, 0, screen_event);
        }
        else { // turn on
            app_sched_event_put(NULL, 0, screen_event);
        }
    }
}

static int  pressed;
static int  held;
static void detection_delay_timeout_handler(void *p_context)
{
    bool set = !nrfx_gpiote_in_is_set(pressed);
    if (set) // still held down count.
    {
        held++;
        app_timer_start(m_detection_delay_timer_id, APP_TIMER_TICKS(50), NULL);
    }
}

// Trivial button handler
void in_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    bool set = !nrfx_gpiote_in_is_set(pin);

    // start timer.
    if (set) {
        pressed = pin;
        held = 0;
        app_timer_start(m_detection_delay_timer_id, APP_TIMER_TICKS(50), NULL);
    }
    else {
        if (held) // basic debounce
        {
            if (pressed == BUT1)
                button1(held);
            else if (pressed == BUT2)
                button2(held);
        }
    }
}

// Can't use scheduler for timers since we're using timer for timeouts
void setup_timers()
{
    // Initialize timer module
    APP_ERROR_CHECK(app_timer_create(
        &m_gps_timer, APP_TIMER_MODE_REPEATED, timer_handler_gps));

    APP_ERROR_CHECK(app_timer_create(
        &m_upl_timer, APP_TIMER_MODE_REPEATED, timer_handler_upload));

    APP_ERROR_CHECK(app_timer_create(
        &m_scr_timer, APP_TIMER_MODE_REPEATED, timer_handler_screen));

    APP_ERROR_CHECK(app_timer_create(&m_detection_delay_timer_id,
                                     APP_TIMER_MODE_SINGLE_SHOT,
                                     detection_delay_timeout_handler));

    // Start application timers
    APP_ERROR_CHECK(app_timer_start(m_gps_timer, GPS_INTERVAL, NULL));

    APP_ERROR_CHECK(app_timer_start(m_upl_timer, UPL_INTERVAL, NULL));
}

void board_init()
{
    APP_ERROR_CHECK(nrfx_gpiote_init());

    nrfx_gpiote_in_config_t in_config =
        NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    APP_ERROR_CHECK(nrfx_gpiote_in_init(BUT1, &in_config, in_pin_handler));
    APP_ERROR_CHECK(nrfx_gpiote_in_init(BUT2, &in_config, in_pin_handler));

    nrfx_gpiote_in_event_enable(BUT1, true);
    nrfx_gpiote_in_event_enable(BUT2, true);

    nrf_gpio_cfg_output(LED_PIN); // LED

    nrf_gpio_cfg_input(BAT_SET, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(CHG_DET, NRF_GPIO_PIN_PULLUP);

    saadc_init(); // adc for battery
    twi_init();   // twi for sensors and screen
}

void dump_version()
{
    static char tmp[256];
    //    info = NRF_FICR->INFO.VARIANT;
    char *p = (uint8_t *)&NRF_FICR->INFO.VARIANT;
    sprintf(tmp,
            "Nordic Semiconductor nRF%x Variant: %c%c%c%c",
            (int)NRF_FICR->INFO.PART,
            p[3],
            p[2],
            p[1],
            p[0]);
    NRF_LOG_INFO(":%s", NRF_LOG_PUSH(tmp));
    sprintf(tmp,
            "RAM: %dKB Flash: %dKB",
            (int)NRF_FICR->INFO.RAM,
            (int)NRF_FICR->INFO.FLASH);
    NRF_LOG_INFO(":%s", NRF_LOG_PUSH(tmp));
    sprintf(tmp,
            "Device ID: %x%x",
            (int)NRF_FICR->DEVICEID[0],
            (int)NRF_FICR->DEVICEID[1]);
    NRF_LOG_INFO(":%s", NRF_LOG_PUSH(tmp));
}

// can be called from handlers so avoid any log/scheduler
void idle_state_handle(void)
{
    //    NRF_LOG_PROCESS();
    //    app_sched_execute();
    nrf_pwr_mgmt_run();
}

// TODO set correct size, pin or timer.
#define SCHED_MAX_EVENT_DATA_SIZE                                              \
    MAX(sizeof(nrfx_gpiote_pin_t), APP_TIMER_SCHED_EVENT_DATA_SIZE)

#define SCHED_QUEUE_SIZE                                                       \
    10 // experiment with this, should be ok now we're flushing during GSMWait

static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/* softdevice_setup
 * timer0
 * rtc0
 * swi1 if no radio notif
 * swi2
 */
// swi0 swi3 ok
// swi0 <- app timer

void utils_setup()
{
    APP_ERROR_CHECK(app_timer_init());
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());
}

void showStatus(char *str)
{
    drawString(0, 2, str);
    OLED_show();
    NRF_LOG_INFO("%s", NRF_LOG_PUSH(str));
}

// Check reset is defined correctly and fixup if not
void fixReset()
{
    if (NRF_UICR->PSELRESET[0] != 18) {
        int      i;
        uint32_t tmp[32];
        for (i = 0; i < 32; i++)
            tmp[i] = NRF_UICR->CUSTOMER[i];

        NRF_LOG_INFO("Reset pin not configured, programming... ");
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        NRF_NVMC->ERASEUICR = 1;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
            ;
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;

        NRF_UICR->PSELRESET[0] = 18;
        NRF_UICR->PSELRESET[1] = 18;

        /* Restore customer area */
        for (i = 0; i < 32; i++)
            NRF_UICR->CUSTOMER[i] = tmp[i];

        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
            ;
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        NRF_LOG_INFO("done.\n");
    }
}

// TODO rework this mess
uint8_t *GetPair(uint8_t **key, uint8_t **val, uint8_t *line)
{
    uint8_t *eol = strchr(line, '\n');
    uint8_t *sep = strchr(line, ':');
    if (eol) *eol = 0;
    if (sep) {
        *sep = 0;    // terminate
        *key = line; // key is start
        while (**key == ' ')
            *key = *key + 1; // skip any lead
        *val = sep + 1;
        while (**val == ' ')
            *val = *val + 1; // skip space
    }
    //    if (eol) eol++; //will either be start of next line or null
    return eol;
}

/* read config.ini text file from flash
format:
 key:value  e.g.

apn:hologram
server:123.456.789.1
port:8080
hrm:12345
*/
void load_config()
{
    // Basic. not robust. or efficient.
    char buffer[1024];
    FIL  config;

    if (file_open(&config, "config.ini") == 0) {
        int read = file_load(&config, buffer, 1024);
        if (read > 0) {

            char *line = buffer;
            do {
                uint8_t *key;
                uint8_t *val;
                line = GetPair(&key, &val, line); // get key value
                if (!line || !key) break;

                if (strcmp(key, "apn") == 0)
                    strncpy(m_config.apn, val, 16);
                else if (strcmp(key, "apnuser") == 0)
                    strncpy(m_config.user, val, 16);
                else if (strcmp(key, "apnpwd") == 0)
                    strncpy(m_config.pwd, val, 16);
                else if (strcmp(key, "server") == 0)
                    strncpy(m_config.server, val, 128);
                else if (strcmp(key, "port") == 0)
                    m_config.port = strtol(val, 0, 0);
                else if (strcmp(key, "hrm") == 0)
                    m_config.hrm = strtol(val, 0, 0);
                else if (strcmp(key, "core") == 0)
                    m_config.core = strtol(val, 0, 0);
                else if (strcmp(key, "aidex") == 0) {
                    // read hex and fill uid
                }
            } while (line++);
        }
        file_close(&config);

        NRF_LOG_INFO("Config read server:%s, port%d",
                     NRF_LOG_PUSH(m_config.server),
                     m_config.port);
    }
    else {
        NRF_LOG_INFO("Unable load load config");
    }
}

int main(void)
{
    // Let HW settle?
    nrf_delay_ms(2000); // slow down startup

    // Initialise
    static const app_usbd_config_t usbd_config = {.ev_state_proc =
                                                      usbd_user_ev_handler};

    ret_code_t ret;

    // Log init
    APP_ERROR_CHECK(NRF_LOG_INIT(app_usbd_sof_timestamp_get));
    NRF_LOG_DEFAULT_BACKENDS_INIT(); // Start logging

    fixReset(); // Make sure reset btn config is correct

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE,
                   SCHED_QUEUE_SIZE); // Prepare scheduler

    utils_setup();

    // must be before usbd init
    lfclk_request();

    // must be before softdevice to activate drv_power
    APP_ERROR_CHECK(app_usbd_init(&usbd_config));

    NRF_LOG_INFO("Board init");
    board_init(); // I2C, GSM, GPS, ANT, BLE

    softdevice_setup(); // move above board init?

    NRF_LOG_INFO("\r\nTracker started.");
    dump_version();

    // Add logic to check if USB connected on turn on.
    fatfs_init();
    load_config();

    app_usbd_class_inst_t const *class_inst_msc =
        app_usbd_msc_class_inst_get(&m_app_msc);

    ret = app_usbd_class_append(class_inst_msc);
    APP_ERROR_CHECK(ret);

    ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);

    NRF_LOG_INFO("Sensors init");
    sensors_init(&m_twi);

    NRF_LOG_INFO("Start OLED");
    OLED_init(&m_twi);
    OLED_on();

    showStatus("Booting");
    GPS_Pins(); // GSM/GPS pins
    GPS_On();   // Power up GPS

    showStatus("GPS_Init");
    GPS_Init(); // GSM uart

    showStatus("BLE/ANT init");
    ble_scan_init(m_config.aidex);                  // ble init
    ant_profile_setup(m_config.hrm, m_config.core); // Kick off ant
    ble_scan_start();                               // ble scan

    showStatus("Start timers");
    setup_timers(); // Start timers

    // TODO do these asynchronously TODO
    showStatus("GPS/GSM init");
    if (GPS_Config() != 0) // Kick off GPS
    {
        showStatus("GPS config fail");
        while (1)
            nrf_pwr_mgmt_run(); // halt
    }
    showStatus("GSM Register");

    if (GSM_Register(m_config.apn, m_config.user, m_config.pwd) != 0) {
        showStatus("Register fail");
        while (1)
            nrf_pwr_mgmt_run(); // halt
    }

    // Main activity loop
    NRF_LOG_INFO("\r\nIdle Loop\r\n");

    // Start screen refresh running
    APP_ERROR_CHECK(app_timer_start(m_scr_timer, SCR_INTERVAL, NULL));

    m_state.run = false; // Start disabled

    while (true) {

        GSM_events();

        while (app_usbd_event_queue_process()) { /* Nothing to do */
        }

        // Process events
        app_sched_execute();

        if (!NRF_LOG_PROCESS()) { nrf_pwr_mgmt_run(); }
    }
}
