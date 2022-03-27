#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"

#include "stdint.h"

#include "nrf.h"
// remove delays since we're using softdevice
#include "nrf_delay.h"

#include "SEGGER_RTT.h"
#include "app_timer.h"

#include "nrfx_twim.h"
#include "nrf_fprintf.h"
#include <math.h>
#include <stdio.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SHT3C_ADDR  (0x70U)
#define LIS3DH_ADDR (0x19U)

#define I2C_TIMEOUT 10000

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
// internal
static const nrfx_twim_t *m_twi=0;
/**
 * @brief TWI events handler.
 */
static bool twi_tx_done;
static bool twi_rx_done;

void TxReady() { twi_tx_done=false;}
bool TxDone(){ return twi_tx_done;}

void idle_state_handle(void);

// do we need timeout logic?
//#define I2C_TIMEOUT 10000

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t *buf)
{
    double humid = *(uint16_t *)&buf[0] / 655.36;
    double temp = 175 * (*(uint16_t *)&buf[3]) / 65536 - 45;

    NRF_LOG_INFO("Temp: %02g, C Hum: %02g", temp, humid);
}

 void twi_event_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRFX_TWIM_XFER_TX:
                case NRFX_TWIM_XFER_TXTX:
//                    NRF_LOG_INFO("TWIM TX");
                    twi_tx_done = true;
                    break;
                case NRFX_TWIM_XFER_RX:
                case NRFX_TWIM_XFER_TXRX:
//                    NRF_LOG_INFO("TWIM RX");
                    twi_rx_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRFX_TWIM_EVT_ADDRESS_NACK:
            break;
        case NRFX_TWIM_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}

const char *hx = "0123456789ABCDEF";

int ifmt(char *buf, int v, int base)
{
    int val = 1;
    int div;
    while (1) {
        div = v / val;
        if (div < base) break;
        val *= base;
    }
    int i = 0;
    do {
        div = v / val;
        v -= div * val;
        buf[i++] = hx[div];
        val /= base;
    } while (val);
    buf[i] = 0;
    return i;
}
// format a float
int ffmt(char *buf, float f, int precision)
{
    uint32_t num;
    memcpy(&num, &f, sizeof(num));

    bool sign = !!(num & 0x80000000);
    int  exp = (num >> 23) & 0xf;
    int  mant = num & ((1 << 23) - 1);
    /* Add leading 1 to mantissa (except 0.0) */
    if ((mant != 0) || (exp != 0)) { mant |= (1ULL << 23); }
    exp = exp - 255;
    int offset = 23 - exp;
    int lead = mant >> offset;
    int low = mant & (~(lead << offset));
    int skipped = 0;
    while (((low & 0x1) == 0) && low > 0) {
        low = low >> 1U;
        skipped++;
    }
    int highest = (offset - skipped);
    int base = 1;

    for (uint8_t i = 0; i < precision; i++) {
        base *= 10;
    }
    int fr = low * base >> highest;

    int cur = 0;
    if (sign) buf[cur++] = '-';
    cur += ifmt(&buf[cur], lead, 10);
    buf[cur++] = '.';
    cur += ifmt(&buf[cur], fr, 10);

    return cur;
}

int LIS3DH_ReadReg(uint8_t reg, uint8_t *val)
{
    int err_code;

    twi_tx_done = false;
    int timeout = I2C_TIMEOUT;
    while (nrfx_twim_is_busy(m_twi)) idle_state_handle();
    err_code = nrfx_twim_tx(m_twi, LIS3DH_ADDR, &reg, 1, false);
    while (!twi_tx_done && timeout--)
        idle_state_handle();
    if (!timeout) return -1;

    twi_rx_done = false;
    timeout = I2C_TIMEOUT;
    while (nrfx_twim_is_busy(m_twi)) idle_state_handle();
    err_code = nrfx_twim_rx(m_twi, LIS3DH_ADDR, val, 1);
    while (!twi_rx_done && timeout--)
        idle_state_handle();
    if (!timeout) return -1;

    return err_code;
}
int LIS3DH_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t cmd[2];
    cmd[0] = reg;
    cmd[1] = val;
    int err_code;
    twi_tx_done = false;
    int timeout = I2C_TIMEOUT;
    while (nrfx_twim_is_busy(m_twi)) idle_state_handle();
    err_code = nrfx_twim_tx(m_twi, LIS3DH_ADDR, cmd, 2, false);
    while (!twi_tx_done && timeout--)
        idle_state_handle();
    if (!timeout) return -1;

    return err_code;
}

#define MEMS_SET   0x01
#define MEMS_RESET 0x00

#define LIS3DH_CTRL_REG1  0x20
#define LIS3DH_CTRL_REG2  0x21
#define LIS3DH_CTRL_REG3  0x22
#define LIS3DH_CTRL_REG4  0x23
#define LIS3DH_CTRL_REG5  0x24
#define LIS3DH_CTRL_REG6  0x25
#define LIS3DH_REFERENCE  0x26
#define LIS3DH_STATUS_REG 0x27
#define LIS3DH_OUT_X_L    0x28
#define LIS3DH_OUT_X_H    0x29
#define LIS3DH_OUT_Y_L    0x2A
#define LIS3DH_OUT_Y_H    0x2B
#define LIS3DH_OUT_Z_L    0x2C
#define LIS3DH_OUT_Z_H    0x2D

#define LIS3DH_ODR_100Hz   0x05 // 10hz 2, 1hz 1
#define LIS3DH_ODR_BIT     (1 << 4)
#define LIS3DH_CTRL_REG4   0x23
#define LIS3DH_FS          (1 << 4)
#define LIS3DH_FULLSCALE_2 0x00
#define LIS3DH_X_ENABLE    0x01
#define LIS3DH_Y_ENABLE    0x02
#define LIS3DH_Z_ENABLE    0x04
#define LIS3DH_HR          (1 << 3)
#define LIS3DH_LPEN        (1 << 3)

// #define TWI_TIMEOUT			10000 //wont this cause softdevice issues??
void read_position(float *x_, float *y_, float *z_)
{
    // int err_code;

    int16_t  value;
    uint8_t *valueL = (uint8_t *)(&value);
    uint8_t *valueH = ((uint8_t *)(&value) + 1);

    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t status = 0;
    int retry=4;
    do {
        LIS3DH_ReadReg(LIS3DH_STATUS_REG, &status);
        retry--;
    } while ((retry) && (status & 0x8) == 0);

    if (!retry){NRF_LOG_INFO("fail read position");}
    LIS3DH_ReadReg(LIS3DH_OUT_X_L, valueL);
    LIS3DH_ReadReg(LIS3DH_OUT_X_H, valueH);
    x = value;

    LIS3DH_ReadReg(LIS3DH_OUT_Y_L, valueL);
    LIS3DH_ReadReg(LIS3DH_OUT_Y_H, valueH);
    y = value;

    LIS3DH_ReadReg(LIS3DH_OUT_Z_L, valueL);
    LIS3DH_ReadReg(LIS3DH_OUT_Z_H, valueH);
    z = value;

    *x_ = x / 16384.0;
    *y_ = y / 16384.0;
    *z_ = z / 16384.0;
}

/**
 * @brief Initialise temp sensor
 */
void SHT3C_set_mode(void)
{
    ret_code_t err_code;

    m_xfer_done = false;
    uint8_t cmd_sleep[2] = {0x80, 0x5d}; //?
    twi_tx_done = false;
    int timeout = I2C_TIMEOUT;
    while (nrfx_twim_is_busy(m_twi)) idle_state_handle();
    err_code = nrfx_twim_tx(m_twi, SHT3C_ADDR, cmd_sleep, 2, false);
    APP_ERROR_CHECK(err_code);
    while (!twi_tx_done && timeout--)
        idle_state_handle();
}

char buf[16];

int read_temperature(float *temp_, float *humid_)
{
    ret_code_t err_code;

    uint8_t cmd_wake[2] = {0x35, 0x17};
    twi_tx_done = false;
    int timeout = I2C_TIMEOUT;
    while (nrfx_twim_is_busy(m_twi)) idle_state_handle();
    err_code = nrfx_twim_tx(m_twi, SHT3C_ADDR, cmd_wake, 2, false);
    APP_ERROR_CHECK(err_code);
    while (!twi_tx_done && timeout--)
        idle_state_handle();
    if (!timeout) return -1;

    uint8_t cmd_read[2] = {0x5c, 0x24};
    twi_tx_done = false;
    timeout = I2C_TIMEOUT;
    while (nrfx_twim_is_busy(m_twi)) idle_state_handle();
    err_code = nrfx_twim_tx(m_twi, SHT3C_ADDR, cmd_read, 2, false);
    APP_ERROR_CHECK(err_code);
    while (!twi_tx_done && timeout--)
        idle_state_handle();
    if (!timeout) return -1;

    uint8_t l_temp[6];

    twi_rx_done = false;
    timeout = I2C_TIMEOUT;
    while (nrfx_twim_is_busy(m_twi)) idle_state_handle();
    err_code = nrfx_twim_rx(m_twi, SHT3C_ADDR, &l_temp[0], 6);
    APP_ERROR_CHECK(err_code);
    while (!twi_rx_done && timeout--)
        idle_state_handle();
    if (!timeout) return -1;

    {
        uint16_t n = (l_temp[0] << 8) + l_temp[1];
        float    humid = (float)n / 655.36;

        uint16_t m = (l_temp[3] << 8) + l_temp[4];
        float    temp = -45 + 175 * (float)m / 65536;

        *temp_ = temp;
        *humid_ = humid;

//        buf[0] = 0;
//        ffmt(buf, temp, 2);
//        NRF_LOG_INFO("%s", NRF_LOG_PUSH(buf));
//        NRF_LOG_INFO("Temp: %d.%d Humid: %d.%d", high, low, hhigh, hlow);
    }

#if 0 // doesnt seem to wake
    m_xfer_done = false;
    uint8_t cmd_sleep[2] = {0xb0, 0x98};
    twi_tx_done = false;
    timeout = I2C_TIMEOUT;
    while (nrfx_twim_is_busy(m_twi)) idle_state_handle();
    err_code = nrfx_twim_tx(m_twi, SHT3C_ADDR, cmd_sleep, 2, false);
    APP_ERROR_CHECK(err_code);
    while (!twi_tx_done && timeout--)
        idle_state_handle();
    if (!timeout) return -1;
#endif
    return 0;
}

void sensors_init(const nrfx_twim_t *twi)
{
    m_twi = twi;

    // SHT3C_set_mode();

    float temp,hum;
    read_temperature(&temp,&hum); //sanity check

    // init
#if 0
    uint8_t value;
    LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value);
    value &= 0x0f;
    value |= LIS3DH_ODR_100Hz << LIS3DH_ODR_BIT;
    LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value);

    // setmode
    uint8_t value2;
    LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value);
    LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value2);

    if ((value & 0xF0) == 0)
        value = value | (0 & 0xF0); // if it comes from POWERDOWN

    value &= 0xF7;
    value |= (MEMS_RESET << LIS3DH_LPEN);
    value2 &= 0xF7;
    value2 |= (MEMS_SET << LIS3DH_HR); // set HighResolution_BIT

    LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value);
    LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value2);

    // scale
    LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value);
    value &= 0xCF;
    value |= (LIS3DH_FULLSCALE_2 << LIS3DH_FS);
    LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value);

    // axis
    LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value);
    value &= 0xF8;
    value |= (0x07 & (LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE));

    LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value);
#endif
NRF_LOG_INFO("Init 3d");
    LIS3DH_WriteReg(LIS3DH_CTRL_REG1, 0x57);  //minimal init
    LIS3DH_WriteReg(LIS3DH_CTRL_REG2, 0);
    LIS3DH_WriteReg(LIS3DH_CTRL_REG3, 0);
    LIS3DH_WriteReg(LIS3DH_CTRL_REG4, 0);
    LIS3DH_WriteReg(LIS3DH_CTRL_REG5, 0);
    LIS3DH_WriteReg(LIS3DH_CTRL_REG6, 0);

    // is twi working?
//    uint8_t cnt = 1;
    float  x, y, z;
//    while (cnt--)
NRF_LOG_INFO("Read pos");
        read_position(&x, &y, &z);
}
