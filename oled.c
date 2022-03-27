#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"

#include "stdint.h"

#include "nrf.h"
#include "nrf_delay.h"

#include "SEGGER_RTT.h"
#include "app_timer.h"

#include "nrfx_twim.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_sdh.h"

#include "oled.h"

static const nrfx_twim_t *m_twi = 0;

#define SCREEN_WIDTH  128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define SET_CONTRAST        0x81
#define SET_ENTIRE_ON       0xa4
#define SET_NORM_INV        0xa6
#define SET_DISP            0xae
#define SET_MEM_ADDR        0x20
#define SET_COL_ADDR        0x21
#define SET_PAGE_ADDR       0x22
#define SET_DISP_START_LINE 0x40
#define SET_SEG_REMAP       0xa0
#define SET_MUX_RATIO       0xa8
#define SET_COM_OUT_DIR     0xc0
#define SET_DISP_OFFSET     0xd3
#define SET_COM_PIN_CFG     0xda
#define SET_DISP_CLK_DIV    0xd5
#define SET_PRECHARGE       0xd9
#define SET_VCOM_DESEL      0xdb
#define SET_CHARGE_PUMP     0x8d

#define OLED_ADDR 0x3c // 0x78/2

#define I2C_TIMEOUT 100

// 16x16 icons
uint16_t icons[] = {
    0xffff, 0xffff, 0xc001, 0xc07f, 0xc07d, 0xc07f, 0xc07d, 0xc07f, //disk
    0xc045, 0xc043, 0xc07d, 0xc07f, 0xc001, 0xc003, 0xffff, 0xfffe,
    
    0xe39c, 0xe39c, 0xc39c, 0x071c, 0x073c, 0x0f3c, 0x1e38, 0xfc78, //data
    0xf870, 0xe0f0, 0x03e0, 0x1fc0, 0xff80, 0xfe00, 0xf000, 0x0000,
    
    0x0004, 0x0008, 0x0010, 0xfffe, 0x0010, 0x0008, 0xf004, 0xf000, //signal low
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    
    0x0004, 0x0008, 0x0010, 0xfffe, 0x0010, 0x0008, 0xf004, 0xf000, //signal high
    0x0000, 0x0000, 0xfe00, 0xfe00, 0x0000, 0x0000, 0xff80, 0xff80,
    
    0x0c03, 0x3f07, 0x7fca, 0xffdc, 0xffb8, 0xff70, 0xfeec, 0xfddc, //gps
    0x7dbe, 0x7e7e, 0x3ffe, 0x1ffe, 0x1ffe, 0x07fe, 0x03fc, 0x0078,
    
    0x1ff0, 0x1010, 0x1550, 0x16d0, 0x17d0, 0x1010, 0x1010, 0x1010,
    0x1010, 0x1010, 0x1010, 0x1010, 0x1010, 0x1010, 0x1c70, 0x07c0, //low
    
    0x1ff0, 0x1010, 0x1550, 0x16d0, 0x1550, 0x16d0, 0x1550, 0x17d0, //mid
    0x1010, 0x1010, 0x1010, 0x1010, 0x1010, 0x1010, 0x1c70, 0x07c0,
    
    0x1ff0, 0x1010, 0x1550, 0x16d0, 0x1550, 0x16d0, 0x1550, 0x16d0, //full
    0x1550, 0x16d0, 0x1550, 0x16d0, 0x17d0, 0x1010, 0x1c70, 0x07c0,
    
    0x1ff0, 0x1010, 0x0550, 0x6450, 0x7190, 0x39d0, 0x1de0, 0x0f70, //charge
    0x1738, 0x131c, 0x144c, 0x1450, 0x17d0, 0x1010, 0x1c70, 0x07c0,
    
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3ff8, 0x3ff8, 0x1ff0, //play
    0x0fe0, 0x07c0, 0x0380, 0x0100, 0x0000, 0x0000, 0x0000, 0x0000,
    
    0x0000, 0x0000, 0x0000, 0x0000, 0x1ff8, 0x1ff8, 0x1ff8, 0x0000, //pause
    0x0000, 0x0000, 0x1ff8, 0x1ff8, 0x1ff8, 0x0000, 0x0000, 0x0000
};
uint16_t glyphs[] = {
//32
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x33fc, 0x33fc, 0x0000, 0x0000, 0x0000,
0x0000, 0x00fc, 0x00fc, 0x0000, 0x0000, 0x00fc, 0x00fc, 0x0000, 0x0330, 0x0ffc, 0x0ffc, 0x0330, 0x0330, 0x0ffc, 0x0ffc, 0x0330,
0x0000, 0x0c78, 0x0cfc, 0x3ccf, 0x3ccf, 0x0fcc, 0x078c, 0x0000, 0x0000, 0x0c1c, 0x0f1c, 0x03c0, 0x00f0, 0x0e3c, 0x0e0c, 0x0000,
0x1f00, 0x3fce, 0x30ff, 0x33f3, 0x1f3f, 0x3f0e, 0x3300, 0x0000, 0x0000, 0x0000, 0x0000, 0x00fc, 0x00fc, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0ff0, 0x1ff8, 0x381c, 0x2004, 0x0000, 0x0000, 0x2004, 0x381c, 0x1ff8, 0x0ff0, 0x0000, 0x0000, 0x0000,
0x00c0, 0x0ccc, 0x0ffc, 0x03f0, 0x03f0, 0x0ffc, 0x0ccc, 0x00c0, 0x0000, 0x00c0, 0x00c0, 0x07f8, 0x07f8, 0x00c0, 0x00c0, 0x0000,
0x0000, 0x0000, 0xc000, 0x7c00, 0x3c00, 0x0000, 0x0000, 0x0000, 0x0000, 0x00c0, 0x00c0, 0x00c0, 0x00c0, 0x00c0, 0x00c0, 0x0000,
0x0000, 0x0000, 0x0000, 0x3c00, 0x3c00, 0x0000, 0x0000, 0x0000, 0x0000, 0x3800, 0x3e00, 0x0780, 0x01e0, 0x007c, 0x001c, 0x0000,
0x0000, 0x1ff8, 0x3ffc, 0x310c, 0x308c, 0x3ffc, 0x1ff8, 0x0000, 0x0000, 0x3000, 0x3030, 0x3ffc, 0x3ffc, 0x3000, 0x3000, 0x0000,
0x0000, 0x3038, 0x3c3c, 0x3f0c, 0x33cc, 0x30fc, 0x3038, 0x0000, 0x0000, 0x1c0c, 0x3c0c, 0x30cc, 0x33fc, 0x3f3c, 0x1c0c, 0x0000,
0x0000, 0x0f00, 0x0fc0, 0x0cf0, 0x3ffc, 0x3ffc, 0x0c00, 0x0000, 0x0000, 0x18fc, 0x38fc, 0x30cc, 0x30cc, 0x3fcc, 0x1f8c, 0x0000,
0x0000, 0x1ff0, 0x3ff8, 0x319c, 0x318c, 0x3f8c, 0x1f00, 0x0000, 0x0000, 0x000c, 0x3c0c, 0x3f0c, 0x03cc, 0x00fc, 0x003c, 0x0000,
0x0000, 0x1f38, 0x3ffc, 0x30cc, 0x30cc, 0x3ffc, 0x1f38, 0x0000, 0x0000, 0x0078, 0x30fc, 0x30cc, 0x38cc, 0x1ffc, 0x0ff8, 0x0000,
0x0000, 0x0000, 0x0000, 0x3cf0, 0x3cf0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xc000, 0x7cf0, 0x3cf0, 0x0000, 0x0000, 0x0000,
0x0080, 0x01c0, 0x03e0, 0x0770, 0x0e38, 0x0c18, 0x0808, 0x0000, 0x0000, 0x0330, 0x0330, 0x0330, 0x0330, 0x0330, 0x0330, 0x0000,
0x0808, 0x0c18, 0x0e38, 0x0770, 0x03e0, 0x01c0, 0x0080, 0x0000, 0x0000, 0x0038, 0x003c, 0x370c, 0x37cc, 0x00fc, 0x0038, 0x0000,
0x0ff0, 0x1ff8, 0x381c, 0x33cc, 0x324c, 0x3398, 0x19f0, 0x0000, 0x0000, 0x3ff0, 0x3ff8, 0x031c, 0x031c, 0x3ff8, 0x3ff0, 0x0000,
0x0000, 0x3ffc, 0x3ffc, 0x30cc, 0x30cc, 0x3ffc, 0x1f78, 0x0000, 0x0000, 0x1ff8, 0x3ffc, 0x300c, 0x300c, 0x3c3c, 0x1c38, 0x0000,
0x0000, 0x3ffc, 0x3ffc, 0x300c, 0x381c, 0x1ff8, 0x0ff0, 0x0000, 0x0000, 0x3ffc, 0x3ffc, 0x30cc, 0x30cc, 0x30cc, 0x300c, 0x0000,
0x0000, 0x3ffc, 0x3ffc, 0x00cc, 0x00cc, 0x00cc, 0x000c, 0x0000, 0x0000, 0x1ff8, 0x3ffc, 0x300c, 0x30cc, 0x3fcc, 0x1fcc, 0x0000,
0x0000, 0x3ffc, 0x3ffc, 0x00c0, 0x00c0, 0x3ffc, 0x3ffc, 0x0000, 0x0000, 0x300c, 0x300c, 0x3ffc, 0x3ffc, 0x300c, 0x300c, 0x0000,
0x0000, 0x1c00, 0x3c00, 0x3000, 0x3000, 0x3ffc, 0x1ffc, 0x0000, 0x3ffc, 0x3ffc, 0x00c0, 0x03f0, 0x0f3c, 0x3c0c, 0x3000, 0x0000,
0x0000, 0x3ffc, 0x3ffc, 0x3000, 0x3000, 0x3000, 0x3000, 0x0000, 0x3ffc, 0x3ffc, 0x0070, 0x01c0, 0x0070, 0x3ffc, 0x3ffc, 0x0000,
0x0000, 0x3ffc, 0x3ffc, 0x01e0, 0x0780, 0x3ffc, 0x3ffc, 0x0000, 0x0000, 0x1ff8, 0x3ffc, 0x300c, 0x300c, 0x3ffc, 0x1ff8, 0x0000,
0x0000, 0x3ffc, 0x3ffc, 0x030c, 0x030c, 0x03fc, 0x01f8, 0x0000, 0x0000, 0x1ff8, 0x3ffc, 0x300c, 0x180c, 0x37fc, 0x2ff8, 0x0000,
0x3ffc, 0x3ffc, 0x018c, 0x038c, 0x0ffc, 0x3cf8, 0x3000, 0x0000, 0x0000, 0x3078, 0x30fc, 0x31cc, 0x338c, 0x3f0c, 0x1e0c, 0x0000,
0x0000, 0x000c, 0x000c, 0x3ffc, 0x3ffc, 0x000c, 0x000c, 0x0000, 0x0000, 0x1ffc, 0x3ffc, 0x3000, 0x3000, 0x3ffc, 0x1ffc, 0x0000,
0x0000, 0x03fc, 0x0ffc, 0x3c00, 0x3c00, 0x0ffc, 0x03fc, 0x0000, 0x3ffc, 0x1ffc, 0x0e00, 0x0780, 0x0e00, 0x1ffc, 0x3ffc, 0x0000,
0x0000, 0x381c, 0x3e7c, 0x07e0, 0x07e0, 0x3e7c, 0x381c, 0x0000, 0x0000, 0x003c, 0x00fc, 0x3fc0, 0x3fc0, 0x00fc, 0x003c, 0x0000,
0x0000, 0x3c0c, 0x3f0c, 0x33cc, 0x30fc, 0x303c, 0x300c, 0x0000, 0x0000, 0x0000, 0x0000, 0x3ffc, 0x3ffc, 0x300c, 0x300c, 0x0000,
0x0000, 0x001c, 0x007c, 0x01e0, 0x0780, 0x3e00, 0x3800, 0x0000, 0x0000, 0x300c, 0x300c, 0x3ffc, 0x3ffc, 0x0000, 0x0000, 0x0000,
0x0180, 0x01e0, 0x0078, 0x001e, 0x0078, 0x01e0, 0x0180, 0x0000, 0x3000, 0x3000, 0x3000, 0x3000, 0x3000, 0x3000, 0x3000, 0x0000,
0x0000, 0x0007, 0x000e, 0x001c, 0x0038, 0x0070, 0x0000, 0x0000, 0x0000, 0x1e00, 0x3f60, 0x3360, 0x3360, 0x3fe0, 0x3fc0, 0x0000,
0x0000, 0x3ffc, 0x3ffc, 0x3060, 0x3060, 0x3fe0, 0x1fc0, 0x0000, 0x0000, 0x1fc0, 0x3fe0, 0x3060, 0x3060, 0x3060, 0x3000, 0x0000,
0x0000, 0x1fc0, 0x3fe0, 0x3060, 0x3060, 0x3ffc, 0x3ffc, 0x0000, 0x0000, 0x1fc0, 0x3fe0, 0x3260, 0x3260, 0x33e0, 0x33c0, 0x0000,
0x0000, 0x00c0, 0x00c0, 0x3ff8, 0x3ffc, 0x00cc, 0x00cc, 0x0000, 0x0000, 0xcfc0, 0xdfe0, 0xd860, 0xd860, 0xffe0, 0x7fe0, 0x0000,
0x0000, 0x3ffc, 0x3ffc, 0x0060, 0x0060, 0x3fe0, 0x3fc0, 0x0000, 0x0000, 0x0000, 0x3060, 0x3fec, 0x3fec, 0x3000, 0x0000, 0x0000,
0x0000, 0xc000, 0xc000, 0xc000, 0xffec, 0x7fec, 0x0000, 0x0000, 0x3ffc, 0x3ffc, 0x0380, 0x07c0, 0x1ee0, 0x3860, 0x3000, 0x0000,
0x0000, 0x0000, 0x300c, 0x3ffc, 0x3ffc, 0x3000, 0x0000, 0x0000, 0x3fc0, 0x3fe0, 0x00e0, 0x07c0, 0x00e0, 0x3fe0, 0x3fc0, 0x0000,
0x0000, 0x3fc0, 0x3fe0, 0x0060, 0x0060, 0x3fe0, 0x3fc0, 0x0000, 0x0000, 0x1fc0, 0x3fe0, 0x3060, 0x3060, 0x3fe0, 0x1fc0, 0x0000,
0x0000, 0xffe0, 0xffe0, 0x3060, 0x3060, 0x3fe0, 0x1fc0, 0x0000, 0x0000, 0x1fc0, 0x3fe0, 0x3060, 0x3060, 0xffe0, 0xffe0, 0x0000,
0x0000, 0x3fe0, 0x3fe0, 0x0060, 0x0060, 0x00e0, 0x00c0, 0x0000, 0x0000, 0x31c0, 0x33e0, 0x3360, 0x3660, 0x3e60, 0x1c60, 0x0000,
0x0000, 0x0060, 0x0060, 0x1ff8, 0x3ff8, 0x3060, 0x3060, 0x0000, 0x0000, 0x1fe0, 0x3fe0, 0x3000, 0x3000, 0x3fe0, 0x3fe0, 0x0000,
0x0000, 0x03e0, 0x0fe0, 0x3c00, 0x3c00, 0x0fe0, 0x03e0, 0x0000, 0x3fe0, 0x1fe0, 0x0e00, 0x0780, 0x0e00, 0x1fe0, 0x3fe0, 0x0000,
0x0000, 0x3060, 0x3de0, 0x0f80, 0x0f80, 0x3de0, 0x3060, 0x0000, 0x0000, 0xcfe0, 0xdfe0, 0xd800, 0xd800, 0xffe0, 0x7fe0, 0x0000,
0x0000, 0x3060, 0x3c60, 0x3f60, 0x33e0, 0x30e0, 0x3060, 0x0000, 0x0180, 0x0180, 0x03c0, 0x3ffc, 0x7e7e, 0x4002, 0x4002, 0x0000,
0x0000, 0x0000, 0x0000, 0x7ffe, 0x7ffe, 0x0000, 0x0000, 0x0000, 0x4002, 0x4002, 0x7e7e, 0x3ffc, 0x03c0, 0x0180, 0x0180, 0x0000,
0x01c0, 0x0060, 0x00e0, 0x01c0, 0x0180, 0x0180, 0x00e0, 0x0000, 0x1800, 0x1e00, 0x1380, 0x10e0, 0x10e0, 0x1380, 0x1e00, 0x1800,
0x0000, 0x0030, 0x0078, 0x00f8, 0x01f0, 0x00f8, 0x0078, 0x0030, 0x0000, 0x0008, 0x0014, 0x0008, 0x00e0, 0x0110, 0x0110, 0x00a0,
0x0000, 0x1c00, 0x3f00, 0x3fcc, 0x3f00, 0x1c00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};
//128 hrm
//129 temp
//130 glu

//#define I2C_TIMEOUT 10000
int      scrn_pages = 0;
uint8_t *scrn_buffer = 0;
uint8_t  cmd_buf[3];
bool     oled_state = false;

void TxReady();
bool TxDone();

void idle_state_handle(void);

void sendCmd1(uint8_t I2C_Command)
{
    cmd_buf[1] = I2C_Command;
    TxReady();
    int        timeout = I2C_TIMEOUT;
    ret_code_t err_code = nrfx_twim_tx(m_twi, OLED_ADDR, cmd_buf, 2, false);
    APP_ERROR_CHECK(err_code);
    while (!TxDone() && timeout--)
        idle_state_handle();
}

void sendCmd2(uint8_t I2C_Command, uint8_t param)
{
    cmd_buf[1] = I2C_Command;
    cmd_buf[2] = param;
    TxReady();
    int        timeout = I2C_TIMEOUT;
    ret_code_t err_code = nrfx_twim_tx(m_twi, OLED_ADDR, cmd_buf, 3, false);
    APP_ERROR_CHECK(err_code);
    while (!TxDone() && timeout--)
        idle_state_handle();
}

void sendData(uint8_t *buf, int len)
{
    TxReady();
    int        timeout = I2C_TIMEOUT;
    ret_code_t err_code = nrfx_twim_tx(m_twi, OLED_ADDR, buf, len, false);
    APP_ERROR_CHECK(err_code);
    while (!TxDone() && timeout--)
        idle_state_handle();
}

bool OLED_state() { return oled_state; }
void OLED_off()
{
    oled_state = false;
    sendCmd1(SET_DISP | 0x00);
}
void OLED_on()
{
    oled_state = true;
    sendCmd1(SET_DISP | 0x01);
}

void OLED_clear() { memset(scrn_buffer, 0x0, scrn_pages * SCREEN_WIDTH); };

// Initiate start of transfer
void OLED_cursor(int x, int y, int cx, int cy)
{
    uint8_t buf[8];
    buf[0] = 0;
    buf[1] = 0x15;
    buf[2] = x / 2;                         // start address
    buf[3] = (uint8_t)(((x + cx) / 2) - 1); // end address
    buf[4] = 0x75;                          // row start/end
    buf[5] = y;                             // start row
    buf[6] = y + cy - 1;                    // end row

    TxReady();
    int        timeout = I2C_TIMEOUT;
    ret_code_t err_code = nrfx_twim_tx(m_twi, OLED_ADDR, buf, 7, false);
    APP_ERROR_CHECK(err_code);
    while (!TxDone() && timeout--)
        idle_state_handle();
}

void OLED_write(uint8_t *buf, int len)
{
    TxReady();
    int        timeout = I2C_TIMEOUT;
    ret_code_t err_code = nrfx_twim_tx(m_twi, OLED_ADDR, buf, len, false);
    APP_ERROR_CHECK(err_code);
    while (!TxDone() && timeout--)
        idle_state_handle();
}

// Id is offset into icons array (8x4)
void drawIcon(int x, int y, int id)
{
    for (int i = 0; i < 16; i++) {
        uint16_t col = icons[id + i];
        scrn_buffer[i + (x * 8) + (y * 128)] = (uint8_t)(col & 0xff);
        scrn_buffer[i + (x * 8) + (y * 128 + 128)] = (uint8_t)(col >> 8);
    }
};

// coordinates array using 16*8 grid.
void drawLetter(int x, int y, uint8_t chr)
{
    int id = chr - 32;
    for (int i = 0; i < 8; i++) {
        uint16_t col = glyphs[(id * 8) + i];

        scrn_buffer[i + (x * 8) + (y * 128)] = (uint8_t)(col & 0xff);
        scrn_buffer[i + (x * 8) + (y * 128) + 128] = (uint8_t)(col >> 8);
    }
};

// coordinates array using 16*8 grid.
void drawString(int x, int y, const uint8_t *msg)
{
    int len = strlen(msg);
    for (int i = 0; i < len; ++i) {
        if (msg[i] == '\n') {
            x = 0;
            y += 2;
            continue;
        }
        drawLetter(x++, y, msg[i]);
        if (x > 16) {
            x = 0;
            y += 2;
        }
    }
};

// Inefficient full buffer copy
void OLED_show()
{
    static uint8_t temp[SCREEN_WIDTH + 1];
    temp[0] = 0x40;

    int x0 = 0;
    int x1 = SCREEN_WIDTH - 1;

    sendCmd1(SET_COL_ADDR);
    sendCmd1(x0);
    sendCmd1(x1);
    sendCmd1(SET_PAGE_ADDR);
    sendCmd1(0);
    sendCmd1(scrn_pages - 1);

    // Line by line xfer
    for (int y = 0; y < scrn_pages; y++) {
        memcpy(&temp[1], &scrn_buffer[y * SCREEN_WIDTH], SCREEN_WIDTH);
        //    for (int x=0; x<SCREEN_WIDTH/32; x++)
        {
            TxReady();
            int        timeout = I2C_TIMEOUT;
//             ret_code_t err_code =
                nrfx_twim_tx(m_twi, OLED_ADDR, temp,  SCREEN_WIDTH+1, false);
//            APP_ERROR_CHECK(err_code);
            while (!TxDone() && timeout--)
                idle_state_handle();
        }
    }
}

// Initialise LCD with defaults
bool lcd_init()
{
    memset(cmd_buf, 0, 3);

    OLED_off();
    sendCmd2(SET_MEM_ADDR, 0x00);
    sendCmd1(SET_DISP_START_LINE | 0x00);
    sendCmd1(SET_SEG_REMAP | 0x00); // column addr 127 mapped to SEG0
    sendCmd2(SET_MUX_RATIO, SCREEN_HEIGHT - 1);
    sendCmd1(SET_COM_OUT_DIR | 0x01); // scan from COM[N] to COM0
    sendCmd2(SET_DISP_OFFSET, 0x00);  //
    sendCmd2(SET_COM_PIN_CFG, 0x12);  //

    //#timing and driving scheme
    sendCmd2(SET_DISP_CLK_DIV, 0x80);
    sendCmd2(SET_PRECHARGE, 0xf1);
    sendCmd2(SET_VCOM_DESEL, 0x30); // 0.83*Vcc

    //#display
    sendCmd2(SET_CONTRAST, 0xff); // maximum
    sendCmd1(SET_ENTIRE_ON);      // output follows RAM contents
    sendCmd1(SET_NORM_INV);       // not inverted

    //#charge pump
    sendCmd2(SET_CHARGE_PUMP, 0x14);
    OLED_on();

    scrn_pages = SCREEN_HEIGHT / 8;
    scrn_buffer = (uint8_t *)malloc(scrn_pages * SCREEN_WIDTH);

    if (!scrn_buffer) APP_ERROR_CHECK(-1);

    OLED_clear();
    return true;
}

bool OLED_init(const nrfx_twim_t *twi)
{
    m_twi = twi;
    lcd_init();
    return true;
}
