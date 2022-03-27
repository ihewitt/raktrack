
#define ICON_DISK   0
#define ICON_DATA   (1 * 16)
#define ICON_SIGLOW (2 * 16)
#define ICON_SIGHI  (3 * 16)
#define ICON_GPS    (4 * 16)
#define ICON_BAT_L  (5 * 16)
#define ICON_BAT_M  (6 * 16)
#define ICON_BAT_H  (7 * 16)
#define ICON_BAT_CH (8 * 16)
#define ICON_PLAY   (9 * 16)
#define ICON_PAUSE  (10 * 16)

bool OLED_state();
void OLED_off();
void OLED_on();
void OLED_clear();
void OLED_invert(bool invert);
bool OLED_init(const nrfx_twim_t *m_twi);
void OLED_show();

void drawIcon(int x, int y, int id);
void drawLetter(int x, int y, uint8_t chr);
void drawString(int x, int y, const uint8_t *msg);
