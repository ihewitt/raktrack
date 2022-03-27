typedef struct {
    uint8_t data;
    uint8_t hrm;
    float   skin;
    float   core;
    float   gluc;
} ant_state_t;

typedef enum {
    S_HR = 1, // Bitmask for state data
    S_SK = 2,
    S_CR = 4,
    S_GL = 8
} state_flags;

void ant_profile_setup(int hrm, int core);
void ble_scan_init(uint8_t *uid);
void ble_scan_start(void);
