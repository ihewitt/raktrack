
typedef struct {
    char  time[7];
    char  date[7];
    int   sats;
    float latitude;
    float longitude;
    float altitude;
    char  val;
    char  fix;
    float direction;
    float speed;
} nav_t;

typedef struct {
    char imei[16];
    bool dat_on; // apn
    bool mob_on; // signal
} gsm_t;

void GPS_Pins();
void GPS_Init();
void GPS_On();
void GPS_Poll();
int GPS_Config();
bool GPS_GetData(char *data, int len);
int UploadData(char *msg, char *sever, int port);
void UploadConnect();
void GSM_Init();
int GSM_Register(char *apn, char *user, char *pwd);

void GSM_events();
