#define FW_TYPE 1
#define FW_VERSION "0.0.21"

//#define DEBUG

#ifdef DEBUG
#define debug(fmt, ...) printf(fmt, ## __VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define SCL_PIN 5
#define SDA_PIN 4
#define LED_R_PIN 2
#define LED_G_PIN 12
#define LED_B_PIN 13
#define BTN_PIN 15
#define READY_PIN 14

#define I2C_BUS 0

#define RTC_READ_PERIOD_US 60U * 60U * 1000000U

#define WAVEFORM_MAX_QTY 100

#define SERVER_PORT 2048

#define MAX_DISCONNECTION_TIME_MS 5000
#define MIN_DISCONNECTION_TIME_MS 200

#define SECURITY_CHECK_MAC 1

extern uint8_t sampling_running;
