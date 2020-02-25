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

#define RTC_MAX_READ_PERIOD_US 70U * 60U * 1000000U

#define WAVEFORM_MAX_QTY 100

#define SERVER_PORT 2048

#define MAX_DISCONNECTION_TIME_MS 5000
#define MIN_DISCONNECTION_TIME_MS 200

#define SECURITY_CHECK_MAC 1

typedef enum {
	LED_OFF,
	LED_COLOR_RED,
	LED_COLOR_GREEN,
	LED_COLOR_YELLOW,
	LED_COLOR_BLUE,
	LED_COLOR_PURPLE,
	LED_COLOR_TEAL,
	LED_COLOR_WHITE
} led_colors_t;

extern uint8_t status_sampling_running;
extern uint8_t status_server_connected;
