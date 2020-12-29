#define POWER_DATA_BUFFER_SIZE 121
#define POWER_EVENT_BUFFER_SIZE 61

typedef struct power_data_s {
	uint32_t timestamp;
	uint32_t samples;
	uint32_t duration_usec;
	float vrms[2];
	float irms[2];
	float p[2];
} power_data_t;

typedef struct power_data_flash_s {
	uint32_t timestamp;
	uint32_t seconds;
	float active[2];
} power_data_flash_t;

typedef struct power_event_s {
	uint32_t timestamp;
	uint8_t type;
	uint8_t channel;
	uint16_t count;
	float avg_value;
	float worst_value;
} power_event_t;

typedef enum {
	POWER_EVENT_TYPE_VOLTAGE_LOW,
	POWER_EVENT_TYPE_VOLTAGE_HIGH,
	POWER_EVENT_TYPE_VOLTAGE_SPIKE,
	POWER_EVENT_TYPE_AC_FREQUENCY_HIGH,
	POWER_EVENT_TYPE_AC_FREQUENCY_LOW,
	POWER_EVENT_TYPE_OVERCURRENT,
	POWER_EVENT_TYPE_QTY
} power_event_type_t;

extern uint16_t power_data_count;
extern uint16_t power_events_count;


void power_processing_task(void *pvParameters);

int get_power_data(power_data_t *data, unsigned int index);
int delete_power_data(unsigned int qty);

int convert_power_data_flash(power_data_flash_t *data);

int get_power_event(power_event_t *data, unsigned int index);
int delete_power_events(unsigned int qty);

void get_waveform(float *buffer, unsigned int channel, unsigned int qty);
