#define PROCESSED_DATA_BUFFER_SIZE 121
#define POWER_EVENT_BUFFER_SIZE 61

typedef struct power_data_s {
	uint32_t timestamp;
	uint32_t samples;
	uint32_t duration_usec;
	float vrms[2];
	float irms[2];
	float p[2];
} power_data_t;

typedef struct power_event_s {
	uint32_t timestamp;
	uint16_t type;
	uint16_t channel;
	uint32_t count;
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

extern uint16_t processed_data_count;
extern uint16_t power_events_count;


void power_processing_task(void *pvParameters);

int get_power_data(power_data_t *data, unsigned int index);
int delete_power_data(unsigned int qty);

int get_power_events(power_event_t *data, unsigned int index);
int delete_power_events(unsigned int qty);

void get_waveform(float *buffer, unsigned int channel, unsigned int qty);
