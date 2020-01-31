#define PROCESSED_DATA_BUFFER_SIZE 121
#define POWER_EVENT_BUFFER_SIZE 61

typedef struct power_data_s {
	uint32_t timestamp;
	uint32_t samples;
	uint32_t duration_usec;
	float vrms[3];
	float irms[3];
	float p[3];
} power_data_t;

typedef struct power_event_s {
	uint32_t timestamp;
	uint16_t type;
	uint16_t channel;
	uint32_t duration;
	float value;
} power_event_t;

typedef enum {
	POWER_EVENT_VOLTAGE_SAG,
	POWER_EVENT_VOLTAGE_SWELL,
	POWER_EVENT_OVERCURRENT,
	POWER_EVENT_FREQUENCY_VARIATION,
} power_event_type_t;

extern power_data_t processed_data[PROCESSED_DATA_BUFFER_SIZE];
extern uint16_t processed_data_head, processed_data_tail, processed_data_count;

extern power_event_t power_events[POWER_EVENT_BUFFER_SIZE];
extern uint16_t power_events_data_head, power_events_data_tail, power_events_data_count;

void power_processing_task(void *pvParameters);
