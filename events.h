#define EVENT_BUFFER_SIZE 61

typedef struct event_s {
	uint32_t timestamp;
	uint16_t type;
	uint16_t count;
	uint32_t value;
} event_t;

typedef enum {
	EVENT_TYPE_ADC_BUFFER_FULL,
	EVENT_TYPE_EVENTS_BUFFER_FULL,
	EVENT_TYPE_POWER_EVENTS_BUFFER_FULL,
	EVENT_TYPE_LOW_SAMPLING_FREQUENCY,
	EVENT_TYPE_SAMPLING_STOPPED,
	EVENT_TYPE_SEND_TIMEOUT,
	EVENT_TYPE_RESPONSE_TIMEOUT,
	EVENT_TYPE_INVALID_MAC,
	EVENT_TYPE_I2C_ERROR,
	EVENT_TYPE_QTY
} event_type_t;

extern uint16_t event_count;


int events_init();

int add_event(int type, int value, uint32_t event_time);
int get_event(event_t *data, unsigned int index);
int delete_events(unsigned int qty);
