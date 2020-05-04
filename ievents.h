#define IEVENT_BUFFER_SIZE 61

typedef struct ievent_s {
	uint32_t timestamp;
	uint16_t type;
	uint16_t count;
	uint32_t value;
} internal_event_t;

typedef enum {
	IEVENT_TYPE_ADC_BUFFER_FULL,
	IEVENT_TYPE_IEVENTS_BUFFER_FULL,
	IEVENT_TYPE_POWER_EVENTS_BUFFER_FULL,
	IEVENT_TYPE_LOW_SAMPLING_FREQUENCY,
	IEVENT_TYPE_SAMPLING_STOPPED,
	IEVENT_TYPE_SEND_TIMEOUT,
	IEVENT_TYPE_RESPONSE_TIMEOUT,
	IEVENT_TYPE_INVALID_MAC,
	IEVENT_TYPE_I2C_ERROR,
	IEVENT_TYPE_QTY
} ievent_type_t;

extern uint16_t internal_events_count;


int ievents_init();

int add_internal_event(int type, int value, uint32_t event_time);
int get_internal_event(internal_event_t *data, unsigned int index);
int delete_internal_events(unsigned int qty);
