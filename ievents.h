#define INTERNAL_EVENT_BUFFER_SIZE 61

typedef struct internal_event_s {
	uint32_t timestamp;
	uint8_t type;
	uint8_t count;
	uint32_t value;
} internal_event_t;

typedef enum {
	INTERNAL_EVENT_BUFFER_FULL,
	INTERNAL_EVENT_LOW_SAMPLING_FREQUENCY,
	INTERNAL_EVENT_SAMPLING_STOPPED,
	INTERNAL_EVENT_SEND_TIMEOUT,
	INTERNAL_EVENT_RESPONSE_TIMEOUT,
	INTERNAL_EVENT_INVALID_MAC,
	INTERNAL_EVENT_I2C_ERROR
} internal_event_type_t;

extern internal_event_t internal_events[INTERNAL_EVENT_BUFFER_SIZE];
extern uint16_t internal_events_head;
extern uint16_t internal_events_tail;
extern uint16_t internal_events_count;

void add_internal_event(int event_type, int value);
