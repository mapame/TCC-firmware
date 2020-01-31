#define IEVENT_BUFFER_SIZE 61

typedef struct ievent_s {
	uint32_t timestamp;
	uint16_t type;
	uint16_t count;
	uint32_t value;
} ievent_t;

typedef enum {
	IEVENT_TYPE_BUFFER_FULL,
	IEVENT_TYPE_LOW_SAMPLING_FREQUENCY,
	IEVENT_TYPE_SAMPLING_STOPPED,
	IEVENT_TYPE_SEND_TIMEOUT,
	IEVENT_TYPE_RESPONSE_TIMEOUT,
	IEVENT_TYPE_INVALID_MAC,
	IEVENT_TYPE_I2C_ERROR
} ievent_type_t;

extern ievent_t ievents_buffer[IEVENT_BUFFER_SIZE];
extern uint16_t ievents_head;
extern uint16_t ievents_tail;
extern uint16_t ievents_count;

int add_ievent(int type, int value, uint32_t event_time);
