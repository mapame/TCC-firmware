#define EVENT_BUFFER_SIZE 100

typedef struct event_s {
	uint32_t timestamp;
	char text[32];
} event_t;

extern uint16_t event_count;

int events_init();

int add_event(const char *event_text, uint32_t event_time);
int get_event(event_t *data, unsigned int index);
int delete_events(unsigned int qty);
