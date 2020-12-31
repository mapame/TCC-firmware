#define POWER_DATA_BUFFER_SIZE 121

typedef struct power_data_s {
	uint32_t timestamp;
	uint32_t samples;
	uint32_t duration_usec;
	float vrms[2];
	float irms[2];
	float p[2];
} power_data_t;

extern uint16_t power_data_count;

void power_processing_task(void *pvParameters);
int get_power_data(power_data_t *data, unsigned int index);
int delete_power_data(unsigned int qty);
void get_waveform(float *buffer, unsigned int channel, unsigned int qty);
