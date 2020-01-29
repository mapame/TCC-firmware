#define GAIN_ADC1 ADS111X_GAIN_0V256
#define GAIN_ADC2 ADS111X_GAIN_2V048
#define GAIN_ADC3 ADS111X_GAIN_2V048

#define RAW_ADC_DATA_BUFFER_SIZE 500

#define RAW_ADC_HISTORY_BUFFER_SIZE 250

extern float adc_volt_scale[3];

// adc0: v1(0), v2(1)
// adc1: i1(2)
// adc2: i2(3), i3(4)

typedef struct raw_adc_data_s {
	uint32_t rtc_time;
	uint32_t usecs_since_time;
	int16_t data[5];
} raw_adc_data_t;

extern MessageBufferHandle_t raw_adc_data_buffer;
extern uint16_t raw_adc_data_count;

extern int16_t raw_adc_history_buffer[5][RAW_ADC_HISTORY_BUFFER_SIZE];
extern int16_t raw_adc_history_buffer_pos;

void IRAM ads_ready_handle(uint8_t gpio_num);
void start_sampling();
void pause_sampling();
void adc_config();
