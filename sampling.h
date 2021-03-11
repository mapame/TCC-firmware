#define GAIN_ADC1 ADS111X_GAIN_0V256
#define GAIN_ADC2 ADS111X_GAIN_2V048
#define GAIN_ADC3 ADS111X_GAIN_2V048

#define RAW_ADC_DATA_BUFFER_SIZE 500

#define RAW_ADC_HISTORY_BUFFER_SIZE 80

extern float adc_volt_scale[3];

// adc0: v1(0), v2(1)
// adc1: i1(2)
// adc2: i2(3)

typedef struct raw_adc_data_s {
	uint32_t rtc_time;
	uint32_t usecs_since_time;
	uint32_t errors;
	int16_t data[4];
} raw_adc_data_t;

extern MessageBufferHandle_t raw_adc_data_buffer;
extern volatile uint16_t raw_adc_data_count;

void IRAM ads_ready_handle(uint8_t gpio_num);
int start_sampling();
void pause_sampling();
int adc_config();
