#define GAIN_ADC1 ADS111X_GAIN_0V256
#define GAIN_ADC2 ADS111X_GAIN_2V048
#define GAIN_ADC3 ADS111X_GAIN_2V048

#define RAW_ADC_DATA_BUFFER_SIZE 1000

extern uint8_t sampling_running;

extern float adc_volt_scale[3];

// adc0: v1(0), v2(1)
// adc1: i1(2)
// adc2: i2(3), i3(4)
extern int16_t raw_adc_data[5][RAW_ADC_DATA_BUFFER_SIZE];
extern uint32_t raw_adc_rtc_time[RAW_ADC_DATA_BUFFER_SIZE];
extern uint32_t raw_adc_usecs_since_time[RAW_ADC_DATA_BUFFER_SIZE];
extern uint16_t raw_adc_data_head, raw_adc_data_tail, raw_adc_data_count;

void IRAM ads_ready_handle(uint8_t gpio_num);
void start_sampling();
void pause_sampling();
void adc_config();
