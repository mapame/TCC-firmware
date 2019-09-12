#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <i2c/i2c.h>

#include <esp8266.h>

#include <FreeRTOS.h>
#include <task.h>

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>

#include <sysparam.h>

#include <ds3231/ds3231.h>
#include "ads111x.h"

#define SCL_PIN 5
#define SDA_PIN 4
#define LED_R_PIN 2
#define LED_G_PIN 12
#define LED_B_PIN 13
#define BTN_PIN 15
#define READY_PIN 14

#define I2C_BUS 0

#define GAIN_ADC1 ADS111X_GAIN_0V256
#define GAIN_ADC2 ADS111X_GAIN_2V048
#define GAIN_ADC3 ADS111X_GAIN_2V048

#define RAW_ADC_DATA_BUFFER_SIZE 1000

#define PROCESSED_DATA_BUFFER_SIZE 241
#define EVENT_BUFFER_SIZE 61

#define RTC_UPDATE_PERIOD_S 12 * 3600
#define RTC_READ_PERIOD_US 30 * 1000000

#define SERVER_ADDR "192.168.1.50"
#define SERVER_PORT 2048

typedef struct power_data_s {
	uint32_t timestamp;
	uint32_t samples;
	uint32_t duration_usec;
	float vrms[3];
	float irms[3];
	float p[3];
} power_data_t;

typedef struct power_event_s {
	uint8_t type;
	uint8_t channel;
	uint32_t timestamp;
	uint32_t duration;
	float value;
} power_event_t;

typedef enum {
	POWER_EVENT_VOLTAGE_SAG,
	POWER_EVENT_VOLTAGE_SWELL,
	POWER_EVENT_OVERCURRENT,
	POWER_EVENT_FREQUENCY_VARIATION,
} power_event_type_t;

typedef enum {
	POWER_SINGLE_PHASE,
	POWER_TWO_PHASE,
	POWER_TWO_PHASE_COMBINED
} power_type_t;

typedef enum {
	SAMPLING_RUNNING = 0,
	SAMPLING_STOPPED_ON_ERROR,
	SAMPLING_STOPPED_ON_REQUEST
} sampling_state_t;

void inline read_rtc();

ads111x_dev_t adc_device[3];

i2c_dev_t rtc_dev = {.addr = DS3231_ADDR, .bus = 0};

sampling_state_t sampling_state;

power_type_t config_measurement_mode;
uint8_t config_line_frequency;
uint16_t config_adc_channel_switch_cycles;

uint32_t rtc_time, rtc_time_sysclock_reference;

uint16_t adc_channel_switch_period;
uint16_t adc_channel_counter;
uint8_t adc_actual_channel, adc_next_channel;

// adc0: v1(0), v2(1)
// adc1: i1(2)
// adc2: i2(3), i3(4)

int16_t raw_adc_data[5][RAW_ADC_DATA_BUFFER_SIZE];
uint32_t raw_adc_rtc_time[RAW_ADC_DATA_BUFFER_SIZE];
uint32_t raw_adc_usecs_past_time[RAW_ADC_DATA_BUFFER_SIZE];
uint16_t raw_adc_data_head, raw_adc_data_tail, raw_adc_data_count;

power_data_t processed_data[PROCESSED_DATA_BUFFER_SIZE];
uint16_t processed_data_head, processed_data_tail, processed_data_count;

char send_buffer[1024];

void IRAM ads_ready_handle(uint8_t gpio_num) {
	if(((raw_adc_data_head + 1) % RAW_ADC_DATA_BUFFER_SIZE) == raw_adc_data_tail) { // Stop sampling and throw buffer full error
		sampling_state = SAMPLING_STOPPED_ON_ERROR;
		return;
	}
	
	if(sampling_state == SAMPLING_RUNNING) {
		if(++adc_channel_counter >= adc_channel_switch_period) {
			adc_channel_counter = 0;
			adc_next_channel = (adc_next_channel + 1) % 2;
			ads111x_set_input_mux(&adc_device[0], (adc_next_channel == 0) ? ADS111X_MUX_0_1 : ADS111X_MUX_2_3);
			ads111x_set_input_mux(&adc_device[2], (adc_next_channel == 0) ? ADS111X_MUX_0_1 : ADS111X_MUX_2_3);
		}
		
		ads111x_start_conversion(&adc_device[0]);
		ads111x_start_conversion(&adc_device[1]);
		ads111x_start_conversion(&adc_device[2]);
	}
	
	uint32_t sysclock_actual_value = sdk_system_get_time();
	
	raw_adc_data[(adc_actual_channel == 0) ? 0 : 1][raw_adc_data_head] = ads111x_get_value(&adc_device[0]);
	raw_adc_data[2][raw_adc_data_head] = ads111x_get_value(&adc_device[1]);
	raw_adc_data[(adc_actual_channel == 0) ? 3 : 4][raw_adc_data_head] = ads111x_get_value(&adc_device[2]);
	
	uint16_t inactive_channel_buffer_position = raw_adc_data_head - adc_channel_switch_period + ((raw_adc_data_head - adc_channel_switch_period < 0) ? RAW_ADC_DATA_BUFFER_SIZE : 0);
	
	raw_adc_data[(adc_actual_channel == 0) ? 1 : 0][raw_adc_data_head] = raw_adc_data[(adc_actual_channel == 0) ? 1 : 0][inactive_channel_buffer_position];
	raw_adc_data[(adc_actual_channel == 0) ? 4 : 3][raw_adc_data_head] = raw_adc_data[(adc_actual_channel == 0) ? 4 : 3][inactive_channel_buffer_position];
	
	adc_actual_channel = adc_next_channel;
	
	raw_adc_data_head = (raw_adc_data_head + 1) % RAW_ADC_DATA_BUFFER_SIZE;
	raw_adc_data_count++;
	
	raw_adc_rtc_time[raw_adc_data_head] = rtc_time;
	
	if(sysclock_actual_value < rtc_time_sysclock_reference)
		raw_adc_usecs_past_time[raw_adc_data_head] = (((uint32_t)0xFFFFFFFF) - rtc_time_sysclock_reference) + sysclock_actual_value + ((uint32_t)1);
	else
		raw_adc_usecs_past_time[raw_adc_data_head] = sysclock_actual_value - rtc_time_sysclock_reference;
	
	if(raw_adc_usecs_past_time[raw_adc_data_head] >= RTC_READ_PERIOD_US)
		read_rtc();
}

void inline start_sampling() {
	raw_adc_data_head = 0;
	raw_adc_data_tail = 0;
	raw_adc_data_count = 0;
	adc_channel_counter = 0;
	adc_actual_channel = 0;
	adc_next_channel = 0;
	
	adc_channel_switch_period = 11; // Hardcoded starting value
	
	sampling_state = SAMPLING_RUNNING;
	
	ads111x_start_conversion(&adc_device[0]);
	ads111x_start_conversion(&adc_device[1]);
	ads111x_start_conversion(&adc_device[2]);
	
	read_rtc();
	raw_adc_rtc_time[raw_adc_data_head] = rtc_time;
	raw_adc_usecs_past_time[raw_adc_data_head] = 0;
}

void inline read_rtc() {
	struct tm time;
	
	//ds3231_getOscillatorStopFlag(&rtc_dev, &OSF);
	//ds3231_clearOscillatorStopFlag(&rtc_dev);
	
	ds3231_getTime(&rtc_dev, &time);
	rtc_time_sysclock_reference = sdk_system_get_time();
	rtc_time = mktime(&time);
}

void IRAM processing_task(void *pvParameters) {
	int16_t raw_adc_data_tmp[5];
	uint32_t raw_adc_usecs_tmp;
	uint32_t raw_adc_rtc_time_tmp;
	
	int raw_adc_data_processed_counter = 0;
	uint32_t first_sample_usecs;
	uint32_t first_sample_rtc_time;
	
	float v[3];
	float i[3];
	
	float vrms_acc[3] = {0.0, 0.0, 0.0};
	float irms_acc[3] = {0.0, 0.0, 0.0};
	float p_acc[3] = {0.0, 0.0, 0.0};
	
	processed_data_head = 0;
	processed_data_tail = 0;
	processed_data_count = 0;
	
	while(true) {
		if(!raw_adc_data_count) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}
		
		raw_adc_data_tmp[0] = raw_adc_data[0][raw_adc_data_tail];
		raw_adc_data_tmp[1] = raw_adc_data[1][raw_adc_data_tail];
		raw_adc_data_tmp[2] = raw_adc_data[2][raw_adc_data_tail];
		raw_adc_data_tmp[3] = raw_adc_data[3][raw_adc_data_tail];
		raw_adc_data_tmp[4] = raw_adc_data[4][raw_adc_data_tail];
		raw_adc_usecs_tmp = raw_adc_usecs_past_time[raw_adc_data_tail];
		raw_adc_rtc_time_tmp = raw_adc_rtc_time[raw_adc_data_tail];
		
		raw_adc_data_tail = (raw_adc_data_tail + 1) % RAW_ADC_DATA_BUFFER_SIZE;
		raw_adc_data_count--;
		
		if(raw_adc_data_processed_counter == 0) {
			first_sample_usecs = raw_adc_usecs_tmp;
			first_sample_rtc_time = raw_adc_rtc_time_tmp;
		}
		
		raw_adc_data_processed_counter++;
		
		v[0] = (ads111x_gain_values[GAIN_ADC1] / (float)ADS111X_MAX_VALUE * (float)raw_adc_data_tmp[0]) * 440000.0 / 120.0;
		v[1] = (ads111x_gain_values[GAIN_ADC1] / (float)ADS111X_MAX_VALUE * (float)raw_adc_data_tmp[1]) * 440000.0 / 120.0;
		v[2] = v[0] + v[1];
		
		i[0] = (ads111x_gain_values[GAIN_ADC2] / (float)ADS111X_MAX_VALUE * (float)raw_adc_data_tmp[2]) * 2000.0 / 23.2;
		i[1] = (ads111x_gain_values[GAIN_ADC3] / (float)ADS111X_MAX_VALUE * (float)raw_adc_data_tmp[3]) * 2000.0 / 23.2;
		i[2] = (ads111x_gain_values[GAIN_ADC3] / (float)ADS111X_MAX_VALUE * (float)raw_adc_data_tmp[4]) * 2000.0 / 23.2;
		
		vrms_acc[0] += v[0] * v[0];
		vrms_acc[1] += v[1] * v[1];
		vrms_acc[2] += v[2] * v[2];
		
		irms_acc[0] += i[0] * i[0];
		irms_acc[1] += i[1] * i[1];
		irms_acc[2] += i[2] * i[2];
		
		p_acc[0] += v[0] * i[0];
		p_acc[1] += v[1] * i[1];
		p_acc[2] += v[2] * i[2];
		
		if((raw_adc_usecs_tmp - first_sample_usecs) >= 250000 || first_sample_rtc_time != raw_adc_rtc_time_tmp) {
			processed_data[processed_data_head].timestamp = first_sample_rtc_time + first_sample_usecs / 1000000;
			processed_data[processed_data_head].duration_usec = raw_adc_usecs_tmp - first_sample_usecs;
			processed_data[processed_data_head].samples = raw_adc_data_processed_counter;
			processed_data[processed_data_head].vrms[0] = sqrtf(vrms_acc[0] / (float) raw_adc_data_processed_counter);
			processed_data[processed_data_head].vrms[1] = sqrtf(vrms_acc[1] / (float) raw_adc_data_processed_counter);
			processed_data[processed_data_head].vrms[2] = sqrtf(vrms_acc[2] / (float) raw_adc_data_processed_counter);
			
			processed_data[processed_data_head].irms[0] = sqrtf(irms_acc[0] / (float) raw_adc_data_processed_counter);
			processed_data[processed_data_head].irms[1] = sqrtf(irms_acc[1] / (float) raw_adc_data_processed_counter);
			processed_data[processed_data_head].irms[2] = sqrtf(irms_acc[2] / (float) raw_adc_data_processed_counter);
			
			processed_data[processed_data_head].p[0] = p_acc[0] / (float) raw_adc_data_processed_counter;
			processed_data[processed_data_head].p[1] = p_acc[1] / (float) raw_adc_data_processed_counter;
			processed_data[processed_data_head].p[2] = p_acc[2] / (float) raw_adc_data_processed_counter;
			
			processed_data_head = (processed_data_head + 1) % PROCESSED_DATA_BUFFER_SIZE;
			if(processed_data_head == processed_data_tail)
				processed_data_tail = (processed_data_tail + 1) % PROCESSED_DATA_BUFFER_SIZE;
			else
				processed_data_count++;
			
			vrms_acc[0] = vrms_acc[1] = vrms_acc[2] = 0.0;
			irms_acc[0] = irms_acc[1] = irms_acc[2] = 0.0;
			p_acc[0] = p_acc[1] = p_acc[2] = 0.0;
			
			raw_adc_data_processed_counter = 0;
		}
	}
}

void IRAM network_task(void *pvParameters) {
	int send_socket;
	struct sockaddr_in server_addr;
	
	start_sampling();
	
	vTaskDelay(pdMS_TO_TICKS(500));
	printf("Waiting for wireless connection...");
	
	while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	
	printf("Connected to network!\n");
	fflush(stdout);
	
	bzero(&server_addr, sizeof(server_addr));
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(SERVER_ADDR);
	server_addr.sin_port = htons(SERVER_PORT);
	
	while(true) {
		while(true) {
			send_socket = socket(PF_INET, SOCK_STREAM, 0);
			
			if(send_socket < 0) {
				printf("Failed to create socket.\n");
				vTaskDelay(pdMS_TO_TICKS(100));
				continue;
			}
			
			if(connect(send_socket, (struct sockaddr *) &server_addr, sizeof(server_addr)) == 0)
				break;
			
			printf("Failed to connect to server! (%d)\n", errno);
			close(send_socket);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		
		struct timeval socket_timeout_value = {.tv_sec = 2, .tv_usec = 0};
		if(setsockopt(send_socket, SOL_SOCKET, SO_SNDTIMEO, &socket_timeout_value, sizeof(socket_timeout_value)) < 0)
			fprintf(stderr, "Unable to set socket timeout value.\n");
		
		printf("Connected to server!\n");
		
		//sprintf(send_buffer, "Hello!\n");
		//send(send_socket, send_buffer, strlen(send_buffer), 0);
		
		power_data_t * current_processed_data;
		
		while (true) {
			if(!processed_data_count) {
				vTaskDelay(pdMS_TO_TICKS(500));
				continue;
			}
			current_processed_data = &processed_data[processed_data_tail];
			
			sprintf(send_buffer, "t %u s %u d %u v %.4f %.4f %.4f i %.4f %.4f %.4f p %.4f %.4f %.4f\n", current_processed_data->timestamp, current_processed_data->samples, current_processed_data->duration_usec,
																						  current_processed_data->vrms[0], current_processed_data->vrms[1], current_processed_data->vrms[2],
																						  current_processed_data->irms[0], current_processed_data->irms[1], current_processed_data->irms[2],
																						  current_processed_data->p[0], current_processed_data->p[1], current_processed_data->p[2]);
			
			if(send(send_socket, send_buffer, strlen(send_buffer), 0) > 0) {
				//printf("Data sent!\n");
				processed_data_tail = (processed_data_tail + 1) % PROCESSED_DATA_BUFFER_SIZE;
				processed_data_count--;
			} else {
				printf("Error sending data!\n");
				fflush(stdout);
				break;
			}
		}
		close(send_socket);
	}
}

void IRAM blink_task(void *pvParameters) {
	while(1){
		gpio_toggle(LED_B_PIN);
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

void adc_config() {
	ads111x_init(&adc_device[0], I2C_BUS, ADS111X_ADDR_GND);
	ads111x_init(&adc_device[1], I2C_BUS, ADS111X_ADDR_VCC);
	ads111x_init(&adc_device[2], I2C_BUS, ADS111X_ADDR_SDA);
	
	ads111x_set_data_rate(&adc_device[0], ADS111X_DATA_RATE_860);
	ads111x_set_data_rate(&adc_device[1], ADS111X_DATA_RATE_860);
	ads111x_set_data_rate(&adc_device[2], ADS111X_DATA_RATE_860);
	
	ads111x_set_input_mux(&adc_device[0], ADS111X_MUX_0_1);
	ads111x_set_input_mux(&adc_device[1], ADS111X_MUX_0_1);
	ads111x_set_input_mux(&adc_device[2], ADS111X_MUX_0_1);
	
	ads111x_set_gain(&adc_device[0], GAIN_ADC1);
	ads111x_set_gain(&adc_device[1], GAIN_ADC2);
	ads111x_set_gain(&adc_device[2], GAIN_ADC3);
	
	ads111x_set_comp_high_thresh(&adc_device[0], 0x8000);
	ads111x_set_comp_high_thresh(&adc_device[1], 0x8000);
	ads111x_set_comp_high_thresh(&adc_device[2], 0x8000);
	ads111x_set_comp_low_thresh(&adc_device[0], 0x7FFF);
	ads111x_set_comp_low_thresh(&adc_device[1], 0x7FFF);
	ads111x_set_comp_low_thresh(&adc_device[2], 0x7FFF);
	
	ads111x_set_comp_queue(&adc_device[0], ADS111X_COMP_QUEUE_1);
	ads111x_set_comp_queue(&adc_device[1], ADS111X_COMP_QUEUE_1);
	ads111x_set_comp_queue(&adc_device[2], ADS111X_COMP_QUEUE_1);
	
	ads111x_set_comp_polarity(&adc_device[0], ADS111X_COMP_POLARITY_HIGH);
	ads111x_set_comp_polarity(&adc_device[1], ADS111X_COMP_POLARITY_HIGH);
	ads111x_set_comp_polarity(&adc_device[2], ADS111X_COMP_POLARITY_HIGH);
	
	ads111x_push_config(&adc_device[0]);
	ads111x_push_config(&adc_device[1]);
	ads111x_push_config(&adc_device[2]);
}

void user_init(void) {
	char *wifi_ssid;
	char *wifi_password;
	
	uart_set_baud(0, 115200);
	
	gpio_enable(READY_PIN, GPIO_INPUT);
	gpio_enable(LED_R_PIN, GPIO_OUTPUT);
	gpio_enable(LED_G_PIN, GPIO_OUTPUT);
	gpio_enable(LED_B_PIN, GPIO_OUTPUT);
	gpio_enable(BTN_PIN, GPIO_INPUT);
	
	for(int i = 1; i < 17; i++) {
		gpio_write(LED_R_PIN, i & 1);
		gpio_write(LED_G_PIN, ((i >> 1) & 1) == 0);
		gpio_write(LED_B_PIN, ((i >> 2) & 1) == 0);
		
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	
	if(sysparam_get_string("wifi_ap_ssid", &wifi_ssid) != SYSPARAM_OK || strlen(wifi_ssid) < 1
	|| sysparam_get_string("wifi_ap_password", &wifi_password) != SYSPARAM_OK || strlen(wifi_password) < 8) {
		printf("WiFi data not avaliable or invalid.\n");
		return;
	}
	
	struct sdk_station_config wifi_config;
	
	strcpy((char *)wifi_config.ssid, wifi_ssid);
	strcpy((char *)wifi_config.password, wifi_password);
	
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&wifi_config);
	
	vTaskDelay(pdMS_TO_TICKS(100));
	
	printf("Initializing I2C bus.\n");
	
	if(i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_500K)) {
		printf("Failed to initialize i2c bus!\n");
		return;
	}
	
	printf("Configuring ADCs.\n");
	
	adc_config();
	
	gpio_set_interrupt(READY_PIN, GPIO_INTTYPE_EDGE_POS, ads_ready_handle);
	
	printf("Starting send task.\n");
	
	xTaskCreate(processing_task, "processing_task", 512, NULL, 3, NULL);
	xTaskCreate(network_task, "network_task", 512, NULL, 2, NULL);
	xTaskCreate(blink_task, "blink_task", 256, NULL, 1, NULL);
}
