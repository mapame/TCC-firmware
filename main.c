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

#include "bearssl.h"

#include <time.h>
#include "ds3231/ds3231.h"
#include "ads111x.h"

#include "common.h"
#include "configuration.h"
#include "ota.h"

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

typedef struct internal_event_s {
	uint8_t type;
	uint32_t timestamp;
} internal_event_t;

typedef struct opcode_metadata_s {
	char opcode_text[3];
	int parameter_qty;
} opcode_metadata_t;

typedef enum {
	OP_PROTOCOL_START,
	OP_SAMPLING_START,
	OP_SAMPLING_PAUSE,
	OP_CONFIG_WRITE,
	OP_CONFIG_READ,
	OP_RESTART,
	OP_FW_UPDATE,
	OP_QUERY_STATUS,
	OP_GET_DATA,
	OP_DELETE_DATA,
	OP_GET_WAVEFORM,
	OP_DISCONNECT,
	OPCODE_NUM
} protocol_opcodes_t;

typedef enum {
	R_SUCESS,
	R_ERR_UNSPECIFIED,
	R_ERR_SAMPLING_RUNNING,
	R_ERR_INVALID_PARAMETER
} protocol_errors_t;

typedef enum {
	POWER_SINGLE_PHASE,
	POWER_TWO_PHASE,
	POWER_TWO_PHASE_COMBINED
} measurement_type_t;

typedef enum {
	POWER_EVENT_VOLTAGE_SAG,
	POWER_EVENT_VOLTAGE_SWELL,
	POWER_EVENT_OVERCURRENT,
	POWER_EVENT_FREQUENCY_VARIATION,
} power_event_type_t;

typedef enum {
	INTERNAL_EVENT_BUFFER_FULL,
	INTERNAL_EVENT_LOW_SAMPLING_FREQUENCY,
	INTERNAL_EVENT_SAMPLING_STOPPED,
	INTERNAL_EVENT_SEND_TIMEOUT,
	INTERNAL_EVENT_RESPONSE_TIMEOUT,
	INTERNAL_EVENT_INVALID_MAC,
	INTERNAL_EVENT_I2C_ERROR,
} internal_event_type_t;

typedef enum {
	SAMPLING_RUNNING = 0,
	SAMPLING_PAUSED_ON_ERROR,
	SAMPLING_PAUSED_ON_REQUEST
} sampling_state_t;

void inline read_rtc();
static int recv_command_line(int s, char *buf, size_t len);
static uint8_t convert_opcode(char *buf);
static void compute_hmac(const br_hmac_key_context *hmac_key_ctx, char *output_mac_text, const char *data, size_t len);
static int validate_hmac(const br_hmac_key_context *hmac_key_ctx, char *data, size_t len);
static int send_response(int socket_fd, const br_hmac_key_context *hmac_key_ctx, uint8_t opcode, uint32_t timestamp, uint32_t counter, uint8_t response_code, char *parameters);

TaskHandle_t processing_task_handle, blink_task_handle;

ads111x_dev_t adc_device[3];

i2c_dev_t rtc_dev = {.addr = DS3231_ADDR, .bus = 0};

opcode_metadata_t opcode_metadata_list[OPCODE_NUM] = {
	{"HE", 0},
	{"SS", 0},
	{"SP", 0},
	{"CW", 2},
	{"CR", 1},
	{"RE", 0},
	{"FU", 1},
	{"QS", 0},
	{"GD", 3},
	{"DD", 3},
	{"GW", 2},
	{"BY", 0}
};

uint8_t sampling_running;

uint32_t rtc_time, rtc_time_sysclock_reference;
uint8_t rtc_oscillator_stopped;

uint32_t rtc_new_time, rtc_new_time_sysclock_reference;

uint16_t adc_channel_switch_period;
uint16_t adc_channel_counter;
uint8_t adc_actual_channel, adc_next_channel;

// adc0: v1(0), v2(1)
// adc1: i1(2)
// adc2: i2(3), i3(4)

int16_t raw_adc_data[5][RAW_ADC_DATA_BUFFER_SIZE];
uint32_t raw_adc_rtc_time[RAW_ADC_DATA_BUFFER_SIZE];
uint32_t raw_adc_usecs_since_time[RAW_ADC_DATA_BUFFER_SIZE];
uint16_t raw_adc_data_head, raw_adc_data_tail, raw_adc_data_count;

power_data_t processed_data[PROCESSED_DATA_BUFFER_SIZE];
uint16_t processed_data_head, processed_data_tail, processed_data_count;

power_event_t power_events[POWER_EVENT_BUFFER_SIZE];
uint16_t power_events_data_head, power_events_data_tail, power_events_data_count;

power_event_t internal_events[INTERNAL_EVENT_BUFFER_SIZE];
uint16_t internal_events_data_head, internal_events_data_tail, internal_events_data_count;

char received_ota_hash_text[33];

void IRAM ads_ready_handle(uint8_t gpio_num) {
	if(((raw_adc_data_head + 1) % RAW_ADC_DATA_BUFFER_SIZE) == raw_adc_data_tail) { // Stop sampling on buffer full error
		sampling_running = 0;
		debug("Raw buffer full!\n");
		return;
	}
	
	if(sampling_running) {
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
		raw_adc_usecs_since_time[raw_adc_data_head] = (((uint32_t)0xFFFFFFFF) - rtc_time_sysclock_reference) + sysclock_actual_value + ((uint32_t)1);
	else
		raw_adc_usecs_since_time[raw_adc_data_head] = sysclock_actual_value - rtc_time_sysclock_reference;
	
	if(raw_adc_usecs_since_time[raw_adc_data_head] >= RTC_READ_PERIOD_US)
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
	
	sampling_running = 1;
	
	ads111x_start_conversion(&adc_device[0]);
	ads111x_start_conversion(&adc_device[1]);
	ads111x_start_conversion(&adc_device[2]);
	
	read_rtc();
	raw_adc_rtc_time[raw_adc_data_head] = rtc_time;
	raw_adc_usecs_since_time[raw_adc_data_head] = 0;
}

void inline pause_sampling() {
	sampling_running = 0;
	vTaskDelay(pdMS_TO_TICKS(50));
}

void inline read_rtc() {
	struct tm time;
	bool osf;
	
	ds3231_getOscillatorStopFlag(&rtc_dev, &osf);
	
	if(osf) {
		rtc_oscillator_stopped++;
		return;
	}
	
	//ds3231_clearOscillatorStopFlag(&rtc_dev);
	
	ds3231_getTime(&rtc_dev, &time);
	rtc_time_sysclock_reference = sdk_system_get_time();
	rtc_time = mktime(&time);
}

void processing_task(void *pvParameters) {
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
		raw_adc_usecs_tmp = raw_adc_usecs_since_time[raw_adc_data_tail];
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

void network_task(void *pvParameters) {
	int socket_fd;
	struct sockaddr_in server_addr;
	int last_errno;
	
	start_sampling();
	
	vTaskDelay(pdMS_TO_TICKS(500));
	debug("Waiting for wireless connection...");
	
	while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	
	debug("Connected to network!\n");
	fflush(stdout);
	
	bzero(&server_addr, sizeof(server_addr));
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(config_server_ip);
	server_addr.sin_port = htons(SERVER_PORT);
	
	while(true) {
		last_errno = 0;
		while(true) {
			socket_fd = socket(PF_INET, SOCK_STREAM, 0);
			
			if(socket_fd < 0) {
				debug("Failed to create socket.\n");
				vTaskDelay(pdMS_TO_TICKS(200));
				continue;
			}
			
			if(connect(socket_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) == 0)
				break;
			
			if(errno != last_errno) {
				last_errno = errno;
				debug("Failed to connect to server, error %d", last_errno);
			}
			
			debug(".");
			close(socket_fd);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		
		debug("\nConnected to server!\n");
		
		struct timeval socket_send_timeout_value = {.tv_sec = 2, .tv_usec = 0};
		struct timeval socket_receive_timeout_value = {.tv_sec = 3, .tv_usec = 0};
		
		setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &socket_send_timeout_value, sizeof(socket_send_timeout_value));
		//setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &socket_receive_timeout_value, sizeof(socket_receive_timeout_value));
		
		br_hmac_key_context hmac_key_ctx;
		
		power_data_t *current_processed_data;
		
		char aux[100];
		
		char receive_buffer[200];
		int received_line_len;
		
		int send_result = -1;
		
		char *token;
		char *saveptr;
		
		uint8_t received_opcode;
		char *received_parameters[5];
		uint32_t received_timestamp;
		uint32_t received_counter;
		
		int parameter_parse_result;
		
		char response_parameters[200];
		int response_code;
		
		int protocol_started = 0;
		
		int16_t waveform_buffer[WAVEFORM_MAX_QTY];
		int input_channel, waveform_qty;
		uint16_t raw_adc_start;
		
		unsigned int disconnection_time;
		
		br_hmac_key_init(&hmac_key_ctx, &br_md5_vtable, config_mac_password, strlen(config_mac_password));
		
		while (protocol_started >= 0) {
			received_line_len = recv_command_line(socket_fd, receive_buffer, sizeof(receive_buffer));
			
			if(received_line_len <= 0) // Timeout or disconnection
				break;
			
			if(received_line_len < 35) // Protocol error - Line too small
				break;
			
			if(validate_hmac(&hmac_key_ctx, receive_buffer, received_line_len)) // Protocol error - Invalid MAC
				break;
			
			token = strtok_r(receive_buffer, ":", &saveptr); // Opcode
			if(!token) // Protocol error - Syntax Error
				break;
			
			received_opcode = convert_opcode(token);
			
			if(received_opcode < 0) // Protocol error - Invalid opcode
				break;
			
			if(!protocol_started && received_opcode != OP_PROTOCOL_START)
				break;
			
			token = strtok_r(NULL, ":", &saveptr); // Timestamp
			if(!token || strlen(token) > 15) // Protocol error - Syntax Error
				break;
			
			received_timestamp = atoi(token);
			
			token = strtok_r(NULL, ":", &saveptr); // Counter
			if(!token || strlen(token) > 15) // Protocol error - Syntax Error
				break;
			
			received_counter = atoi(token);
			
			for(int i = 0; i < opcode_metadata_list[received_opcode].parameter_qty; i++) {
				token = strtok_r(NULL, "\t", &saveptr);
				if(!token || strlen(token) >= 64)
					break;
				
				received_parameters[i] = token;
			}
			
			debug("Opcode: %s\n", opcode_metadata_list[received_opcode].opcode_text);
			debug("Timestamp: %d\n", received_timestamp);
			debug("Counter: %d\n", received_counter);
			
			response_code = R_SUCESS;
			response_parameters[0] = '\0';
			
			switch(received_opcode) {
				case OP_PROTOCOL_START:
					if(protocol_started) {
						response_code = R_ERR_UNSPECIFIED;
						break;
					}
					
					protocol_started = 1;
					sprintf(response_parameters, "%d\t%s\t%s\t", FW_TYPE, config_device_id, FW_VERSION);
					break;
				case OP_SAMPLING_START:
					if(!sampling_running)
						start_sampling();
					
					break;
				case OP_SAMPLING_PAUSE:
					if(sampling_running)
						pause_sampling();
					
					break;
				case OP_CONFIG_WRITE:
					if(sampling_running) {
						response_code = R_ERR_SAMPLING_RUNNING;
						break;
					}
					
					if(configuration_write(received_parameters[0], received_parameters[1], 1))
						response_code = R_ERR_UNSPECIFIED;
					
					break;
				case OP_CONFIG_READ:
					if(configuration_read(received_parameters[0], aux)) {
						response_code = R_ERR_UNSPECIFIED;
						break;
					}
					
					sprintf(response_parameters, "%s\t", aux);
					
					break;
				case OP_RESTART:
					break;
				case OP_FW_UPDATE:
					if(sampling_running) {
						response_code = R_ERR_SAMPLING_RUNNING;
						break;
					}
					
					if(strlen(received_parameters[0]) != 32) {
						response_code = R_ERR_INVALID_PARAMETER;
						break;
					}
					
					strlcpy(received_ota_hash_text, received_parameters[0], 33);
					break;
				case OP_QUERY_STATUS:
					sprintf(response_parameters, "%u\t%u\t%u\t%u\t", sampling_running, 25, rtc_oscillator_stopped, processed_data_count);
					
					break;
				case OP_GET_DATA:
					
					break;
				case OP_DELETE_DATA:
					if(sampling_running && *received_parameters[1] == 'f') {
						response_code = R_ERR_SAMPLING_RUNNING;
						break;
					}
					
					break;
				case OP_GET_WAVEFORM:
					if(!sampling_running) {
						response_code = R_ERR_UNSPECIFIED;
						break;
					}
					
					parameter_parse_result = 0;
					parameter_parse_result += sscanf(received_parameters[0], "%d", &input_channel);
					parameter_parse_result += sscanf(received_parameters[1], "%d", &waveform_qty);
					
					if(parameter_parse_result != 2 || waveform_qty < 1 || waveform_qty > WAVEFORM_MAX_QTY || input_channel < 1 || input_channel > 5) {
						response_code = R_ERR_INVALID_PARAMETER;
						break;
					}
					
					raw_adc_start = raw_adc_data_head;
					vTaskDelay(pdMS_TO_TICKS(100));
					
					for(int i = 0; i < waveform_qty; i++)
						waveform_buffer[i] = raw_adc_data[input_channel][(raw_adc_start + i) % RAW_ADC_DATA_BUFFER_SIZE];
					
					for(int i = 0; i < waveform_qty; i++) {
						sprintf(response_parameters, "%i\t", waveform_buffer[i]);
						send_result = send_response(socket_fd, &hmac_key_ctx, received_opcode, received_timestamp, received_counter, response_code, response_parameters);
						if(send_result < 0)
							break;
					}
					
					break;
				case OP_DISCONNECT:
					if(sscanf(received_parameters[0], "%u", &disconnection_time) != 1) {
						response_code = R_ERR_INVALID_PARAMETER;
						break;
					}
					
					disconnection_time = MIN(disconnection_time, MAX_DISCONNECTION_TIME_MS);
					disconnection_time = MAX(disconnection_time, MIN_DISCONNECTION_TIME_MS);
					break;
				default:
					break;
			}
			
			debug("Response %d - Parameters: %s\n", response_code, response_parameters);
			
			if((received_opcode != OP_GET_DATA && received_opcode != OP_GET_WAVEFORM) || response_code != R_SUCESS)
				send_result = send_response(socket_fd, &hmac_key_ctx, received_opcode, received_timestamp, received_counter, response_code, response_parameters);
			
			if(send_result < 0) {
				debug("Failed to send response.\n");
				break;
			}
			
			debug("Done processing command.\n");
			
			if(response_code != R_SUCESS)
				continue;
			
			switch(received_opcode) {
				case OP_RESTART:
					vTaskDelay(pdMS_TO_TICKS(200));
					close(socket_fd);
					vTaskDelay(pdMS_TO_TICKS(1000));
					sdk_system_restart();
					break;
				case OP_FW_UPDATE:
					vTaskDelete(processing_task_handle);
					vTaskDelete(blink_task_handle);
					vTaskDelay(pdMS_TO_TICKS(500));
					debug("Starting OTA task.\n");
					if(xTaskCreate(ota_task, "ota_task", 1280, (void*) &received_ota_hash_text, 4, NULL) != pdPASS) {
						debug("Failed to create OTA task. Restarting...\n");
						vTaskDelay(pdMS_TO_TICKS(1000));
						sdk_system_restart();
					}
					vTaskDelete(NULL);
					break;
				case OP_DISCONNECT:
					shutdown(socket_fd, SHUT_RDWR);
					vTaskDelay(pdMS_TO_TICKS(disconnection_time));
					protocol_started = -1;
					break;
				default:
					break;
			}
		}
		
		/*
		current_processed_data = &processed_data[processed_data_tail];
		
		sprintf(send_buffer, "t %u s %u d %u v %.4f %.4f %.4f i %.4f %.4f %.4f p %.4f %.4f %.4f\n", current_processed_data->timestamp, current_processed_data->samples, current_processed_data->duration_usec,
																					  current_processed_data->vrms[0], current_processed_data->vrms[1], current_processed_data->vrms[2],
																					  current_processed_data->irms[0], current_processed_data->irms[1], current_processed_data->irms[2],
																					  current_processed_data->p[0], current_processed_data->p[1], current_processed_data->p[2]);
		
		if(send(socket_fd, send_buffer, strlen(send_buffer), 0) > 0) {
			//printf("Data sent!\n");
			processed_data_tail = (processed_data_tail + 1) % PROCESSED_DATA_BUFFER_SIZE;
			processed_data_count--;
		} else {
			printf("Error sending data!\n");
			fflush(stdout);
			break;
		}
		*/
		
		debug("Closing socket.\n");
		close(socket_fd);
	}
}

static int recv_command_line(int socket_fd, char *buf, size_t len) {
	int num = 0;
	
	do {
		char c;
		
		if (recv(socket_fd, &c, 1, 0) <= 0)
			return -1;
		
		if (c == '\n')
			break;
		
		if (num < len)
            buf[num] = c;
		
		num++;
	} while(1);
	
	buf[(num >= len) ? len - 1 : num] = 0; // Null terminate
	
	return num;
}

static uint8_t convert_opcode(char *buf) {
	if(!buf)
		return -1;
	
	for(int i = 0; i < OPCODE_NUM; i++)
		if(!strcmp(buf, opcode_metadata_list[i].opcode_text))
			return i;
	
	return -1;
}

static void compute_hmac(const br_hmac_key_context *hmac_key_ctx, char *output_mac_text, const char *data, size_t len) {
	br_hmac_context hmac_ctx;
	uint8_t computed_mac[16];
	char aux[3];
	
	br_hmac_init(&hmac_ctx, hmac_key_ctx, 0);
	
	br_hmac_update(&hmac_ctx, data, len);
	
	br_hmac_out(&hmac_ctx, computed_mac);
	
	output_mac_text[0] = '\0';
	
	for(int i = 0; i < 16; i++) {
		sprintf(aux, "%02hx", computed_mac[i]);
		strlcat(output_mac_text, aux, 33);
	}
}

static int validate_hmac(const br_hmac_key_context *hmac_key_ctx, char *data, size_t len) {
	char *received_mac_text;
	char computed_mac_text[33];
	
	if(data[len - 33] != '*') // Protocol error - Syntax Error
		return -1;
	
	data[len - 33] = '\0';
	
	received_mac_text = &(data[len - 32]);
	
	if(strlen(received_mac_text) != 32)
		return -1;
	
	//debug("Message: %s\n", data);
	
	//debug("Received HMAC:   %s\n", received_mac_text);
	
	compute_hmac(hmac_key_ctx, computed_mac_text, data, len - 33);
	
	//debug("Calculated HMAC: %s\n", computed_mac_text);
	
	if(strcmp(received_mac_text, computed_mac_text)) // Protocol error - Invalid MAC
		return -2;
	
	return 0;
}

static int send_response(int socket_fd, const br_hmac_key_context *hmac_key_ctx, uint8_t opcode, uint32_t timestamp, uint32_t counter, uint8_t response_code, char *parameters) {
	char aux[36];
	char send_buffer[200];
	char computed_mac_text[33];
	
	sprintf(send_buffer, "A:%s:%d:%d:%u:", opcode_metadata_list[opcode].opcode_text, timestamp, counter, response_code);
	
	if(parameters)
		strlcat(send_buffer, parameters, 200);
	
	compute_hmac(hmac_key_ctx, computed_mac_text, send_buffer, strlen(send_buffer));
	
	sprintf(aux, "*%s\n", computed_mac_text);
	strlcat(send_buffer, aux, 200);
	
	return send(socket_fd, send_buffer, strlen(send_buffer), 0);
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
	
	debug("Firmware version: "FW_VERSION"\n");
	debug("Build date: "__DATE__" "__TIME__"\n");
	
	load_configuration();
	
	if(strlen(config_wifi_ssid) < 1 || strcmp(config_wifi_ssid, "###NOT_SET###") == 0
	|| strlen(config_wifi_password) < 1 || strcmp(config_wifi_password, "###NOT_SET###") == 0
	|| strlen(config_mac_password) < 1 || strcmp(config_mac_password, "###NOT_SET###") == 0) {
		debug("Device not configured.\n");
		return;
	}
	
	struct sdk_station_config wifi_config;
	
	strcpy((char *)wifi_config.ssid, config_wifi_ssid);
	strcpy((char *)wifi_config.password, config_wifi_password);
	
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&wifi_config);
	
	vTaskDelay(pdMS_TO_TICKS(100));
	
	debug("Initializing I2C bus.\n");
	
	if(i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_500K)) {
		debug("Failed to initialize i2c bus!\n");
		return;
	}
	
	debug("Configuring ADCs.\n");
	
	adc_config();
	
	gpio_set_interrupt(READY_PIN, GPIO_INTTYPE_EDGE_POS, ads_ready_handle);
	
	debug("Starting tasks.\n");
	
	xTaskCreate(processing_task, "processing_task", 512, NULL, 3, &processing_task_handle);
	xTaskCreate(network_task, "network_task", 768, NULL, 2, NULL);
	xTaskCreate(blink_task, "blink_task", 256, NULL, 1, &blink_task_handle);
}
