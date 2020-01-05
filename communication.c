#include <espressif/esp_common.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>

#include "bearssl.h"

#include "common.h"
#include "sampling.h"
#include "power.h"
#include "communication.h"
#include "configuration.h"
#include "rtc.h"
#include "ota.h"

extern TaskHandle_t power_processing_task_handle, blink_task_handle;

static int recv_command_line(int s, char *buf, size_t len);
static uint8_t convert_opcode(char *buf);
static void compute_hmac(const br_hmac_key_context *hmac_key_ctx, char *output_mac_text, const char *data, size_t len);
static int validate_hmac(const br_hmac_key_context *hmac_key_ctx, char *data, size_t len);
static int send_response(int socket_fd, const br_hmac_key_context *hmac_key_ctx, uint8_t opcode, uint32_t timestamp, uint32_t counter, uint8_t response_code, char *parameters);

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

char received_ota_hash_text[33];

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
					vTaskDelete(power_processing_task_handle);
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
