#include <espressif/esp_common.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <message_buffer.h>
#include <semphr.h>

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
#include "ievents.h"
#include "rtc.h"
#include "ota.h"

extern TaskHandle_t power_processing_task_handle, blink_task_handle;

static int recv_command_line(int s, char *buf, size_t len);
static int convert_opcode(char *buf);
static void compute_hmac(const br_hmac_key_context *hmac_key_ctx, char *output_mac_text, const char *data, size_t len);
static int validate_hmac(const br_hmac_key_context *hmac_key_ctx, char *data, size_t len);

static int receive_command(int socket_fd, const br_hmac_key_context *hmac_key_ctx, int *op, uint32_t *received_timestamp, uint32_t counter, char parameters[][PARAM_STR_SIZE]);
static int parse_command(char *receive_buffer, int *op, uint32_t *timestamp, uint32_t *counter, char **parameters);
static int parse_parameters(char *parameter_buffer, char parsed_parameters[][PARAM_STR_SIZE], unsigned int parameter_qty);
static int send_response(int socket_fd, const br_hmac_key_context *hmac_key_ctx, int opcode, uint32_t timestamp, uint32_t counter, int response_code, char *parameters);

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
	{"BY", 1}
};

char received_ota_hash_text[33];

void network_task(void *pvParameters) {
	int socket_fd;
	struct sockaddr_in server_addr;
	
	vTaskDelay(pdMS_TO_TICKS(500));
	debug("Waiting for wireless connection...");
	
	while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	
	debug("Connected to network!\n");
	#ifdef DEBUG
		fflush(stdout);
	#endif
	
	bzero(&server_addr, sizeof(server_addr));
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(config_server_ip);
	server_addr.sin_port = htons(SERVER_PORT);
	
	while(true) {
		int last_errno = 0;
		while(true) {
			socket_fd = socket(PF_INET, SOCK_STREAM, 0);
			
			if(socket_fd < 0) {
				debug("Failed to create socket.\n");
				vTaskDelay(pdMS_TO_TICKS(500));
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
		
		status_server_connected = 1;
		
		debug("\nConnected to server!\n");
		
		struct timeval socket_send_timeout_value = {.tv_sec = 2, .tv_usec = 0};
		struct timeval socket_receive_timeout_value = {.tv_sec = 3, .tv_usec = 0};
		
		setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &socket_send_timeout_value, sizeof(socket_send_timeout_value));
		setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &socket_receive_timeout_value, sizeof(socket_receive_timeout_value));
		
		br_hmac_key_context hmac_key_ctx;
		
		uint32_t command_counter = 0;
		
		int protocol_started = 0;
		
		br_hmac_key_init(&hmac_key_ctx, &br_md5_vtable, config_mac_password, strlen(config_mac_password));
		
		while (protocol_started >= 0) {
			int received_opcode;
			uint32_t received_timestamp;
			char received_parameters[PARAM_MAX_QTY][PARAM_STR_SIZE];
			
			int parameter_parse_result;
			
			power_data_t aux_power_data;
			power_event_t aux_power_event;
			ievent_t aux_ievent;
			
			unsigned int aux_channel, aux_qty;
			float aux_waveform_buffer[WAVEFORM_MAX_QTY];
			
			unsigned int disconnection_time;
			
			int response_code;
			char response_parameters[200];
			
			int send_result;
			
			if(receive_command(socket_fd, &hmac_key_ctx, &received_opcode, &received_timestamp, command_counter, received_parameters))
				break;
			
			if(!protocol_started && received_opcode != OP_PROTOCOL_START)
				break;
			
			debug("Opcode: %s\n", opcode_metadata_list[received_opcode].opcode_text);
			debug("Timestamp: %u\n", received_timestamp);
			debug("Counter: %u\n", command_counter);
			
			response_code = R_SUCESS;
			response_parameters[0] = '\0';
			
			send_result = -1;
			
			switch(received_opcode) {
				case OP_PROTOCOL_START:
					if(protocol_started) {
						response_code = R_ERR_UNSPECIFIED;
						break;
					}
					
					if(get_time() == 0 && !status_sampling_running)
						update_rtc(received_timestamp);
					
					protocol_started = 1;
					sprintf(response_parameters, "%d\t%s\t%s\t%d\t", FW_TYPE, config_device_id, FW_VERSION, CONFIG_NUMBER);
					break;
				case OP_SAMPLING_START:
					if(!status_sampling_running) {
						update_rtc(received_timestamp);
						start_sampling();
					}
					
					break;
				case OP_SAMPLING_PAUSE:
					if(status_sampling_running)
						pause_sampling();
					
					break;
				case OP_CONFIG_WRITE:
					if(status_sampling_running) {
						response_code = R_ERR_SAMPLING_RUNNING;
						break;
					}
					
					if(configuration_write(received_parameters[0], received_parameters[1], 1))
						response_code = R_ERR_UNSPECIFIED;
					
					break;
				case OP_CONFIG_READ:
					if(configuration_read(received_parameters[0], response_parameters)) {
						response_code = R_ERR_UNSPECIFIED;
						break;
					}
					
					strcat(response_parameters, "\t");
					
					break;
				case OP_RESTART:
					break;
				case OP_FW_UPDATE:
					if(status_sampling_running) {
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
					sprintf(response_parameters, "%u\t%u\t%.2f\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t", status_sampling_running, (xTaskGetTickCount() / configTICK_RATE_HZ), get_temp(), get_time(), rtc_oscillator_stopped, ievents_count, 0, power_events_data_count, 0, processed_data_count, 0);
					rtc_oscillator_stopped = 0;
					
					break;
				case OP_GET_DATA:
					if(sscanf(received_parameters[2], "%u", &aux_qty) != 1) {
						response_code = R_ERR_INVALID_PARAMETER;
						break;
					}
					
					if(strcmp(received_parameters[0], "pd") == 0) {
						if(*received_parameters[1] == 'r') {
							if(aux_qty > processed_data_count) {
								response_code = R_ERR_INVALID_PARAMETER;
								break;
							}
							
							for(int i = 0; i < aux_qty; i++) {
								get_power_data(&aux_power_data, i);
								
								sprintf(response_parameters, "%u\t%u\t%u\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t", aux_power_data.timestamp, aux_power_data.samples, aux_power_data.duration_usec,
																																	aux_power_data.vrms[0], aux_power_data.vrms[1],
																																	aux_power_data.irms[0], aux_power_data.irms[1],
																																	aux_power_data.p[0], aux_power_data.p[1]);
								
								send_result = send_response(socket_fd, &hmac_key_ctx, received_opcode, received_timestamp, command_counter, R_SUCESS, response_parameters);
								if(send_result < 0)
									break;
							}
						} else {
							response_code = R_ERR_INVALID_PARAMETER;
							break;
						}
					} else if(strcmp(received_parameters[0], "pe") == 0) {
						if(*received_parameters[1] == 'r') {
							if(aux_qty > power_events_data_count) {
								response_code = R_ERR_INVALID_PARAMETER;
								break;
							}
							
							for(int i = 0; i < aux_qty; i++) {
								get_power_events(&aux_power_event, i);
								
								sprintf(response_parameters, "%u\t%u\t%u\t%u\t%.3f\t%.3f\t", aux_power_event.timestamp, aux_power_event.type, aux_power_event.channel, aux_power_event.duration, aux_power_event.avg_value, aux_power_event.worst_value);
								send_result = send_response(socket_fd, &hmac_key_ctx, received_opcode, received_timestamp, command_counter, R_SUCESS, response_parameters);
								if(send_result < 0)
									break;
							}
						} else {
							response_code = R_ERR_INVALID_PARAMETER;
							break;
						}
					} else if(strcmp(received_parameters[0], "ie") == 0) {
						if(*received_parameters[1] == 'r') {
							if(aux_qty > ievents_count) {
								response_code = R_ERR_INVALID_PARAMETER;
								break;
							}
							
							for(int i = 0; i < aux_qty; i++) {
								get_ievents(&aux_ievent, i);
								
								sprintf(response_parameters, "%u\t%u\t%u\t%u\t", aux_ievent.timestamp, aux_ievent.type, aux_ievent.count, aux_ievent.value);
								send_result = send_response(socket_fd, &hmac_key_ctx, received_opcode, received_timestamp, command_counter, R_SUCESS, response_parameters);
								if(send_result < 0)
									break;
							}
						} else {
							response_code = R_ERR_INVALID_PARAMETER;
							break;
						}
					} else {
						response_code = R_ERR_INVALID_PARAMETER;
						break;
					}
					break;
				case OP_DELETE_DATA:
					if(sscanf(received_parameters[2], "%u", &aux_qty) != 1) {
						response_code = R_ERR_INVALID_PARAMETER;
						break;
					}
					
					if(*received_parameters[1] == 'r') {
						if(strcmp(received_parameters[0], "pd") == 0) {
							if(aux_qty > processed_data_count) {
								response_code = R_ERR_INVALID_PARAMETER;
								break;
							}
							
							delete_power_data(aux_qty);
							
						} else if(strcmp(received_parameters[0], "pe") == 0) {
							if(aux_qty > power_events_data_count) {
								response_code = R_ERR_INVALID_PARAMETER;
								break;
							}
							
							delete_power_events(aux_qty);
							
						} else if(strcmp(received_parameters[0], "ie") == 0) {
							if(aux_qty > ievents_count) {
								response_code = R_ERR_INVALID_PARAMETER;
								break;
							}
							
							delete_ievents(aux_qty);
						}
						
					} else if(*received_parameters[1] == 'f') {
						if(status_sampling_running) {
							response_code = R_ERR_SAMPLING_RUNNING;
							break;
						}
						
						response_code = R_ERR_UNSPECIFIED;
					} else {
						response_code = R_ERR_INVALID_PARAMETER;
						break;
					}
					
					break;
				case OP_GET_WAVEFORM:
					if(!status_sampling_running) {
						response_code = R_ERR_UNSPECIFIED;
						break;
					}
					
					parameter_parse_result = 0;
					parameter_parse_result += sscanf(received_parameters[0], "%u", &aux_channel);
					parameter_parse_result += sscanf(received_parameters[1], "%u", &aux_qty);
					
					if(parameter_parse_result != 2 || aux_qty == 0 || aux_qty > WAVEFORM_MAX_QTY || aux_channel > 6) {
						response_code = R_ERR_INVALID_PARAMETER;
						break;
					}
					
					get_waveform(aux_waveform_buffer, aux_channel, aux_qty);
					
					for(int i = 0; i < aux_qty; i++) {
						sprintf(response_parameters, "%.3f\t", aux_waveform_buffer[i]);
						send_result = send_response(socket_fd, &hmac_key_ctx, received_opcode, received_timestamp, command_counter, R_SUCESS, response_parameters);
						if(send_result < 0)
							break;
					}
					
					break;
				case OP_DISCONNECT:
					if(sscanf(received_parameters[0], "%u", &disconnection_time) != 1) {
						response_code = R_ERR_INVALID_PARAMETER;
						disconnection_time = 0;
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
				send_result = send_response(socket_fd, &hmac_key_ctx, received_opcode, received_timestamp, command_counter, response_code, response_parameters);
			
			if(send_result < 0) {
				debug("Failed to send response.\n");
				break;
			}
			
			command_counter++;
			
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
		
		status_server_connected = 0;
		
		debug("Closing socket.\n");
		close(socket_fd);
		
		#ifdef DEBUG
			fflush(stdout);
		#endif
	}
}

static int recv_command_line(int socket_fd, char *buf, size_t len) {
	int num = 0;
	
	do {
		char c;
		
		if(recv(socket_fd, &c, 1, 0) <= 0)
			return -1;
		
		if(c == '\n')
			break;
		
		if(num < len)
            buf[num] = c;
		
		num++;
	} while(1);
	
	buf[(num >= len) ? len - 1 : num] = 0; // Null terminate
	
	return num;
}

static int convert_opcode(char *buf) {
	if(!buf)
		return -1;
	
	for(int i = 0; i < OPCODE_NUM; i++)
		if(!strcmp(buf, opcode_metadata_list[i].opcode_text))
			return i;
	
	return -1;
}

static int receive_command(int socket_fd, const br_hmac_key_context *hmac_key_ctx, int *op, uint32_t *received_timestamp, uint32_t counter, char parameters[][PARAM_STR_SIZE]) {
	char receive_buffer[200];
	int received_line_len;
	
	uint32_t time_now;
	uint32_t received_counter;
	
	char *str_parameters_ptr;
	int parameter_qty;
	
	received_line_len = recv_command_line(socket_fd, receive_buffer, sizeof(receive_buffer));
	
	if(received_line_len <= 0) // Timeout or disconnection
		return COMM_ERR_RECEVING_RESPONSE;
	
	time_now = get_time();
	
	if(received_line_len < 35) // Protocol error - Line too small
		return COMM_ERR_RECEVING_RESPONSE;
	
	if(validate_hmac(hmac_key_ctx, receive_buffer, received_line_len)) { // Protocol error - Invalid MAC
		add_ievent(IEVENT_TYPE_INVALID_MAC, 0, get_time());
		debug("Invalid MAC.\n");
		return COMM_ERR_INVALID_MAC;
	}
	
	if(parse_command(receive_buffer, op, received_timestamp, &received_counter, &str_parameters_ptr))
		return COMM_ERR_PARSING_COMMAND;
	
	if(received_counter != counter)
		return COMM_ERR_INVALID_COUNTER;
	
	if(time_now > ((*received_timestamp) + SECURITY_MAX_TIMESTAMP_DIFF_SEC))
		return COMM_ERR_INVALID_TIMESTAMP;
	
	if(parameters != NULL) {
		parameter_qty = opcode_metadata_list[(*op)].parameter_qty;
		
		if(parse_parameters(str_parameters_ptr, parameters, parameter_qty) != parameter_qty)
			return COMM_ERR_PARSING_COMMAND;
	}
	
	return COMM_OK;
}

static int parse_command(char *receive_buffer, int *op, uint32_t *timestamp, uint32_t *counter, char **parameters) {
	char *token;
	char *saveptr;
	
	token = strtok_r(receive_buffer, ":", &saveptr); // Opcode
	if(!token) // Protocol error - Syntax Error
		return -1;
	
	*op = convert_opcode(token);
	if(*op < 0) // Protocol error - Invalid opcode
		return -1;
	
	token = strtok_r(NULL, ":", &saveptr); // Timestamp
	if(!token) // Protocol error - Syntax Error
		return -1;
	
	if(sscanf(token, "%u", timestamp) != 1)
		return -1;
	
	token = strtok_r(NULL, ":", &saveptr); // Counter
	if(!token) // Protocol error - Syntax Error
		return -1;
	
	if(sscanf(token, "%u", counter) != 1)
		return -1;
	
	if(parameters != NULL)
		*parameters = strtok_r(NULL, ":", &saveptr); // Parameters
	
	return 0;
}

static int parse_parameters(char *parameter_buffer, char parsed_parameters[][PARAM_STR_SIZE], unsigned int parameter_qty) {
	char *buffer_ptr = parameter_buffer;
	char *token;
	char *saveptr;
	
	int count = 0;
	
	if(parameter_buffer == NULL)
		return 0;
	
	while((token = strtok_r(buffer_ptr, "\t", &saveptr)) && count < parameter_qty && count < PARAM_MAX_QTY) {
		if(strlen(token) >= PARAM_STR_SIZE)
			break;
		
		strcpy(parsed_parameters[count], token);
		count++;
		buffer_ptr = NULL;
	}
	
	return count;
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
	#if SECURITY_CHECK_MAC
	if(strcmp(received_mac_text, computed_mac_text)) // Protocol error - Invalid MAC
		return -2;
	#endif
	
	return 0;
}

static int send_response(int socket_fd, const br_hmac_key_context *hmac_key_ctx, int opcode, uint32_t timestamp, uint32_t counter, int response_code, char *parameters) {
	char aux[36];
	char send_buffer[200];
	char computed_mac_text[33];
	
	sprintf(send_buffer, "A:%s:%u:%u:%d:", opcode_metadata_list[opcode].opcode_text, timestamp, counter, response_code);
	
	if(parameters)
		strlcat(send_buffer, parameters, 200);
	
	compute_hmac(hmac_key_ctx, computed_mac_text, send_buffer, strlen(send_buffer));
	
	sprintf(aux, "*%s\n", computed_mac_text);
	strlcat(send_buffer, aux, 200);
	
	return send(socket_fd, send_buffer, strlen(send_buffer), 0);
}
