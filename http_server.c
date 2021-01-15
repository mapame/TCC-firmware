#include <espressif/esp_common.h>

#include <esp8266.h>

#include <string.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <message_buffer.h>

#include <httpd/httpd.h>

#include "common.h"
#include "rtc.h"
#include "configuration.h"


#define CMD_BUFFER_SIZE 2048

MessageBufferHandle_t cmd_buffer = NULL;

struct tcp_pcb *client_pcb = NULL;


static void ws_rcv_msg_cb(struct tcp_pcb *pcb, uint8_t *data, u16_t data_len, uint8_t mode) {
	char data_cpy[128];
	
	if(pcb == NULL || pcb != client_pcb)
		return;
	
	memcpy(data_cpy, data, data_len);
	
	data_cpy[data_len] = '\0';
	
	xMessageBufferSend(cmd_buffer, (void*) data_cpy, data_len + 1, pdMS_TO_TICKS(200));
}

static void ws_open_cb(struct tcp_pcb *pcb, const char *uri) {
	if(client_pcb == NULL)
		client_pcb = pcb;
}

static int ws_client_send_msg(const char *msg) {
	err_t result;
	
	if(client_pcb == NULL || TCP_STATE_IS_CLOSING(client_pcb->state))
		return -2;
		
	LOCK_TCPIP_CORE();
	result = websocket_write(client_pcb, (const unsigned char *) msg, (uint16_t) strlen(msg), WS_TEXT_MODE);
	UNLOCK_TCPIP_CORE();
	
	if(result != ERR_OK)
		return -1;
	
	return 0;
}

int cmd_config_read(char *parameters) {
	char value_buffer[CONFIG_STR_SIZE];
	char send_buffer[CONFIG_STR_SIZE + 32 + 6];
	
	if(parameters == NULL)
		return -1;
	
	if(configuration_read((const char*)parameters, value_buffer, 0))
		return -2;
	
	snprintf(send_buffer, sizeof(send_buffer), "CFGV\t%s\t%s", parameters, value_buffer);
	
	ws_client_send_msg(send_buffer);
	
	return 0;
}

int cmd_config_write(char *parameters) {
	char *value_ptr;
	
	if(parameters == NULL)
		return -1;
	
	value_ptr = strchr(parameters, '\t');
	
	if(value_ptr == NULL)
		return -2;
	
	*value_ptr = '\0';
	
	value_ptr++;
	
	if(configuration_write((const char*)parameters, (const char*)value_ptr, 0))
		return -3;
	
	return 0;
}

static int cmd_update_rtc(char *parameters) {
	uint32_t newtime;
	
	if(parameters == NULL)
		return -1;
	
	if(sscanf(parameters, "%u", &newtime) != 1)
		return -2;
	
	if(update_rtc(newtime))
		return -3;
	
	return 0;
}

static inline void process_cmd_buffer() {
	char cmdstr[150];
	
	while(!xMessageBufferIsEmpty(cmd_buffer)) {
		xMessageBufferReceive(cmd_buffer, (void*) &cmdstr, sizeof(cmdstr), 0);
		
		if(!strncmp(cmdstr, "RST\t", 4)) {
			
			sdk_system_restart();
			
		} else if(!strncmp(cmdstr, "CFGRD\t", 6)) {
			
			cmd_config_read(cmdstr + 6);
			
		} else if(!strncmp(cmdstr, "CFGWR\t", 6)) {
			
			cmd_config_write(cmdstr + 6);
			cmd_config_read(cmdstr + 6);
			
		} else if(!strncmp(cmdstr, "RTCU\t", 5)) {
			
			cmd_update_rtc(cmdstr + 5);
			
		}
	}
}

static void send_rtc_msg() {
	char send_buffer[32];
	
	snprintf(send_buffer, sizeof(send_buffer), "RTC\t%u", (uint32_t) get_time());
	
	ws_client_send_msg(send_buffer);
}

void httpd_task(void *pvParameters) {
	int cycle = 0;
	
	cmd_buffer = xMessageBufferCreate(CMD_BUFFER_SIZE);
	
	websocket_register_callbacks((tWsOpenHandler) ws_open_cb, (tWsHandler) ws_rcv_msg_cb);
	httpd_init();
	
	for(;;) {
		vTaskDelay(pdMS_TO_TICKS(500));
		
		if(client_pcb != NULL && TCP_STATE_IS_CLOSING(client_pcb->state))
			client_pcb = NULL;
		
		if(client_pcb == NULL)
			continue;
		
		process_cmd_buffer();
		
		if((cycle = (cycle + 1) % 2) == 0)
			send_rtc_msg();
		
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
