#include <espressif/esp_common.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include "common.h"
#include "ievents.h"

ievent_t ievents_buffer[IEVENT_BUFFER_SIZE];
uint16_t ievents_head, ievents_tail, ievents_count;

SemaphoreHandle_t ievents_mutex = NULL;

int add_ievent(int type, int value, uint32_t event_time) {
	uint16_t last_ievent_pos;
	
	if(!xSemaphoreTake(ievents_mutex, pdMS_TO_TICKS(500)))
		return 1;
	
	if(ievents_count) {
		last_ievent_pos = (ievents_head == 0) ? (IEVENT_BUFFER_SIZE - 1) : (ievents_head - 1);
		
		if(ievents_buffer[last_ievent_pos].type == type && ievents_buffer[last_ievent_pos].value == value && ievents_buffer[last_ievent_pos].timestamp == event_time) {
			xSemaphoreGive(ievents_mutex);
			ievents_buffer[last_ievent_pos].count++;
			return 0;
		}
	}
	
	ievents_buffer[ievents_head].timestamp = event_time;
	ievents_buffer[ievents_head].count = 1;
	ievents_buffer[ievents_head].type = type;
	ievents_buffer[ievents_head].value = value;
	
	ievents_head = (ievents_head + 1) % IEVENT_BUFFER_SIZE;
	
	if(ievents_head == ievents_tail)
		ievents_tail = (ievents_tail + 1) % IEVENT_BUFFER_SIZE;
	else
		ievents_count++;
	
	xSemaphoreGive(ievents_mutex);
	
	return 0;
}

int ievents_init() {
	ievents_mutex = xSemaphoreCreateMutex();
	
	ievents_head = 0;
	ievents_tail = 0;
	ievents_count = 0;
	
	return 0;
}
