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
	
	last_ievent_pos = (ievents_head == 0) ? (IEVENT_BUFFER_SIZE - 1) : (ievents_head - 1);
	
	if(ievents_count == IEVENT_BUFFER_SIZE) {
		if(ievents_buffer[last_ievent_pos].type == IEVENT_TYPE_IEVENTS_BUFFER_FULL)
			ievents_buffer[last_ievent_pos].count++;
		
		return 2;
	}
	
	if(ievents_count && ievents_buffer[last_ievent_pos].type == type && ievents_buffer[last_ievent_pos].value == value && ievents_buffer[last_ievent_pos].timestamp == event_time) {
		ievents_buffer[last_ievent_pos].count++;
		xSemaphoreGive(ievents_mutex);
		return 0;
	}
	
	ievents_buffer[ievents_head].timestamp = event_time;
	ievents_buffer[ievents_head].count = 1;
	ievents_buffer[ievents_head].type = type;
	ievents_buffer[ievents_head].value = value;
	
	ievents_head = (ievents_head + 1) % IEVENT_BUFFER_SIZE;
	ievents_count++;
	
	if(ievents_count == (IEVENT_BUFFER_SIZE - 1)) {
		ievents_buffer[ievents_head].timestamp = event_time;
		ievents_buffer[ievents_head].count = 1;
		ievents_buffer[ievents_head].type = IEVENT_TYPE_IEVENTS_BUFFER_FULL;
		ievents_buffer[ievents_head].value = ievents_count;
		
		ievents_head = (ievents_head + 1) % IEVENT_BUFFER_SIZE;
		ievents_count++;
	}
	
	xSemaphoreGive(ievents_mutex);
	
	return 0;
}

int get_ievents(ievent_t *data, unsigned int index) {
	xSemaphoreTake(ievents_mutex, pdMS_TO_TICKS(300));
	
	if(index >= ievents_count) {
		xSemaphoreGive(ievents_mutex);
		return 1;
	}
	
	memcpy(data, &ievents_buffer[(ievents_tail + index) % IEVENT_BUFFER_SIZE], sizeof(ievent_t));
	
	xSemaphoreGive(ievents_mutex);
	
	return 0;
}

int delete_ievents(unsigned int qty) {
	int real_qty;
	
	xSemaphoreTake(ievents_mutex, pdMS_TO_TICKS(500));
	
	real_qty = MIN(qty, ievents_count);
	
	ievents_tail = (ievents_tail + real_qty) % IEVENT_BUFFER_SIZE;
	ievents_count -= real_qty;
	
	xSemaphoreGive(ievents_mutex);
	
	return real_qty;
}

int ievents_init() {
	ievents_mutex = xSemaphoreCreateMutex();
	
	ievents_head = 0;
	ievents_tail = 0;
	ievents_count = 0;
	
	return 0;
}
