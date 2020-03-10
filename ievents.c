#include <espressif/esp_common.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include "common.h"
#include "ievents.h"

internal_event_t internal_events[IEVENT_BUFFER_SIZE];
uint16_t internal_events_head, internal_events_tail, internal_events_count;

SemaphoreHandle_t internal_events_mutex = NULL;

int add_internal_event(int type, int value, uint32_t event_time) {
	uint16_t last_ievent_pos;
	
	if(!xSemaphoreTake(internal_events_mutex, pdMS_TO_TICKS(500)))
		return 1;
	
	last_ievent_pos = (internal_events_head == 0) ? (IEVENT_BUFFER_SIZE - 1) : (internal_events_head - 1);
	
	if(internal_events_count == IEVENT_BUFFER_SIZE) {
		if(internal_events[last_ievent_pos].type == IEVENT_TYPE_IEVENTS_BUFFER_FULL)
			internal_events[last_ievent_pos].count++;
		
		xSemaphoreGive(internal_events_mutex);
		return 2;
	}
	
	if(internal_events_count && internal_events[last_ievent_pos].type == type && internal_events[last_ievent_pos].value == value && internal_events[last_ievent_pos].timestamp == event_time) {
		internal_events[last_ievent_pos].count++;
		xSemaphoreGive(internal_events_mutex);
		return 0;
	}
	
	internal_events[internal_events_head].timestamp = event_time;
	internal_events[internal_events_head].count = 1;
	internal_events[internal_events_head].type = type;
	internal_events[internal_events_head].value = value;
	
	internal_events_head = (internal_events_head + 1) % IEVENT_BUFFER_SIZE;
	internal_events_count++;
	
	if(internal_events_count == (IEVENT_BUFFER_SIZE - 1)) {
		internal_events[internal_events_head].timestamp = event_time;
		internal_events[internal_events_head].count = 1;
		internal_events[internal_events_head].type = IEVENT_TYPE_IEVENTS_BUFFER_FULL;
		internal_events[internal_events_head].value = internal_events_count;
		
		internal_events_head = (internal_events_head + 1) % IEVENT_BUFFER_SIZE;
		internal_events_count++;
	}
	
	xSemaphoreGive(internal_events_mutex);
	
	return 0;
}

int get_internal_event(internal_event_t *data, unsigned int index) {
	xSemaphoreTake(internal_events_mutex, pdMS_TO_TICKS(300));
	
	if(index >= internal_events_count) {
		xSemaphoreGive(internal_events_mutex);
		return 1;
	}
	
	memcpy(data, &internal_events[(internal_events_tail + index) % IEVENT_BUFFER_SIZE], sizeof(internal_event_t));
	
	xSemaphoreGive(internal_events_mutex);
	
	return 0;
}

int delete_internal_events(unsigned int qty) {
	xSemaphoreTake(internal_events_mutex, pdMS_TO_TICKS(500));
	
	if(qty > internal_events_count) {
		xSemaphoreGive(internal_events_mutex);
		
		return -1;
	}
	
	internal_events_tail = (internal_events_tail + qty) % IEVENT_BUFFER_SIZE;
	internal_events_count -= qty;
	
	xSemaphoreGive(internal_events_mutex);
	
	return 0;
}

int ievents_init() {
	internal_events_mutex = xSemaphoreCreateMutex();
	
	internal_events_head = 0;
	internal_events_tail = 0;
	internal_events_count = 0;
	
	return 0;
}
