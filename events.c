#include <espressif/esp_common.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include "common.h"
#include "events.h"

event_t events[EVENT_BUFFER_SIZE];
uint16_t events_head, events_tail, event_count;

SemaphoreHandle_t events_mutex = NULL;

int add_event(int type, int value, uint32_t event_time) {
	uint16_t last_event_pos;
	
	if(!xSemaphoreTake(events_mutex, pdMS_TO_TICKS(500)))
		return 1;
	
	last_event_pos = (events_head == 0) ? (EVENT_BUFFER_SIZE - 1) : (events_head - 1);
	
	if(event_count == EVENT_BUFFER_SIZE) {
		if(events[last_event_pos].type == EVENT_TYPE_EVENTS_BUFFER_FULL)
			events[last_event_pos].count++;
		
		xSemaphoreGive(events_mutex);
		return 2;
	}
	
	if(event_count && events[last_event_pos].type == type && events[last_event_pos].value == value && events[last_event_pos].timestamp == event_time) {
		events[last_event_pos].count++;
		xSemaphoreGive(events_mutex);
		return 0;
	}
	
	events[events_head].timestamp = event_time;
	events[events_head].count = 1;
	events[events_head].type = type;
	events[events_head].value = value;
	
	events_head = (events_head + 1) % EVENT_BUFFER_SIZE;
	event_count++;
	
	if(event_count == (EVENT_BUFFER_SIZE - 1)) {
		events[events_head].timestamp = event_time;
		events[events_head].count = 1;
		events[events_head].type = EVENT_TYPE_EVENTS_BUFFER_FULL;
		events[events_head].value = event_count;
		
		events_head = (events_head + 1) % EVENT_BUFFER_SIZE;
		event_count++;
	}
	
	xSemaphoreGive(events_mutex);
	
	return 0;
}

int get_event(event_t *data, unsigned int index) {
	xSemaphoreTake(events_mutex, pdMS_TO_TICKS(300));
	
	if(index >= event_count) {
		xSemaphoreGive(events_mutex);
		return 1;
	}
	
	memcpy(data, &events[(events_tail + index) % EVENT_BUFFER_SIZE], sizeof(event_t));
	
	xSemaphoreGive(events_mutex);
	
	return 0;
}

int delete_events(unsigned int qty) {
	xSemaphoreTake(events_mutex, pdMS_TO_TICKS(500));
	
	if(qty > event_count) {
		xSemaphoreGive(events_mutex);
		
		return -1;
	}
	
	events_tail = (events_tail + qty) % EVENT_BUFFER_SIZE;
	event_count -= qty;
	
	xSemaphoreGive(events_mutex);
	
	return 0;
}

int events_init() {
	events_mutex = xSemaphoreCreateMutex();
	
	events_head = 0;
	events_tail = 0;
	event_count = 0;
	
	return 0;
}
