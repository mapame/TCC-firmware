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

int add_event(const char *event_text, uint32_t event_time) {
	if(!xSemaphoreTake(events_mutex, pdMS_TO_TICKS(500)))
		return 1;
	
	if(event_count == EVENT_BUFFER_SIZE) {
		xSemaphoreGive(events_mutex);
		return 2;
	}
	
	events[events_head].timestamp = event_time;
	strncpy(events[events_head].text, event_text, 31);
	events[events_head].text[31] = '\0';
	
	events_head = (events_head + 1) % EVENT_BUFFER_SIZE;
	event_count++;
	
	if(event_count == (EVENT_BUFFER_SIZE - 1)) {
		events[events_head].timestamp = event_time;
		strcpy(events[events_head].text, "EVENT_BUFFER_FULL");
		
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
