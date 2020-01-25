#include <espressif/esp_common.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "common.h"
#include "rtc.h"
#include "ievents.h"

internal_event_t internal_events[INTERNAL_EVENT_BUFFER_SIZE];
uint16_t internal_events_head = 0;
uint16_t internal_events_tail = 0;
uint16_t internal_events_count = 0;

void add_internal_event(int event_type, int value) {
	
	internal_events[internal_events_head].timestamp = 0;
	internal_events[internal_events_head].count = 1;
	internal_events[internal_events_head].type = event_type;
	internal_events[internal_events_head].value = value;
	
	
	internal_events_head = (internal_events_head + 1) % INTERNAL_EVENT_BUFFER_SIZE;
	if(internal_events_head == internal_events_tail)
		internal_events_tail = (internal_events_tail + 1) % INTERNAL_EVENT_BUFFER_SIZE;
	else
		internal_events_count++;
}
