#include <espressif/esp_common.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <esp8266.h>

#include <FreeRTOS.h>
#include <task.h>
#include <message_buffer.h>

#include "common.h"
#include "configuration.h"
#include "sampling.h"
#include "power.h"

power_data_t processed_data[PROCESSED_DATA_BUFFER_SIZE];
uint16_t processed_data_head, processed_data_tail, processed_data_count;

power_event_t power_events[POWER_EVENT_BUFFER_SIZE];
uint16_t power_events_data_head, power_events_data_tail, power_events_data_count;

void power_processing_task(void *pvParameters) {
	raw_adc_data_t raw_adc_data;
	
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
	
	start_sampling();
	
	while(true) {
		if(xMessageBufferReceive(raw_adc_data_buffer, (void*) &raw_adc_data, sizeof(raw_adc_data_t), pdMS_TO_TICKS(200)) == 0)
			continue;
		
		raw_adc_data_count--;
		
		if(raw_adc_data_processed_counter == 0) {
			first_sample_usecs = raw_adc_data.usecs_since_time;
			first_sample_rtc_time = raw_adc_data.rtc_time;
		}
		
		raw_adc_data_processed_counter++;
		
		v[0] = (adc_volt_scale[0] * (float)raw_adc_data.data[0]) * config_voltage_factors[0];
		v[1] = (adc_volt_scale[0] * (float)raw_adc_data.data[1]) * config_voltage_factors[1];
		v[2] = v[0] - v[1];
		
		i[0] = (adc_volt_scale[1] * (float)raw_adc_data.data[2]) * config_current_factors[0];
		i[1] = (adc_volt_scale[2] * (float)raw_adc_data.data[3]) * config_current_factors[1];
		i[2] = (adc_volt_scale[2] * (float)raw_adc_data.data[4]) * config_current_factors[2];
		
		vrms_acc[0] += v[0] * v[0];
		vrms_acc[1] += v[1] * v[1];
		vrms_acc[2] += v[2] * v[2];
		
		irms_acc[0] += i[0] * i[0];
		irms_acc[1] += i[1] * i[1];
		irms_acc[2] += i[2] * i[2];
		
		p_acc[0] += v[0] * i[0];
		p_acc[1] += v[1] * i[1];
		p_acc[2] += v[2] * i[2];
		
		if((raw_adc_data.usecs_since_time - first_sample_usecs) >= 1000000 || first_sample_rtc_time != raw_adc_data.rtc_time) {
			processed_data[processed_data_head].timestamp = first_sample_rtc_time + first_sample_usecs / 1000000U;
			processed_data[processed_data_head].duration_usec = raw_adc_data.usecs_since_time - first_sample_usecs;
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
