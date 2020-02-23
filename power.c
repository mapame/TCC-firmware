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
#include <semphr.h>

#include "common.h"
#include "configuration.h"
#include "sampling.h"
#include "rtc.h"
#include "ievents.h"
#include "power.h"

power_data_t processed_data[PROCESSED_DATA_BUFFER_SIZE];
uint16_t processed_data_head, processed_data_tail, processed_data_count;

power_event_t power_events[POWER_EVENT_BUFFER_SIZE];
uint16_t power_events_data_head, power_events_data_tail, power_events_data_count;

float waveform_buffer[4][WAVEFORM_MAX_QTY];
uint16_t waveform_buffer_pos;

SemaphoreHandle_t processed_data_mutex = NULL;
SemaphoreHandle_t power_events_mutex = NULL;
SemaphoreHandle_t waveform_buffer_mutex = NULL;


void power_processing_task(void *pvParameters) {
	raw_adc_data_t raw_adc_data;
	
	int raw_adc_data_processed_counter = 0;
	uint32_t first_sample_usecs;
	uint32_t first_sample_rtc_time;
	
	float v[2];
	float i[2];
	
	float vrms_acc[2] = {0.0, 0.0};
	float irms_acc[2] = {0.0, 0.0};
	float p_acc[2] = {0.0, 0.0};
	
	processed_data_mutex = xSemaphoreCreateMutex();
	power_events_mutex = xSemaphoreCreateMutex();
	waveform_buffer_mutex = xSemaphoreCreateMutex();
	
	processed_data_head = 0;
	processed_data_tail = 0;
	processed_data_count = 0;
	
	config_power_phases = MAX(config_power_phases, 1);
	config_power_phases = MIN(config_power_phases, 2);
	
	start_sampling();
	
	while(true) {
		if(xMessageBufferReceive(raw_adc_data_buffer, (void*) &raw_adc_data, sizeof(raw_adc_data_t), pdMS_TO_TICKS(200)) == 0) {
			if(status_sampling_running) {
				status_sampling_running = 0;
				add_ievent(IEVENT_TYPE_SAMPLING_STOPPED, 0, get_time());
			}
			
			continue;
		}
		
		if(raw_adc_data_count == RAW_ADC_DATA_BUFFER_SIZE)
				add_ievent(IEVENT_TYPE_ADC_BUFFER_FULL, raw_adc_data_count, get_time());
		
		raw_adc_data_count--;
		
		if(raw_adc_data_processed_counter == 0) {
			first_sample_usecs = raw_adc_data.usecs_since_time;
			first_sample_rtc_time = raw_adc_data.rtc_time;
		}
		
		raw_adc_data_processed_counter++;
		
		v[0] = (adc_volt_scale[0] * (float)raw_adc_data.data[0]) * config_voltage_factors[0];
		
		vrms_acc[0] += v[0] * v[0];
		
		if(config_power_phases == 2) {
			v[1] = (adc_volt_scale[0] * (float)raw_adc_data.data[1]) * config_voltage_factors[1];
			
			vrms_acc[1] += v[1] * v[1];
		}
		
		i[0] = (adc_volt_scale[1] * (float)raw_adc_data.data[2]) * config_current_factors[0];
		
		irms_acc[0] += i[0] * i[0];
		
		p_acc[0] += v[0] * i[0];
		
		if(config_power_phases == 2) {
			i[1] = (adc_volt_scale[2] * (float)raw_adc_data.data[3]) * config_current_factors[1];
			
			irms_acc[1] += i[1] * i[1];
			
			p_acc[1] += v[1] * i[1];
		}
		
		xSemaphoreTake(waveform_buffer_mutex, pdMS_TO_TICKS(200));
		
		waveform_buffer[0][waveform_buffer_pos] = v[0];
		waveform_buffer[1][waveform_buffer_pos] = v[1];
		waveform_buffer[2][waveform_buffer_pos] = i[0];
		waveform_buffer[3][waveform_buffer_pos] = i[1];
		
		waveform_buffer_pos = (waveform_buffer_pos + 1) % WAVEFORM_MAX_QTY;
		
		xSemaphoreGive(waveform_buffer_mutex);
		
		if((raw_adc_data.usecs_since_time - first_sample_usecs) >= 1000000 || first_sample_rtc_time != raw_adc_data.rtc_time) {
			xSemaphoreTake(processed_data_mutex, pdMS_TO_TICKS(200));
			
			processed_data[processed_data_head].timestamp = first_sample_rtc_time + first_sample_usecs / 1000000U;
			processed_data[processed_data_head].duration_usec = raw_adc_data.usecs_since_time - first_sample_usecs;
			processed_data[processed_data_head].samples = raw_adc_data_processed_counter;
			
			processed_data[processed_data_head].vrms[0] = sqrtf(vrms_acc[0] / (float) raw_adc_data_processed_counter);
			processed_data[processed_data_head].vrms[1] = sqrtf(vrms_acc[1] / (float) raw_adc_data_processed_counter);
			
			processed_data[processed_data_head].irms[0] = sqrtf(irms_acc[0] / (float) raw_adc_data_processed_counter);
			processed_data[processed_data_head].irms[1] = sqrtf(irms_acc[1] / (float) raw_adc_data_processed_counter);
			
			processed_data[processed_data_head].p[0] = p_acc[0] / (float) raw_adc_data_processed_counter;
			processed_data[processed_data_head].p[1] = p_acc[1] / (float) raw_adc_data_processed_counter;
			
			processed_data_head = (processed_data_head + 1) % PROCESSED_DATA_BUFFER_SIZE;
			if(processed_data_head == processed_data_tail)
				processed_data_tail = (processed_data_tail + 1) % PROCESSED_DATA_BUFFER_SIZE;
			else
				processed_data_count++;
			
			xSemaphoreGive(processed_data_mutex);
			
			vrms_acc[0] = vrms_acc[1] = 0.0;
			irms_acc[0] = irms_acc[1] = 0.0;
			p_acc[0] = p_acc[1] = 0.0;
			
			raw_adc_data_processed_counter = 0;
		}
	}
}

int get_power_data(power_data_t *data, unsigned int index) {
	xSemaphoreTake(processed_data_mutex, pdMS_TO_TICKS(300));
	
	if(index >= processed_data_count) {
		xSemaphoreGive(processed_data_mutex);
		return 1;
	}
	
	memcpy(data, &processed_data[(processed_data_tail + index) % PROCESSED_DATA_BUFFER_SIZE], sizeof(power_data_t));
	
	xSemaphoreGive(processed_data_mutex);
	
	return 0;
}

int delete_power_data(unsigned int qty) {
	int real_qty;
	
	xSemaphoreTake(processed_data_mutex, pdMS_TO_TICKS(500));
	
	real_qty = MIN(qty, processed_data_count);
	
	processed_data_tail = (processed_data_tail + real_qty) % PROCESSED_DATA_BUFFER_SIZE;
	processed_data_count -= real_qty;
	
	xSemaphoreGive(processed_data_mutex);
	
	return real_qty;
}

int get_power_events(power_event_t *data, unsigned int index) {
	xSemaphoreTake(power_events_mutex, pdMS_TO_TICKS(300));
	
	if(index >= power_events_data_count) {
		xSemaphoreGive(power_events_mutex);
		return 1;
	}
	
	memcpy(data, &power_events[(power_events_data_tail + index) % POWER_EVENT_BUFFER_SIZE], sizeof(power_event_t));
	
	xSemaphoreGive(power_events_mutex);
	
	return 0;
}

int delete_power_events(unsigned int qty) {
	int real_qty;
	
	xSemaphoreTake(power_events_mutex, pdMS_TO_TICKS(500));
	
	real_qty = MIN(qty, power_events_data_count);
	
	power_events_data_tail = (power_events_data_tail + real_qty) % POWER_EVENT_BUFFER_SIZE;
	power_events_data_count -= real_qty;
	
	xSemaphoreGive(power_events_mutex);
	
	return real_qty;
}

void get_waveform(float *buffer, unsigned int channel, unsigned int qty) {
	xSemaphoreTake(waveform_buffer_mutex, pdMS_TO_TICKS(300));
	
	for(int i = 0; i < qty; i++)
		buffer[i] = waveform_buffer[channel][(waveform_buffer_pos + i) % WAVEFORM_MAX_QTY];
	
	xSemaphoreGive(waveform_buffer_mutex);
}
