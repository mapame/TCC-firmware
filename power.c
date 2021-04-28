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
#include "power.h"
#include "events.h"
#include "configuration.h"
#include "sampling.h"
#include "rtc.h"


power_data_t power_data[POWER_DATA_BUFFER_SIZE];
uint16_t power_data_head, power_data_tail, power_data_count;

float waveform_buffer_v[2][WAVEFORM_BUFFER_SIZE];
float waveform_buffer_i[2][WAVEFORM_BUFFER_SIZE];
uint16_t waveform_buffer_pos;

SemaphoreHandle_t power_data_mutex = NULL;
SemaphoreHandle_t waveform_buffer_mutex = NULL;

int add_power_data(const power_data_t *data);

void power_processing_task(void *pvParameters) {
	raw_adc_data_t raw_adc_data;
	
	power_data_t aux_power_data;
	
	int empty_msgbuf = 0;
	
	int raw_adc_data_processed_counter = 0;
	
	uint32_t first_sample_timestamp = 0;
	
	uint32_t last_timestamp = 0;
	
	uint32_t total_duration = 0;
	
	float v[2] = {0.0, 0.0};
	float i[2] = {0.0, 0.0};
	
	float vrms_acc[2] = {0.0, 0.0};
	float irms_acc[2] = {0.0, 0.0};
	float p_acc[2] = {0.0, 0.0};
	
	power_data_mutex = xSemaphoreCreateMutex();
	waveform_buffer_mutex = xSemaphoreCreateMutex();
	
	power_data_head = 0;
	power_data_tail = 0;
	power_data_count = 0;
	
	start_sampling();
	
	while(true) {
		if(xMessageBufferReceive(raw_adc_data_buffer, (void*) &raw_adc_data, sizeof(raw_adc_data_t), pdMS_TO_TICKS(200)) == 0) {
			if(status_sampling_running == 0) {
				last_timestamp = 0;
			} else if(++empty_msgbuf >= 3) {
				empty_msgbuf = 0;
				status_sampling_running = 0;
				add_event("ADC_BUFFER_EMPTY", get_time());
			}
			
			continue;
		}
		
		empty_msgbuf = 0;
		
		if(raw_adc_data_count == RAW_ADC_DATA_BUFFER_SIZE)
			add_event("ADC_BUFFER_FULL", get_time());
		
		raw_adc_data_count--;
		
		if(raw_adc_data.errors)
			add_event("I2C_ERROR_ADC_SAMPLING", get_time());
		
		
		xSemaphoreTake(waveform_buffer_mutex, pdMS_TO_TICKS(200));
		
		waveform_buffer_v[0][waveform_buffer_pos] = v[0];
		waveform_buffer_v[1][waveform_buffer_pos] = v[1];
		waveform_buffer_i[0][waveform_buffer_pos] = i[0];
		waveform_buffer_i[1][waveform_buffer_pos] = i[1];
		
		waveform_buffer_pos = (waveform_buffer_pos + 1) % WAVEFORM_BUFFER_SIZE;
		
		xSemaphoreGive(waveform_buffer_mutex);
		
		
		if(raw_adc_data.timestamp < first_sample_timestamp)
			raw_adc_data_processed_counter = 0;
		
		if(raw_adc_data_processed_counter == 0) {
			first_sample_timestamp = raw_adc_data.timestamp;
			
			total_duration = 0;
			
			vrms_acc[0] = vrms_acc[1] = 0.0;
			irms_acc[0] = irms_acc[1] = 0.0;
			p_acc[0] = p_acc[1] = 0.0;
		}
		
		total_duration += raw_adc_data.duration;
		
		if(total_duration < (1000000U + 2000U) && raw_adc_data.timestamp == first_sample_timestamp) {
			
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
			
			raw_adc_data_processed_counter++;
		}
		
		if(raw_adc_data.timestamp != first_sample_timestamp) {
			aux_power_data.timestamp = first_sample_timestamp;
			aux_power_data.duration_usec = total_duration;
			aux_power_data.samples = raw_adc_data_processed_counter;
			
			aux_power_data.vrms[0] = sqrtf(vrms_acc[0] / (float) raw_adc_data_processed_counter);
			aux_power_data.vrms[1] = sqrtf(vrms_acc[1] / (float) raw_adc_data_processed_counter);
			
			aux_power_data.irms[0] = sqrtf(irms_acc[0] / (float) raw_adc_data_processed_counter);
			aux_power_data.irms[1] = sqrtf(irms_acc[1] / (float) raw_adc_data_processed_counter);
			
			aux_power_data.p[0] = p_acc[0] / (float) raw_adc_data_processed_counter;
			aux_power_data.p[1] = p_acc[1] / (float) raw_adc_data_processed_counter;
			
			add_power_data(&aux_power_data);
			
			if(last_timestamp && (aux_power_data.timestamp - last_timestamp) > 1)
				add_event("POWER_DATA_TIME_GAP", aux_power_data.timestamp);
			
			last_timestamp = aux_power_data.timestamp;
			
			raw_adc_data_processed_counter = 0;
		}
	}
}

int add_power_data(const power_data_t *data) {
	xSemaphoreTake(power_data_mutex, pdMS_TO_TICKS(300));
	
	memcpy(&power_data[power_data_head], data, sizeof(power_data_t));
	
	power_data_head = (power_data_head + 1) % POWER_DATA_BUFFER_SIZE;
	
	if(power_data_count == POWER_DATA_BUFFER_SIZE)
		power_data_tail = (power_data_tail + 1) % POWER_DATA_BUFFER_SIZE;
	else
		power_data_count++;
	
	xSemaphoreGive(power_data_mutex);
	
	return 0;
}

int get_power_data(power_data_t *data, unsigned int index) {
	xSemaphoreTake(power_data_mutex, pdMS_TO_TICKS(300));
	
	if(index >= power_data_count) {
		xSemaphoreGive(power_data_mutex);
		return 1;
	}
	
	memcpy(data, &power_data[(power_data_tail + index) % POWER_DATA_BUFFER_SIZE], sizeof(power_data_t));
	
	xSemaphoreGive(power_data_mutex);
	
	return 0;
}

int delete_power_data(unsigned int qty) {
	xSemaphoreTake(power_data_mutex, pdMS_TO_TICKS(500));
	
	if(qty > power_data_count) {
		xSemaphoreGive(power_data_mutex);
		
		return -1;
	}
	
	power_data_tail = (power_data_tail + qty) % POWER_DATA_BUFFER_SIZE;
	power_data_count -= qty;
	
	xSemaphoreGive(power_data_mutex);
	
	return 0;
}

int get_waveform(float *buffer_v, float *buffer_i, unsigned int phase, unsigned int qty) {
	if(phase < 1 || phase > config_power_phases)
		return -1;
	
	if(qty > WAVEFORM_BUFFER_SIZE)
		return -2;
	
	xSemaphoreTake(waveform_buffer_mutex, pdMS_TO_TICKS(300));
	
	for(int i = 0; i < qty; i++) {
		buffer_v[i] = waveform_buffer_v[phase - 1][(waveform_buffer_pos + i) % WAVEFORM_BUFFER_SIZE];
		buffer_i[i] = waveform_buffer_i[phase - 1][(waveform_buffer_pos + i) % WAVEFORM_BUFFER_SIZE];
	}
	
	xSemaphoreGive(waveform_buffer_mutex);
	
	return 0;
}
