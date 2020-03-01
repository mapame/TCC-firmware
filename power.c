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
uint16_t power_events_head, power_events_tail, power_events_count;

float waveform_buffer[4][WAVEFORM_MAX_QTY];
uint16_t waveform_buffer_pos;

SemaphoreHandle_t processed_data_mutex = NULL;
SemaphoreHandle_t power_events_mutex = NULL;
SemaphoreHandle_t waveform_buffer_mutex = NULL;


int add_power_event(const power_event_t *pev);

void power_processing_task(void *pvParameters) {
	raw_adc_data_t raw_adc_data;
	
	power_event_t aux_power_event;
	
	int raw_adc_data_processed_counter = 0;
	uint32_t first_sample_usecs;
	uint32_t first_sample_rtc_time;
	
	float v[2];
	float i[2];
	
	float vrms_acc[2] = {0.0, 0.0};
	float irms_acc[2] = {0.0, 0.0};
	float p_acc[2] = {0.0, 0.0};
	
	unsigned int pevents_count[2][POWER_EVENT_TYPE_QTY];
	float pevents_avg_acc[2][POWER_EVENT_TYPE_QTY];
	float pevents_worst[2][POWER_EVENT_TYPE_QTY];
	
	float last_v[2];
	
	int pevent_first_rise[2] = {1, 1};
	
	uint32_t pevent_cycle_start_us[2] = {0, 0};
	
	float pevent_rms_count[2] = {0, 0};
	float pevent_vrms_acc[2] = {0.0, 0.0};
	float pevent_irms_acc[2] = {0.0, 0.0};
	
	float pevent_vrms[2];
	float pevent_irms[2];
	float pevent_freq[2];
	
	processed_data_mutex = xSemaphoreCreateMutex();
	power_events_mutex = xSemaphoreCreateMutex();
	waveform_buffer_mutex = xSemaphoreCreateMutex();
	
	processed_data_head = 0;
	processed_data_tail = 0;
	processed_data_count = 0;
	
	config_power_phases = MAX(config_power_phases, 1);
	config_power_phases = MIN(config_power_phases, 2);
	
	for(int evt = 0; evt < POWER_EVENT_TYPE_QTY; evt++) {
		pevents_count[0][evt] = 0;
		pevents_count[1][evt] = 0;
	}
	
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
		
		if(first_sample_rtc_time != raw_adc_data.rtc_time) {
			vrms_acc[0] = vrms_acc[1] = 0.0;
			irms_acc[0] = irms_acc[1] = 0.0;
			p_acc[0] = p_acc[1] = 0.0;
			
			first_sample_usecs = raw_adc_data.usecs_since_time;
			first_sample_rtc_time = raw_adc_data.rtc_time;
			
			raw_adc_data_processed_counter = 0;
			
			pevent_first_rise[0] = 1;
			pevent_first_rise[1] = 1;
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
		
		
		for(int ch = 0; ch < 2; ch++) {
			if(fabsf(v[ch]) > config_ac_peak_max) {
				if(pevents_count[ch][POWER_EVENT_TYPE_VOLTAGE_SPIKE] == 0)
					pevents_avg_acc[ch][POWER_EVENT_TYPE_VOLTAGE_SPIKE] = fabsf(v[ch]);
				else
					pevents_avg_acc[ch][POWER_EVENT_TYPE_VOLTAGE_SPIKE] += fabsf(v[ch]);
				
				if(fabsf(v[ch]) > fabsf(pevents_worst[ch][POWER_EVENT_TYPE_VOLTAGE_SPIKE]) || pevents_count[ch][POWER_EVENT_TYPE_VOLTAGE_SPIKE] == 0)
					pevents_worst[ch][POWER_EVENT_TYPE_VOLTAGE_SPIKE] = v[ch];
				
				pevents_count[ch][POWER_EVENT_TYPE_VOLTAGE_SPIKE]++;
			}
			
			if(last_v[ch] < 0 && v[ch] > 0) {
				if(!pevent_first_rise[ch]) {
					pevent_freq[ch] = 1000000.0 / (float) (raw_adc_data.usecs_since_time - pevent_cycle_start_us[ch]);
					pevent_vrms[ch] = sqrtf(pevent_vrms_acc[ch] / (float) pevent_rms_count[ch]);
					pevent_irms[ch] = sqrtf(pevent_irms_acc[ch] / (float) pevent_rms_count[ch]);
					
					if(pevent_freq[ch] > config_ac_frequency_max) {
						if(pevents_count[ch][POWER_EVENT_TYPE_AC_FREQUENCY_HIGH] == 0)
							pevents_avg_acc[ch][POWER_EVENT_TYPE_AC_FREQUENCY_HIGH] = pevent_freq[ch];
						else
							pevents_avg_acc[ch][POWER_EVENT_TYPE_AC_FREQUENCY_HIGH] += pevent_freq[ch];
						
						if(pevent_freq[ch] > pevents_worst[ch][POWER_EVENT_TYPE_AC_FREQUENCY_HIGH] || pevents_count[ch][POWER_EVENT_TYPE_AC_FREQUENCY_HIGH] == 0)
							pevents_worst[ch][POWER_EVENT_TYPE_AC_FREQUENCY_HIGH] = pevent_freq[ch];
						
						pevents_count[ch][POWER_EVENT_TYPE_AC_FREQUENCY_HIGH]++;
					}
					
					if(pevent_freq[ch] < config_ac_frequency_min) {
						if(pevents_count[ch][POWER_EVENT_TYPE_AC_FREQUENCY_LOW] == 0)
							pevents_avg_acc[ch][POWER_EVENT_TYPE_AC_FREQUENCY_LOW] = pevent_freq[ch];
						else
							pevents_avg_acc[ch][POWER_EVENT_TYPE_AC_FREQUENCY_LOW] += pevent_freq[ch];
						
						if(pevent_freq[ch] < pevents_worst[ch][POWER_EVENT_TYPE_AC_FREQUENCY_LOW] || pevents_count[ch][POWER_EVENT_TYPE_AC_FREQUENCY_LOW] == 0)
							pevents_worst[ch][POWER_EVENT_TYPE_AC_FREQUENCY_LOW] = pevent_freq[ch];
						
						pevents_count[ch][POWER_EVENT_TYPE_AC_FREQUENCY_LOW]++;
					}
					
					if(pevent_irms[ch] > config_max_current[ch]) {
						if(pevents_count[ch][POWER_EVENT_TYPE_OVERCURRENT] == 0)
							pevents_avg_acc[ch][POWER_EVENT_TYPE_OVERCURRENT] = pevent_irms[ch];
						else
							pevents_avg_acc[ch][POWER_EVENT_TYPE_OVERCURRENT] += pevent_irms[ch];
						
						if(pevent_irms[ch] > pevents_worst[ch][POWER_EVENT_TYPE_OVERCURRENT] || pevents_count[ch][POWER_EVENT_TYPE_OVERCURRENT] == 0)
							pevents_worst[ch][POWER_EVENT_TYPE_OVERCURRENT] = pevent_irms[ch];
						
						pevents_count[ch][POWER_EVENT_TYPE_OVERCURRENT]++;
					}
					
					if(pevent_vrms[ch] > config_ac_voltage_max) {
						if(pevents_count[ch][POWER_EVENT_TYPE_VOLTAGE_HIGH] == 0)
							pevents_avg_acc[ch][POWER_EVENT_TYPE_VOLTAGE_HIGH] = pevent_vrms[ch];
						else
							pevents_avg_acc[ch][POWER_EVENT_TYPE_VOLTAGE_HIGH] += pevent_vrms[ch];
						
						if(pevent_vrms[ch] > pevents_worst[ch][POWER_EVENT_TYPE_VOLTAGE_HIGH] || pevents_count[ch][POWER_EVENT_TYPE_VOLTAGE_HIGH] == 0)
							pevents_worst[ch][POWER_EVENT_TYPE_VOLTAGE_HIGH] = pevent_vrms[ch];
						
						pevents_count[ch][POWER_EVENT_TYPE_VOLTAGE_HIGH]++;
					}
					
					if(pevent_vrms[ch] < config_ac_voltage_min) {
						if(pevents_count[ch][POWER_EVENT_TYPE_VOLTAGE_LOW] == 0)
							pevents_avg_acc[ch][POWER_EVENT_TYPE_VOLTAGE_LOW] = pevent_vrms[ch];
						else
							pevents_avg_acc[ch][POWER_EVENT_TYPE_VOLTAGE_LOW] += pevent_vrms[ch];
						
						if(pevent_vrms[ch] < pevents_worst[ch][POWER_EVENT_TYPE_VOLTAGE_LOW] || pevents_count[ch][POWER_EVENT_TYPE_VOLTAGE_LOW] == 0)
							pevents_worst[ch][POWER_EVENT_TYPE_VOLTAGE_LOW] = pevent_vrms[ch];
						
						pevents_count[ch][POWER_EVENT_TYPE_VOLTAGE_LOW]++;
					}
				}
				
				pevent_first_rise[ch] = 0;
				
				pevent_vrms_acc[ch] = 0.0;
				pevent_irms_acc[ch] = 0.0;
				pevent_rms_count[ch] = 0;
				pevent_cycle_start_us[ch] = raw_adc_data.usecs_since_time;
			}
			
			pevent_vrms_acc[ch] += v[ch] * v[ch];
			pevent_irms_acc[ch] += i[ch] * i[ch];
			pevent_rms_count[ch]++;
			
			last_v[ch] = v[ch];
		}
		
		xSemaphoreTake(waveform_buffer_mutex, pdMS_TO_TICKS(200));
		
		waveform_buffer[0][waveform_buffer_pos] = v[0];
		waveform_buffer[1][waveform_buffer_pos] = v[1];
		waveform_buffer[2][waveform_buffer_pos] = i[0];
		waveform_buffer[3][waveform_buffer_pos] = i[1];
		
		waveform_buffer_pos = (waveform_buffer_pos + 1) % WAVEFORM_MAX_QTY;
		
		xSemaphoreGive(waveform_buffer_mutex);
		
		if((raw_adc_data.usecs_since_time - first_sample_usecs) >= 1000000) {
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
			
			for(int ch = 0; ch < 2; ch++)
				for(int evt = 0; evt < POWER_EVENT_TYPE_QTY; evt++)
					if(pevents_count[ch][evt]) {
						aux_power_event.timestamp = first_sample_rtc_time + first_sample_usecs / 1000000U;
						aux_power_event.type = evt;
						aux_power_event.channel = ch + 1;
						aux_power_event.count = pevents_count[ch][evt];
						aux_power_event.avg_value = pevents_avg_acc[ch][evt] / (float) pevents_count[ch][evt];
						aux_power_event.worst_value = pevents_worst[ch][evt];
						
						add_power_event(&aux_power_event);
						
						pevents_count[ch][evt] = 0;
					}
			
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

int add_power_event(const power_event_t *pev) {
	xSemaphoreTake(power_events_mutex, pdMS_TO_TICKS(300));
	
	if(power_events_count == POWER_EVENT_BUFFER_SIZE) {
		xSemaphoreGive(power_events_mutex);
		
		return -1;
	}
	
	memcpy(&power_events[power_events_head], pev, sizeof(power_event_t));
	
	power_events_head = (power_events_head + 1) % POWER_EVENT_BUFFER_SIZE;
	power_events_count++;
	
	if(power_events_count == POWER_EVENT_BUFFER_SIZE)
		add_ievent(IEVENT_TYPE_POWER_EVENTS_BUFFER_FULL, power_events_count, get_time());
	
	
	xSemaphoreGive(power_events_mutex);
	
	return 0;
}

int get_power_events(power_event_t *data, unsigned int index) {
	xSemaphoreTake(power_events_mutex, pdMS_TO_TICKS(300));
	
	if(index >= power_events_count) {
		xSemaphoreGive(power_events_mutex);
		return 1;
	}
	
	memcpy(data, &power_events[(power_events_tail + index) % POWER_EVENT_BUFFER_SIZE], sizeof(power_event_t));
	
	xSemaphoreGive(power_events_mutex);
	
	return 0;
}

int delete_power_events(unsigned int qty) {
	int real_qty;
	
	xSemaphoreTake(power_events_mutex, pdMS_TO_TICKS(500));
	
	real_qty = MIN(qty, power_events_count);
	
	power_events_tail = (power_events_tail + real_qty) % POWER_EVENT_BUFFER_SIZE;
	power_events_count -= real_qty;
	
	xSemaphoreGive(power_events_mutex);
	
	return real_qty;
}

void get_waveform(float *buffer, unsigned int channel, unsigned int qty) {
	xSemaphoreTake(waveform_buffer_mutex, pdMS_TO_TICKS(300));
	
	for(int i = 0; i < qty; i++)
		buffer[i] = waveform_buffer[channel][(waveform_buffer_pos + i) % WAVEFORM_MAX_QTY];
	
	xSemaphoreGive(waveform_buffer_mutex);
}
