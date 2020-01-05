#include <espressif/esp_common.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <esp8266.h>

#include <FreeRTOS.h>
#include <task.h>

#include "common.h"
#include "sampling.h"
#include "power.h"

power_data_t processed_data[PROCESSED_DATA_BUFFER_SIZE];
uint16_t processed_data_head, processed_data_tail, processed_data_count;

power_event_t power_events[POWER_EVENT_BUFFER_SIZE];
uint16_t power_events_data_head, power_events_data_tail, power_events_data_count;

void power_processing_task(void *pvParameters) {
	int16_t raw_adc_data_tmp[5];
	uint32_t raw_adc_usecs_tmp;
	uint32_t raw_adc_rtc_time_tmp;
	
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
	
	while(true) {
		if(!raw_adc_data_count) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}
		
		raw_adc_data_tmp[0] = raw_adc_data[0][raw_adc_data_tail];
		raw_adc_data_tmp[1] = raw_adc_data[1][raw_adc_data_tail];
		raw_adc_data_tmp[2] = raw_adc_data[2][raw_adc_data_tail];
		raw_adc_data_tmp[3] = raw_adc_data[3][raw_adc_data_tail];
		raw_adc_data_tmp[4] = raw_adc_data[4][raw_adc_data_tail];
		raw_adc_usecs_tmp = raw_adc_usecs_since_time[raw_adc_data_tail];
		raw_adc_rtc_time_tmp = raw_adc_rtc_time[raw_adc_data_tail];
		
		raw_adc_data_tail = (raw_adc_data_tail + 1) % RAW_ADC_DATA_BUFFER_SIZE;
		raw_adc_data_count--;
		
		if(raw_adc_data_processed_counter == 0) {
			first_sample_usecs = raw_adc_usecs_tmp;
			first_sample_rtc_time = raw_adc_rtc_time_tmp;
		}
		
		raw_adc_data_processed_counter++;
		
		v[0] = (adc_volt_scale[0] * (float)raw_adc_data_tmp[0]) * 440000.0 / 120.0;
		v[1] = (adc_volt_scale[0] * (float)raw_adc_data_tmp[1]) * 440000.0 / 120.0;
		v[2] = v[0] + v[1];
		
		i[0] = (adc_volt_scale[1] * (float)raw_adc_data_tmp[2]) * 2000.0 / 23.2;
		i[1] = (adc_volt_scale[2] * (float)raw_adc_data_tmp[3]) * 2000.0 / 23.2;
		i[2] = (adc_volt_scale[2] * (float)raw_adc_data_tmp[4]) * 2000.0 / 23.2;
		
		vrms_acc[0] += v[0] * v[0];
		vrms_acc[1] += v[1] * v[1];
		vrms_acc[2] += v[2] * v[2];
		
		irms_acc[0] += i[0] * i[0];
		irms_acc[1] += i[1] * i[1];
		irms_acc[2] += i[2] * i[2];
		
		p_acc[0] += v[0] * i[0];
		p_acc[1] += v[1] * i[1];
		p_acc[2] += v[2] * i[2];
		
		if((raw_adc_usecs_tmp - first_sample_usecs) >= 250000 || first_sample_rtc_time != raw_adc_rtc_time_tmp) {
			processed_data[processed_data_head].timestamp = first_sample_rtc_time + first_sample_usecs / 1000000;
			processed_data[processed_data_head].duration_usec = raw_adc_usecs_tmp - first_sample_usecs;
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
