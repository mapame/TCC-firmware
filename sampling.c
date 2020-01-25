#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <i2c/i2c.h>

#include <esp8266.h>

#include <FreeRTOS.h>
#include <task.h>

#include "ads111x.h"

#include "common.h"
#include "rtc.h"
#include "sampling.h"
#include "configuration.h"
#include "ievents.h"

ads111x_dev_t adc_device[3];

float adc_volt_scale[3];

uint8_t sampling_running;

uint16_t adc_channel_switch_period;
uint16_t adc_channel_counter;
uint8_t adc_actual_channel, adc_next_channel;

int16_t raw_adc_data[5][RAW_ADC_DATA_BUFFER_SIZE];
uint32_t raw_adc_rtc_time[RAW_ADC_DATA_BUFFER_SIZE];
uint32_t raw_adc_usecs_since_time[RAW_ADC_DATA_BUFFER_SIZE];
uint16_t raw_adc_data_head, raw_adc_data_tail, raw_adc_data_count;

void IRAM ads_ready_handle(uint8_t gpio_num) {
	if(((raw_adc_data_head + 1) % RAW_ADC_DATA_BUFFER_SIZE) == raw_adc_data_tail) { // Stop sampling on buffer full error
		sampling_running = 0;
		debug("Raw buffer full!\n");
		add_internal_event(INTERNAL_EVENT_BUFFER_FULL, raw_adc_data_head);
		add_internal_event(INTERNAL_EVENT_BUFFER_FULL, raw_adc_data_tail);
		add_internal_event(INTERNAL_EVENT_BUFFER_FULL, raw_adc_data_count);
		add_internal_event(INTERNAL_EVENT_BUFFER_FULL, 1);
		return;
	}
	
	if(sampling_running) {
		if(++adc_channel_counter >= adc_channel_switch_period) {
			adc_channel_counter = 0;
			adc_next_channel = (adc_next_channel + 1) % 2;
			ads111x_set_input_mux(&adc_device[0], (adc_next_channel == 0) ? ADS111X_MUX_0_1 : ADS111X_MUX_2_3);
			ads111x_set_input_mux(&adc_device[2], (adc_next_channel == 0) ? ADS111X_MUX_0_1 : ADS111X_MUX_2_3);
		}
		
		ads111x_start_conversion(&adc_device[0]);
		ads111x_start_conversion(&adc_device[1]);
		ads111x_start_conversion(&adc_device[2]);
	}
	
	uint32_t sysclock_actual_value = sdk_system_get_time();
	
	raw_adc_data[(adc_actual_channel == 0) ? 0 : 1][raw_adc_data_head] = ads111x_get_value(&adc_device[0]);
	raw_adc_data[2][raw_adc_data_head] = ads111x_get_value(&adc_device[1]);
	raw_adc_data[(adc_actual_channel == 0) ? 3 : 4][raw_adc_data_head] = ads111x_get_value(&adc_device[2]);
	
	uint16_t inactive_channel_buffer_position = raw_adc_data_head - adc_channel_switch_period + ((raw_adc_data_head - adc_channel_switch_period < 0) ? RAW_ADC_DATA_BUFFER_SIZE : 0);
	
	raw_adc_data[(adc_actual_channel == 0) ? 1 : 0][raw_adc_data_head] = raw_adc_data[(adc_actual_channel == 0) ? 1 : 0][inactive_channel_buffer_position];
	raw_adc_data[(adc_actual_channel == 0) ? 4 : 3][raw_adc_data_head] = raw_adc_data[(adc_actual_channel == 0) ? 4 : 3][inactive_channel_buffer_position];
	
	adc_actual_channel = adc_next_channel;
	
	raw_adc_data_head = (raw_adc_data_head + 1) % RAW_ADC_DATA_BUFFER_SIZE;
	raw_adc_data_count++;
	
	raw_adc_rtc_time[raw_adc_data_head] = rtc_time;
	
	if(sysclock_actual_value < rtc_time_sysclock_reference)
		raw_adc_usecs_since_time[raw_adc_data_head] = (((uint32_t)0xFFFFFFFF) - rtc_time_sysclock_reference) + sysclock_actual_value + ((uint32_t)1);
	else
		raw_adc_usecs_since_time[raw_adc_data_head] = sysclock_actual_value - rtc_time_sysclock_reference;
	
	if(raw_adc_usecs_since_time[raw_adc_data_head] >= RTC_READ_PERIOD_US)
		read_rtc_time();
}

void start_sampling() {
	raw_adc_data_head = 0;
	raw_adc_data_tail = 0;
	raw_adc_data_count = 0;
	adc_channel_counter = 0;
	adc_actual_channel = 0;
	adc_next_channel = 0;
	
	adc_channel_switch_period = 11; // Hardcoded starting value
	
	read_rtc_temp();
	
	if(read_rtc_time() < 0) {
		return;
	}
	raw_adc_rtc_time[raw_adc_data_head] = rtc_time;
	raw_adc_usecs_since_time[raw_adc_data_head] = 0;
	
	sampling_running = 1;
	
	ads111x_start_conversion(&adc_device[0]);
	ads111x_start_conversion(&adc_device[1]);
	ads111x_start_conversion(&adc_device[2]);
}

void pause_sampling() {
	sampling_running = 0;
	vTaskDelay(pdMS_TO_TICKS(50));
}

void adc_config() {
	ads111x_init(&adc_device[0], I2C_BUS, ADS111X_ADDR_GND);
	ads111x_init(&adc_device[1], I2C_BUS, ADS111X_ADDR_VCC);
	ads111x_init(&adc_device[2], I2C_BUS, ADS111X_ADDR_SDA);
	
	ads111x_set_data_rate(&adc_device[0], ADS111X_DATA_RATE_860);
	ads111x_set_data_rate(&adc_device[1], ADS111X_DATA_RATE_860);
	ads111x_set_data_rate(&adc_device[2], ADS111X_DATA_RATE_860);
	
	ads111x_set_input_mux(&adc_device[0], ADS111X_MUX_0_1);
	ads111x_set_input_mux(&adc_device[1], ADS111X_MUX_0_1);
	ads111x_set_input_mux(&adc_device[2], ADS111X_MUX_0_1);
	
	ads111x_set_gain(&adc_device[0], GAIN_ADC1);
	ads111x_set_gain(&adc_device[1], GAIN_ADC2);
	ads111x_set_gain(&adc_device[2], GAIN_ADC3);
	
	ads111x_set_comp_high_thresh(&adc_device[0], 0x8000);
	ads111x_set_comp_high_thresh(&adc_device[1], 0x8000);
	ads111x_set_comp_high_thresh(&adc_device[2], 0x8000);
	ads111x_set_comp_low_thresh(&adc_device[0], 0x7FFF);
	ads111x_set_comp_low_thresh(&adc_device[1], 0x7FFF);
	ads111x_set_comp_low_thresh(&adc_device[2], 0x7FFF);
	
	ads111x_set_comp_queue(&adc_device[0], ADS111X_COMP_QUEUE_1);
	ads111x_set_comp_queue(&adc_device[1], ADS111X_COMP_QUEUE_1);
	ads111x_set_comp_queue(&adc_device[2], ADS111X_COMP_QUEUE_1);
	
	ads111x_set_comp_polarity(&adc_device[0], ADS111X_COMP_POLARITY_HIGH);
	ads111x_set_comp_polarity(&adc_device[1], ADS111X_COMP_POLARITY_HIGH);
	ads111x_set_comp_polarity(&adc_device[2], ADS111X_COMP_POLARITY_HIGH);
	
	ads111x_push_config(&adc_device[0]);
	ads111x_push_config(&adc_device[1]);
	ads111x_push_config(&adc_device[2]);
	
	gpio_set_interrupt(READY_PIN, GPIO_INTTYPE_EDGE_POS, ads_ready_handle);
	
	adc_volt_scale[0] = ads111x_gain_values[GAIN_ADC1] / (float)ADS111X_MAX_VALUE;
	adc_volt_scale[1] = ads111x_gain_values[GAIN_ADC2] / (float)ADS111X_MAX_VALUE;
	adc_volt_scale[2] = ads111x_gain_values[GAIN_ADC3] / (float)ADS111X_MAX_VALUE;
}
