#include <espressif/esp_common.h>
#include <esp8266.h>
#include <i2c/i2c.h>

#include <stdio.h>
#include <stdlib.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include <time.h>
#include <ds3231/ds3231.h>

#include "common.h"
#include "ievents.h"

i2c_dev_t rtc_dev = {.addr = DS3231_ADDR, .bus = 0};

SemaphoreHandle_t rtc_mutex = NULL;

uint32_t rtc_time, rtc_time_sysclock_reference;
uint8_t rtc_oscillator_stopped;

float rtc_temp = 0.0;

int init_rtc() {
	rtc_mutex = xSemaphoreCreateMutex();
	
	return 0;
}

int read_rtc_time() {
	struct tm time;
	bool osf;
	
	if(!ds3231_getOscillatorStopFlag(&rtc_dev, &osf))
		return -1;
	
	if(osf) {
		rtc_oscillator_stopped++;
		return -2;
	}
	
	if(!ds3231_getTime(&rtc_dev, &time))
		return -1;
	
	rtc_time_sysclock_reference = sdk_system_get_time();
	rtc_time = (uint32_t) mktime(&time);
	
	return 0;
}

int read_rtc_temp() {
	return (ds3231_getTempFloat(&rtc_dev, &rtc_temp)) ? 0 : -1;
}

uint32_t get_time() {
	uint32_t system_time_now;
	uint32_t result_us_time;
	
	int result;
	
	if(!status_sampling_running) {
		if(!xSemaphoreTake(rtc_mutex, pdMS_TO_TICKS(500)))
			return 0;
		
		result = read_rtc_time();
		if(result) {
			xSemaphoreGive(rtc_mutex);
			
			if(result == -1)
				add_internal_event(IEVENT_TYPE_I2C_ERROR, 2, rtc_time);
			
			return 0;
		}
		
		xSemaphoreGive(rtc_mutex);
	}
	
	system_time_now = sdk_system_get_time();
	
	if(system_time_now < rtc_time_sysclock_reference)
		result_us_time = (((uint32_t)0xFFFFFFFF) - rtc_time_sysclock_reference) + system_time_now + ((uint32_t)1);
	else
		result_us_time = system_time_now - rtc_time_sysclock_reference;
	
	return rtc_time + result_us_time / 1000000U;
}

float get_temp() {
	if(!status_sampling_running) {
		if(xSemaphoreTake(rtc_mutex, pdMS_TO_TICKS(500))) {
			read_rtc_temp();
			xSemaphoreGive(rtc_mutex);
		}
	}
	
	return rtc_temp;
}

int update_rtc(uint32_t new_time) {
	time_t new_time_aux;
	struct tm new_time_tm;
	
	if(status_sampling_running)
		return -3;
	
	new_time_aux = new_time;
	
	gmtime_r(&new_time_aux, &new_time_tm);
	
	if(!xSemaphoreTake(rtc_mutex, pdMS_TO_TICKS(500)))
		return -1;
	
	if(!ds3231_clearOscillatorStopFlag(&rtc_dev)) {
		add_internal_event(IEVENT_TYPE_I2C_ERROR, 2, rtc_time);
		
		return -2;
	}
	
	if(ds3231_setTime(&rtc_dev, &new_time_tm)) {
		add_internal_event(IEVENT_TYPE_I2C_ERROR, 2, rtc_time);
		
		return -2;
	}
	
	xSemaphoreGive(rtc_mutex);
	
	return 0;
}
