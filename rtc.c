#include <espressif/esp_common.h>
#include <esp8266.h>
#include <i2c/i2c.h>

#include <stdio.h>
#include <stdlib.h>

#include <time.h>
#include <ds3231/ds3231.h>

i2c_dev_t rtc_dev = {.addr = DS3231_ADDR, .bus = 0};

uint32_t rtc_time, rtc_time_sysclock_reference;
uint8_t rtc_oscillator_stopped;

float rtc_temp = 0.0;

int read_rtc_time() {
	struct tm time;
	bool osf;
	
	ds3231_getOscillatorStopFlag(&rtc_dev, &osf);
	
	if(osf) {
		rtc_oscillator_stopped++;
		return -1;
	}
	
	ds3231_getTime(&rtc_dev, &time);
	rtc_time_sysclock_reference = sdk_system_get_time();
	rtc_time = (uint32_t) mktime(&time);
	
	return 0;
}

int read_rtc_temp() {
	return (int) ds3231_getTempFloat(&rtc_dev, &rtc_temp);
}

void update_rtc(uint32_t new_time) {
	time_t new_time_aux;
	struct tm new_time_tm;
	bool osf;
	
	ds3231_getOscillatorStopFlag(&rtc_dev, &osf);
	
	if(osf)
		ds3231_clearOscillatorStopFlag(&rtc_dev);
	
	new_time_aux = new_time;
	
	gmtime_r(&new_time_aux, &new_time_tm);
	
	ds3231_setTime(&rtc_dev, &new_time_tm);
}
