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

uint32_t rtc_new_time, rtc_new_time_sysclock_reference;

int read_rtc() {
	struct tm time;
	bool osf;
	
	ds3231_getOscillatorStopFlag(&rtc_dev, &osf);
	
	if(osf) {
		rtc_oscillator_stopped++;
		return -1;
	}
	
	ds3231_getTime(&rtc_dev, &time);
	rtc_time_sysclock_reference = sdk_system_get_time();
	rtc_time = mktime(&time);
	
	return 0;
}

void update_rtc(const time_t new_time) {
	struct tm time;
	bool osf;
	
	ds3231_getOscillatorStopFlag(&rtc_dev, &osf);
	
	if(osf) {
		ds3231_clearOscillatorStopFlag(&rtc_dev);
		rtc_oscillator_stopped++;
		return;
	}
	
	gmtime_r(&new_time, &time);
	
	ds3231_setTime(&rtc_dev, &time);
}
