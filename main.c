#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <i2c/i2c.h>

#include <FreeRTOS.h>
#include <task.h>
#include <message_buffer.h>

#include "common.h"
#include "sampling.h"
#include "power.h"
#include "rtc.h"
#include "configuration.h"
#include "communication.h"

TaskHandle_t power_processing_task_handle, blink_task_handle;

uint8_t sampling_running;

void IRAM blink_task(void *pvParameters) {
	int cycle = 0;
	while(1){
		//gpio_toggle(LED_R_PIN);
		gpio_write(((sampling_running) ? LED_R_PIN : LED_B_PIN), ((sampling_running) ? 0 : 1));
		gpio_write(((sampling_running) ? LED_B_PIN : LED_R_PIN), cycle);
		
		cycle = (cycle + 1) % 2;
		
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

void user_init(void) {
	uart_set_baud(0, 115200);
	
	gpio_enable(READY_PIN, GPIO_INPUT);
	gpio_enable(LED_R_PIN, GPIO_OUTPUT);
	gpio_enable(LED_G_PIN, GPIO_OUTPUT);
	gpio_enable(LED_B_PIN, GPIO_OUTPUT);
	gpio_enable(BTN_PIN, GPIO_INPUT);
	
	for(int i = 1; i < 17; i++) {
		gpio_write(LED_R_PIN, i & 1);
		gpio_write(LED_G_PIN, ((i >> 1) & 1) == 0);
		gpio_write(LED_B_PIN, ((i >> 2) & 1) == 0);
		
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	
	debug("Firmware version: "FW_VERSION"\n");
	debug("Build date: "__DATE__" "__TIME__"\n");
	
	load_configuration();
	
	if(strlen(config_wifi_ssid) < 1 || strcmp(config_wifi_ssid, "###NOT_SET###") == 0
	|| strlen(config_wifi_password) < 1 || strcmp(config_wifi_password, "###NOT_SET###") == 0
	|| strlen(config_mac_password) < 1 || strcmp(config_mac_password, "###NOT_SET###") == 0) {
		debug("Device not configured.\n");
		return;
	}
	
	struct sdk_station_config wifi_config;
	
	strcpy((char *)wifi_config.ssid, config_wifi_ssid);
	strcpy((char *)wifi_config.password, config_wifi_password);
	
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&wifi_config);
	
	vTaskDelay(pdMS_TO_TICKS(100));
	
	debug("Initializing I2C bus.\n");
	
	if(i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_500K)) {
		debug("Failed to initialize i2c bus!\n");
		return;
	}
	
	init_rtc();
	
	debug("Configuring ADCs.\n");
	
	adc_config();
	
	raw_adc_data_buffer = xMessageBufferCreate(RAW_ADC_DATA_BUFFER_SIZE * (sizeof(raw_adc_data_t) + 4));
	
	debug("Starting tasks.\n");
	
	xTaskCreate(power_processing_task, "power_processing_task", 512, NULL, 3, &power_processing_task_handle);
	xTaskCreate(network_task, "network_task", 768, NULL, 2, NULL);
	xTaskCreate(blink_task, "blink_task", 256, NULL, 1, &blink_task_handle);
}
