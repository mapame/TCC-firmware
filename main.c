#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <i2c/i2c.h>
#include <dhcpserver.h>

#include <FreeRTOS.h>
#include <task.h>
#include <message_buffer.h>
#include <semphr.h>

#include "common.h"
#include "sampling.h"
#include "power.h"
#include "events.h"
#include "rtc.h"
#include "configuration.h"
#include "communication.h"

TaskHandle_t power_processing_task_handle, blink_task_handle;

unsigned int status_sampling_running = 0;
unsigned int status_server_connected = 0;

void httpd_task(void *pvParameters);

void set_led_color(int color);

void IRAM blink_task(void *pvParameters) {
	int cycle = 0;
	while(1){
		if(cycle) {
			if(status_server_connected)
				set_led_color(LED_COLOR_GREEN);
			else if(sdk_wifi_station_get_connect_status() == STATION_GOT_IP)
				set_led_color(LED_COLOR_BLUE);
			else
				set_led_color(LED_COLOR_YELLOW);
		} else {
			if(status_sampling_running)
				set_led_color(LED_OFF);
			else
				set_led_color(LED_COLOR_RED);
		}
		
		cycle = (cycle + 1) % 2;
		
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

void set_led_color(int color) {
	gpio_write(LED_R_PIN, color & 1);
	gpio_write(LED_G_PIN, ((color >> 1) & 1) == 0);
	gpio_write(LED_B_PIN, ((color >> 2) & 1) == 0);
}

void user_init(void) {
	int button;
	
	struct sdk_station_config wifi_config;
	
	uart_set_baud(0, 115200);
	
	debug("Firmware version: "FW_VERSION"\n");
	debug("Build date: "__DATE__" "__TIME__"\n");
	
	gpio_enable(READY_PIN, GPIO_INPUT);
	gpio_enable(LED_R_PIN, GPIO_OUTPUT);
	gpio_enable(LED_G_PIN, GPIO_OUTPUT);
	gpio_enable(LED_B_PIN, GPIO_OUTPUT);
	gpio_enable(BTN_PIN, GPIO_INPUT);
	
	gpio_set_pullup(BTN_PIN, 0, 0);
	
	set_led_color(LED_OFF);
	
	vTaskDelay(pdMS_TO_TICKS(500));
	
	events_init();
	
	debug("Initializing I2C bus.\n");
	
	if(i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_500K)) {
		debug("Failed to initialize i2c bus!\n");
		return;
	}
	
	init_rtc();
	
	button = 0;
	for(int i = LED_COLOR_RED; i <= LED_COLOR_TEAL; i++) {
		set_led_color(i);
		
		button += gpio_read(BTN_PIN);
		
		vTaskDelay(pdMS_TO_TICKS(400));
	}
	
	load_configuration();
	
	if(button > 3
		|| strlen(config_device_id) < 1
		|| strlen(config_wifi_ssid) < 1
		|| strlen(config_wifi_password) < 1
		|| strlen(config_mac_password) < 1
		|| strlen(config_server_ip) < 1) {
		
		struct sdk_softap_config ap_config;
		struct ip_info ap_ip;
		ip4_addr_t dhcp_first_ip;
		
		debug("Device not configured.\n");
		set_led_color(LED_COLOR_WHITE);
		
		sdk_wifi_set_opmode(SOFTAP_MODE);
		
		IP4_ADDR(&ap_ip.ip, 10, 0, 0, 1);
		IP4_ADDR(&ap_ip.gw, 0, 0, 0, 0);
		IP4_ADDR(&ap_ip.netmask, 255, 255, 0, 0);
		
		sdk_wifi_set_ip_info(1, &ap_ip);
		
		strcpy((char *)ap_config.ssid, WIFI_AP_SSID);
		strcpy((char *)ap_config.password, config_wifi_ap_password);
		
		ap_config.ssid_len = strlen(WIFI_AP_SSID);
		ap_config.ssid_hidden = 0;
		ap_config.channel = 1;
		ap_config.authmode = AUTH_WPA_WPA2_PSK;
		ap_config.max_connection = 2;
		ap_config.beacon_interval = 100;
		
		sdk_wifi_softap_set_config(&ap_config);
		
		dhcp_first_ip.addr = ap_ip.ip.addr + htonl(1);
		
		dhcpserver_start(&dhcp_first_ip, 3);
		dhcpserver_set_router(&ap_ip.ip);
		
		xTaskCreate(&httpd_task, "httpd_task", 1024, NULL, 2, NULL);
		
		return;
	}
	
	strcpy((char *)wifi_config.ssid, config_wifi_ssid);
	strcpy((char *)wifi_config.password, config_wifi_password);
	
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&wifi_config);
	
	raw_adc_data_buffer = xMessageBufferCreate(RAW_ADC_DATA_BUFFER_SIZE * (sizeof(raw_adc_data_t) + 4));
	
	gpio_set_interrupt(READY_PIN, GPIO_INTTYPE_EDGE_POS, ads_ready_handle);
	
	debug("Starting tasks.\n");
	
	xTaskCreate(power_processing_task, "power_processing_task", 512, NULL, 3, &power_processing_task_handle);
	xTaskCreate(network_task, "network_task", 1024, NULL, 2, NULL);
	xTaskCreate(blink_task, "blink_task", 256, NULL, 1, &blink_task_handle);
}
