#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <i2c/i2c.h>

#include "esp8266.h"
#include "ads111x.h"

#include <FreeRTOS.h>
#include <task.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define I2C_BUS 0
#define SCL_PIN 5
#define SDA_PIN 4

#define WIFI_SSID "Batata"
#define WIFI_PSK "***REMOVED***"

#define SERVER_ADDR "192.168.1.50"
#define SERVER_PORT 2048

#define GAIN_ADC1 ADS111X_GAIN_0V256
#define GAIN_ADC2 ADS111X_GAIN_4V096

i2c_dev_t adc1 = {
	.addr = ADS111X_ADDR_GND,
	.bus = I2C_BUS,
};

i2c_dev_t adc2 = {
	.addr = ADS111X_ADDR_VCC,
	.bus = I2C_BUS,
};

int send_socket;
struct sockaddr_in server_addr;

void send_task(void *pvParameters) {
	int16_t raw_adc1, raw_adc2;
	float v_adc1, v_adc2;
	char aux[64];
	char tbuf[255];
	
	vTaskDelay(pdMS_TO_TICKS(500));
	printf("Waiting for wireless connection...");
	
	while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	
	printf(" Connected!\n");
	fflush(stdout);
	
	while(1){
		send_socket = socket(PF_INET, SOCK_STREAM, 0);
		
		if(send_socket < 0) {
			printf("Failed to create socket.\n");
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}
		
		if(connect(send_socket, (struct sockaddr *) &server_addr, sizeof(server_addr)) == 0)
			break;
		
		printf("Failed to connect to server! (%d)\n", errno);
		close(send_socket);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	
	printf("Connected to server!\n");
	
	int buffer_counter = 0;
	int print_counter = 0;
	tbuf[0] = '\0';
	
	while (true) {
		ads111x_start_conversion(&adc1);
		ads111x_start_conversion(&adc2);
		
		vTaskDelay(pdMS_TO_TICKS(1));
		
		while (!ads111x_busy(&adc1) || !ads111x_busy(&adc2)) {}
		
		raw_adc1 = ads111x_get_value(&adc1);
		raw_adc2 = ads111x_get_value(&adc2);
		
		v_adc1 = ads111x_gain_values[GAIN_ADC1] / ADS111X_MAX_VALUE * raw_adc1;
		v_adc2 = ads111x_gain_values[GAIN_ADC2] / ADS111X_MAX_VALUE * raw_adc2;
		
		v_adc1 = v_adc1 * 440000.0 / 120.0;
		
		sprintf(aux, "%.4f %.4f\n", v_adc1, v_adc2);
		strcat(tbuf, aux);
		
		if(++print_counter == 860) {
			printf("%.4f %.4f\n", v_adc1, v_adc2);
			print_counter = 0;
		}
		
		if(++buffer_counter == 10) {
			send(send_socket, tbuf, strlen(tbuf) + 1, 0);
			buffer_counter = 0;
			tbuf[0] = '\0';
		}
	}
}

void adc_config() {
	//ads111x_set_mode(&dev, ADS111X_MODE_CONTINUOUS);
	ads111x_set_data_rate(&adc1, ADS111X_DATA_RATE_860);
	ads111x_set_data_rate(&adc2, ADS111X_DATA_RATE_860);
	
	//ads111x_set_input_mux(&adc1, ADS111X_MUX_0_GND);
	//ads111x_set_input_mux(&adc2, ADS111X_MUX_0_GND);
	
	ads111x_set_gain(&adc1, GAIN_ADC1);
	ads111x_set_gain(&adc2, GAIN_ADC2);
}

void user_init(void) {
	uart_set_baud(0, 115200);
	printf("SDK version:%s\n", sdk_system_get_sdk_version());
	
	struct sdk_station_config wifi_config = {
		.ssid = WIFI_SSID,
		.password = WIFI_PSK,
	};
	
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&wifi_config);
	
	i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_400K);
	
	adc_config();
	
	//gpio_enable(16, GPIO_OUTPUT);
	//gpio_write(16, 0);
	
	//vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	bzero(&server_addr, sizeof(server_addr));
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(SERVER_ADDR);
	server_addr.sin_port = htons(SERVER_PORT);
	
	printf("Starting send task.\n");
	
	xTaskCreate(send_task, "send_task", 512, NULL, 2, NULL);
}
