#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <i2c/i2c.h>

#include <esp8266.h>

#include <FreeRTOS.h>
#include <task.h>

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>

#include <sysparam.h>

#include "ads111x.h"

#define SCL_PIN 5
#define SDA_PIN 4
#define LED_R_PIN 2
#define LED_G_PIN 12
#define LED_B_PIN 13
#define BTN_PIN 15
#define READY_PIN 14

#define I2C_BUS 0

#define GAIN_ADC1 ADS111X_GAIN_0V256
#define GAIN_ADC2 ADS111X_GAIN_2V048
#define GAIN_ADC3 ADS111X_GAIN_2V048

#define RAW_ADC_DATA_BUFFER_SIZE 2000
#define SEND_BUFFER_SIZE 700

#define SERVER_ADDR "192.168.1.50"
#define SERVER_PORT 2048

typedef struct send_data {
	char test;
	float data[3];
} send_data_t;

ads111x_dev_t adc1, adc2, adc3;

int16_t raw_adc1, raw_adc2, raw_adc3;

int16_t raw_adc_data[3][RAW_ADC_DATA_BUFFER_SIZE];
uint16_t raw_adc_data_head, raw_adc_data_tail, raw_adc_data_count, raw_adc_data_lost;

send_data_t send_buffer[SEND_BUFFER_SIZE];

void IRAM ads_ready_handle(uint8_t gpio_num) {
	//gpio_write(13, 1);
	ads111x_start_conversion(&adc1);
	ads111x_start_conversion(&adc2);
	ads111x_start_conversion(&adc3);
	
	raw_adc_data_head = (raw_adc_data_head + 1) % RAW_ADC_DATA_BUFFER_SIZE;
	if(((raw_adc_data_head + 9) % RAW_ADC_DATA_BUFFER_SIZE) == raw_adc_data_tail) {
		raw_adc_data_tail = (raw_adc_data_tail + 1) % RAW_ADC_DATA_BUFFER_SIZE;
		raw_adc_data_lost++;
	} else {
		raw_adc_data_count++;
	}
	
	raw_adc_data[0][raw_adc_data_head] = ads111x_get_value(&adc1);
	raw_adc_data[1][raw_adc_data_head] = ads111x_get_value(&adc2);
	raw_adc_data[2][raw_adc_data_head] = ads111x_get_value(&adc3);
	//gpio_write(13, 0);
}

void IRAM blink_task(void *pvParameters) {
	while(1){
		gpio_toggle(LED_R_PIN);
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

void IRAM send_task(void *pvParameters) {
	int send_socket;
	struct sockaddr_in server_addr;
	
	char aux[64];
	
	ads111x_start_conversion(&adc1);
	ads111x_start_conversion(&adc2);
	ads111x_start_conversion(&adc3);
	
	raw_adc_data_head = 0;
	raw_adc_data_tail = 0;
	raw_adc_data_count = 0;
	raw_adc_data_lost = 0;
	
	vTaskDelay(pdMS_TO_TICKS(500));
	printf("Waiting for wireless connection...");
	
	while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	
	printf("Connected to network!\n");
	fflush(stdout);
	
	bzero(&server_addr, sizeof(server_addr));
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(SERVER_ADDR);
	server_addr.sin_port = htons(SERVER_PORT);
	
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
	
	//sprintf(aux, "Hello!\n");
	//send(send_socket, aux, strlen(aux) + 1, 0);
	
	float v_adc1, v_adc2, v_adc3;
	int16_t raw_adc_data_tmp[3];
	
	int buffer_counter = 0;
	uint16_t read_tail;
	long int total_data_lost = 0;
	//tbuf[0] = '\0';
	
	while (true) {
		//gpio_write(12, 1);
		
		while(raw_adc_data_count && buffer_counter < SEND_BUFFER_SIZE) {
			read_tail = raw_adc_data_tail;
			raw_adc_data_tmp[0] = raw_adc_data[0][read_tail];
			raw_adc_data_tmp[1] = raw_adc_data[1][read_tail];
			raw_adc_data_tmp[2] = raw_adc_data[2][read_tail];
			
			raw_adc_data_tail = (raw_adc_data_tail + 1) % RAW_ADC_DATA_BUFFER_SIZE;
			raw_adc_data_count--;
			
			send_buffer[buffer_counter].data[0] = ads111x_gain_values[GAIN_ADC1] / (float)ADS111X_MAX_VALUE * (float)raw_adc_data_tmp[0];
			send_buffer[buffer_counter].data[1] = ads111x_gain_values[GAIN_ADC2] / (float)ADS111X_MAX_VALUE * (float)raw_adc_data_tmp[1];
			send_buffer[buffer_counter].data[2] = ads111x_gain_values[GAIN_ADC3] / (float)ADS111X_MAX_VALUE * (float)raw_adc_data_tmp[2];
			send_buffer[buffer_counter].test = 'A';
			
			//v_adc1 = v_adc1 * 440000.0 / 120.0;
			buffer_counter++;
		}
		
		if(buffer_counter >= SEND_BUFFER_SIZE) {
			if(send(send_socket, send_buffer, buffer_counter * sizeof(send_data_t), 0) > 0) {
				printf("Sent %d captures! (%d lost, total %ld)\n", buffer_counter, raw_adc_data_lost, total_data_lost);
				total_data_lost += raw_adc_data_lost;
				raw_adc_data_lost = 0;
			} else {
				raw_adc_data_lost += buffer_counter / 3;
				printf("Error sending data!\n");
			}
			
			buffer_counter = 0;
		}
		//gpio_write(12, 0);
	}
}

void adc_config() {
	ads111x_init(&adc1, I2C_BUS, ADS111X_ADDR_GND);
	ads111x_init(&adc2, I2C_BUS, ADS111X_ADDR_VCC);
	ads111x_init(&adc3, I2C_BUS, ADS111X_ADDR_SDA);
	
	ads111x_set_data_rate(&adc1, ADS111X_DATA_RATE_860);
	ads111x_set_data_rate(&adc2, ADS111X_DATA_RATE_860);
	ads111x_set_data_rate(&adc3, ADS111X_DATA_RATE_860);
	
	ads111x_set_input_mux(&adc1, ADS111X_MUX_2_3);
	ads111x_set_input_mux(&adc2, ADS111X_MUX_0_1);
	ads111x_set_input_mux(&adc3, ADS111X_MUX_0_1);
	
	ads111x_set_gain(&adc1, GAIN_ADC1);
	ads111x_set_gain(&adc2, GAIN_ADC2);
	ads111x_set_gain(&adc3, GAIN_ADC3);
	
	ads111x_set_comp_high_thresh(&adc1, 0x8000);
	ads111x_set_comp_high_thresh(&adc2, 0x8000);
	ads111x_set_comp_high_thresh(&adc3, 0x8000);
	ads111x_set_comp_low_thresh(&adc1, 0x7FFF);
	ads111x_set_comp_low_thresh(&adc2, 0x7FFF);
	ads111x_set_comp_low_thresh(&adc3, 0x7FFF);
	
	ads111x_set_comp_queue(&adc1, ADS111X_COMP_QUEUE_1);
	ads111x_set_comp_queue(&adc2, ADS111X_COMP_QUEUE_1);
	ads111x_set_comp_queue(&adc3, ADS111X_COMP_QUEUE_1);
	
	ads111x_set_comp_polarity(&adc1, ADS111X_COMP_POLARITY_HIGH);
	ads111x_set_comp_polarity(&adc2, ADS111X_COMP_POLARITY_HIGH);
	ads111x_set_comp_polarity(&adc3, ADS111X_COMP_POLARITY_HIGH);
	
	ads111x_push_config(&adc1);
	ads111x_push_config(&adc2);
	ads111x_push_config(&adc3);
}

void user_init(void) {
	char *wifi_ssid;
	char *wifi_password;
	
	uart_set_baud(0, 115200);
	
	gpio_enable(READY_PIN, GPIO_INPUT);
	gpio_enable(LED_R_PIN, GPIO_OUTPUT);
	gpio_enable(LED_G_PIN, GPIO_OUTPUT);
	gpio_enable(LED_B_PIN, GPIO_OUTPUT);
	gpio_enable(BTN_PIN, GPIO_INPUT);
	
	for(int i = 1; i < 9; i++) {
		gpio_write(LED_R_PIN, i & 1);
		gpio_write(LED_G_PIN, ((i >> 1) & 1) == 0);
		gpio_write(LED_B_PIN, ((i >> 2) & 1) == 0);
		
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	
	if(sysparam_get_string("wifi_ap_ssid", &wifi_ssid) != SYSPARAM_OK || strlen(wifi_ssid) < 1
	|| sysparam_get_string("wifi_ap_password", &wifi_password) != SYSPARAM_OK || strlen(wifi_password) < 8) {
		printf("WiFi data not avaliable or invalid.\n");
		return;
	}
	
	struct sdk_station_config wifi_config;
	
	strcpy(wifi_config.ssid, wifi_ssid);
	strcpy(wifi_config.password, wifi_password);
	
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&wifi_config);
	
	vTaskDelay(pdMS_TO_TICKS(100));
	
	printf("Initializing I2C bus.\n");
	
	if(i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_500K)) {
		printf("Failed to initialize i2c bus!\n");
		return;
	}
	
	printf("Configuring ADCs.\n");
	
	adc_config();
	
	gpio_set_interrupt(READY_PIN, GPIO_INTTYPE_EDGE_POS, ads_ready_handle);
	
	printf("Starting send task.\n");
	
	xTaskCreate(send_task, "send_task", 512, NULL, 2, NULL);
	xTaskCreate(blink_task, "blink_task", 128, NULL, 1, NULL);
}
