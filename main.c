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
#define I2C_BUS 0
#define SCL_PIN 5
#define SDA_PIN 4

#define GAIN_ADC1 ADS111X_GAIN_4V096
#define GAIN_ADC2 ADS111X_GAIN_4V096
#define GAIN_ADC3 ADS111X_GAIN_4V096

ads111x_dev_t adc1, adc2, adc3;

int16_t raw_adc1, raw_adc2, raw_adc3;

void ads_ready_handle(uint8_t gpio_num) {
	ads111x_start_conversion(&adc1);
	ads111x_start_conversion(&adc2);
	ads111x_start_conversion(&adc3);
	
	raw_adc1 = ads111x_get_value(&adc1);
	raw_adc2 = ads111x_get_value(&adc2);
	raw_adc2 = ads111x_get_value(&adc3);
}

void user_init(void) {
	uart_set_baud(0, 115200);
	
	printf("Initializing I2C bus.\n");
	
	if(i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_600K)) {
		printf("Failed to initialize i2c bus!\n");
		return;
	}
	
	printf("Configuring ADCs.\n");
	
	ads111x_init(&adc1, I2C_BUS, ADS111X_ADDR_GND);
	ads111x_init(&adc2, I2C_BUS, ADS111X_ADDR_VCC);
	ads111x_init(&adc3, I2C_BUS, ADS111X_ADDR_SCL);
	
	ads111x_set_data_rate(&adc1, ADS111X_DATA_RATE_860);
	ads111x_set_data_rate(&adc2, ADS111X_DATA_RATE_860);
	ads111x_set_data_rate(&adc3, ADS111X_DATA_RATE_860);
	
	//ads111x_set_input_mux(&adc1, ADS111X_MUX_0_GND);
	//ads111x_set_input_mux(&adc2, ADS111X_MUX_0_GND);
	//ads111x_set_input_mux(&adc3, ADS111X_MUX_0_GND);
	
	//ads111x_set_gain(&adc1, GAIN_ADC1);
	//ads111x_set_gain(&adc2, GAIN_ADC2);
	//ads111x_set_gain(&adc3, GAIN_ADC3);
	
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
	
	//ads111x_set_mode(&adc1, ADS111X_MODE_CONTINUOUS);
	//ads111x_set_mode(&adc2, ADS111X_MODE_CONTINUOUS);
	//ads111x_set_mode(&adc3, ADS111X_MODE_CONTINUOUS);
	
	ads111x_push_config(&adc1);
	ads111x_push_config(&adc2);
	ads111x_push_config(&adc3);
	
	//vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	gpio_enable(14, GPIO_INPUT);
	gpio_set_interrupt(14, GPIO_INTTYPE_EDGE_POS, ads_ready_handle);
	
	gpio_enable(12, GPIO_OUTPUT);
	
	printf("Starting loop!\n");
	
	float v_adc1, v_adc2, v_adc3;
	int count = 0;
	
	ads111x_start_conversion(&adc1);
	ads111x_start_conversion(&adc2);
	ads111x_start_conversion(&adc3);
	
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(50));
		
		gpio_write(12, 1);
		
		count = (count + 1) % 5;
		if(count == 0) {
			v_adc1 = ads111x_gain_values[GAIN_ADC1] / ADS111X_MAX_VALUE * raw_adc1;
			v_adc2 = ads111x_gain_values[GAIN_ADC2] / ADS111X_MAX_VALUE * raw_adc2;
			v_adc3 = ads111x_gain_values[GAIN_ADC3] / ADS111X_MAX_VALUE * raw_adc3;
			
			printf("%.4f %.4f %.4f\n", v_adc1, v_adc2, v_adc3);
		}
		
		gpio_write(12, 0);
	}
}
