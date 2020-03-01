#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sysparam.h>

#include "common.h"
#include "configuration.h"

char config_device_id[CONFIG_STR_SIZE];
char config_wifi_ap_password[CONFIG_STR_SIZE];
char config_wifi_ssid[CONFIG_STR_SIZE];
char config_wifi_password[CONFIG_STR_SIZE];
char config_mac_password[CONFIG_STR_SIZE];
char config_server_ip[CONFIG_STR_SIZE];

int config_use_flash_storage;
int config_power_phases;

float config_current_factors[2];
float config_voltage_factors[2];
float config_ac_frequency_max;
float config_ac_frequency_min;
float config_ac_voltage_max;
float config_ac_voltage_min;
float config_ac_peak_max;
float config_max_current[2];

typedef struct config_metadata_s {
	char name[33];
	char type;
	char default_value[CONFIG_STR_SIZE];
	uint8_t ext_r;
	uint8_t ext_w;
	uint8_t apply_now;
	void *variable;
} config_metadata_t;

const config_metadata_t configuration_table[CONFIG_NUMBER] = {
	{"device_id",					's', "NO_ID",			1, 0, 1, (void*) &config_device_id},
	{"wifi_ap_password",			's', "matawattap",		0, 0, 0, (void*) &config_wifi_ap_password},
	{"wifi_ssid",					's', "###NOT_SET###",	1, 1, 1, (void*) &config_wifi_ssid},
	{"wifi_password",				's', "###NOT_SET###",	0, 1, 0, (void*) &config_wifi_password},
	{"mac_password",				's', "###NOT_SET###",	0, 0, 0, (void*) &config_mac_password},
	{"server_ip",					's', "###NOT_SET###",	1, 1, 1, (void*) &config_server_ip},
	{"use_flash_storage",			'i', "0",				1, 1, 0, (void*) &config_use_flash_storage},
	{"power_phases",				'i', "2",				1, 1, 0, (void*) &config_power_phases},
	{"current_factor1",				'f', "86.21",			1, 1, 1, (void*) &config_current_factors[0]},
	{"current_factor2",				'f', "86.21",			1, 1, 1, (void*) &config_current_factors[1]},
	{"voltage_factor1",				'f', "3666.67",			1, 1, 1, (void*) &config_voltage_factors[0]},
	{"voltage_factor2",				'f', "3666.67",			1, 1, 1, (void*) &config_voltage_factors[1]},
	{"ac_frequency_max",			'f', "66.0",			1, 1, 1, (void*) &config_ac_frequency_max},
	{"ac_frequency_min",			'f', "54.0",			1, 1, 1, (void*) &config_ac_frequency_min},
	{"ac_voltage_max",				'f', "139.7",			1, 1, 1, (void*) &config_ac_voltage_max},
	{"ac_voltage_min",				'f', "114.3",			1, 1, 1, (void*) &config_ac_voltage_min},
	{"ac_voltage_peak_max",			'f', "200.0",			1, 1, 1, (void*) &config_ac_peak_max},
	{"maximum_current1",			'f', "40.0",			1, 1, 1, (void*) &config_max_current[0]},
	{"maximum_current2",			'f', "40.0",			1, 1, 1, (void*) &config_max_current[1]},
};


int configuration_read(const char *configuration_name, char *value_buffer) {
	if(!(configuration_name && value_buffer))
		return -1;
	
	for(int i = 0; i < CONFIG_NUMBER; i++)
		if(!strcmp(configuration_name, configuration_table[i].name)) {
			if(!configuration_table[i].ext_r)
				return -3;
			
			switch(configuration_table[i].type) {
				case 's':
					strlcpy(value_buffer, (char*) configuration_table[i].variable, CONFIG_STR_SIZE);
					break;
				case 'i':
					sprintf(value_buffer, "%d", *((int*) configuration_table[i].variable));
					break;
				case 'f':
					sprintf(value_buffer, "%f", *((float*) configuration_table[i].variable));
					break;
			}
			return 0;
		}
	
	return -2;
}

int configuration_write(const char *configuration_name, const char *value_buffer, int external) {
	char tmp_s[CONFIG_STR_SIZE];
	int tmp_i;
	float tmp_f;
	
	if(!(configuration_name && value_buffer))
		return -1;
	
	for(int i = 0; i < CONFIG_NUMBER; i++)
		if(!strcmp(configuration_name, configuration_table[i].name)) {
			if(external && (!configuration_table[i].ext_w))
				return -3;
			
			int conversion_result = 0;
			
			switch(configuration_table[i].type) {
				case 's':
					conversion_result = strlcpy((char*) tmp_s, value_buffer, CONFIG_STR_SIZE);
					
					if(conversion_result && configuration_table[i].apply_now)
						strlcpy((char*) configuration_table[i].variable, tmp_s, CONFIG_STR_SIZE);
					break;
				case 'i':
					conversion_result = sscanf(value_buffer, "%d", &tmp_i);
					
					if(conversion_result && configuration_table[i].apply_now)
						*((int*) configuration_table[i].variable) = tmp_f;
					
					break;
				case 'f':
					conversion_result = sscanf(value_buffer, "%f", &tmp_f);
					
					if(conversion_result && configuration_table[i].apply_now)
						*((float*) configuration_table[i].variable) = tmp_f;
					
					break;
				default:
					break;
			}
			
			if(conversion_result < 1)
				return -4;
			
			if(sysparam_set_string(configuration_name, value_buffer) != SYSPARAM_OK)
				return -5;
			
			return 0;
		}
	
	return -2;
}

void load_configuration() {
	char *value;
	
	for(int i = 0; i < CONFIG_NUMBER; i++) {
		value = NULL;
		sysparam_get_string(configuration_table[i].name, &value);
		
		switch(configuration_table[i].type) {
			case 's':
				if(!value || strlcpy((char*) configuration_table[i].variable, value, CONFIG_STR_SIZE) < 1)
					strlcpy((char*) configuration_table[i].variable, configuration_table[i].default_value, CONFIG_STR_SIZE);
				break;
			case 'i':
				if(!value || sscanf(value, "%d", (int*) configuration_table[i].variable) != 1)
					sscanf(configuration_table[i].default_value, "%d", (int*) configuration_table[i].variable);
				break;
			case 'f':
				if(!value || sscanf(value, "%f", (float*) configuration_table[i].variable) != 1)
					sscanf(configuration_table[i].default_value, "%f", (float*) configuration_table[i].variable);
				break;
		}
		
		if(value)
			free(value);
	}
	
}
