#define CONFIG_NUMBER 23
#define CONFIG_STR_SIZE 65

extern char config_device_id[CONFIG_STR_SIZE];
extern char config_wifi_ap_password[CONFIG_STR_SIZE];
extern char config_wifi_ssid[CONFIG_STR_SIZE];
extern char config_wifi_password[CONFIG_STR_SIZE];
extern char config_mac_password[CONFIG_STR_SIZE];
extern char config_server_ip[CONFIG_STR_SIZE];

extern int config_use_flash_storage;
extern int config_channel_mode;
extern int config_channel_switch_cycles;
extern int config_p3_voltage_channel;

extern float config_current_factors[3];
extern float config_voltage_factors[2];
extern float config_line_frequency;
extern float config_line_frequency_tolerance;
extern float config_phase_voltage;
extern float config_combined_voltage;
extern float config_voltage_tolerance;
extern float config_max_phase_current;
extern float config_max_combined_current;

int configuration_read(const char *configuration_name, char *value_buffer);
int configuration_write(const char *configuration_name, const char *value_buffer, int external);
void load_configuration();
