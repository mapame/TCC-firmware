#define CONFIG_NUMBER 19
#define CONFIG_STR_SIZE 65

extern char config_device_id[CONFIG_STR_SIZE];
extern char config_wifi_ap_password[CONFIG_STR_SIZE];
extern char config_wifi_ssid[CONFIG_STR_SIZE];
extern char config_wifi_password[CONFIG_STR_SIZE];
extern char config_mac_password[CONFIG_STR_SIZE];
extern char config_server_ip[CONFIG_STR_SIZE];

extern int config_use_flash_storage;
extern int config_power_phases;

extern float config_current_factors[2];
extern float config_voltage_factors[2];
extern float config_ac_frequency_max;
extern float config_ac_frequency_min;
extern float config_ac_voltage_max;
extern float config_ac_voltage_min;
extern float config_ac_peak_max;
extern float config_max_current[2];

int configuration_read(const char *configuration_name, char *value_buffer);
int configuration_write(const char *configuration_name, const char *value_buffer, int external);
void load_configuration();
