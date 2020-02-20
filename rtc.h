extern uint32_t rtc_time, rtc_time_sysclock_reference;
extern uint8_t rtc_oscillator_stopped;

int init_rtc();
int read_rtc_time();
uint32_t get_time();
int read_rtc_temp();
float get_temp();
int update_rtc(uint32_t new_time);
