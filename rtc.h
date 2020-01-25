extern uint32_t rtc_time, rtc_time_sysclock_reference;
extern uint8_t rtc_oscillator_stopped;

extern float rtc_temp;

int read_rtc_time();
int read_rtc_temp();
void update_rtc(uint32_t new_time);
