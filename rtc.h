extern uint32_t rtc_time, rtc_time_sysclock_reference;
extern uint8_t rtc_oscillator_stopped;

int read_rtc();
void update_rtc(const time_t new_time);
