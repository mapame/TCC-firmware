extern uint32_t flash_power_data_count;
extern uint32_t flash_power_events_count;
extern uint32_t flash_internal_events_count;

void erase_flash();

void flash_search_power_data();
void flash_search_power_events();
void flash_search_internal_events();

int flash_add_power_data(power_data_flash_t *data);
int flash_get_power_data(power_data_flash_t *data, unsigned int index);
int flash_delete_power_data(unsigned int qty);

int flash_add_power_event(power_event_t *data);
int flash_get_power_event(power_event_t *data, unsigned int index);
int flash_delete_power_events(unsigned int qty);

int flash_add_internal_event(internal_event_t *data);
int flash_get_internal_event(internal_event_t *data, unsigned int index);
int flash_delete_internal_events(unsigned int qty);
