#include <espressif/esp_common.h>
#include <esp8266.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <spiflash.h>

#include "common.h"
#include "power.h"
#include "ievents.h"

#define FLASH_AREA_SIZE 0x00080000

#define FLASH_POWER_DATA_START_ADDR 0x00102000
#define FLASH_POWER_DATA_END_ADDR FLASH_POWER_DATA_START_ADDR + (FLASH_AREA_SIZE - (FLASH_AREA_SIZE % sizeof(power_data_flash_t)))

#define FLASH_POWER_EVENTS_START_ADDR 0x00282000
#define FLASH_POWER_EVENTS_END_ADDR FLASH_POWER_EVENTS_START_ADDR + (FLASH_AREA_SIZE - (FLASH_AREA_SIZE % sizeof(power_event_t)))

#define FLASH_INTERNAL_EVENTS_START_ADDR 0x00302000
#define FLASH_INTERNAL_EVENTS_END_ADDR FLASH_INTERNAL_EVENTS_START_ADDR + (FLASH_AREA_SIZE - (FLASH_AREA_SIZE % sizeof(internal_event_t)))

uint32_t flash_power_data_head_addr, flash_power_data_tail_addr, flash_power_data_count;
uint32_t flash_power_events_head_addr, flash_power_events_tail_addr, flash_power_events_count;
uint32_t flash_internal_events_head_addr, flash_internal_events_tail_addr, flash_internal_events_count;

void erase_flash() {
	for(uint32_t addr = FLASH_POWER_DATA_START_ADDR; addr < FLASH_POWER_DATA_END_ADDR; addr += 4096)
		spiflash_erase_sector(addr);
	
	for(uint32_t addr = FLASH_POWER_EVENTS_START_ADDR; addr < FLASH_POWER_EVENTS_END_ADDR; addr += 4096)
		spiflash_erase_sector(addr);
		
	for(uint32_t addr = FLASH_INTERNAL_EVENTS_START_ADDR; addr < FLASH_INTERNAL_EVENTS_END_ADDR; addr += 4096)
		spiflash_erase_sector(addr);
}

void flash_search_power_data() {
	power_data_flash_t aux_power_data;
	
	uint32_t head_timestamp, tail_timestamp;
	
	flash_power_data_count = 0;
	
	flash_power_data_head_addr = FLASH_POWER_DATA_START_ADDR;
	flash_power_data_tail_addr = FLASH_POWER_DATA_START_ADDR;
	
	head_timestamp = 0;
	tail_timestamp = 0xFFFFFFFF;
	
	for(uint32_t addr = FLASH_POWER_DATA_START_ADDR; addr < FLASH_POWER_DATA_END_ADDR; addr += sizeof(power_data_flash_t)) {
		spiflash_read(addr, (uint8_t*) &aux_power_data, sizeof(power_data_flash_t));
		
		if(aux_power_data.timestamp == 0xFFFFFFFF)
			continue;
		
		if(aux_power_data.timestamp < 1580000000) {
			if(head_timestamp == 0 && tail_timestamp == 0xFFFFFFFF) {
				flash_power_data_head_addr = addr + sizeof(power_data_flash_t);
				
				if(flash_power_data_head_addr >= FLASH_POWER_DATA_END_ADDR)
					flash_power_data_head_addr = FLASH_POWER_DATA_START_ADDR;
				
				flash_power_data_tail_addr = flash_power_data_head_addr;
			}
			
			continue;
		}
		
		if(aux_power_data.timestamp > head_timestamp) {
			head_timestamp = aux_power_data.timestamp;
			flash_power_data_head_addr = addr;
		}
		
		if(aux_power_data.timestamp < tail_timestamp) {
			tail_timestamp = aux_power_data.timestamp;
			flash_power_data_tail_addr = addr;
		}
	}
	
	if(head_timestamp != 0) {
		flash_power_data_head_addr += sizeof(power_data_flash_t);
		if(flash_power_data_head_addr >= FLASH_POWER_DATA_END_ADDR)
			flash_power_data_head_addr = FLASH_POWER_DATA_START_ADDR;
	}
	
	for(uint32_t addr = flash_power_data_tail_addr; addr != flash_power_data_head_addr; addr += sizeof(power_data_flash_t)) {
		if(addr >= FLASH_POWER_DATA_END_ADDR)
			addr = FLASH_POWER_DATA_START_ADDR;
		
		spiflash_read(addr, (uint8_t*) &aux_power_data, sizeof(power_data_flash_t));
		
		if(aux_power_data.timestamp == 0xFFFFFFFF || aux_power_data.timestamp < 1580000000)
			continue;
		
		flash_power_data_count++;
	}
	
	if(flash_power_data_count == 0)
		spiflash_erase_sector(flash_power_data_head_addr & 0xFFFFF000);
}

void flash_search_power_events() {
	power_event_t aux_power_event;
	
	uint32_t head_timestamp, tail_timestamp;
	
	flash_power_events_count = 0;
	
	flash_power_events_head_addr = FLASH_POWER_EVENTS_START_ADDR;
	flash_power_events_tail_addr = FLASH_POWER_EVENTS_START_ADDR;
	
	head_timestamp = 0;
	tail_timestamp = 0xFFFFFFFF;
	
	for(uint32_t addr = FLASH_POWER_EVENTS_START_ADDR; addr < FLASH_POWER_EVENTS_END_ADDR; addr += sizeof(power_event_t)) {
		spiflash_read(addr, (uint8_t*) &aux_power_event, sizeof(power_event_t));
		
		if(aux_power_event.timestamp == 0xFFFFFFFF)
			continue;
		
		if(aux_power_event.timestamp < 1580000000) {
			if(head_timestamp == 0 && tail_timestamp == 0xFFFFFFFF) {
				flash_power_events_head_addr = addr + sizeof(power_event_t);
				
				if(flash_power_events_head_addr >= FLASH_POWER_EVENTS_END_ADDR)
					flash_power_events_head_addr = FLASH_POWER_EVENTS_START_ADDR;
				
				flash_power_events_tail_addr = flash_power_events_head_addr;
			}
			
			continue;
		}
		
		if(aux_power_event.timestamp > head_timestamp) {
			head_timestamp = aux_power_event.timestamp;
			flash_power_events_head_addr = addr;
		}
		
		if(aux_power_event.timestamp < tail_timestamp) {
			tail_timestamp = aux_power_event.timestamp;
			flash_power_events_tail_addr = addr;
		}
	}
	
	if(head_timestamp != 0) {
		flash_power_events_head_addr += sizeof(power_event_t);
		if(flash_power_events_head_addr >= FLASH_POWER_EVENTS_END_ADDR)
			flash_power_events_head_addr = FLASH_POWER_EVENTS_START_ADDR;
	}
	
	for(uint32_t addr = flash_power_events_tail_addr; addr != flash_power_events_head_addr; addr += sizeof(power_event_t)) {
		if(addr >= FLASH_POWER_EVENTS_END_ADDR)
			addr = FLASH_POWER_EVENTS_START_ADDR;
		
		spiflash_read(addr, (uint8_t*) &aux_power_event, sizeof(power_event_t));
		
		if(aux_power_event.timestamp == 0xFFFFFFFF || aux_power_event.timestamp < 1580000000)
			continue;
		
		flash_power_events_count++;
	}
	
	if(flash_power_events_count == 0)
		spiflash_erase_sector(flash_power_events_head_addr & 0xFFFFF000);
}

void flash_search_internal_events() {
	internal_event_t aux_internal_event;
	
	uint32_t head_timestamp, tail_timestamp;
	
	flash_internal_events_count = 0;
	
	flash_internal_events_head_addr = FLASH_INTERNAL_EVENTS_START_ADDR;
	flash_internal_events_tail_addr = FLASH_INTERNAL_EVENTS_START_ADDR;
	
	head_timestamp = 0;
	tail_timestamp = 0xFFFFFFFF;
	
	for(uint32_t addr = FLASH_INTERNAL_EVENTS_START_ADDR; addr < FLASH_INTERNAL_EVENTS_END_ADDR; addr += sizeof(internal_event_t)) {
		spiflash_read(addr, (uint8_t*) &aux_internal_event, sizeof(internal_event_t));
		
		if(aux_internal_event.timestamp == 0xFFFFFFFF)
			continue;
		
		if(aux_internal_event.timestamp < 1580000000) {
			if(head_timestamp == 0 && tail_timestamp == 0xFFFFFFFF) {
				flash_internal_events_head_addr = addr + sizeof(internal_event_t);
				
				if(flash_internal_events_head_addr >= FLASH_INTERNAL_EVENTS_END_ADDR)
					flash_internal_events_head_addr = FLASH_INTERNAL_EVENTS_START_ADDR;
				
				flash_internal_events_tail_addr = flash_internal_events_head_addr;
			}
			
			continue;
		}
		
		if(aux_internal_event.timestamp > head_timestamp) {
			head_timestamp = aux_internal_event.timestamp;
			flash_internal_events_head_addr = addr;
		}
		
		if(aux_internal_event.timestamp < tail_timestamp) {
			tail_timestamp = aux_internal_event.timestamp;
			flash_internal_events_tail_addr = addr;
		}
	}
	
	if(head_timestamp != 0) {
		flash_internal_events_head_addr += sizeof(internal_event_t);
		if(flash_internal_events_head_addr >= FLASH_INTERNAL_EVENTS_END_ADDR)
			flash_internal_events_head_addr = FLASH_INTERNAL_EVENTS_START_ADDR;
	}
	
	for(uint32_t addr = flash_internal_events_tail_addr; addr != flash_internal_events_head_addr; addr += sizeof(internal_event_t)) {
		if(addr >= FLASH_INTERNAL_EVENTS_END_ADDR)
			addr = FLASH_INTERNAL_EVENTS_START_ADDR;
		
		spiflash_read(addr, (uint8_t*) &aux_internal_event, sizeof(internal_event_t));
		
		if(aux_internal_event.timestamp == 0xFFFFFFFF || aux_internal_event.timestamp < 1580000000)
			continue;
		
		flash_internal_events_count++;
	}
	
	if(flash_internal_events_count == 0)
		spiflash_erase_sector(flash_internal_events_head_addr & 0xFFFFF000);
}

int flash_add_power_data(power_data_flash_t *data) {
	if((flash_power_data_head_addr & 0xFFFFF000) == (flash_power_data_tail_addr & 0xFFFFF000) && flash_power_data_head_addr < flash_power_data_tail_addr)
		return -1;
	
	spiflash_write(flash_power_data_head_addr, (uint8_t*) data, sizeof(power_data_flash_t));
	
	flash_power_data_head_addr += sizeof(power_data_flash_t);
	if(flash_power_data_head_addr >= FLASH_POWER_DATA_END_ADDR)
		flash_power_data_head_addr = FLASH_POWER_DATA_START_ADDR;
	
	flash_power_data_count++;
	
	if((flash_power_data_head_addr & 0xFFF) == 0)
		spiflash_erase_sector(flash_power_data_head_addr);
	
	return 0;
}

int flash_get_power_data(power_data_flash_t *data, unsigned int index) {
	uint32_t addr;
	
	if(index >= flash_power_data_count)
		return -1;
	
	addr = flash_power_data_tail_addr + index * sizeof(power_data_flash_t);
	if(addr >= FLASH_POWER_DATA_END_ADDR)
		addr = FLASH_POWER_DATA_START_ADDR + (addr - FLASH_POWER_DATA_END_ADDR);
	
	spiflash_read(addr, (uint8_t*) data, sizeof(power_data_flash_t));
	
	return 0;
}

int flash_delete_power_data(unsigned int qty) {
	uint32_t zero = 0;
	
	if(qty > flash_power_data_count)
		return -1;
	
	for(int i = 0; i < qty; i++) {
		spiflash_write(flash_power_data_tail_addr, (uint8_t*) &zero, sizeof(uint32_t));
		
		flash_power_data_tail_addr += sizeof(power_data_flash_t);
		if(flash_power_data_tail_addr >= FLASH_POWER_DATA_END_ADDR)
			flash_power_data_tail_addr = FLASH_POWER_DATA_START_ADDR;
		
		flash_power_data_count--;
	}
	
	return 0;
}

int flash_add_power_event(power_event_t *event) {
	if((flash_power_events_head_addr & 0xFFFFF000) == (flash_power_events_tail_addr & 0xFFFFF000) && flash_power_events_head_addr < flash_power_events_tail_addr)
		return -1;
	
	spiflash_write(flash_power_events_head_addr, (uint8_t*) event, sizeof(power_event_t));
	
	flash_power_events_head_addr += sizeof(power_event_t);
	if(flash_power_events_head_addr >= FLASH_POWER_EVENTS_END_ADDR)
		flash_power_events_head_addr = FLASH_POWER_EVENTS_START_ADDR;
	
	flash_power_events_count++;
	
	if((flash_power_events_head_addr & 0xFFF) == 0)
		spiflash_erase_sector(flash_power_events_head_addr);
	
	return 0;
}

int flash_get_power_event(power_event_t *event, unsigned int index) {
	uint32_t addr;
	
	if(index >= flash_power_events_count)
		return -1;
	
	addr = flash_power_events_tail_addr + index * sizeof(power_event_t);
	if(addr >= FLASH_POWER_EVENTS_END_ADDR)
		addr = FLASH_POWER_EVENTS_START_ADDR + (addr - FLASH_POWER_EVENTS_END_ADDR);
	
	spiflash_read(addr, (uint8_t*) event, sizeof(power_event_t));
	
	return 0;
}

int flash_delete_power_events(unsigned int qty) {
	uint32_t zero = 0;
	
	if(qty > flash_power_events_count)
		return -1;
	
	for(int i = 0; i < qty; i++) {
		spiflash_write(flash_power_events_tail_addr, (uint8_t*) &zero, sizeof(uint32_t));
		
		flash_power_events_tail_addr += sizeof(power_event_t);
		if(flash_power_events_tail_addr >= FLASH_POWER_EVENTS_END_ADDR)
			flash_power_events_tail_addr = FLASH_POWER_EVENTS_START_ADDR;
		
		flash_power_events_count--;
	}
	
	return 0;
}

int flash_add_internal_event(internal_event_t *event) {
	if((flash_internal_events_head_addr & 0xFFFFF000) == (flash_internal_events_tail_addr & 0xFFFFF000) && flash_internal_events_head_addr < flash_internal_events_tail_addr)
		return -1;
	
	spiflash_write(flash_internal_events_head_addr, (uint8_t*) event, sizeof(internal_event_t));
	
	flash_internal_events_head_addr += sizeof(internal_event_t);
	if(flash_internal_events_head_addr >= FLASH_INTERNAL_EVENTS_END_ADDR)
		flash_internal_events_head_addr = FLASH_INTERNAL_EVENTS_START_ADDR;
	
	flash_internal_events_count++;
	
	if((flash_internal_events_head_addr & 0xFFF) == 0)
		spiflash_erase_sector(flash_internal_events_head_addr);
	
	return 0;
}

int flash_get_internal_event(internal_event_t *event, unsigned int index) {
	uint32_t addr;
	
	if(index >= flash_internal_events_count)
		return -1;
	
	addr = flash_internal_events_tail_addr + index * sizeof(internal_event_t);
	if(addr >= FLASH_INTERNAL_EVENTS_END_ADDR)
		addr = FLASH_INTERNAL_EVENTS_START_ADDR + (addr - FLASH_INTERNAL_EVENTS_END_ADDR);
	
	spiflash_read(addr, (uint8_t*) event, sizeof(internal_event_t));
	
	return 0;
}

int flash_delete_internal_events(unsigned int qty) {
	uint32_t zero = 0;
	
	if(qty > flash_internal_events_count)
		return -1;
	
	for(int i = 0; i < qty; i++) {
		spiflash_write(flash_internal_events_tail_addr, (uint8_t*) &zero, sizeof(uint32_t));
		
		flash_internal_events_tail_addr += sizeof(internal_event_t);
		if(flash_internal_events_tail_addr >= FLASH_INTERNAL_EVENTS_END_ADDR)
			flash_internal_events_tail_addr = FLASH_INTERNAL_EVENTS_START_ADDR;
		
		flash_internal_events_count--;
	}
	
	return 0;
}
