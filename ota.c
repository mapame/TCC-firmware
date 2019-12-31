#include <stdio.h>
#include <string.h>

#include "ota-tftp.h"
#include "rboot-api.h"
#include "bearssl.h"

#include "common.h"
#include "ota.h"

extern char config_server_ip[CONFIG_STR_SIZE];

void ota_task(void *pvParameters) {
	rboot_config conf;
	uint8_t dest_slot;
	uint32_t image_length;
	br_md5_context hash_context;
	uint8_t hash_result[16];
	char hash_result_text[33];
	char *correct_file_hash_text = (char*) pvParameters;
	
	char aux[3];
	
	debug("Starting OTA update...\n");
	
	conf = rboot_get_config();
	
	if(conf.count < 2) {
		debug("Only one slot is avaliable, cannot proceed. Restarting...\n");
		sdk_system_restart();
	}
	
	dest_slot = (conf.current_rom + 1) % conf.count;
	
	debug("Image will be flashed into slot %d.\n", dest_slot);
	
	if(ota_tftp_download(config_server_ip, 6969, "firmware.bin", 2000, dest_slot, NULL)) {
		debug("Failed to download image. Restarting...\n");
		sdk_system_restart();
	}
	
	if(!rboot_verify_image(conf.roms[dest_slot], &image_length, NULL)) {
		debug("Downloaded image is not valid. Restarting...\n");
		sdk_system_restart();
	}
	
	debug("Downloaded and flashed a valid image, checking integrity...\n");
	
	br_md5_init(&hash_context);
	
	rboot_digest_image(conf.roms[dest_slot], image_length, (rboot_digest_update_fn)br_md5_update, &hash_context);
	
	br_md5_out(&hash_context, hash_result);
	
	hash_result_text[0] = '\0';
	
	for(int i = 0; i < 16; i++) {
		sprintf(aux, "%02hx", hash_result[i]);
		strlcat(hash_result_text, aux, 33);
	}
	
	if(strcmp(hash_result_text, correct_file_hash_text)) {
		debug("Hash does not match. Restarting...\n");
		sdk_system_restart();
	}
	
	debug("Image hash match, restarting into new slot...\n");
	
	conf.current_rom = dest_slot;
	
	if(!rboot_set_config(&conf))
		debug("Failed to set new slot as active.\n");
	
	sdk_system_restart();
}
