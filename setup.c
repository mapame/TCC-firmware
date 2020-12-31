#include <espressif/esp_common.h>
#include <esp8266.h>
#include <espressif/spi_flash.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include <lwip/sockets.h>
#include <lwip/sys.h>

#include <sysparam.h>

#include "common.h"
#include "configuration.h"
#include "power.h"

static int recv_telnet_line(int socket_fd, char *buf, size_t len) {
	int num = 0;
	
	do {
		char c;
		
		if(recv(socket_fd, &c, 1, 0) <= 0)
			return -1;
		
		if(c == '\n')
			break;
		
		if(c < 0x20 || c > 0x7e)
			continue;
		
		if(num < len)
			buf[num] = c;
		
		num++;
	} while(1);
	
	buf[(num >= len) ? len - 1 : num] = 0; // Null terminate
	
	return num;
}

static int telnet_send(int socket_fd, const char *buf) {
	return send(socket_fd, buf, strlen(buf), 0);
}

static int telnet_send_line(int socket_fd, const char *buf) {
	char send_buf[205];
	
	sprintf(send_buf, "%s\r\n", buf);
	
	return send(socket_fd, send_buf, strlen(send_buf), 0);
}

void setup_task(void *pvParameters) {
	int main_socket, client_socket;
	struct sockaddr_in server_bind_address;
	const struct timeval timeout = {5, 0};
	
	int len;
	char aux[50];
	char *aux_ptr;
	
	char receive_buffer[200];
	char send_buffer[200];
	
	main_socket = socket(PF_INET, SOCK_STREAM, 0);
	
	bzero(&server_bind_address, sizeof(server_bind_address));
	server_bind_address.sin_family      = AF_INET;
	server_bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
	server_bind_address.sin_port        = htons(23);
	
	bind(main_socket, (struct sockaddr *) &server_bind_address, sizeof(server_bind_address));
	
	listen(main_socket, 2);
	
	while(1) {
		client_socket = accept(main_socket, (struct sockaddr *)NULL, (socklen_t *)NULL);
		
		if(client_socket < 0)
			continue;
		
		setsockopt(client_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
		
		
		telnet_send_line(client_socket, "Mattawat Configuration Console.");
		sprintf(send_buffer, "Firmare version: %s", FW_VERSION);
		telnet_send_line(client_socket, send_buffer);
		telnet_send_line(client_socket, "Type help for list of commands.");
		
		while((len = recv_telnet_line(client_socket, receive_buffer, 200)) >= 0) {
			if(!len) {
				telnet_send(client_socket, "> ");
				continue;
			}
			
			if(receive_buffer[len - 1] == '?') {
				receive_buffer[len - 1] = '\0';
				
				if(configuration_read(receive_buffer, aux, 0))
					sprintf(send_buffer, "Error reading configuration '%s'.", receive_buffer);
				else
					sprintf(send_buffer, "%s = %s", receive_buffer, aux);
				
				telnet_send_line(client_socket, send_buffer);
				
			} else if((aux_ptr = strchr(receive_buffer, '='))) {
				*aux_ptr = '\0';
				aux_ptr += 1;
				
				if(strlen(aux_ptr) > (CONFIG_STR_SIZE - 1))
					sprintf(send_buffer, "Value '%s' too long.", aux_ptr);
				else if(configuration_write(receive_buffer, aux_ptr, 0))
					sprintf(send_buffer, "Error setting '%s' configuration.", receive_buffer);
				else
					sprintf(send_buffer, "Set '%s' configuration to value '%s'.", receive_buffer, aux_ptr);
				
				telnet_send_line(client_socket, send_buffer);
				
			} else if(!strcmp(receive_buffer, "list")){
				for(int i = 0; i < CONFIG_NUMBER; i++) {
					configuration_index_name(i, aux);
					
					sprintf(send_buffer, "%s = ", aux);
					
					configuration_index_value(i, aux, 0);
					
					strcat(send_buffer, aux);
					
					telnet_send_line(client_socket, send_buffer);
				}
				
			} else if(!strcmp(receive_buffer, "compact")){
				telnet_send(client_socket, "Compacting sysparam area... ");
				
				sysparam_compact();
				
				telnet_send_line(client_socket, "Done.");
				
			} else if(!strcmp(receive_buffer, "restore")){
				uint32_t sysparam_base_addr, sysparam_num_sectors;
				
				telnet_send(client_socket, "Reformating sysparam area... ");
				
				if (sysparam_get_info(&sysparam_base_addr, &sysparam_num_sectors) != SYSPARAM_OK) {
					sysparam_num_sectors = DEFAULT_SYSPARAM_SECTORS;
					sysparam_base_addr = sdk_flashchip.chip_size - (5 + sysparam_num_sectors) * sdk_flashchip.sector_size;
				}
				
				if (sysparam_create_area(sysparam_base_addr, sysparam_num_sectors, true) == SYSPARAM_OK)
					sysparam_init(sysparam_base_addr, 0);
				
				telnet_send_line(client_socket, "Done.");
				
			} else if(!strcmp(receive_buffer, "help")){
				telnet_send(client_socket,
					"Available commands:\n"
					" <name>?         -> Read the <name> configuration value\r\n"
					" <name>=<value>  -> Set <name> configuration to <value>\r\n"
					" list            -> List configurations and its values\r\n"
					" compact         -> Compact the sysparam area\r\n"
					" restore         -> Restore configurations to its default values\r\n"
					" restart         -> Restart device\r\n"
					" help            -> Show this message\r\n"
				);
				
			} else if(!strcmp(receive_buffer, "restart")){
				vTaskDelay(pdMS_TO_TICKS(200));
				close(client_socket);
				vTaskDelay(pdMS_TO_TICKS(1000));
				sdk_system_restart();
				
			} else {
				telnet_send_line(client_socket, "Unrecognized command.");
			}
			
			telnet_send(client_socket, "> ");
		}
		
		close(client_socket);
	}
}
