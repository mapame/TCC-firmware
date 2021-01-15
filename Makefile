PROGRAM = matawatt-firmware

FLASH_SIZE = 32

EXTRA_CFLAGS+=-DCOMM_SKIP_CHECK_TIMESTAMP=1
EXTRA_CFLAGS=-I./fsdata

EXTRA_COMPONENTS = extras/rboot-ota extras/mbedtls extras/httpd extras/bearssl extras/dhcpserver extras/ds3231 extras/i2c
#ESPBAUD = 460800
LIBS ?= gcc hal m
include $(ESP_OPEN_RTOS_PATH)/common.mk

mkfsdata:
	@echo "Generating fsdata.."
	cd fsdata && ./makefsdata
