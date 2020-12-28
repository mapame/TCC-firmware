PROGRAM = matawatt-firmware

FLASH_SIZE = 32

EXTRA_COMPONENTS = extras/rboot-ota extras/bearssl extras/dhcpserver extras/ds3231 extras/i2c
#ESPBAUD = 460800
LIBS ?= gcc hal m
include $(ESP_OPEN_RTOS_PATH)/common.mk
