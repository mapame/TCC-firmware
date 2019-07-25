PROGRAM = esp_firmware
EXTRA_COMPONENTS = extras/i2c
#ESPBAUD = 460800
LIBS ?= gcc hal m
include ../esp-open-rtos/common.mk
