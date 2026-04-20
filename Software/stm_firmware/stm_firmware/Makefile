# Makefile for STM32F429ZI firmware

# Toolchain
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

# Project directories
THIS_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
SOFTWARE_DIR := $(abspath $(THIS_DIR)/../..)
STM32CUBE_DIR ?= $(SOFTWARE_DIR)/STM32CubeF4
HAL_DIR = $(STM32CUBE_DIR)/Drivers/STM32F4xx_HAL_Driver
CMSIS_DIR = $(STM32CUBE_DIR)/Drivers/CMSIS
DEVICE_DIR = $(CMSIS_DIR)/Device/ST/STM32F4xx

ifeq ($(wildcard $(HAL_DIR)/Src/stm32f4xx_hal.c),)
$(error STM32CubeF4 not found at $(STM32CUBE_DIR). Set STM32CUBE_DIR=/path/to/STM32CubeF4)
endif

# Includes
INCLUDES = -I.
INCLUDES += -I$(HAL_DIR)/Inc
INCLUDES += -I$(DEVICE_DIR)/Include
INCLUDES += -I$(CMSIS_DIR)/Include

# Sources
C_SOURCES = main.c
C_SOURCES += system_stm32f4xx.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_can.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_uart.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_gpio.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_rcc.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_cortex.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_dma.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_flash.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_flash_ex.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_pwr.c
C_SOURCES += $(HAL_DIR)/Src/stm32f4xx_hal_pwr_ex.c

ASM_SOURCES = startup_stm32f429xx.s

# MCU
MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# CFLAGS
CFLAGS = $(MCU) $(INCLUDES) -DSTM32F429xx -DUSE_HAL_DRIVER -O2 -Wall -Wextra -g -gdwarf-2

# LDFLAGS
LDFLAGS = $(MCU) -specs=nosys.specs -TSTM32F429ZITX_FLASH.ld -Wl,--gc-sections

# Objects
OBJ_DIR = .
OBJECTS = $(OBJ_DIR)/main.o $(OBJ_DIR)/system_stm32f4xx.o $(OBJ_DIR)/startup_stm32f429xx.o
OBJECTS += $(OBJ_DIR)/stm32f4xx_hal.o $(OBJ_DIR)/stm32f4xx_hal_can.o $(OBJ_DIR)/stm32f4xx_hal_uart.o
OBJECTS += $(OBJ_DIR)/stm32f4xx_hal_gpio.o $(OBJ_DIR)/stm32f4xx_hal_rcc.o $(OBJ_DIR)/stm32f4xx_hal_cortex.o
OBJECTS += $(OBJ_DIR)/stm32f4xx_hal_dma.o $(OBJ_DIR)/stm32f4xx_hal_flash.o $(OBJ_DIR)/stm32f4xx_hal_flash_ex.o
OBJECTS += $(OBJ_DIR)/stm32f4xx_hal_pwr.o $(OBJ_DIR)/stm32f4xx_hal_pwr_ex.o

# Targets
all: firmware.bin

firmware.elf: $(OBJECTS)
	$(LD) $(LDFLAGS) -o $@ $^

firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $< $@

$(OBJ_DIR)/main.o: main.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/system_stm32f4xx.o: system_stm32f4xx.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/startup_stm32f429xx.o: startup_stm32f429xx.s
	$(AS) $(MCU) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal.o: $(HAL_DIR)/Src/stm32f4xx_hal.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_can.o: $(HAL_DIR)/Src/stm32f4xx_hal_can.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_uart.o: $(HAL_DIR)/Src/stm32f4xx_hal_uart.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_gpio.o: $(HAL_DIR)/Src/stm32f4xx_hal_gpio.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_rcc.o: $(HAL_DIR)/Src/stm32f4xx_hal_rcc.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_cortex.o: $(HAL_DIR)/Src/stm32f4xx_hal_cortex.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_dma.o: $(HAL_DIR)/Src/stm32f4xx_hal_dma.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_flash.o: $(HAL_DIR)/Src/stm32f4xx_hal_flash.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_flash_ex.o: $(HAL_DIR)/Src/stm32f4xx_hal_flash_ex.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_pwr.o: $(HAL_DIR)/Src/stm32f4xx_hal_pwr.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/stm32f4xx_hal_pwr_ex.o: $(HAL_DIR)/Src/stm32f4xx_hal_pwr_ex.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) firmware.elf firmware.bin

.PHONY: all clean