################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fpga_mon.c \
../Core/Src/main.c \
../Core/Src/pdu_adc.c \
../Core/Src/pdu_app.c \
../Core/Src/pdu_mcan_app.c \
../Core/Src/pdu_selftest_cli_v3.c \
../Core/Src/ssd_energy.c \
C:/Users/charl/Documents/STM32Cube/Shared/Src/st_mcan.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/fpga_mon.o \
./Core/Src/main.o \
./Core/Src/pdu_adc.o \
./Core/Src/pdu_app.o \
./Core/Src/pdu_mcan_app.o \
./Core/Src/pdu_selftest_cli_v3.o \
./Core/Src/ssd_energy.o \
./Core/Src/st_mcan.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/fpga_mon.d \
./Core/Src/main.d \
./Core/Src/pdu_adc.d \
./Core/Src/pdu_app.d \
./Core/Src/pdu_mcan_app.d \
./Core/Src/pdu_selftest_cli_v3.d \
./Core/Src/ssd_energy.d \
./Core/Src/st_mcan.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DPDU_BUILD_MODE_RUNTIME -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../../Shared/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/st_mcan.o: C:/Users/charl/Documents/STM32Cube/Shared/Src/st_mcan.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DPDU_BUILD_MODE_RUNTIME -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../../Shared/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/fpga_mon.cyclo ./Core/Src/fpga_mon.d ./Core/Src/fpga_mon.o ./Core/Src/fpga_mon.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pdu_adc.cyclo ./Core/Src/pdu_adc.d ./Core/Src/pdu_adc.o ./Core/Src/pdu_adc.su ./Core/Src/pdu_app.cyclo ./Core/Src/pdu_app.d ./Core/Src/pdu_app.o ./Core/Src/pdu_app.su ./Core/Src/pdu_mcan_app.cyclo ./Core/Src/pdu_mcan_app.d ./Core/Src/pdu_mcan_app.o ./Core/Src/pdu_mcan_app.su ./Core/Src/pdu_selftest_cli_v3.cyclo ./Core/Src/pdu_selftest_cli_v3.d ./Core/Src/pdu_selftest_cli_v3.o ./Core/Src/pdu_selftest_cli_v3.su ./Core/Src/ssd_energy.cyclo ./Core/Src/ssd_energy.d ./Core/Src/ssd_energy.o ./Core/Src/ssd_energy.su ./Core/Src/st_mcan.cyclo ./Core/Src/st_mcan.d ./Core/Src/st_mcan.o ./Core/Src/st_mcan.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

