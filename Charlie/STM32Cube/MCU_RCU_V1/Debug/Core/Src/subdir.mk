################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/eth_stack.c \
../Core/Src/eth_udp.c \
../Core/Src/imu.c \
../Core/Src/main.c \
../Core/Src/mcan_pdu.c \
../Core/Src/motor_bus.c \
../Core/Src/rcu_app.c \
../Core/Src/rcu_selftest_cli_v1.c \
../Core/Src/rs04.c \
C:/Users/charl/Documents/STM32Cube/Shared/Src/st_mcan.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/telem_pack.c 

OBJS += \
./Core/Src/eth_stack.o \
./Core/Src/eth_udp.o \
./Core/Src/imu.o \
./Core/Src/main.o \
./Core/Src/mcan_pdu.o \
./Core/Src/motor_bus.o \
./Core/Src/rcu_app.o \
./Core/Src/rcu_selftest_cli_v1.o \
./Core/Src/rs04.o \
./Core/Src/st_mcan.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/telem_pack.o 

C_DEPS += \
./Core/Src/eth_stack.d \
./Core/Src/eth_udp.d \
./Core/Src/imu.d \
./Core/Src/main.d \
./Core/Src/mcan_pdu.d \
./Core/Src/motor_bus.d \
./Core/Src/rcu_app.d \
./Core/Src/rcu_selftest_cli_v1.d \
./Core/Src/rs04.d \
./Core/Src/st_mcan.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/telem_pack.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DRCU_BUILD_MODE_RUNTIME -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../../Shared/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/st_mcan.o: C:/Users/charl/Documents/STM32Cube/Shared/Src/st_mcan.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DRCU_BUILD_MODE_RUNTIME -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../../Shared/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/eth_stack.cyclo ./Core/Src/eth_stack.d ./Core/Src/eth_stack.o ./Core/Src/eth_stack.su ./Core/Src/eth_udp.cyclo ./Core/Src/eth_udp.d ./Core/Src/eth_udp.o ./Core/Src/eth_udp.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mcan_pdu.cyclo ./Core/Src/mcan_pdu.d ./Core/Src/mcan_pdu.o ./Core/Src/mcan_pdu.su ./Core/Src/motor_bus.cyclo ./Core/Src/motor_bus.d ./Core/Src/motor_bus.o ./Core/Src/motor_bus.su ./Core/Src/rcu_app.cyclo ./Core/Src/rcu_app.d ./Core/Src/rcu_app.o ./Core/Src/rcu_app.su ./Core/Src/rcu_selftest_cli_v1.cyclo ./Core/Src/rcu_selftest_cli_v1.d ./Core/Src/rcu_selftest_cli_v1.o ./Core/Src/rcu_selftest_cli_v1.su ./Core/Src/rs04.cyclo ./Core/Src/rs04.d ./Core/Src/rs04.o ./Core/Src/rs04.su ./Core/Src/st_mcan.cyclo ./Core/Src/st_mcan.d ./Core/Src/st_mcan.o ./Core/Src/st_mcan.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/telem_pack.cyclo ./Core/Src/telem_pack.d ./Core/Src/telem_pack.o ./Core/Src/telem_pack.su

.PHONY: clean-Core-2f-Src

