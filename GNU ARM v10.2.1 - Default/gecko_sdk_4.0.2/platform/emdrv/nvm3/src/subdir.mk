################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/Darek/SimplicityStudio/SDKs/gecko_sdk/platform/emdrv/nvm3/src/nvm3_default_common_linker.c \
/Users/Darek/SimplicityStudio/SDKs/gecko_sdk/platform/emdrv/nvm3/src/nvm3_hal_flash.c \
/Users/Darek/SimplicityStudio/SDKs/gecko_sdk/platform/emdrv/nvm3/src/nvm3_lock.c 

OBJS += \
./gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_default_common_linker.o \
./gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_hal_flash.o \
./gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_lock.o 

C_DEPS += \
./gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_default_common_linker.d \
./gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_hal_flash.d \
./gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_lock.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_default_common_linker.o: /Users/Darek/SimplicityStudio/SDKs/gecko_sdk/platform/emdrv/nvm3/src/nvm3_default_common_linker.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DEFR32BG22C224F512IM40=1' '-DSL_BOARD_NAME="BRD4184A"' '-DSL_BOARD_REV="A02"' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' '-DSLI_RADIOAES_REQUIRES_MASKING=1' -I"/Users/Darek/SimplicityStudio/v5_workspace/BLE_Device_1_0" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32BG22/Include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//app/common/util/app_assert" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//protocol/bluetooth/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/bootloader" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/bootloader/api" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_cryptoacc_library/include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/dmadrv/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/hfxo_manager/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/library" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/mpu/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/nvm3/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/power_manager/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/common" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ble" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ieee802154" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/zwave" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/chip/efr32/efr32xg2x" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions/efr32xg22" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/se_manager/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/se_manager/src" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/silicon_labs/silabs_core/memory_manager" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/sleeptimer/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_protocol_crypto/src" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/uartdrv/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/udelay/inc" -I"/Users/Darek/SimplicityStudio/v5_workspace/BLE_Device_1_0/autogen" -I"/Users/Darek/SimplicityStudio/v5_workspace/BLE_Device_1_0/config" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_default_common_linker.d" -MT"gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_default_common_linker.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_hal_flash.o: /Users/Darek/SimplicityStudio/SDKs/gecko_sdk/platform/emdrv/nvm3/src/nvm3_hal_flash.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DEFR32BG22C224F512IM40=1' '-DSL_BOARD_NAME="BRD4184A"' '-DSL_BOARD_REV="A02"' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' '-DSLI_RADIOAES_REQUIRES_MASKING=1' -I"/Users/Darek/SimplicityStudio/v5_workspace/BLE_Device_1_0" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32BG22/Include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//app/common/util/app_assert" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//protocol/bluetooth/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/bootloader" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/bootloader/api" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_cryptoacc_library/include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/dmadrv/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/hfxo_manager/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/library" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/mpu/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/nvm3/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/power_manager/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/common" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ble" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ieee802154" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/zwave" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/chip/efr32/efr32xg2x" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions/efr32xg22" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/se_manager/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/se_manager/src" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/silicon_labs/silabs_core/memory_manager" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/sleeptimer/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_protocol_crypto/src" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/uartdrv/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/udelay/inc" -I"/Users/Darek/SimplicityStudio/v5_workspace/BLE_Device_1_0/autogen" -I"/Users/Darek/SimplicityStudio/v5_workspace/BLE_Device_1_0/config" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_hal_flash.d" -MT"gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_hal_flash.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_lock.o: /Users/Darek/SimplicityStudio/SDKs/gecko_sdk/platform/emdrv/nvm3/src/nvm3_lock.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DEFR32BG22C224F512IM40=1' '-DSL_BOARD_NAME="BRD4184A"' '-DSL_BOARD_REV="A02"' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' '-DSLI_RADIOAES_REQUIRES_MASKING=1' -I"/Users/Darek/SimplicityStudio/v5_workspace/BLE_Device_1_0" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32BG22/Include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//app/common/util/app_assert" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//protocol/bluetooth/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/bootloader" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/bootloader/api" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_cryptoacc_library/include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/dmadrv/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/hfxo_manager/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/include" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/library" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/mpu/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/nvm3/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/power_manager/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/common" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ble" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ieee802154" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/zwave" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/chip/efr32/efr32xg2x" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions/efr32xg22" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/se_manager/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/se_manager/src" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/silicon_labs/silabs_core/memory_manager" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/sleeptimer/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_protocol_crypto/src" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/uartdrv/inc" -I"/Users/Darek/SimplicityStudio/SDKs/gecko_sdk//platform/service/udelay/inc" -I"/Users/Darek/SimplicityStudio/v5_workspace/BLE_Device_1_0/autogen" -I"/Users/Darek/SimplicityStudio/v5_workspace/BLE_Device_1_0/config" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_lock.d" -MT"gecko_sdk_4.0.2/platform/emdrv/nvm3/src/nvm3_lock.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


