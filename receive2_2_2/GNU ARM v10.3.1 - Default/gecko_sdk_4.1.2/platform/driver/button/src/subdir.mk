################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Apps/SiliconLabs/SDKs/gecko_sdk/platform/driver/button/src/sl_button.c \
D:/Apps/SiliconLabs/SDKs/gecko_sdk/platform/driver/button/src/sl_simple_button.c 

OBJS += \
./gecko_sdk_4.1.2/platform/driver/button/src/sl_button.o \
./gecko_sdk_4.1.2/platform/driver/button/src/sl_simple_button.o 

C_DEPS += \
./gecko_sdk_4.1.2/platform/driver/button/src/sl_button.d \
./gecko_sdk_4.1.2/platform/driver/button/src/sl_simple_button.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.1.2/platform/driver/button/src/sl_button.o: D:/Apps/SiliconLabs/SDKs/gecko_sdk/platform/driver/button/src/sl_button.c gecko_sdk_4.1.2/platform/driver/button/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32BG13P632F512GM48=1' '-DSL_APP_PROPERTIES=1' '-DSL_BOARD_NAME="BRD4104A"' '-DSL_BOARD_REV="A02"' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CLIENT=1' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"D:\BLE_1\receive2_2_2\config" -I"D:\BLE_1\receive2_2_2" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32BG13P/Include" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/common/util/app_assert" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/app_btmesh_util" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/common/util/app_log" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//protocol/bluetooth/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/common/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/board/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/bootloader" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/bootloader/api" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/btmesh_factory_reset" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/btmesh_lpn" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/btmesh_wstk_lcd" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/driver/button/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/driver/debug/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/device_init/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/middleware/glib/dmd" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/middleware/glib" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/emlib/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/middleware/glib/glib" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/driver/i2cspm/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/in_place_ota_dfu" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/iostream/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/memlcd/src/ls013b7dh03" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/include" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/library" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/memlcd/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/memlcd/inc/memlcd_usart" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/mpu/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/emdrv/nvm3/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/power_manager/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_psa_driver/inc/public" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/common" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ble" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ieee802154" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/zwave" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/rail_util_pti" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/si70xx/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/silicon_labs/silabs_core/memory_manager" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/simple_timer" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/system/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/sleeptimer/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_protocol_crypto/src" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/udelay/inc" -I"D:\BLE_1\receive2_2_2\autogen" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.1.2/platform/driver/button/src/sl_button.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.1.2/platform/driver/button/src/sl_simple_button.o: D:/Apps/SiliconLabs/SDKs/gecko_sdk/platform/driver/button/src/sl_simple_button.c gecko_sdk_4.1.2/platform/driver/button/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32BG13P632F512GM48=1' '-DSL_APP_PROPERTIES=1' '-DSL_BOARD_NAME="BRD4104A"' '-DSL_BOARD_REV="A02"' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CLIENT=1' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"D:\BLE_1\receive2_2_2\config" -I"D:\BLE_1\receive2_2_2" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32BG13P/Include" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/common/util/app_assert" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/app_btmesh_util" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/common/util/app_log" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//protocol/bluetooth/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/common/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/board/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/bootloader" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/bootloader/api" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/btmesh_factory_reset" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/btmesh_lpn" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/btmesh_wstk_lcd" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/driver/button/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/driver/debug/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/device_init/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/middleware/glib/dmd" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/middleware/glib" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/emlib/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/middleware/glib/glib" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/driver/i2cspm/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/in_place_ota_dfu" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/iostream/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/memlcd/src/ls013b7dh03" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/include" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/library" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/memlcd/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/memlcd/inc/memlcd_usart" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/mpu/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/emdrv/nvm3/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/power_manager/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_psa_driver/inc/public" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/common" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ble" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ieee802154" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/zwave" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/rail_util_pti" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/si70xx/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/silicon_labs/silabs_core/memory_manager" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/simple_timer" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/system/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/sleeptimer/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_protocol_crypto/src" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/udelay/inc" -I"D:\BLE_1\receive2_2_2\autogen" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.1.2/platform/driver/button/src/sl_simple_button.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


