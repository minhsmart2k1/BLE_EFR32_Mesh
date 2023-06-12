################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Apps/SiliconLabs/SDKs/gecko_sdk/app/bluetooth/common/in_place_ota_dfu/sl_bt_in_place_ota_dfu.c 

OBJS += \
./gecko_sdk_4.1.2/app/bluetooth/common/in_place_ota_dfu/sl_bt_in_place_ota_dfu.o 

C_DEPS += \
./gecko_sdk_4.1.2/app/bluetooth/common/in_place_ota_dfu/sl_bt_in_place_ota_dfu.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.1.2/app/bluetooth/common/in_place_ota_dfu/sl_bt_in_place_ota_dfu.o: D:/Apps/SiliconLabs/SDKs/gecko_sdk/app/bluetooth/common/in_place_ota_dfu/sl_bt_in_place_ota_dfu.c gecko_sdk_4.1.2/app/bluetooth/common/in_place_ota_dfu/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32BG13P632F512GM48=1' '-DSL_APP_PROPERTIES=1' '-DSL_BOARD_NAME="BRD4104A"' '-DSL_BOARD_REV="A02"' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CLIENT=1' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"D:\BLE\ADV\config" -I"D:\BLE\ADV\config\btconf" -I"D:\BLE\ADV\autogen" -I"D:\BLE\ADV" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32BG13P/Include" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/common/util/app_assert" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/common/util/app_log" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//protocol/bluetooth/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/common/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/board/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/bootloader" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/bootloader/api" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/driver/debug/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/device_init/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/emlib/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/driver/i2cspm/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/in_place_ota_dfu" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/iostream/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/include" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/mbedtls/library" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/mpu/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/power_manager/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_psa_driver/inc/public" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/common" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ble" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ieee802154" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/zwave" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/rail_util_pti" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/sensor_rht" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/sensor_select" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//hardware/driver/si70xx/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/silicon_labs/silabs_core/memory_manager" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//app/bluetooth/common/simple_timer" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/system/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/sleeptimer/inc" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//util/third_party/crypto/sl_component/sl_protocol_crypto/src" -I"D:/Apps/SiliconLabs/SDKs/gecko_sdk//platform/service/udelay/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.1.2/app/bluetooth/common/in_place_ota_dfu/sl_bt_in_place_ota_dfu.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


