cmake_minimum_required(VERSION 3.15.3)

project(Node)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(STM32CUBEMX_GENERATED_FILES
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c
        Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c

        Core/Inc/main.h
        Core/Inc/stm32f7xx_hal_conf.h
        Core/Inc/stm32f7xx_it.h

        Core/Src/main.c
        Core/Src/stm32f7xx_hal_msp.c
        Core/Src/stm32f7xx_it.c
        Core/Src/system_stm32f7xx.c

        startup_stm32f722xx.s)

set(EXECUTABLE ${PROJECT_NAME}.out)

add_executable(${EXECUTABLE} ${STM32CUBEMX_GENERATED_FILES})

target_compile_definitions(${EXECUTABLE} PRIVATE
        -DUSE_HAL_DRIVER
        -DSTM32F722xx
        )

target_include_directories(${EXECUTABLE} PRIVATE
        Core/Inc
        Drivers/STM32F7xx_HAL_Driver/Inc
        Drivers/CMSIS/Device/ST/STM32F7xx/Include
        Drivers/CMSIS/Include
        )

target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-sp-d16
        -mfloat-abi=hard

        -fdata-sections
        -ffunction-sections

        -Wall

        $<$<CONFIG:Debug>:-Og>
        )

target_link_options(${EXECUTABLE} PRIVATE
        -T${CMAKE_SOURCE_DIR}/STM32F722ICKx_FLASH.ld
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-sp-d16
        -mfloat-abi=hard
        -specs=nano.specs
        -lc
        -lm
        -lnosys
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        )

# Print executable size
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE})

# Create hex file
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)
