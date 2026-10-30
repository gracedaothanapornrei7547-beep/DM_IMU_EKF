cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DM02)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # User_File/BSP
    User_File/BSP/DWT/bsp_dwt.c
    User_File/BSP/UART/bsp_uart.c
    User_File/BSP/USB/bsp_usb.c
    User_File/BSP/USB/vofa.c
    User_File/BSP/USB/usb_descriptors.c
    # User_File/Driver
    User_File/Driver/BMI088/BMI088driver.c
    User_File/Driver/BMI088/BMI088Middleware.c
    User_File/Driver/LCD/lcd.c
    User_File/Driver/Key/adc_key.c
    # User_File/Algorithm
    User_File/Algorithm/Quaternion/qekf.c
    # User_File/Module
    User_File/Module/IMU/imu.c
    # User_File/Application
    User_File/Application/Menu/menu_task.c
    User_File/Application/Demo/prio_inv.c
    User_File/Application/robot_task.c
    User_File/Application/control_task.c
    # ADC1/PA5 is used by the MC02 LCD five-way key.
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    # ---- TinyUSB (device CDC, dwc2) ----
    TinyUSB/src/tusb.c
    TinyUSB/src/common/tusb_fifo.c
    TinyUSB/src/device/usbd.c
    TinyUSB/src/device/usbd_control.c
    TinyUSB/src/class/cdc/cdc_device.c
    TinyUSB/src/portable/synopsys/dwc2/dcd_dwc2.c
    TinyUSB/src/portable/synopsys/dwc2/dwc2_common.c
)

# The vendor Chinese font table uses flat aggregate initializers accepted by GCC.
set_source_files_properties(User_File/Driver/LCD/lcd.c PROPERTIES
    COMPILE_OPTIONS "-Wno-missing-braces"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User_File/BSP/DWT
    User_File/BSP/UART
    User_File/BSP/USB
    User_File/Driver/BMI088
    User_File/Driver/LCD
    User_File/Driver/Key
    User_File/Algorithm/Quaternion
    User_File/Module/IMU
    User_File/Application/Menu
    User_File/Application/Demo
    User_File/Application
    USB_DEVICE/App
    USB_DEVICE/Target
    TinyUSB/src                # TinyUSB headers
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    # IMU_RAW_LOG   # 取消注释可让USB改流原始IMU数据(7通道@200Hz)用于离线调参(见 /tmp/replay.py)
    # ---- 优先级反转 demo(Step 7b 学习用,默认关)----
    # PRIO_INV_DEMO        # 取消注释:开启 L/M/H 三任务+共享锁,H 等锁延迟经 USART1 打印
    # PRIO_INV_USE_MUTEX   # 再取消注释:锁用互斥锁(优先级继承);保持注释=二值信号量(复现反转)
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 启用 newlib-nano 的浮点 printf/vsnprintf（默认关闭，打印 float 必需）
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
