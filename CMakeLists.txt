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
set(CMAKE_PROJECT_NAME Qust_version)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)
enable_language(CXX)

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
 applications/can_task.cpp # <- 添加这一行
    sp_middleware/io/can/can.cpp # <- 添加这一行
    sp_middleware/motor/rm_motor/rm_motor.cpp # <- 添加这一行

 applications/led_task.cpp # <- 添加这一行
    sp_middleware/io/led/led.cpp # <- 添加这一行

        applications/voltage_detect.cpp # <- 添加这一行
    sp_middleware/io/adc/adc.cpp # <- 添加这一行

       applications/buzzer_task.cpp # <- 添加这一行
    sp_middleware/io/buzzer/buzzer.cpp # <- 添加这一行

      applications/uart_task.cpp # <- 添加这一行
    sp_middleware/io/dbus/dbus.cpp # <- 添加这一行

 applications/vision/vision_task.cpp # <- 上位机 USB 虚拟串口协议
    sp_middleware/tools/crc/crc.cpp # <- CRC16 校验

 applications/usb_device/usbd_conf.c # <- USB OTG FS 低层驱动
 applications/usb_device/usbd_desc.c # <- USB 设备描述符
 applications/usb_device/usbd_cdc_if.c # <- USB CDC 收发接口

    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    sp_middleware/
    applications/usb_device/
    applications/vision/
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
