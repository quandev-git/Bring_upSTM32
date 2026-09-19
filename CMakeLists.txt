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
set(CMAKE_PROJECT_NAME Bring_upSTM32)

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
    Core/Src/door_control.c
    Core/Src/rc522.c
    Core/Src/lcd2004.c
    Core/Src/pi_uart.c
    Core/Src/tasks/task_nfc_read.c
    Core/Src/tasks/task_uart_pi.c
    Core/Src/tasks/task_front_door_control.c
    Core/Src/tasks/task_lcd.c
    Core/Src/tasks/task_buzzer.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c
)

option(DOOR_BUILD_LEGACY_TASKS "Compile inactive legacy tasks" OFF)
if(DOOR_BUILD_LEGACY_TASKS)
    function(door_add_optional_task task_name source_file)
        option(DOOR_TASK_${task_name} "Compile inactive legacy task ${task_name}" ON)
        if(DOOR_TASK_${task_name})
            target_sources(${CMAKE_PROJECT_NAME} PRIVATE Core/Src/tasks/${source_file})
        endif()
    endfunction()

    door_add_optional_task(CURRENT_MONITOR   task_current_monitor.c)
    door_add_optional_task(SKY_DOOR_CONTROL  task_sky_door_control.c)
    door_add_optional_task(BUTTON            task_button.c)
    door_add_optional_task(ACCESS_CONTROL    task_access_control.c)
    door_add_optional_task(SKY_DOOR_LOGIC    task_sky_door_logic.c)
    door_add_optional_task(RAIN_SENSOR       task_rain_sensor.c)
    door_add_optional_task(PARKING_COUNTER   task_parking_counter.c)
    door_add_optional_task(IR_OBSTACLE       task_ir_obstacle.c)
    door_add_optional_task(WATCHDOG          task_watchdog.c)
endif()

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src
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
