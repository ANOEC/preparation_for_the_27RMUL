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
set(CMAKE_PROJECT_NAME my_chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Generate strongly typed chassis parameters from the editable TOML source.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(CHASSIS_PARAM_TOML "${CMAKE_CURRENT_SOURCE_DIR}/User/config/chassis.toml")
set(CHASSIS_PARAM_GENERATOR "${CMAKE_CURRENT_SOURCE_DIR}/User/config/generate_params.py")
set(CHASSIS_PARAM_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/User/config/generated/chassis_params.hpp")
add_custom_command(
    OUTPUT "${CHASSIS_PARAM_HEADER}"
    COMMAND "${Python3_EXECUTABLE}" "${CHASSIS_PARAM_GENERATOR}"
            "${CHASSIS_PARAM_TOML}" "${CHASSIS_PARAM_HEADER}"
    DEPENDS "${CHASSIS_PARAM_GENERATOR}" "${CHASSIS_PARAM_TOML}"
    VERBATIM
    COMMENT "Generating chassis parameters from chassis.toml"
)
add_custom_target(generate_chassis_params ALL DEPENDS "${CHASSIS_PARAM_HEADER}")

# Generate the RTT topic monitor registry from its editable TOML source.
set(TOPIC_MONITOR_TOML "${CMAKE_CURRENT_SOURCE_DIR}/User/debug/topic_monitor.toml")
set(TOPIC_MONITOR_GENERATOR "${CMAKE_CURRENT_SOURCE_DIR}/User/debug/generate_topic_monitor.py")
set(TOPIC_MONITOR_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/User/debug/generated/topic_monitor.hpp")
add_custom_command(
    OUTPUT "${TOPIC_MONITOR_HEADER}"
    COMMAND "${Python3_EXECUTABLE}" "${TOPIC_MONITOR_GENERATOR}"
            "${TOPIC_MONITOR_TOML}" "${TOPIC_MONITOR_HEADER}"
    DEPENDS "${TOPIC_MONITOR_GENERATOR}" "${TOPIC_MONITOR_TOML}"
    VERBATIM
    COMMENT "Generating RTT topic monitor registry from topic_monitor.toml"
)
add_custom_target(generate_topic_monitor ALL DEPENDS "${TOPIC_MONITOR_HEADER}")

# Generate the LibXR/FreeRTOS task framework from its editable TOML source.
set(TASK_CONFIG_TOML "${CMAKE_CURRENT_SOURCE_DIR}/User/task/task.toml")
set(TASK_GENERATOR "${CMAKE_CURRENT_SOURCE_DIR}/User/task/generate_task.py")
set(TASK_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/task")
set(TASK_GENERATED_HEADER "${TASK_GENERATED_DIR}/task_generated.hpp")
set(TASK_GENERATED_SOURCE "${TASK_GENERATED_DIR}/task_generated.cpp")
add_custom_command(
    OUTPUT "${TASK_GENERATED_HEADER}" "${TASK_GENERATED_SOURCE}"
    COMMAND "${Python3_EXECUTABLE}" "${TASK_GENERATOR}"
            "${TASK_CONFIG_TOML}" "${TASK_GENERATED_DIR}"
    DEPENDS "${TASK_GENERATOR}" "${TASK_CONFIG_TOML}"
    VERBATIM
    COMMENT "Generating LibXR task framework from task.toml"
)
add_custom_target(generate_tasks ALL
    DEPENDS "${TASK_GENERATED_HEADER}" "${TASK_GENERATED_SOURCE}")

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
add_dependencies(${CMAKE_PROJECT_NAME}
    generate_chassis_params
    generate_topic_monitor
    generate_tasks
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    User/app/buzzer/you_buzzer.cpp
    User/debug/topic_debug_app.cpp
    User/app/remote/remote_app.cpp
    User/app/chassis/body_velocity_controller.cpp
    User/app/chassis/chassis.cpp
    User/app/chassis/chassis_kinematics.cpp
    User/app/chassis/chassis_solver.cpp
    User/app/chassis/power_control.cpp
    User/app/chassis/wheel_controller.cpp
    User/task/task_runtime.cpp
    User/task/chassis_task.cpp
    "${TASK_GENERATED_SOURCE}"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "${TASK_GENERATED_DIR}"
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

# Add LibXR
include(${CMAKE_CURRENT_LIST_DIR}/cmake/LibXR.CMake)
