cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(my_chassis LANGUAGES C CXX ASM)
message(STATUS "Building ${PROJECT_NAME} (${CMAKE_BUILD_TYPE})")

if(NOT ROBOT_ROOT)
    get_filename_component(ROBOT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
endif()
if(NOT FIRMWARE_OUTPUT_DIR)
    set(FIRMWARE_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/firmware")
endif()

set(FIRMWARE_HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hal")
set(FIRMWARE_XR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/xr")
set(USER_DIR "${FIRMWARE_XR_DIR}/User")
set(CHASSIS_CONFIG_DIR "${ROBOT_ROOT}/src/config/chassis")
set(TASK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/task")

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(CHASSIS_PARAM_TOML "${CHASSIS_CONFIG_DIR}/params/chassis.toml")
set(CHASSIS_PARAM_GENERATOR "${CHASSIS_CONFIG_DIR}/scripts/generate_params.py")
set(CHASSIS_PARAM_HEADER "${CHASSIS_CONFIG_DIR}/generated/chassis_params.hpp")
add_custom_command(
    OUTPUT "${CHASSIS_PARAM_HEADER}"
    COMMAND "${Python3_EXECUTABLE}" "${CHASSIS_PARAM_GENERATOR}"
            "${CHASSIS_PARAM_TOML}" "${CHASSIS_PARAM_HEADER}"
    DEPENDS "${CHASSIS_PARAM_GENERATOR}" "${CHASSIS_PARAM_TOML}"
    COMMENT "Generating chassis parameters from chassis.toml"
    VERBATIM
)
add_custom_target(generate_chassis_params ALL DEPENDS "${CHASSIS_PARAM_HEADER}")

set(TASK_CONFIG_TOML "${TASK_DIR}/task.toml")
set(TASK_GENERATOR "${TASK_DIR}/generate_task.py")
set(TASK_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/task")
set(TASK_GENERATED_HEADER "${TASK_GENERATED_DIR}/task_generated.hpp")
set(TASK_GENERATED_SOURCE "${TASK_GENERATED_DIR}/task_generated.cpp")
add_custom_command(
    OUTPUT "${TASK_GENERATED_HEADER}" "${TASK_GENERATED_SOURCE}"
    COMMAND "${Python3_EXECUTABLE}" "${TASK_GENERATOR}"
            "${TASK_CONFIG_TOML}" "${TASK_GENERATED_DIR}"
    DEPENDS "${TASK_GENERATOR}" "${TASK_CONFIG_TOML}"
    COMMENT "Generating LibXR task framework"
    VERBATIM
)
add_custom_target(generate_tasks ALL
    DEPENDS "${TASK_GENERATED_HEADER}" "${TASK_GENERATED_SOURCE}"
)

add_executable(${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}
    generate_chassis_params
    generate_tasks
)

target_sources(${PROJECT_NAME} PRIVATE
    "${USER_DIR}/app/buzzer/you_buzzer.cpp"
    "${USER_DIR}/app/remote/remote_app.cpp"
    "${USER_DIR}/app/chassis/body_velocity_controller.cpp"
    "${USER_DIR}/app/chassis/chassis.cpp"
    "${USER_DIR}/app/chassis/chassis_kinematics.cpp"
    "${USER_DIR}/app/chassis/chassis_solver.cpp"
    "${USER_DIR}/app/chassis/power_control.cpp"
    "${USER_DIR}/app/chassis/wheel_controller.cpp"
    "${TASK_DIR}/task_runtime.cpp"
    "${TASK_DIR}/chassis_task.cpp"
    "${TASK_GENERATED_SOURCE}"
)
target_include_directories(${PROJECT_NAME} PRIVATE
    "${USER_DIR}"
    "${ROBOT_ROOT}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${TASK_DIR}"
    "${TASK_GENERATED_DIR}"
)

add_subdirectory(
    "${ROBOT_ROOT}/cmake/platform/stm32"
    "${CMAKE_CURRENT_BINARY_DIR}/platform/stm32"
)

file(MAKE_DIRECTORY "${FIRMWARE_OUTPUT_DIR}")
target_link_options(${PROJECT_NAME} PRIVATE
    "-T${FIRMWARE_HAL_DIR}/STM32H723xG_flash.ld"
    "-T${FIRMWARE_HAL_DIR}/dma_buffer.ld"
    "-Wl,-Map=${FIRMWARE_OUTPUT_DIR}/${PROJECT_NAME}.map"
    "-Wl,--gc-sections"
    "-Wl,--print-memory-usage"
)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS
    "${FIRMWARE_HAL_DIR}/STM32H723xG_flash.ld"
    "${FIRMWARE_HAL_DIR}/dma_buffer.ld"
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
include("${ROBOT_ROOT}/cmake/LibXR.CMake")
