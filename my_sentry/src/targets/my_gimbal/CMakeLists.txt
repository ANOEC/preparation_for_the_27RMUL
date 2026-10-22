cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(my_gimbal LANGUAGES C CXX ASM)
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
set(GIMBAL_CONFIG_DIR "${ROBOT_ROOT}/src/config/gimbal")

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GIMBAL_PARAM_TOML "${GIMBAL_CONFIG_DIR}/gimbal.toml")
set(GIMBAL_PARAM_GENERATOR "${GIMBAL_CONFIG_DIR}/generate_params.py")
set(GIMBAL_PARAM_HEADER "${GIMBAL_CONFIG_DIR}/generated/gimbal_params.hpp")
add_custom_command(
    OUTPUT "${GIMBAL_PARAM_HEADER}"
    COMMAND "${Python3_EXECUTABLE}" "${GIMBAL_PARAM_GENERATOR}"
            "${GIMBAL_PARAM_TOML}" "${GIMBAL_PARAM_HEADER}"
    DEPENDS "${GIMBAL_PARAM_GENERATOR}" "${GIMBAL_PARAM_TOML}"
    COMMENT "Generating gimbal parameters from gimbal.toml"
    VERBATIM
)
add_custom_target(generate_gimbal_params ALL DEPENDS "${GIMBAL_PARAM_HEADER}")

add_executable(${PROJECT_NAME})
add_dependencies(${PROJECT_NAME} generate_gimbal_params)
target_include_directories(${PROJECT_NAME} PRIVATE
    "${USER_DIR}"
    "${ROBOT_ROOT}/src"
)

add_subdirectory(
    "${ROBOT_ROOT}/cmake/platform/stm32"
    "${CMAKE_CURRENT_BINARY_DIR}/platform/stm32"
)

file(MAKE_DIRECTORY "${FIRMWARE_OUTPUT_DIR}")
target_link_options(${PROJECT_NAME} PRIVATE
    "-T${FIRMWARE_HAL_DIR}/STM32H723xG_flash.ld"
    "-Wl,-Map=${FIRMWARE_OUTPUT_DIR}/${PROJECT_NAME}.map"
    "-Wl,--gc-sections"
    "-Wl,--print-memory-usage"
)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS
    "${FIRMWARE_HAL_DIR}/STM32H723xG_flash.ld"
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
include("${ROBOT_ROOT}/cmake/LibXR.CMake")
