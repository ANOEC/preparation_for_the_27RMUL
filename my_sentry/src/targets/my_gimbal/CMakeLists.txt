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

add_executable(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PRIVATE "${USER_DIR}")

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
