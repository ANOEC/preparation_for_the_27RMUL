# Shared STM32 platform integration for isolated firmware builds.
# The parent project must define FIRMWARE_HAL_DIR and create its executable.

if(NOT TARGET ${CMAKE_PROJECT_NAME})
    message(FATAL_ERROR "Create ${CMAKE_PROJECT_NAME} before adding the STM32 platform subdirectory")
endif()
if(NOT IS_DIRECTORY "${FIRMWARE_HAL_DIR}/Core")
    message(FATAL_ERROR "Invalid FIRMWARE_HAL_DIR: ${FIRMWARE_HAL_DIR}")
endif()

set(MX_DEFINES
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H723xx
    $<$<CONFIG:Debug>:DEBUG>
)

set(MX_INCLUDE_DIRS
    "${FIRMWARE_HAL_DIR}/Core/Inc"
    "${FIRMWARE_HAL_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc"
    "${FIRMWARE_HAL_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
    "${FIRMWARE_HAL_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include"
    "${FIRMWARE_HAL_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
    "${FIRMWARE_HAL_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    "${FIRMWARE_HAL_DIR}/Drivers/CMSIS/RTOS2/Include"
    "${FIRMWARE_HAL_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    "${FIRMWARE_HAL_DIR}/Drivers/CMSIS/Include"
)

file(GLOB MX_APPLICATION_SOURCES CONFIGURE_DEPENDS
    "${FIRMWARE_HAL_DIR}/Core/Src/*.c"
)
list(REMOVE_ITEM MX_APPLICATION_SOURCES
    "${FIRMWARE_HAL_DIR}/Core/Src/system_stm32h7xx.c"
)
list(APPEND MX_APPLICATION_SOURCES
    "${FIRMWARE_HAL_DIR}/startup_stm32h723xx.s"
)

file(GLOB STM32_HAL_SOURCES CONFIGURE_DEPENDS
    "${FIRMWARE_HAL_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/*.c"
)
list(APPEND STM32_HAL_SOURCES
    "${FIRMWARE_HAL_DIR}/Core/Src/system_stm32h7xx.c"
)

set(FREERTOS_ROOT
    "${FIRMWARE_HAL_DIR}/Middlewares/Third_Party/FreeRTOS/Source"
)
file(GLOB FREERTOS_KERNEL_SOURCES CONFIGURE_DEPENDS
    "${FREERTOS_ROOT}/*.c"
    "${FREERTOS_ROOT}/CMSIS_RTOS_V2/*.c"
    "${FREERTOS_ROOT}/portable/Common/*.c"
)
list(APPEND FREERTOS_KERNEL_SOURCES
    "${FREERTOS_ROOT}/portable/MemMang/heap_4.c"
    "${FREERTOS_ROOT}/portable/GCC/ARM_CM4F/port.c"
)

add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_INCLUDE_DIRS})
target_compile_definitions(stm32cubemx INTERFACE ${MX_DEFINES})

add_library(STM32_Drivers OBJECT ${STM32_HAL_SOURCES})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

add_library(FreeRTOS OBJECT ${FREERTOS_KERNEL_SOURCES})
target_link_libraries(FreeRTOS PUBLIC stm32cubemx)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_APPLICATION_SOURCES})
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    stm32cubemx
    STM32_Drivers
    FreeRTOS
    ${TOOLCHAIN_LINK_LIBRARIES}
)

if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(FATAL_ERROR "STM32CubeMX generated code requires C11 or newer")
endif()
