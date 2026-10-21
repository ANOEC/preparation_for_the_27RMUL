# Shared LibXR/XRobot integration for one isolated firmware sub-build.

set(LIBXR_SYSTEM FreeRTOS)
set(LIBXR_DRIVER st)
set(XROBOT_MODULES_DIR "${FIRMWARE_XR_DIR}/Modules")
set(LIBXR_USER_SOURCES_GLOB "${FIRMWARE_XR_DIR}/User/*.cpp")
set(LIBXR_OPT_DEBUG "-Og")
set(LIBXR_OPT_RELEASE "")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LIBHT_SOURCE_DIR "${ROBOT_ROOT}/lib/libht")
add_subdirectory(
    "${LIBHT_SOURCE_DIR}/utility/serialization"
    "${CMAKE_CURRENT_BINARY_DIR}/LibHT/serialization"
)
add_subdirectory(
    "${LIBHT_SOURCE_DIR}/utility/crc"
    "${CMAKE_CURRENT_BINARY_DIR}/LibHT/crc"
)

set(LIBXR_SOURCE_DIR "${ROBOT_ROOT}/lib/LibXR" CACHE PATH
    "LibXR source directory")
add_subdirectory("${LIBXR_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/LibXR")

target_link_libraries(xr PUBLIC stm32cubemx)
target_include_directories(xr PUBLIC
    "${FIRMWARE_HAL_DIR}/Core/Inc"
    "${FIRMWARE_XR_DIR}/User"
)
target_compile_features(xr PUBLIC cxx_std_20)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    RUNTIME_OUTPUT_DIRECTORY "${FIRMWARE_OUTPUT_DIR}"
)

if(LIBXR_USER_SOURCES_GLOB)
    file(GLOB LIBXR_USER_SOURCES CONFIGURE_DEPENDS "${LIBXR_USER_SOURCES_GLOB}")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${LIBXR_USER_SOURCES})
endif()

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE xr)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:${LIBXR_OPT_DEBUG}>
    $<$<CONFIG:Release>:${LIBXR_OPT_RELEASE}>
)

foreach(library IN ITEMS xr STM32_Drivers FreeRTOS)
    target_compile_options(${library} PRIVATE
        $<$<CONFIG:Debug>:-O2>
        $<$<CONFIG:Release>:${LIBXR_OPT_RELEASE}>
    )
endforeach()

file(MAKE_DIRECTORY "${FIRMWARE_OUTPUT_DIR}")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} --output-target ihex
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        "${FIRMWARE_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.hex"
    COMMAND ${CMAKE_OBJCOPY} --output-target binary --strip-all
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        "${FIRMWARE_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.bin"
    VERBATIM
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    ADDITIONAL_CLEAN_FILES
        "${FIRMWARE_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.map;${FIRMWARE_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.hex;${FIRMWARE_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.bin"
)
