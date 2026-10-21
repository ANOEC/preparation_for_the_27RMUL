cmake_minimum_required(VERSION 3.22)

project(my_sentry NONE)

include(ExternalProject)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(FIRMWARE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/firmware")
set(ROBOT_TOOLCHAIN "${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")

function(add_firmware_subbuild name)
    set(child_cmake_args
        "-DCMAKE_TOOLCHAIN_FILE=${ROBOT_TOOLCHAIN}"
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
        "-DROBOT_ROOT=${CMAKE_SOURCE_DIR}"
        "-DFIRMWARE_OUTPUT_DIR=${FIRMWARE_OUTPUT_DIR}"
    )
    if(Python3_EXECUTABLE)
        list(APPEND child_cmake_args
            "-DPython3_EXECUTABLE=${Python3_EXECUTABLE}"
        )
    endif()

    ExternalProject_Add(${name}_firmware
        SOURCE_DIR "${CMAKE_SOURCE_DIR}/src/targets/${name}"
        BINARY_DIR "${CMAKE_BINARY_DIR}/targets/${name}"
        CMAKE_GENERATOR "${CMAKE_GENERATOR}"
        CMAKE_ARGS ${child_cmake_args}
        INSTALL_COMMAND ""
        BUILD_ALWAYS TRUE
        BUILD_BYPRODUCTS
            "${FIRMWARE_OUTPUT_DIR}/${name}.elf"
            "${FIRMWARE_OUTPUT_DIR}/${name}.hex"
            "${FIRMWARE_OUTPUT_DIR}/${name}.bin"
            "${FIRMWARE_OUTPUT_DIR}/${name}.map"
        USES_TERMINAL_CONFIGURE TRUE
        USES_TERMINAL_BUILD TRUE
    )
endfunction()

add_firmware_subbuild(my_chassis)
add_firmware_subbuild(my_gimbal)

add_custom_target(merge_compile_commands
    COMMAND ${CMAKE_COMMAND}
        "-DINPUT_A=${CMAKE_BINARY_DIR}/targets/my_chassis/compile_commands.json"
        "-DINPUT_B=${CMAKE_BINARY_DIR}/targets/my_gimbal/compile_commands.json"
        "-DOUTPUT=${CMAKE_BINARY_DIR}/compile_commands.json"
        -P "${CMAKE_SOURCE_DIR}/cmake/MergeCompileCommands.cmake"
    DEPENDS my_chassis_firmware my_gimbal_firmware
    COMMENT "Merging chassis and gimbal compilation databases"
    VERBATIM
)

add_custom_target(robot
    DEPENDS merge_compile_commands
)
