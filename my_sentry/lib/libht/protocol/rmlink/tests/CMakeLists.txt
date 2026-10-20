cmake_minimum_required(VERSION 3.22)

project(rmlink_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(RMLINK_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
add_subdirectory("${RMLINK_ROOT}/../../utility/serialization" serialization)

include(FetchContent)
FetchContent_Declare(crc SOURCE_DIR "${RMLINK_ROOT}/../../utility/crc")
FetchContent_MakeAvailable(crc)

find_package(GTest REQUIRED)

add_library(rmlink_core STATIC
  "${RMLINK_ROOT}/src/core.cpp"
  "${RMLINK_ROOT}/src/types.cpp"
  "${RMLINK_ROOT}/src/wire.cpp"
)
target_include_directories(rmlink_core PUBLIC "${RMLINK_ROOT}/include")
target_link_libraries(rmlink_core PUBLIC crc::crc LibHT::serialization)
target_compile_features(rmlink_core PUBLIC cxx_std_20)

add_executable(rmlink_wire_tests wire_tests.cpp)
target_link_libraries(rmlink_wire_tests PRIVATE rmlink_core GTest::gtest_main)

add_executable(rmlink_core_tests core_tests.cpp shared_codec_tests.cpp)
target_link_libraries(rmlink_core_tests PRIVATE rmlink_core GTest::gtest_main)

add_executable(rmlink_basic_example "${RMLINK_ROOT}/examples/basic.cpp")
target_link_libraries(rmlink_basic_example PRIVATE rmlink_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(rmlink_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_compile_options(rmlink_wire_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_compile_options(rmlink_core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_compile_options(rmlink_basic_example PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

enable_testing()
include(GoogleTest)
gtest_discover_tests(rmlink_wire_tests)
gtest_discover_tests(rmlink_core_tests)
add_test(NAME rmlink_basic_example COMMAND rmlink_basic_example)
