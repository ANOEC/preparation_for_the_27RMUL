cmake_minimum_required(VERSION 3.22)

project(crc_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(crc SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
FetchContent_MakeAvailable(crc)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

add_executable(crc_tests crc_tests.cpp)
target_link_libraries(crc_tests PRIVATE crc::crc GTest::gtest_main Threads::Threads)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(crc_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

enable_testing()
include(GoogleTest)
gtest_discover_tests(crc_tests)
