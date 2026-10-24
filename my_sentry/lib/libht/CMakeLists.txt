cmake_minimum_required(VERSION 3.22)

# LibHT umbrella: defines every library in this collection in one pass.
#
# Everything defined here is a plain target -- nothing writes to a consumer's
# targets and nothing depends on LibXR existing yet. That is what lets a
# consumer call add_subdirectory() on this directory exactly once, at any point
# in its own build, and link whatever it needs afterwards:
#
#   add_subdirectory(<path>/LibHT)
#   ...
#   target_link_libraries(<consumer> PUBLIC LibHT::Wave LibHT::rmlink LibHT::crc)
#
# Individual libraries remain usable on their own (FetchContent or
# add_subdirectory on the subdirectory) for host test builds.

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(LibHT LANGUAGES CXX)
endif()

set(LIBHT_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")

option(LIBHT_BUILD_ALGORITHM "Define the Algorithm INTERFACE targets" ON)
option(LIBHT_BUILD_CRC "Define the utility/crc target" ON)
option(LIBHT_BUILD_RMLINK "Define the protocol/rmlink target" ON)
option(LIBHT_BUILD_SERIALIZATION "Define the shared payload codec target" ON)
option(LIBHT_BUILD_VOFA_PLUS "Define the VOFA+ FireWater target" ON)

if(LIBHT_BUILD_SERIALIZATION OR LIBHT_BUILD_RMLINK)
  add_subdirectory(utility/serialization)
endif()

# RMLink links crc::crc, so Crc is built whenever either is requested and is
# also what gives RMLink's own FetchContent fallback nothing to do.
if(LIBHT_BUILD_CRC OR LIBHT_BUILD_RMLINK)
  add_subdirectory(utility/crc)
endif()

if(LIBHT_BUILD_ALGORITHM)
  add_subdirectory(algorithm)
endif()

if(LIBHT_BUILD_RMLINK)
  add_subdirectory(protocol/rmlink)
endif()

if(LIBHT_BUILD_VOFA_PLUS)
  add_subdirectory(debug/vofa_plus)
endif()
