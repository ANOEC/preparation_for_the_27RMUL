# Header-only CRC library shared by RMLink and firmware modules.
# Usable standalone (FetchContent / add_subdirectory) or through the LibHT
# umbrella; the guard keeps repeated inclusion from different consumers in one
# build harmless.

if(NOT TARGET crc)
  add_library(crc INTERFACE)
  add_library(crc::crc ALIAS crc)
  target_include_directories(crc INTERFACE "${CMAKE_CURRENT_LIST_DIR}/include")
  target_compile_features(crc INTERFACE cxx_std_20)

  # Optional STM32 backend. The source guards itself on ST's CMSIS macros and
  # compiles to nothing when no STM32 device header is reachable, so a consumer
  # may add ${CRC_STM32_ADAPTER_SOURCES} unconditionally. The device headers must
  # come from the consumer, hence the exported variables rather than a source in
  # this target.
  set(CRC_STM32_ADAPTER_DIR "${CMAKE_CURRENT_LIST_DIR}/adapters/stm32" CACHE INTERNAL "")
  set(CRC_STM32_ADAPTER_SOURCES "${CMAKE_CURRENT_LIST_DIR}/adapters/stm32/crc_stm32.cpp"
      CACHE INTERNAL "")
endif()

# Namespaced alias. Added separately so it also appears when a consumer created
# the crc target through an earlier FetchContent call.
if(NOT TARGET LibHT::crc)
  add_library(LibHT::crc ALIAS crc)
endif()
