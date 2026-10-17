# Transport-independent protocol core; platform bridges belong to consumers.

if(NOT TARGET rmlink)
  if(NOT TARGET LibHT::serialization)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../utility/serialization"
      "${CMAKE_CURRENT_BINARY_DIR}/serialization")
  endif()
  # Shared CRC library; local source until it moves to its own repository. The
  # LibHT umbrella normally defines it first, so only a standalone consumer of
  # protocol/rmlink reaches the FetchContent call.
  if(NOT TARGET crc::crc)
    include(FetchContent)
    FetchContent_Declare(crc SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../utility/crc")
    FetchContent_MakeAvailable(crc)
  endif()

  add_library(rmlink STATIC
    "${CMAKE_CURRENT_LIST_DIR}/src/core.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/types.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/wire.cpp"
  )
  add_library(LibHT::rmlink ALIAS rmlink)
  target_include_directories(rmlink PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
  target_link_libraries(rmlink PUBLIC crc::crc LibHT::serialization)
  target_compile_features(rmlink PUBLIC cxx_std_20)

endif()
