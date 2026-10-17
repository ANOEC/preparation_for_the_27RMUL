# Platform-independent algorithm library collection.
# All algorithms are INTERFACE targets defined here; consumers link them explicitly.
# No LibXR dependency: algorithms use Algorithm::ErrorCode from error_code.hpp.

# The bundled snapshot is independent of the consumer's runtime/framework.
set(LIBHT_EIGEN_DIR "${CMAKE_CURRENT_LIST_DIR}/../lib/eigen" CACHE PATH "Eigen include root")
option(LIBHT_BUILD_RLS "Define the Eigen-based RLS target" ON)

# LESO: Linear Extended State Observer
add_library(LESO INTERFACE)
add_library(LibHT::LESO ALIAS LESO)
target_include_directories(LESO INTERFACE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/leso)
target_compile_features(LESO INTERFACE cxx_std_23)

# LADRC: Linear Active Disturbance Rejection Control (depends on LESO)
add_library(LADRC INTERFACE)
add_library(LibHT::LADRC ALIAS LADRC)
target_include_directories(LADRC INTERFACE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/ladrc)
target_link_libraries(LADRC INTERFACE LESO)
target_compile_features(LADRC INTERFACE cxx_std_23)

# PID: PID Controller
add_library(PID INTERFACE)
add_library(LibHT::PID ALIAS PID)
target_include_directories(PID INTERFACE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/pid)
target_compile_features(PID INTERFACE cxx_std_23)

# RLS: Recursive Least Squares (depends on Eigen)
if(LIBHT_BUILD_RLS)
  if(NOT EXISTS "${LIBHT_EIGEN_DIR}/Eigen/Core")
    message(FATAL_ERROR "RLS requires Eigen: set LIBHT_EIGEN_DIR or disable LIBHT_BUILD_RLS")
  endif()
  add_library(RLS INTERFACE)
  add_library(LibHT::RLS ALIAS RLS)
  target_include_directories(RLS INTERFACE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/rls)
  target_include_directories(RLS SYSTEM INTERFACE ${LIBHT_EIGEN_DIR})
  target_compile_features(RLS INTERFACE cxx_std_23)
endif()

# Wave: Signal Generators (sine, chirp, prbs, etc.)
add_library(Wave INTERFACE)
add_library(LibHT::Wave ALIAS Wave)
target_include_directories(Wave INTERFACE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/wave)
target_compile_features(Wave INTERFACE cxx_std_23)
