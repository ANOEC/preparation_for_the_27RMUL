# Install script for directory: D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/my_gimbal")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "D:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin/arm-none-eabi-objdump.exe")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/AdolcForward"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/AlignedVector3"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/ArpackSupport"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/AutoDiff"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/BVH"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/EulerAngles"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/FFT"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/IterativeSolvers"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/KroneckerProduct"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/LevenbergMarquardt"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/MatrixFunctions"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/MPRealSupport"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/NNLS"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/NonLinearOptimization"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/NumericalDiff"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/OpenGLSupport"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/Polynomials"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/SparseExtra"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/SpecialFunctions"
    "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/Splines"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/Middlewares/Third_Party/LibXR/lib/Eigen/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/build/Debug/LibXR/lib/Eigen/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "D:/stm32_workSpace/preparation_for_the_27RMUL/my_gimbal/build/Debug/LibXR/lib/Eigen/unsupported/Eigen/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
