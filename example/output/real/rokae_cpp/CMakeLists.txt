cmake_minimum_required(VERSION 3.12)
project(anydex_rokae_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(ROKAE_SDK_ROOT "/SSD-512G/Project/rokae-cpp/xCoreSDK-CPP" CACHE PATH
    "Path to the official ROKAE xCoreSDK-CPP repository")

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ROKAE_SDK_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ROKAE_SDK_ARCH "x86_64")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(ROKAE_SDK_LIBRARY_DIR
    "${ROKAE_SDK_ROOT}/lib/Linux/cpp/${ROKAE_SDK_ARCH}")
if(ROKAE_SDK_ARCH STREQUAL "x86_64")
  set(ROKAE_SDK_LIBRARY
      "${ROKAE_SDK_LIBRARY_DIR}/libxCoreSDK.nomodel.so.0.3.4")
else()
  set(ROKAE_SDK_LIBRARY
      "${ROKAE_SDK_LIBRARY_DIR}/libxCoreSDK.so.0.3.4")
endif()

if(NOT EXISTS "${ROKAE_SDK_ROOT}/include/rokae/robot.h")
  message(FATAL_ERROR "ROKAE headers not found under ${ROKAE_SDK_ROOT}")
endif()
if(NOT EXISTS "${ROKAE_SDK_LIBRARY}")
  message(FATAL_ERROR "ROKAE shared library not found: ${ROKAE_SDK_LIBRARY}")
endif()

find_package(Threads REQUIRED)
add_library(anydex_rokae_bridge SHARED rokae_bridge.cpp)
target_include_directories(anydex_rokae_bridge PRIVATE
    "${ROKAE_SDK_ROOT}/include"
    "${ROKAE_SDK_ROOT}/external")
target_link_libraries(anydex_rokae_bridge PRIVATE
    "${ROKAE_SDK_LIBRARY}" Threads::Threads)
set_target_properties(anydex_rokae_bridge PROPERTIES
    BUILD_RPATH "${ROKAE_SDK_LIBRARY_DIR}"
    INSTALL_RPATH "${ROKAE_SDK_LIBRARY_DIR}")
