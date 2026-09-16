cmake_minimum_required(VERSION 3.16)
project(anydex_rokae_cpp VERSION 0.3.4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Threads REQUIRED)

if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  message(FATAL_ERROR "The vendored xCoreSDK-CPP runtime is aarch64-only; got ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(SDK_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/sdk/include")
set(SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib/aarch64")
set(SDK_LIBRARY "${SDK_LIB_DIR}/libxCoreSDK.so.0.3.4")
set(PYBIND_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pybind11/include")

foreach(required
    "${SDK_INCLUDE}/rokae/robot.h"
    "${SDK_INCLUDE}/Eigen/Core"
    "${SDK_LIBRARY}"
    "${PYBIND_INCLUDE}/pybind11/pybind11.h")
  if(NOT EXISTS "${required}")
    message(FATAL_ERROR "Missing vendored dependency: ${required}")
  endif()
endforeach()

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c
          "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX') or '.so')"
  OUTPUT_VARIABLE PYTHON_EXTENSION_SUFFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE PYTHON_SUFFIX_RESULT)
if(NOT PYTHON_SUFFIX_RESULT EQUAL 0)
  message(FATAL_ERROR "Failed to query Python extension suffix")
endif()

add_library(_rokae_cpp MODULE
  src/rokae_driver.cpp
  src/pybind_module.cpp)
target_include_directories(_rokae_cpp PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${SDK_INCLUDE}"
  "${PYBIND_INCLUDE}"
  ${Python3_INCLUDE_DIRS})
target_compile_options(_rokae_cpp PRIVATE -Wall -Wextra -Wpedantic -fvisibility=hidden)
target_link_libraries(_rokae_cpp PRIVATE "${SDK_LIBRARY}" Threads::Threads ${CMAKE_DL_LIBS})
set_target_properties(_rokae_cpp PROPERTIES
  PREFIX ""
  SUFFIX "${PYTHON_EXTENSION_SUFFIX}"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python"
  BUILD_RPATH "${SDK_LIB_DIR}"
  INSTALL_RPATH "\$ORIGIN/../sdk/lib/aarch64")
