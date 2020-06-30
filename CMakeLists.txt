cmake_minimum_required(VERSION 3.5)

project(KinovaControlExperiments VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Activate C++ 11
set (CMAKE_CXX_STANDARD 11)

# linux specific options
add_compile_options(-Wall)
add_compile_options(-Wno-reorder)
add_definitions(-D_OS_UNIX)

# kortex arm library and include files
set(EXTRACT_DIR "${PROJECT_SOURCE_DIR}/kinova_libs/extracted")
set(KORTEX_DIR "${EXTRACT_DIR}/linux_gcc_x86-64")

# unclear what this is used for, needs further investigation
include_directories(${EXTRACT_DIR}/cxxopts/)

link_libraries(${KORTEX_DIR}/lib/release/libKortexApiCpp.a)
include_directories(${KORTEX_DIR}/include)
include_directories(${KORTEX_DIR}/include/client)
include_directories(${KORTEX_DIR}/include/common)
include_directories(${KORTEX_DIR}/include/messages)
include_directories(${KORTEX_DIR}/include/client_stubs)

# user code
include_directories(${PROJECT_SOURCE_DIR}/generated)

link_libraries(pthread)

add_executable(control_kinova src/control_kinova.cpp)
