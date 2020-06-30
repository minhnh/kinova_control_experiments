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
