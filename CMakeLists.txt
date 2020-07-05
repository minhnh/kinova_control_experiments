cmake_minimum_required(VERSION 3.5)

project(KinovaControlExperiments VERSION 0.0.1 LANGUAGES CXX)

find_package(kdl_parser)
find_package(orocos_kdl)

# configuration library
find_library(CONFIG_CPP_LIBRARY "config++" "/usr/lib" "/usr/local/lib")
if (NOT CONFIG_CPP_LIBRARY)
    MESSAGE(FATAL_ERROR "required config++ library not found, try installing libconfig++ and libconfig++-dev")
else()
    MESSAGE("found config++ library at ${CONFIG_CPP_LIBRARY}")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_VERBOSE_MAKEFILE      OFF)
set(CMAKE_COLOR_MAKEFILE        ON)

set(CMAKE_BUILD_TYPE Release)

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
include_directories(
    ${EXTRACT_DIR}/cxxopts/
)

link_libraries(
    ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
)
include_directories(
    ${KORTEX_DIR}/include
    ${KORTEX_DIR}/include/client
    ${KORTEX_DIR}/include/common
    ${KORTEX_DIR}/include/messages
    ${KORTEX_DIR}/include/client_stubs
)

# KDL related
include_directories(
    ${orocos_kdl_INCLUDE_DIRS}
    ${kdl_parser_INCLUDE_DIRS}
)
link_libraries(
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    pthread
)

# user code
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/generated
)

set_source_files_properties(generated/abag.c PROPERTIES LANGUAGE CXX)
add_library(kinova_control
    ${PROJECT_SOURCE_DIR}/generated/abag.c
    ${PROJECT_SOURCE_DIR}/src/kinova_util.cpp
    ${PROJECT_SOURCE_DIR}/src/util.cpp
)

# find all CPP files in the ./executables directory and create a executable target
# for each of them
file(GLOB EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR} "executables/*.cpp")
foreach ( SRC_FILE ${EXE_LIST} )
    string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
    string(REPLACE "executables/" "" TARGET_EXE_NAME ${TARGET_EXE_NAME})

    MESSAGE("creating TARGET_EXE_NAME: '${TARGET_EXE_NAME}'")
    add_executable(${TARGET_EXE_NAME} ${SRC_FILE})
    target_link_libraries(${TARGET_EXE_NAME} kinova_control ${CONFIG_CPP_LIBRARY})
endforeach()
