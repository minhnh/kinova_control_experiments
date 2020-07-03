cmake_minimum_required(VERSION 3.5)

project(KinovaControlExperiments VERSION 0.0.1 LANGUAGES CXX)

find_package(kdl_parser)
find_package(orocos_kdl)

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

# user code
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/generated
)

list(APPEND control_kinova_SOURCES
    ${PROJECT_SOURCE_DIR}/generated/abag.c
    ${PROJECT_SOURCE_DIR}/src/kinova_util.cpp
)
set_source_files_properties(generated/abag.c PROPERTIES LANGUAGE CXX)

link_libraries(
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    pthread
)

# find all CPP files in the ./executables directory and create a executable target
# for each of them
file(GLOB EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR} "executables/*.cpp")
foreach ( SRC_FILE ${EXE_LIST} )
    MESSAGE(${SRC_FILE})
    string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
    string(REPLACE "executables/" "" TARGET_EXE_NAME ${TARGET_EXE_NAME})

    MESSAGE("creating TARGET_EXE_NAME: '${TARGET_EXE_NAME}'")

    add_executable(${TARGET_EXE_NAME} ${SRC_FILE} ${control_kinova_SOURCES})

endforeach()
